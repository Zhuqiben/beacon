"""Pure EKF-SLAM core, plus observation selection and small coordinate helpers. Imports
numpy and math only, never rclpy, so it is testable without a ROS graph.

Measurement convention: the detector's planar range runs from the **camera** to the
**near face** of a block, while ``z_hat`` and the map store **robot origin** to
**centre**. ``EKFSLAM.geometry_bias()`` converts between the two, and both
``init_landmark`` and ``update`` go through it; otherwise the map starts about 0.28 m
too close.
"""
import math

import numpy as np


def wrap(a):
    """Normalise an angle to [-pi, pi)."""
    return (a + np.pi) % (2 * np.pi) - np.pi


# ---------------- coordinate / quaternion helpers (pure, directly testable) ----------------
def yaw_to_quat(yaw):
    """yaw -> (x, y, z, w), a rotation about z only."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_to_yaw(q):
    """Quaternion -> yaw. ``q`` may be a Quaternion message or an (x, y, z, w) tuple."""
    if hasattr(q, 'w'):
        x, y, z, w = q.x, q.y, q.z, q.w
    else:
        x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def world_from_camera(x, y, th, r, phi):
    """Body-frame observation (range, bearing) -> map coordinates.

    phi is the bearing in the body frame, positive to the left, with the sign already
    converted by the detector.
    """
    ang = wrap(th + phi)
    return x + r * math.cos(ang), y + r * math.sin(ang)


def stamp_dt(prev_stamp_ns, stamp_ns):
    """EKF prediction step dt from two ``header.stamp`` values (nanoseconds); None means
    there is no previous frame yet and the caller skips the first prediction.

    dt comes from the **message timestamp**, not the node clock: under ``use_sim_time``
    the clock only advances on a /clock message, so it is quantised to /clock's 0.1 s
    period while /odom runs at 15 Hz (0.0667 s). The two do not divide evenly, so roughly
    a third of the callbacks get dt <= 0 and are dropped by the ``0.0 < dt < 0.5`` rule.
    ``header.stamp`` is exactly 0.067 s apart without jitter.
    """
    if prev_stamp_ns is None:
        return None
    return (stamp_ns - prev_stamp_ns) * 1e-9


def map_to_odom(x, y, th, ob_x, ob_y, ob_th):
    """map->odom = (map->base) composed with (odom->base)^-1.

    (x, y, th) is the EKF estimate of base_link in map, (ob_x, ob_y, ob_th) is the
    odom->base_link TF, and the result is (mo_x, mo_y, mo_th).
    """
    c, s = math.cos(-ob_th), math.sin(-ob_th)
    inv_x = -(c * ob_x - s * ob_y)
    inv_y = -(s * ob_x + c * ob_y)

    c2, s2 = math.cos(th), math.sin(th)
    return (x + c2 * inv_x - s2 * inv_y,
            y + s2 * inv_x + c2 * inv_y,
            float(wrap(th - ob_th)))


class EKFSLAM:
    """EKF-SLAM state: x = [xr, yr, thr, l1x, l1y, l2x, l2y, l3x, l3y, l4x, l4y] (11).

    Bearing is a body-frame angle, positive to the left, with the sign already converted
    by the detector.
    """

    def __init__(self, colors=('red', 'green', 'blue', 'yellow'),
                 camera_dx=0.069, camera_dy=-0.047,
                 landmark_half_size=0.2, range_bias_fix=True):
        self.colors = list(colors)
        self.idx = {c: 3 + 2 * i for i, c in enumerate(self.colors)}
        # The detector's range goes from the camera to the near face of a block, while
        # z_hat and the map store the distance from the robot origin to its centre. The
        # two differ by two terms, which geometry_bias() converts.
        self.camera_dx = float(camera_dx)
        self.camera_dy = float(camera_dy)
        self.landmark_half_size = float(landmark_half_size)
        self.range_bias_fix = bool(range_bias_fix)
        n = 3 + 2 * len(self.colors)
        self.x = np.zeros(n)
        self.P = np.eye(n) * 1e6
        self.P[:3, :3] = np.diag([1e-4, 1e-4, 1e-4])
        self.Q = np.diag([0.02 ** 2, 0.02 ** 2, 0.01 ** 2])
        # Observation noise grows with range -- a depth camera is roughly a constant term
        # plus a percentage of the range -- so it is not a fixed value.
        self.sigma_r0, self.k_r = 0.03, 0.02          # 3 cm + 2% of range
        self.sigma_p0, self.k_p = np.deg2rad(1.0), 0.010
        self.chi2_gate = 5.991                        # chi2(2, 0.95)
        self.inited = {c: False for c in self.colors}
        self.gated = 0                                # observations rejected by the gate
        self.obs_seen = 0                             # observations that reached the gate
        self.psd_fixes = 0                            # times enforce_psd clamped the spectrum
        self.last_psd_fix = None                      # (tag, smallest eigenvalue)

    def reset(self):
        """Restore the filter to its ``__init__`` state: pose zero, landmarks uninited.

        Gazebo's ``/reset_world`` restores the simulation, but ``x``, ``P`` and ``inited``
        live in this process's memory and are not touched by it. Leaving the map->odom
        correction at its pre-reset value drags the whole map, robot included, along with
        it: ``/robot_pose`` then reports a position roughly 0.33 m off truth, and that
        offset never heals on its own.

        Only the state is cleared, never the parameters (``Q``, the ``R_of`` noise
        parameters, the gate, the measurement convention), so a reset filter matches a
        freshly constructed one. Uninited landmark blocks stay at 1e6 rather than 0,
        which would be read as certainty and hold the ``update`` gain at zero forever.
        """
        n = len(self.x)
        self.x = np.zeros(n)
        self.P = np.eye(n) * 1e6
        self.P[:3, :3] = np.diag([1e-4, 1e-4, 1e-4])
        self.inited = {c: False for c in self.colors}
        self.gated = 0
        self.obs_seen = 0
        self.psd_fixes = 0
        self.last_psd_fix = None

    def R_of(self, r):
        sr = self.sigma_r0 + self.k_r * r
        sp = self.sigma_p0 + self.k_p * r
        return np.diag([sr ** 2, sp ** 2])

    # ---------------- positive semidefiniteness (a numerical invariant, not a fix) --------
    def psd_tol(self):
        """Absolute tolerance for calling P positive semidefinite.

        The eigenvalues scale with ``trace(P)`` (uninited landmark blocks sit at 1e6, so an
        11-dimensional trace reaches 4e6) while the error of an eigendecomposition is
        **relative**, so the tolerance has to follow the trace. 1e-12 relative is 4e-6 at
        trace = 4e6, i.e. 2e-3 m of sigma, far below any meaningful uncertainty.
        """
        return 1e-12 * max(1.0, float(np.trace(self.P)))

    def psd_min_eig(self):
        """Smallest eigenvalue of the symmetric part of P (the diagnostic of enforce_psd)."""
        return float(np.linalg.eigvalsh(0.5 * (self.P + self.P.T))[0])

    def enforce_psd(self, tag):
        """Clamp the eigenvalues of P to >= 0 and return the smallest one that was clamped.

        EKF covariances are positive semidefinite by construction, but two **structural**
        approximations break that here: ``init_landmark`` zeroes the cross-blocks between a
        new landmark and every other landmark and leaves an uninited block at 1e6, and
        ``update``'s gain can be large when ``S`` is nearly singular, amplifying any
        negative eigenvalue to ``|lambda| * |K|^2``. A frame rolled back after a single
        valid update can then leave 2x2 blocks with **both** diagonal entries negative, and
        a negative diagonal makes ``landmark_sigma`` return NaN, which makes ``ceil(NaN)``
        in ``map_node`` crash the node and freeze ``/map``. The Joseph form in ``update``
        preserves positive semidefiniteness for an already valid P but cannot repair a
        broken block, so this layer is needed.

        Raising eigenvalues only, without touching the mean, errs towards "more
        uncertain", the conservative direction for a filter, and is the standard symmetric
        positive-semidefinite projection.
        """
        w, V = np.linalg.eigh(0.5 * (self.P + self.P.T))
        if w[0] >= -self.psd_tol():
            return 0.0
        self.P = (V * np.clip(w, 0.0, None)) @ V.T
        self.psd_fixes += 1
        self.last_psd_fix = (tag, float(w[0]))
        return float(w[0])

    # ---------------- measurement convention: camera to near face vs origin to centre ----
    def geometry_bias(self, ang, r):
        """Increment in metres that turns the detector's range (camera -> near face) into
        robot origin -> block centre.

        `ang` is the landmark's bearing in the world frame. `r` is not used yet (it is
        reserved for a per-range calibration) but stays in the signature for readability.
        Two terms:

        1. **Near-face term** `h / max(|cos ang|, |sin ang|)`, with `h` the half side
           length. The ray leaves the camera towards the block centre and hits the near
           face first, so the hit point is that much closer than the centre. It peaks at a
           45 degree bearing (h * sqrt 2) and equals h along an axis.
        2. **Lever-arm term** `u . R(theta) d`, with `u` the unit line of sight, `d` the
           camera offset from base_link in the body frame and `R(theta)` the body-to-world
           rotation: the measurement originates at the camera, not at the origin.

        Left out as negligible: the bias derivative with respect to theta (about 3 % of the
        third column of `Gx`) and the lever arm on the bearing, an order of magnitude below
        `R_of`'s sigma_phi.
        """
        if not self.range_bias_fix:
            return 0.0
        ux, uy = math.cos(ang), math.sin(ang)
        face = self.landmark_half_size / max(abs(ux), abs(uy), 1e-6)
        c, s = math.cos(self.x[2]), math.sin(self.x[2])
        lever = (c * self.camera_dx - s * self.camera_dy) * ux + \
                (s * self.camera_dx + c * self.camera_dy) * uy
        return float(face + lever)

    # ---------------- prediction ----------------
    def predict(self, v, w, dt):
        th = self.x[2]
        self.x[0] += v * np.cos(th) * dt
        self.x[1] += v * np.sin(th) * dt
        self.x[2] = wrap(th + w * dt)
        n = len(self.x)
        F = np.eye(n)
        F[0, 2] = -v * np.sin(th) * dt
        F[1, 2] = v * np.cos(th) * dt
        Qf = np.zeros((n, n))
        # Process noise scales with dt: adding a fixed self.Q would lose the physical
        # meaning of a "noise density per second" and would not follow dt when odometry
        # drops frames, which distorts the filter gain.
        Qf[:3, :3] = self.Q * max(dt, 1e-6)
        self.P = F @ self.P @ F.T + Qf
        self.enforce_psd('predict')

    # ---------------- landmark initialisation ----------------
    def init_landmark(self, color, r, phi):
        """Standard EKF-SLAM initialisation: mean, covariance, and the cross-covariance
        with the robot pose, following Probabilistic Robotics table 7.1.

        Writing only the mean and leaving the landmark block at 1e6 breaks the correlation
        between robot and landmark and shows up as an error above 0.20 m when the robot
        returns to the origin.
        """
        i = self.idx[color]
        ang = wrap(self.x[2] + phi)
        # Convert the observation (camera -> near face) to the map convention (origin ->
        # centre) first and add afterwards; Gx, Gz and R all use the converted range.
        r_eff = float(r) + self.geometry_bias(ang, float(r))
        self.x[i] = self.x[0] + r_eff * np.cos(ang)
        self.x[i + 1] = self.x[1] + r_eff * np.sin(ang)

        Gx = np.array([[1.0, 0.0, -r_eff * np.sin(ang)],
                       [0.0, 1.0, r_eff * np.cos(ang)]])
        Gz = np.array([[np.cos(ang), -r_eff * np.sin(ang)],
                       [np.sin(ang), r_eff * np.cos(ang)]])
        R = self.R_of(r_eff)
        Prr = self.P[:3, :3]

        self.P[i:i + 2, i:i + 2] = Gx @ Prr @ Gx.T + Gz @ R @ Gz.T
        self.P[i:i + 2, :3] = Gx @ Prr
        self.P[:3, i:i + 2] = self.P[i:i + 2, :3].T
        for c in self.colors:                          # cross-blocks with other landmarks
            if c == color:
                continue
            k = self.idx[c]
            self.P[i:i + 2, k:k + 2] = 0.0
            self.P[k:k + 2, i:i + 2] = 0.0
        self.inited[color] = True
        self.enforce_psd('init:' + color)

    # ---------------- update (with a chi-square gate) ----------------
    def update(self, color, r, phi):
        """True if the observation was accepted, False if it was rejected (unknown or
        uninited landmark, degenerate geometry, or above the gate)."""
        if color not in self.idx or not self.inited[color]:
            return False
        i = self.idx[color]
        dx = self.x[i] - self.x[0]
        dy = self.x[i + 1] - self.x[1]
        q = dx * dx + dy * dy
        if q < 1e-6:
            return False
        sr = np.sqrt(q)
        ang = float(np.arctan2(dy, dx))
        # Bring the measurement to the same convention as sr in z_hat (origin -> centre).
        # H keeps its usual form; the bias derivative with respect to the state is
        # negligible.
        r_eff = float(r) + self.geometry_bias(ang, float(r))
        z = np.array([r_eff, phi])
        z_hat = np.array([sr, wrap(ang - self.x[2])])

        n = len(self.x)
        H = np.zeros((2, n))
        H[0, 0] = -dx / sr
        H[0, 1] = -dy / sr
        H[0, i] = dx / sr
        H[0, i + 1] = dy / sr
        H[1, 0] = dy / q
        H[1, 1] = -dx / q
        H[1, 2] = -1.0
        H[1, i] = -dy / q
        H[1, i + 1] = dx / q

        R = self.R_of(r_eff)
        S = H @ self.P @ H.T + R
        innov = z - z_hat
        innov[1] = wrap(innov[1])

        # Chi-square gate against misdetections and colour mix-ups: colour *is* identity,
        # so one mix-up pollutes the whole map.
        try:
            d2 = float(innov @ np.linalg.solve(S, innov))
        except np.linalg.LinAlgError:
            return False
        if d2 > self.chi2_gate:
            self.gated += 1
            return False

        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        self.x[2] = wrap(self.x[2])
        # Joseph form: P = (I-KH) P (I-KH)^T + K R K^T, symmetric positive semidefinite for
        # any K and equivalent to (I-KH) P in exact arithmetic. A plain (I-KH) P accumulates
        # rounding error over a long run, and a negative diagonal then makes sqrt in
        # landmark_sigma return NaN and ceil(NaN) in map_node crash the node.
        IKH = np.eye(n) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)             # symmetrise
        self.enforce_psd('update:' + color)
        return True

    # ---------------- whole-frame update ----------------
    def update_frame(self, obs, min_landmarks=2):
        """Process one frame of observations, returning ``(used, applied)``.

        **Fewer than `min_landmarks` (2 by default) accepted updates means the frame only
        predicts and does not touch the pose.** Whether an observation is accepted depends
        on the pose, the covariance at that moment and the chi-square gate, so the frame
        cannot be filtered up front: back up, attempt it as a whole, and roll back if the
        result falls short, which is equivalent to never having updated.

        `obs` is an iterable of `(color, r, phi)`. `self.gated` counts gate rejections and
        does **not** take part in the rollback.
        """
        x_before, P_before = self.x.copy(), self.P.copy()
        used = 0
        for color, r, phi in obs:
            self.obs_seen += 1
            if self.update(color, r, phi):
                used += 1
        if used < min_landmarks:
            self.x, self.P = x_before, P_before
            return used, False
        return used, True

    # ---------------- queries ----------------
    def pose(self):
        return self.x[:3].copy()

    def pose_cov(self):
        return self.P[:3, :3].copy()

    def landmark(self, color):
        if not self.inited[color]:
            return None
        i = self.idx[color]
        return self.x[i], self.x[i + 1]

    def landmark_sigma(self, color):
        i = self.idx[color]
        # The variance must be non-negative. A negative diagonal or a NaN from a numerical
        # problem is **not** clamped to 0, which would pretend the position is certain:
        # NaN goes out so the consumers see it and warn, and they all tolerate NaN.
        var = float(self.P[i, i] + self.P[i + 1, i + 1])
        if not math.isfinite(var) or var < 0.0:
            return float('nan')
        return math.sqrt(var)


# ---------------- observation selection (here, to keep slam_node self-contained) --------
def select_observations(obs, max_dist=5.0, k=3):
    """Pick which landmarks take part in an update: range filter plus maximum span.

    A strict GDOP criterion would come from the Fisher information matrix; this is a
    "largest angular span" heuristic that avoids the degenerate geometry of collinear or
    same-direction close landmarks.
    """
    cand = [o for o in obs if o.role == 'landmark' and o.range <= max_dist]
    if len(cand) < 2:
        return cand
    cand = sorted(cand, key=lambda o: o.bearing)
    best, best_span = cand[:k], -1.0
    n = len(cand)
    for i in range(n):
        for j in range(i + 1, n):
            span = cand[j].bearing - cand[i].bearing
            if span > best_span:
                best_span = span
                group = cand[i:j + 1]
                if len(group) <= k:
                    best = group
                else:
                    # Keep the two extremes of the span and fill in evenly between them;
                    # a plain group[:k] slice would drop group[-1], the far end of the very
                    # span being maximised.
                    pick = np.linspace(0, len(group) - 1, k).round().astype(int)
                    best = [group[t] for t in sorted(set(pick.tolist()))]
    return best
