"""EKF-SLAM core unit tests (pure numpy, no ROS graph).

predict must follow the motion model and update must shrink the covariance.
"""
from collections import namedtuple

import numpy as np
import pytest

from ekf_slam.ekf_core import EKFSLAM, select_observations, stamp_dt, wrap

Obs = namedtuple('Obs', 'color role range bearing')
TAU = 2.0 * np.pi


# ---------------- motion model and covariance decrease ----------------
def test_predict_straight():
    e = EKFSLAM()
    e.predict(v=1.0, w=0.0, dt=1.0)
    assert abs(e.pose()[0] - 1.0) < 1e-6


def test_update_reduces_cov():
    e = EKFSLAM()
    e.init_landmark('red', r=2.0, phi=0.0)
    before = np.trace(e.pose_cov())
    for _ in range(5):
        e.update('red', 2.0, 0.0)
    assert np.trace(e.pose_cov()) <= before


# ---------------- state layout and landmark indices ----------------
def test_state_layout_is_11_dim_with_expected_offsets():
    e = EKFSLAM()
    assert e.x.shape == (11,)
    assert e.P.shape == (11, 11)
    assert e.idx == {'red': 3, 'green': 5, 'blue': 7, 'yellow': 9}
    assert e.colors == ['red', 'green', 'blue', 'yellow']


def test_wrap_normalises_to_pi_range():
    for a in (-3 * np.pi, -np.pi, -0.1, 0.0, np.pi, 3 * np.pi, 7.0):
        w = float(wrap(a))
        assert -np.pi - 1e-12 <= w <= np.pi + 1e-12
        assert np.cos(w) == pytest.approx(np.cos(a))
        assert np.sin(w) == pytest.approx(np.sin(a))
    assert float(wrap(0.5 + TAU)) == pytest.approx(0.5)


# ---------------- predict ----------------
def test_predict_in_place_rotation():
    e = EKFSLAM()
    e.predict(v=0.0, w=1.0, dt=1.0)
    assert e.pose()[2] == pytest.approx(1.0)
    assert e.pose()[0] == pytest.approx(0.0)
    assert e.pose()[1] == pytest.approx(0.0)


def test_predict_arc_matches_analytic_arc():
    # v=0.5, w=0.5, T=1 s -> a 1 m radius arc: x=sin(wT), y=1-cos(wT)
    e = EKFSLAM()
    for _ in range(10):
        e.predict(0.5, 0.5, 0.1)
    x, y, th = e.pose()
    assert th == pytest.approx(0.5, abs=1e-9)
    assert x == pytest.approx(np.sin(0.5), abs=0.02)      # Euler vs closed form
    assert y == pytest.approx(1.0 - np.cos(0.5), abs=0.02)


def test_predict_wraps_heading():
    e = EKFSLAM()
    e.x[2] = 3.0
    e.predict(0.0, 1.0, 1.0)                              # 3.0 + 1.0 > pi
    assert -np.pi <= e.pose()[2] < np.pi


def test_process_noise_scales_with_dt():
    # Q scales with max(dt, 1e-6): a smaller dt must shrink the noise too
    traces = []
    for dt in (0.01, 0.1, 1.0):
        e = EKFSLAM()
        e.predict(1.0, 0.0, dt)
        traces.append(float(np.trace(e.pose_cov())))
    assert traces[0] < traces[1] < traces[2]


def test_predict_with_zero_dt_stays_finite():
    e = EKFSLAM()
    e.predict(1.0, 1.0, 0.0)                              # max(dt, 1e-6) stops 0 noise
    assert np.isfinite(e.P).all()
    assert e.pose()[0] == pytest.approx(0.0)


# ---------------- dt source ----------------
def test_stamp_dt_first_frame_has_no_dt():
    assert stamp_dt(None, 1_000_000_000) is None          # no predecessor -> nothing


def test_stamp_dt_is_the_stamp_difference():
    assert stamp_dt(1_000_000_000, 1_066_666_667) == pytest.approx(0.066666667, abs=1e-9)


def test_stamp_dt_keeps_every_frame_where_the_node_clock_drops_a_third():
    """Quantifies both dt sources in both directions so the node clock cannot creep back.

    /clock ticks every 0.1 s while /odom runs at 15 Hz (0.0667 s). Callbacks inside one
    clock tick see dt <= 0 on the node clock and are dropped by the `0.0 < dt < 0.5`
    rule; header.stamp drops nothing.
    """
    n = 15
    stamps = [int(round(i / n * 1e9)) for i in range(n + 1)]

    kept = sum(1 for a, b in zip(stamps, stamps[1:]) if 0.0 < stamp_dt(a, b) < 0.5)
    assert kept == n                                      # header.stamp: nothing dropped

    tick = 100_000_000                                    # 0.1 s
    ticks = [(s // tick) * tick for s in stamps]
    kept_clock = sum(1 for a, b in zip(ticks, ticks[1:]) if 0.0 < (b - a) * 1e-9 < 0.5)
    assert kept_clock < n                                 # node clock: most frames dropped


# ---------------- observation noise ----------------
def test_observation_noise_grows_with_range():
    e = EKFSLAM()
    assert np.trace(e.R_of(5.0)) > np.trace(e.R_of(1.0))
    assert e.R_of(0.0)[0, 0] == pytest.approx(0.03 ** 2)
    assert e.R_of(0.0)[1, 1] == pytest.approx(np.deg2rad(1.0) ** 2)


# ---------------- landmark initialisation ----------------
def test_init_landmark_places_mean_in_the_map_frame():
    e = EKFSLAM(range_bias_fix=False)                     # check the coordinate transform alone
    e.x[0], e.x[1], e.x[2] = 1.0, 1.0, np.pi / 2.0
    e.init_landmark('green', r=2.0, phi=0.0)              # straight ahead -> 2 m along +y
    gx, gy = e.landmark('green')
    assert gx == pytest.approx(1.0, abs=1e-9)
    assert gy == pytest.approx(3.0, abs=1e-9)
    assert e.landmark('green') is not None
    assert e.landmark('blue') is None                     # uninitialised -> None


def test_init_landmark_builds_cross_covariance_and_lowers_own_uncertainty():
    e = EKFSLAM()
    e.init_landmark('red', 2.0, 0.0)
    i = e.idx['red']
    assert np.allclose(e.P[i:i + 2, :3], e.P[:3, i:i + 2].T)
    assert np.abs(e.P[:3, i:i + 2]).max() > 1e-4           # robot and landmark must be correlated
    assert e.P[i, i] < 1e6                                 # no longer the 1e6 prior
    assert e.landmark_sigma('red') < 0.5


def test_new_landmark_clears_cross_covariance_with_other_landmarks():
    e = EKFSLAM()
    e.init_landmark('red', 2.0, 0.0)
    e.init_landmark('blue', 3.0, 0.5)
    ir, ib = e.idx['red'], e.idx['blue']
    assert np.allclose(e.P[ir:ir + 2, ib:ib + 2], 0.0)
    assert np.allclose(e.P[ib:ib + 2, ir:ir + 2], 0.0)


# ---------------- measurement conversion (camera -> near face vs origin -> centre) ----------------
def test_geometry_bias_face_term_is_maximal_at_45_deg():
    e = EKFSLAM()
    e.x[2] = 0.0
    e.camera_dx = e.camera_dy = 0.0                       # near-face term only
    assert e.geometry_bias(0.0, 2.8) == pytest.approx(0.2, abs=1e-9)
    assert e.geometry_bias(np.pi / 2, 2.8) == pytest.approx(0.2, abs=1e-9)
    # At 45 deg the principal axis turns diagonal: 0.2 / cos45 = 0.2*sqrt(2), the largest
    assert e.geometry_bias(np.pi / 4, 2.8) == pytest.approx(0.2 * np.sqrt(2.0), abs=1e-9)
    assert e.geometry_bias(-np.pi / 4, 2.8) == pytest.approx(0.2 * np.sqrt(2.0), abs=1e-9)
    assert e.geometry_bias(np.deg2rad(30.0), 2.8) == pytest.approx(
        0.2 / np.cos(np.deg2rad(30.0)), abs=1e-9)


def test_geometry_bias_reproduces_the_d203_measured_shortfall():
    """green at bearing -46.00 deg: 2.4820 measured against a 2.8271 centre truth, a
    0.3452 gap; yellow at +47.26 deg: 2.5547 against 0.2725. The model returns
    0.3598 / 0.2846, leaving a ~1.3 cm residual from pixel quantile sampling and pose
    quantisation."""
    e = EKFSLAM()
    e.x[2] = 0.0
    assert e.geometry_bias(np.deg2rad(-46.00), 2.4820) == pytest.approx(0.3452, abs=0.02)
    assert e.geometry_bias(np.deg2rad(47.26), 2.5547) == pytest.approx(0.2725, abs=0.02)


def test_geometry_bias_lever_arm_follows_the_robot_heading():
    """Lever arm u.R(theta)d: d lives in the body frame and must rotate with theta."""
    e = EKFSLAM()
    e.landmark_half_size = 0.0                            # lever arm only
    ang = np.deg2rad(-46.0)
    ux, uy = np.cos(ang), np.sin(ang)
    e.x[2] = 0.0
    assert e.geometry_bias(ang, 2.5) == pytest.approx(0.069 * ux - 0.047 * uy, abs=1e-12)
    e.x[2] = np.pi / 2.0
    assert e.geometry_bias(ang, 2.5) == pytest.approx(0.047 * ux + 0.069 * uy, abs=1e-12)


def test_geometry_bias_is_zero_when_the_fix_is_disabled():
    e = EKFSLAM(range_bias_fix=False)
    for ang in (0.0, 0.7, -2.0, np.pi):
        assert e.geometry_bias(ang, 2.5) == 0.0


def test_init_landmark_offset_moves_the_landmark_out_to_the_cube_centre():
    """Axial 2 m observation: the bias is the near-face term h plus the lever arm d_x."""
    plain = EKFSLAM(range_bias_fix=False)
    plain.init_landmark('red', 2.0, 0.0)
    fixed = EKFSLAM()
    fixed.init_landmark('red', 2.0, 0.0)
    bias = fixed.geometry_bias(0.0, 2.0)
    assert bias == pytest.approx(0.2 + 0.069, abs=1e-9)
    assert fixed.landmark('red')[0] == pytest.approx(plain.landmark('red')[0] + bias, abs=1e-9)
    assert fixed.landmark('red')[1] == pytest.approx(plain.landmark('red')[1], abs=1e-9)


def test_corrected_landmark_lands_on_the_true_cube_centre():
    """After conversion the green observation (2.4820 m at -46 deg) puts the landmark
    2.827 m from the origin, the true centre distance of green (2, -2)."""
    e = EKFSLAM()
    phi = np.deg2rad(-46.00)
    e.init_landmark('green', 2.4820, phi)
    lx, ly = e.landmark('green')
    assert np.hypot(lx, ly) == pytest.approx(2.8271, abs=0.02)
    assert lx == pytest.approx(2.0, abs=0.05)
    assert ly == pytest.approx(-2.0, abs=0.05)


def test_update_is_flat_when_init_and_update_share_the_convention():
    """Repeating one observation: the innovation stays 0 after conversion, so a biased
    measurement cannot drag the pose into a 0.43 m systematic offset."""
    e = EKFSLAM()
    phi = np.deg2rad(-46.00)
    e.init_landmark('green', 2.4820, phi)
    p0 = e.pose().copy()
    for _ in range(10):
        assert e.update('green', 2.4820, phi) is True
    assert e.gated == 0
    assert np.hypot(e.pose()[0] - p0[0], e.pose()[1] - p0[1]) < 5e-3
    assert abs(float(wrap(e.pose()[2] - p0[2]))) < np.deg2rad(0.2)


# ---------------- update ----------------
def test_update_rejects_uninitialised_and_unknown_colours():
    e = EKFSLAM()
    assert e.update('red', 1.0, 0.0) is False
    assert e.update('pink', 1.0, 0.0) is False


def test_update_accepts_a_small_innovation():
    e = EKFSLAM()
    e.init_landmark('red', 2.0, 0.0)
    assert e.update('red', 2.0, 0.0) is True
    assert e.update('red', 2.02, 0.005) is True
    assert e.gated == 0


def test_chi2_gate_rejects_an_outlier():
    e = EKFSLAM()
    e.init_landmark('red', 2.0, 0.0)
    assert e.update('red', 2.0, 0.0) is True
    before = e.gated
    assert e.update('red', 4.0, 0.0) is False              # same colour suddenly 2 m further away
    assert e.gated == before + 1
    assert e.chi2_gate == pytest.approx(5.991)


def test_chi2_gate_boundary_uses_the_configured_value():
    e = EKFSLAM()
    e.init_landmark('red', 2.0, 0.0)
    e.update('red', 2.0, 0.0)
    # with a looser gate the previously rejected observation must be accepted
    e.chi2_gate = 1e9
    assert e.update('red', 4.0, 0.0) is True


def test_landmark_sigma_shrinks_with_consistent_updates():
    e = EKFSLAM()
    e.init_landmark('blue', 2.0, 0.0)
    s0 = e.landmark_sigma('blue')
    for _ in range(10):
        assert e.update('blue', 2.0, 0.0) is True
    assert e.landmark_sigma('blue') < s0


def test_covariance_stays_symmetric_and_positive_semidefinite():
    e = EKFSLAM()
    e.init_landmark('red', 2.0, 0.0)
    e.init_landmark('yellow', 3.0, -0.4)
    for _ in range(10):
        e.predict(0.3, 0.1, 0.1)
        e.update('red', 2.0, 0.0)
        e.update('yellow', 3.0, -0.4)
    assert np.allclose(e.P, e.P.T, atol=1e-9)
    assert np.linalg.eigvalsh(e.P).min() > -1e-9


def test_covariance_stays_positive_semidefinite_over_a_long_run():
    """Covariance diagonal regression: a negative entry makes landmark_sigma return NaN.

    A NaN out of ``sqrt`` makes map_node raise ValueError, the landmark map node exits
    1 and ``/map`` freezes. This case walks the same long-run plus single-landmark
    rollback path, keeping only the observations that force a full rollback of
    update_frame.
    """
    e = EKFSLAM()
    e.init_landmark('red', 2.0, 0.0)
    e.init_landmark('green', 2.0, np.pi / 2)
    rng = np.random.default_rng(20261009)
    for k in range(600):
        e.predict(0.15 + 0.05 * rng.standard_normal(), 0.3 * rng.standard_normal(), 0.05)
        # half the frames carry one observation, forcing a full update_frame rollback
        obs = [('red', 2.0 + 0.02 * rng.standard_normal(), 0.01 * rng.standard_normal())]
        if k % 2:
            obs.append(('green', 2.0 + 0.02 * rng.standard_normal(),
                        np.pi / 2 + 0.01 * rng.standard_normal()))
        e.update_frame(obs, min_landmarks=2)

        assert np.all(np.diag(e.P) >= 0.0), f'第 {k} 帧出现负方差: {np.diag(e.P).min()}'
        assert np.all(np.isfinite(e.P)), f'第 {k} 帧协方差非有限'
        assert np.allclose(e.P, e.P.T, atol=1e-9), f'第 {k} 帧协方差不反倒'
        for c in ('red', 'green'):
            assert np.isfinite(e.landmark_sigma(c)), f'第 {k} 帧 {c} 的 sigma 非有限'


def test_landmark_sigma_reports_nan_instead_of_a_wrong_number():
    """A broken covariance must yield NaN sigma (warnable, tolerated downstream), never a

    fabricated number. ``math.sqrt`` raises on a negative input while ``np.sqrt`` only
    warns and returns NaN, so the two paths are made explicit here and both the
    negative variance and NaN cases are mapped to NaN.
    """
    e = EKFSLAM()
    e.init_landmark('red', 2.0, 0.0)
    i = e.idx['red']
    e.P[i, i] = -1.0
    assert np.isnan(e.landmark_sigma('red'))
    e.P[i, i] = float('nan')
    assert np.isnan(e.landmark_sigma('red'))
    e.P[i, i] = 4.0
    e.P[i + 1, i + 1] = 9.0
    assert e.landmark_sigma('red') == pytest.approx(13.0 ** 0.5)


def test_enforce_psd_repairs_the_d213_block_and_is_idempotent():
    """Worst observed shape: a landmark block with a negative main diagonal and

    correlations of the same order, as when all three colours break right after a
    frame whose 1 valid update falls below the minimum. The Joseph form only preserves
    positive semi-definiteness for a valid input, so an explicit projection is needed.
    """
    e = EKFSLAM()
    e.init_landmark('red', 2.0, 0.0)
    i = e.idx['red']
    e.P[i:i + 2, i:i + 2] = np.array([[-5.799e-02, 1.693e-01],
                                      [1.693e-01, -2.703e-01]])
    assert e.psd_min_eig() < 0.0
    assert np.isnan(e.landmark_sigma('red'))

    worst = e.enforce_psd('update:red')
    assert worst < 0.0, '必须报告被夹掉的本征值'
    assert e.psd_min_eig() >= -e.psd_tol()
    assert np.allclose(e.P, e.P.T, atol=1e-9)
    assert np.all(np.diag(e.P) >= 0.0)
    assert np.isfinite(e.landmark_sigma('red'))
    assert (e.psd_fixes, e.last_psd_fix[0]) == (1, 'update:red')

    # idempotent: clamping an already valid matrix changes nothing and does not count
    assert e.enforce_psd('update:red') == 0.0
    assert e.psd_fixes == 1


def test_predict_self_heals_a_corrupted_covariance():
    """Every write path in the filter carries the positive semi-definite guard, so a bad

    block cannot survive the next frame. This covers the case where the source stays
    unknown: one numeric fault used to take the whole chain down.
    """
    e = EKFSLAM()
    e.init_landmark('red', 2.0, 0.0)
    i = e.idx['red']
    e.P[i:i + 2, i:i + 2] = np.array([[-1.0, 0.5], [0.5, -1.0]])
    e.predict(0.0, 0.0, 0.05)
    assert e.psd_min_eig() >= -e.psd_tol()
    assert np.isfinite(e.landmark_sigma('red'))
    assert e.psd_fixes >= 1


# ---------------- whole-frame update (predict only if too few updates pass) ----------------
def _two_landmark_ekf():
    """Origin facing +x: red at (2,0) and green at (0,2), consistent with init."""
    e = EKFSLAM()
    e.init_landmark('red', 2.0, 0.0)
    e.init_landmark('green', 2.0, np.pi / 2)
    return e


def test_update_frame_applies_when_enough_observations_are_accepted():
    e = _two_landmark_ekf()
    p_before = np.trace(e.pose_cov())
    used, applied = e.update_frame([('red', 2.0, 0.0), ('green', 2.0, np.pi / 2)],
                                   min_landmarks=2)
    assert (used, applied) == (2, True)
    assert np.trace(e.pose_cov()) <= p_before


def test_update_frame_discards_the_whole_frame_below_min_landmarks():
    """With a single valid update the frame must leave the state untouched."""
    e = _two_landmark_ekf()
    x0, P0 = e.x.copy(), e.P.copy()
    used, applied = e.update_frame([('red', 2.0, 0.0)], min_landmarks=2)
    assert (used, applied) == (1, False)
    assert np.array_equal(e.x, x0)
    assert np.array_equal(e.P, P0)


def test_update_frame_threshold_one_keeps_a_single_observation():
    e = _two_landmark_ekf()
    x0 = e.x.copy()
    used, applied = e.update_frame([('red', 2.0, 0.05)], min_landmarks=1)
    assert (used, applied) == (1, True)
    assert not np.array_equal(e.x, x0)


def test_update_frame_counts_only_accepted_observations():
    """A chi-square rejected observation is not a valid update: 1 accepted + 1 rejected
    still falls short."""
    e = _two_landmark_ekf()
    x0, P0 = e.x.copy(), e.P.copy()
    used, applied = e.update_frame([('red', 2.0, 0.0), ('green', 2.0, np.pi / 2 + 2.0)],
                                   min_landmarks=2)
    assert (used, applied) == (1, False)
    assert np.array_equal(e.x, x0)
    assert np.array_equal(e.P, P0)


def test_update_frame_treats_uninitialised_landmarks_as_no_evidence():
    e = EKFSLAM()
    assert e.update_frame([('blue', 1.0, 0.0)], min_landmarks=2) == (0, False)
    assert e.update_frame([], min_landmarks=2) == (0, False)


def test_update_frame_does_not_roll_back_the_gate_counter():
    """gated is a diagnostic counter: dropping the frame must not erase it."""
    e = _two_landmark_ekf()
    n = e.gated
    used, applied = e.update_frame([('red', 2.0, 0.0), ('green', 2.0, np.pi / 2 + 2.0)],
                                   min_landmarks=2)
    assert (used, applied) == (1, False)
    assert e.gated == n + 1


def test_update_frame_does_not_touch_landmarks_initialised_in_this_frame():
    """init is not a pose update: a new landmark still stays in the state."""
    e = EKFSLAM()
    e.init_landmark('red', 2.0, 0.0)
    lm_before = e.landmark('red')
    used, applied = e.update_frame([('red', 2.0, 0.0)], min_landmarks=2)
    assert applied is False
    assert e.landmark('red') == lm_before


# ---------------- observation selection ----------------
def test_select_observations_filters_role_and_range():
    obs = [Obs('red', 'landmark', 2.0, 0.0),
           Obs('green', 'landmark', 9.0, 0.1),            # beyond max_dist
           Obs('white', 'obstacle', 1.0, 0.2),            # obstacles never enter a SLAM update
           Obs('blue', 'landmark', 3.0, 0.3)]
    got = select_observations(obs, max_dist=5.0)
    assert [o.color for o in got] == ['red', 'blue']


def test_select_observations_returns_single_candidate_as_is():
    obs = [Obs('red', 'landmark', 2.0, 0.0)]
    assert select_observations(obs) == obs


def test_select_observations_locks_both_endpoints_when_more_than_k():
    # 4 candidates, k=3: the widest pair must be kept, not 3 consecutive bearings
    obs = [Obs('red', 'landmark', 2.0, 0.0),
           Obs('green', 'landmark', 2.0, 0.2),
           Obs('blue', 'landmark', 2.0, 0.5),
           Obs('yellow', 'landmark', 2.0, 1.2)]
    got = select_observations(obs, max_dist=5.0, k=3)
    assert [o.bearing for o in got] == [0.0, 0.5, 1.2]


def test_select_observations_picks_at_most_k():
    obs = [Obs('c%d' % i, 'landmark', 2.0, i * 0.1) for i in range(8)]
    got = select_observations(obs, max_dist=5.0, k=3)
    assert len(got) == 3
    assert got[0].bearing == pytest.approx(0.0)
    assert got[-1].bearing == pytest.approx(0.7)


# ---------------- reset (used by the /reset_slam service in slam_node) ----------------
def _driven_ekf():
    """A filter that has flown a trajectory: pose, covariance, landmarks and counters have
    all moved off their initial values."""
    e = EKFSLAM()
    for _ in range(20):
        e.predict(v=0.2, w=0.5, dt=0.05)
    e.init_landmark('red', 2.0, 0.0)
    e.update('red', 2.0, 0.0)
    e.update('red', 9.0, 0.0)                    # one out-of-gate observation to make gated > 0
    e.P[e.idx['red'], e.idx['red']] = -1.0       # one negative eigenvalue
    e.enforce_psd('test')                        # -> psd_fixes > 0
    return e


def test_reset_restores_the_initial_state_bit_for_bit():
    """A reset filter must match a freshly built one bit for bit."""
    e = _driven_ekf()
    assert e.gated > 0 and e.psd_fixes > 0 and e.inited['red']
    e.reset()
    fresh = EKFSLAM()
    assert np.array_equal(e.x, fresh.x)
    assert np.array_equal(e.P, fresh.P)
    assert e.inited == fresh.inited
    assert (e.gated, e.obs_seen, e.psd_fixes, e.last_psd_fix) == \
           (fresh.gated, fresh.obs_seen, fresh.psd_fixes, fresh.last_psd_fix)


def test_reset_clears_landmarks_so_they_can_be_reinitialised():
    """An uninitialised landmark block goes back to 1e6, not 0: a zero block would pin the
    update gain at 0."""
    e = _driven_ekf()
    e.reset()
    assert all(not v for v in e.inited.values())
    assert e.P[3, 3] == pytest.approx(1e6)
    e.init_landmark('red', 2.0, 0.0)
    assert e.inited['red']
    assert np.trace(e.P[3:5, 3:5]) < 1e6


def test_reset_keeps_the_noise_parameters_so_a_rerun_is_comparable():
    """State is cleared while parameters stay, so the two runs remain comparable."""
    e = EKFSLAM(camera_dx=0.1, camera_dy=-0.2, landmark_half_size=0.3,
                range_bias_fix=False)
    q_before = e.Q.copy()
    noise_before = (e.sigma_r0, e.k_r, e.sigma_p0, e.k_p, e.chi2_gate)
    e.reset()
    assert np.array_equal(e.Q, q_before)
    assert (e.sigma_r0, e.k_r, e.sigma_p0, e.k_p, e.chi2_gate) == noise_before
    assert e.colors == ['red', 'green', 'blue', 'yellow']
    assert e.camera_dx == 0.1 and e.camera_dy == -0.2
    assert e.landmark_half_size == 0.3 and e.range_bias_fix is False
