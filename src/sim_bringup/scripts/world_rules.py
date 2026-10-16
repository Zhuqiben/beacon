#!/usr/bin/env python3
"""World-map rules: four quadrant-randomised landmarks and two cubes roaming the arena.

Why
---
The evaluation world pins the four 0.4 m cubes at (+-2, +-2).  That is the right thing for a
benchmark, but in a demo it *is* the answer: the four corner coordinates are constants and
even the mapping step becomes optional.  This module turns "each colour appears somewhere in
its own quadrant" into a **reproducible** rule -- one seed gives one world -- and every
randomised scene goes through :func:`validate_scene` before it is written, so a rejected
scene is regenerated from the next seed instead of being handed to Gazebo.

Rules (world-map rules v2)
--------------------------
1. **Quadrants**: the arena is split by the x / y axes, red in the third quadrant, green in
   the fourth, blue in the second and yellow in the first -- the colour convention shared with
   ``scripts/benchmark.py``'s ``LANDMARK_TRUTH``.  Only the placement changes: instead of a
   fixed +-2 m each landmark draws a grid point from [quadrant_lo, quadrant_hi].  Neighbouring
   quadrants stay at least 2*quadrant_lo = 2.4 m apart, so two landmarks can never sit next to
   each other (the top-view calibration maps 139 px to 1 m, so 2.4 m is ~334 px, far beyond any
   colour-matching radius, and one block cannot be mistaken for another).

2. **Random walk, not a fixed patrol loop**: the black and white cubes own no territory and
   follow no preset polyline.  They wander the whole 8x8 m arena like pedestrians with inertia
   (the heading is a slow random walk, one tick turning no more than the angle ``max_yaw_rate``
   allows, so the path curves rather than spirals), which is what forces the robot's obstacle
   avoidance and re-planning to run.  The path is **closed at generation time** (after
   ``walk_seconds`` it heads back to its start point), so the node can still replay it as a
   closed polyline advanced by arc length.

3. **Occupancy exclusion, the whole anti-collision argument**: every cube is 0.4 m on a side
   and is registered with the circumscribed radius 0.283 m (conservative: it covers any yaw,
   unlike the inscribed radius).  The four static landmarks are **permanent claimants**, and
   each dynamic cube computes the cell it wants next and, if ``CellClaims`` shows that cell
   taken, waits in place (or turns) until the other cube **releases** it.  "No collision"
   therefore follows from the registry itself rather than from a geometric separation arranged
   in advance.  ``clear_static`` (0.70 m by default) is the clearance between a dynamic and a
   static cube centre: the two circumscribed squares together span 0.57 m, so 0.70 m leaves
   margin.

4. **Different speeds give different trajectories**: each cube draws a speed from
   [speed_lo, speed_hi], at least ``speed_gap`` apart.  The paths differ anyway -- different
   seeds, start points and speeds.

Runtime half
------------
:func:`sample_loop` maps arc length to pose and twist, and :class:`CellClaims` is the
occupancy registry; ``scripts/dynamic_obstacles.py`` calls both.  The module imports no rclpy,
gazebo or numpy bindings, so a launch file can call it synchronously before gzserver starts
(to generate the derived world file) and offline property tests can still run over thousands
of seeds (``world_setup.py --selftest``).

Ground truth
------------
``scripts/benchmark.py`` needs the truth to score a run, so ``world_setup.py`` writes the
``landmarks`` list into ``scene.json``.  Only ``benchmark.py`` reads that list: no robot-side
node does (``/landmark_map`` is still driven purely by camera observations and
``performance_monitor`` uses ``/gazebo/model_states``), and nothing derives a pose from it.
The dynamic cubes' kinematics use only the coordinates of their own path, which is a runtime
behaviour, not a function of where the robot is or what it sees.
"""
import json
import math
import os
import random

# The four coloured landmarks, in the order ``benchmark.LANDMARK_TRUTH`` uses.
COLORS = ('red', 'green', 'blue', 'yellow')

# Quadrant sign per colour: (sign_x, sign_y).  red is south-west, green south-east,
# blue north-west and yellow north-east; "third quadrant" is now a range of grid
# points instead of a single point.
QUADRANT = {
    'red': (-1, -1),
    'green': (1, -1),
    'blue': (-1, 1),
    'yellow': (1, 1),
}

# Dynamic obstacles.  The world file names them <model name> = MODEL_PREFIX + colour.
OBSTACLES = ('white', 'black')
MODEL_PREFIX = 'obstacle_'

# Defaults for the world-map rules; params.yaml's ``world_rules`` section overrides any.
DEFAULT_RULES = {
    # Arena: ground_plane is 8x8 m, i.e. +-4.0.  ``wall_margin`` keeps cubes off the walls.
    'arena_half': 4.0,
    'wall_margin': 0.45,
    'block_half': 0.2,            # 0.4 m cube
    # Landmark placement: the axis distance a colour draws from, and the grid used for it.
    'quadrant_lo': 1.20,
    'quadrant_hi': 2.80,
    'grid_m': 0.05,
    # Minimum distance from a dynamic to a static cube centre: the two circumscribed
    # squares span 0.57 m, so 0.70 m leaves margin and a passing cube blocks no stop pose.
    'clear_static': 0.70,
    # Speed: each cube draws from [speed_lo, speed_hi], at least speed_gap apart.
    'speed_lo': 0.55,
    'speed_hi': 1.80,
    'speed_gap': 0.25,
    # Heading smoothing: yaw follows the chord over +-yaw_window/2, not the tangent of the
    # current segment, which jumps at every vertex and would snap the cube around corners.
    'yaw_window': 0.30,
    'max_yaw_rate': 2.0,
    'max_attempts': 400,
    # The random-walk knobs (``walk_*`` / ``sim_*``) live in WANDER_DEFAULTS below and are
    # merged into this table at import time, so params.yaml reaches them as well.
}


_INT_KEYS = ('max_attempts',)


def make_rules(overrides=None):
    """Merge ``overrides`` (params.yaml's ``world_rules`` section) into the defaults."""
    rules = dict(DEFAULT_RULES)
    for key, value in (overrides or {}).items():
        if key not in rules or value is None:
            continue
        try:
            rules[key] = int(value) if key in _INT_KEYS else float(value)
        except (TypeError, ValueError):
            continue
    return rules


def _flat_scalar_section(path, top, sub='ros__parameters'):
    """PyYAML-free fallback parser: flat scalar lines under ``<top>:`` / ``<sub>:``.

    The ``world_rules`` section is ``key: number`` by convention, which this scanner
    handles in a few lines; it lets this module (and ``world_setup.py --selftest``) run
    where PyYAML is absent.  PyYAML is preferred when installed, and nested keys parse.
    """
    found = {}
    inside_top = False
    inside_sub = False
    with open(path, 'r', encoding='utf-8-sig') as handle:
        for raw in handle:
            line = raw.split('#', 1)[0].rstrip()
            if not line.strip():
                continue
            indent = len(line) - len(line.lstrip(' '))
            stripped = line.strip()
            if indent == 0:
                inside_top = stripped == top + ':'
                inside_sub = False
                continue
            if not inside_top:
                continue
            if indent == 2 and stripped.endswith(':'):
                inside_sub = stripped == sub + ':'
                continue
            if not inside_sub or indent < 4 or ':' not in stripped:
                continue
            key, _, value = stripped.partition(':')
            value = value.strip().strip('"').strip("'")
            if not value:
                continue
            try:
                found[key.strip()] = float(value) if ('.' in value or 'e' in value.lower()) \
                    else int(value)
            except ValueError:
                continue
    return found


def rules_from_params_file(path):
    """Read the ``world_rules.ros__parameters`` section of ``params.yaml``.

    The section is written as the parameter block of a node that does not exist, which
    keeps one file as the single source for both node parameters and world-map rules: the
    ROS 2 parameter parser wants a ``ros__parameters`` map under every top-level key, and
    since no node is named ``world_rules`` the extra section is silently ignored.
    """
    try:
        import yaml
    except ImportError:
        section = _flat_scalar_section(path, 'world_rules')
    else:
        with open(path, 'r', encoding='utf-8') as handle:
            doc = yaml.safe_load(handle) or {}
        section = doc.get('world_rules') or {}
        if 'ros__parameters' in section:
            section = section['ros__parameters']
    return make_rules(section)


def wrap(a):
    """Wrap an angle into [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# --------------------------------------------------------------- basic geometry

def orient(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def seg_point_dist(a, b, p):
    """Distance from point ``p`` to segment ``ab``."""
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    if length2 <= 1e-18:
        return math.hypot(p[0] - ax, p[1] - ay)
    t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / length2
    t = min(1.0, max(0.0, t))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def seg_seg_dist(a, b, c, d):
    """Distance between two segments; 0 when they cross (touching endpoints included)."""
    d1 = orient(c, d, a)
    d2 = orient(c, d, b)
    d3 = orient(a, b, c)
    d4 = orient(a, b, d)
    if ((d1 > 0.0) != (d2 > 0.0)) and ((d3 > 0.0) != (d4 > 0.0)):
        return 0.0
    return min(seg_point_dist(a, b, c), seg_point_dist(a, b, d),
               seg_point_dist(c, d, a), seg_point_dist(c, d, b))


def edges(poly):
    """Edges of a closed polyline (the last one returns to the start)."""
    n = len(poly)
    return [(poly[i], poly[(i + 1) % n]) for i in range(n)]


def perimeter(poly):
    n = len(poly)
    return sum(math.hypot(poly[(i + 1) % n][0] - poly[i][0],
                          poly[(i + 1) % n][1] - poly[i][1]) for i in range(n))


def min_static_clearance(poly, static_pts):
    """Smallest distance from a path to a set of static block centres."""
    best = float('inf')
    for a, b in edges(poly):
        for p in static_pts:
            best = min(best, seg_point_dist(a, b, p))
    return best


def _snap(value, grid):
    return round(round(value / grid) * grid, 6)


def randomize_landmarks(rng, rules):
    """One grid point per colour inside its own quadrant -> ``{colour: [x, y, yaw]}``."""
    lo = rules['quadrant_lo']
    hi = rules['quadrant_hi']
    grid = rules['grid_m']
    if hi <= lo + 2.0 * grid:
        raise ValueError(f'quadrant_lo={lo} / quadrant_hi={hi} 之间放不下一个格点')
    out = {}
    for colour in COLORS:
        sign_x, sign_y = QUADRANT[colour]
        x = _snap(rng.uniform(lo + grid, hi - grid), grid)
        y = _snap(rng.uniform(lo + grid, hi - grid), grid)
        # yaw is always 0: a 0.4 m cube looks the same at any yaw, and the top-view
        # calibration fits the centroid of an axis-aligned square outline.
        out[colour] = [sign_x * x, sign_y * y, 0.0]
    return out


def _random_speeds(rng, rules):
    """Draw a speed per cube, at least ``speed_gap`` apart (rule 4 wants that visible)."""
    lo, hi = rules['speed_lo'], rules['speed_hi']
    if hi - lo < rules['speed_gap']:
        raise ValueError('speed_lo/speed_hi 之间的宽度小于 speed_gap')
    first, second = rng.uniform(lo, hi), rng.uniform(lo, hi)
    for _ in range(64):
        if abs(first - second) >= rules['speed_gap']:
            break
        second = rng.uniform(lo, hi)
    else:
        first, second = lo, hi
    if rng.random() < 0.5:
        first, second = second, first
    return {OBSTACLES[0]: round(first, 3), OBSTACLES[1]: round(second, 3)}


# ---------------------------------------------------------------------------
# Random walk: the black and white cubes roam the whole arena, and collisions are
# excluded purely by occupancy exclusion.
# ---------------------------------------------------------------------------
#
# There is no territory and no patrol ring here.  Both cubes wander the arena freely,
# and "no collision" rests on two layers:
#
#   * At generation time (short time window): `_PeerField` remembers only what the other
#     cube walked within the last `walk_peer_window` ticks (12 ticks = 0.6 s) and keeps
#     `walk_peer_keep` away from it.  Once the window has slid past, that floor becomes
#     usable again, and that is what lets both cubes reach the whole arena.  A permanent
#     keep-out cannot work here: over a 60 s random walk both cubes sweep almost every
#     reachable cell, so any permanent spatial constraint ends up covering the arena and
#     walls each cube into half of it.  Coverage alone would not reveal that (two
#     half-arena paths still add up to a large number), which is why `_quad_stats`
#     requires every cube to leave a footprint in all four quadrants.
#   * At run time (cell level): each cube claims the cell it wants next and waits when
#     the claim fails; after `walk_yield` blocked ticks it backs up along its own loop.
#     This layer covers the static landmarks, the walls and the other cube, and it is the
#     final arbiter.
#
# Giving way by a single step does not help: the freed cell is claimed again on the very
# next tick, so the two cubes meet head-on on one line and neither can resolve it.  The
# commitment to back up for `walk_yield_lock` ticks carries the cube about 2.3 m away
# (40 ticks at 0.057 m/tick), which is usually far enough for the paths to diverge.
#
# A cube that backs up can still stall: when the cell behind it is taken by the other
# cube the commitment is cancelled and the pair settles into a limit cycle.  A longer
# commitment period behaves identically, because the blocking condition is the occupied
# rear cell and not the tick count.  The cube that has failed to advance for
# `walk_mutual_after` ticks is therefore released one-sidedly: it reverses along its own
# loop and leaves the crossing from the other end, while the other cube gets a "may not
# retreat" gate.  Letting both cubes move at once only meets them again at the same
# phase offset, so the release has to stay single-sided.
#
# The paths are built by simulation (`_build_tracks`): both cubes are walked once under
# the same tick rate (`sim_rate_hz`) and the same occupancy rules the node uses, and the
# resulting polylines are written to the scene file for the node to replay.  The offline
# self-test and the simulation therefore replay the same path under the same rules.

WANDER_DEFAULTS = {
    'cell_size': 0.10,        # side of one occupancy cell
    'walk_seconds': 60.0,     # simulated seconds per path, both cubes walking together
    'walk_candidates': 12,    # candidate directions resampled per tick
    'walk_cone': 1.10,        # spread of the candidates around the target heading, rad
    'walk_probe': 6,          # ticks a candidate is simulated ahead before it is scored
    'walk_arrive': 0.45,      # distance at which the path counts as closed at its start
    'walk_bias_turn': 0.18,   # max heading turn per tick, rad; same order as max_yaw_rate / rate_hz
    'walk_path_step': 0.16,   # vertex spacing of the stored polyline, m; large rounds corners
    'walk_escape': 8.0,       # stuck ticks after which candidates cover the full circle
    'walk_stuck': 400,        # non-moving ticks in a row that discard the path and reseed
    'walk_yield': 4,          # Blocked ticks after which a lower-priority cube starts backing away
    # Commitment period (ticks): once a cube decides to back up it keeps going this long.
    # A single step back livelocks -- the freed 0.04 m cell is claimed again on the very
    # next tick, so the two cubes meet head-on on one line and neither can resolve it (see
    # `replay_stats`).  Over 40 ticks (~2.3 m at 0.057 m/tick) the paths usually diverge.
    'walk_yield_lock': 40,
    # Slack generation keeps over validation, m: the stored polyline is thinned by
    # `walk_path_step`, so the chord between two vertices cuts inside the true arc by about
    # path_step**2 / (8 R) ~ 6 mm.  Validation measures the chord while generation keeps the
    # vertex, and the same keep on both sides leaves no margin at all -- a few mm short.
    'walk_chord_slack': 0.03,
    'walk_peer_keep': 0.85,   # minimum Chebyshev distance between the two paths; see below
    # That distance applies only to where the other cube walked within the last
    # `walk_peer_window` ticks (a sliding window).  Once the window has passed the floor is
    # free again, which is what lets both cubes reach the whole arena.
    'walk_peer_window': 12,
    # Deadlock release (ticks): a cube that has failed to advance for this many ticks
    # reverses and leaves the crossing along the other end of its loop, while the other cube
    # gets a "may not retreat" gate (lengthened x1 / x2 / x3 / x4 per round), which keeps the
    # release one-sided.  "Lower priority yields, higher priority only waits" handles steep
    # crossings, but not the stretch where the two paths run side by side: the lower-priority
    # cube backs up until the cell behind it is taken by the other cube, which cancels the
    # commitment and folds back into a limit cycle.  A longer commitment period behaves
    # identically, because the blocking condition is the occupied rear cell and not the tick
    # count, and both cubes moving at once meets them again at the same phase offset.
    'walk_mutual_after': 120,
    'walk_mutual_lock': 60,   # base gate (ticks) imposed on the other cube, x4 at most
    'walk_laps_min': 1.0,     # laps each cube must really complete (main criterion, see replay)
    'sim_rate_hz': 20.0,      # control rate of the joint simulation; must match the node rate
    'sim_gap_min': 0.42,      # lower bound on the centre distance (0.4 m cubes cannot overlap)
    'sim_stall_max': 0.65,    # stalled-tick fraction bound, diagnostics only (not a criterion)

    # Robot keep-out at path level: a cube path has to stay clear of the parked robot at
    # `robot_spawn_*`, see `_walk_robot_keep`.  spawn_x / spawn_y are passed through by the
    # launch file (`world_setup.py --spawn-x/--spawn-y`); the default is the arena centre.
    'robot_half': 0.22,       # robot radius, the nav2 ``robot_radius`` value
    'clear_robot': 0.70,      # minimum Chebyshev clearance between a path and the robot
    # Must stay consistent with ``robot_keep`` in the ``dynamic_obstacles`` section, see
    # `_walk_robot_keep`: a path is a closed loop fixed at start-up, and at run time a cube
    # only waits or backs up when its next position would enter the robot's keep-out square,
    # it never edits its path.  A design-time clearance smaller than the run-time square would
    # let the path cross the square, and the cube would stall there.  Change both together.
    'robot_keep': 1.10,
    'robot_spawn_x': 0.0,
    'robot_spawn_y': 0.0,
    # The "roam everywhere" group.  A heading random walk alone is not enough: the heading
    # decorrelates only after hundreds of ticks, so a 60 s path keeps grazing the arena on the
    # same line and collapses into a narrow band.  Hence two additions, a novelty grid and an
    # aiming term in the heading, plus coverage as a hard criterion that can reseed a scene.
    'walk_grid': 0.30,        # coarse cell shared by the novelty score and the coverage
    'walk_novelty': 0.60,     # weight of freshness in the per-tick candidate score
    'walk_novelty_cap': 2,    # passes over a cell that count as used up (keeps seeking new ground)
    'walk_aim_gain': 0.85,    # share of the aiming term in the free-run heading (0 = pure bias)
    'walk_aim_rays': 16,      # rays cast when aiming
    'walk_aim_reach': 3.20,   # ray length, m; too short sees one cell ahead and spins in place
    'walk_aim_replan': 8,     # ticks between two aim computations
    'walk_cover_min': 0.18,   # coverage floor for one cube, share of the arena's usable cells
    'walk_cover_union': 0.42,  # coverage floor for the two cubes together
    'walk_quad_min': 0.08,    # per-quadrant coverage floor, relative to the cells it covered
}

# Merged into DEFAULT_RULES so that `make_rules` does not filter params.yaml's walk_* /
# sim_* keys out as unknown names.
DEFAULT_RULES.update(WANDER_DEFAULTS)


def _walk_rules(rules):
    """Fill in the random-walk keys, keeping whatever value params.yaml supplied."""
    out = dict(rules)
    for key, value in WANDER_DEFAULTS.items():
        out.setdefault(key, value)
    return out


def _walk_half(rules):
    """Occupancy half-side: the cube's circumscribed circle radius, the conservative one."""
    return float(rules['block_half']) * math.sqrt(2.0)


def _walk_static_half(rules):
    """Occupancy half-side of a static landmark: it never turns, so the true extent fits."""
    return float(rules['block_half'])


def _walk_slack(rules):
    """Extra clearance (m) the generator keeps beyond the validation criterion.

    The stored path is thinned to `walk_path_step` (vertices about 0.16 m apart) and the
    chord between two vertices cuts inside the true arc by about path_step**2/(8R), which
    is roughly 6 mm at the minimum turning radius.  `validate_scene` measures chords while
    the generator only guarantees clearance at the vertices, so using the same keep on
    both sides leaves no margin at all and a scene gets rejected over a millimetre-sized
    gap.
    """
    return max(0.0, float(rules['walk_chord_slack']))


def _walk_keep(rules):
    """Hard clearance from a static block: Chebyshev (box), not Euclidean distance.

    Runtime occupancy asks whether the grid cells overlap, which is equivalent to the
    Chebyshev distance: with the boxes separated by K on one axis, each still needs a
    rounding cell, so
        K >= half + static_half + 2*cell
    keeps the two cell sets apart, plus one more cell for rounding a vertex to 0.1 mm.

    This is the line validation checks; generation adds `_walk_slack()` on top of it.
    """
    need = (_walk_half(rules) + _walk_static_half(rules)
            + 2.0 * float(rules['cell_size']))
    return max(float(rules['clear_static']), need) + float(rules['cell_size'])


def _walk_robot_keep(rules):
    """Clearance the path keeps from the robot parking spot (Chebyshev as well).

    The robot sits at `robot_spawn_x/y` (arena centre by default) while the cubes run, so
    the path has to go around it geometrically: the loop is fixed at start-up and a cube
    has no room to yield on the fly.

    This value must be >= the runtime `robot_keep`, otherwise the path crosses the runtime
    exclusion ring: a cube reaching that stretch would see its next cell inside the ring,
    wait, stay blocked for `yield_ticks`, retreat, come back along the same stretch and be
    blocked again, so it never leaves the neighbourhood of the robot and coverage
    collapses.  It cannot actually hit the robot, the exclusion ring still holds.

    The lower bound stays the larger of `clear_robot` and "outer half-diagonal + robot
    occupancy radius + two cells", so lowering `robot_keep` cannot make the path hug the
    robot more than before.
    """
    need = (_walk_half(rules) + float(rules['robot_half'])
            + 2.0 * float(rules['cell_size']))
    keep = max(float(rules['clear_robot']), need,
               float(rules.get('robot_keep', 1.10)))
    return keep + float(rules['cell_size'])


def _robot_pts(rules):
    """Robot parking spot, kept as a point list so further spots need no new criterion."""
    return [(float(rules['robot_spawn_x']), float(rules['robot_spawn_y']))]


def _blockers(rules, static_pts, robot_pts=None, slack=0.0):
    """Every point a path must not enter, each with its own clearance radius.

    Static landmarks and the robot parking spot share one table and differ only in the
    radius (`_walk_keep` / `_walk_robot_keep`); start selection, per-tick advance, closing
    and offline validation then all consult the same list.

    ``slack`` is the extra margin generation adds (`_walk_slack`).  Validation always uses
    ``slack=0``, so the hard criterion is the line runtime really has to satisfy and the
    margin only exists while building a path.
    """
    pad = max(0.0, float(slack))
    out = [(float(p[0]), float(p[1]), _walk_keep(rules) + pad) for p in static_pts]
    for point in (_robot_pts(rules) if robot_pts is None else robot_pts):
        out.append((float(point[0]), float(point[1]), _walk_robot_keep(rules) + pad))
    return out


def _clear_of(x, y, blockers):
    """Whether (x, y) is far enough from every forbidden point, Chebyshev as at runtime."""
    for bx, by, keep in blockers:
        if abs(x - bx) < keep and abs(y - by) < keep:
            return False
    return True


def _walk_peer_keep(rules):
    """Minimum separation between the two paths, Chebyshev (stricter than at runtime).

    Runtime treats a cube as its circumscribed circle of radius ``half`` and forbids two
    circles to intersect, i.e. centre distance >= ``2*half = 0.5657 m``.  Generation asks
    for the same distance in Chebyshev terms: the inscribed circle of a Chebyshev square
    has its half side as radius, so ``K = 2*half + slack`` already implies the Euclidean
    bound and no cell-rounding allowance is needed, since the runtime no longer rounds.

    The criterion is sliding-window scoped, only the peer's last `walk_peer_window` ticks
    are reserved rather than the whole path.
    """
    need = 2.0 * _walk_half(rules) + _walk_slack(rules)
    return max(float(rules['walk_peer_keep']), need)


def _min_cheb(poly, blockers, step):
    """Smallest margin along the path to any forbidden point: Chebyshev distance - keep.

    A value >= 0 means every point of the path passes; a negative value means the closest
    point still lacks ``-value``.  The criterion matches the runtime occupancy test, so
    "offline >= 0" implies "no contact at run time".
    """
    if not blockers:
        return float('inf')
    cum, total = path_geometry(poly)
    best = float('inf')
    s = 0.0
    while s <= total:
        point = point_along(poly, cum, total, s)[0]
        for bx, by, keep in blockers:
            best = min(best, max(abs(point[0] - bx), abs(point[1] - by)) - keep)
        s += step
    return best


def _min_static_cheb(poly, static_pts, step):
    """Smallest Chebyshev distance from the path to a set of equal-clearance points."""
    return _min_cheb(poly, [(p[0], p[1], 0.0) for p in static_pts], step)


def _walk_limit(rules, half):
    return float(rules['arena_half']) - float(rules['wall_margin']) - half


def _pick_home(rng, taken, blockers, rules, half, lim):
    """Pick the start position: clear of every forbidden point and >= 1.0 m from the peer.

    The start is the first vertex of the path, so the same forbidden table as `_Walker` is
    used; otherwise a legal start could still have its first leg run through the robot.
    """
    for _ in range(400):
        x, y = rng.uniform(-lim, lim), rng.uniform(-lim, lim)
        if not _clear_of(x, y, blockers):
            continue
        if any(max(abs(x - q[0]), abs(y - q[1])) < 1.0 for q in taken):
            continue
        return (round(x, 4), round(y, 4))
    return None


class _FastClaims:
    """Occupancy registry, rotation invariant: owners are circles, conflict = intersection.

    ``half`` is the circumscribed radius of the real shape (``block_half*sqrt(2)`` for a
    cube), so "circles do not intersect" implies "cubes do not intersect" and the test is
    conservative.  It is also rotation invariant: the hard threshold stays
    ``2*half = 0.5657 m`` whatever the headings, so the verdict never flickers.

    ``cell`` is left for the `CellClaims` debug map and takes no part in the verdict.
    """

    def __init__(self, cell=0.10, half=0.283):
        self.cell = float(cell) if cell and cell > 0.0 else 0.10
        self.half = float(half)
        self.owner = {}

    def conflict(self, owner, x, y, half):
        for name, (ox, oy, oh) in self.owner.items():
            if name == owner:
                continue
            if math.hypot(x - ox, y - oy) < half + oh:
                return name
        return None

    def commit(self, owner, x, y, half):
        self.owner[owner] = (float(x), float(y), float(half))

    def release(self, owner):
        self.owner.pop(owner, None)

    def reset(self):
        self.owner.clear()


def _box_corners(x, y, yaw, half):
    """The four corners of a 0.4 m cube, axis aligned at yaw=0."""
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for dx, dy in ((half, half), (half, -half), (-half, -half), (-half, half)):
        out.append((x + dx * c - dy * s, y + dx * s + dy * c))
    return out


def _boxes_overlap(a, b):
    """Separating-axis test for two cubes: no overlap when any edge normal separates them."""
    for first, second in ((a, b), (b, a)):
        for k in range(4):
            ex = first[(k + 1) % 4][0] - first[k][0]
            ey = first[(k + 1) % 4][1] - first[k][1]
            nx, ny = -ey, ex
            pa = [nx * px + ny * py for px, py in first]
            pb = [nx * px + ny * py for px, py in second]
            if max(pa) <= min(pb) or max(pb) <= min(pa):
                return False
    return True


def claims_agree(cell=0.10, half=0.283, box_half=0.2, samples=200, seed=7):
    """Self-test of two properties, called by `world_setup.py --selftest`:

    1. `_FastClaims` and `CellClaims` agree bit for bit on the same poses;
    2. safety: every pose declared conflict-free really is overlap-free when checked with
       the true 0.4 m cubes at random headings using SAT.  The second property is the one
       worth keeping, it catches any later relaxation of the criterion.
    """
    import random as _random
    fast = _FastClaims(cell, half)
    slow = CellClaims(cell)
    rng = _random.Random(seed)
    names = ['a', 'b', 'c']
    for _ in range(samples):
        placed = {}
        for name in names:
            x, y = rng.uniform(-3.5, 3.5), rng.uniform(-3.5, 3.5)
            fast.commit(name, x, y, half)
            slow.commit(name, x, y, half)
            placed[name] = (x, y)
        for name in names:
            x, y = rng.uniform(-3.5, 3.5), rng.uniform(-3.5, 3.5)
            got = fast.conflict(name, x, y, half)
            want = slow.conflict(name, x, y, half)
            if (got is None) != (want is None):
                return False, ('impl', name, round(x, 3), round(y, 3), got, want)
            if got is not None:
                continue
            a = _box_corners(x, y, rng.uniform(-math.pi, math.pi), box_half)
            for other, (ox, oy) in placed.items():
                if other == name:
                    # The cube's own cell from last tick says nothing: a cube can sit as close to
                    # itself as it likes, and the checks exclude it by owner name.
                    continue
                b = _box_corners(ox, oy, rng.uniform(-math.pi, math.pi), box_half)
                if _boxes_overlap(a, b):
                    return False, ('overlap', name, other, round(x, 3), round(y, 3),
                                   round(ox, 3), round(oy, 3))
    return True, None


class _PeerField:
    """Index of the peer path, queried while walking, Chebyshev (stricter than runtime).

    Two modes selected by `window`:

    * ``window > 0`` (sliding time window): only the peer's last ``window`` ticks are
      stored, and an area becomes usable again once the window has moved past it.
    * ``window <= 0`` (permanent): every position the peer has ever visited is dilated by
      ``keep`` and blocked for the whole run.

    The generator uses the sliding window because the arena is only 8 x 8 m and a 105 s
    random walk covers nearly every reachable cell, so any "keep the two paths apart
    everywhere" constraint walls each cube into half of the arena.

    A window is sound only while the runtime reproduces the generation-time tick for tick,
    which holds: ``advance(dt)`` always sees ``dt = 1/sim_rate_hz`` from the
    `dynamic_obstacles.on_control` budget, so tick n at run time is tick n during
    generation, and both sides use the same occupancy formula (circumscribed circle plus
    Euclidean distance), so a conflict-free generation implies a conflict-free run and the
    arbitration never fires.

    Permanent-mode dilation uses a grid of side ``cell`` with radius ``ceil(keep / cell)``
    cells: an index difference of ``radius + 1`` or more means a Chebyshev distance of at
    least ``radius * cell >= keep``, so "not blocked" is equivalent to "at least keep away
    from the peer path everywhere", with one cell of conservative rounding.
    """

    def __init__(self, keep, window, cell=0.10):
        self.keep = float(keep)
        self.window = int(window)
        self.cell = float(cell) if cell and cell > 0.0 else 0.10
        self.permanent = self.window <= 0
        self.points = []
        self.marked = set()
        self.last = None
        self.radius = (int(math.ceil(self.keep / self.cell))
                       if self.permanent else 0)

    def _index(self, x, y):
        return (int(math.floor(x / self.cell)), int(math.floor(y / self.cell)))

    def add(self, x, y):
        key = self._index(x, y)
        if key == self.last:
            # A cube that did not move (held by the occupancy table or a blocker) must not write the
            # same point into the window again: one coordinate would flood the sliding window, the
            # peer keep-out would collapse to a single cell, and the peer path would run straight
            # through it.  Both the permanent and the windowed mode share this de-duplication.
            return
        self.last = key
        if not self.permanent:
            self.points.append((float(x), float(y)))
            if len(self.points) > self.window:
                del self.points[0]
            return
        i, j = key
        radius = self.radius
        marked = self.marked
        for di in range(-radius, radius + 1):
            for dj in range(-radius, radius + 1):
                marked.add((i + di, j + dj))

    def clear_enough(self, x, y):
        """Whether (x, y) is far enough: whole path in permanent mode, nearest part else."""
        if self.permanent:
            return self._index(x, y) not in self.marked
        keep = self.keep
        for px, py in self.points:
            if abs(x - px) < keep and abs(y - py) < keep:
                return False
        return True


def _quad_stats(paths, rules, blockers):
    """Cells each path sweeps per quadrant, as a share of the cells that path sweeps.

    A combined total hides a split into half-arenas, so roaming has to hold per cube: each
    one must leave a footprint in all four quadrants, which `validate_scene` enforces.

    Quadrant numbers follow the IViz convention: I top right, II top left, III bottom
    left, IV bottom right.
    """
    cell = float(rules['walk_grid'])
    box = _cover_box(rules, blockers)
    out = {}
    for colour, poly in paths.items():
        cells = _cover_cells(poly, cell) & box
        quad = {'I': 0, 'II': 0, 'III': 0, 'IV': 0}
        for i, j in cells:
            x, y = (i + 0.5) * cell, (j + 0.5) * cell
            if y >= 0.0:
                quad['I' if x >= 0.0 else 'II'] += 1
            else:
                quad['IV' if x >= 0.0 else 'III'] += 1
        total = max(1, len(cells))
        out[colour] = {key: quad[key] / float(total) for key in ('I', 'II', 'III', 'IV')}
    return out


class _Fence:
    """Unused: coarse-grid permanent exclusion of the peer path.

    Superseded by `_PeerField`, whose sliding window is the mode the generator uses.
    """

    def __init__(self, keep, cell):
        self.keep = float(keep)
        self.cell = float(cell) if cell and cell > 0.0 else 0.30
        radius = (self.keep + 0.5 * self.cell) / self.cell
        self.radius = max(1, int(math.ceil(radius)))
        self.marked = set()
        self.last = None

    def _index(self, x, y):
        return (int(math.floor(x / self.cell)), int(math.floor(y / self.cell)))

    def add(self, x, y):
        key = self._index(x, y)
        if key == self.last:
            return
        self.last = key
        radius = self.radius
        i, j = key
        for di in range(-radius, radius + 1):
            for dj in range(-radius, radius + 1):
                self.marked.add((i + di, j + dj))

    def blocked(self, x, y):
        return self._index(x, y) in self.marked


def _cover_cells(poly, cell):
    """Set of cells a closed path sweeps, sampled along the path at half-cell spacing."""
    seen = set()
    if len(poly) < 2:
        return seen
    cum, total = path_geometry(poly)
    step = 0.5 * cell
    s = 0.0
    while s <= total:
        x, y = point_along(poly, cum, total, s)[0]
        seen.add((int(math.floor(x / cell)), int(math.floor(y / cell))))
        s += step
    return seen


def _cover_box(rules, blockers):
    """Coverage denominator: cells of the arena that are actually reachable, clear of blockers."""
    cell = float(rules['walk_grid'])
    lim = _walk_limit(rules, _walk_half(rules))
    span = int(math.floor(lim / cell))
    out = set()
    for i in range(-span, span + 1):
        for j in range(-span, span + 1):
            cx, cy = (i + 0.5) * cell, (j + 0.5) * cell
            if abs(cx) > lim or abs(cy) > lim:
                continue
            if _clear_of(cx, cy, blockers):
                out.add((i, j))
    return out


def _cover_stats(paths, rules, blockers):
    """Coverage of the two paths: ``(per path, union of both)``.

    Distance travelled alone is misleading, so roaming is measured as the share of
    reachable cells a path sweeps, which is the criterion `validate_scene` uses.
    """
    cell = float(rules['walk_grid'])
    box = _cover_box(rules, blockers)
    if not box:
        return {colour: 0.0 for colour in paths}, 0.0
    cells = {colour: _cover_cells(paths[colour], cell) & box for colour in paths}
    per = {colour: len(cells[colour]) / float(len(box)) for colour in paths}
    union = len(set().union(*cells.values())) / float(len(box)) if cells else 0.0
    return per, union


class _Walker:
    """Random walk of one cube, one step per tick, the same beat as the node at run time.

    `tick()` returns True when the cube advanced.  When the next cell is taken or the cube
    is cornered it releases its cell, picks another direction and returns False, standing
    still for that tick.
    """

    def __init__(self, name, home, yaw, speed, blockers, cell, rules, rng):
        self.name = name
        self.home = (float(home[0]), float(home[1]))
        self.x, self.y = self.home
        self.yaw = float(yaw)
        self.speed = float(speed)
        self.rules = rules
        self.rng = rng
        self.cell = cell
        # `blockers` is a list of `(x, y, keep)` points: static landmarks plus the robot spot.
        # Both carry their own clearance radius, so every occupancy test below covers both.
        self.blockers = blockers
        self.half = _walk_half(rules)
        self.static_half = _walk_static_half(rules)
        self.keep = _walk_keep(rules)
        # Distance kept from the peer path, permanent occupancy, see `_PeerField`.
        # Path-level avoidance: the paths stay apart, so they never share an occupancy box.
        self.peer_keep = float(rules['walk_peer_keep'])
        self.peer = None
        self.field = _PeerField(self.peer_keep, rules['walk_peer_window'],
                                float(rules['cell_size']))
        # The cube's own spawn point is indexed into its path right away, for the peer to query.
        self.field.add(self.x, self.y)
        self.lim = _walk_limit(rules, self.half)
        self.step = self.speed / float(rules['sim_rate_hz'])
        self.travel = self.speed * float(rules['walk_seconds'])
        # Corner escape turns the two cubes opposite ways: turning alike keeps their relative
        # bearing fixed and leaves both wedged in the corner, neither able to move.
        self.spin = 1.0 if name == OBSTACLES[0] else -1.0
        # Turn limit per tick, derived from max_yaw_rate so replay stays within reach.
        self.max_turn = float(rules['max_yaw_rate']) / float(rules['sim_rate_hz'])
        self.cone = float(rules['walk_cone'])
        self.probe = max(3, int(rules['walk_probe']))
        self.n_cand = max(2, int(rules['walk_candidates']))
        self.arrive = float(rules['walk_arrive'])
        self.bias_turn = float(rules['walk_bias_turn'])
        self.path_step = max(float(rules['walk_path_step']), self.step)
        # Resolution of the chord test.  Stored paths are chords, interpolated by `point_along`,
        # while keep-out regions are squares (Chebyshev).  A square is convex but its complement
        # is not, so a chord can cut a corner, so every chord is sampled on its own as well.
        self.chord_grid = min(0.02, float(rules['cell_size']))
        self.pending = []
        # Current heading bias: slow and persistent, never resampled per tick, which would spin.
        self.bias = self.yaw
        # Novelty: visit count on the coarse grid.  A heading random walk alone grazes the arena
        # along the same line and collapses into a narrow band, so unvisited cells score higher.
        self.novelty_w = float(rules['walk_novelty'])
        self.novelty_cap = max(1, int(rules['walk_novelty_cap']))
        self.grid = max(1e-3, float(rules['walk_grid']))
        self.visits = {}
        # Aiming term: every `aim_replan` ticks the target heading is set towards the emptiest
        # direction nearby and leads the free-run heading, the random walk being a perturbation.
        self.aim_weight = min(1.0, max(0.0, float(rules['walk_aim_gain'])))
        self.aim_rays = max(4, int(rules['walk_aim_rays']))
        self.aim_reach = max(self.step * 2.0, float(rules['walk_aim_reach']))
        self.aim_every = max(1, int(rules['walk_aim_replan']))
        self.aim = self.yaw
        self.escape = float(rules['walk_escape'])
        self.stuck = int(rules['walk_stuck'])
        self.pts = [(round(self.x, 4), round(self.y, 4))]
        self.travelled = 0.0
        self.stalled = 0
        self.stall_ticks = 0
        self.ticks = 0
        self.done = False
        self.failed = False

    def _dist(self, point):
        return math.hypot(point[0] - self.x, point[1] - self.y)

    def _free(self, x, y):
        """Whether (x, y) may be taken: inside the arena, clear of blockers and of the peer."""
        if abs(x) > self.lim or abs(y) > self.lim:
            return False
        if not _clear_of(x, y, self.blockers):
            return False
        if self.peer is not None and not self.peer.clear_enough(x, y):
            return False
        return True

    def _span(self, ux, uy):
        """How many ticks the cube can advance along (ux, uy) before a wall or a blocker.

        The farthest reachable point comes back as well, callers need it to judge whether
        this direction would hug a wall.
        """
        n = 0
        ex, ey = self.x, self.y
        for k in range(1, self.probe + 1):
            x, y = self.x + ux * self.step * k, self.y + uy * self.step * k
            if abs(x) > self.lim or abs(y) > self.lim:
                break
            if not _clear_of(x, y, self.blockers):
                break
            if self.peer is not None and not self.peer.clear_enough(x, y):
                break
            n = k
            ex, ey = x, y
        return n, ex, ey

    def _chord_ok(self, a, b):
        """Whether the chord a->b, not the arc, stays outside every forbidden area.

        Samples are taken every `chord_grid` (2 cm).  Chebyshev distance along a straight
        line is convex, so the sampled minimum overshoots the true one by at most half a
        step (<= 1 cm), which is conservative enough for a boundary test.
        """
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        if length <= 1e-9:
            return True
        steps = max(1, int(math.ceil(length / self.chord_grid)))
        for k in range(1, steps + 1):
            px = a[0] + (b[0] - a[0]) * k / steps
            py = a[1] + (b[1] - a[1]) * k / steps
            if not _clear_of(px, py, self.blockers):
                return False
            if self.peer is not None and not self.peer.clear_enough(px, py):
                return False
        return True

    def _drain(self, force=False):
        """Fold the per-tick positions in `self.pending` into the path (thin + chord check).

        Thinning cannot rely on vertex clearance alone: forbidden areas are squares
        (Chebyshev) whose complement is not convex, so two legal vertices can still have a
        chord cutting a corner, by roughly 0.35 of the chord length in the worst case.  A
        failing chord pushes a nearer point back in, which adapts the sampling density.
        """
        if not self.pending:
            return
        if not force:
            last = self.pts[-1]
            far = self.pending[-1]
            if math.hypot(far[0] - last[0], far[1] - last[1]) < self.path_step:
                return
        while self.pending:
            last = self.pts[-1]
            take = None
            for index in range(len(self.pending) - 1, -1, -1):
                if self._chord_ok(last, self.pending[index]):
                    take = index
                    break
            if take is None:
                # No chord fits this tick: record the shortest, most conservative one and move on.
                take = 0
            for px, py in self.pending[:take + 1]:
                self.pts.append((round(px, 4), round(py, 4)))
            del self.pending[:take + 1]
            if not force:
                return

    def _finish(self):
        """Back near the start: append the first point to close the loop, clearance checked."""
        x, y = self.home
        distance = math.hypot(x - self.x, y - self.y)
        # The closing segment is a chord too and gets the same fine sampling as the rest.
        steps = max(1, int(math.ceil(distance / self.chord_grid)))
        closing = []
        for k in range(1, steps + 1):
            px = self.x + (x - self.x) * k / steps
            py = self.y + (y - self.y) * k / steps
            if not _clear_of(px, py, self.blockers):
                return False
            if self.peer is not None and not self.peer.clear_enough(px, py):
                return False
            closing.append((px, py))
        for px, py in closing:
            self.field.add(px, py)
        self._drain(force=True)
        last = (round(x, 4), round(y, 4))
        if self.pts[-1] != last:
            self.pts.append(last)
        self.done = True
        return True

    def _cell_key(self, x, y):
        return (int(math.floor(x / self.grid)), int(math.floor(y / self.grid)))

    def _novelty(self, x, y):
        """How novel a candidate is: 0 = well trodden, 1 = never visited."""
        seen = min(self.visits.get(self._cell_key(x, y), 0), self.novelty_cap)
        return 1.0 - seen / float(self.novelty_cap)

    def _ray_novelty(self, ux, uy, span):
        """Average novelty along the whole ray.

        Looking at the ray endpoint alone is not enough: when the whole neighbourhood is
        trodden every endpoint scores 0 and the search degenerates into walking straight
        on, leaving the cube inside the area it already swept.  Averaging along the ray
        lets a direction with fresh ground ahead of it win.
        """
        if span <= 0:
            return 0.0
        total = 0.0
        for k in range(1, span + 1):
            total += self._novelty(self.x + ux * self.step * k,
                                   self.y + uy * self.step * k)
        return total / span

    def _plan_aim(self):
        """Short-term heading towards the nearest fresh ground that is still walkable.

        This is what makes the walk roam: a heading random walk decorrelates only after
        hundreds of ticks, so a 60 s path rubs back and forth over the same strip.

        Rays are cast in metres, not ticks.  In tick units the look-ahead is only
        `walk_probe * step` (about 0.35 m), which reduces the search to testing the cell in
        front and keeps the cube circling inside the area it already swept.  Here a ray
        spans `walk_aim_reach` metres in `walk_grid` steps so that remaining fresh cells
        dominate the score and the walk really drifts over the whole arena.  A random
        perturbation keeps the trajectory from repeating.
        """
        best, best_score = self.aim, None
        rays = self.aim_rays
        march = max(self.grid, self.step)
        count = max(1, int(self.aim_reach / march))
        for k in range(rays):
            theta = wrap(self.yaw + 2.0 * math.pi * k / rays)
            ux, uy = math.cos(theta), math.sin(theta)
            span, fresh = 0, 0.0
            for j in range(1, count + 1):
                s = j * march
                x, y = self.x + ux * s, self.y + uy * s
                if abs(x) > self.lim or abs(y) > self.lim:
                    break
                if not _clear_of(x, y, self.blockers):
                    break
                if self.peer is not None and not self.peer.clear_enough(x, y):
                    break
                span = j
                fresh += self._novelty(x, y)
            if span < 1:
                continue
            reach = min(span * march, self.aim_reach) / self.aim_reach
            # `fresh` is a sum, not an average: an average would let brushing one new cell win over
            # crossing visited ground towards a large unvisited area, so the cube would peck along
            # the boundary of what it already covered instead of spreading out.
            score = (0.22 * fresh + 1.00 * reach
                     - 0.22 * abs(wrap(theta - self.yaw)) / math.pi)
            if best_score is None or score > best_score:
                best, best_score = theta, score
        self.aim = best
        return best

    def tick(self):
        if self.done:
            return True
        self.ticks += 1
        if self.ticks > self.stuck * 40:
            self.failed = True
            return False
        if self.travelled >= self.travel:
            # Closing segment: head back to the start so the path becomes a closed loop.
            if self._dist(self.home) < self.arrive and self._finish():
                return True
            want = math.atan2(self.home[1] - self.y, self.home[0] - self.x)
            aim = want
        else:
            # Free run: the heading random-walks on its own, slowly and with persistence, combined
            # with an aiming term recomputed every few ticks: random, and covering the arena.
            self.bias = wrap(self.bias + self.rng.uniform(-1.0, 1.0) * self.bias_turn)
            if self.ticks % self.aim_every == 1:
                self._plan_aim()
            aim = self.aim
            want = wrap(self.bias + self.aim_weight * wrap(aim - self.bias))

        # The longer the cube is stuck the wider the candidates spread, up to the full circle.
        widen = self.cone + min(1.0, self.stalled / self.escape) * math.pi
        best, best_score = None, None
        for k in range(self.n_cand):
            delta = 0.0 if k == 0 else self.rng.uniform(-widen, widen)
            probe = wrap(want + delta)
            # The cube is a differential drive: it can turn at most max_turn per tick, so it walks
            # in directions it can reach while moving; stopping to turn wastes most of the time.
            turn = wrap(probe - self.yaw)
            theta = wrap(self.yaw + math.copysign(min(abs(turn), self.max_turn), turn))
            span, ex, ey = self._span(math.cos(theta), math.sin(theta))
            # Normalised weighted score: walk freely first, no dead ends, then avoid turning back
            # and hitting walls, then seek unvisited ground.  Novelty is what keeps coverage up.
            score = (min(span, self.probe) / self.probe
                     - 0.55 * abs(wrap(theta - self.yaw)) / math.pi
                     - 0.25 * abs(wrap(theta - want)) / math.pi
                     + 0.20 * min(self.lim - abs(ex), self.lim - abs(ey)) / self.lim
                     + self.novelty_w * self._ray_novelty(math.cos(theta),
                                                          math.sin(theta), span))
            if span >= 1 and (best_score is None or score > best_score):
                best, best_score = theta, score
        if best is None:
            # Nothing is reachable this tick: turn a little in place towards the fixed side, never
            # along the walk direction, which would drag the nose around inside a dead end.  A full
            # turn is certain to find a way out, and the two cubes turn opposite ways.
            self.yaw = wrap(self.yaw + self.spin * self.max_turn)
            self.stalled += 1
            self.stall_ticks += 1
            if self.stalled >= self.stuck:
                self.failed = True
            return False

        x = self.x + math.cos(best) * self.step
        y = self.y + math.sin(best) * self.step
        self.cell.commit(self.name, x, y, self.half)
        self.x, self.y, self.yaw = x, y, best
        self.travelled += self.step
        self.stalled = 0
        # Record the visit for next tick's novelty score, the start cell included.
        key = self._cell_key(x, y)
        self.visits[key] = self.visits.get(key, 0) + 1
        # Path thinning: stored vertices are at least path_step apart, but every chord between
        # them has to pass the same checks, see `_drain`.  Testing vertex spacing and vertex
        # clearance alone misses a chord cutting a square corner by ~0.35 of its length.
        self.pending.append((x, y))
        if len(self.pending) > 256:
            del self.pending[:-256]
        self._drain()
        # Indexed densely into its own path for the peer to query; thinned vertices are sparse.
        self.field.add(x, y)
        return True


def _build_tracks(rng, rules, static_pts, speeds, robot_pts=None):
    """Run both cubes under the runtime rules, then emit their paths once they get through.

    Generation walks tick by tick through `_Walker.tick()`, the same beat as the node at
    run time.  The difference is the escape route: during generation a blocked cube may
    turn in place and pick another direction, while at run time it can only advance or
    retreat along its pre-generated loop.  A path that walks here is therefore not yet
    proof that the run will not interlock.

    Interlocking comes from crossing paths rather than from simultaneous overlap: the two
    cubes run at different speeds, so they can reach a crossing at the same phase even
    when generation never brought them close.  Conflicts are therefore resolved in the
    time dimension, by the sliding window of `_PeerField` and `walk_peer_window`.

    Whether the window suffices is not argued but replayed: `validate_scene` replays the
    scene at several relative phases (`PHASE_OFFSETS`) and rebuilds with a new seed when
    any phase deadlocks or a cube walks fewer than `walk_laps_min` laps.

    `robot_pts` holds the robot parking spot (the launch spawn_x/spawn_y, arena centre by
    default).  It joins `blockers`, so a path avoids it geometrically instead of expecting
    a cube to dodge the robot on the fly.
    """
    half = _walk_half(rules)
    cell = _FastClaims(float(rules['cell_size']), half)
    for index, point in enumerate(static_pts):
        cell.commit(f'static_{index}', point[0], point[1], half)

    blockers = _blockers(rules, static_pts, robot_pts)
    # Generation uses the table with extra slack: validation measures the thinned chords
    # while generation only guarantees vertices, see `_walk_slack`.
    walk_blockers = _blockers(rules, static_pts, robot_pts, slack=_walk_slack(rules))
    homes, taken = {}, []
    for colour in OBSTACLES:
        home = _pick_home(rng, taken, walk_blockers, rules, half,
                          _walk_limit(rules, half))
        if home is None:
            return None
        homes[colour] = home
        taken.append(home)
    for colour in OBSTACLES:
        cell.commit(colour, homes[colour][0], homes[colour][1], half)

    walkers = {
        colour: _Walker(colour, homes[colour], rng.uniform(-math.pi, math.pi),
                        speeds[colour], walk_blockers, cell, rules, rng)
        for colour in OBSTACLES
    }
    for colour in OBSTACLES:
        # `walker.field` holds the cube own path and the peer queries it, so this has to be
        # wired to the peer `field`: a fresh empty table would drop the constraint, the two
        # paths would cross and the cubes would meet head-on at some phase.
        walkers[colour].peer = walkers[[c for c in OBSTACLES if c != colour][0]].field
    min_gap = float('inf')
    while True:
        alive = [c for c in OBSTACLES
                 if not walkers[c].done and not walkers[c].failed]
        if not alive:
            break
        for colour in OBSTACLES:
            walkers[colour].tick()
        for i in range(len(alive)):
            for j in range(i + 1, len(alive)):
                a, b = walkers[alive[i]], walkers[alive[j]]
                min_gap = min(min_gap, math.hypot(a.x - b.x, a.y - b.y))
    if any(walkers[c].failed or not walkers[c].done for c in OBSTACLES):
        return None

    tracks = {}
    for colour in OBSTACLES:
        walker = walkers[colour]
        poly = list(walker.pts)
        tracks[colour] = {
            'home': walker.home,
            'path': [[p[0], p[1]] for p in poly],
            'ticks': walker.ticks,
            'stall_ratio': walker.stall_ticks / max(1, walker.ticks),
        }
    # Coverage is measured on the stored, thinned path, which is what the node replays.
    paths = {colour: [(p[0], p[1]) for p in tracks[colour]['path']] for colour in OBSTACLES}
    cover, union = _cover_stats(paths, rules, blockers)
    return tracks, {
        'min_gap': min_gap,
        'ticks': max(w.ticks for w in walkers.values()),
        'cover': cover,
        'cover_union': union,
    }


PHASE_OFFSETS = (0.0, 0.25, 0.5, 0.75)
# Relative phases tried when replaying both paths, the peer start as a share of a lap.
#
# What it covers: both cubes run at phase 0 at run time, `dynamic_paths.yaml` stores
# `*_phase` as 0.0 and the node starts at `s = phase*total`, while run-time advance is
# an integer tick loop at a fixed 1 / sim_rate_hz, so run-time tick n equals generation
# tick n.  The phase-0 replay is a tick-exact preview of run time, the main criterion.
#
# A dense sweep, say 96 even phases, is not used: a phase offset moves the start point
# into the middle of the path, where the two starts can touch as real discs overlap, and
# the replay then jams from tick 0.  That is a property of the phase, not of run time.
# Denser sampling only manufactures false deadlocks that force the generator to reseed.
# The interlock worth catching is a false report where the geometry allows passing but
# the check disagrees, see `_FastClaims` and `world_rules.CellClaims`; the fix is to
# give both sides the same check.  Four quarter points cover that whole class.


def replay_stats(scene, rules=None, max_ticks=40000, phase=0.0, observe=None):
    """Replay both paths with the runtime semantics: arc advance, arbitration, retreat.

    Generation can turn in place when blocked, while the node only advances or retreats
    along its pre-generated loop, so it needs a second way out; otherwise a head-on
    encounter interlocks for good.  This implements the same rules the node uses:

    * every owner claims the cells it wants for the next tick and moves when it gets them;
    * a failed claim means waiting; the high-priority cube (``OBSTACLES[0]``) only ever
      waits and never retreats, so at least one cube always has a way out;
    * a low-priority cube still blocked after ``walk_yield`` ticks retreats along its own
      loop and commits to retreating for ``walk_yield_lock`` ticks (``s`` decreases,
      position and heading follow).  A retreat retraces ground the cube just released, so
      those cells are free;
    * a cube that fails to advance for ``walk_mutual_after`` ticks triggers the deadlock
      release: it reverses and leaves the crossing along the other end of its loop while
      the peer gets the ``hold`` gate, which keeps the release one-sided.

    The release has to be one-sided: with both cubes moving the parallel stretch frees up
    and they turn around together onto the same offset, a symmetric limit cycle.
    Reversing only the longest-stalled cube frees the next cell of the peer as the first
    one walks away, so the peer can advance through that stretch.

    A retreat has to run for a whole commitment, because retreating one step and retrying
    the advance re-claims the freed cell at once and the pair oscillates in place.  For
    the same reason the commitment period blocks advance entirely, otherwise the cube
    grabs the cell back the tick after releasing it; only an occupied cell behind cancels
    the commitment.

    ``phase`` is the starting arc length of the second cube as a fraction of its own lap.
    The cubes run at different speeds, so in a real run their time offset drifts through a
    whole lap; replaying ``phase=0`` alone would miss interlocks where two windows never
    overlap yet still meet head on at some offset, which is why `_one_scene` replays the
    representative offsets in `PHASE_OFFSETS`.

    Replay length is two laps of the slower cube: within two laps the offset between the
    random-walk loops samples every representative phase, while the least common multiple
    of the two lap times can grow to tens of thousands of ticks.

    ``observe`` is an optional per-tick hook for diagnostic scripts, called with
    ``(tick, state, outcome, blocker)``.
    """
    rules = _walk_rules(rules or DEFAULT_RULES)
    half = _walk_half(rules)
    cell = _FastClaims(float(rules['cell_size']), half)
    for index, colour in enumerate(COLORS):
        item = scene['landmarks'][colour]
        cell.commit(f'static_{index}', float(item[0]), float(item[1]), half)

    rate = float(rules['sim_rate_hz'])
    geom, state = {}, {}
    for index, colour in enumerate(OBSTACLES):
        item = scene['dynamic'][colour]
        path = [(float(p[0]), float(p[1])) for p in item['path']]
        cum, total = path_geometry(path)
        s0 = (total * float(phase)) if index else 0.0
        (hx, hy), _ = point_along(path, cum, total, s0)
        step = float(item['speed']) / rate
        geom[colour] = (path, cum, total)
        state[colour] = {'s': s0, 'x': hx, 'y': hy, 'step': step, 'wait': 0,
                         'yield': 0, 'net': 0.0, 'stall_run': 0, 'hold': 0}
        cell.commit(colour, hx, hy, half)

    laps = [geom[c][2] / max(state[c]['step'], 1e-9) for c in OBSTACLES]
    # Two laps of replay are enough: the paths are closed loops from a random walk, so the
    # offset between the cubes sweeps the representative phases within two laps.
    ticks = int(min(max_ticks, math.ceil(max(laps) * 2.0) + 8))
    yield_ticks = max(1, int(rules['walk_yield']))
    lock_ticks = max(1, int(rules['walk_yield_lock']))
    mutual_after = int(rules['walk_mutual_after'])
    mutual_ticks = max(1, int(rules['walk_mutual_lock']))
    veto = {c: 0 for c in OBSTACLES}
    back = {c: 0 for c in OBSTACLES}
    advance = {c: 0.0 for c in OBSTACLES}
    mutual = {c: 0 for c in OBSTACLES}
    tally = {c: {} for c in OBSTACLES}
    blocker_tally = {c: {} for c in OBSTACLES}
    gap = float('inf')
    rounds = 0
    still = 0
    deadlock = False
    for tick in range(ticks):
        moved = False
        outcome = {}
        blocker = {}
        for rank, colour in enumerate(OBSTACLES):
            path, cum, total = geom[colour]
            st = state[colour]
            if st['hold'] > 0:
                st['hold'] -= 1
            step = st['step']
            if st['yield'] > 0:
                # Commitment period: retreat and nothing else this tick, until the count runs out.
                # Without blocking advance the cube would take the freed cell back at once and
                # would oscillate one tick forward and one back, which is no yielding at all.
                b = st['s'] - step
                bx, by = point_along(path, cum, total, b)[0]
                if cell.conflict(colour, bx, by, half) is None:
                    cell.commit(colour, bx, by, half)
                    st['s'] = (st['s'] - step) % total
                    st['x'], st['y'] = bx, by
                    st['yield'] -= 1
                    st['net'] -= step
                    back[colour] += 1
                    outcome[colour] = 'back'
                    moved = True
                    continue
                # The cell behind is taken as well: the commitment is cancelled, normal rules apply.
                st['yield'] = 0
            s = st['s'] + step
            nx, ny = point_along(path, cum, total, s)[0]
            who = cell.conflict(colour, nx, ny, half)
            blocker[colour] = who
            if who is None:
                cell.commit(colour, nx, ny, half)
                st['s'] = (st['s'] + step) % total
                st['x'], st['y'] = nx, ny
                st['wait'] = 0
                outcome[colour] = 'fwd'
                advance[colour] += abs(float(step))
                st['net'] += step
                moved = True
                continue
            veto[colour] += 1
            st['wait'] += 1
            if st['hold'] > 0 or (rank == 0 and st['yield'] <= 0):
                # A higher-priority cube normally only waits and never backs up, keeping at
                # least one cube able to move; it reverses only when the mutual-stall trigger
                # below pulls it into a commitment period.  `hold` is the "may not retreat"
                # gate the release puts on the peer: a one-sided release is what breaks the cycle.
                outcome[colour] = 'wait'
                continue
            if st['yield'] <= 0 and st['wait'] < yield_ticks:
                outcome[colour] = 'wait'
                continue
            b = st['s'] - step
            bx, by = point_along(path, cum, total, b)[0]
            if cell.conflict(colour, bx, by, half) is not None:
                # The cell behind is taken too: drop the commitment and wait again.
                st['yield'] = 0
                outcome[colour] = 'stuck'
                continue
            cell.commit(colour, bx, by, half)
            st['s'] = (st['s'] - step) % total
            st['x'], st['y'] = bx, by
            st['net'] -= step
            if st['yield'] > 0:
                st['yield'] -= 1
            else:
                st['yield'] = lock_ticks - 1
            back[colour] += 1
            outcome[colour] = 'back'
            moved = True
        a, b = state[OBSTACLES[0]], state[OBSTACLES[1]]
        gap = min(gap, math.hypot(a['x'] - b['x'], a['y'] - b['y']))
        for colour in OBSTACLES:
            result = outcome.get(colour, 'idle')
            tally[colour][result] = tally[colour].get(result, 0) + 1
            who = blocker.get(colour)
            if who is not None:
                blocker_tally[colour][who] = blocker_tally[colour].get(who, 0) + 1
            # One "ticks without advancing" counter per cube is the direct measure of being
            # blocked.  "Neither cube advanced" does not work: a cube that backs up every 40 ticks
            # keeps interrupting that window while its peer stays frozen.
            state[colour]['stall_run'] = (0 if result == 'fwd'
                                          else state[colour]['stall_run'] + 1)
        if observe is not None:
            observe(tick, state, outcome, blocker)
        if moved:
            still = 0
        else:
            still += 1
        if any(outcome.get(c) == 'fwd' for c in OBSTACLES):
            rounds = 0
        # A cube blocked for `walk_mutual_after` ticks in a row triggers the deadlock release:
        # it reverses and leaves the crossing along the other end of its loop, while the
        # peer gate is lengthened x1 / x2 / x3 / x4 per round.  Reversing alone does not work:
        # both cubes are wedged at the same crossing of their own loops and come straight back
        # once the commitment expires.  Flipping the direction instead takes this cube away
        # along the other end of its path and frees the peer side of the crossing.  The `dirs`
        # flip has to match `dynamic_obstacles.advance` tick for tick.
        #
        # Which cube flips matters: the cell it would take after flipping has to be free, or it
        # lands on another occupancy.  Candidates are ordered by how long they have been
        # blocked (`stall_run`), longest first, and the first one with a free cell is used;
        # when that cube cannot move, the peer flips instead, better than a standing deadlock.
        if mutual_after > 0:
            cands = sorted(OBSTACLES, key=lambda c: -state[c]['stall_run'])
            if state[cands[0]]['stall_run'] >= mutual_after:
                for colour in cands:
                    st = state[colour]
                    if st['hold'] > 0:
                        continue
                    # Only a genuinely blocked cube is a candidate, matching
                    # `dynamic_obstacles.advance` tick for tick: skipping cubes inside a yield or
                    # hold period would skip the one that needs releasing and flip a healthy peer.
                    if st['stall_run'] < mutual_after:
                        continue
                    path, cum, total = geom[colour]
                    bx, by = point_along(path, cum, total, st['s'] - st['step'])[0]
                    if cell.conflict(colour, bx, by, half) is not None:
                        continue
                    hold_ticks = mutual_ticks * min(rounds + 1, 4)
                    st['step'] = -st['step']      # Reversal: this is the next cell after the flip
                    st['yield'] = 0
                    st['stall_run'] = 0
                    mutual[colour] += 1
                    rounds += 1
                    # The peer gets the "may not retreat" gate: the release has to be one-sided,
                    # or both cubes turn around as soon as the parallel stretch frees up and jam
                    # again at the same offset as a symmetric limit cycle.  The gate is twice the
                    # release window so the peer cannot start moving mid-release.
                    for other in OBSTACLES:
                        if other != colour:
                            state[other]['wait'] = 0
                            state[other]['hold'] = hold_ticks * 2
                    break
        if still > 60:
            # Even backing up cannot move, the cell behind is taken: report a deadlock and stop.
            deadlock = True
            break
    return {
        'ticks': ticks,
        'veto': veto,
        'retreat': back,
        'stall': {c: round(veto[c] / float(max(1, ticks)), 4) for c in OBSTACLES},
        # `laps` uses net arc length, advance minus retreat, which is what counts as a lap:
        # forward ticks alone would score a cube that oscillates in place as covering ground.
        # `fwd_laps` keeps the forward-only count for diagnostics.
        'laps': {c: round(state[c]['net'] / max(geom[c][2], 1e-9), 3) for c in OBSTACLES},
        'fwd_laps': {c: round(advance[c] / max(geom[c][2], 1e-9), 3) for c in OBSTACLES},
        # `mutual`: how often the deadlock release fired in this phase, each time retreating
        # the blocked cube and gating the peer.  A high count means the paths run too close.
        'mutual': {c: mutual[c] for c in OBSTACLES},
        'outcome': {c: dict(tally[c]) for c in OBSTACLES},
        'blocked_by': {c: dict(blocker_tally[c]) for c in OBSTACLES},
        'gap': round(gap, 4),
        'deadlock': deadlock,
    }


def _one_scene(rng, rules):
    """One attempt: four random static landmarks plus two random-walk paths."""
    rules = _walk_rules(rules)
    landmarks = randomize_landmarks(rng, rules)
    static_pts = [tuple(landmarks[c][:2]) for c in COLORS]
    speeds = _random_speeds(rng, rules)
    built = _build_tracks(rng, rules, static_pts, speeds)
    if built is None:
        return None
    tracks, stats = built

    dynamic = {}
    for colour in OBSTACLES:
        walk = tracks[colour]
        poly = [tuple(p) for p in walk['path']]
        dynamic[colour] = {
            'speed': speeds[colour],
            'phase': 0.0,
            'home': [walk['home'][0], walk['home'][1]],
            'path': walk['path'],
            'perimeter': round(perimeter(poly), 4),
            'clearance_static': round(min_static_clearance(poly, static_pts), 4),
            'stall_ratio': round(walk['stall_ratio'], 4),
            'ticks': walk['ticks'],
        }
    scene = {
        'landmarks': landmarks,
        'dynamic': dynamic,
        'cross_clearance': round(stats['min_gap'], 4),
        # The robot's parking spot and the coverage figures go into the scene so they can be
        # reviewed later, as `world_setup.py --show` and manifest.json both use them.
        'robot': [round(float(v), 4) for v in _robot_pts(rules)[0]],
        'cover': {colour: round(stats['cover'][colour], 4) for colour in OBSTACLES},
        'cover_union': round(stats['cover_union'], 4),
        'walk': {
            'seconds': float(rules['walk_seconds']),
            'rate_hz': float(rules['sim_rate_hz']),
            'ticks': stats['ticks'],
        },
    }
    # Coverage and quadrant spread use the stored, thinned path, what the node replays.
    paths = {c: [(p[0], p[1]) for p in dynamic[c]['path']] for c in OBSTACLES}
    blockers = _blockers(rules, static_pts, _robot_pts(rules))
    scene['quad'] = _quad_stats(paths, rules, blockers)
    scene['replay'] = _worst_replay(scene, rules)
    return scene


def _worst_replay(scene, rules, max_ticks=40000):
    """Replay every representative phase in `PHASE_OFFSETS` and keep the worst result.

    The `replay_stats` return value is passed through unchanged
    (``ticks``/``veto``/``retreat``/``stall``/``gap``/``deadlock``) so `validate_scene`,
    `world_setup.py --show` and `--selftest` need no change; the multi-phase detail stays
    in ``phases`` for later inspection.
    """
    runs = [replay_stats(scene, rules, max_ticks=max_ticks, phase=p) for p in PHASE_OFFSETS]
    worst = dict(runs[0])
    worst['veto'] = {c: max(r['veto'][c] for r in runs) for c in OBSTACLES}
    worst['retreat'] = {c: max(r['retreat'][c] for r in runs) for c in OBSTACLES}
    worst['stall'] = {c: max(r['stall'][c] for r in runs) for c in OBSTACLES}
    worst['mutual'] = {c: max(r['mutual'][c] for r in runs) for c in OBSTACLES}
    worst['laps'] = {c: min(r['laps'][c] for r in runs) for c in OBSTACLES}
    worst['fwd_laps'] = {c: min(r['fwd_laps'][c] for r in runs) for c in OBSTACLES}
    worst['outcome'] = {
        c: {key: sum(r['outcome'][c].get(key, 0) for r in runs)
            for key in ('fwd', 'wait', 'back', 'stuck', 'idle')}
        for c in OBSTACLES}
    worst['blocked_by'] = {
        c: {owner: sum(r['blocked_by'][c].get(owner, 0) for r in runs)
            for owner in set().union(*[r['blocked_by'][c] for r in runs])}
        for c in OBSTACLES}
    worst['gap'] = min(r['gap'] for r in runs)
    worst['deadlock'] = any(r['deadlock'] for r in runs)
    worst['deadlock_phases'] = [p for p, r in zip(PHASE_OFFSETS, runs) if r['deadlock']]
    worst['phases'] = [{'phase': p, 'deadlock': r['deadlock'], 'gap': r['gap'],
                        'stall': r['stall'], 'laps': r['laps'], 'mutual': r['mutual'],
                        'fwd_laps': r['fwd_laps'], 'outcome': r['outcome']}
                       for p, r in zip(PHASE_OFFSETS, runs)]
    return worst


def validate_scene(scene, rules):
    """Return every violated world rule; an empty list means a legal world.

    Only four things are checked: static landmarks on the grid points of their own
    quadrant, legal initial values and distance for both paths, a joint simulation without
    out-of-bounds or contact or overlap or a long freeze, and the two distribution
    measures of roaming (coverage and quadrant spread).  The shape of the path itself is
    free to self-intersect and to hug walls, that is what a random walk looks like.
    """
    rules = _walk_rules(rules)
    problems = []
    landmarks = scene.get('landmarks') or {}
    dynamic = scene.get('dynamic') or {}
    lo, hi = rules['quadrant_lo'], rules['quadrant_hi']
    lim = rules['arena_half'] - rules['wall_margin']

    for colour in COLORS:
        if colour not in landmarks:
            problems.append(f'缺少地标 {colour}')
            continue
        x, y = landmarks[colour][0], landmarks[colour][1]
        sign_x, sign_y = QUADRANT[colour]
        if not (lo - 1e-6 <= sign_x * x <= hi + 1e-6):
            problems.append(f'{colour} 的 x={x:.3f} 不在象限 [{lo}, {hi}] 内')
        if not (lo - 1e-6 <= sign_y * y <= hi + 1e-6):
            problems.append(f'{colour} 的 y={y:.3f} 不在象限 [{lo}, {hi}] 内')
        if abs(x) > lim + 1e-6 or abs(y) > lim + 1e-6:
            problems.append(f'{colour} ({x:.3f}, {y:.3f}) 出了场地')
        grid = rules['grid_m']
        if abs(_snap(x, grid) - x) > 1e-6 or abs(_snap(y, grid) - y) > 1e-6:
            problems.append(f'{colour} ({x:.3f}, {y:.3f}) 没落在 {grid:g} m 网格上')

    pts = [tuple(landmarks[c][:2]) for c in COLORS if c in landmarks]
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            dist = math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1])
            if dist < 2.0 * rules['block_half'] - 1e-6:
                problems.append(f'静态块 {i}/{j} 间距 {dist:.3f} m 小于一个方块边长')

    half = _walk_half(rules)
    limit = _walk_limit(rules, half)
    for colour in OBSTACLES:
        item = dynamic.get(colour)
        if not item:
            problems.append(f'缺少动态块 {colour}')
            continue
        poly = [tuple(p) for p in item.get('path') or []]
        if len(poly) < 3:
            problems.append(f'{colour} 的走线少于 3 个顶点')
            continue
        for point in poly:
            if abs(point[0]) > limit + 1e-6 or abs(point[1]) > limit + 1e-6:
                problems.append(f'{colour} 的走线顶点 ({point[0]:.2f}, {point[1]:.2f}) 出了场地')
                break
        home = item.get('home') or [poly[0][0], poly[0][1]]
        if math.hypot(home[0] - poly[0][0], home[1] - poly[0][1]) > float(rules['walk_arrive']):
            problems.append(f'{colour} 的起始位姿和走线首点不重合（回放会瞬移）')
        cheb = _min_static_cheb(poly, pts, float(rules['cell_size']))
        keep = _walk_keep(rules)
        if cheb < keep - 1e-6:
            problems.append(
                f'{colour} 的走线离静态块最近只有 {cheb:.3f} m（切比雪夫），'
                f'不到 {keep:.3f} m，运行时会踩到静态块占的格子')
        # Robot parking spot: the path has to avoid it geometrically, because a cube has no
        # room to yield on the fly while the robot sits there, its loop being fixed at start-up.
        robots = _robot_pts(rules)
        robot_keep = _walk_robot_keep(rules)
        margin = _min_cheb(poly, [(p[0], p[1], robot_keep) for p in robots],
                           float(rules['cell_size']))
        if margin < -1e-6:
            problems.append(
                f'{colour} 的走线离机器人停驻点 ({robots[0][0]:.2f}, {robots[0][1]:.2f}) '
                f'最近只有 {margin + robot_keep:.3f} m（切比雪夫），不到 {robot_keep:.3f} m')
        speed = item.get('speed', float('nan'))
        if not math.isfinite(speed) or speed <= 0.0:
            problems.append(f'{colour} 的速度 {speed} 不是正数')
        # Note: `stall_ratio`, the share of refused advances while `_Walker` builds a path, is
        # not a criterion: it describes sampling noise during generation rather than the stored
        # path, and mixes passive waiting, a pending yield and a real freeze.  Only `laps` counts.

    gap = scene.get('cross_clearance')
    if isinstance(gap, (int, float)) and math.isfinite(gap):
        if gap < float(rules['sim_gap_min']) - 1e-9:
            problems.append(
                f'联合仿真里两块最近只到 {gap:.3f} m，小于 {float(rules["sim_gap_min"]):.2f} m')

    # Roaming criterion (1): coverage and quadrant spread.
    paths = {}
    for colour in OBSTACLES:
        item = dynamic.get(colour)
        if item and item.get('path'):
            paths[colour] = [(float(p[0]), float(p[1])) for p in item['path']]
    if len(paths) == len(OBSTACLES):
        # Distance travelled alone is misleading: the heading random walk decorrelates only
        # after hundreds of ticks, far longer than crossing the arena takes, so the path is a
        # narrow band that grazes the arena.  Coverage is the fallback criterion per attempt.
        blockers = _blockers(rules, pts, _robot_pts(rules))
        cover, union = _cover_stats(paths, rules, blockers)
        floor_each = float(rules['walk_cover_min'])
        floor_union = float(rules['walk_cover_union'])
        for colour in OBSTACLES:
            if cover[colour] < floor_each - 1e-9:
                problems.append(
                    f'{colour} 的走线只覆盖了 {cover[colour] * 100:.1f}% 的可用区域，'
                    f'低于 {floor_each * 100:.1f}%（走线塌成一条窄带，不是"到处游走"）')
        if union < floor_union - 1e-9:
            problems.append(
                f'两块合并只覆盖了 {union * 100:.1f}% 的可用区域，'
                f'低于 {floor_union * 100:.1f}%')

        # Roaming criterion (2): quadrant spread.  Coverage added up over both cubes hides a
        # split into half-arenas, where the totals look healthy while one cube never enters
        # the other half at all.  Each cube therefore has to leave a footprint in all four
        # quadrants.
        quad = _quad_stats(paths, rules, blockers)
        floor_quad = float(rules['walk_quad_min'])
        for colour in OBSTACLES:
            thin = [key for key in ('I', 'II', 'III', 'IV')
                    if quad[colour][key] < floor_quad - 1e-9]
            if thin:
                shown = ' '.join(f'{key}={quad[colour][key] * 100:.0f}%'
                                 for key in ('I', 'II', 'III', 'IV'))
                problems.append(
                    f'{colour} 的走线在象限 {"、".join(thin)} 里几乎没有足迹'
                    f'（{shown}，每个象限都要 ≥ {floor_quad * 100:.0f}%）'
                    f'—— 两块各占半个场地了，不是"到处游走"')

        # Keeping the two paths apart everywhere is not a criterion: over a random walk in an
        # 8 x 8 m arena both cubes sweep almost every reachable cell, so any such requirement
        # walls each cube into half of the arena and the quadrant figures drop to zero.  The
        # risk of two paths running side by side is covered by the phase replay below, whose
        # `laps` measure whether the cubes really move at run time.

    a = dynamic.get(OBSTACLES[0], {}).get('speed')
    b = dynamic.get(OBSTACLES[1], {}).get('speed')
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if abs(a - b) < rules['speed_gap'] - 1e-9:
            problems.append(f'两块速度差 {abs(a - b):.3f} m/s 小于 {rules["speed_gap"]:.2f}')

    replay = scene.get('replay')
    if not replay:
        problems.append('缺少"按运行时语义回放"的结果')
    elif replay.get('deadlock'):
        phases = replay.get('deadlock_phases') or []
        where = ('（相位 ' + '、'.join(f'{p:g}' for p in phases) + '）') if phases else ''
        problems.append(
            f'按运行时语义回放两条走线时两块互相锁死（都推不动）{where}'
            f'：说明某个时间差上两块会迎面顶住')
    else:
        # The main criterion is how many laps were really walked (`laps`), not the share of
        # refused advances (`stall`).  `stall` mixes passive waiting, a pending yield and a real
        # freeze, so a threshold on it misjudges often; a phase offset alone can push it high.
        # Motion is measured by arc length.
        floor_laps = float(rules['walk_laps_min'])
        for colour in OBSTACLES:
            laps = (replay.get('laps') or {}).get(colour)
            if isinstance(laps, (int, float)) and laps < floor_laps - 1e-9:
                problems.append(
                    f'运行时回放里 {colour} 只走了 {laps:.2f} 圈（下限 {floor_laps:.2f} 圈），'
                    f'其余拍都在等/退让 —— 说明某个时间差上它被顶住走不动')
    return problems


def generate_scene(seed=None, rules=None):
    """Generate a legal world.  ``seed`` None or negative picks one at random and records it."""
    rules = rules or make_rules()
    if seed is None or int(seed) < 0:
        seed = random.SystemRandom().randrange(1, 2 ** 31 - 1)
    seed = int(seed)
    first_problems = None
    for attempt in range(int(rules['max_attempts'])):
        rng = random.Random(seed + attempt)
        scene = _one_scene(rng, rules)
        if scene is None:
            continue
        scene['seed'] = seed + attempt
        scene['seed_arg'] = seed
        scene['attempt'] = attempt + 1
        problems = validate_scene(scene, rules)
        if not problems:
            return scene
        if first_problems is None:
            first_problems = problems
    raise RuntimeError(
        f'seed={seed} 连试 {rules["max_attempts"]} 次都造不出合法世界'
        + (f'；第一次的违规：{first_problems}' if first_problems else ''))


# --------------------------------------------------------------------- runtime

def path_geometry(path):
    """Closed polyline -> (cumulative arc-length table, total length)."""
    cum = []
    total = 0.0
    n = len(path)
    for i in range(n):
        a, b = path[i], path[(i + 1) % n]
        total += math.hypot(b[0] - a[0], b[1] - a[1])
        cum.append(total)
    return cum, total


def point_along(path, cum, total, s):
    """Point and unit tangent at arc length ``s``, taken modulo the total length."""
    if total <= 1e-12:
        return (float(path[0][0]), float(path[0][1])), (1.0, 0.0)
    s = s % total
    low = 0.0
    n = len(path)
    for i in range(n):
        high = cum[i]
        if s <= high or i == n - 1:
            a, b = path[i], path[(i + 1) % n]
            span = high - low
            t = 0.0 if span <= 1e-12 else min(1.0, max(0.0, (s - low) / span))
            dx, dy = b[0] - a[0], b[1] - a[1]
            norm = math.hypot(dx, dy)
            ux, uy = (1.0, 0.0) if norm <= 1e-12 else (dx / norm, dy / norm)
            return (a[0] + t * dx, a[1] + t * dy), (ux, uy)
        low = high
    a = path[-1]
    return (float(a[0]), float(a[1])), (1.0, 0.0)


def sample_loop(path, cum, total, s, speed, yaw_window, max_yaw_rate=2.0):
    """Closed patrol loop at arc length ``s`` -> (x, y, yaw, vx, vy, wz).

    The position comes from ``point_along(s)``, strictly on the path.  The heading is the
    chord spanning +-yaw_window/2, because a polyline tangent jumps at every vertex and
    would snap the cube around a corner; a chord gives a smooth heading curve that lags
    the direction of motion slightly, as a turning car does.

    Linear velocity follows the tangent rather than the chord, so ``|v| == speed`` and the
    per-tick displacement matches ``v*dt``.  Supplying a pose without a velocity lets the
    ODE drag the body off the path.  The z rate is a finite difference of the heading over
    arc length times the speed, clamped to ``max_yaw_rate``.  With ``speed <= 0`` the cube
    stays at ``s`` without turning.
    """
    def chord_heading(ss):
        half = 0.5 * max(0.0, yaw_window)
        (ax, ay), _ = point_along(path, cum, total, ss - half)
        (bx, by), _ = point_along(path, cum, total, ss + half)
        if math.hypot(bx - ax, by - ay) < 1e-9:
            _, (ux, uy) = point_along(path, cum, total, ss)
            return math.atan2(uy, ux)
        return math.atan2(by - ay, bx - ax)

    (x, y), (ux, uy) = point_along(path, cum, total, s)
    yaw = chord_heading(s)
    if speed <= 0.0 or total <= 1e-12:
        return x, y, yaw, 0.0, 0.0, 0.0
    step = 0.05
    wz = wrap(chord_heading(s + step) - yaw) * (speed / step)
    if max_yaw_rate and abs(wz) > abs(max_yaw_rate):
        wz = math.copysign(abs(max_yaw_rate), wz)
    return x, y, yaw, speed * ux, speed * uy, wz


class CellClaims:
    """Occupancy registry, rotation invariant circumscribed-circle criterion.

    The runtime half of the cube occupancy design.  `dynamic_obstacles` asks
    :meth:`conflict` every tick: it converts the cells the cube wants next into an
    occupancy of radius ``half`` and compares against the other owners, waiting in place
    when the centre distance is below the sum of the radii.  On success it calls
    :meth:`commit` and only then pushes the pose; `commit` also releases the cells the
    owner no longer holds, which is what lets another cube take them.

    ``half`` is always the circumscribed radius of the shape (0.28284 for a 0.4 m cube),
    so "circles do not intersect" implies "cubes do not intersect" and the single hard
    threshold ``2*half = 0.5657 m`` is rotation invariant, with no flicker between
    conflict and no conflict as the headings change.  The offline
    ``world_rules.claims_agree()`` keeps that property honest with a SAT test on real
    cubes.

    ``mark`` is a debug-only occupancy map drawing ``half``-sized cells per owner; it takes
    no part in any verdict.
    """

    def __init__(self, cell=0.10):
        self.cell = float(cell) if cell and cell > 0.0 else 0.10
        self.owner = {}
        self.mark = {}

    def cells_of(self, x, y, half):
        """Debug map: cells covered by a ``half``-sized square centred on (x, y)."""
        cell = self.cell
        i0 = int(math.floor((x - half) / cell))
        i1 = int(math.floor((x + half) / cell))
        j0 = int(math.floor((y - half) / cell))
        j1 = int(math.floor((y + half) / cell))
        return {(i, j) for i in range(i0, i1 + 1) for j in range(j0, j1 + 1)}

    def conflict(self, owner, x, y, half):
        """Whether a circle of radius ``half`` at (x, y) intersects another occupancy.

        Returns the name of the first owner hit, or None.  Cells an owner already holds
        are not a conflict, because standing still is always legal.
        """
        for name, (ox, oy, oh) in self.owner.items():
            if name == owner:
                continue
            if math.hypot(x - ox, y - oy) < half + oh:
                return name
        return None

    def commit(self, owner, x, y, half):
        self.owner[owner] = (float(x), float(y), float(half))
        for cell, holder in list(self.mark.items()):
            if holder == owner:
                del self.mark[cell]
        for cell in self.cells_of(x, y, half):
            self.mark[cell] = owner

    def release(self, owner):
        self.owner.pop(owner, None)
        for cell, holder in list(self.mark.items()):
            if holder == owner:
                del self.mark[cell]

    def reset(self):
        self.owner.clear()
        self.mark.clear()


# -------------------------------------------------------------------- artefacts

def scene_to_world_poses(scene, rules, z=0.2):
    """Scene -> ``{model name in the world: '<pose>' string}``.

    Landmarks get yaw=0 (a 0.4 m cube centred 0.2 m above the ground, axis aligned).  A
    cube's yaw is the chord at arc length 0, so the home pose written into the world
    matches the first ``push()`` of the node and the cube does not jump when
    ``set_enabled`` fires.
    """
    poses = {}
    for colour in COLORS:
        item = scene['landmarks'][colour]
        poses['landmark_' + colour] = f'{item[0]:.3f} {item[1]:.3f} {z:.3f} 0 0 0'
    yaw_window = rules['yaw_window']
    for colour in OBSTACLES:
        item = scene['dynamic'][colour]
        path = [tuple(p) for p in item['path']]
        cum, total = path_geometry(path)
        x, y, yaw, _vx, _vy, _wz = sample_loop(
            path, cum, total, item.get('phase', 0.0) * total,
            item['speed'], yaw_window)
        poses[MODEL_PREFIX + colour] = f'{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}'
    return poses


def _yaml_floats(values):
    return '[' + ', '.join(f'{float(v):.4f}' for v in values) + ']'


def scene_to_params_overlay(scene, header=None, rules=None):
    """Scene -> contents of the ``dynamic_obstacles`` parameter overlay (YAML text).

    The file is passed as the second ``--params-file``, after ``config/params.yaml``, so it
    overrides only the trajectory and speed of that node and leaves the other sections
    untouched.
    """
    block_half = float((rules or DEFAULT_RULES)['block_half'])
    lines = [
        '# 由 sim_bringup/scripts/world_setup.py 生成，不要手改。',
    ]
    if header:
        lines.append('# ' + str(header).replace('\n', ' '))
    lines += [
        '# 世界地图规则的一次随机化结果：两条巡逻路径、两块速度、两块相位。',
        'dynamic_obstacles:',
        '  ros__parameters:',
        "    colors: ['%s']" % "', '".join(OBSTACLES),
    ]
    for colour in OBSTACLES:
        item = scene['dynamic'][colour]
        flat = []
        for point in item['path']:
            flat.extend(point)
        lines.append(f'    {colour}_path: {_yaml_floats(flat)}')
        lines.append(f'    {colour}_speed: {item["speed"]:.4f}')
        lines.append(f'    {colour}_phase: {float(item.get("phase", 0.0)):.4f}')
    # Static landmarks are permanent occupancy too, as they never release a cell.  This is
    # world geometry supplied by the generator so that `dynamic_obstacles` does not have
    # to guess it; the node must not read scene.json, which holds the scoring truth.
    static = []
    for colour in COLORS:
        static.extend(scene['landmarks'][colour][:2])
    lines.append(f'    static_box_half: {block_half:.4f}')
    lines.append(f'    static_boxes: {_yaml_floats(static)}')
    return '\n'.join(lines) + '\n'


def write_world(template_text, poses, out_path):
    """Replace the ``<pose>`` of the six models in the template world, keeping the rest.

    Only ``<pose>`` is touched: the cameras, plugins, physics and the many comments of the
    world file are assets, and re-serialising the tree would drop them.  The first
    ``<pose>`` after ``<model name="X">`` is used, after checking that no further
    ``<model`` appears in between, so the replacement cannot run past the intended model.
    """
    text = template_text
    for name, pose in poses.items():
        marker = f'<model name="{name}">'
        start = text.find(marker)
        if start < 0:
            raise KeyError(f'模板 world 里找不到 {marker}')
        nxt = text.find('<model name=', start + len(marker))
        limit = len(text) if nxt < 0 else nxt
        open_at = text.find('<pose>', start, limit)
        if open_at < 0:
            raise KeyError(f'{name} 在模板 world 里没有 <pose>')
        close_at = text.find('</pose>', open_at, limit)
        if close_at < 0:
            raise KeyError(f'{name} 的 <pose> 没有闭合')
        text = text[:open_at + len('<pose>')] + pose + text[close_at:]
    # newline='' keeps the platform from translating line endings, so the LF template is
    # written back byte for byte apart from the six <pose> lines and diffs stay readable.
    with open(out_path, 'w', encoding='utf-8', newline='') as handle:
        handle.write(text)
    return out_path


def load_scene(path):
    with open(path, 'r', encoding='utf-8') as handle:
        return json.load(handle)


def dumps_scene(scene):
    return json.dumps(scene, indent=2, ensure_ascii=False, sort_keys=True)


def default_scene_dir():
    return os.path.join(os.path.expanduser('~'), '.ros', 'sim_bringup_scene')
