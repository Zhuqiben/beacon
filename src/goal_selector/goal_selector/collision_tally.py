#!/usr/bin/env python3
"""Robot-to-block collision counting, one ledger per GoToBlock goal.

The source is the Gazebo contact sensor, not geometric inference: ``tb3_bumper`` on
``base_link`` (``libgazebo_ros_bumper.so``) publishes ``gazebo_msgs/msg/ContactsState`` on
``/bumper_states``, and every entry is a real ODE contact pair, so what is counted is
physical contact rather than "distance below some threshold".

One collision body on ``base_link`` is enough: the chassis box (0.265 m) is the largest xy
envelope of the robot, the lidar and camera lie strictly inside it, and the wheels exceed
it by about 11.5 mm, far below one grid cell.

One collision is one contact rising edge at least ``rearm_sec`` after the last release,
because ODE chatters between contact and separation in the microsecond range and a single
impact would otherwise be counted dozens of times. Only robot against block counts, static
and dynamic alike; walls, floor and self-contact do not. The ``touching`` set survives
across goals, so a block still pressed at the end of one goal is not mistaken for a new
collision in the next one.

Pure logic: no rclpy, so pytest runs it directly.
"""
import time

# Every colour block in the scene, matching the <model name> entries in
# worlds/landmark_world_dynamic.world.
BLOCK_MODELS = ('landmark_red', 'landmark_green', 'landmark_blue', 'landmark_yellow',
                'obstacle_white', 'obstacle_black')

# The first four are static landmarks, the last two are the roaming dynamic cubes.
STATIC_MODELS = frozenset(BLOCK_MODELS[:4])

# Chinese labels for log and result text.
BLOCK_LABELS = {
    'landmark_red': '红块',
    'landmark_green': '绿块',
    'landmark_blue': '蓝块',
    'landmark_yellow': '黄块',
    'obstacle_white': '白块(动态)',
    'obstacle_black': '黑块(动态)',
}

# Window after a release within which another contact counts as the same collision. 0.35 s
# sits between the two relevant scales: ODE chatter is in the microsecond range, while
# "hit, back off, hit again" is in the second range.
DEFAULT_REARM_SEC = 0.35


def blocks_in_contact(collision_names):
    """Map scoped collision names from a contact pair to the blocks that were touched.

    Gazebo reports names such as ``landmark_red::link::c``; the first field is the model
    name. Anything outside :data:`BLOCK_MODELS` is discarded (robot, floor, walls).
    """
    touched = set()
    for name in collision_names:
        if not name:
            continue
        head = name.split('::', 1)[0]
        if head in BLOCK_MODELS:
            touched.add(head)
    return touched


class CollisionTally:
    """Per-goal collision counter; pure logic, no rclpy dependency."""

    def __init__(self, models=BLOCK_MODELS, rearm_sec=DEFAULT_REARM_SEC):
        self.models = tuple(models)
        self.rearm_sec = float(rearm_sec)
        # Blocks currently in contact; survives across goals (see the module docstring).
        self.touching = set()
        self._released_at = {}
        self._counts = {m: 0 for m in self.models}
        self.active = False
        self.target = ''
        self.trip = 0
        self.started_at = None
        self.stopped_at = None

    # ---------------- per-frame contact state ----------------
    def observe(self, touched, now=None):
        """Feed one frame of contact state, with or without a running goal, otherwise
        ``reset`` cannot establish its baseline.

        ``touched`` is the set of blocks in contact this frame, ``now`` a monotonic
        timestamp in seconds. Returns the number of new collisions.
        """
        if now is None:
            now = time.monotonic()
        newly = set(touched) - self.touching
        gained = 0
        for model in newly:
            if not self.active:
                continue                        # not counting, only track contact state
            last = self._released_at.get(model)
            if last is None or now - last >= self.rearm_sec:
                self._counts[model] += 1
                gained += 1
        for model in self.touching - set(touched):
            self._released_at[model] = now
        self.touching = set(touched)
        return gained

    # ---------------- goal lifecycle ----------------
    def reset(self, target_color, now=None):
        """Start a new goal: clear the ledger and begin counting. Returns the trip number.

        ``touching`` and ``_released_at`` are left alone: a contact inherited from the
        previous goal is not a collision of this one.
        """
        if now is None:
            now = time.monotonic()
        self.trip += 1
        self.target = target_color or ''
        self._counts = {m: 0 for m in self.models}
        self.active = True
        self.started_at = now
        self.stopped_at = None
        return self.trip

    def stop(self, now=None):
        """Close the ledger, whether the goal succeeded or failed. Returns summary().

        Idempotent: a second call does not move the stop time.
        """
        if now is None:
            now = time.monotonic()
        if self.active:
            self.active = False
            self.stopped_at = now
        return self.summary()

    # ---------------- readings ----------------
    @property
    def total(self):
        return sum(self._counts.values())

    def hits(self):
        return {m: n for m, n in self._counts.items() if n}

    def summary(self, now=None):
        """Snapshot of the ledger. ``now`` only affects ``elapsed_sec``.
        """
        hits = self.hits()
        total = sum(hits.values())
        static = sum(n for m, n in hits.items() if m in STATIC_MODELS)
        if self.stopped_at is not None:
            end = self.stopped_at
        elif now is not None:
            end = now
        else:
            end = self.started_at
        elapsed = 0.0 if (self.started_at is None or end is None) else end - self.started_at
        return {
            'trip': self.trip,
            'target_color': self.target,
            'collisions': total,
            'static_collisions': static,
            'dynamic_collisions': total - static,
            'by_block': hits,
            'elapsed_sec': round(elapsed, 3),
        }

    def describe(self):
        """One-line Chinese summary for res.message and logs, without a prefix."""
        return describe_summary(self.summary())


def describe_summary(summary):
    """Render the dict from :meth:`CollisionTally.summary` as a Chinese summary."""
    total = int(summary.get('collisions', 0))
    if total == 0:
        return '未与色块碰撞'
    hits = summary.get('by_block') or {}
    static = int(summary.get('static_collisions', 0))
    parts = []
    if static:
        parts.append(f'静态 {static}')
    if total - static:
        parts.append(f'动态 {total - static}')
    detail = '、'.join(f'{BLOCK_LABELS.get(m, m)}×{n}' for m, n in sorted(hits.items()))
    return f'与色块碰撞 {total} 次（{"，".join(parts)}：{detail}）'
