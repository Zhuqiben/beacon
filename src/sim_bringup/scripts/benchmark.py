#!/usr/bin/env python3
"""Quantitative benchmark for the BEACON stack.

Metrics
-------
localisation RMS  : ||EKF pose - Gazebo ground truth||      (/robot_pose vs /gazebo/model_states)
map error         : ||estimated landmark - true landmark||  (/landmark_map vs scene.json)
observation rate  : landmarks seen per second               (/observations)
module latency    : per-stage compute time                  (/performance_monitor)

Gazebo ground truth is used for scoring only. It must never be fed into the control loop;
the whole point of the project is that the robot localises itself from camera observations
alone.

The true landmark poses come from the scene.json written by the launcher, not from a
constant table: the four landmarks are placed randomly inside their own quadrants once per
launch (see scripts/world_rules.py). Pass --scene to point at a different file; without one
the built-in constants are used and the report says so, because a stale truth table makes
every map-error number meaningless.

Usage
-----
    ros2 run sim_bringup benchmark.py                 # default model 'waffle'
    python3 scripts/benchmark.py --model waffle --duration 60 --csv perf.csv
"""

import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray

from slam_interfaces.msg import LandmarkArray, LandmarkMap


# Fallback truth for the four landmarks: on the DIAGONALS of the 8x8 m arena, not on the
# axis lines (lambda = 2 m, 0.4 m cubes centred at z = 0.2).
#
# These coordinates are a fallback only. Every launch randomises the landmark positions
# (only at start-up, see scripts/world_rules.py), so scoring has to read this run's
# scene.json; otherwise the localisation error silently absorbs the fact that the landmarks
# were moved and the numbers mean nothing. The constants are used only when no scene.json can
# be found (a standalone benchmark run, or a run with the randomisation switched off).
LANDMARK_TRUTH = {
    'red': (-2.0, -2.0),
    'green': (2.0, -2.0),
    'blue': (-2.0, 2.0),
    'yellow': (2.0, 2.0),
}


def load_landmark_truth(scene_path=None):
    """Read the true landmark coordinates of this run from ``scene.json`` and fall back to
    ``LANDMARK_TRUTH`` if that is not possible.

    Returns ``(truth, source)``, where ``source`` is a human-readable description that ends
    up in the report: the one thing a scoring script must never hide is that it scored
    against a stale truth.
    """
    if scene_path is None:
        scene_path = os.path.join(
            os.path.expanduser('~'), '.ros', 'sim_bringup_scene', 'scene.json')
    if not os.path.exists(scene_path):
        return dict(LANDMARK_TRUTH), f'内置常量（找不到 {scene_path}）'
    try:
        with open(scene_path, 'r', encoding='utf-8') as handle:
            scene = json.load(handle)
        truth = {colour: (float(pos[0]), float(pos[1]))
                 for colour, pos in scene['landmarks'].items()}
    except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
        return dict(LANDMARK_TRUTH), f'内置常量（{scene_path} 解析失败：{exc}）'
    missing = [c for c in LANDMARK_TRUTH if c not in truth]
    if missing:
        return dict(LANDMARK_TRUTH), f'内置常量（{scene_path} 缺 {missing}）'
    return truth, f'{scene_path}（seed={scene.get("seed")}）'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Bench(Node):
    def __init__(self, model_name='waffle', csv_path=None, scene_path=None):
        super().__init__('benchmark')
        self.model_name = model_name
        self.csv_path = csv_path
        # The truth source follows the world of this run; see load_landmark_truth.
        self.truth, self.truth_source = load_landmark_truth(scene_path)

        self.gt = None
        self.est = None
        self.errs = []            # planar localisation error samples, metres
        self.yaw_errs = []        # heading error samples, radians
        self.t0 = None
        self.obs_count = 0
        self.matched = {}         # colour -> (dx, dy) of the latest accepted update
        self.latency = {}         # module -> latest reported ms

        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(ModelStates, '/gazebo/model_states', self.on_gt, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/robot_pose', self.on_est, 10)
        self.create_subscription(LandmarkArray, '/observations', self.on_obs, 10)
        self.create_subscription(LandmarkMap, '/landmark_map', self.on_map, latched)
        self.create_subscription(Float32MultiArray, '/performance_monitor', self.on_perf, 10)
        self.create_timer(0.1, self.record)

    # ------------------------------------------------------------------ inputs

    def on_gt(self, msg):
        if self.model_name not in msg.name:
            return
        i = msg.name.index(self.model_name)
        p = msg.pose[i].position
        self.gt = (p.x, p.y, yaw_of(msg.pose[i].orientation))

    def on_est(self, msg):
        p = msg.pose.pose.position
        self.est = (p.x, p.y, yaw_of(msg.pose.pose.orientation))

    def on_obs(self, msg):
        self.obs_count += len(msg.landmarks)
        if self.t0 is None:
            self.t0 = time.monotonic()

    def on_map(self, msg):
        """Compare each initialised landmark estimate against the true pose of this run."""
        for colour, x, y, valid in zip(msg.colors, msg.x, msg.y, msg.valid):
            if not valid or colour not in self.truth:
                continue
            tx, ty = self.truth[colour]
            self.matched[colour] = (x - tx, y - ty)

    def on_perf(self, msg):
        """Layout: [stage_id, milliseconds, ...] as published by performance_monitor."""
        data = list(msg.data)
        for i in range(0, len(data) - 1, 2):
            self.latency[int(data[i])] = data[i + 1]

    # ----------------------------------------------------------------- scoring

    def record(self):
        if self.gt is None or self.est is None:
            return
        dx, dy = self.gt[0] - self.est[0], self.gt[1] - self.est[1]
        self.errs.append(math.hypot(dx, dy))
        self.yaw_errs.append(abs(wrap_pi(self.gt[2] - self.est[2])))

    def _samples(self):
        n = len(self.errs)
        if not n:
            self.get_logger().warn(
                'No samples collected - check /robot_pose and /gazebo/model_states')
            return None
        return np.asarray(self.errs)

    def report(self):
        a = self._samples()
        lines = []
        lines.append(f'truth         {self.truth_source}')
        if a is not None:
            lines.append(
                f'localisation  RMS={np.sqrt((a ** 2).mean()):.3f} m  '
                f'max={a.max():.3f} m  mean={a.mean():.3f} m  samples={a.size}')
            y = np.asarray(self.yaw_errs)
            lines.append(f'heading       RMS={np.sqrt((y ** 2).mean()):.3f} rad  max={y.max():.3f} rad')

        if self.matched:
            d = np.asarray([math.hypot(dx, dy) for dx, dy in self.matched.values()])
            detail = ', '.join(
                f'{c}={math.hypot(dx, dy):.3f}' for c, (dx, dy) in sorted(self.matched.items()))
            lines.append(f'map error     RMS={np.sqrt((d ** 2).mean()):.3f} m  '
                         f'max={d.max():.3f} m  n={d.size}  [{detail}]')
        else:
            lines.append('map error     NOT MEASURED (no valid entries on /landmark_map)')

        if self.t0 is not None:
            rate = self.obs_count / max(1e-9, time.monotonic() - self.t0)
            lines.append(f'observations  {self.obs_count} landmarks total, {rate:.2f} /s')
        else:
            lines.append('observations  NONE (/observations silent - lifecycle not activated?)')

        if self.latency:
            for stage, ms in sorted(self.latency.items()):
                lines.append(f'latency       stage {stage}: {ms:.2f} ms')
        else:
            lines.append('latency       NOT MEASURED (/performance_monitor silent)')

        if len(self.errs) >= 2:
            a = np.asarray(self.errs)
            lines.append(f'serial corr   lag-1={np.corrcoef(a[:-1], a[1:])[0, 1]:.3f}')

        for line in lines:
            self.get_logger().info(line)
        self._dump(lines)

    def _dump(self, lines):
        text = '\n'.join(lines)
        try:
            with open('performance_raw.txt', 'a', encoding='utf-8') as f:
                f.write(f'# {time.strftime("%Y-%m-%d %H:%M:%S")} model={self.model_name}\n')
                f.write(text + '\n')
        except OSError as exc:
            self.get_logger().warn(f'Cannot append performance_raw.txt: {exc}')

        if not self.csv_path:
            return
        try:
            with open(self.csv_path, 'a', newline='', encoding='utf-8') as f:
                w = csv.writer(f)
                if f.tell() == 0:
                    w.writerow(['metric', 'value'])
                for line in lines:
                    head, _, rest = line.partition(' ')
                    w.writerow([head, rest.strip()])
        except OSError as exc:
            self.get_logger().warn(f'Cannot write {self.csv_path}: {exc}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='waffle',
                        help="Gazebo model name; MUST match the spawned robot (default: waffle)")
    parser.add_argument('--duration', type=float, default=0.0,
                        help='seconds to sample; 0 = run until Ctrl-C')
    parser.add_argument('--csv', default=None, help='optional CSV output path')
    parser.add_argument('--scene', default=None,
                        help='scene.json written by the launcher (default: '
                             '~/.ros/sim_bringup_scene/scene.json); the landmark truth '
                             'moves with the randomisation, so keep this in sync')
    args, _ = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = Bench(args.model, args.csv, args.scene)
    node.get_logger().info(
        f'benchmark running, model={args.model}, duration={args.duration or "unbounded"}')
    node.get_logger().info(f'landmark truth: {node.truth_source}')
    try:
        if args.duration > 0.0:
            deadline = time.monotonic() + args.duration
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
