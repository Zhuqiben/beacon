#!/usr/bin/env python3
"""Semantic landmark map to OccupancyGrid (latched QoS).

Only the static landmarks and their inflation are drawn here. Dynamic obstacles reach
the Nav2 costmaps through /scan instead, so a moving obstacle never reloads the
static_layer or resets the costmap.

Frame of reference: the arena is [-4, 4] x [-4, 4], size 8.0 with resolution 0.05,
giving a 160x160 grid with origin at -4. Landmarks sit at (+-2, +-2), i.e. cells
(40,40), (120,40), (40,120) and (120,120).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from slam_interfaces.msg import LandmarkMap


class MapNode(Node):
    def __init__(self):
        super().__init__('landmark_map')
        # use_sim_time normally arrives from the parameter file; declaring it only when
        # it is still absent avoids a duplicate-declaration error.
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('size', 8.0)              # matches the arena size in the world
        self.declare_parameter('inflation_base', 0.30)   # circumscribed circle of the 0.4 m cube plus margin
        self.declare_parameter('sigma_k', 0.0)           # 0 = constant radius, otherwise sigma scales the radius
        # Upper bound on the inflation radius. Without it an outlying covariance grows the
        # radius to hundreds of metres and stamp_circle's O(r^2) loop stalls the node.
        self.declare_parameter('max_inflation', 1.0)
        self.res = float(self.get_parameter('resolution').value)
        self.size = float(self.get_parameter('size').value)
        self.inflation_base = float(self.get_parameter('inflation_base').value)
        self.sigma_k = float(self.get_parameter('sigma_k').value)
        self.max_inflation = float(self.get_parameter('max_inflation').value)

        self.n = int(round(self.size / self.res))        # 8.0 / 0.05 = 160
        self.origin = -self.size / 2.0
        self.grid = np.zeros((self.n, self.n), dtype=np.int8)   # 0 = free
        self.bad_landmarks = set()                       # colours already warned about

        latched = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.create_subscription(LandmarkMap, '/landmark_map', self.on_map, 10)
        self.create_timer(1.0, self.publish)             # 1 Hz republish for late subscribers

        self.get_logger().info(
            f'landmark_map: {self.n}x{self.n} @ {self.res} m, origin {self.origin:.1f}, '
            f'max_inflation {self.max_inflation:.2f} m')

    def world_to_cell(self, x, y):
        return int((x - self.origin) / self.res), int((y - self.origin) / self.res)

    def stamp_circle(self, cx, cy, radius):
        r = int(math.ceil(radius / self.res))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue
                gx, gy = cx + dx, cy + dy
                if 0 <= gx < self.n and 0 <= gy < self.n:
                    self.grid[gy, gx] = 100              # OccupancyGrid is row-major: rows are y

    def on_map(self, msg):
        self.grid[:] = 0
        n_valid = 0
        # The colour, position, sigma and validity arrays are parallel; index them, never resort.
        for c, x, y, sigma, valid in zip(msg.colors, msg.x, msg.y, msg.sigma, msg.valid):
            if not valid:
                continue
            x, y, sigma = float(x), float(y), float(sigma)
            # A single non-finite input must not kill the node: min(NaN, max_inflation) stays
            # NaN, ceil(NaN) raises, and the process exits - which freezes /map for the rest of
            # the run, leaving the planner reporting "no valid path found".
            if not (math.isfinite(x) and math.isfinite(y)):
                self._warn_bad(c, f'位置非有限值 x={x!r} y={y!r}')
                continue
            if not math.isfinite(sigma):
                self._warn_bad(c, f'sigma 非有限值 {sigma!r}（按 0 膨胀处理）')
                sigma = 0.0
            sigma = max(0.0, sigma)
            n_valid += 1
            cx, cy = self.world_to_cell(x, y)
            # Larger uncertainty inflates further. Sigma_k is 0 in the shipped parameters, so
            # the radius degenerates to the constant inflation_base; sigma is still published
            # unchanged on /landmark_map for the marker visualiser.
            radius = self.inflation_base + self.sigma_k * sigma
            radius = min(radius, self.max_inflation)
            self.stamp_circle(cx, cy, radius)
            self.get_logger().debug(f'{c} @ ({x:.2f},{y:.2f}) inflate={radius:.2f} m')
        if n_valid == 0:
            self.get_logger().debug('/landmark_map 目前没有有效地标，/map 保持空白')

    def _warn_bad(self, color, why):
        # Warn once per colour: bad input can repeat every frame and flood the launch log.
        if color in self.bad_landmarks:
            return
        self.bad_landmarks.add(color)
        self.get_logger().warn(f'地标 {color} 的输入无效：{why}；该地标本帧不画')

    def publish(self):
        g = OccupancyGrid()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'map'
        g.info.resolution = self.res
        g.info.width = self.n
        g.info.height = self.n
        g.info.origin.position.x = self.origin
        g.info.origin.position.y = self.origin
        g.info.origin.orientation.w = 1.0
        g.data = self.grid.reshape(-1).tolist()
        self.pub.publish(g)


def main(args=None):
    rclpy.init(args=args)
    node = MapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
