"""Obstacle monitor — consumes /scan and publishes compact min-distance-
per-octant plus a boolean danger flag on /obstacle_near. Keeps the state
machine agnostic of LiDAR details; it just needs 'is it safe to go this way'.
"""
from __future__ import annotations

import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray

from artpark_msgs.msg import Thought


OCTANTS = 8  # N, NE, E, SE, S, SW, W, NW


class ObstacleMonitor(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_monitor')

        self.declare_parameter('danger_radius_m', 0.35)
        self.declare_parameter('forward_arc_deg', 40.0)
        self.danger = float(self.get_parameter('danger_radius_m').value)
        self.fwd_arc = math.radians(float(self.get_parameter('forward_arc_deg').value) / 2.0)

        qos_scan = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_out = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.sub = self.create_subscription(LaserScan, '/scan', self._on_scan, qos_scan)
        self.pub_near = self.create_publisher(Bool, '/obstacle_near', qos_out)
        self.pub_oct  = self.create_publisher(Float32MultiArray, '/obstacle_octants', qos_out)
        self.pub_thought = self.create_publisher(Thought, '/thought', qos_out)
        self._seq = 0
        self.get_logger().info('obstacle_monitor up')

    # ------------------------------------------------------------
    def _on_scan(self, msg: LaserScan) -> None:
        if not msg.ranges:
            return

        n = len(msg.ranges)
        octants: List[float] = [float('inf')] * OCTANTS

        # Center octant index 0 on forward direction (angle 0 in LaserScan frame
        # normally points along +x of lidar_link, which is robot-forward).
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min or r >= msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            # Normalise to [-pi, pi]
            a = math.atan2(math.sin(angle), math.cos(angle))
            # Map angle to octant 0..7 where 0 = forward (±22.5°)
            idx = int(((a + math.pi + math.pi / OCTANTS) % (2 * math.pi)) * OCTANTS / (2 * math.pi))
            idx %= OCTANTS
            if r < octants[idx]:
                octants[idx] = r

        # Forward danger: any return within self.danger within ±fwd_arc
        fwd_min = float('inf')
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            a = math.atan2(math.sin(angle), math.cos(angle))
            if abs(a) <= self.fwd_arc and r < fwd_min:
                fwd_min = r

        danger_flag = fwd_min < self.danger

        self.pub_near.publish(Bool(data=danger_flag))
        out = Float32MultiArray()
        out.data = [float(v) if math.isfinite(v) else -1.0 for v in octants]
        self.pub_oct.publish(out)

        self._seq += 1
        if self._seq % 15 == 0:  # throttled thought-log
            t = Thought()
            t.stamp = self.get_clock().now().to_msg()
            t.seq = self._seq
            t.phase = 'OBSTACLE'
            t.hypothesis = f'fwd_min={fwd_min:.2f} m, danger={danger_flag}'
            t.action_chosen = 'publish_obstacle_state'
            t.rule_applied = 'obstacle_monitor.danger_arc'
            t.alt_considered = ''
            t.extras_json = f'{{"fwd_min":{fwd_min:.3f}, "octants":{out.data}}}'
            t.confidence = 1.0
            self.pub_thought.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
