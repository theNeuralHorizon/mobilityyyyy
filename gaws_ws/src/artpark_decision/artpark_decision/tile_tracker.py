"""Tile tracker — combines /odom with LiDAR-based wall centering to produce
a discrete (row, col) tile index and entry-edge memory. Publishes a
TileEvent each time the robot crosses a tile boundary.

Grid reference (from SDF, unchanged from judge's world):
  - tile pitch:  0.9 m in both x and y
  - row 0 center y=+1.8; row 4 center y=-1.8
  - col 0 center x=-1.35; col 3 center x=+1.35
"""
from __future__ import annotations

import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from artpark_msgs.msg import TileEvent, Thought


TILE_PITCH = 0.9
COL0_X = -1.35
ROW0_Y = 1.80


def world_to_tile(x: float, y: float) -> tuple[int, int]:
    col = int(round((x - COL0_X) / TILE_PITCH))
    row = int(round((ROW0_Y - y) / TILE_PITCH))
    return max(0, min(4, row)), max(0, min(3, col))


def edge_crossed(prev: tuple[int, int], cur: tuple[int, int]) -> str:
    dr, dc = cur[0] - prev[0], cur[1] - prev[1]
    if dr == -1 and dc == 0:
        return 'N'   # row number decreased → moved north
    if dr == 1 and dc == 0:
        return 'S'
    if dr == 0 and dc == 1:
        return 'E'
    if dr == 0 and dc == -1:
        return 'W'
    return ''


class TileTracker(Node):
    def __init__(self) -> None:
        super().__init__('tile_tracker')

        # Odom starts at (0, 0) relative to spawn; translate to world via the
        # known spawn pose so tile indexing is correct.
        self.declare_parameter('spawn_world_x', -1.35)
        self.declare_parameter('spawn_world_y',  1.80)
        self.spawn_wx = float(self.get_parameter('spawn_world_x').value)
        self.spawn_wy = float(self.get_parameter('spawn_world_y').value)

        self._prev_tile: Optional[tuple[int, int]] = None
        self._last_yaw: float = 0.0
        self._seq = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_odom = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.sub_odom = self.create_subscription(Odometry, '/odom', self._on_odom, qos_odom)
        self.pub_tile = self.create_publisher(TileEvent, '/tile_event', qos)
        self.pub_thought = self.create_publisher(Thought, '/thought', qos)
        self.get_logger().info(
            f'tile_tracker up | odom origin -> world ({self.spawn_wx}, {self.spawn_wy})')

    def _on_odom(self, msg: Odometry) -> None:
        # Translate odom-frame pose into world-frame pose.
        world_x = msg.pose.pose.position.x + self.spawn_wx
        world_y = msg.pose.pose.position.y + self.spawn_wy
        q = msg.pose.pose.orientation
        # yaw from quaternion
        self._last_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        tile = world_to_tile(world_x, world_y)
        if self._prev_tile is None:
            self._prev_tile = tile
            return
        if tile == self._prev_tile:
            return

        entry_edge = edge_crossed(self._prev_tile, tile)
        self._prev_tile = tile

        ev = TileEvent()
        ev.stamp = self.get_clock().now().to_msg()
        ev.tile_row, ev.tile_col = tile
        ev.entry_edge = entry_edge
        ev.dominant_color = ''   # filled by logger by joining with /edge_sample
        ev.in_green_zone = False  # resolved downstream
        ev.in_orange_zone = False
        self.pub_tile.publish(ev)

        self._seq += 1
        t = Thought()
        t.stamp = ev.stamp
        t.seq = self._seq
        t.phase = 'TILE'
        t.hypothesis = f'entered tile ({tile[0]},{tile[1]}) via {entry_edge}'
        t.action_chosen = 'publish_tile_event'
        t.rule_applied = 'tile_tracker.odom_grid_discretize'
        t.alt_considered = ''
        t.extras_json = (
            f'{{"world_x":{world_x:.3f},"world_y":{world_y:.3f},'
            f'"yaw_deg":{math.degrees(self._last_yaw):.1f}}}'
        )
        t.confidence = 0.95
        self.pub_thought.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = TileTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
