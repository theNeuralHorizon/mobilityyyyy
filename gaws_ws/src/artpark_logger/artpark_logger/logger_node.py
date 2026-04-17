"""Run logger.

Creates a fresh run directory under `~/artpark_runs/<ISO_timestamp>/` and
writes four artefacts:
  1. judge_scorecard.csv — ONLY the scoreable events, for the judge to grade.
  2. thought_log.jsonl    — every /thought message, one JSON object per line.
  3. raw_sensor.csv       — light periodic sensor snapshot (best-effort).
  4. images/              — PNG frame per tag commit and per color marker hit.

The logger is a passive sink — it never touches /cmd_vel. If it dies, the
run continues (but scoring is lost, so watch it).
"""
from __future__ import annotations

import csv
import json
import os
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from artpark_msgs.msg import EdgeSample, TagEvent, TileEvent, Thought


def iso_now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec='milliseconds')


def fs_stamp() -> str:
    return datetime.now().strftime('%Y%m%dT%H%M%S')


class LoggerNode(Node):
    def __init__(self) -> None:
        super().__init__('logger_node')

        self.declare_parameter('runs_root', os.path.expanduser('~/artpark_runs'))
        root = Path(self.get_parameter('runs_root').value)
        self.run_dir = root / fs_stamp()
        (self.run_dir / 'images').mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f'logging → {self.run_dir}')

        self._lock = threading.Lock()
        self._seq = 0

        self._scorecard = open(self.run_dir / 'judge_scorecard.csv', 'w', newline='', encoding='utf-8')
        self._score_writer = csv.writer(self._scorecard)
        self._score_writer.writerow([
            'seq', 'iso_timestamp', 'event_type', 'tag_id', 'tag_label',
            'decision', 'image_path', 'tile_row', 'tile_col', 'color_marker',
        ])
        self._scorecard.flush()

        self._thought_log = open(self.run_dir / 'thought_log.jsonl', 'w', encoding='utf-8')
        self._raw = open(self.run_dir / 'raw_sensor.csv', 'w', newline='', encoding='utf-8')
        self._raw_writer = csv.writer(self._raw)
        self._raw_writer.writerow([
            'iso_timestamp', 'green_n', 'green_s', 'green_e', 'green_w',
            'orange_n', 'orange_s', 'orange_e', 'orange_w',
        ])

        self._bridge = CvBridge()
        self._last_front: Optional['cv2.Mat'] = None

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(TagEvent,   '/tag_event',    self._on_tag,    qos)
        self.create_subscription(TileEvent,  '/tile_event',   self._on_tile,   qos)
        self.create_subscription(EdgeSample, '/edge_sample',  self._on_edge,   qos)
        self.create_subscription(Thought,    '/thought',      self._on_thought, qos)
        self.create_subscription(String,     '/scorecard_event', self._on_scorestr, qos)
        self.create_subscription(Image,      '/front_cam/image_raw', self._on_front, qos_be)

    # ====================================================================
    def _on_front(self, msg: Image) -> None:
        try:
            self._last_front = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'front_cam decode: {exc}')

    def _save_image(self, event_type: str, label: int) -> str:
        if self._last_front is None:
            return ''
        fname = f'{fs_stamp()}_{self._seq:04d}_{event_type}_tag{label}.png'
        fpath = self.run_dir / 'images' / fname
        try:
            cv2.imwrite(str(fpath), self._last_front)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'imwrite: {exc}')
            return ''
        return str(fpath.relative_to(self.run_dir))

    # ====================================================================
    def _on_tag(self, msg: TagEvent) -> None:
        if not msg.first_sighting:
            return  # don't double-log; re-sightings go into thought_log
        with self._lock:
            self._seq += 1
            img = self._save_image('TAG_LOG', msg.logical_label)
            self._score_writer.writerow([
                self._seq, iso_now(), 'TAG_LOG',
                msg.tag_id, msg.logical_label, msg.decision,
                img, '', '', '',
            ])
            self._scorecard.flush()
            # Also emit a TAG_DECISION row for the +5 decision credit.
            self._seq += 1
            self._score_writer.writerow([
                self._seq, iso_now(), 'TAG_DECISION',
                msg.tag_id, msg.logical_label, msg.decision,
                '', '', '', '',
            ])
            self._scorecard.flush()

    def _on_tile(self, msg: TileEvent) -> None:
        with self._lock:
            self._seq += 1
            self._score_writer.writerow([
                self._seq, iso_now(), 'TILE_ENTER',
                '', '', '', '', msg.tile_row, msg.tile_col,
                msg.dominant_color or '',
            ])
            self._scorecard.flush()
            if msg.dominant_color in ('green', 'orange') and (msg.in_green_zone or msg.in_orange_zone):
                self._seq += 1
                img = self._save_image(f'COLOR_{msg.dominant_color.upper()}', 0)
                self._score_writer.writerow([
                    self._seq, iso_now(), 'COLOR_MARKER_HIT',
                    '', '', '', img, msg.tile_row, msg.tile_col,
                    msg.dominant_color,
                ])
                self._scorecard.flush()

    def _on_edge(self, msg: EdgeSample) -> None:
        with self._lock:
            self._raw_writer.writerow([
                iso_now(),
                msg.green_n, msg.green_s, msg.green_e, msg.green_w,
                msg.orange_n, msg.orange_s, msg.orange_e, msg.orange_w,
            ])
            self._raw.flush()

    def _on_thought(self, msg: Thought) -> None:
        try:
            extras = json.loads(msg.extras_json) if msg.extras_json else {}
        except Exception:  # noqa: BLE001
            extras = {'raw': msg.extras_json}
        entry = {
            'seq': msg.seq,
            'iso_timestamp': iso_now(),
            'phase': msg.phase,
            'hypothesis': msg.hypothesis,
            'action_chosen': msg.action_chosen,
            'rule_applied': msg.rule_applied,
            'alt_considered': msg.alt_considered,
            'confidence': float(msg.confidence),
            'extras': extras,
        }
        with self._lock:
            self._thought_log.write(json.dumps(entry, ensure_ascii=False) + '\n')
            self._thought_log.flush()

    def _on_scorestr(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._seq += 1
            self._score_writer.writerow([
                self._seq, iso_now(),
                payload.get('type', ''),
                payload.get('tag_id', ''),
                payload.get('label', ''),
                payload.get('decision', ''),
                payload.get('image_path', ''),
                payload.get('tile_row', ''),
                payload.get('tile_col', ''),
                payload.get('color_marker', ''),
            ])
            self._scorecard.flush()

    # ====================================================================
    def destroy_node(self) -> bool:
        try:
            self._scorecard.close()
            self._thought_log.close()
            self._raw.close()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
