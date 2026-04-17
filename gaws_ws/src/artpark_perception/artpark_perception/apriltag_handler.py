"""AprilTag handler — in-process OpenCV-ArUco detection + 3-frame vote.

Why we don't use apriltag_ros
-----------------------------
The ros-jazzy-apriltag-ros / ros-jazzy-apriltag-msgs packages ship with a
FastCDR ABI mismatch on stock Ubuntu 24.04; the detector binary dies with
`undefined symbol: _ZN8eprosima7fastcdr3Cdr9serializeEj`. Rather than
patching ABIs, we run detection in-process using OpenCV's built-in
DICT_APRILTAG_36h11 ArUco dictionary — same tag family, no apriltag_ros
dependency, no FastCDR drama.

Rules enforced:
  * 3 consecutive frames agreeing on tag_id and centre (<20 px) → commit.
  * Slow-down publish on /approach_mode while a candidate is in frame.
  * id → logical-label lookup from config/tag_label_map.yaml as a JSON
    string (ROS 2 parameters don't support dicts natively).
"""
from __future__ import annotations

import json
import math
import threading
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from artpark_msgs.msg import TagEvent, Thought


@dataclass
class DetectionSample:
    tag_id: int
    cx: float
    cy: float
    pixel_size: float   # avg edge length in px, used for distance estimate
    corners: np.ndarray


class AprilTagHandler(Node):
    def __init__(self) -> None:
        super().__init__('apriltag_handler')

        # ---------------- parameters (JSON strings to dodge ROS param-type limits) ----------------
        self.declare_parameter('vote_window',             3)
        self.declare_parameter('pixel_tolerance',         20.0)
        self.declare_parameter('min_tag_px',              25)
        self.declare_parameter('tag_side_m',              0.15)
        self.declare_parameter('focal_length_px',         337.0)   # 640/(2·tan(87°/2))
        self.declare_parameter('tag_label_map_json',      '{}')
        self.declare_parameter('action_by_label_json',    '{}')

        self.vote_window     = int(self.get_parameter('vote_window').value)
        self.px_tol          = float(self.get_parameter('pixel_tolerance').value)
        self.min_tag_px      = int(self.get_parameter('min_tag_px').value)
        self.tag_side_m      = float(self.get_parameter('tag_side_m').value)
        self.focal_px        = float(self.get_parameter('focal_length_px').value)

        try:
            raw_id_map = json.loads(self.get_parameter('tag_label_map_json').value or '{}')
            raw_act_map = json.loads(self.get_parameter('action_by_label_json').value or '{}')
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Failed to parse tag_label_map_json / action_by_label_json: {exc}')
            raw_id_map, raw_act_map = {}, {}

        self.id_to_label:  Dict[int, int] = {int(k): int(v) for k, v in raw_id_map.items()}
        self.label_to_act: Dict[int, str] = {int(k): str(v) for k, v in raw_act_map.items()}

        # ---------------- AprilTag detector ----------------
        # pupil_apriltags is the primary: pure Python binding over the C
        # apriltag library, stable across distros. cv2.aruco 4.6 segfaults
        # on 36h11 (Ubuntu 24.04 stock) so we avoid it unless necessary.
        self._pupil_detector = None
        self._cv_detect_fn = None
        try:
            from pupil_apriltags import Detector as _ATDetector  # type: ignore
            self._pupil_detector = _ATDetector(
                families='tag36h11',
                nthreads=2,
                quad_decimate=1.0,
                refine_edges=True,
            )
            self.get_logger().info('Using pupil_apriltags (tag36h11)')
        except ImportError:
            self.get_logger().warn(
                'pupil_apriltags not installed — falling back to cv2.aruco '
                '(WARNING: segfaults on OpenCV 4.6). '
                'Install with: pip install --break-system-packages pupil-apriltags')
            self._aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            try:
                params = cv2.aruco.DetectorParameters()
            except AttributeError:
                params = cv2.aruco.DetectorParameters_create()
            if hasattr(cv2.aruco, 'ArucoDetector'):
                det = cv2.aruco.ArucoDetector(self._aruco_dict, params)
                self._cv_detect_fn = det.detectMarkers
            else:
                _d, _p = self._aruco_dict, params
                self._cv_detect_fn = lambda g: cv2.aruco.detectMarkers(g, _d, parameters=_p)

        # ---------------- state ----------------
        self._buffers: Dict[int, Deque[DetectionSample]] = {}
        self._committed_ids: set[int] = set()
        self._lock = threading.Lock()
        self._seq = 0
        self._bridge = CvBridge()

        # ---------------- I/O ----------------
        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_be  = QoSProfile(depth=5,  reliability=ReliabilityPolicy.BEST_EFFORT)

        self.sub = self.create_subscription(Image, '/front_cam/image_raw', self._on_image, qos_be)
        self.pub_event    = self.create_publisher(TagEvent, '/tag_event', qos_rel)
        self.pub_approach = self.create_publisher(Bool, '/approach_mode', qos_rel)
        self.pub_thought  = self.create_publisher(Thought, '/thought', qos_rel)

        self.get_logger().info(
            f'apriltag_handler up | id→label={self.id_to_label} | action_by_label={self.label_to_act}')

    # ========================================================================
    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge failure: {exc}')
            return

        gray = np.ascontiguousarray(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), dtype=np.uint8)

        # Normalise detections from either backend into a common list of
        # (tag_id, corners_4x2_ndarray).
        detections: list[tuple[int, np.ndarray]] = []
        if self._pupil_detector is not None:
            try:
                for d in self._pupil_detector.detect(gray):
                    detections.append((int(d.tag_id), np.asarray(d.corners, dtype=np.float32)))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'pupil_apriltags failure: {exc}')
        else:
            try:
                res = self._cv_detect_fn(gray)
                cl, ids = res[0], res[1]
                if ids is not None and len(ids) > 0:
                    for i, tid in enumerate(ids.flatten().tolist()):
                        detections.append((int(tid), cl[i].reshape(4, 2).astype(np.float32)))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'cv2.aruco failure: {exc}')

        if not detections:
            self.pub_approach.publish(Bool(data=False))
            return

        any_candidate = False
        for tag_id, corners in detections:
            cx = float(corners[:, 0].mean())
            cy = float(corners[:, 1].mean())
            edges = [
                float(np.linalg.norm(corners[(k + 1) % 4] - corners[k]))
                for k in range(4)
            ]
            pixel_size = float(sum(edges) / 4.0)
            if pixel_size < self.min_tag_px:
                continue

            sample = DetectionSample(tag_id=tag_id, cx=cx, cy=cy,
                                     pixel_size=pixel_size, corners=corners)
            buf = self._buffers.setdefault(tag_id, deque(maxlen=self.vote_window))
            buf.append(sample)
            any_candidate = True

            if len(buf) == self.vote_window and self._vote_agrees(buf):
                if tag_id not in self._committed_ids:
                    self._committed_ids.add(tag_id)
                    self._emit_commit(sample, first_sighting=True)
                else:
                    self._emit_commit(sample, first_sighting=False)
                buf.clear()

        self.pub_approach.publish(Bool(data=any_candidate))

    def _vote_agrees(self, buf: Deque[DetectionSample]) -> bool:
        first = buf[0]
        for s in list(buf)[1:]:
            if s.tag_id != first.tag_id:
                return False
            if math.hypot(s.cx - first.cx, s.cy - first.cy) > self.px_tol:
                return False
        return True

    def _emit_commit(self, sample: DetectionSample, first_sighting: bool) -> None:
        label = self.id_to_label.get(sample.tag_id, 0)
        action = self.label_to_act.get(label, 'UNKNOWN')

        # Distance estimate from pixel size (pinhole model).
        distance = (self.tag_side_m * self.focal_px) / max(sample.pixel_size, 1.0)

        ev = TagEvent()
        ev.stamp = self.get_clock().now().to_msg()
        ev.tag_id = int(sample.tag_id)
        ev.logical_label = int(label)
        ev.decision = action
        ev.tag_in_camera = Pose()
        ev.distance = float(distance)
        ev.bearing = math.atan2(sample.cx - 320.0, self.focal_px)
        ev.vote_frames = self.vote_window
        ev.first_sighting = bool(first_sighting)
        self.pub_event.publish(ev)

        self._seq += 1
        t = Thought()
        t.stamp = ev.stamp
        t.seq = self._seq
        t.phase = 'APPROACH_TAG'
        t.hypothesis = (f'Tag id={sample.tag_id} committed (label={label}, action={action}) '
                        f'at ~{distance:.2f} m')
        t.action_chosen = f'publish_TagEvent(label={label},action={action})'
        t.rule_applied = 'apriltag_handler.vote_pass'
        t.alt_considered = 'reject_as_noise'
        t.extras_json = json.dumps({
            'tag_id': sample.tag_id,
            'label': label,
            'first': first_sighting,
            'pixel_size': sample.pixel_size,
            'center_px': [sample.cx, sample.cy],
        })
        t.confidence = 1.0 if label != 0 else 0.3
        self.pub_thought.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
