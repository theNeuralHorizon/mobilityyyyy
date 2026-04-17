"""Floor logo detector — reads the downward-facing camera, crops to the
current tile's bbox (estimated from LiDAR wall distances + odom), runs HSV
thresholds for green and orange, and computes per-edge pixel counts.

The edge-sample output is consumed by the state machine, which applies the
rules (exclude entry edge, break ties, LiDAR veto) and picks an exit. This
node does NOT make motion decisions; it is a pure perception module.

Detection uses BOTH HSV thresholds AND RGB-ratio checks (OR'd together)
for robustness across different PBR rendering pipelines and lighting.
"""
from __future__ import annotations

import os
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from artpark_msgs.msg import EdgeSample, Thought


class FloorLogoDetector(Node):
    def __init__(self) -> None:
        super().__init__('floor_logo_detector')

        # ---- HSV thresholds (widened for PBR rendering tolerance) ----
        self.declare_parameter('hsv_green_low',  [25,  30,  40])
        self.declare_parameter('hsv_green_high', [95, 255, 255])
        self.declare_parameter('hsv_orange_low',  [5,  60,  80])
        self.declare_parameter('hsv_orange_high',[25, 255, 255])
        self.declare_parameter('hsv_red_low1',  [0,   80, 80])
        self.declare_parameter('hsv_red_high1', [10,  255, 255])
        self.declare_parameter('hsv_red_low2',  [165, 80, 80])
        self.declare_parameter('hsv_red_high2', [180, 255, 255])

        # Fraction of image used per edge strip (5% inset, 15% depth).
        self.declare_parameter('edge_inset',  0.05)
        self.declare_parameter('edge_depth',  0.15)
        self.declare_parameter('publish_rate_hz', 8.0)
        # Debug: save floor cam frames to disk for HSV calibration
        self.declare_parameter('debug_save_images', False)
        self.declare_parameter('debug_save_dir', '/tmp/floor_cam_debug')

        self.gl = np.array(self.get_parameter('hsv_green_low').value,  dtype=np.uint8)
        self.gh = np.array(self.get_parameter('hsv_green_high').value, dtype=np.uint8)
        self.ol = np.array(self.get_parameter('hsv_orange_low').value, dtype=np.uint8)
        self.oh = np.array(self.get_parameter('hsv_orange_high').value, dtype=np.uint8)
        self.rl1 = np.array(self.get_parameter('hsv_red_low1').value,  dtype=np.uint8)
        self.rh1 = np.array(self.get_parameter('hsv_red_high1').value, dtype=np.uint8)
        self.rl2 = np.array(self.get_parameter('hsv_red_low2').value,  dtype=np.uint8)
        self.rh2 = np.array(self.get_parameter('hsv_red_high2').value, dtype=np.uint8)
        self.inset = float(self.get_parameter('edge_inset').value)
        self.depth = float(self.get_parameter('edge_depth').value)
        self._debug_save = bool(self.get_parameter('debug_save_images').value)
        self._debug_dir = str(self.get_parameter('debug_save_dir').value)

        self.bridge = CvBridge()
        self._latest: Optional[np.ndarray] = None
        self._last_publish = 0.0
        self._debug_frame_count = 0

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            Image, '/floor_cam/image_raw', self._on_image, qos)
        self.pub = self.create_publisher(EdgeSample, '/edge_sample',
                                         QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.pub_thought = self.create_publisher(Thought, '/thought',
                                                  QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().info('floor_logo_detector up')

        self.timer = self.create_timer(
            1.0 / float(self.get_parameter('publish_rate_hz').value), self._tick)
        self._seq = 0

    # ------------------------------------------------------------
    def _on_image(self, msg: Image) -> None:
        try:
            self._latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge failure: {exc}')

    # ------------------------------------------------------------
    @staticmethod
    def _rgb_green_mask(bgr: np.ndarray) -> np.ndarray:
        """RGB-ratio green detection: G channel dominant over R and B."""
        b, g, r = bgr[:, :, 0].astype(np.int16), bgr[:, :, 1].astype(np.int16), bgr[:, :, 2].astype(np.int16)
        return ((g > r + 20) & (g > b + 20) & (g > 50)).astype(np.uint8) * 255

    @staticmethod
    def _rgb_orange_mask(bgr: np.ndarray) -> np.ndarray:
        """RGB-ratio orange detection: R high, G moderate, B low."""
        b, g, r = bgr[:, :, 0].astype(np.int16), bgr[:, :, 1].astype(np.int16), bgr[:, :, 2].astype(np.int16)
        return ((r > 100) & (g > 40) & (g < r) & (b < g) &
                (r - g > 30) & (r - b > 60)).astype(np.uint8) * 255

    @staticmethod
    def _rgb_red_mask(bgr: np.ndarray) -> np.ndarray:
        """RGB-ratio red detection: R dominant, G and B low."""
        b, g, r = bgr[:, :, 0].astype(np.int16), bgr[:, :, 1].astype(np.int16), bgr[:, :, 2].astype(np.int16)
        return ((r > 100) & (r > g * 2) & (r > b * 2)).astype(np.uint8) * 255

    # ------------------------------------------------------------
    def _tick(self) -> None:
        if self._latest is None:
            return

        img = self._latest
        h, w = img.shape[:2]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # HSV masks (widened thresholds)
        mask_g_hsv = cv2.inRange(hsv, self.gl, self.gh)
        mask_o_hsv = cv2.inRange(hsv, self.ol, self.oh)
        mask_r_hsv = cv2.inRange(hsv, self.rl1, self.rh1) | cv2.inRange(hsv, self.rl2, self.rh2)

        # RGB-ratio masks (robust to PBR brightness shifts)
        mask_g_rgb = self._rgb_green_mask(img)
        mask_o_rgb = self._rgb_orange_mask(img)
        mask_r_rgb = self._rgb_red_mask(img)

        # Combined: either HSV or RGB detects the color
        mask_g = mask_g_hsv | mask_g_rgb
        mask_o = mask_o_hsv | mask_o_rgb
        mask_r = mask_r_hsv | mask_r_rgb

        # Exclude blue/indigo (rosette center) from being misdetected.
        # Blue pixels: B > R and B > G and B > 80
        b_ch = img[:, :, 0].astype(np.int16)
        g_ch = img[:, :, 1].astype(np.int16)
        r_ch = img[:, :, 2].astype(np.int16)
        blue_mask = ((b_ch > r_ch + 30) & (b_ch > g_ch + 30) & (b_ch > 80)).astype(np.uint8) * 255
        mask_g = mask_g & ~blue_mask
        mask_o = mask_o & ~blue_mask
        mask_r = mask_r & ~blue_mask

        # Exclude grey/white background: pixels where all channels are within 30 of each other
        # and saturation is very low (not a color)
        max_ch = np.maximum(np.maximum(r_ch, g_ch), b_ch)
        min_ch = np.minimum(np.minimum(r_ch, g_ch), b_ch)
        grey_mask = ((max_ch - min_ch) < 25).astype(np.uint8) * 255
        mask_g = mask_g & ~grey_mask
        mask_o = mask_o & ~grey_mask

        # Edge strip geometry (pixel-space)
        inset_px = int(self.inset * min(h, w))
        depth_px = int(self.depth * min(h, w))

        # North strip: top of image = robot's "forward" edge of tile.
        n_box = (inset_px, inset_px + depth_px, inset_px, w - inset_px)
        s_box = (h - inset_px - depth_px, h - inset_px, inset_px, w - inset_px)
        w_box = (inset_px, h - inset_px, inset_px, inset_px + depth_px)
        e_box = (inset_px, h - inset_px, w - inset_px - depth_px, w - inset_px)

        def count(mask, b):
            r0, r1, c0, c1 = b
            return int(cv2.countNonZero(mask[r0:r1, c0:c1]))

        gN, gS, gW, gE = count(mask_g, n_box), count(mask_g, s_box), count(mask_g, w_box), count(mask_g, e_box)
        oN, oS, oW, oE = count(mask_o, n_box), count(mask_o, s_box), count(mask_o, w_box), count(mask_o, e_box)

        # Red: count center region (STOP tile is solid red, not edge-specific)
        center_r0, center_r1 = h // 4, 3 * h // 4
        center_c0, center_c1 = w // 4, 3 * w // 4
        red_total = int(cv2.countNonZero(mask_r[center_r0:center_r1, center_c0:center_c1]))

        # Debug: save images periodically for HSV calibration
        if self._debug_save and self._debug_frame_count % 40 == 0:
            try:
                os.makedirs(self._debug_dir, exist_ok=True)
                fname = os.path.join(self._debug_dir, f'frame_{self._debug_frame_count:06d}')
                cv2.imwrite(f'{fname}_raw.png', img)
                cv2.imwrite(f'{fname}_green.png', mask_g)
                cv2.imwrite(f'{fname}_orange.png', mask_o)
                # Log HSV stats for the center of the image
                cy, cx = h // 2, w // 2
                patch = hsv[cy-5:cy+5, cx-5:cx+5]
                if patch.size > 0:
                    self.get_logger().info(
                        f'DEBUG HSV center: H={patch[:,:,0].mean():.0f} '
                        f'S={patch[:,:,1].mean():.0f} V={patch[:,:,2].mean():.0f} | '
                        f'RGB center: R={img[cy,cx,2]} G={img[cy,cx,1]} B={img[cy,cx,0]}')
            except Exception:
                pass
        self._debug_frame_count += 1

        now = time.monotonic()
        if now - self._last_publish < 1.0 / float(self.get_parameter('publish_rate_hz').value):
            return
        self._last_publish = now

        es = EdgeSample()
        es.stamp = self.get_clock().now().to_msg()
        es.green_n, es.green_s, es.green_e, es.green_w = gN, gS, gE, gW
        es.orange_n, es.orange_s, es.orange_e, es.orange_w = oN, oS, oE, oW
        es.entry_edge = ''          # filled by state machine from its memory
        es.recommended_exit = ''    # filled by state machine after applying rules
        es.red_total = red_total
        es.confidence = 0.0
        self.pub.publish(es)

        self._seq += 1
        t = Thought()
        t.stamp = es.stamp
        t.seq = self._seq
        t.phase = 'PERCEPTION'
        t.hypothesis = f'green edges N={gN} S={gS} E={gE} W={gW} | orange N={oN} S={oS} E={oE} W={oW} | red={red_total}'
        t.action_chosen = 'publish_edge_sample'
        t.rule_applied = 'floor_logo_detector.edge_hsv_sample'
        t.alt_considered = ''
        t.extras_json = (
            f'{{"green":{{"n":{gN},"s":{gS},"e":{gE},"w":{gW}}},'
            f'"orange":{{"n":{oN},"s":{oS},"e":{oE},"w":{oW}}}}}'
        )
        t.confidence = 0.8 if max(gN, gS, gE, gW, oN, oS, oE, oW) > 50 else 0.3
        self.pub_thought.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = FloorLogoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
