"""Top-level state machine.

Phases:
  INIT            - settle, bring sensors online
  EXPLORE         - PD right-wall-follow until tag detected
  APPROACH_TAG    - slow down, center on tag, wait for vote commit
  ACT_LEFT/RIGHT  - rotate +/-90deg
  ACT_UTURN       - rotate 180deg
  POST_ROTATE     - drive forward ~0.8 m after rotation to enter next tile
  GREEN_FOLLOW    - per-tile edge-sample -> exit toward green
  ORANGE_FOLLOW   - same but orange
  SEEK_TAG_5      - wall-follow + scan for AprilTags after green exhausts
  RECOVERY        - escape stuck situations (spin / reverse / escape)
  REACH_STOP      - solid-red tile or STOP pose -> final halt

All scoreable events are surfaced on /tag_event (from apriltag_handler) and
/tile_event (from tile_tracker); the state machine mirrors them into
/scorecard for the logger. The state machine itself only owns /cmd_vel.
"""
from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, String

from artpark_msgs.msg import EdgeSample, TagEvent, TileEvent, Thought

from .edge_sampler import decide_exit, EDGES, OPPOSITE


class Phase(Enum):
    INIT = auto()
    EXPLORE = auto()
    APPROACH_TAG = auto()
    ACT_LEFT = auto()
    ACT_RIGHT = auto()
    ACT_UTURN = auto()
    POST_ROTATE = auto()
    GREEN_FOLLOW = auto()
    ORANGE_FOLLOW = auto()
    SEEK_TAG_5 = auto()
    RECOVERY = auto()
    REACH_STOP = auto()


@dataclass
class SMState:
    phase: Phase = Phase.INIT
    seq: int = 0
    entry_edge: Optional[str] = None     # world-frame edge for current tile
    tile_rc: Optional[tuple[int, int]] = None
    obstacle_near: bool = False
    approach_mode: bool = False
    last_edge_sample: Optional[EdgeSample] = None
    tags_logged: set[int] = field(default_factory=set)
    committed_action: Optional[str] = None
    rotation_target_yaw: Optional[float] = None
    current_yaw: float = 0.0
    odom_x: float = 0.0
    odom_y: float = 0.0

    # PD wall-following state
    wall_error_prev: float = 0.0

    # Post-rotation forward drive
    post_rotate_start_x: float = 0.0
    post_rotate_start_y: float = 0.0
    post_rotate_target: Phase = Phase.EXPLORE

    # U-turn entry edge preservation
    pre_uturn_entry: Optional[str] = None

    # Approach timeout
    approach_start: float = 0.0

    # Stuck detection
    position_history: deque = field(default_factory=lambda: deque(maxlen=60))
    last_history_time: float = 0.0

    # Recovery
    recovery_strategy: int = 0
    recovery_start: float = 0.0
    recovery_target_yaw: Optional[float] = None
    phase_before_recovery: Phase = Phase.EXPLORE

    # Color-following grace period tracking
    color_follow_start_x: float = 0.0
    color_follow_start_y: float = 0.0
    color_follow_start_time: float = 0.0
    color_zero_streak: int = 0          # consecutive ticks with 0 color pixels
    color_ever_seen: bool = False       # whether we ever saw color in this phase


class StateMachine(Node):
    def __init__(self) -> None:
        super().__init__('state_machine')

        # -------- parameters --------
        self.declare_parameter('v_explore',  0.20)
        self.declare_parameter('v_approach', 0.08)
        self.declare_parameter('v_follow',   0.15)
        self.declare_parameter('w_turn',     0.8)
        self.declare_parameter('idle_watchdog_s', 4.0)
        self.declare_parameter('debug_hold', False)
        # PD wall-following
        self.declare_parameter('wall_target_m', 0.40)
        self.declare_parameter('wall_kp', 1.0)
        self.declare_parameter('wall_kd', 0.5)
        # Stuck detection
        self.declare_parameter('stuck_timeout_s', 10.0)
        self.declare_parameter('stuck_distance_m', 0.10)
        # Post-rotation
        self.declare_parameter('post_rotate_dist_m', 0.80)
        # Wall-following thresholds
        self.declare_parameter('emergency_front_m', 0.25)
        self.declare_parameter('slow_front_m', 0.40)
        self.declare_parameter('opening_threshold_m', 0.80)
        # Color-following grace
        self.declare_parameter('color_grace_dist_m', 0.40)
        self.declare_parameter('color_grace_time_s', 3.0)
        self.declare_parameter('color_exhaust_streak', 15)
        # Red STOP detection
        self.declare_parameter('red_stop_threshold', 3000)

        self.v_exp   = float(self.get_parameter('v_explore').value)
        self.v_app   = float(self.get_parameter('v_approach').value)
        self.v_fol   = float(self.get_parameter('v_follow').value)
        self.w_turn  = float(self.get_parameter('w_turn').value)
        self.idle_s  = float(self.get_parameter('idle_watchdog_s').value)
        self.debug_hold = bool(self.get_parameter('debug_hold').value)
        self.wall_target = float(self.get_parameter('wall_target_m').value)
        self.wall_kp = float(self.get_parameter('wall_kp').value)
        self.wall_kd = float(self.get_parameter('wall_kd').value)
        self.stuck_timeout = float(self.get_parameter('stuck_timeout_s').value)
        self.stuck_dist = float(self.get_parameter('stuck_distance_m').value)
        self.post_rotate_dist = float(self.get_parameter('post_rotate_dist_m').value)
        self.emergency_front = float(self.get_parameter('emergency_front_m').value)
        self.slow_front = float(self.get_parameter('slow_front_m').value)
        self.opening_thresh = float(self.get_parameter('opening_threshold_m').value)
        self.color_grace_dist = float(self.get_parameter('color_grace_dist_m').value)
        self.color_grace_time = float(self.get_parameter('color_grace_time_s').value)
        self.color_exhaust_streak = int(self.get_parameter('color_exhaust_streak').value)
        self.red_stop_threshold = int(self.get_parameter('red_stop_threshold').value)

        # -------- state --------
        self.s = SMState()
        self._last_cmd_time = time.monotonic()

        # -------- I/O --------
        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_be  = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(TagEvent,   '/tag_event',   self._on_tag,  qos_rel)
        self.create_subscription(EdgeSample, '/edge_sample', self._on_edge, qos_rel)
        self.create_subscription(TileEvent,  '/tile_event',  self._on_tile, qos_rel)
        self.create_subscription(Bool,       '/obstacle_near', self._on_obs, qos_rel)
        self.create_subscription(Bool,       '/approach_mode', self._on_app, qos_rel)
        self.create_subscription(Float32MultiArray, '/obstacle_octants',
                                  self._on_octants, qos_rel)
        self.create_subscription(Odometry,   '/odom', self._on_odom, qos_be)

        self.pub_cmd     = self.create_publisher(Twist, '/cmd_vel', qos_rel)
        self.pub_thought = self.create_publisher(Thought, '/thought', qos_rel)
        self.pub_score   = self.create_publisher(String, '/scorecard_event', qos_rel)

        # 20 Hz tick drives motion
        self.timer = self.create_timer(0.05, self._tick)
        # 0.5 Hz heartbeat so operators see what the node is doing.
        self._heartbeat_timer = self.create_timer(2.0, self._heartbeat)
        self._prev_phase_logged = None

        # Octant layout (from obstacle_monitor formula):
        #   oct[0]=back, oct[1]=back-right, oct[2]=right, oct[3]=front-right,
        #   oct[4]=forward, oct[5]=front-left, oct[6]=left, oct[7]=back-left
        self._octants = [float('inf')] * 8
        self.get_logger().info('state_machine up, entering INIT')

    # ================================================================
    # callbacks
    def _on_tag(self, msg: TagEvent) -> None:
        if msg.tag_id in self.s.tags_logged and not msg.first_sighting:
            return
        self.s.tags_logged.add(msg.tag_id)

        # Emit scorecard row (logger listens)
        row = {
            'type': 'TAG_LOG',
            'tag_id': msg.tag_id,
            'label': msg.logical_label,
            'decision': msg.decision,
            'distance': msg.distance,
            'bearing': msg.bearing,
        }
        self.pub_score.publish(String(data=json.dumps(row)))

        action = msg.decision
        self._think(f'TagEvent committed: label={msg.logical_label}, action={action}',
                    rule='state_machine.on_tag_commit',
                    confidence=1.0)

        # LiDAR cross-check: reject LEFT/RIGHT if wall blocks the target direction
        if action == 'LEFT':
            left_dist = self._oct(6) if len(self._octants) > 6 else float('inf')
            if left_dist < 0.40:
                self._think(f'LiDAR rejects LEFT: wall at {left_dist:.2f}m on left',
                            rule='state_machine.lidar_veto', confidence=0.9)
                return
            self._start_rotation(+math.pi / 2, Phase.ACT_LEFT)
        elif action == 'RIGHT':
            right_dist = self._oct(2) if len(self._octants) > 2 else float('inf')
            if right_dist < 0.40:
                self._think(f'LiDAR rejects RIGHT: wall at {right_dist:.2f}m on right',
                            rule='state_machine.lidar_veto', confidence=0.9)
                return
            self._start_rotation(-math.pi / 2, Phase.ACT_RIGHT)
        elif action == 'U_TURN':
            self.s.pre_uturn_entry = self.s.entry_edge
            self._start_rotation(math.pi, Phase.ACT_UTURN)
        elif action == 'GREEN':
            self._enter_color_follow(Phase.GREEN_FOLLOW)
        elif action == 'ORANGE':
            self._enter_color_follow(Phase.ORANGE_FOLLOW)

    def _on_edge(self, msg: EdgeSample) -> None:
        self.s.last_edge_sample = msg

        # Red STOP detection — only active during ORANGE_FOLLOW or SEEK_TAG_5
        if self.s.phase in (Phase.ORANGE_FOLLOW, Phase.SEEK_TAG_5):
            if msg.red_total > self.red_stop_threshold:
                # Require robot has moved >1m from spawn to avoid false-positive
                # on the green START tile
                dist = math.hypot(self.s.odom_x - (-1.35), self.s.odom_y - 1.80)
                if dist > 1.0:
                    self.s.phase = Phase.REACH_STOP
                    self.pub_score.publish(String(data=json.dumps({'type': 'STOP_REACHED'})))
                    self._think(f'RED STOP detected: red_total={msg.red_total}, dist={dist:.2f}m',
                                rule='state_machine.red_stop', confidence=0.9)

    def _on_tile(self, msg: TileEvent) -> None:
        self.s.tile_rc = (msg.tile_row, msg.tile_col)
        self.s.entry_edge = msg.entry_edge if msg.entry_edge else self.s.entry_edge

    def _on_obs(self, msg: Bool) -> None:
        self.s.obstacle_near = msg.data

    def _on_app(self, msg: Bool) -> None:
        self.s.approach_mode = msg.data
        if self.s.approach_mode and self.s.phase in (Phase.EXPLORE, Phase.SEEK_TAG_5):
            self.s.phase = Phase.APPROACH_TAG
            self.s.approach_start = time.monotonic()

    def _on_octants(self, msg: Float32MultiArray) -> None:
        self._octants = list(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.s.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.s.odom_x = msg.pose.pose.position.x
        self.s.odom_y = msg.pose.pose.position.y

    # ================================================================
    def _heartbeat(self) -> None:
        self.get_logger().info(
            f'PHASE={self.s.phase.name} | '
            f'odom=({self.s.odom_x:.2f},{self.s.odom_y:.2f}) | '
            f'tile={self.s.tile_rc} | '
            f'entry_edge={self.s.entry_edge} | tags_logged={sorted(self.s.tags_logged)} | '
            f'yaw={math.degrees(self.s.current_yaw):.0f}deg | '
            f'obs_near={self.s.obstacle_near} | approach={self.s.approach_mode} | '
            f'hold={self.debug_hold} | '
            f'oct_fwd={self._oct(4):.2f} oct_right={self._oct(2):.2f} '
            f'oct_left={self._oct(6):.2f}')

    def _oct(self, idx: int) -> float:
        """Safe octant access."""
        if idx < len(self._octants):
            v = self._octants[idx]
            return v if math.isfinite(v) and v > 0 else float('inf')
        return float('inf')

    # ================================================================
    def _tick(self) -> None:
        ph = self.s.phase
        if ph != self._prev_phase_logged:
            self.get_logger().info(f'PHASE TRANSITION -> {ph.name}')
            self._prev_phase_logged = ph
        cmd = Twist()

        # ---- Stuck detection (active motion phases only) ----
        now = time.monotonic()
        if now - self.s.last_history_time > 2.0:
            self.s.position_history.append((now, self.s.odom_x, self.s.odom_y))
            self.s.last_history_time = now

        if ph in (Phase.EXPLORE, Phase.GREEN_FOLLOW, Phase.ORANGE_FOLLOW,
                  Phase.SEEK_TAG_5, Phase.POST_ROTATE):
            if self._check_stuck(now):
                return

        # ---- Phase dispatch ----
        if ph == Phase.INIT:
            if self.debug_hold:
                self._publish(cmd)
                return
            if time.monotonic() - self._last_cmd_time > 1.0:
                self.s.phase = Phase.EXPLORE
                self._think('INIT -> EXPLORE (settle complete)',
                            rule='state_machine.init_timeout',
                            confidence=1.0)
            self._publish(cmd)
            return

        if ph == Phase.EXPLORE:
            self._explore_wall_follow(cmd)
            return

        if ph == Phase.APPROACH_TAG:
            # Timeout: if tag vote doesn't commit within 8s, revert to EXPLORE
            if time.monotonic() - self.s.approach_start > 8.0:
                self.s.phase = Phase.EXPLORE
                self.s.approach_mode = False
                self._think('APPROACH_TAG timeout (8s) -> EXPLORE',
                            rule='state_machine.approach_timeout', confidence=0.8)
                return
            if self.s.obstacle_near:
                self._publish(Twist())
                return
            cmd.linear.x = self.v_app
            self._publish(cmd)
            return

        if ph in (Phase.ACT_LEFT, Phase.ACT_RIGHT, Phase.ACT_UTURN):
            self._run_rotation(cmd)
            return

        if ph == Phase.POST_ROTATE:
            self._run_post_rotate(cmd)
            return

        if ph == Phase.GREEN_FOLLOW:
            self._follow_color(cmd, color='green')
            return

        if ph == Phase.ORANGE_FOLLOW:
            self._follow_color(cmd, color='orange')
            return

        if ph == Phase.SEEK_TAG_5:
            self._explore_wall_follow(cmd)
            return

        if ph == Phase.RECOVERY:
            self._run_recovery(cmd)
            return

        if ph == Phase.REACH_STOP:
            self._publish(Twist())
            return

    # ================================================================
    # EXPLORE: PD right-wall-following
    # ================================================================
    def _explore_wall_follow(self, cmd: Twist) -> None:
        """PD-controlled right-wall-follower using octant LiDAR data.

        Octant layout:
          oct[4]=forward, oct[3]=front-right, oct[2]=right,
          oct[5]=front-left, oct[6]=left

        Behaviors (checked in priority order):
          1. Emergency: front < 0.25m -> hard left turn
          2. Slow approach: front < 0.40m -> creep + left turn
          3. Too close right: front-right < 0.30m -> veer left
          4. Opening right: right + front-right > 0.80m -> gentle right
          5. PD tracking: maintain right wall at ~0.40m
          6. No right wall: drive forward + gentle right drift
        """
        fwd = self._oct(4)
        fwd_right = self._oct(3)
        right = self._oct(2)

        # 1. Emergency stop + hard left turn
        if fwd < self.emergency_front:
            cmd.angular.z = self.w_turn
            self._think(f'EXPLORE: EMERGENCY front={fwd:.2f}m, hard left',
                        rule='wall_follow.emergency', confidence=1.0)
            self._publish(cmd)
            return

        # 2. Slow approach + medium left turn
        if fwd < self.slow_front:
            cmd.linear.x = 0.08
            cmd.angular.z = self.w_turn * 0.6
            self._think(f'EXPLORE: slow approach front={fwd:.2f}m',
                        rule='wall_follow.slow_approach', confidence=0.9)
            self._publish(cmd)
            return

        # 3. Front-right too close — veer left
        if fwd_right < 0.30:
            cmd.linear.x = self.v_exp * 0.7
            cmd.angular.z = self.w_turn * 0.4
            self._publish(cmd)
            return

        # 4. Opening on right (lost the wall) — gentle right to follow it
        if right > self.opening_thresh and fwd_right > self.opening_thresh:
            cmd.linear.x = self.v_exp
            cmd.angular.z = -0.3  # gentle right turn into the opening
            self._publish(cmd)
            return

        # 5. PD wall tracking: maintain right wall at target distance
        if right < 2.0:  # right wall is visible (within 2m)
            error = right - self.wall_target
            d_error = error - self.s.wall_error_prev
            self.s.wall_error_prev = error

            cmd.linear.x = self.v_exp
            cmd.angular.z = -(self.wall_kp * error + self.wall_kd * d_error)
            # Clamp angular velocity
            cmd.angular.z = max(-self.w_turn, min(self.w_turn, cmd.angular.z))
            self._publish(cmd)
            return

        # 6. No right wall detected — drive forward with gentle right drift
        cmd.linear.x = self.v_exp
        cmd.angular.z = -0.2
        self.s.wall_error_prev = 0.0
        self._publish(cmd)

    # ================================================================
    # Rotation control
    # ================================================================
    def _start_rotation(self, delta_yaw: float, new_phase: Phase) -> None:
        target = self.s.current_yaw + delta_yaw
        target = math.atan2(math.sin(target), math.cos(target))
        self.s.rotation_target_yaw = target
        self.s.phase = new_phase
        self._think(f'starting rotation delta={math.degrees(delta_yaw):.0f}deg '
                    f'-> target yaw {math.degrees(target):.0f}deg',
                    rule='state_machine.rotation_start', confidence=1.0)

    def _run_rotation(self, cmd: Twist) -> None:
        if self.s.rotation_target_yaw is None:
            self.s.phase = Phase.GREEN_FOLLOW if 3 in self.s.tags_logged else Phase.EXPLORE
            return

        diff = math.atan2(
            math.sin(self.s.rotation_target_yaw - self.s.current_yaw),
            math.cos(self.s.rotation_target_yaw - self.s.current_yaw),
        )

        if abs(diff) < math.radians(5):
            # Rotation complete — transition to POST_ROTATE
            self.s.rotation_target_yaw = None
            self.s.post_rotate_start_x = self.s.odom_x
            self.s.post_rotate_start_y = self.s.odom_y

            if self.s.phase == Phase.ACT_UTURN:
                # After U-turn: set entry_edge to OPPOSITE of pre-uturn entry
                if self.s.pre_uturn_entry:
                    self.s.entry_edge = OPPOSITE.get(self.s.pre_uturn_entry, '')
                else:
                    self.s.entry_edge = None
                self.s.pre_uturn_entry = None
                self.s.post_rotate_target = Phase.GREEN_FOLLOW
            else:
                # After LEFT/RIGHT: drive forward then resume EXPLORE
                self.s.entry_edge = None
                self.s.post_rotate_target = Phase.EXPLORE

            self.s.phase = Phase.POST_ROTATE
            self._think(f'rotation complete -> POST_ROTATE (target={self.s.post_rotate_target.name})',
                        rule='state_machine.rotation_done', confidence=1.0)
            return

        # Proportional rotation for smoother approach
        speed = self.w_turn if abs(diff) > math.radians(20) else self.w_turn * 0.5
        cmd.angular.z = speed if diff > 0 else -speed
        self._publish(cmd)

    # ================================================================
    # POST_ROTATE: drive forward after rotation
    # ================================================================
    def _run_post_rotate(self, cmd: Twist) -> None:
        dist = math.hypot(
            self.s.odom_x - self.s.post_rotate_start_x,
            self.s.odom_y - self.s.post_rotate_start_y,
        )

        # Done if we've driven far enough or hit an obstacle
        if dist >= self.post_rotate_dist or self.s.obstacle_near:
            target_phase = self.s.post_rotate_target
            # If transitioning to color follow, initialize grace tracking
            if target_phase in (Phase.GREEN_FOLLOW, Phase.ORANGE_FOLLOW):
                self._enter_color_follow(target_phase)
            else:
                self.s.phase = target_phase
            self._think(f'POST_ROTATE done (dist={dist:.2f}m) -> {target_phase.name}',
                        rule='state_machine.post_rotate_done', confidence=1.0)
            self._publish(Twist())
            return

        # Obstacle avoidance even during post-rotate
        fwd = self._oct(4)
        if fwd < self.emergency_front:
            self.s.phase = self.s.post_rotate_target
            self._think(f'POST_ROTATE wall ahead ({fwd:.2f}m) -> {self.s.post_rotate_target.name}',
                        rule='state_machine.post_rotate_wall', confidence=0.9)
            self._publish(Twist())
            return

        cmd.linear.x = self.v_exp
        self._publish(cmd)

    # ================================================================
    # Color following (robot-frame) with grace period
    # ================================================================
    def _enter_color_follow(self, phase: Phase) -> None:
        """Initialize color-following state with grace period tracking."""
        self.s.phase = phase
        self.s.entry_edge = None
        self.s.color_follow_start_x = self.s.odom_x
        self.s.color_follow_start_y = self.s.odom_y
        self.s.color_follow_start_time = time.monotonic()
        self.s.color_zero_streak = 0
        self.s.color_ever_seen = False

    def _color_follow_dist(self) -> float:
        """Distance driven since entering color-follow."""
        return math.hypot(
            self.s.odom_x - self.s.color_follow_start_x,
            self.s.odom_y - self.s.color_follow_start_y,
        )

    def _follow_color(self, cmd: Twist, color: str) -> None:
        """Follow green/orange floor markers using robot-frame edge sampling.

        The floor camera's edge labels are in ROBOT frame:
          N = image top = robot forward
          S = image bottom = robot backward
          E = image right = robot right
          W = image left = robot left

        Grace period: don't declare exhaustion until we've driven at least
        color_grace_dist_m AND waited color_grace_time_s AND had
        color_exhaust_streak consecutive zero-pixel frames.
        """
        # --- Obstacle avoidance (highest priority) ---
        fwd = self._oct(4)
        fwd_right = self._oct(3)

        if fwd < self.emergency_front:
            cmd.angular.z = self.w_turn
            self._publish(cmd)
            return

        if fwd < self.slow_front:
            cmd.linear.x = 0.08
            cmd.angular.z = self.w_turn * 0.5
            self._publish(cmd)
            return

        if fwd_right < 0.25:
            cmd.linear.x = self.v_fol * 0.5
            cmd.angular.z = self.w_turn * 0.3
            self._publish(cmd)
            return

        # --- Color decision ---
        es = self.s.last_edge_sample
        if es is None:
            cmd.linear.x = self.v_fol * 0.5
            self._publish(cmd)
            return

        if color == 'green':
            counts = {'N': es.green_n, 'S': es.green_s, 'E': es.green_e, 'W': es.green_w}
        else:
            counts = {'N': es.orange_n, 'S': es.orange_s, 'E': es.orange_e, 'W': es.orange_w}

        total_color = sum(counts.values())

        # Robot-frame wall veto
        walls = self._walls_from_octants()

        # Always exclude behind us (robot-frame 'S')
        decision = decide_exit(counts, entry_edge='S',
                               wall_blocked=walls,
                               min_pixels=30, near_tie_ratio=0.15)

        self._think(
            f'{color}_FOLLOW: counts={counts} walls={walls} '
            f'-> {decision.chosen or "NONE"} ({decision.reason}) '
            f'zero_streak={self.s.color_zero_streak} ever_seen={self.s.color_ever_seen}',
            rule=f'state_machine.{color}_follow.{decision.reason}',
            confidence=decision.confidence,
        )

        # Track whether we've ever seen this color
        if total_color > 30:
            self.s.color_ever_seen = True
            self.s.color_zero_streak = 0
        else:
            self.s.color_zero_streak += 1

        if not decision.chosen:
            # Check grace period before declaring exhaustion
            dist = self._color_follow_dist()
            elapsed = time.monotonic() - self.s.color_follow_start_time
            grace_ok = (dist >= self.color_grace_dist and
                        elapsed >= self.color_grace_time)
            streak_ok = self.s.color_zero_streak >= self.color_exhaust_streak

            if grace_ok and streak_ok:
                if color == 'green':
                    self.s.phase = Phase.SEEK_TAG_5
                    self._think(f'green arc exhausted (dist={dist:.2f}m, streak={self.s.color_zero_streak}) '
                                f'-> SEEK_TAG_5',
                                rule='state_machine.green_exhausted', confidence=0.7)
                else:
                    # Orange exhausted — likely at STOP
                    self.s.phase = Phase.REACH_STOP
                    self.pub_score.publish(String(data=json.dumps({'type': 'STOP_REACHED'})))
                    self._think(f'orange exhausted (dist={dist:.2f}m, streak={self.s.color_zero_streak}) '
                                f'-> REACH_STOP',
                                rule='state_machine.orange_exhausted', confidence=0.8)
                self._publish(Twist())
                return

            # Grace period not met — keep driving forward
            cmd.linear.x = self.v_fol
            self._publish(cmd)
            return

        # Convert robot-frame exit to heading delta
        delta = {'N': 0.0, 'E': -math.pi / 2, 'S': math.pi, 'W': math.pi / 2}
        target_heading = self.s.current_yaw + delta[decision.chosen]
        target_heading = math.atan2(math.sin(target_heading), math.cos(target_heading))

        diff = math.atan2(
            math.sin(target_heading - self.s.current_yaw),
            math.cos(target_heading - self.s.current_yaw),
        )
        if abs(diff) > math.radians(15):
            cmd.angular.z = self.w_turn * (1 if diff > 0 else -1) * 0.8
        else:
            cmd.linear.x = self.v_fol
            cmd.angular.z = 0.6 * diff  # P-correction while driving

        self._publish(cmd)

    def _walls_from_octants(self) -> Dict[str, bool]:
        """Map octant distances to robot-frame wall flags."""
        def blocked(idx: int) -> bool:
            v = self._oct(idx)
            return 0 < v < 0.45

        return {
            'N': blocked(4),  # forward
            'E': blocked(2),  # right
            'S': blocked(0),  # back
            'W': blocked(6),  # left
        }

    # ================================================================
    # Stuck detection + Recovery
    # ================================================================
    def _check_stuck(self, now: float) -> bool:
        """Check if robot is stuck. Returns True if entering RECOVERY."""
        if len(self.s.position_history) < 5:
            return False

        for t, x, y in self.s.position_history:
            if now - t >= self.stuck_timeout:
                dist = math.hypot(self.s.odom_x - x, self.s.odom_y - y)
                if dist < self.stuck_dist:
                    self.s.phase_before_recovery = self.s.phase
                    self.s.phase = Phase.RECOVERY
                    self.s.recovery_start = now
                    self.s.recovery_target_yaw = None
                    self._think(
                        f'STUCK detected: moved only {dist:.3f}m in {now - t:.1f}s. '
                        f'Strategy {self.s.recovery_strategy % 3}',
                        rule='state_machine.stuck_detect', confidence=1.0)
                    self.s.position_history.clear()
                    self.s.last_history_time = now
                    return True
                break
        return False

    def _run_recovery(self, cmd: Twist) -> None:
        """Cycling recovery strategies: spin, reverse, escape."""
        elapsed = time.monotonic() - self.s.recovery_start
        strategy = self.s.recovery_strategy % 3

        if strategy == 0:
            if elapsed < 6.0:
                cmd.angular.z = 0.6
                self._publish(cmd)
            else:
                self._end_recovery()
            return

        if strategy == 1:
            if elapsed < 2.0:
                cmd.linear.x = -0.15
                self._publish(cmd)
            else:
                self._end_recovery()
            return

        if strategy == 2:
            if elapsed < 2.0:
                cmd.linear.x = -0.15
                self._publish(cmd)
                return
            if self.s.recovery_target_yaw is None:
                target = self.s.current_yaw + math.pi / 2
                self.s.recovery_target_yaw = math.atan2(
                    math.sin(target), math.cos(target))
            diff = math.atan2(
                math.sin(self.s.recovery_target_yaw - self.s.current_yaw),
                math.cos(self.s.recovery_target_yaw - self.s.current_yaw),
            )
            if abs(diff) < math.radians(10):
                self._end_recovery()
            else:
                cmd.angular.z = 0.6 if diff > 0 else -0.6
                self._publish(cmd)
            return

    def _end_recovery(self) -> None:
        self.s.recovery_strategy += 1
        self.s.recovery_target_yaw = None
        self.s.phase = self.s.phase_before_recovery
        self._think(f'RECOVERY complete (strategy {(self.s.recovery_strategy - 1) % 3}) '
                    f'-> {self.s.phase.name}',
                    rule='state_machine.recovery_done', confidence=0.8)

    # ================================================================
    # Publishing + diagnostics
    # ================================================================
    def _publish(self, cmd: Twist) -> None:
        self.pub_cmd.publish(cmd)
        if abs(cmd.linear.x) + abs(cmd.angular.z) < 1e-3:
            if time.monotonic() - self._last_cmd_time > self.idle_s:
                nudge = Twist()
                nudge.angular.z = 0.3
                self.pub_cmd.publish(nudge)
                self._last_cmd_time = time.monotonic()
                self._think('idle watchdog nudge',
                            rule='state_machine.idle_watchdog', confidence=1.0)
        else:
            self._last_cmd_time = time.monotonic()

    def _think(self, hypothesis: str, rule: str, confidence: float,
               action: str = '', alternatives: str = '', extras: dict | None = None) -> None:
        self.s.seq += 1
        t = Thought()
        t.stamp = self.get_clock().now().to_msg()
        t.seq = self.s.seq
        t.phase = self.s.phase.name
        t.hypothesis = hypothesis
        t.action_chosen = action or 'cmd_vel'
        t.rule_applied = rule
        t.alt_considered = alternatives
        t.extras_json = json.dumps(extras or {})
        t.confidence = float(confidence)
        self.pub_thought.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
