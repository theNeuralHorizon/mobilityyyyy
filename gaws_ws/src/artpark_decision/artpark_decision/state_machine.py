"""Top-level state machine.

Phases:
  INIT            - settle, bring sensors online
  EXPLORE         - PD right-wall-follow until tag detected
  APPROACH_TAG    - slow down, center on tag, wait for vote commit
  WAIT_OPENING    - drive forward until side opening found, then turn
  ACT_LEFT/RIGHT  - rotate +/-90deg
  ACT_UTURN       - rotate 180deg
  POST_ROTATE     - drive forward ~0.8 m after rotation to enter next tile
  GREEN_FOLLOW    - per-tile edge-sample -> exit toward green
  ORANGE_FOLLOW   - same but orange
  SEEK_TAG_5      - wall-follow + scan for AprilTags after green exhausts
  RECOVERY        - escape stuck situations (spin / reverse / escape)
  REACH_STOP      - solid-red tile or STOP pose -> final halt
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
    WAIT_OPENING = auto()
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
    entry_edge: Optional[str] = None
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
    color_zero_streak: int = 0
    color_ever_seen: bool = False

    # WAIT_OPENING: pending action to execute when opening found
    pending_action: Optional[str] = None
    # Compound actions: phase to enter after POST_ROTATE (set before rotation)
    pending_post_phase: Optional[Phase] = None


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
        self.declare_parameter('red_stop_threshold', 8000)
        # Tag execution distance (only act when this close to tag)
        self.declare_parameter('tag_execute_dist_m', 0.80)

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
        self.tag_execute_dist = float(self.get_parameter('tag_execute_dist_m').value)

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
        self._heartbeat_timer = self.create_timer(2.0, self._heartbeat)
        self._prev_phase_logged = None

        self._octants = [float('inf')] * 8
        self._octants_received = False
        self._init_start = time.monotonic()
        self.get_logger().info('state_machine up, entering INIT')

    # ================================================================
    # callbacks
    # ================================================================
    def _on_tag(self, msg: TagEvent) -> None:
        # Skip already-executed tags
        if msg.tag_id in self.s.tags_logged and not msg.first_sighting:
            return

        action = msg.decision

        # Distance gate: only execute when close enough to the tag.
        # Don't add to tags_logged yet — keep re-processing until close.
        if msg.distance > self.tag_execute_dist:
            self._think(
                f'Tag {msg.tag_id} seen at {msg.distance:.2f}m (> {self.tag_execute_dist:.2f}m), '
                f'approaching closer before acting',
                rule='state_machine.tag_distance_wait', confidence=0.8)
            return

        # Mark as executed
        self.s.tags_logged.add(msg.tag_id)

        # Emit scorecard row
        row = {
            'type': 'TAG_LOG',
            'tag_id': msg.tag_id,
            'label': msg.logical_label,
            'decision': msg.decision,
            'distance': msg.distance,
            'bearing': msg.bearing,
        }
        self.pub_score.publish(String(data=json.dumps(row)))

        self._think(f'TagEvent committed: label={msg.logical_label}, action={action}, '
                    f'dist={msg.distance:.2f}m',
                    rule='state_machine.on_tag_commit', confidence=1.0)

        # Execute the tag action unconditionally — tag instructions are law.
        if action == 'LEFT':
            self.s.pending_action = 'LEFT'
            self.s.pending_post_phase = None
            self.s.phase = Phase.WAIT_OPENING
        elif action == 'RIGHT':
            self.s.pending_action = 'RIGHT'
            self.s.pending_post_phase = None
            self.s.phase = Phase.WAIT_OPENING
        elif action == 'RIGHT_GREEN':
            self.s.pending_action = 'RIGHT'
            self.s.pending_post_phase = Phase.GREEN_FOLLOW
            self.s.phase = Phase.WAIT_OPENING
        elif action == 'LEFT_ORANGE':
            self.s.pending_action = 'LEFT'
            self.s.pending_post_phase = Phase.ORANGE_FOLLOW
            self.s.phase = Phase.WAIT_OPENING
        elif action == 'U_TURN':
            self.s.pre_uturn_entry = self.s.entry_edge
            self._start_rotation(math.pi, Phase.ACT_UTURN)

    def _on_edge(self, msg: EdgeSample) -> None:
        self.s.last_edge_sample = msg

        # Red STOP detection — only active during ORANGE_FOLLOW and only
        # after a grace period.
        if self.s.phase == Phase.ORANGE_FOLLOW:
            if msg.red_total > self.red_stop_threshold:
                elapsed = time.monotonic() - self.s.color_follow_start_time
                cf_dist = self._color_follow_dist()
                if elapsed > self.color_grace_time and cf_dist > self.color_grace_dist:
                    dist = math.hypot(self.s.odom_x, self.s.odom_y)
                    if dist > 1.0:
                        self.s.phase = Phase.REACH_STOP
                        self.pub_score.publish(String(data=json.dumps({'type': 'STOP_REACHED'})))
                        self._think(f'RED STOP detected: red_total={msg.red_total}, '
                                    f'dist={dist:.2f}m, cf_dist={cf_dist:.2f}m, elapsed={elapsed:.1f}s',
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
        if not self._octants_received:
            any_finite = any(math.isfinite(v) and v < 50.0 for v in self._octants)
            if any_finite:
                self._octants_received = True

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
            f'pending={self.s.pending_action} | '
            f'oct_fwd={self._oct(4):.2f} oct_right={self._oct(2):.2f} '
            f'oct_left={self._oct(6):.2f}')

    def _oct(self, idx: int) -> float:
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
                  Phase.SEEK_TAG_5, Phase.POST_ROTATE, Phase.WAIT_OPENING):
            if self._check_stuck(now):
                return

        # ---- Phase dispatch ----
        if ph == Phase.INIT:
            if self.debug_hold:
                self._publish(cmd)
                return
            elapsed = time.monotonic() - self._init_start
            # Wait until LiDAR octants arrive (perception pipeline ready)
            # or a 15s safety cap so we don't wait forever.
            ready = self._octants_received and elapsed > 2.0
            timed_out = elapsed > 15.0
            if ready or timed_out:
                # Clear stuck-detection history so EXPLORE starts fresh
                self.s.position_history.clear()
                self.s.last_history_time = time.monotonic()
                self.s.phase = Phase.EXPLORE
                self._think(
                    f'INIT -> EXPLORE ({"perception ready" if ready else "15s cap"}, '
                    f'waited {elapsed:.1f}s)',
                    rule='state_machine.init_timeout', confidence=1.0)
            self._publish(cmd)
            return

        if ph == Phase.EXPLORE:
            self._explore_wall_follow(cmd)
            return

        if ph == Phase.APPROACH_TAG:
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

        if ph == Phase.WAIT_OPENING:
            self._wait_for_opening(cmd)
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
    # WAIT_OPENING: drive forward until the side opens up, then turn
    # ================================================================
    def _wait_for_opening(self, cmd: Twist) -> None:
        """Drive forward until an opening appears on the pending turn side,
        then execute the rotation. This ensures the robot reaches the
        junction before turning."""
        action = self.s.pending_action
        if action == 'RIGHT':
            side = self._oct(2)
            side_fwd = self._oct(3)
            label = 'right'
        else:  # LEFT
            side = self._oct(6)
            side_fwd = self._oct(5)
            label = 'left'

        # Opening detected: side AND side-forward are both clear
        if side > self.opening_thresh and side_fwd > self.opening_thresh:
            self._think(
                f'WAIT_OPENING: {label} opening found (side={side:.2f}m, '
                f'side_fwd={side_fwd:.2f}m) -> rotating',
                rule='state_machine.opening_found', confidence=1.0)
            if action == 'RIGHT':
                self._start_rotation(-math.pi / 2, Phase.ACT_RIGHT)
            else:
                self._start_rotation(+math.pi / 2, Phase.ACT_LEFT)
            return

        # No opening yet — drive forward with obstacle avoidance
        fwd = self._oct(4)
        if fwd < self.emergency_front:
            cmd.angular.z = self.w_turn
            self._publish(cmd)
            return

        if fwd < self.slow_front:
            cmd.linear.x = 0.08
            cmd.angular.z = self.w_turn * 0.5
            self._publish(cmd)
            return

        cmd.linear.x = self.v_exp
        self._publish(cmd)

    # ================================================================
    # EXPLORE: PD right-wall-following
    # ================================================================
    def _explore_wall_follow(self, cmd: Twist) -> None:
        fwd = self._oct(4)
        fwd_right = self._oct(3)
        right = self._oct(2)

        if fwd < self.emergency_front:
            cmd.angular.z = self.w_turn
            self._publish(cmd)
            return

        if fwd < self.slow_front:
            cmd.linear.x = 0.08
            cmd.angular.z = self.w_turn * 0.6
            self._publish(cmd)
            return

        if fwd_right < 0.30:
            cmd.linear.x = self.v_exp * 0.7
            cmd.angular.z = self.w_turn * 0.4
            self._publish(cmd)
            return

        if right > self.opening_thresh and fwd_right > self.opening_thresh:
            cmd.linear.x = self.v_exp
            cmd.angular.z = -0.3
            self._publish(cmd)
            return

        if right < 2.0:
            error = right - self.wall_target
            d_error = error - self.s.wall_error_prev
            self.s.wall_error_prev = error
            cmd.linear.x = self.v_exp
            cmd.angular.z = -(self.wall_kp * error + self.wall_kd * d_error)
            cmd.angular.z = max(-self.w_turn, min(self.w_turn, cmd.angular.z))
            self._publish(cmd)
            return

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
            self.s.phase = Phase.EXPLORE
            return

        diff = math.atan2(
            math.sin(self.s.rotation_target_yaw - self.s.current_yaw),
            math.cos(self.s.rotation_target_yaw - self.s.current_yaw),
        )

        if abs(diff) < math.radians(5):
            # Rotation complete -> POST_ROTATE
            self.s.rotation_target_yaw = None
            self.s.post_rotate_start_x = self.s.odom_x
            self.s.post_rotate_start_y = self.s.odom_y

            # Determine what to do after driving forward
            if self.s.pending_post_phase is not None:
                # Compound action: turn was part of RIGHT_GREEN or LEFT_ORANGE
                self.s.post_rotate_target = self.s.pending_post_phase
                self.s.pending_post_phase = None
            elif self.s.phase == Phase.ACT_UTURN:
                if self.s.pre_uturn_entry:
                    self.s.entry_edge = OPPOSITE.get(self.s.pre_uturn_entry, '')
                else:
                    self.s.entry_edge = None
                self.s.pre_uturn_entry = None
                self.s.post_rotate_target = Phase.EXPLORE
            else:
                self.s.entry_edge = None
                self.s.post_rotate_target = Phase.EXPLORE

            self.s.pending_action = None
            self.s.phase = Phase.POST_ROTATE
            self._think(f'rotation complete -> POST_ROTATE (target={self.s.post_rotate_target.name})',
                        rule='state_machine.rotation_done', confidence=1.0)
            return

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

        if dist >= self.post_rotate_dist or self.s.obstacle_near:
            target_phase = self.s.post_rotate_target
            if target_phase in (Phase.GREEN_FOLLOW, Phase.ORANGE_FOLLOW):
                self._enter_color_follow(target_phase)
            else:
                self.s.phase = target_phase
            self._think(f'POST_ROTATE done (dist={dist:.2f}m) -> {target_phase.name}',
                        rule='state_machine.post_rotate_done', confidence=1.0)
            self._publish(Twist())
            return

        fwd = self._oct(4)
        if fwd < self.emergency_front:
            target_phase = self.s.post_rotate_target
            if target_phase in (Phase.GREEN_FOLLOW, Phase.ORANGE_FOLLOW):
                self._enter_color_follow(target_phase)
            else:
                self.s.phase = target_phase
            self._think(f'POST_ROTATE wall ahead ({fwd:.2f}m) -> {target_phase.name}',
                        rule='state_machine.post_rotate_wall', confidence=0.9)
            self._publish(Twist())
            return

        cmd.linear.x = self.v_exp
        self._publish(cmd)

    # ================================================================
    # Color following (robot-frame) with grace period
    # ================================================================
    def _enter_color_follow(self, phase: Phase) -> None:
        self.s.phase = phase
        self.s.entry_edge = None
        self.s.color_follow_start_x = self.s.odom_x
        self.s.color_follow_start_y = self.s.odom_y
        self.s.color_follow_start_time = time.monotonic()
        self.s.color_zero_streak = 0
        self.s.color_ever_seen = False

    def _color_follow_dist(self) -> float:
        return math.hypot(
            self.s.odom_x - self.s.color_follow_start_x,
            self.s.odom_y - self.s.color_follow_start_y,
        )

    def _follow_color(self, cmd: Twist, color: str) -> None:
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
        walls = self._walls_from_octants()

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

        if total_color > 30:
            self.s.color_ever_seen = True
            self.s.color_zero_streak = 0
        else:
            self.s.color_zero_streak += 1

        if not decision.chosen:
            dist = self._color_follow_dist()
            elapsed = time.monotonic() - self.s.color_follow_start_time
            grace_ok = (dist >= self.color_grace_dist and
                        elapsed >= self.color_grace_time)
            streak_ok = self.s.color_zero_streak >= self.color_exhaust_streak

            if grace_ok and streak_ok:
                if color == 'green':
                    self.s.phase = Phase.SEEK_TAG_5
                    self._think(f'green arc exhausted (dist={dist:.2f}m) -> SEEK_TAG_5',
                                rule='state_machine.green_exhausted', confidence=0.7)
                else:
                    self.s.phase = Phase.REACH_STOP
                    self.pub_score.publish(String(data=json.dumps({'type': 'STOP_REACHED'})))
                    self._think(f'orange exhausted (dist={dist:.2f}m) -> REACH_STOP',
                                rule='state_machine.orange_exhausted', confidence=0.8)
                self._publish(Twist())
                return

            cmd.linear.x = self.v_fol
            self._publish(cmd)
            return

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
            cmd.angular.z = 0.6 * diff

        self._publish(cmd)

    def _walls_from_octants(self) -> Dict[str, bool]:
        def blocked(idx: int) -> bool:
            v = self._oct(idx)
            return 0 < v < 0.45

        return {
            'N': blocked(4),
            'E': blocked(2),
            'S': blocked(0),
            'W': blocked(6),
        }

    # ================================================================
    # Stuck detection + Recovery
    # ================================================================
    def _check_stuck(self, now: float) -> bool:
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
