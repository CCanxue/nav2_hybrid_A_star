#!/usr/bin/env python3
"""
Corrected Nav2 Metrics Collector with fixes for critical errors identified.
"""
import os
import csv
import math
import json
import threading
import queue
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple, Set
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rclpy.executors import MultiThreadedExecutor

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry  # Fixed: Odometry is in nav_msgs, not geometry_msgs
from action_msgs.msg import GoalStatusArray, GoalStatus
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

def _quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# collision monitor (optional)
try:
    from nav2_msgs.msg import CollisionMonitorState
    HAS_CM = True
    # Track collision states for edge counting
except ImportError:
    HAS_CM = False


def _time_msg_to_float_sec(t) -> float:
    """Convert ROS time message to seconds as float."""
    return float(t.sec) + float(t.nanosec) * 1e-9


def _uuid_to_hex(goal_id_msg) -> str:
    """Convert UUID message to hex string."""
    return bytes(goal_id_msg.uuid).hex()


@dataclass
class RunningStats:
    """Accumulate running statistics for navigation metrics."""
    n: int = 0
    sum_sq: float = 0.0
    max_abs: float = 0.0
    
    def push(self, x: float):
        """Add a new value to the running statistics."""
        ax = abs(x)
        self.n += 1
        self.sum_sq += x * x
        self.max_abs = max(self.max_abs, ax)

    def rms(self) -> float:
        """Calculate root mean square."""
        return math.sqrt(self.sum_sq / self.n) if self.n > 0 else 0.0


@dataclass
class TrialMetrics:
    """Store metrics for a single navigation trial."""
    goal_id: str = ""
    start_sec: float = 0.0
    end_sec: float = 0.0
    result: str = "UNKNOWN"

    # distance from TF (odom->base)
    last_xy: Optional[Tuple[float, float]] = None
    distance_m: float = 0.0
    start_x: float = float("nan")
    start_y: float = float("nan")
    start_yaw: float = float("nan")
    goal_x: float = float("nan")
    goal_y: float = float("nan")
    goal_yaw: float = float("nan")

    # cmd_vel stats (commanded velocity)
    cmd_vx_stats: RunningStats = field(default_factory=RunningStats)
    cmd_wz_stats: RunningStats = field(default_factory=RunningStats)
    cmd_ax_max: float = 0.0
    cmd_aw_max: float = 0.0
    _cmd_last_t: Optional[float] = None
    _cmd_last_vx: Optional[float] = None
    _cmd_last_wz: Optional[float] = None

    # odom stats (actual velocity from odometry)
    odom_vx_stats: RunningStats = field(default_factory=RunningStats)
    odom_wz_stats: RunningStats = field(default_factory=RunningStats)
    odom_ax_max: float = 0.0
    odom_aw_max: float = 0.0
    _odom_last_t: Optional[float] = None
    _odom_last_vx: Optional[float] = None
    _odom_last_wz: Optional[float] = None

    # BT counters
    bt_success: Dict[str, int] = field(default_factory=lambda: defaultdict(int))
    bt_failure: Dict[str, int] = field(default_factory=lambda: defaultdict(int))

    # collision monitor (optional) - track state changes
    cm_counts: Dict[str, int] = field(default_factory=lambda: defaultdict(int))
    cm_durations: Dict[str, float] = field(default_factory=lambda: defaultdict(float))

    # strictly-correct FSM state
    cm_last_action: int = 0
    cm_last_polygon: str = ""
    cm_last_change_sec: Optional[float] = None


    def duration(self) -> float:
        """Calculate duration of the trial."""
        return max(0.0, self.end_sec - self.start_sec)


class Nav2MetricsCollector(Node):
    """Enhanced ROS2 node for collecting navigation performance metrics with dual source support."""
    
    def __init__(self):
        super().__init__("nav2_metrics_collector")
        
        # Initialize parameters with validation
        self._declare_parameters()
        
        # Load parameter values with validation
        self._load_parameters()

        self.latest_amcl = None
        self.latest_goal = None
        self.latest_goal_t = None  # float sec

        
        # Initialize state
        self.current: Optional[TrialMetrics] = None
        self.active_goal_id: Optional[str] = None
        
        # Thread-safe queue for CSV writing (simplified approach without flush markers)
        self.csv_queue = queue.Queue()
        self.csv_writer_thread = threading.Thread(target=self._csv_writer_worker, daemon=True)
        self.csv_writer_thread.start()
        
        self._cm_lock = threading.Lock()
        # Initialize subscribers
        self._initialize_subscribers()
        
        # Log configuration
        self._log_configuration()

    def _cm_do_nothing(self) -> int:
        if HAS_CM and hasattr(CollisionMonitorState, "DO_NOTHING"):
            return int(CollisionMonitorState.DO_NOTHING)
        return 0

    def _cm_action_name(self, action_type: int) -> str:
        if not HAS_CM:
            return str(action_type)
        if hasattr(CollisionMonitorState, "DO_NOTHING") and action_type == CollisionMonitorState.DO_NOTHING:
            return "DO_NOTHING"
        if hasattr(CollisionMonitorState, "STOP") and action_type == CollisionMonitorState.STOP:
            return "STOP"
        if hasattr(CollisionMonitorState, "SLOWDOWN") and action_type == CollisionMonitorState.SLOWDOWN:
            return "SLOWDOWN"
        if hasattr(CollisionMonitorState, "APPROACH") and action_type == CollisionMonitorState.APPROACH:
            return "APPROACH"
        if hasattr(CollisionMonitorState, "LIMIT") and action_type == CollisionMonitorState.LIMIT:
            return "LIMIT"
        return f"UNKNOWN_{action_type}"

    def _cm_stamp_or_now(self, msg) -> float:
    # CollisionMonitorState 在不同版本字段可能不完全一致：尽量用 stamp，没有就用 now
        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            t = _time_msg_to_float_sec(msg.header.stamp)
            if t > 0.0:
                return t
        return self._now_sec()


    def _declare_parameters(self):
        """Declare all required parameters."""
        self.declare_parameter("output_csv", "/tmp/nav2_metrics.csv")
        self.declare_parameter("nav_status_topic", "/navigate_to_pose/_action/status")
        self.declare_parameter("bt_log_topic", "/behavior_tree_log")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("cmd_vel_type", "twist")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("use_odom_for_metrics", True)
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("collision_state_topic", "/collision_monitor_state")
        self.declare_parameter("bt_watch_nodes", [
            "ComputePathToPose",
            "FollowPath",
            "NavigateRecovery",
            "NavigateWithReplanning",
            "ClearGlobalCostmap-Context",
            "ClearLocalCostmap-Context",
            "ClearLocalCostmap-Subtree",
            "ClearGlobalCostmap-Subtree",
            "Spin",
            "BackUp",
            "Wait",
            "GoalUpdated",
            "RecoveryFallback",
            "RecoveryActions",
            "CPP", "SP", "FP"
        ])
        self.declare_parameter("enable_csv_buffering", True)  # Can be disabled via param
        self.declare_parameter("csv_flush_interval", 10.0)

    def _load_parameters(self):
        """Load and validate parameter values."""
        self.output_csv = self.get_parameter("output_csv").value
        self.nav_status_topic = self.get_parameter("nav_status_topic").value
        self.bt_log_topic = self.get_parameter("bt_log_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.cmd_vel_type = str(self.get_parameter("cmd_vel_type").value).lower()
        self.odom_topic = self.get_parameter("odom_topic").value
        self.use_odom_for_metrics = self.get_parameter("use_odom_for_metrics").value
        self.tf_topic = self.get_parameter("tf_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.collision_state_topic = self.get_parameter("collision_state_topic").value
        self.bt_watch_nodes = set(self.get_parameter("bt_watch_nodes").value)
        self.enable_csv_buffering = self.get_parameter("enable_csv_buffering").value
        self.csv_flush_interval = self.get_parameter("csv_flush_interval").value
        
        # Validate cmd_vel_type
        if self.cmd_vel_type not in ["twist", "twist_stamped"]:
            self.get_logger().warn(f"Invalid cmd_vel_type '{self.cmd_vel_type}', defaulting to 'twist'")
            self.cmd_vel_type = "twist"

    def _initialize_subscribers(self):
        """Initialize all ROS2 subscribers."""
        # Use custom QoS profile for better reliability
        custom_qos = QoSProfile(depth=10)
        
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_amcl_pose,
            QoSProfile(depth=5)
            )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self._on_goal_pose,
            QoSProfile(depth=5)
            )

        # Navigation status subscription
        self.nav_status_sub = self.create_subscription(
            GoalStatusArray, 
            self.nav_status_topic, 
            self._on_nav_status, 
            custom_qos
        )
        
        # Behavior tree log subscription
        self.bt_log_sub = self.create_subscription(
            BehaviorTreeLog, 
            self.bt_log_topic, 
            self._on_bt_log, 
            custom_qos
        )
        
        # TF subscription with sensor data QoS
        self.tf_sub = self.create_subscription(
            TFMessage, 
            self.tf_topic, 
            self._on_tf, 
            qos_profile_sensor_data
        )
        
        # Odom subscription
        self.odom_sub = self.create_subscription(
            Odometry, 
            self.odom_topic, 
            self._on_odom, 
            qos_profile_sensor_data
        )
        
        # Command velocity subscription based on type
        if self.cmd_vel_type == "twist":
            self.cmd_vel_sub = self.create_subscription(
                Twist, 
                self.cmd_vel_topic, 
                self._on_cmd_twist, 
                qos_profile_sensor_data
            )
        else:
            self.cmd_vel_sub = self.create_subscription(
                TwistStamped, 
                self.cmd_vel_topic, 
                self._on_cmd_twist_stamped, 
                qos_profile_sensor_data
            )

        # Collision monitor subscription (if available)
        if HAS_CM:
            self.cm_sub = self.create_subscription(
                CollisionMonitorState, 
                self.collision_state_topic, 
                self._on_cm, 
                custom_qos
            )

        # Timer for periodic CSV flush (only if buffering enabled)
        if self.enable_csv_buffering:
            self.flush_timer = self.create_timer(
                self.csv_flush_interval, 
                self._flush_csv_queue
            )
        
        # Ensure CSV header exists
        self._ensure_csv_header()

    def _log_configuration(self):
        """Log current configuration."""
        self.get_logger().info(f"CSV Output: {os.path.abspath(self.output_csv)}")
        self.get_logger().info(f"Nav Status Topic: {self.nav_status_topic}")
        self.get_logger().info(f"BT Log Topic: {self.bt_log_topic}")
        self.get_logger().info(f"TF Distance: {self.tf_topic} using {self.odom_frame} -> {self.base_frame}")
        self.get_logger().info(f"Cmd Vel: {self.cmd_vel_topic} ({self.cmd_vel_type})")
        self.get_logger().info(f"Odom Topic: {self.odom_topic}")
        self.get_logger().info(f"Use Odom for Metrics: {self.use_odom_for_metrics}")
        self.get_logger().info(f"Buffering Enabled: {self.enable_csv_buffering}")
        self.get_logger().info(f"Flush Interval: {self.csv_flush_interval}s")
        if HAS_CM:
            self.get_logger().info(f"Collision Monitor Topic: {self.collision_state_topic}")

    def _ensure_csv_header(self):
        """Ensure CSV file has proper header."""
        try:
            need_header = (not os.path.exists(self.output_csv)) or os.path.getsize(self.output_csv) == 0
            if not need_header:
                return
                
            os.makedirs(os.path.dirname(self.output_csv) or ".", exist_ok=True)
            with open(self.output_csv, "w", newline="", encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "goal_id",
                    "start_sec", "end_sec", "duration_sec",
                    "result",
                    "distance_m",
                    # Odom-based metrics (primary)
                    "odom_vx_rms", "odom_vx_maxabs",
                    "odom_wz_rms", "odom_wz_maxabs",
                    "odom_ax_maxabs", "odom_aw_maxabs",
                    # Cmd_vel-based metrics (secondary)
                    "cmd_vx_rms", "cmd_vx_maxabs",
                    "cmd_wz_rms", "cmd_wz_maxabs",
                    "cmd_ax_maxabs", "cmd_aw_maxabs",
                    "bt_success_json",
                    "bt_failure_json",
                    "cm_counts_json",
                    "cm_durations_json",
                    "start_x","start_y","start_yaw",
                    "goal_x","goal_y","goal_yaw",
                ])
        except IOError as e:
            self.get_logger().error(f"Failed to create CSV header: {e}")
    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        self.latest_amcl = msg

    def _on_goal_pose(self, msg: PoseStamped):
        self.latest_goal = msg
        t = _time_msg_to_float_sec(msg.header.stamp)
        self.latest_goal_t = t if t > 0.0 else self._now_sec()

    def _csv_writer_worker(self):
        """Background thread to write CSV data safely."""
        buffer = []
        while rclpy.ok():
            try:
                item = self.csv_queue.get(timeout=1.0)
                if item is None:  # Shutdown signal
                    # Write any remaining buffered items
                    if buffer:
                        self._write_csv_batch(buffer)
                        buffer.clear()
                    break
                buffer.append(item)
                
                # Write buffered items when buffer reaches threshold
                if len(buffer) >= 10:
                    self._write_csv_batch(buffer)
                    buffer.clear()
                    
            except queue.Empty:
                # Periodically write buffered items
                if buffer:
                    self._write_csv_batch(buffer)
                    buffer.clear()
        
        # Final flush on shutdown
        if buffer:
            self._write_csv_batch(buffer)

    def _write_csv_batch(self, batch):
        """Write a batch of records to CSV file."""
        try:
            with open(self.output_csv, "a", newline="", encoding='utf-8') as f:
                writer = csv.writer(f)
                for row in batch:
                    writer.writerow(row)
        except IOError as e:
            self.get_logger().error(f"Failed to write CSV batch: {e}")

    def _queue_csv_row(self, tm: TrialMetrics):
        """Queue a trial metric for CSV writing."""
        row = [
            tm.goal_id,
            tm.start_sec, tm.end_sec, tm.duration(),
            tm.result,
            tm.distance_m,
            # Primary metrics: Odom-based (actual robot motion)
            tm.odom_vx_stats.rms(), tm.odom_vx_stats.max_abs,
            tm.odom_wz_stats.rms(), tm.odom_wz_stats.max_abs,
            tm.odom_ax_max, tm.odom_aw_max,
            # Secondary metrics: Cmd_vel-based (intended motion)
            tm.cmd_vx_stats.rms(), tm.cmd_vx_stats.max_abs,
            tm.cmd_wz_stats.rms(), tm.cmd_wz_stats.max_abs,
            tm.cmd_ax_max, tm.cmd_aw_max,
            json.dumps(dict(tm.bt_success), ensure_ascii=False),
            json.dumps(dict(tm.bt_failure), ensure_ascii=False),
            json.dumps(dict(tm.cm_counts), ensure_ascii=False),
            json.dumps(dict(tm.cm_durations), ensure_ascii=False),
            tm.start_x, tm.start_y, tm.start_yaw,
            tm.goal_x, tm.goal_y, tm.goal_yaw
        ]
        
        if self.enable_csv_buffering:
            self.csv_queue.put(row)
        else:
            # Direct write without buffering
            try:
                with open(self.output_csv, "a", newline="", encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerow(row)
            except IOError as e:
                self.get_logger().error(f"Failed to write CSV row: {e}")

    def _flush_csv_queue(self):
        """Force flush the CSV queue."""
        # Simply trigger a write of any pending items
        pass  # We'll let the worker thread handle periodic writes

    def _now_sec(self) -> float:
        """Get current time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _norm(self, s: str) -> str:
        """Normalize frame ID by removing leading slash if present."""
        return s[1:] if s.startswith("/") else s

    def _start_trial(self, goal_id: str, start_sec: float):
        """Start a new navigation trial."""
        if self.current is not None:
            # Close previous trial if still active
            self._finish_trial("INTERRUPTED", self._now_sec())

        self.current = TrialMetrics(goal_id=goal_id, start_sec=start_sec)
        # 1) start from latest AMCL pose (map frame)
        if self.latest_amcl is not None:
            p = self.latest_amcl.pose.pose
            self.current.start_x = p.position.x
            self.current.start_y = p.position.y
            self.current.start_yaw = _quat_to_yaw(p.orientation)

        # 2) goal from /goal_pose (map frame)
        #    用时间戳做一个简单关联：goal 发布通常发生在 trial start 附近
        if self.latest_goal is not None and self.latest_goal_t is not None:
            if abs(self.latest_goal_t - start_sec) < 2.0:  # 2s 容差，够用
                p = self.latest_goal.pose
                self.current.goal_x = p.position.x
                self.current.goal_y = p.position.y
                self.current.goal_yaw = _quat_to_yaw(p.orientation)

        self.active_goal_id = goal_id
        self.get_logger().info(f"[TRIAL START] {goal_id} @ {start_sec:.3f}")

    def _finish_trial(self, result: str, end_sec: float):
        """Finish the current navigation trial."""
        if self.current is None:
            return
            
        self.current.end_sec = end_sec
        self.current.result = result
        
        # End any ongoing collision monitoring
        if HAS_CM:
            do_nothing = self._cm_do_nothing()
            with self._cm_lock:
                tm = self.current
                if tm is not None and tm.cm_last_change_sec is not None and tm.cm_last_action != do_nothing:
                    name = self._cm_action_name(tm.cm_last_action)
                    dt = max(0.0, end_sec - tm.cm_last_change_sec)
                    tm.cm_durations[name] += dt
                    if tm.cm_last_polygon:
                        tm.cm_durations[f"{name}@{tm.cm_last_polygon}"] += dt
        
        # Queue the trial for CSV writing
        self._queue_csv_row(self.current)
        
        # Show summary with both odom and cmd_vel stats
        self.get_logger().info(
            f"[TRIAL END] {self.current.goal_id} {result} "
            f"dur={self.current.duration():.3f}s dist={self.current.distance_m:.3f}m "
            f"odom_vx_rms={self.current.odom_vx_stats.rms():.3f}m/s "
            f"cmd_vx_rms={self.current.cmd_vx_stats.rms():.3f}m/s "
            f"bt_success={dict(self.current.bt_success)}"
        )
        
        self.current = None
        self.active_goal_id = None

    # Callback methods
    def _on_nav_status(self, msg: GoalStatusArray):
        """Handle navigation status updates."""
        try:
            # Find newest ACCEPTED/EXECUTING goal (by timestamp)
            chosen = None
            latest_stamp = 0.0
            for st in msg.status_list:
                if st.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                    stamp = _time_msg_to_float_sec(st.goal_info.stamp)
                    if stamp > latest_stamp:
                        chosen = st
                        latest_stamp = stamp

            if chosen is not None:
                gid = _uuid_to_hex(chosen.goal_info.goal_id)
                stamp = _time_msg_to_float_sec(chosen.goal_info.stamp)
                start_sec = stamp if stamp > 0.0 else self._now_sec()
                
                if self.active_goal_id is None:
                    self._start_trial(gid, start_sec)
                elif gid != self.active_goal_id:
                    # Preempted goal
                    self._start_trial(gid, start_sec)

            # Finish if active goal reached terminal state
            if self.active_goal_id is None:
                return
                
            for st in msg.status_list:
                gid = _uuid_to_hex(st.goal_info.goal_id)
                if gid != self.active_goal_id:
                    continue
                    
                if st.status == GoalStatus.STATUS_SUCCEEDED:
                    self._finish_trial("SUCCEEDED", self._now_sec())
                elif st.status == GoalStatus.STATUS_ABORTED:
                    self._finish_trial("ABORTED", self._now_sec())
                elif st.status == GoalStatus.STATUS_CANCELED:
                    self._finish_trial("CANCELED", self._now_sec())
                break
        except Exception as e:
            self.get_logger().error(f"Error processing nav status: {e}")

    def _on_bt_log(self, msg: BehaviorTreeLog):
        """Handle behavior tree log events."""
        if self.current is None:
            return
            
        try:
            for ev in msg.event_log:
                name = ev.node_name
                if name not in self.bt_watch_nodes:
                    continue
                    
                if ev.current_status == "SUCCESS":
                    self.current.bt_success[name] += 1
                elif ev.current_status == "FAILURE":
                    self.current.bt_failure[name] += 1
        except Exception as e:
            self.get_logger().error(f"Error processing BT log: {e}")

    def _on_tf(self, msg: TFMessage):
        """Handle TF transform messages to calculate distance traveled."""
        if self.current is None:
            return
            
        try:
            odom_f = self._norm(self.odom_frame)
            base_f = self._norm(self.base_frame)
            
            for t in msg.transforms:
                parent = self._norm(t.header.frame_id)
                child = self._norm(t.child_frame_id)
                
                if parent != odom_f or child != base_f:
                    continue
                    
                x = t.transform.translation.x
                y = t.transform.translation.y
                
                if self.current.last_xy is None:
                    self.current.last_xy = (x, y)
                    return
                    
                lx, ly = self.current.last_xy
                self.current.distance_m += math.hypot(x - lx, y - ly)
                self.current.last_xy = (x, y)
                return
        except Exception as e:
            self.get_logger().error(f"Error processing TF: {e}")

    def _process_velocity_stats(self, vx: float, wz: float, t_sec: float, 
                               vx_stats: RunningStats, wz_stats: RunningStats,
                               last_t_attr: str, last_vx_attr: str, last_wz_attr: str,
                               ax_max_attr: str, aw_max_attr: str):
        """Generic method to process velocity statistics."""
        vx_stats.push(vx)
        wz_stats.push(wz)

        last_t = getattr(self.current, last_t_attr)
        last_vx = getattr(self.current, last_vx_attr)
        last_wz = getattr(self.current, last_wz_attr)

        if last_t is not None:
            dt = t_sec - last_t
            if dt > 1e-4:
                ax = (vx - last_vx) / dt
                aw = (wz - last_wz) / dt
                setattr(self.current, ax_max_attr, max(getattr(self.current, ax_max_attr), abs(ax)))
                setattr(self.current, aw_max_attr, max(getattr(self.current, aw_max_attr), abs(aw)))
                
        setattr(self.current, last_t_attr, t_sec)
        setattr(self.current, last_vx_attr, vx)
        setattr(self.current, last_wz_attr, wz)

    def _push_cmd_vel_stats(self, vx: float, wz: float, t_sec: float):
        """Process command velocity for statistics."""
        if self.current is None:
            return
            
        try:
            self._process_velocity_stats(
                vx, wz, t_sec,
                self.current.cmd_vx_stats, self.current.cmd_wz_stats,
                '_cmd_last_t', '_cmd_last_vx', '_cmd_last_wz',
                'cmd_ax_max', 'cmd_aw_max'
            )
        except Exception as e:
            self.get_logger().error(f"Error processing command velocity: {e}")

    def _push_odom_vel_stats(self, vx: float, wz: float, t_sec: float):
        """Process odometry velocity for statistics."""
        if self.current is None:
            return
            
        try:
            self._process_velocity_stats(
                vx, wz, t_sec,
                self.current.odom_vx_stats, self.current.odom_wz_stats,
                '_odom_last_t', '_odom_last_vx', '_odom_last_wz',
                'odom_ax_max', 'odom_aw_max'
            )
        except Exception as e:
            self.get_logger().error(f"Error processing odometry velocity: {e}")

    def _on_cmd_twist(self, msg: Twist):
        """Handle Twist message."""
        self._push_cmd_vel_stats(msg.linear.x, msg.angular.z, self._now_sec())

    def _on_cmd_twist_stamped(self, msg: TwistStamped):
        """Handle TwistStamped message."""
        t = _time_msg_to_float_sec(msg.header.stamp)
        self._push_cmd_vel_stats(msg.twist.linear.x, msg.twist.angular.z, t if t > 0.0 else self._now_sec())

    def _on_odom(self, msg: Odometry):
        """Handle Odometry message to get actual robot velocity."""
        # Extract linear and angular velocities from odometry
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        wz = msg.twist.twist.angular.z  # Only care about z rotation
        
        # For navigation metrics, we typically care about forward speed
        # Calculate the effective forward velocity magnitude
        effective_vx = math.sqrt(vx**2 + vy**2) * (1 if vx >= 0 else -1)  # Preserve direction sign
        
        t = _time_msg_to_float_sec(msg.header.stamp)
        self._push_odom_vel_stats(effective_vx, wz, t if t > 0.0 else self._now_sec())

    def _on_cm(self, msg):

        """Strictly-correct collision monitor stats: edge count + duration with state transitions."""

        if self.current is None or not HAS_CM:
            return

        act = int(getattr(msg, "action_type", self._cm_do_nothing()))
        poly = str(getattr(msg, "polygon_name", "") or "")
        t_sec = self._cm_stamp_or_now(msg)
        do_nothing = self._cm_do_nothing()

        with self._cm_lock:
            tm = self.current
            if tm is None:
                return

        # init state
            if tm.cm_last_change_sec is None:
                tm.cm_last_action = act
                tm.cm_last_polygon = poly
                tm.cm_last_change_sec = t_sec

            # edge count: entering an active action at trial start
                if act != do_nothing:
                    name = self._cm_action_name(act)
                    tm.cm_counts[name] += 1
                    if poly:
                        tm.cm_counts[f"{name}@{poly}"] += 1
                return

        # no state change -> do nothing (prevents counting by publish frequency)
            if act == tm.cm_last_action and poly == tm.cm_last_polygon:
                return

        # 1) close previous segment duration (if it was active)
            if tm.cm_last_action != do_nothing:
                prev_name = self._cm_action_name(tm.cm_last_action)
                dt = max(0.0, t_sec - tm.cm_last_change_sec)
                tm.cm_durations[prev_name] += dt
                if tm.cm_last_polygon:
                    tm.cm_durations[f"{prev_name}@{tm.cm_last_polygon}"] += dt

        # 2) open new segment + edge count (if new state is active)
            if act != do_nothing:
                name = self._cm_action_name(act)
                tm.cm_counts[name] += 1
                if poly:
                    tm.cm_counts[f"{name}@{poly}"] += 1

        # 3) update FSM state
            tm.cm_last_action = act
            tm.cm_last_polygon = poly
            tm.cm_last_change_sec = t_sec


    def destroy_node(self):
        """Clean up resources before destroying node."""
        # Signal CSV writer thread to stop
        if hasattr(self, 'csv_queue'):
            self.csv_queue.put(None)
            self.csv_writer_thread.join(timeout=2.0)
        
        super().destroy_node()


def main():
    """Main entry point."""
    rclpy.init()
    
    try:
        node = Nav2MetricsCollector()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()