#!/usr/bin/env python3
import os
import csv
import math
import json
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist, TwistStamped
from action_msgs.msg import GoalStatusArray
from action_msgs.msg import GoalStatus
from nav2_msgs.msg import BehaviorTreeLog

# collision monitor (optional)
try:
    from nav2_msgs.msg import CollisionMonitorState
    HAS_CM = True
except Exception:
    HAS_CM = False


def _time_msg_to_float_sec(t) -> float:
    return float(t.sec) + float(t.nanosec) * 1e-9


def _uuid_to_hex(goal_id_msg) -> str:
    return bytes(goal_id_msg.uuid).hex()


@dataclass
class RunningStats:
    n: int = 0
    sum_sq: float = 0.0
    max_abs: float = 0.0

    def push(self, x: float):
        ax = abs(x)
        self.n += 1
        self.sum_sq += x * x
        self.max_abs = max(self.max_abs, ax)

    def rms(self) -> float:
        return math.sqrt(self.sum_sq / self.n) if self.n > 0 else 0.0


@dataclass
class TrialMetrics:
    goal_id: str = ""
    start_sec: float = 0.0
    end_sec: float = 0.0
    result: str = "UNKNOWN"

    # distance from TF (odom->base)
    last_xy: Optional[Tuple[float, float]] = None
    distance_m: float = 0.0

    # cmd_vel stats
    vx_stats: RunningStats = field(default_factory=RunningStats)
    wz_stats: RunningStats = field(default_factory=RunningStats)
    ax_max: float = 0.0
    aw_max: float = 0.0
    _last_cmd_t: Optional[float] = None
    _last_vx: Optional[float] = None
    _last_wz: Optional[float] = None

    # BT counters
    bt_success: Dict[str, int] = field(default_factory=dict)
    bt_failure: Dict[str, int] = field(default_factory=dict)

    # collision monitor (optional)
    cm_counts: Dict[str, int] = field(default_factory=dict)

    def duration(self) -> float:
        return max(0.0, self.end_sec - self.start_sec)


class Nav2MetricsCollector(Node):
    def __init__(self):
        super().__init__("nav2_metrics_collector")

        # ------------ params ------------
        self.declare_parameter("output_csv", "/tmp/nav2_metrics.csv")

        # Action status topic (full path, no guessing)
        self.declare_parameter("nav_status_topic", "/navigate_to_pose/_action/status")

        # BT log
        self.declare_parameter("bt_log_topic", "/behavior_tree_log")

        # cmd_vel
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("cmd_vel_type", "twist_stamped")  # twist | twist_stamped

        # TF distance
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")  # or base_footprint

        # collision monitor (optional)
        self.declare_parameter("collision_state_topic", "/collision_monitor_state")

        # BT nodes to watch (match your XML names)
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
            # If you later add explicit name=CPP/SP/FP, put them here too:
            "CPP", "SP", "FP"
        ])

        self.output_csv = self.get_parameter("output_csv").value
        self.nav_status_topic = self.get_parameter("nav_status_topic").value
        self.bt_log_topic = self.get_parameter("bt_log_topic").value

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.cmd_vel_type = str(self.get_parameter("cmd_vel_type").value).lower()

        self.tf_topic = self.get_parameter("tf_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.collision_state_topic = self.get_parameter("collision_state_topic").value
        self.bt_watch_nodes = set(self.get_parameter("bt_watch_nodes").value)

        # ------------ state ------------
        self.current: Optional[TrialMetrics] = None
        self.active_goal_id: Optional[str] = None

        # ------------ subs ------------
        self.create_subscription(GoalStatusArray, self.nav_status_topic, self._on_nav_status, 10)
        self.create_subscription(BehaviorTreeLog, self.bt_log_topic, self._on_bt_log, 10)
        self.create_subscription(TFMessage, self.tf_topic, self._on_tf, qos_profile_sensor_data)

        if self.cmd_vel_type == "twist":
            self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_twist, qos_profile_sensor_data)
        else:
            self.create_subscription(TwistStamped, self.cmd_vel_topic, self._on_cmd_twist_stamped, qos_profile_sensor_data)

        if HAS_CM:
            self.create_subscription(CollisionMonitorState, self.collision_state_topic, self._on_cm, 10)

        self._ensure_csv_header()

        self.get_logger().info(f"CSV: {os.path.abspath(self.output_csv)}")
        self.get_logger().info(f"Nav status: {self.nav_status_topic}")
        self.get_logger().info(f"BT log: {self.bt_log_topic}")
        self.get_logger().info(f"TF distance: {self.tf_topic} using {self.odom_frame} -> {self.base_frame}")
        self.get_logger().info(f"cmd_vel: {self.cmd_vel_topic} ({self.cmd_vel_type})")
        if HAS_CM:
            self.get_logger().info(f"collision monitor topic (optional): {self.collision_state_topic}")

    # ---------- helpers ----------
    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _norm(self, s: str) -> str:
        return s[1:] if s.startswith("/") else s

    def _ensure_csv_header(self):
        need = (not os.path.exists(self.output_csv)) or os.path.getsize(self.output_csv) == 0
        if not need:
            return
        os.makedirs(os.path.dirname(self.output_csv) or ".", exist_ok=True)
        with open(self.output_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "goal_id",
                "start_sec", "end_sec", "duration_sec",
                "result",
                "distance_m",
                "vx_rms", "vx_maxabs",
                "wz_rms", "wz_maxabs",
                "ax_maxabs", "aw_maxabs",
                "bt_success_json",
                "bt_failure_json",
                "cm_counts_json",
            ])

    def _append_trial(self, tm: TrialMetrics):
        with open(self.output_csv, "a", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                tm.goal_id,
                tm.start_sec, tm.end_sec, tm.duration(),
                tm.result,
                tm.distance_m,
                tm.vx_stats.rms(), tm.vx_stats.max_abs,
                tm.wz_stats.rms(), tm.wz_stats.max_abs,
                tm.ax_max, tm.aw_max,
                json.dumps(tm.bt_success, ensure_ascii=False),
                json.dumps(tm.bt_failure, ensure_ascii=False),
                json.dumps(tm.cm_counts, ensure_ascii=False),
            ])

    def _start_trial(self, goal_id: str, start_sec: float):
        if self.current is not None:
            # close previous
            self._finish_trial("INTERRUPTED", self._now_sec())

        self.current = TrialMetrics(goal_id=goal_id, start_sec=start_sec)
        self.active_goal_id = goal_id
        self.get_logger().info(f"[TRIAL START] {goal_id} @ {start_sec:.3f}")

    def _finish_trial(self, result: str, end_sec: float):
        if self.current is None:
            return
        self.current.end_sec = end_sec
        self.current.result = result
        self._append_trial(self.current)
        self.get_logger().info(
            f"[TRIAL END] {self.current.goal_id} {result} "
            f"dur={self.current.duration():.3f}s dist={self.current.distance_m:.3f}m "
            f"bt_success={self.current.bt_success}"
        )
        self.current = None
        self.active_goal_id = None

    # ---------- callbacks ----------
    def _on_nav_status(self, msg: GoalStatusArray):
        """
        Trial boundary from /navigate_to_pose/_action/status.
        """
        # Find newest ACCEPTED/EXECUTING goal
        chosen = None
        for st in msg.status_list:
            if st.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                chosen = st  # keep last; list is usually small

        if chosen is not None:
            gid = _uuid_to_hex(chosen.goal_info.goal_id)
            stamp = _time_msg_to_float_sec(chosen.goal_info.stamp)
            start_sec = stamp if stamp > 0.0 else self._now_sec()
            if self.active_goal_id is None:
                self._start_trial(gid, start_sec)
            elif gid != self.active_goal_id:
                # preempted goal
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

    def _on_bt_log(self, msg: BehaviorTreeLog):
        if self.current is None:
            return
        for ev in msg.event_log:
            name = ev.node_name
            if name not in self.bt_watch_nodes:
                continue
            if ev.current_status == "SUCCESS":
                self.current.bt_success[name] = self.current.bt_success.get(name, 0) + 1
            elif ev.current_status == "FAILURE":
                self.current.bt_failure[name] = self.current.bt_failure.get(name, 0) + 1

    def _on_tf(self, msg: TFMessage):
        if self.current is None:
            return
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

    def _push_cmd(self, vx: float, wz: float, t_sec: float):
        if self.current is None:
            return
        tm = self.current
        tm.vx_stats.push(vx)
        tm.wz_stats.push(wz)

        if tm._last_cmd_t is not None:
            dt = t_sec - tm._last_cmd_t
            if dt > 1e-4:
                ax = (vx - tm._last_vx) / dt
                aw = (wz - tm._last_wz) / dt
                tm.ax_max = max(tm.ax_max, abs(ax))
                tm.aw_max = max(tm.aw_max, abs(aw))
        tm._last_cmd_t = t_sec
        tm._last_vx = vx
        tm._last_wz = wz

    def _on_cmd_twist(self, msg: Twist):
        self._push_cmd(msg.linear.x, msg.angular.z, self._now_sec())

    def _on_cmd_twist_stamped(self, msg: TwistStamped):
        t = _time_msg_to_float_sec(msg.header.stamp)
        self._push_cmd(msg.twist.linear.x, msg.twist.angular.z, t if t > 0.0 else self._now_sec())

    def _on_cm(self, msg):
        if self.current is None:
            return
        # count non-do-nothing actions
        if getattr(msg, "action_type", 0) == 0:
            return
        # best-effort label
        act = str(getattr(msg, "action_type"))
        self.current.cm_counts[act] = self.current.cm_counts.get(act, 0) + 1


def main():
    rclpy.init()
    node = Nav2MetricsCollector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
