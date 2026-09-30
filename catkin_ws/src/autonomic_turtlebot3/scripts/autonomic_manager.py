#!/usr/bin/env python3
"""MAPE-K mission coordinator for the autonomic TurtleBot3 lift truck."""

import json
import math
import os
import threading
from typing import Any, Dict, List, Optional, Tuple

import actionlib
import rospy
import tf2_ros
from rospy.timer import TimerEvent
import yaml
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler

from autonomic_turtlebot3.policies import StuckDetector, target_speed


class AutonomicManager:
    VALID_LIFT_COMMANDS = {"up", "down", "stop"}

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._state = "IDLE"
        self._queued_orders = 0
        self._battery_fraction: Optional[float] = None
        self._odom_xy: Optional[Tuple[float, float]] = None
        self._localized_xy: Optional[Tuple[float, float]] = None
        self._commanded_speed = 0.0
        self._actions: List[Dict[str, Any]] = []
        self._action_index = 0
        self._action_started_at: Optional[rospy.Time] = None
        self._goal_active = False
        self._mission_frame = "map"
        self._mission_calibrated = False
        self._lift_state = "unknown"
        self._assembly_ready = False
        self._last_motion_event = rospy.Time(0)

        self._base_speed = rospy.get_param("~base_speed", 0.02)
        self._speed_per_order = rospy.get_param("~speed_per_order", 0.02)
        self._max_orders = rospy.get_param("~max_orders", 11)
        self._low_battery_fraction = rospy.get_param("~low_battery_fraction", 0.15)
        self._auto_start = rospy.get_param("~auto_start_on_orders", False)
        self._base_frame = rospy.get_param("~base_frame", "base_footprint")
        self._map_frame = rospy.get_param("~map_frame", "map")

        self._detector = StuckDetector(
            window_seconds=rospy.get_param("~stuck/window_seconds", 3.0),
            min_command_speed=rospy.get_param("~stuck/min_command_speed", 0.03),
            min_odom_motion=rospy.get_param("~stuck/min_odom_motion", 0.08),
            max_localized_motion=rospy.get_param("~stuck/max_localized_motion", 0.025),
        )

        self._state_pub = rospy.Publisher("/autonomic/state", String, queue_size=1, latch=True)
        self._alert_pub = rospy.Publisher("/autonomic/alert", String, queue_size=10, latch=True)
        self._event_pub = rospy.Publisher("/autonomic/mapek_event", String, queue_size=100)
        self._speed_pub = rospy.Publisher("/autonomic/target_speed", Float32, queue_size=1, latch=True)
        self._lift_pub = rospy.Publisher("/lift/command", String, queue_size=5)
        self._completed_pub = rospy.Publisher("/autonomic/completed_cycles", Int32, queue_size=1, latch=True)
        self._completed_cycles = 0

        rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=20)
        rospy.Subscriber("/cmd_vel", Twist, self._velocity_callback, queue_size=20)
        rospy.Subscriber("/factory/queued_orders", Int32, self._orders_callback, queue_size=10)
        rospy.Subscriber("/battery_state", BatteryState, self._battery_callback, queue_size=10)
        rospy.Subscriber("/lift/state", String, self._lift_state_callback, queue_size=10)
        rospy.Subscriber(
            "/factory/assembly_ready", Bool, self._assembly_ready_callback, queue_size=10
        )

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.Service("~start_mission", Trigger, self._start_service)
        rospy.Service("~cancel_mission", Trigger, self._cancel_service)
        rospy.Service("~acknowledge_recovery", Trigger, self._recovery_service)

        self._load_mission(rospy.get_param("~mission_file", ""))
        self._publish_state("IDLE")
        self._publish_alert("")
        self._update_target_speed()
        self._timer = rospy.Timer(rospy.Duration(0.1), self._tick)
        rospy.loginfo("Autonomic manager ready with %d mission actions", len(self._actions))

    def _load_mission(self, path: str) -> None:
        if not path:
            rospy.logwarn("No mission_file supplied; navigation remains disabled until configured")
            return
        expanded = os.path.abspath(os.path.expanduser(path))
        with open(expanded, "r", encoding="utf-8") as stream:
            mission = yaml.safe_load(stream) or {}
        actions = mission.get("actions", [])
        if not isinstance(actions, list) or not actions:
            raise ValueError("mission file must contain a non-empty 'actions' list")
        for index, action in enumerate(actions):
            if not isinstance(action, dict) or action.get("type") not in {
                "navigate",
                "lift",
                "wait",
                "wait_for_assembly",
            }:
                raise ValueError(f"mission action {index} has an unsupported type")
            if action["type"] == "navigate" and not all(key in action for key in ("x", "y", "yaw")):
                raise ValueError(f"navigate action {index} requires x, y, and yaw")
            if action["type"] == "lift" and action.get("command") not in self.VALID_LIFT_COMMANDS:
                raise ValueError(f"lift action {index} requires up, down, or stop")
        self._mission_frame = str(mission.get("frame_id", self._map_frame))
        self._mission_calibrated = bool(mission.get("calibrated", False))
        self._actions = actions
        rospy.loginfo("Loaded mission %s (calibrated=%s)", expanded, self._mission_calibrated)

    def _publish_state(self, state: str) -> None:
        self._state = state
        self._state_pub.publish(String(data=state))

    def _publish_alert(self, message: str) -> None:
        self._alert_pub.publish(String(data=message))

    def _event(self, phase: str, capability: str, action: str, **details: Any) -> None:
        message = {
            "stamp": rospy.Time.now().to_sec(),
            "phase": phase,
            "capability": capability,
            "action": action,
            "details": details,
        }
        self._event_pub.publish(String(data=json.dumps(message, separators=(",", ":"), sort_keys=True)))

    def _odom_callback(self, message: Odometry) -> None:
        with self._lock:
            self._odom_xy = (message.pose.pose.position.x, message.pose.pose.position.y)

    def _velocity_callback(self, message: Twist) -> None:
        with self._lock:
            self._commanded_speed = math.hypot(message.linear.x, message.linear.y)

    def _orders_callback(self, message: Int32) -> None:
        with self._lock:
            self._queued_orders = max(0, int(message.data))
            self._event("MONITOR", "SELF_OPTIMIZATION", "queued_orders", value=self._queued_orders)
            self._update_target_speed()
            should_start = self._auto_start and self._queued_orders > 0 and self._state == "IDLE"
        if should_start:
            self._start_mission()

    def _battery_callback(self, message: BatteryState) -> None:
        with self._lock:
            fraction = float(message.percentage)
            self._battery_fraction = fraction if math.isfinite(fraction) and fraction >= 0.0 else None
            self._update_target_speed()

    def _lift_state_callback(self, message: String) -> None:
        with self._lock:
            self._lift_state = message.data.strip().lower()

    def _assembly_ready_callback(self, message: Bool) -> None:
        with self._lock:
            self._assembly_ready = bool(message.data)
            self._event(
                "MONITOR",
                "SELF_CONFIGURATION",
                "assembly_ready",
                value=self._assembly_ready,
            )

    def _update_target_speed(self) -> None:
        speed = target_speed(
            self._queued_orders,
            base_speed=self._base_speed,
            speed_per_order=self._speed_per_order,
            max_orders=self._max_orders,
            battery_fraction=self._battery_fraction,
            low_battery_fraction=self._low_battery_fraction,
        )
        self._speed_pub.publish(Float32(data=speed))
        self._event(
            "PLAN",
            "SELF_OPTIMIZATION",
            "set_target_speed",
            queued_orders=self._queued_orders,
            battery_fraction=self._battery_fraction,
            target_speed=speed,
        )

    def _update_localized_pose(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rospy.Time(0), rospy.Duration(0.02)
            )
            self._localized_xy = (
                transform.transform.translation.x,
                transform.transform.translation.y,
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def _check_stuck(self, now: rospy.Time) -> None:
        if self._state != "RUNNING" or self._odom_xy is None or self._localized_xy is None:
            return
        if (now - self._last_motion_event).to_sec() >= 1.0:
            self._event(
                "MONITOR",
                "SELF_HEALING",
                "motion_sample",
                commanded_speed=self._commanded_speed,
            )
            self._last_motion_event = now
        if self._detector.update(
            now.to_sec(), self._commanded_speed, self._odom_xy, self._localized_xy
        ):
            self._event("ANALYZE", "SELF_HEALING", "stuck_detected")
            self._move_base.cancel_all_goals()
            self._goal_active = False
            self._lift_pub.publish(String(data="stop"))
            self._publish_state("STUCK")
            self._publish_alert("Mobility fault: operator assistance required")
            self._event("EXECUTE", "SELF_HEALING", "operator_alert")

    def _start_service(self, _request: Trigger) -> TriggerResponse:
        ok, message = self._start_mission()
        return TriggerResponse(success=ok, message=message)

    def _start_mission(self) -> Tuple[bool, str]:
        with self._lock:
            if not self._actions:
                return False, "No calibrated mission is loaded"
            if not self._mission_calibrated:
                return False, "Mission poses have not been calibrated for this map"
            if self._state not in {"IDLE", "COMPLETE"}:
                return False, f"Mission cannot start while state is {self._state}"
            if not self._move_base.wait_for_server(rospy.Duration(2.0)):
                return False, "move_base is not available"
            self._action_index = 0
            self._action_started_at = None
            self._goal_active = False
            self._detector.reset()
            self._publish_alert("")
            self._publish_state("RUNNING")
            self._event("PLAN", "SELF_CONFIGURATION", "mission_started")
            return True, "Mission started"

    def _cancel_service(self, _request: Trigger) -> TriggerResponse:
        with self._lock:
            self._move_base.cancel_all_goals()
            self._lift_pub.publish(String(data="stop"))
            self._goal_active = False
            self._action_started_at = None
            self._detector.reset()
            self._publish_state("IDLE")
            self._publish_alert("")
            self._event("EXECUTE", "SELF_PROTECTION", "mission_cancelled")
        return TriggerResponse(success=True, message="Mission cancelled and actuators stopped")

    def _recovery_service(self, _request: Trigger) -> TriggerResponse:
        with self._lock:
            if self._state != "STUCK":
                return TriggerResponse(success=False, message="No stuck fault is active")
            self._detector.reset()
            self._goal_active = False
            self._action_started_at = None
            self._publish_alert("")
            self._publish_state("RUNNING")
            self._event("EXECUTE", "SELF_HEALING", "operator_recovery_acknowledged")
        return TriggerResponse(success=True, message="Recovery acknowledged; current action will retry")

    def _send_navigation_goal(self, action: Dict[str, Any]) -> None:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = str(action.get("frame_id", self._mission_frame))
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(action["x"])
        goal.target_pose.pose.position.y = float(action["y"])
        quaternion = quaternion_from_euler(0.0, 0.0, float(action["yaw"]))
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        self._goal_active = True
        self._event(
            "PLAN",
            "SELF_CONFIGURATION",
            "rrt_route_requested",
            name=action.get("name", f"action_{self._action_index}"),
            x=action["x"],
            y=action["y"],
            yaw=action["yaw"],
        )
        self._move_base.send_goal(goal, done_cb=self._navigation_done)

    def _navigation_done(self, status: int, _result: Any) -> None:
        with self._lock:
            self._goal_active = False
            if self._state != "RUNNING":
                return
            if status == GoalStatus.SUCCEEDED:
                self._event("EXECUTE", "SELF_CONFIGURATION", "route_completed")
                self._action_index += 1
                self._action_started_at = None
            else:
                self._publish_state("ERROR")
                self._publish_alert(f"Navigation failed with move_base status {status}")
                self._event(
                    "ANALYZE", "SELF_CONFIGURATION", "route_failed", move_base_status=status
                )

    def _execute_current_action(self, now: rospy.Time) -> None:
        if self._action_index >= len(self._actions):
            self._completed_cycles += 1
            self._completed_pub.publish(Int32(data=self._completed_cycles))
            self._publish_state("COMPLETE")
            self._event(
                "KNOWLEDGE",
                "SELF_CONFIGURATION",
                "mission_completed",
                completed_cycles=self._completed_cycles,
            )
            return

        action = self._actions[self._action_index]
        action_type = action["type"]
        if action_type == "navigate":
            if not self._goal_active:
                self._send_navigation_goal(action)
            return

        if self._action_started_at is None:
            self._action_started_at = now
            if action_type == "lift":
                command = str(action["command"])
                self._lift_pub.publish(String(data=command))
                self._event("EXECUTE", "SELF_CONFIGURATION", "lift_command", command=command)
        duration = float(action.get("duration", 0.0))
        elapsed = (now - self._action_started_at).to_sec()
        if action_type == "wait_for_assembly":
            if self._assembly_ready:
                self._assembly_ready = False
                self._action_index += 1
                self._action_started_at = None
                self._event(
                    "EXECUTE", "SELF_CONFIGURATION", "assembly_release_received"
                )
            elif duration > 0.0 and elapsed >= duration:
                self._publish_state("ERROR")
                self._publish_alert("Timed out waiting for Smart Factory assembly-ready signal")
                self._event(
                    "ANALYZE", "SELF_HEALING", "factory_signal_timeout"
                )
            return
        expected_lift_state = str(action.get("wait_for_state", "")).strip().lower()
        if action_type == "lift" and expected_lift_state:
            if self._lift_state == expected_lift_state:
                self._action_index += 1
                self._action_started_at = None
            elif elapsed >= duration:
                self._lift_pub.publish(String(data="stop"))
                self._publish_state("ERROR")
                self._publish_alert(
                    f"Lift did not reach {expected_lift_state} within {duration:.1f} seconds"
                )
                self._event(
                    "ANALYZE",
                    "SELF_HEALING",
                    "lift_timeout",
                    expected_state=expected_lift_state,
                    observed_state=self._lift_state,
                )
        elif elapsed >= duration:
            self._action_index += 1
            self._action_started_at = None

    def _tick(self, event: TimerEvent) -> None:
        with self._lock:
            self._update_localized_pose()
            self._check_stuck(event.current_real)
            if self._state == "RUNNING":
                self._execute_current_action(event.current_real)


def main() -> None:
    rospy.init_node("autonomic_manager")
    try:
        AutonomicManager()
    except Exception as exc:
        rospy.logfatal("Unable to start autonomic manager: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
