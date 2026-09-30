#!/usr/bin/env python3
"""Final velocity gate for collision avoidance and workload speed limits."""

import json
import math
import threading
from typing import Iterable, Optional

import rospy
from rospy.timer import TimerEvent
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


class SafetyFilter:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._raw = Twist()
        self._raw_stamp = rospy.Time(0)
        self._scan: Optional[LaserScan] = None
        self._scan_stamp = rospy.Time(0)
        self._target_speed = rospy.get_param("~default_target_speed", 0.12)
        self._state = "IDLE"
        self._blocked = False

        self._command_timeout = rospy.get_param("~command_timeout", 0.5)
        self._scan_timeout = rospy.get_param("~scan_timeout", 0.5)
        self._emergency_distance = rospy.get_param("~emergency_distance", 0.22)
        self._caution_distance = rospy.get_param("~caution_distance", 0.55)
        self._rotation_distance = rospy.get_param("~rotation_emergency_distance", 0.20)
        self._sector_radians = math.radians(rospy.get_param("~motion_sector_degrees", 40.0))
        self._max_angular_speed = rospy.get_param("~max_angular_speed", 1.2)
        self._enforce_autonomy_state = rospy.get_param("~enforce_autonomy_state", True)

        self._cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._blocked_pub = rospy.Publisher("/autonomic/emergency_stop", Bool, queue_size=1, latch=True)
        self._distance_pub = rospy.Publisher("/autonomic/nearest_obstacle", Float32, queue_size=5)
        self._event_pub = rospy.Publisher("/autonomic/mapek_event", String, queue_size=20)

        rospy.Subscriber("/cmd_vel_raw", Twist, self._raw_callback, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self._scan_callback, queue_size=5)
        rospy.Subscriber("/autonomic/target_speed", Float32, self._speed_callback, queue_size=5)
        rospy.Subscriber("/autonomic/state", String, self._state_callback, queue_size=5)
        self._timer = rospy.Timer(rospy.Duration(0.05), self._tick)
        self._blocked_pub.publish(Bool(data=False))

    def _raw_callback(self, message: Twist) -> None:
        with self._lock:
            self._raw = message
            self._raw_stamp = rospy.Time.now()

    def _scan_callback(self, message: LaserScan) -> None:
        with self._lock:
            self._scan = message
            self._scan_stamp = rospy.Time.now()

    def _speed_callback(self, message: Float32) -> None:
        with self._lock:
            self._target_speed = max(0.0, float(message.data))

    def _state_callback(self, message: String) -> None:
        with self._lock:
            self._state = message.data

    @staticmethod
    def _minimum(values: Iterable[float]) -> float:
        usable = [value for value in values if math.isfinite(value) and value > 0.0]
        return min(usable) if usable else float("inf")

    def _sector_distance(self, direction: float) -> float:
        if self._scan is None or not self._scan.ranges:
            return float("inf")
        selected = []
        for index, distance in enumerate(self._scan.ranges):
            angle = self._scan.angle_min + index * self._scan.angle_increment
            delta = math.atan2(math.sin(angle - direction), math.cos(angle - direction))
            if abs(delta) <= self._sector_radians:
                selected.append(distance)
        return self._minimum(selected)

    def _all_around_distance(self) -> float:
        return float("inf") if self._scan is None else self._minimum(self._scan.ranges)

    @staticmethod
    def _stopped() -> Twist:
        return Twist()

    def _publish_block_transition(self, blocked: bool, distance: float, reason: str) -> None:
        if blocked == self._blocked:
            return
        self._blocked = blocked
        self._blocked_pub.publish(Bool(data=blocked))
        event = {
            "stamp": rospy.Time.now().to_sec(),
            "phase": "EXECUTE",
            "capability": "SELF_PROTECTION",
            "action": "emergency_stop" if blocked else "path_clear",
            "details": {"distance": distance, "reason": reason},
        }
        self._event_pub.publish(String(data=json.dumps(event, separators=(",", ":"))))

    def _tick(self, _event: TimerEvent) -> None:
        with self._lock:
            now = rospy.Time.now()
            if (now - self._raw_stamp).to_sec() > self._command_timeout:
                self._publish_block_transition(False, float("inf"), "command_timeout")
                self._cmd_pub.publish(self._stopped())
                return
            if self._enforce_autonomy_state and self._state != "RUNNING":
                self._publish_block_transition(False, float("inf"), "autonomy_not_running")
                self._cmd_pub.publish(self._stopped())
                return

            command = Twist()
            command.linear.x = self._raw.linear.x
            command.linear.y = self._raw.linear.y
            command.angular.z = max(
                -self._max_angular_speed,
                min(self._max_angular_speed, self._raw.angular.z),
            )

            planar_speed = math.hypot(command.linear.x, command.linear.y)
            is_moving = planar_speed > 1e-4 or abs(command.angular.z) > 1e-4
            if is_moving and (
                self._scan is None or (now - self._scan_stamp).to_sec() > self._scan_timeout
            ):
                self._publish_block_transition(True, 0.0, "lidar_timeout")
                self._cmd_pub.publish(self._stopped())
                return
            if self._target_speed <= 0.0:
                command.linear.x = 0.0
                command.linear.y = 0.0
            elif planar_speed > self._target_speed:
                scale = self._target_speed / planar_speed
                command.linear.x *= scale
                command.linear.y *= scale

            direction = 0.0 if command.linear.x >= 0.0 else math.pi
            distance = self._sector_distance(direction)
            reason = "translation_clearance"
            blocked = False
            if abs(command.linear.x) > 1e-4:
                if distance <= self._emergency_distance:
                    blocked = True
                elif distance < self._caution_distance:
                    scale = (distance - self._emergency_distance) / (
                        self._caution_distance - self._emergency_distance
                    )
                    command.linear.x *= max(0.0, min(1.0, scale))
            elif abs(command.angular.z) > 1e-4:
                distance = self._all_around_distance()
                reason = "rotation_clearance"
                blocked = distance <= self._rotation_distance

            self._distance_pub.publish(Float32(data=distance))
            self._publish_block_transition(blocked, distance, reason)
            self._cmd_pub.publish(self._stopped() if blocked else command)


def main() -> None:
    rospy.init_node("safety_filter")
    SafetyFilter()
    rospy.spin()


if __name__ == "__main__":
    main()
