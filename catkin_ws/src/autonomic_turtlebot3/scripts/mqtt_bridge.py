#!/usr/bin/env python3
"""Bidirectional encrypted MQTT bridge between ROS and the Smart Factory."""

import json
import math
import os
import threading
from typing import Any, Dict

import paho.mqtt.client as mqtt
import rospy
from rospy.timer import TimerEvent
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import Trigger

from autonomic_turtlebot3.secure_payload import SecurePayload, SecurePayloadError, key_from_hex


class MQTTBridge:
    def __init__(self) -> None:
        key_env = rospy.get_param("~master_key_env", "AUTONOMIC_MASTER_KEY_HEX")
        key_value = os.environ.get(key_env, "")
        if not key_value:
            raise RuntimeError(f"required environment variable {key_env} is not set")
        self._secure = SecurePayload(
            key_from_hex(key_value),
            max_age_seconds=rospy.get_param("~max_message_age_seconds", 120),
        )
        self._lock = threading.RLock()
        self._status: Dict[str, Any] = {
            "state": "OFFLINE",
            "alert": "",
            "target_speed": 0.0,
            "battery_fraction": None,
            "emergency_stop": False,
            "completed_cycles": 0,
        }

        self._orders_topic = rospy.get_param("~orders_topic", "factory/secure/orders")
        self._control_topic = rospy.get_param("~control_topic", "factory/secure/control")
        self._status_topic = rospy.get_param("~status_topic", "robot/secure/status")
        self._event_topic = rospy.get_param("~event_topic", "robot/secure/events")
        self._aux_command_topic = rospy.get_param(
            "~aux_command_topic", "robot/secure/lift_command"
        )
        self._aux_status_topic = rospy.get_param(
            "~aux_status_topic", "robot/secure/aux_status"
        )

        self._orders_pub = rospy.Publisher("/factory/queued_orders", Int32, queue_size=10, latch=True)
        self._assembly_pub = rospy.Publisher(
            "/factory/assembly_ready", Bool, queue_size=10, latch=True
        )
        self._battery_pub = rospy.Publisher("/battery_state", BatteryState, queue_size=10, latch=True)
        self._lift_state_pub = rospy.Publisher("/lift/state", String, queue_size=10, latch=True)
        rospy.Subscriber("/autonomic/state", String, self._state_callback, queue_size=10)
        rospy.Subscriber("/autonomic/alert", String, self._alert_callback, queue_size=10)
        rospy.Subscriber("/autonomic/target_speed", Float32, self._speed_callback, queue_size=10)
        rospy.Subscriber("/battery_state", BatteryState, self._battery_callback, queue_size=10)
        rospy.Subscriber("/autonomic/emergency_stop", Bool, self._stop_callback, queue_size=10)
        rospy.Subscriber("/autonomic/completed_cycles", Int32, self._completed_callback, queue_size=10)
        rospy.Subscriber("/autonomic/mapek_event", String, self._event_callback, queue_size=100)
        rospy.Subscriber("/lift/command", String, self._lift_command_callback, queue_size=20)

        client_id = rospy.get_param("~client_id", "autonomic_turtlebot3")
        try:
            self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        except AttributeError:
            self._client = mqtt.Client(client_id=client_id)
        username = rospy.get_param("~username", "")
        password_env = rospy.get_param("~password_env", "AUTONOMIC_MQTT_PASSWORD")
        if username:
            self._client.username_pw_set(username, os.environ.get(password_env, ""))

        ca_cert = rospy.get_param("~tls_ca_cert", "")
        if ca_cert:
            self._client.tls_set(ca_certs=ca_cert)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.on_disconnect = self._on_disconnect

        host = rospy.get_param("~host", "127.0.0.1")
        port = int(rospy.get_param("~port", 1883))
        keepalive = int(rospy.get_param("~keepalive", 30))
        self._client.connect_async(host, port, keepalive)
        self._client.loop_start()
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / max(0.2, rospy.get_param("~status_rate", 2.0))),
            self._publish_status,
        )
        rospy.on_shutdown(self.shutdown)

    def _on_connect(self, client: mqtt.Client, _userdata: Any, _flags: Any, reason: Any, *_args: Any) -> None:
        code = int(getattr(reason, "value", reason))
        if code != 0:
            rospy.logerr("MQTT connection failed with reason %s", reason)
            return
        client.subscribe(
            [
                (self._orders_topic, 1),
                (self._control_topic, 1),
                (self._aux_status_topic, 1),
            ]
        )
        rospy.loginfo("Connected to MQTT broker and subscribed to secure factory topics")

    @staticmethod
    def _on_disconnect(_client: mqtt.Client, _userdata: Any, *args: Any) -> None:
        # Paho 1.x passes (rc); Paho 2.x passes (flags, reason_code, properties).
        reason = args[0] if len(args) == 1 else (args[1] if len(args) >= 2 else "unknown")
        rospy.logwarn("MQTT disconnected: %s", reason)

    def _on_message(self, _client: mqtt.Client, _userdata: Any, message: mqtt.MQTTMessage) -> None:
        try:
            value = self._secure.decrypt(message.payload.decode("utf-8"))
            if message.topic == self._orders_topic:
                orders = max(0, int(value["queued_orders"]))
                self._orders_pub.publish(Int32(data=orders))
            elif message.topic == self._control_topic:
                self._handle_control(str(value.get("action", "")))
            elif message.topic == self._aux_status_topic:
                self._handle_auxiliary_status(value)
        except (SecurePayloadError, KeyError, TypeError, ValueError) as exc:
            rospy.logerr_throttle(5.0, "Rejected MQTT message on %s: %s", message.topic, exc)

    @staticmethod
    def _service_for_action(action: str) -> str:
        return {
            "start": "/autonomic_manager/start_mission",
            "cancel": "/autonomic_manager/cancel_mission",
            "acknowledge_recovery": "/autonomic_manager/acknowledge_recovery",
        }.get(action, "")

    def _handle_control(self, action: str) -> None:
        if action in {"assembly_ready", "assembly_not_ready"}:
            self._assembly_pub.publish(Bool(data=action == "assembly_ready"))
            rospy.loginfo("Factory assembly-ready flag set to %s", action == "assembly_ready")
            return
        service_name = self._service_for_action(action)
        if not service_name:
            rospy.logwarn("Ignored unsupported factory control action: %s", action)
            return
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
            response = rospy.ServiceProxy(service_name, Trigger)()
            rospy.loginfo("Factory control '%s': %s", action, response.message)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("Factory control '%s' failed: %s", action, exc)

    def _handle_auxiliary_status(self, value: Dict[str, Any]) -> None:
        battery = value.get("battery", {})
        if isinstance(battery, dict):
            def numeric(field: str) -> float:
                raw = battery.get(field)
                return float(raw) if raw is not None else float("nan")

            message = BatteryState()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = "base_link"
            message.voltage = numeric("voltage")
            message.current = numeric("current")
            message.percentage = numeric("percentage")
            message.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            message.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            message.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
            message.present = True
            self._battery_pub.publish(message)
        lift_state = value.get("lift_state")
        if lift_state is not None:
            self._lift_state_pub.publish(String(data=str(lift_state)))

    def _lift_command_callback(self, message: String) -> None:
        command = message.data.strip().lower()
        if command not in {"up", "down", "stop"}:
            rospy.logwarn("Not forwarding unsupported lift command: %s", command)
            return
        packet = self._secure.encrypt(
            {"kind": "lift_command", "command": command, "stamp": rospy.Time.now().to_sec()}
        )
        self._client.publish(self._aux_command_topic, packet, qos=1, retain=False)

    def _state_callback(self, message: String) -> None:
        with self._lock:
            self._status["state"] = message.data

    def _alert_callback(self, message: String) -> None:
        with self._lock:
            self._status["alert"] = message.data

    def _speed_callback(self, message: Float32) -> None:
        with self._lock:
            self._status["target_speed"] = round(float(message.data), 4)

    def _battery_callback(self, message: BatteryState) -> None:
        with self._lock:
            fraction = float(message.percentage)
            self._status["battery_fraction"] = (
                round(fraction, 4) if math.isfinite(fraction) and fraction >= 0.0 else None
            )
            self._status["battery_voltage"] = (
                round(float(message.voltage), 3) if math.isfinite(message.voltage) else None
            )

    def _stop_callback(self, message: Bool) -> None:
        with self._lock:
            self._status["emergency_stop"] = bool(message.data)

    def _completed_callback(self, message: Int32) -> None:
        with self._lock:
            self._status["completed_cycles"] = int(message.data)

    def _event_callback(self, message: String) -> None:
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            event = {"raw": message.data}
        event["kind"] = "mapek_event"
        self._client.publish(self._event_topic, self._secure.encrypt(event), qos=1)

    def _publish_status(self, _event: TimerEvent) -> None:
        with self._lock:
            status = dict(self._status)
        status["stamp"] = rospy.Time.now().to_sec()
        status["kind"] = "robot_status"
        self._client.publish(self._status_topic, self._secure.encrypt(status), qos=1, retain=True)

    def shutdown(self) -> None:
        if hasattr(self, "_client"):
            try:
                self._client.loop_stop()
                self._client.disconnect()
            except Exception:
                pass


def main() -> None:
    rospy.init_node("mqtt_bridge")
    try:
        MQTTBridge()
    except Exception as exc:
        rospy.logfatal("Unable to start encrypted MQTT bridge: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
