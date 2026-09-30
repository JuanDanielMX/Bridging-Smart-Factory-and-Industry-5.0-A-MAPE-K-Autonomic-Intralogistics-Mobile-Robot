#!/usr/bin/env python3
"""Encrypted MQTT transport for Raspberry Pi 2 lift and battery data."""

import math
import os
import threading
from typing import Any, Dict

import paho.mqtt.client as mqtt
import rospy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from autonomic_turtlebot3.secure_payload import SecurePayload, SecurePayloadError, key_from_hex


class AuxiliaryMQTTBridge:
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
        self._lift_state = "unknown"
        self._battery: Dict[str, Any] = {
            "voltage": None,
            "current": None,
            "percentage": None,
        }
        self._command_topic = rospy.get_param(
            "~command_topic", "robot/secure/lift_command"
        )
        self._status_topic = rospy.get_param(
            "~status_topic", "robot/secure/aux_status"
        )

        self._lift_pub = rospy.Publisher("/lift/command_local", String, queue_size=10)
        rospy.Subscriber("/lift/state_local", String, self._lift_callback, queue_size=10)
        rospy.Subscriber(
            "/battery_state_local", BatteryState, self._battery_callback, queue_size=10
        )

        client_id = rospy.get_param("~client_id", "autonomic_auxiliary")
        try:
            self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        except AttributeError:
            self._client = mqtt.Client(client_id=client_id)
        username = rospy.get_param("~username", "autonomic_auxiliary")
        password_env = rospy.get_param("~password_env", "AUTONOMIC_MQTT_AUX_PASSWORD")
        self._client.username_pw_set(username, os.environ.get(password_env, ""))
        ca_cert = rospy.get_param("~tls_ca_cert", "")
        if ca_cert:
            self._client.tls_set(ca_certs=ca_cert)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.on_disconnect = self._on_disconnect
        self._client.connect_async(
            rospy.get_param("~host", "192.168.10.10"),
            int(rospy.get_param("~port", 1883)),
            int(rospy.get_param("~keepalive", 30)),
        )
        self._client.loop_start()
        rate = max(0.2, float(rospy.get_param("~status_rate", 2.0)))
        self._timer = rospy.Timer(rospy.Duration(1.0 / rate), self._publish_status)
        rospy.on_shutdown(self.shutdown)

    def _on_connect(self, client: mqtt.Client, _userdata: Any, _flags: Any, reason: Any, *_args: Any) -> None:
        code = int(getattr(reason, "value", reason))
        if code != 0:
            rospy.logerr("Auxiliary MQTT connection failed with reason %s", reason)
            return
        client.subscribe(self._command_topic, qos=1)
        rospy.loginfo("Auxiliary MQTT bridge connected")

    @staticmethod
    def _on_disconnect(_client: mqtt.Client, _userdata: Any, *args: Any) -> None:
        reason = args[0] if len(args) == 1 else (args[1] if len(args) >= 2 else "unknown")
        rospy.logwarn("Auxiliary MQTT disconnected: %s", reason)

    def _on_message(self, _client: mqtt.Client, _userdata: Any, message: mqtt.MQTTMessage) -> None:
        try:
            value = self._secure.decrypt(message.payload.decode("utf-8"))
            command = str(value["command"]).strip().lower()
            if command not in {"up", "down", "stop"}:
                raise ValueError("unsupported lift command")
            self._lift_pub.publish(String(data=command))
        except (SecurePayloadError, KeyError, TypeError, ValueError) as exc:
            rospy.logerr_throttle(5.0, "Rejected auxiliary command: %s", exc)

    def _lift_callback(self, message: String) -> None:
        with self._lock:
            self._lift_state = message.data

    def _battery_callback(self, message: BatteryState) -> None:
        def finite(value: float):
            return round(float(value), 4) if math.isfinite(value) else None

        with self._lock:
            self._battery = {
                "voltage": finite(message.voltage),
                "current": finite(message.current),
                "percentage": finite(message.percentage),
            }

    def _publish_status(self, _event: rospy.TimerEvent) -> None:
        with self._lock:
            payload = {
                "kind": "auxiliary_status",
                "stamp": rospy.Time.now().to_sec(),
                "lift_state": self._lift_state,
                "battery": dict(self._battery),
            }
        self._client.publish(
            self._status_topic,
            self._secure.encrypt(payload),
            qos=1,
            retain=True,
        )

    def shutdown(self) -> None:
        if hasattr(self, "_client"):
            try:
                self._client.loop_stop()
                self._client.disconnect()
            except Exception:
                pass


def main() -> None:
    rospy.init_node("auxiliary_mqtt_bridge")
    try:
        AuxiliaryMQTTBridge()
    except Exception as exc:
        rospy.logfatal("Unable to start auxiliary MQTT bridge: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
