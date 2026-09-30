#!/usr/bin/env python3
"""Safe TB6600 stepper controller for the lift mechanism on Raspberry Pi 2."""

import json
import threading
import time

import rospy
from std_msgs.msg import String


class LiftController:
    VALID_COMMANDS = {"up", "down", "stop"}

    def __init__(self) -> None:
        self._simulation = rospy.get_param("~simulation", False)
        self._step_pin = int(rospy.get_param("~step_pin", 18))
        self._direction_pin = int(rospy.get_param("~direction_pin", 23))
        self._enable_pin = int(rospy.get_param("~enable_pin", 24))
        self._upper_limit_pin = int(rospy.get_param("~upper_limit_pin", 17))
        self._lower_limit_pin = int(rospy.get_param("~lower_limit_pin", 27))
        self._limit_active_high = rospy.get_param("~limit_active_high", True)
        self._limit_pull_up = rospy.get_param("~limit_pull_up", True)
        self._enable_active_low = rospy.get_param("~enable_active_low", True)
        self._up_direction_high = rospy.get_param("~up_direction_high", True)
        self._step_frequency = float(rospy.get_param("~step_frequency_hz", 400.0))
        self._max_travel_seconds = float(rospy.get_param("~max_travel_seconds", 8.0))
        self._simulation_travel_seconds = float(rospy.get_param("~simulation_travel_seconds", 1.0))

        self._condition = threading.Condition()
        self._desired = "stop"
        self._state = "stopped"
        self._stop_requested = False
        self._gpio = None

        command_topic = rospy.get_param("~command_topic", "/lift/command")
        state_topic = rospy.get_param("~state_topic", "/lift/state")
        self._state_pub = rospy.Publisher(state_topic, String, queue_size=5, latch=True)
        self._event_pub = rospy.Publisher("/autonomic/mapek_event", String, queue_size=20)
        rospy.Subscriber(command_topic, String, self._command_callback, queue_size=10)

        if not self._simulation:
            self._configure_gpio()
        self._worker = threading.Thread(target=self._run, name="lift-worker", daemon=True)
        self._worker.start()
        self._publish_state("stopped")
        rospy.on_shutdown(self.shutdown)

    def _configure_gpio(self) -> None:
        try:
            import RPi.GPIO as gpio
        except ImportError as exc:
            raise RuntimeError("RPi.GPIO is required when lift simulation is disabled") from exc
        self._gpio = gpio
        gpio.setwarnings(False)
        gpio.setmode(gpio.BCM)
        gpio.setup(self._step_pin, gpio.OUT, initial=gpio.LOW)
        gpio.setup(self._direction_pin, gpio.OUT, initial=gpio.LOW)
        gpio.setup(
            self._enable_pin,
            gpio.OUT,
            initial=gpio.HIGH if self._enable_active_low else gpio.LOW,
        )
        pull = gpio.PUD_UP if self._limit_pull_up else gpio.PUD_DOWN
        gpio.setup(self._upper_limit_pin, gpio.IN, pull_up_down=pull)
        gpio.setup(self._lower_limit_pin, gpio.IN, pull_up_down=pull)

    def _publish_state(self, state: str, reason: str = "") -> None:
        self._state = state
        self._state_pub.publish(String(data=state))
        event = {
            "stamp": rospy.Time.now().to_sec(),
            "phase": "EXECUTE",
            "capability": "SELF_CONFIGURATION",
            "action": "lift_state",
            "details": {"state": state, "reason": reason},
        }
        self._event_pub.publish(String(data=json.dumps(event, separators=(",", ":"))))

    def _command_callback(self, message: String) -> None:
        command = message.data.strip().lower()
        if command not in self.VALID_COMMANDS:
            rospy.logwarn("Rejected unknown lift command: %s", command)
            return
        with self._condition:
            self._desired = command
            self._condition.notify_all()

    def _limit_active(self, pin: int) -> bool:
        if self._simulation or self._gpio is None:
            return False
        value = bool(self._gpio.input(pin))
        return value if self._limit_active_high else not value

    def _set_enabled(self, enabled: bool) -> None:
        if self._simulation or self._gpio is None:
            return
        level = not enabled if self._enable_active_low else enabled
        self._gpio.output(self._enable_pin, self._gpio.HIGH if level else self._gpio.LOW)

    def _move(self, command: str) -> None:
        limit_pin = self._upper_limit_pin if command == "up" else self._lower_limit_pin
        if self._limit_active(limit_pin):
            self._publish_state(f"{command}_limit", "limit_already_active")
            return

        self._publish_state(f"moving_{command}")
        start = time.monotonic()
        if self._simulation:
            while time.monotonic() - start < self._simulation_travel_seconds:
                with self._condition:
                    if self._desired != command or self._stop_requested:
                        self._publish_state("stopped", "commanded_stop")
                        return
                time.sleep(0.02)
            self._publish_state(f"{command}_limit", "simulation_complete")
            return

        assert self._gpio is not None
        direction_high = self._up_direction_high if command == "up" else not self._up_direction_high
        self._gpio.output(
            self._direction_pin,
            self._gpio.HIGH if direction_high else self._gpio.LOW,
        )
        self._set_enabled(True)
        half_period = 0.5 / max(1.0, self._step_frequency)
        reason = "limit_reached"
        while not rospy.is_shutdown() and not self._stop_requested:
            with self._condition:
                if self._desired != command:
                    reason = "commanded_stop"
                    break
            if self._limit_active(limit_pin):
                break
            if time.monotonic() - start >= self._max_travel_seconds:
                reason = "travel_timeout"
                break
            self._gpio.output(self._step_pin, self._gpio.HIGH)
            time.sleep(half_period)
            self._gpio.output(self._step_pin, self._gpio.LOW)
            time.sleep(half_period)
        self._set_enabled(False)
        if reason == "travel_timeout":
            self._publish_state("fault", reason)
            rospy.logerr("Lift stopped after maximum travel time; inspect limit switches")
        elif reason == "limit_reached":
            self._publish_state(f"{command}_limit", reason)
        else:
            self._publish_state("stopped", reason)

    def _run(self) -> None:
        while not self._stop_requested and not rospy.is_shutdown():
            with self._condition:
                self._condition.wait_for(
                    lambda: self._desired != "stop" or self._stop_requested,
                    timeout=0.25,
                )
                if self._stop_requested:
                    break
                command = self._desired
            if command in {"up", "down"}:
                self._move(command)
                with self._condition:
                    if self._desired == command:
                        self._desired = "stop"

    def shutdown(self) -> None:
        self._stop_requested = True
        with self._condition:
            self._desired = "stop"
            self._condition.notify_all()
        if hasattr(self, "_worker") and self._worker.is_alive():
            self._worker.join(timeout=1.0)
        if self._gpio is not None:
            self._set_enabled(False)
            self._gpio.cleanup(
                [
                    self._step_pin,
                    self._direction_pin,
                    self._enable_pin,
                    self._upper_limit_pin,
                    self._lower_limit_pin,
                ]
            )


def main() -> None:
    rospy.init_node("lift_controller")
    try:
        LiftController()
    except Exception as exc:
        rospy.logfatal("Unable to start lift controller: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
