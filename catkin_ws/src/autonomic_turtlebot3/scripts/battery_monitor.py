#!/usr/bin/env python3
"""LiFePO4 battery telemetry using an INA219 or a simulation source."""

import math
import time
from typing import Optional, Tuple

import rospy
from sensor_msgs.msg import BatteryState


class BatteryMonitor:
    def __init__(self) -> None:
        self._backend = str(rospy.get_param("~backend", "simulation")).lower()
        self._frame_id = rospy.get_param("~frame_id", "battery_link")
        self._voltage_empty = float(rospy.get_param("~voltage_empty", 12.0))
        self._voltage_full = float(rospy.get_param("~voltage_full", 14.4))
        self._capacity_ah = float(rospy.get_param("~capacity_ah", 10.0))
        self._smoothing = max(0.0, min(1.0, float(rospy.get_param("~smoothing", 0.2))))
        self._filtered_voltage: Optional[float] = None
        self._sensor = None
        self._simulation_start = time.monotonic()
        self._simulation_runtime = float(rospy.get_param("~simulation_runtime_seconds", 1800.0))

        if self._backend == "ina219":
            self._configure_ina219()
        elif self._backend != "simulation":
            raise ValueError("battery backend must be 'ina219' or 'simulation'")

        output_topic = rospy.get_param("~output_topic", "/battery_state")
        self._publisher = rospy.Publisher(output_topic, BatteryState, queue_size=10, latch=True)
        rate = max(0.1, float(rospy.get_param("~publish_rate", 1.0)))
        self._timer = rospy.Timer(rospy.Duration(1.0 / rate), self._publish)

    def _configure_ina219(self) -> None:
        try:
            import board
            from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219
        except ImportError as exc:
            raise RuntimeError(
                "INA219 backend requires adafruit-circuitpython-ina219 and Blinka"
            ) from exc
        sensor = INA219(board.I2C(), addr=int(rospy.get_param("~i2c_address", 0x40)))
        sensor.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        sensor.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        sensor.bus_voltage_range = BusVoltageRange.RANGE_16V
        self._sensor = sensor

    def _read(self) -> Tuple[float, float, float]:
        if self._backend == "simulation":
            elapsed = time.monotonic() - self._simulation_start
            fraction = max(0.0, 1.0 - elapsed / max(1.0, self._simulation_runtime))
            voltage = self._voltage_empty + fraction * (self._voltage_full - self._voltage_empty)
            return voltage, -0.8, fraction

        assert self._sensor is not None
        voltage = float(self._sensor.bus_voltage + self._sensor.shunt_voltage / 1000.0)
        current = -float(self._sensor.current) / 1000.0
        fraction = (voltage - self._voltage_empty) / (self._voltage_full - self._voltage_empty)
        return voltage, current, max(0.0, min(1.0, fraction))

    def _publish(self, _event: rospy.TimerEvent) -> None:
        try:
            voltage, current, fraction = self._read()
        except (OSError, RuntimeError, ValueError) as exc:
            rospy.logerr_throttle(10.0, "Battery sensor read failed: %s", exc)
            return

        if self._filtered_voltage is None:
            self._filtered_voltage = voltage
        else:
            self._filtered_voltage = (
                self._smoothing * voltage + (1.0 - self._smoothing) * self._filtered_voltage
            )
        filtered_fraction = max(
            0.0,
            min(
                1.0,
                (self._filtered_voltage - self._voltage_empty)
                / (self._voltage_full - self._voltage_empty),
            ),
        )

        message = BatteryState()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self._frame_id
        message.voltage = self._filtered_voltage
        message.current = current
        message.charge = float("nan")
        message.capacity = self._capacity_ah
        message.design_capacity = self._capacity_ah
        message.percentage = filtered_fraction if math.isfinite(fraction) else float("nan")
        message.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            if current < 0.0
            else BatteryState.POWER_SUPPLY_STATUS_CHARGING
        )
        message.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        message.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        message.present = True
        self._publisher.publish(message)


def main() -> None:
    rospy.init_node("battery_monitor")
    try:
        BatteryMonitor()
    except Exception as exc:
        rospy.logfatal("Unable to start battery monitor: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
