"""ROS-independent Self-CHOP policies, kept small enough to unit test."""

from collections import deque
from dataclasses import dataclass
from math import hypot
from typing import Deque, Optional, Tuple


Point = Tuple[float, float]


def target_speed(
    queued_orders: int,
    base_speed: float = 0.02,
    speed_per_order: float = 0.02,
    max_orders: int = 11,
    battery_fraction: Optional[float] = None,
    low_battery_fraction: float = 0.15,
) -> float:
    """Implement Sf = 0.02 + 0.02B with optional low-battery derating."""
    bounded_orders = max(0, min(int(queued_orders), int(max_orders)))
    speed = float(base_speed) + float(speed_per_order) * bounded_orders
    if battery_fraction is not None and battery_fraction <= low_battery_fraction:
        speed = min(speed, max(base_speed, 0.06))
    return round(speed, 6)


@dataclass(frozen=True)
class MotionSample:
    timestamp: float
    commanded_speed: float
    odom: Point
    localized: Point


class StuckDetector:
    """Detect wheel-motion/localized-motion divergence over a time window."""

    def __init__(
        self,
        window_seconds: float = 3.0,
        min_command_speed: float = 0.03,
        min_odom_motion: float = 0.08,
        max_localized_motion: float = 0.025,
    ) -> None:
        self.window_seconds = float(window_seconds)
        self.min_command_speed = float(min_command_speed)
        self.min_odom_motion = float(min_odom_motion)
        self.max_localized_motion = float(max_localized_motion)
        self._samples: Deque[MotionSample] = deque()
        self._stuck = False

    @property
    def stuck(self) -> bool:
        return self._stuck

    def reset(self) -> None:
        self._samples.clear()
        self._stuck = False

    def update(
        self,
        timestamp: float,
        commanded_speed: float,
        odom: Point,
        localized: Point,
    ) -> bool:
        sample = MotionSample(float(timestamp), abs(float(commanded_speed)), odom, localized)
        if sample.commanded_speed < self.min_command_speed:
            self.reset()
            return False

        self._samples.append(sample)
        cutoff = sample.timestamp - self.window_seconds
        while self._samples and self._samples[0].timestamp < cutoff:
            self._samples.popleft()

        if len(self._samples) < 2:
            return False
        first = self._samples[0]
        elapsed = sample.timestamp - first.timestamp
        if elapsed < self.window_seconds * 0.9:
            return False

        odom_distance = hypot(sample.odom[0] - first.odom[0], sample.odom[1] - first.odom[1])
        localized_distance = hypot(
            sample.localized[0] - first.localized[0],
            sample.localized[1] - first.localized[1],
        )
        self._stuck = (
            odom_distance >= self.min_odom_motion
            and localized_distance <= self.max_localized_motion
        )
        return self._stuck
