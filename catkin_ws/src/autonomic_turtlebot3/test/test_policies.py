#!/usr/bin/env python3

import unittest

from autonomic_turtlebot3.policies import StuckDetector, target_speed


class SpeedPolicyTest(unittest.TestCase):
    def test_paper_equation(self):
        self.assertEqual(target_speed(0), 0.02)
        self.assertEqual(target_speed(5), 0.12)
        self.assertEqual(target_speed(11), 0.24)

    def test_orders_are_bounded(self):
        self.assertEqual(target_speed(-4), 0.02)
        self.assertEqual(target_speed(99), 0.24)

    def test_low_battery_derates_high_demand(self):
        self.assertEqual(target_speed(11, battery_fraction=0.10), 0.06)


class StuckDetectorTest(unittest.TestCase):
    def test_detects_encoder_localization_divergence(self):
        detector = StuckDetector(window_seconds=3.0)
        self.assertFalse(detector.update(0.0, 0.1, (0.0, 0.0), (0.0, 0.0)))
        self.assertFalse(detector.update(1.5, 0.1, (0.06, 0.0), (0.005, 0.0)))
        self.assertTrue(detector.update(3.0, 0.1, (0.12, 0.0), (0.01, 0.0)))

    def test_normal_motion_is_not_stuck(self):
        detector = StuckDetector(window_seconds=3.0)
        detector.update(0.0, 0.1, (0.0, 0.0), (0.0, 0.0))
        self.assertFalse(detector.update(3.0, 0.1, (0.12, 0.0), (0.11, 0.0)))

    def test_stopping_resets_window(self):
        detector = StuckDetector(window_seconds=3.0)
        detector.update(0.0, 0.1, (0.0, 0.0), (0.0, 0.0))
        detector.update(2.0, 0.0, (0.08, 0.0), (0.0, 0.0))
        self.assertFalse(detector.stuck)


if __name__ == "__main__":
    unittest.main()
