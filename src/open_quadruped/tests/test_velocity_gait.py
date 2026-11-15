#!/usr/bin/env python3
"""Pure-math regression tests for keyboard foot-space crawl gait generation."""

import math
import unittest

from control_library import velocity_gait


PHASE_OFFSETS = {"fl": 0.75, "br": 0.50, "fr": 0.25, "bl": 0.0}


class VelocityGaitTest(unittest.TestCase):
    def test_forward_stance_moves_foot_backward(self):
        dx, dy = velocity_gait.stance_displacement((0.0, 0.0), 0.04, 0.0, 0.0, 0.6)
        self.assertAlmostEqual(dx, -0.024)
        self.assertAlmostEqual(dy, 0.0)

    def test_lateral_stance_opposes_body_motion(self):
        dx, dy = velocity_gait.stance_displacement((0.0, 0.0), 0.0, 0.03, 0.0, 0.5)
        self.assertAlmostEqual(dx, 0.0)
        self.assertAlmostEqual(dy, -0.015)

    def test_yaw_creates_tangential_foot_paths(self):
        fl = velocity_gait.stance_displacement((0.10, 0.08), 0.0, 0.0, 0.5, 0.5)
        br = velocity_gait.stance_displacement((-0.10, -0.08), 0.0, 0.0, 0.5, 0.5)
        self.assertGreater(fl[0], 0.0)
        self.assertLess(fl[1], 0.0)
        self.assertLess(br[0], 0.0)
        self.assertGreater(br[1], 0.0)

    def test_each_crawl_leg_starts_at_measured_position(self):
        for offset in PHASE_OFFSETS.values():
            actual = velocity_gait.relative_foot_path(
                0.0, offset, -0.03, 0.02, 0.82, 0.015
            )
            self.assertEqual(actual, (0.0, 0.0, 0.0))

    def test_swing_lifts_foot(self):
        stance = velocity_gait.sample_foot_path(0.25, -0.03, 0.01, 0.82, 0.015)
        swing_midpoint = velocity_gait.sample_foot_path(0.91, -0.03, 0.01, 0.82, 0.015)
        self.assertEqual(stance[2], 0.0)
        self.assertAlmostEqual(swing_midpoint[2], 0.015)

    def test_stance_swing_boundary_is_continuous(self):
        left = velocity_gait.sample_foot_path(0.82, -0.03, 0.02, 0.82, 0.015)
        right = velocity_gait.sample_foot_path(0.82 + 1e-9, -0.03, 0.02, 0.82, 0.015)
        for lhs, rhs in zip(left, right):
            self.assertAlmostEqual(lhs, rhs, places=8)

    def test_at_most_one_leg_swings_and_three_remain_in_stance(self):
        for index in range(1000):
            phase = index / 1000.0
            self.assertLessEqual(
                velocity_gait.maximum_swinging_legs(phase, PHASE_OFFSETS, 0.82),
                1,
            )

    def test_safe_all_feet_down_stop_windows_exist(self):
        for phase in (0.0, 0.25, 0.5, 0.75):
            self.assertTrue(
                velocity_gait.in_all_feet_stance(phase, PHASE_OFFSETS, 0.82)
            )
        for phase in (0.1, 0.35, 0.6, 0.85):
            self.assertFalse(
                velocity_gait.in_all_feet_stance(phase, PHASE_OFFSETS, 0.82)
            )

    def test_cycle_wrap_is_continuous(self):
        start = velocity_gait.sample_foot_path(0.0, -0.03, 0.02, 0.82, 0.015)
        end = velocity_gait.sample_foot_path(1.0, -0.03, 0.02, 0.82, 0.015)
        for lhs, rhs in zip(start, end):
            self.assertTrue(math.isclose(lhs, rhs, abs_tol=1e-12))


if __name__ == "__main__":
    unittest.main()
