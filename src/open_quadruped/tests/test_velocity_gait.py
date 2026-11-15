#!/usr/bin/env python3
"""Pure-math regression tests for keyboard foot-space gait generation."""

import math
import unittest

from control_library import velocity_gait


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

    def test_each_diagonal_pair_starts_at_zero_offset(self):
        for phase_offset in (0.0, 0.5):
            actual = velocity_gait.relative_foot_path(
                0.0, phase_offset, -0.03, 0.02, 0.65, 0.02
            )
            self.assertEqual(actual, (0.0, 0.0, 0.0))

    def test_swing_lifts_foot(self):
        stance = velocity_gait.sample_foot_path(0.25, -0.03, 0.01, 0.65, 0.02)
        swing_midpoint = velocity_gait.sample_foot_path(0.825, -0.03, 0.01, 0.65, 0.02)
        self.assertEqual(stance[2], 0.0)
        self.assertAlmostEqual(swing_midpoint[2], 0.02)

    def test_stance_swing_boundary_is_continuous(self):
        left = velocity_gait.sample_foot_path(0.65, -0.03, 0.02, 0.65, 0.02)
        right = velocity_gait.sample_foot_path(0.65 + 1e-9, -0.03, 0.02, 0.65, 0.02)
        for lhs, rhs in zip(left, right):
            self.assertAlmostEqual(lhs, rhs, places=8)

    def test_diagonal_double_support_windows(self):
        self.assertTrue(velocity_gait.in_diagonal_double_support(0.05, 0.65))
        self.assertTrue(velocity_gait.in_diagonal_double_support(0.55, 0.65))
        self.assertFalse(velocity_gait.in_diagonal_double_support(0.25, 0.65))
        self.assertFalse(velocity_gait.in_diagonal_double_support(0.80, 0.65))

    def test_cycle_wrap_is_continuous(self):
        start = velocity_gait.sample_foot_path(0.0, -0.03, 0.02, 0.65, 0.02)
        end = velocity_gait.sample_foot_path(1.0, -0.03, 0.02, 0.65, 0.02)
        for lhs, rhs in zip(start, end):
            self.assertTrue(math.isclose(lhs, rhs, abs_tol=1e-12))


if __name__ == "__main__":
    unittest.main()
