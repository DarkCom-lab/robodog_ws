#!/usr/bin/env python3
"""Validate positive mass and diagonal inertia values in the expanded URDF."""

import subprocess
import unittest
import xml.etree.ElementTree as ET


def expanded_urdf():
    result = subprocess.run(
        [
            "bash",
            "-lc",
            "source /opt/ros/humble/setup.bash && "
            "source /home/friday/robodog_ws/install/setup.bash && "
            "xacro /home/friday/robodog_ws/src/open_quadruped/urdf/spot.urdf.xacro",
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=15,
    )
    return ET.fromstring(result.stdout)


class MassPropertiesTest(unittest.TestCase):
    def test_all_inertials_are_positive_and_finite(self):
        root = expanded_urdf()
        inertials = root.findall("link/inertial")
        self.assertEqual(len(inertials), 24)
        total_mass = 0.0
        for inertial in inertials:
            mass = float(inertial.find("mass").attrib["value"])
            inertia = inertial.find("inertia").attrib
            self.assertGreater(mass, 0.0)
            total_mass += mass
            for key in ("ixx", "iyy", "izz"):
                self.assertGreater(float(inertia[key]), 0.0)
            for key in ("ixy", "ixz", "iyx", "iyz", "izx", "izy"):
                self.assertAlmostEqual(float(inertia[key]), 0.0, places=12)
        self.assertAlmostEqual(total_mass, 3.14, places=6)


if __name__ == "__main__":
    unittest.main()
