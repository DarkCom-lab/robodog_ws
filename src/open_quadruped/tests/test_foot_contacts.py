#!/usr/bin/env python3
"""Check that all four configured foot collisions contact the ground plane."""

import re
import subprocess
import unittest


EXPECTED_FEET = {
    "front_left_foot",
    "front_right_foot",
    "back_left_foot",
    "back_right_foot",
}


def contact_stream(duration=1):
    result = subprocess.run(
        [
            "gz",
            "topic",
            "-e",
            "/gazebo/default/physics/contacts",
            "-d",
            str(duration),
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=duration + 5,
    )
    return result.stdout


def contacted_feet(text):
    feet = set()
    for match in re.finditer(
        r'collision[12]: "open_quadruped::.*__(front_left|front_right|back_left|back_right)_foot_collision"',
        text,
    ):
        feet.add(f'{match.group(1)}_foot')
    return feet


class FootContactTest(unittest.TestCase):
    def test_all_four_feet_contact_ground(self):
        text = contact_stream()
        self.assertIn('collision2: "ground_plane::link::collision"', text)
        self.assertEqual(contacted_feet(text), EXPECTED_FEET)


if __name__ == "__main__":
    unittest.main()
