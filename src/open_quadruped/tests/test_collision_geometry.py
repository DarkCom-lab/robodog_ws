#!/usr/bin/env python3
"""Audit collision primitives in the expanded baseline URDF."""

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


class CollisionGeometryTest(unittest.TestCase):
    def test_only_feet_have_positive_collision_primitives(self):
        root = expanded_urdf()
        links = root.findall("link")
        self.assertEqual(len(links), 25)
        self.assertEqual(len({link.attrib["name"] for link in links}), len(links))

        for link in links:
            name = link.attrib["name"]
            collisions = link.findall("collision")
            if name == "base_link" or not name.endswith("_foot"):
                self.assertEqual(collisions, [])
                continue
            self.assertEqual(len(collisions), 1, name)
            geometry = collisions[0].find("geometry")
            self.assertIsNotNone(geometry, name)
            primitive = next(iter(geometry), None)
            self.assertIsNotNone(primitive, name)
            if primitive.tag == "box":
                dimensions = [float(value) for value in primitive.attrib["size"].split()]
                self.assertEqual(len(dimensions), 3, name)
                self.assertTrue(all(value > 0.0 for value in dimensions), name)
            elif primitive.tag == "sphere":
                self.assertGreater(float(primitive.attrib["radius"]), 0.0, name)
            else:
                self.fail(f"Unsupported collision primitive {primitive.tag!r} on {name}")

    def test_joint_names_are_unique(self):
        root = expanded_urdf()
        joints = root.findall("joint")
        names = [joint.attrib["name"] for joint in joints]
        self.assertEqual(len(names), len(set(names)))
        self.assertEqual(len(joints), 24)


if __name__ == "__main__":
    unittest.main()
