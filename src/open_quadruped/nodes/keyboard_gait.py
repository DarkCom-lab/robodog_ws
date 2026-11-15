#!/usr/bin/env python3
"""Keyboard teleoperation using a statically conservative crawl gait."""

import math
import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, TransformException

from control_library import leg_ik, velocity_gait
from open_quadruped.msg import JointAngles


LEGS = ("fl", "fr", "bl", "br")
LEG_JOINTS = {
    "fl": ("motor_front_left_hip", "motor_front_left_upper_leg", "motor_front_left_lower_leg"),
    "fr": ("motor_front_right_hip", "motor_front_right_upper_leg", "motor_front_right_lower_leg"),
    "bl": ("motor_back_left_hip", "motor_back_left_upper_leg", "motor_back_left_lower_leg"),
    "br": ("motor_back_right_hip", "motor_back_right_upper_leg", "motor_back_right_lower_leg"),
}
FOOT_FRAMES = {
    "fl": "front_left_foot",
    "fr": "front_right_foot",
    "bl": "back_left_foot",
    "br": "back_right_foot",
}
# Origins of the actuated hip joints relative to base_link, in metres.
HIP_ORIGINS = {
    "fl": (0.0915, 0.0394, 0.022),
    "fr": (0.0915, -0.0394, 0.022),
    "bl": (-0.1365, 0.0394, 0.022),
    "br": (-0.1365, -0.0394, 0.022),
}
# FL+BR and FR+BL are the two diagonal pairs.
PHASE_OFFSETS = {"fl": 0.75, "br": 0.50, "fr": 0.25, "bl": 0.0}
JOINT_LIMITS = (
    (-1.04, 1.04),
    (-math.pi / 2.0, 2.59),
    (-2.9, math.pi / 2.0),
)


class KeyboardGait(Node):
    def __init__(self):
        super().__init__("keyboard_gait")
        self.declare_parameter("step_period_sec", 1.2)
        self.declare_parameter("stance_ratio", 0.82)
        self.declare_parameter("swing_height_m", 0.015)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("joint_angles_topic", "/joint_angles")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("max_forward_speed_mps", 0.04)
        self.declare_parameter("max_lateral_speed_mps", 0.025)
        self.declare_parameter("max_yaw_rate_rps", 0.35)
        self.declare_parameter("command_ramp_sec", 0.30)
        self.declare_parameter("max_foot_step_m", 0.06)

        self.step_period = float(self.get_parameter("step_period_sec").value)
        self.stance_ratio = float(self.get_parameter("stance_ratio").value)
        self.swing_height = float(self.get_parameter("swing_height_m").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.max_forward_speed = float(self.get_parameter("max_forward_speed_mps").value)
        self.max_lateral_speed = float(self.get_parameter("max_lateral_speed_mps").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate_rps").value)
        self.command_ramp = float(self.get_parameter("command_ramp_sec").value)
        self.max_foot_step = float(self.get_parameter("max_foot_step_m").value)

        for name, value in (
            ("step_period_sec", self.step_period),
            ("publish_rate_hz", rate),
            ("max_forward_speed_mps", self.max_forward_speed),
            ("max_lateral_speed_mps", self.max_lateral_speed),
            ("max_yaw_rate_rps", self.max_yaw_rate),
            ("command_ramp_sec", self.command_ramp),
            ("max_foot_step_m", self.max_foot_step),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be a positive finite number")
        if not 0.0 < self.stance_ratio < 1.0:
            raise ValueError("stance_ratio must be between zero and one")
        if not math.isfinite(self.swing_height) or not 0.0 < self.swing_height <= 0.08:
            raise ValueError("swing_height_m must be in (0, 0.08]")

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ik = leg_ik.LegIKModel(109.868, 144.580, 11.369, 63.763)
        self.publisher = self.create_publisher(
            JointAngles, str(self.get_parameter("joint_angles_topic").value), 10
        )
        self.joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._on_joint_state,
            10,
        )

        self.home_feet = {}
        self.home_ik = {}
        self.home_joints = {}
        self.ready = False

        # Normalized velocity request: forward (+X), lateral (+Y), yaw (+CCW).
        self.requested_twist = (0.0, 0.0, 0.0)
        self.active_twist = [0.0, 0.0, 0.0]
        self.speed_scale = 0.5
        self.phase = 0.0
        self.movement_active = False
        self.stopping = False
        self.last_tick = self.get_clock().now()
        self.timer = self.create_timer(1.0 / rate, self._tick)

    def _on_joint_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(name in values for names in LEG_JOINTS.values() for name in names):
            self.joint_state = {
                name: float(values[name])
                for names in LEG_JOINTS.values()
                for name in names
            }

    def _read_feet(self):
        feet = {}
        for leg, frame in FOOT_FRAMES.items():
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, frame, Time()
            )
            point = transform.transform.translation
            feet[leg] = [float(point.x), float(point.y), float(point.z)]
        return feet

    def _calculate_ik(self, feet):
        angles = {}
        for leg in LEGS:
            hx, hy, hz = HIP_ORIGINS[leg]
            fx, fy, fz = feet[leg]
            vector_mm = (
                (hx - fx) * 1000.0,
                (hy - fy) * 1000.0,
                (hz - fz) * 1000.0,
            )
            angles[leg] = self.ik.ja_from_htf_vecs([vector_mm])[0]
        return angles

    def _initialize(self):
        if self.joint_state is None:
            return False
        try:
            feet = self._read_feet()
            home_ik = self._calculate_ik(feet)
            self.home_feet = {leg: list(point) for leg, point in feet.items()}
            self.home_ik = home_ik
            self.home_joints = {
                leg: [self.joint_state[name] for name in LEG_JOINTS[leg]]
                for leg in LEGS
            }
            self.ready = True
            self.get_logger().info(
                "Foot TF and joint feedback ready. W/S forward, A/D turn, Q/E strafe."
            )
            return True
        except (TransformException, ArithmeticError, ValueError) as exc:
            self.get_logger().warning(f"Waiting for foot TF/IK feedback: {exc}")
            return False

    @staticmethod
    def _approach(current, target, step):
        difference = target - current
        return current + max(-step, min(step, difference))

    def set_command(self, forward=0.0, lateral=0.0, yaw=0.0):
        self.requested_twist = tuple(
            max(-1.0, min(1.0, float(value)))
            for value in (forward, lateral, yaw)
        )
        if any(abs(value) > 0.0 for value in self.requested_twist):
            if not self.movement_active:
                self.phase = 0.0
                self.active_twist = [0.0, 0.0, 0.0]
                self.last_tick = self.get_clock().now()
                self.movement_active = True
            self.stopping = False
        else:
            self.stopping = self.movement_active

    def request_stop(self):
        if self.movement_active:
            self.set_command()
            self.get_logger().info("Decelerating gait; will stop when all four feet are down.")

    def handle_key(self, key):
        key = key.lower()
        if key == "w":
            self.set_command(forward=1.0)
        elif key == "s":
            self.set_command(forward=-1.0)
        elif key == "a":
            self.set_command(yaw=1.0)
        elif key == "d":
            self.set_command(yaw=-1.0)
        elif key == "q":
            self.set_command(lateral=1.0)
        elif key == "e":
            self.set_command(lateral=-1.0)
        elif key in (" ", "x"):
            self.request_stop()
        elif key in ("+", "="):
            self.speed_scale = min(1.0, self.speed_scale + 0.1)
            self.get_logger().info(f"Speed scale: {self.speed_scale:.1f}")
        elif key == "-":
            self.speed_scale = max(0.1, self.speed_scale - 0.1)
            self.get_logger().info(f"Speed scale: {self.speed_scale:.1f}")

    def _publish_targets(self, feet):
        matrix = {}
        for leg in LEGS:
            hx, hy, hz = HIP_ORIGINS[leg]
            fx, fy, fz = feet[leg]
            vector_mm = (
                (hx - fx) * 1000.0,
                (hy - fy) * 1000.0,
                (hz - fz) * 1000.0,
            )
            requested = self.ik.ja_from_htf_vecs([vector_mm])[0]
            delta = [
                requested[index] - self.home_ik[leg][index]
                for index in range(3)
            ]
            matrix[leg] = [
                self.home_joints[leg][index] + delta[index]
                for index in range(3)
            ]

        for leg in LEGS:
            if any(
                value < lower or value > upper
                for value, (lower, upper) in zip(matrix[leg], JOINT_LIMITS)
            ):
                raise ValueError(f"{leg.upper()} gait target would exceed a joint limit")

        msg = JointAngles()
        msg.fl = [float(value) for value in matrix["fl"]]
        msg.fr = [float(value) for value in matrix["fr"]]
        msg.bl = [float(value) for value in matrix["bl"]]
        msg.br = [float(value) for value in matrix["br"]]
        self.publisher.publish(msg)

    def _finish_stop(self):
        try:
            feet = self._read_feet()
            home_ik = self._calculate_ik(feet)
        except (TransformException, ArithmeticError, ValueError) as exc:
            self.get_logger().warning(f"Could not capture stop pose: {exc}")
            return

        self.home_feet = {leg: list(point) for leg, point in feet.items()}
        self.home_ik = home_ik
        self.home_joints = {
            leg: [self.joint_state[name] for name in LEG_JOINTS[leg]]
            for leg in LEGS
        }
        self.active_twist = [0.0, 0.0, 0.0]
        self.requested_twist = (0.0, 0.0, 0.0)
        self.phase = 0.0
        self.movement_active = False
        self.stopping = False
        self.get_logger().info("Crawl stopped in all-feet stance; controller holds the measured pose.")

    def _tick(self):
        if not self.ready and not self._initialize():
            return
        if not self.movement_active:
            return

        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds / 1e9
        self.last_tick = now
        dt = max(0.0, min(0.1, dt))
        if dt <= 0.0:
            return

        ramp_step = dt / self.command_ramp
        for index, target in enumerate(self.requested_twist):
            self.active_twist[index] = self._approach(
                self.active_twist[index], target, ramp_step
            )
        self.phase = (self.phase + dt / self.step_period) % 1.0

        forward = self.active_twist[0] * self.speed_scale * self.max_forward_speed
        lateral = self.active_twist[1] * self.speed_scale * self.max_lateral_speed
        yaw_rate = self.active_twist[2] * self.speed_scale * self.max_yaw_rate
        stance_duration = self.step_period * self.stance_ratio
        
        targets = {}
        try:
            for leg in LEGS:
                home = self.home_feet[leg]
                dx, dy = velocity_gait.stance_displacement(
                    home[:2], forward, lateral, yaw_rate, stance_duration
                )
                stride = math.hypot(dx, dy)
                if stride > self.max_foot_step:
                    scale = self.max_foot_step / stride
                    dx *= scale
                    dy *= scale

                offset = velocity_gait.relative_foot_path(
                    self.phase,
                    PHASE_OFFSETS[leg],
                    dx,
                    dy,
                    self.stance_ratio,
                    self.swing_height,
                )
                targets[leg] = [
                    home[0] + offset[0],
                    home[1] + offset[1],
                    home[2] + offset[2],
                ]

            self._publish_targets(targets)
        except (ArithmeticError, TransformException, ValueError) as exc:
            self.requested_twist = (0.0, 0.0, 0.0)
            self.stopping = True
            self.get_logger().error(f"Gait command rejected; decelerating: {exc}")
            return

        if (
            self.stopping
            and max(abs(value) for value in self.active_twist) < 1e-3
            and velocity_gait.in_all_feet_stance(self.phase, PHASE_OFFSETS, self.stance_ratio)
        ):
            self._finish_stop()


def read_key():
    ready, _, _ = select.select([sys.stdin], [], [], 0.02)
    if not ready:
        return None
    first = os.read(sys.stdin.fileno(), 1)
    if first != b"\x1b":
        return first.decode(errors="ignore")
    if not select.select([sys.stdin], [], [], 0.02)[0]:
        return "escape"
    os.read(sys.stdin.fileno(), 1)  # '['
    tail = os.read(sys.stdin.fileno(), 1)
    return {b"A": "w", b"B": "s", b"C": "d", b"D": "a"}.get(tail, "escape")


def main(args=None):
    if not sys.stdin.isatty():
        raise RuntimeError("keyboard_gait must run in an interactive terminal")
    rclpy.init(args=args)
    node = KeyboardGait()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    exiting = False
    try:
        tty.setcbreak(fd)
        print(
            "W/S forward/back | A/D turn | Q/E strafe | SPACE/X stop | +/- speed | ESC quit"
        )
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key()
            if key:
                if key == "escape":
                    node.request_stop()
                    exiting = True
                else:
                    node.handle_key(key)
            if exiting and not node.movement_active:
                break
    except KeyboardInterrupt:
        node.request_stop()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
