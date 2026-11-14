#!/usr/bin/env python3
"""Non-blocking keyboard teleoperation for a bounded diagonal trot."""

import math
import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = [
    'motor_front_left_hip', 'motor_front_left_upper_leg', 'motor_front_left_lower_leg',
    'motor_front_right_hip', 'motor_front_right_upper_leg', 'motor_front_right_lower_leg',
    'motor_back_left_hip', 'motor_back_left_upper_leg', 'motor_back_left_lower_leg',
    'motor_back_right_hip', 'motor_back_right_upper_leg', 'motor_back_right_lower_leg',
]
LIMITS = [(-1.04, 1.04), (-math.pi / 2, 2.59), (-2.9, math.pi / 2)] * 4
STAND = [0.0] * 12
LEGS = {'fl': 0, 'fr': 3, 'bl': 6, 'br': 9}


class KeyboardGait(Node):
    def __init__(self):
        super().__init__('keyboard_gait')
        self.pub = self.create_publisher(
            JointTrajectory, '/quadruped_controller/joint_trajectory', 10
        )
        self.speed = 0.5
        self.command = (0.0, 0.0, 0.0)  # forward, lateral, yaw
        self.phase = 0.0
        self.last = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.tick)

    def key(self, key):
        if key == 'w': self.command = (1.0, 0.0, 0.0)
        elif key == 's': self.command = (-1.0, 0.0, 0.0)
        elif key == 'a': self.command = (0.0, 0.0, 1.0)
        elif key == 'd': self.command = (0.0, 0.0, -1.0)
        elif key == 'q': self.command = (0.0, 1.0, 0.0)
        elif key == 'e': self.command = (0.0, -1.0, 0.0)
        elif key == ' ':
            self.command = (0.0, 0.0, 0.0)
            self.phase = 0.0
        elif key == 'x':
            self.command = (0.0, 0.0, 0.0)
            self.publish([0.0, 0.35, -0.65] * 4)
        elif key in ('+', '='):
            self.speed = min(1.0, self.speed + 0.1)
        elif key == '-':
            self.speed = max(0.1, self.speed - 0.1)

    def publish(self, positions):
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = [max(lo, min(hi, float(v))) for v, (lo, hi) in zip(positions, LIMITS)]
        point.time_from_start.nanosec = 100_000_000
        msg.points = [point]
        self.pub.publish(msg)

    def tick(self):
        now = self.get_clock().now()
        dt = min(0.1, max(0.0, (now - self.last).nanoseconds / 1e9))
        self.last = now
        forward, lateral, yaw = self.command
        if forward == lateral == yaw == 0.0:
            self.publish(STAND)
            return
        self.phase = (self.phase + dt * (1.2 + self.speed)) % (2.0 * math.pi)
        positions = list(STAND)
        for leg, index in LEGS.items():
            diagonal = leg in ('fl', 'br')
            swing = math.sin(self.phase + (0.0 if diagonal else math.pi))
            lift = max(0.0, swing)
            side = 1.0 if leg.endswith('l') else -1.0
            front = 1.0 if leg.startswith('f') else -1.0
            positions[index] = 0.10 * (yaw * front - lateral * side) * (0.4 + 0.6 * lift)
            positions[index + 1] = -0.22 * forward * swing
            positions[index + 2] = 0.38 * forward * swing - 0.10 * lift
        self.publish(positions)


def read_key():
    ready, _, _ = select.select([sys.stdin], [], [], 0.02)
    return os.read(sys.stdin.fileno(), 1).decode(errors='ignore') if ready else None


def main(args=None):
    if not sys.stdin.isatty():
        raise RuntimeError('keyboard_gait must run in an interactive terminal')
    rclpy.init(args=args)
    node = KeyboardGait()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        print('w/s forward, a/d turn, q/e lateral, space stop/stand, x crouch, +/- speed, Ctrl-C quit')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key()
            if key:
                node.key(key.lower())
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.publish(STAND)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
