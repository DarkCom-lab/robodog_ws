#!/usr/bin/env python3
"""Command the measured neutral standing pose before releasing Gazebo pause."""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = [
    'motor_front_left_hip', 'motor_front_left_upper_leg', 'motor_front_left_lower_leg',
    'motor_front_right_hip', 'motor_front_right_upper_leg', 'motor_front_right_lower_leg',
    'motor_back_left_hip', 'motor_back_left_upper_leg', 'motor_back_left_lower_leg',
    'motor_back_right_hip', 'motor_back_right_upper_leg', 'motor_back_right_lower_leg',
]

# The active URDF's zero joint configuration is the measured four-foot contact
# pose (spawn z=0.2422 m).  Holding this pose is safer than allowing the
# uncommanded position interfaces to let the articulated body fall at startup.
STAND = [
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
]


class StandController(Node):
    def __init__(self):
        super().__init__('stand_controller')
        self.pub = self.create_publisher(
            JointTrajectory, '/quadruped_controller/joint_trajectory', 10
        )
        self.sent = False
        self.timer = self.create_timer(0.05, self.tick)

    def tick(self):
        if self.sent or self.pub.get_subscription_count() == 0:
            return
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = STAND
        point.time_from_start.sec = 1
        msg.points = [point]
        self.pub.publish(msg)
        self.sent = True
        self.get_logger().info('Standing trajectory sent to the active controller.')


def main(args=None):
    rclpy.init(args=args)
    node = StandController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
