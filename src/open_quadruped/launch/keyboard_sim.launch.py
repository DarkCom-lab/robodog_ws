"""Start Gazebo with the keyboard locomotion controller."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('open_quadruped'), 'launch', 'sim.launch.py'
            ])),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'joystick_control': 'false',
            }.items(),
        ),
        Node(
            package='open_quadruped',
            executable='joint_angles_to_trajectory.py',
            name='joint_angles_to_trajectory',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='open_quadruped',
            executable='keyboard_gait.py',
            name='keyboard_gait',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True}],
        ),
    ])
