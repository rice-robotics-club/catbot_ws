from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='catbot_perception',
            executable='imu_node',
            name='imu_node',
            output='screen',
        ),
        Node(
            package='catbot_perception',
            executable='encoder_node',
            name='encoder_node',
            output='screen',
        ),
        Node(
            package='catbot_perception',
            executable='command_node',
            name='command_node',
            output='screen',
        ),
        Node(
            package='catbot_perception',
            executable='perceive',
            name='perceive_node',
            output='screen',
        ),
    ])
