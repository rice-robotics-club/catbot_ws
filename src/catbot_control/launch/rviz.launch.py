from this import s

from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("catbot_leg_description"),
            "rviz",
            "preview.rviz",
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        parameters=[{"source_list": ["/hardware_joint_states"]}],
    )

    nodes = [
        rviz_node,
        joint_state_publisher_gui_node,
    ]

    return LaunchDescription(nodes)
