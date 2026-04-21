from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    gait_node = Node(
        package="catbot_ik",
        executable="gait_node",
        output="both",
    )

    delay_gait_node_after_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[gait_node],
        )
    )

    nodes = [
        joint_trajectory_controller_spawner,
        delay_gait_node_after_joint_trajectory_controller,
    ]

    return LaunchDescription(nodes)
