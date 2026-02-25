'''
Launch file for 3D robot visualization.
Reads robot config from CATBOT_CONFIG env var (default: CATBOT_WS/config/robot.yaml).
Starts: robot_state_publisher, rviz2, action_bridge.
'''

import os
import yaml
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    workspace_dir = Path(os.environ.get('CATBOT_WS', '/catbot-ros2'))
    config_path = os.environ.get('CATBOT_CONFIG', str(workspace_dir / 'config' / 'robot.yaml'))

    with open(config_path) as f:
        config = yaml.safe_load(f)

    urdf_path = workspace_dir / config['robot']['urdf_path']
    rviz_config = workspace_dir / config['robot']['rviz_config']

    with open(urdf_path) as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', str(rviz_config)]
        ),
        Node(
            package='rsl_runner',
            executable='action_bridge',
            output='screen'
        ),
    ])
