'''
Bridges the /actions topic (Float32MultiArray) to /joint_states (JointState)
so that robot_state_publisher and RViz can visualize the model's output.

Joint names, default positions, and action scale are read from the training
config (via CATBOT_CONFIG → robot.yaml).  Each raw action value is converted
to a physical joint angle with the same formula used in simulation:

    target = clip(action * action_scale + default_dof_pos, lower, upper)

Joint names are sorted alphabetically to match genesis.py's sorted() ordering.
Joint limits are derived from the joint name suffix (±π/6 for *_bot, ±π/2 otherwise).
'''

import math
import os
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

# URDF joint limits derived from joint name suffix
_BOT_LIM = math.pi / 6   # *_bot joints: ±30°
_DEFAULT_LIM = math.pi / 2  # all others: ±90°


def _limit_for(name: str) -> float:
    return _BOT_LIM if name.endswith('_bot') else _DEFAULT_LIM


def load_training_config():
    workspace_dir = os.environ.get('CATBOT_WS', '/catbot-ros2')
    config_path = os.environ.get('CATBOT_CONFIG', os.path.join(workspace_dir, 'config', 'robot.yaml'))
    with open(config_path) as f:
        config = yaml.safe_load(f)
    training_config_path = os.path.join(workspace_dir, config['robot']['training_config'])
    with open(training_config_path) as f:
        training_config = yaml.safe_load(f)
    joints = training_config['env']['joints']
    names = sorted(joints.keys())
    defaults = [joints[n] for n in names]
    action_scale = training_config['env'].get('action_scale', 0.25)
    return names, defaults, action_scale


class ActionBridge(Node):
    def __init__(self):
        super().__init__('action_bridge')

        self.joint_names, self.default_dof_pos, self.action_scale = load_training_config()
        self.limits = [_limit_for(n) for n in self.joint_names]

        self.sub = self.create_subscription(
            Float32MultiArray,
            'actions',
            self.actions_callback,
            10
        )
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.get_logger().info(
            f'Action bridge ready: {len(self.joint_names)} joints, action_scale={self.action_scale}'
        )

    def actions_callback(self, msg):
        raw = list(msg.data[:len(self.joint_names)])
        positions = [
            max(-lim, min(lim, a * self.action_scale + d))
            for a, d, lim in zip(raw, self.default_dof_pos, self.limits)
        ]
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = positions
        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = ActionBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
