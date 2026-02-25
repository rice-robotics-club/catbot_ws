import os
import math
import yaml
import glob
import importlib

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

PUBLISHING_RATE = 10.0  # Hz

def load_training_config():
    workspace_dir = os.environ.get('CATBOT_WS', '/catbot-ros2')
    config_path = os.environ.get('CATBOT_CONFIG', os.path.join(workspace_dir, 'config', 'robot.yaml'))
    with open(config_path) as f:
        robot_cfg = yaml.safe_load(f)
    training_cfg_path = os.path.join(workspace_dir, robot_cfg['robot']['training_config'])
    with open(training_cfg_path) as f:
        return yaml.safe_load(f)


def projected_gravity(qx, qy, qz, qw):
    """Rotate world gravity [0,0,-1] into body frame using inverse of orientation quaternion."""
    # R column 2 (body-to-world): [2(xz+wy), 2(yz-wx), 1-2(x²+y²)]
    # projected_gravity = -R_col2 = R^T * [0,0,-1]
    return [
        -(2.0 * (qx * qz + qw * qy)),
        -(2.0 * (qy * qz - qw * qx)),
        -(1.0 - 2.0 * (qx * qx + qy * qy)),
    ]


class PerceiveNode(Node):
    def __init__(self):
        super().__init__('perceive_node')

        training_cfg = load_training_config()
        joints = training_cfg['env']['joints']
        self.default_dof_pos = list(joints.values())   # 12 defaults in training order

        scales = training_cfg['obs']['scales']
        self.scale_ang_vel  = scales.get('ang_vel_z', 0.25)
        self.scale_dof_pos  = scales.get('dof_pos',  1.0)
        self.scale_dof_vel  = scales.get('dof_vel',  0.5)
        self.cmd_scales = [
            scales.get('lin_vel_x', 2.0),
            scales.get('lin_vel_y', 2.0),
            scales.get('ang_vel_z', 0.25),
        ]

        num_joints = len(self.default_dof_pos)

        # Sensor state
        self.imu_data      = None  # Imu msg
        self.encoder_pos   = [0.0] * num_joints
        self.encoder_vel   = [0.0] * num_joints
        self.cmd_data      = [0.0, 0.0, 0.0]          # [lin_x, lin_y, ang_z]
        self.last_actions  = [0.0] * num_joints

        self.create_subscription(Imu,              'imu/data',      self._cb_imu,      10)
        self.create_subscription(Float32MultiArray, 'encoders/data', self._cb_encoders, 10)
        self.create_subscription(Twist,             'commands/data', self._cb_commands, 10)
        self.create_subscription(Float32MultiArray, 'actions',       self._cb_actions,  10)

        self.obs_pub = self.create_publisher(Float32MultiArray, 'obs', 10)
        self.create_timer(1.0 / PUBLISHING_RATE, self._publish_obs)

        self.get_logger().info(
            f'PerceiveNode ready: {num_joints} joints, obs_size=45'
        )

    def _cb_imu(self, msg):
        self.imu_data = msg

    def _cb_encoders(self, msg):
        n = len(self.default_dof_pos)
        data = list(msg.data)
        self.encoder_pos = data[:n] if len(data) >= n else data + [0.0] * (n - len(data))

    def _cb_commands(self, msg):
        self.cmd_data = [msg.linear.x, msg.linear.y, msg.angular.z]

    def _cb_actions(self, msg):
        n = len(self.default_dof_pos)
        data = list(msg.data)
        self.last_actions = data[:n] if len(data) >= n else data + [0.0] * (n - len(data))

    def _publish_obs(self):
        # --- angular velocity (3) ---
        if self.imu_data is not None:
            av = self.imu_data.angular_velocity
            ang_vel = [av.x * self.scale_ang_vel,
                       av.y * self.scale_ang_vel,
                       av.z * self.scale_ang_vel]
            o = self.imu_data.orientation
            grav = projected_gravity(o.x, o.y, o.z, o.w)
        else:
            ang_vel = [0.0, 0.0, 0.0]
            grav    = [0.0, 0.0, -1.0]

        # --- commands (3) ---
        cmd = [self.cmd_data[i] * self.cmd_scales[i] for i in range(3)]

        # --- dof_pos offset (12) ---
        dof_pos = [(self.encoder_pos[i] - self.default_dof_pos[i]) * self.scale_dof_pos
                   for i in range(len(self.default_dof_pos))]

        # --- dof_vel (12) ---
        dof_vel = [v * self.scale_dof_vel for v in self.encoder_vel]

        # --- last actions (12) ---
        obs_vec = ang_vel + grav + cmd + dof_pos + dof_vel + self.last_actions

        msg_out = Float32MultiArray()
        msg_out.data = [float(x) for x in obs_vec]
        self.obs_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = PerceiveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
