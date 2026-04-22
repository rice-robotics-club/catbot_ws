import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from catbot_ik.gait_controller import catbot_crawl
from catbot_ik.gait_utils import configure_vel_to_gait_func

# Must match joint_trajectory_controller joint_names order in controllers.yaml
JOINT_NAMES = [
    'FL_hip', 'FL_a', 'FL_l',
    'FR_hip', 'FR_a', 'FR_l',
    'BL_hip', 'BL_a', 'BL_l',
    'BR_hip', 'BR_a', 'BR_l',
]

# Right legs (FR=1, BR=3) have mirrored motor directions vs. left legs
RIGHT_LEG_INDICES = {1, 3}

CALIBRATION_DURATION_S = 1.0


class GaitNode(Node):

    def __init__(self):
        super().__init__('gait_node')

        self.declare_parameter('rate_hz', 50.0)

        rate = self.get_parameter('rate_hz').value
        self.dt = 1.0 / rate

        self.gait_func = configure_vel_to_gait_func(
            step_length=[-5, -1],
            step_width=[-1, 1],
            step_height=[-25, -12],
        )

        self.vx = 0.0
        self.vy = 0.0
        self.turning = False
        self.phase = 0.0

        # Compute IK neutral output (vel=0, phase=0) to use as reference delta
        self._ik_neutral = self._compute_ik_neutral()

        # Calibration state
        self._calibrated = False
        self._calib_samples = []     # list of position dicts {joint_name: rad}
        self._hw_startup = {}        # {joint_name: rad} — hardware positions at boot

        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(JointState, 'joint_states', self._joint_state_cb, 10)

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            'joint_trajectory_controller/joint_trajectory',
            10,
        )

        # Calibration ends after CALIBRATION_DURATION_S, then control starts
        self.create_timer(CALIBRATION_DURATION_S, self._finish_calibration)
        self._control_timer = None

        self.get_logger().info(
            'Calibrating for %.1f s — hold robot in desired zero position.' % CALIBRATION_DURATION_S
        )

    # ── Calibration ──────────────────────────────────────────────────────────

    def _compute_ik_neutral(self):
        """IK output at vel=0, phase=0 — the reference point the calibration delta is measured from."""
        neutral_angles = catbot_crawl(
            vel=[0.0, 0.0],
            cycle_period=0.0,
            turning=False,
            generated_leg_gait_func=self.gait_func,
        )
        result = {}
        for i, angles in enumerate(neutral_angles):
            hip_deg, t_a_deg, t_l_deg = angles
            leg_joints = JOINT_NAMES[i * 3: i * 3 + 3]
            if hip_deg is None:
                for name in leg_joints:
                    result[name] = 0.0
                continue
            if i in RIGHT_LEG_INDICES:
                t_a_motor, t_l_motor = -t_a_deg, -t_l_deg
            else:
                t_a_motor, t_l_motor = t_a_deg, t_l_deg
            result[leg_joints[0]] = math.radians(hip_deg)
            result[leg_joints[1]] = math.radians(t_a_motor)
            result[leg_joints[2]] = math.radians(t_l_motor)
        return result

    def _joint_state_cb(self, msg: JointState):
        if self._calibrated:
            return
        sample = {name: pos for name, pos in zip(msg.name, msg.position)}
        self._calib_samples.append(sample)

    def _finish_calibration(self):
        if not self._calib_samples:
            self.get_logger().warn('No joint states received during calibration — using hardware zeros.')
            self._hw_startup = {name: 0.0 for name in JOINT_NAMES}
        else:
            sums = {name: 0.0 for name in JOINT_NAMES}
            counts = {name: 0 for name in JOINT_NAMES}
            for sample in self._calib_samples:
                for name in JOINT_NAMES:
                    if name in sample:
                        sums[name] += sample[name]
                        counts[name] += 1
            self._hw_startup = {
                name: (sums[name] / counts[name] if counts[name] > 0 else 0.0)
                for name in JOINT_NAMES
            }
            self.get_logger().info(
                'Calibration complete (%d samples). Hardware startup (deg): %s' % (
                    len(self._calib_samples),
                    {k: round(math.degrees(v), 1) for k, v in self._hw_startup.items()},
                )
            )

        self._calibrated = True
        self._control_timer = self.create_timer(self.dt, self._tick)

    # ── Control loop ─────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        lin_x, lin_y = msg.linear.x, msg.linear.y
        ang_z = msg.angular.z

        if abs(ang_z) > 0.01 and abs(lin_x) < 0.01 and abs(lin_y) < 0.01:
            self.turning = True
            self.vx = ang_z
            self.vy = 0.0
        else:
            self.turning = False
            self.vx = lin_x
            self.vy = lin_y

    def _ik_to_motor(self, leg_idx, hip_deg, t_a_deg, t_l_deg):
        """
        Convert IK output (degrees) to motor-frame radians.
        Right legs mirror the a and l axes relative to left legs.
        The commanded position is: hw_startup + (ik_output - ik_neutral),
        so the robot moves relative to its startup pose, not to the IK geometric zero.
        """
        leg_joints = JOINT_NAMES[leg_idx * 3: leg_idx * 3 + 3]

        if leg_idx in RIGHT_LEG_INDICES:
            t_a_motor = -t_a_deg
            t_l_motor = -t_l_deg
        else:
            t_a_motor = t_a_deg
            t_l_motor = t_l_deg

        ik_out = [
            math.radians(hip_deg),
            math.radians(t_a_motor),
            math.radians(t_l_motor),
        ]

        return [
            self._hw_startup[name] + (ik - self._ik_neutral[name])
            for ik, name in zip(ik_out, leg_joints)
        ]

    def _tick(self):
        self.phase = (self.phase + 2 * math.pi * self.dt) % (2 * math.pi)

        leg_angles = catbot_crawl(
            vel=[self.vx, self.vy],
            cycle_period=self.phase,
            turning=self.turning,
            generated_leg_gait_func=self.gait_func,
        )

        positions = []
        for i, angles in enumerate(leg_angles):
            hip_deg, t_a_deg, t_l_deg = angles
            if hip_deg is None:
                self.get_logger().warn('IK failed for leg %d, skipping cycle' % i)
                return
            positions += self._ik_to_motor(i, hip_deg, t_a_deg, t_l_deg)

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9))
        traj.points = [pt]

        self.traj_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
