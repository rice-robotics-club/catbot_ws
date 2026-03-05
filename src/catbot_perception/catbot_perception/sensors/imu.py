import adafruit_icm20x
import board
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        self.timer_rate: float = (
            self.declare_parameter("timer_rate", 0.1).get_parameter_value().double_value
        )
        self.imu_frame: str = (
            self.declare_parameter("imu_frame", "imu_link")
            .get_parameter_value()
            .string_value
        )
        self._i2c = board.I2C()
        self._icm = adafruit_icm20x.ICM20948(self._i2c)
        self.imu_pub = self.create_publisher(Imu, "imu/raw", 10)
        self._imu = Imu()
        self._imu.header.frame_id = self.imu_frame
        self.mag_pub = self.create_publisher(MagneticField, "imu/magnetic_field", 10)
        self._mag = MagneticField()
        self._mag.header.frame_id = self.imu_frame
        self.timer = self.create_timer(self.timer_rate, self.publish)

    def publish(self):
        self._imu.header.stamp = self.get_clock().now().to_msg()
        self._imu.angular_velocity.x = self._icm.gyro[0]
        self._imu.angular_velocity.y = self._icm.gyro[1]
        self._imu.angular_velocity.z = self._icm.gyro[2]
        self._imu.linear_acceleration.x = self._icm.acceleration[0]
        self._imu.linear_acceleration.y = self._icm.acceleration[1]
        self._imu.linear_acceleration.z = self._icm.acceleration[2]
        self.imu_pub.publish(self._imu)

        self._mag.header.stamp = self.get_clock().now().to_msg()
        self._mag.magnetic_field.x = self._icm.magnetic[0]
        self._mag.magnetic_field.y = self._icm.magnetic[1]
        self._mag.magnetic_field.z = self._icm.magnetic[2]
        self.mag_pub.publish(self._mag)


def main(args=None):
    rclpy.init(args=args)
    imu_node = ImuNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()
