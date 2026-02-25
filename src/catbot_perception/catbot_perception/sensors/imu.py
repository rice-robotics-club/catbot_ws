''' 
Dummy node. Just returns a flat plane for all IMU readings.
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.1, self.publish_imu)  # Publish IMU data at 10 Hz
    
    def publish_imu(self):
        imu_data = Imu()
        imu_data.orientation.x = 0.0
        imu_data.orientation.y = 0.0
        imu_data.orientation.z = 0.0
        imu_data.orientation.w = 1.0  # No rotation (flat plane)
        imu_data.angular_velocity.x = 0.0
        imu_data.angular_velocity.y = 0.0
        imu_data.angular_velocity.z = 0.0  # No angular velocity
        imu_data.linear_acceleration.x = 0.0
        imu_data.linear_acceleration.y = 0.0
        imu_data.linear_acceleration.z = -9.81  # Gravity pointing downwards
        self.imu_pub.publish(imu_data)

def main(args=None):
    rclpy.init(args=args)
    imu_node = ImuNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

