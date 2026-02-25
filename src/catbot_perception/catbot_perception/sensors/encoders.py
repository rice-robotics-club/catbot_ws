''' dummy node. just returns 0 for all encoder readings. '''

NUM_JOINTS = 12
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')
        self.enc_pub = self.create_publisher(Float32MultiArray, 'encoders/data', 10)
        self.timer = self.create_timer(0.1, self.publish_enc)  # Publish encoder data at 10 Hz
    
    def publish_enc(self):
        enc_data = Float32MultiArray()
        enc_data.data = [0.0] * NUM_JOINTS  # Dummy encoder readings (all zeros)
        self.enc_pub.publish(enc_data)

def main(args=None):
    rclpy.init(args=args)
    encoder_node = EncoderNode()
    rclpy.spin(encoder_node)
    encoder_node.destroy_node()
    rclpy.shutdown()