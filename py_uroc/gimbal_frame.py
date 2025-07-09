# Python file to create gimbal_frame
import rclpy
from geometry_msgs.msg import TransformStamped, Point, PoseStamped
from mavros_msgs.msg import GimbalDeviceSetAttitude
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


class GimbalFrame(Node):
    def __init__(self):
        super().__init__("gimbal_frame")
        self.tf_broadcaster = TransformBroadcaster(self)

    def publish_loop(self):
        stamp = self.get_clock().now().to_msg()


def main(args=None):
    rclpy.init(args=args)
    node = GimbalFrame()
    node.get_logger().info("Gimbal Frame Publisher Started")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
