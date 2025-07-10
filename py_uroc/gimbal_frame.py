# Python file to create gimbal_frame
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# Load .env file from the package share directory
package_share_directory = get_package_share_directory("py_uroc")
load_dotenv(os.path.join(package_share_directory, ".env"))
REFRESH_RATE_HZ = float(os.getenv("REFRESH_RATE_HZ"))


class GimbalFrame(Node):
    def __init__(self):
        super().__init__("gimbal_frame")
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create timer to publish transform at 50Hz
        self.timer = self.create_timer(REFRESH_RATE_HZ, self.publish_loop)

    def publish_loop(self):
        # Get current time
        stamp = self.get_clock().now().to_msg()

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "base_link"  # Changed from "map" to "base_link"
        tf_msg.child_frame_id = "gimbal_frame"
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        # Set identity quaternion (no rotation)
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_msg)


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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
