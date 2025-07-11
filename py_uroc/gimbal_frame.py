"""Gimbal frame transform publisher for UROC visualization.

This module publishes a static transform between the drone's base_link frame
and a gimbal_frame. This provides a reference frame for gimbal visualization
markers and allows for proper coordinate system representation in 3D viewers
like Foxglove.
"""

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# Load environment configuration from the package share directory
package_share_directory = get_package_share_directory("py_uroc")
load_dotenv(os.path.join(package_share_directory, ".env"))
# Refresh rate for transform publication (Hz)
REFRESH_RATE_HZ = float(os.getenv("REFRESH_RATE_HZ"))


class GimbalFrame(Node):
    """ROS2 node for publishing gimbal frame transforms.

    This node publishes a static transform from base_link to gimbal_frame
    at a regular interval. The gimbal_frame serves as a reference coordinate
    system for gimbal-related visualizations and transformations.

    Attributes:
        tf_broadcaster: Transform broadcaster for publishing TF data
        timer: Timer for periodic transform publication
    """

    def __init__(self):
        """Initialize the GimbalFrame node."""
        super().__init__("gimbal_frame")
        # Initialize transform broadcaster for publishing coordinate frames
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create timer for periodic transform publication
        self.timer = self.create_timer(REFRESH_RATE_HZ, self.publish_loop)

    def publish_loop(self):
        """Publish the base_link to gimbal_frame transform.

        Creates and publishes a static transform with no translation or rotation,
        establishing gimbal_frame as coincident with base_link. This provides
        a stable reference frame for gimbal visualization markers.
        """
        # Get current time for transform timestamp
        stamp = self.get_clock().now().to_msg()

        # Create transform message
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "base_link"  # Parent frame
        tf_msg.child_frame_id = "gimbal_frame"  # Child frame

        # Set zero translation (gimbal frame coincident with base_link)
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0

        # Set identity quaternion (no rotation relative to base_link)
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0

        # Publish the transform
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    """Main entry point for the gimbal frame publisher node.

    Args:
        args: Command line arguments (optional)
    """
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
