"""
Drone flight path visualization for UROC system.

This module provides real-time visualization of drone flight paths by collecting
pose data and creating path trails for display in 3D visualization tools like
Foxglove. It publishes both individual pose messages and accumulated path data
in the global map frame.
"""

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node

from .qos_profile import BEST_EFFORT_QOS

# Load environment configuration from the package share directory
package_share_directory = get_package_share_directory("umd_uroc_data_visualizer")
load_dotenv(os.path.join(package_share_directory, ".env"))
# Refresh rate for path visualization updates (Hz)
REFRESH_RATE_HZ = float(os.getenv("REFRESH_RATE_HZ"))


class PathVisualizer(Node):
    """
    ROS2 node for visualizing drone flight paths in global map frame.

    This node subscribes to MAVROS pose data, republishes it on a standardized
    topic, and maintains an accumulated path trail for visualization in tools
    like Foxglove. The path grows continuously as the drone moves.

    Attributes
    ----------
    drone_pose_pub : Publisher
        Publisher for individual drone pose messages
    path : Path
        Accumulated path message containing flight history
    path_pub : Publisher
        Publisher for flight path visualization
    drone_pos : list
        Current drone position [x, y, z] in map frame
    drone_q : list
        Current drone orientation quaternion [x, y, z, w]
    latest_header : Header
        Most recent message header for timestamping
    timer : Timer
        Timer for periodic publication updates

    """

    def __init__(self):
        """Initialize the PathVisualizer node."""
        super().__init__("path_visualizer_node")

        # Initialize publishers for pose and path data
        self.drone_pose_pub = self.create_publisher(PoseStamped, "/drone/pose", 1)

        # Initialize path message for accumulating flight trail
        self.path = Path()
        self.path.header.frame_id = "map"  # Global ENU reference frame
        self.path_pub = self.create_publisher(Path, "/drone/flight_path", 1)

        # Initialize state variables for current drone pose
        self.drone_pos = [0.0, 0.0, 0.0]  # Position in map frame (m)
        self.drone_q = [0.0, 0.0, 0.0, 1.0]  # Orientation quaternion

        # Subscribe to MAVROS local position data (already in ENU map frame)
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.mavros_pose_callback,
            BEST_EFFORT_QOS,
        )

        # Timer for periodic pose and path publication
        self.timer = self.create_timer(REFRESH_RATE_HZ, self.publish_loop)

    def mavros_pose_callback(self, msg: PoseStamped):
        """
        Handle incoming MAVROS pose messages.

        Updates the current drone pose and header information from MAVROS.
        The pose is already in the ENU map frame from MAVROS processing.

        Parameters
        ----------
        msg : PoseStamped
            Pose message from MAVROS local_position topic

        """
        # Extract position from pose message
        self.drone_pos = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]

        # Extract orientation quaternion
        self.drone_q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]

        # Store header for consistent timestamping
        self.latest_header = msg.header

    def publish_loop(self):
        """
        Publish pose and path data.

        Publishes the current drone pose and adds it to the accumulated path
        for continuous trail visualization. Uses consistent timestamping
        across all published messages.
        """
        # Use timestamp from latest MAVROS message, or current time as fallback
        if hasattr(self, "latest_header"):
            stamp = self.latest_header.stamp
        else:
            stamp = self.get_clock().now().to_msg()

        # Create and publish current pose message in map frame
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "map"  # Global ENU reference frame

        # Set position from current state
        pose_msg.pose.position.x = self.drone_pos[0]
        pose_msg.pose.position.y = self.drone_pos[1]
        pose_msg.pose.position.z = self.drone_pos[2]

        # Set orientation from current state
        pose_msg.pose.orientation.x = self.drone_q[0]
        pose_msg.pose.orientation.y = self.drone_q[1]
        pose_msg.pose.orientation.z = self.drone_q[2]
        pose_msg.pose.orientation.w = self.drone_q[3]

        # Publish current pose
        self.drone_pose_pub.publish(pose_msg)

        # Add current pose to accumulated path trail
        self.path.poses.append(pose_msg)
        self.path.header.stamp = stamp

        # Publish updated path for trail visualization
        self.path_pub.publish(self.path)


def main(args=None):
    """
    Execute the path visualizer node.

    Parameters
    ----------
    args : list, optional
        Command line arguments

    """
    rclpy.init(args=args)
    node = PathVisualizer()
    node.get_logger().info("UROC Foxglove 3D Path Visualization Node started")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
