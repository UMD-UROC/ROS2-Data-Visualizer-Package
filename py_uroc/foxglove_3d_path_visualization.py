import os
import rclpy
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile , QoSReliabilityPolicy , QoSHistoryPolicy

from .qos_profile import BEST_EFFORT_QOS

# Load .env file from the package share directory
package_share_directory = get_package_share_directory('py_uroc')
load_dotenv(os.path.join(package_share_directory, '.env'))
REFRESH_RATE_HZ = float(os.getenv('REFRESH_RATE_HZ'))


class PathVisualizerNode(Node):
    """ROS2 node for visualizing drone path in Foxglove (global map frame)."""

    def __init__(self):
        super().__init__("path_visualizer_node")

        # Publishers: PoseStamped and Path, both in "map"
        self.drone_pose_pub = self.create_publisher(PoseStamped, "/drone/pose", 1)
        self.path = Path()
        self.path.header.frame_id = "map"
        self.path_pub = self.create_publisher(Path, "/drone/flight_path", 1)

        # State
        self.drone_pos = [0.0, 0.0, 0.0]
        self.drone_q = [0.0, 0.0, 0.0, 1.0]

        # Subscribe to MAVROS local_position (ENU) in map
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.mavros_pose_callback,
            BEST_EFFORT_QOS,
        )

        # Publish at 1 Hz
        self.timer = self.create_timer(REFRESH_RATE_HZ, self.publish_loop)

    def mavros_pose_callback(self, msg: PoseStamped):
        # Update latest pose and header
        self.drone_pos = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]
        self.drone_q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        self.latest_header = msg.header

    def publish_loop(self):
        # Timestamp from latest MAVROS message, or now
        if hasattr(self, "latest_header"):
            stamp = self.latest_header.stamp
        else:
            stamp = self.get_clock().now().to_msg()

        # Publish PoseStamped in "map"
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.drone_pos[0]
        pose_msg.pose.position.y = self.drone_pos[1]
        pose_msg.pose.position.z = self.drone_pos[2]
        pose_msg.pose.orientation.x = self.drone_q[0]
        pose_msg.pose.orientation.y = self.drone_q[1]
        pose_msg.pose.orientation.z = self.drone_q[2]
        pose_msg.pose.orientation.w = self.drone_q[3]
        self.drone_pose_pub.publish(pose_msg)

        # Append to Path and publish
        self.path.poses.append(pose_msg)
        self.path.header.stamp = stamp
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizerNode()
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
