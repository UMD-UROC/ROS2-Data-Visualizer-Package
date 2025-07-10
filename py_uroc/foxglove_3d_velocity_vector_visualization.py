# import message_filters

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from geometry_msgs.msg import PoseStamped , Point
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from visualization_msgs.msg import Marker

from .qos_profile import BEST_EFFORT_QOS

# Load .env file from the package share directory
package_share_directory = get_package_share_directory("py_uroc")
load_dotenv(os.path.join(package_share_directory, ".env"))
REFRESH_RATE_HZ = float(os.getenv("REFRESH_RATE_HZ"))


class VelocityVectorVisualizer(Node):
    """ROS2 node that draws a green arrow from drone to target in map frame."""

    def __init__(self):
        super().__init__("vector_visualizer_node")
        self.drone_velocity = [0.0, 0.0, 0.0]
        self.drone_pos = [0.0, 0.0, 0.0]
        self.target_velocity = [0.0, 0.0, 0.0]
        self.target_pos = [0.0, 0.0, 0.0]

        # Sync PositionTarget (map) and drone PoseStamped (map)
        self.create_subscription(
            PositionTarget,
            "/mavros/setpoint_raw/local",
            self.on_local_position,
            BEST_EFFORT_QOS,
        )
        self.create_subscription(
            PoseStamped, "/drone/pose", self.on_drone_pos, BEST_EFFORT_QOS
        )

        self.marker_pub = self.create_publisher(
            Marker, "/drone/velocity_vector/marker", 1
        )

        self.timer = self.create_timer(REFRESH_RATE_HZ, self.publish_loop)

    def mavV_to_rosV(self, mavV):
        return [mavV[1], mavV[0], -mavV[2]]

    def on_local_position(self, target_msg: PositionTarget):
        self.drone_velocity = self.mavV_to_rosV(
            [target_msg.velocity.x, target_msg.velocity.y, target_msg.velocity.z]
        )

    def on_drone_pos(self, drone_pose_msg: PoseStamped):
        self.drone_pos = [
            drone_pose_msg.pose.position.x,
            drone_pose_msg.pose.position.y,
            drone_pose_msg.pose.position.z,
        ]

    def publish_loop(self):
        if hasattr(self, "latest_header"):
            stamp = self.latest_header.stamp
        else:
            stamp = self.get_clock().now().to_msg()

            target_pos = [
                self.drone_pos[0] + self.drone_velocity[0],
                self.drone_pos[1] + self.drone_velocity[1],
                self.drone_pos[2] + self.drone_velocity[2],
            ]

            # Build arrow marker in "map"
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = "map"
            marker.ns = "velocity_vector_arrow"
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            # Absolute start/end in ENU
            start_point = Point(
                x=self.drone_pos[0], y=self.drone_pos[1], z=self.drone_pos[2]
            )
            end_point = Point(
                x=self.target_pos[0], y=self.target_pos[1], z=self.target_pos[2]
            )
            marker.points = [start_point, end_point]

            # Arrow style
            marker.scale.x = 0.1
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            self.marker_pub.publish(marker)

    # # -------------------------------------------------------------------------------------
    #
    # def synchronized_callback(
    #     self, target_msg: PositionTarget, drone_pose_msg: PoseStamped
    # ):
    #     stamp = drone_pose_msg.header.stamp
    #
    #     # Extract global ENU positions
    #     drone_pos = [
    #         drone_pose_msg.pose.position.x,
    #         drone_pose_msg.pose.position.y,
    #         drone_pose_msg.pose.position.z,
    #     ]
    #
    #     # Use velocity from PositionTarget message to create velocity vector
    #     drone_velocity = mavV_to_rosV(
    #         [target_msg.velocity.x, target_msg.velocity.y, target_msg.velocity.z]
    #     )
    #
    #     # Create target position by adding velocity vector to current position
    #     # This creates a velocity vector visualization
    #     target_pos = [
    #         drone_pos[0] + drone_velocity[0],
    #         drone_pos[1] + drone_velocity[1],
    #         drone_pos[2] + drone_velocity[2],
    #     ]
    #
    #     # Build arrow marker in "map"
    #     marker = Marker()
    #     marker.header.stamp = stamp
    #     marker.header.frame_id = "map"
    #     marker.ns = "vector_arrow"
    #     marker.id = 0
    #     marker.type = Marker.ARROW
    #     marker.action = Marker.ADD
    #
    #     # Absolute start/end in ENU
    #     start_point = Point(x=drone_pos[0], y=drone_pos[1], z=drone_pos[2])
    #     end_point = Point(x=target_pos[0], y=target_pos[1], z=target_pos[2])
    #     marker.points = [start_point, end_point]
    #
    #     # Arrow style
    #     marker.scale.x = 0.1
    #     marker.scale.y = 0.2
    #     marker.scale.z = 0.2
    #     marker.color.r = 0.0
    #     marker.color.g = 1.0
    #     marker.color.b = 0.0
    #     marker.color.a = 1.0
    #
    #     self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityVectorVisualizer()
    node.get_logger().info("UROC Vector Visualizer Node started")
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
