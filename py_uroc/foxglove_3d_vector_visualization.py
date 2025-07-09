import message_filters
import rclpy
from geometry_msgs.msg import Point , PoseStamped
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from rclpy.qos import QoSProfile , QoSReliabilityPolicy , QoSHistoryPolicy
from visualization_msgs.msg import Marker

from .qos_profile import BEST_EFFORT_QOS


class VectorVisualizerNode(Node):
    """ROS2 node that draws a green arrow from drone to target in map frame."""

    def __init__(self):
        super().__init__("vector_visualizer_node")

        # Sync PositionTarget (map) and drone PoseStamped (map)
        self.target_sub = message_filters.Subscriber(
            self,
            PositionTarget,
            "/mavros/setpoint_raw/local",
            qos_profile=BEST_EFFORT_QOS,
        )
        self.drone_pose_sub = message_filters.Subscriber(
            self, PoseStamped, "/drone/pose", qos_profile=BEST_EFFORT_QOS
        )
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.target_sub, self.drone_pose_sub],
            queue_size=10,
            slop=0.1,
        )
        self.ts.registerCallback(self.synchronized_callback)

        self.marker_pub = self.create_publisher(Marker, "/drone/vector/marker", 1)

    def synchronized_callback(
        self, target_msg: PositionTarget, drone_pose_msg: PoseStamped
    ):
        stamp = drone_pose_msg.header.stamp

        # Extract global ENU positions
        drone_pos = [
            drone_pose_msg.pose.position.x,
            drone_pose_msg.pose.position.y,
            drone_pose_msg.pose.position.z,
        ]

        # Use velocity from PositionTarget message to create velocity vector
        drone_velocity = mavV_to_rosV([target_msg.velocity.x,
                                       target_msg.velocity.y,
                                       target_msg.velocity.z])

        # Create target position by adding velocity vector to current position
        # This creates a velocity vector visualization
        target_pos = [
            drone_pos[0] + drone_velocity[0],
            drone_pos[1] + drone_velocity[1],
            drone_pos[2] + drone_velocity[2],
        ]

        # Build arrow marker in "map"
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "map"
        marker.ns = "vector_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Absolute start/end in ENU
        start_point = Point(x=drone_pos[0], y=drone_pos[1], z=drone_pos[2])
        end_point = Point(x=target_pos[0], y=target_pos[1], z=target_pos[2])
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


def main(args=None):
    rclpy.init(args=args)
    node = VectorVisualizerNode()
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

def mavV_to_rosV(mavV):
    return [mavV[1], mavV[0], -mavV[2]]
