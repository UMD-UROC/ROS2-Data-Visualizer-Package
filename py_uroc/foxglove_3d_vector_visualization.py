"""Minimal Foxglove 3D Gimbal Visualization Node for UROC drone operations."""

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
import message_filters


class VectorVisualizerNode(Node):
    def __init__(self):
        super().__init__("vector_visualizer_node")
        self.tf_broadcaster = TransformBroadcaster(self)
        
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Use message synchronization to ensure drone pose and target are synchronized
        self.target_sub = message_filters.Subscriber(
            self, PositionTarget, "/mavros/setpoint_raw/local", qos_profile=qos
        )
        self.drone_pose_sub = message_filters.Subscriber(
            self, PoseStamped, "/drone/pose", qos_profile=qos
        )
        
        # Synchronize messages with a small time tolerance
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.target_sub, self.drone_pose_sub], 
            queue_size=10, 
            slop=0.1  # 100ms tolerance
        )
        self.ts.registerCallback(self.synchronized_callback)

        self.marker_pub = self.create_publisher(Marker, "/drone/vector/marker", 1)

    def synchronized_callback(self, target_msg: PositionTarget, drone_pose_msg: PoseStamped):
        """Callback that receives synchronized target and drone pose messages."""
        # Use the timestamp from the messages for better synchronization
        stamp = drone_pose_msg.header.stamp
        
        # Extract drone position
        drone_pos = [
            drone_pose_msg.pose.position.x,
            drone_pose_msg.pose.position.y,
            drone_pose_msg.pose.position.z
        ]
        
        # Extract target position
        target_pos = target_msg.position

        # Create synchronized vector marker
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "base_link"  # Use base_link frame for consistency
        marker.ns = "vector_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Start point is the drone's position at the time of the synchronized messages
        start_point = Point(
            x=drone_pos[0],
            y=drone_pos[1],
            z=drone_pos[2],
        )
        # End point is the target position from the synchronized message
        end_point = Point(
            x=target_pos.x,
            y=target_pos.y,
            z=target_pos.z,
        )
        marker.points = [start_point, end_point]

        marker.scale.x = 0.1  # shaft diameter
        marker.scale.y = 0.2  # head diameter
        marker.scale.z = 0.2  # head length

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
