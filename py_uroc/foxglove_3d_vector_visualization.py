"""Minimal Foxglove 3D Gimbal Visualization Node for UROC drone operations."""

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


class VectorVisualizerNode(Node):
    def __init__(self):
        super().__init__("vector_visualizer_node")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.gimbal_q = [0.0, 0.0, 0.0, 1.0]
        self.loc = None  # Initialize location to None
        self.drone_pos = None  # Init drone pos

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            PositionTarget, "/mavros/setpoint_raw/local", self.mavros_pose_callback, qos
        )

        self.create_subscription(
            PoseStamped, "/drone/pose", self.mavros_pose_callback_drone, qos
        )

        self.marker_pub = self.create_publisher(Marker, "/drone/vector/marker", 1)
        self.timer = self.create_timer(1.0 / 1.0, self.publish_loop)

    def mavros_pose_callback(self, msg: PositionTarget):
        self.loc = msg.position  # Store the full Vector3

    def mavros_pose_callback_drone(self, msg: PoseStamped):
        # Receive ENU pose
        self.drone_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.drone_q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        self.latest_header = msg.header

    def publish_loop(self):
        if self.loc is None or self.drone_pos is None:
            return  # Wait until position data is received

        stamp = self.get_clock().now().to_msg()

        # Optional: Broadcast TF (currently commented out)
        # tf_msg = TransformStamped()
        # tf_msg.header.stamp = stamp
        # tf_msg.header.frame_id = 'drone_frame'
        # tf_msg.child_frame_id = 'gimbal_frame'
        # tf_msg.transform.translation.x = 0.0
        # tf_msg.transform.translation.y = 0.0
        # tf_msg.transform.translation.z = 0.0
        # tf_msg.transform.rotation.x = self.gimbal_q[0]
        # tf_msg.transform.rotation.y = self.gimbal_q[1]
        # tf_msg.transform.rotation.z = self.gimbal_q[2]
        # tf_msg.transform.rotation.w = self.gimbal_q[3]
        # self.tf_broadcaster.sendTransform(tf_msg)

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "base_link"  # Use base_link frame for consistency
        marker.ns = "vector_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Start point is the drone's current position in base_link frame
        start_point = Point(
            x=self.drone_pos[0],
            y=self.drone_pos[1],
            z=self.drone_pos[2],
        )
        # End point is the target position
        end_point = Point(
            x=self.loc.x,
            y=self.loc.y,
            z=self.loc.z,
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
