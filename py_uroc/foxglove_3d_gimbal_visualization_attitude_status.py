"""Minimal Foxglove 3D Gimbal Visualization Node for UROC drone operations."""

import rclpy
from geometry_msgs.msg import TransformStamped, Point, PoseStamped
from mavros_msgs.msg import GimbalDeviceAttitudeStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


class GimbalVisualizerNode(Node):
    def __init__(self):
        super().__init__("gimbal_visualizer_node")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.gimbal_q = [0.0, 0.0, 0.0, 1.0]
        self.drone_pos = [0.0, 0.0, 0.0]
        self.drone_q = [0.0, 0.0, 0.0, 1.0]

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            GimbalDeviceAttitudeStatus,
            "/mavros/gimbal_control/device/attitude_status",
            self.gimbal_attitude_callback,
            qos,
        )
        
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.drone_pose_callback,
            qos,
        )

        self.marker_pub = self.create_publisher(Marker, "/drone/gimbal/marker", 1)
        self.timer = self.create_timer(1.0 / 1.0, self.publish_loop)

    def gimbal_attitude_callback(self, msg: GimbalDeviceAttitudeStatus):
        self.gimbal_q = [msg.q.x, msg.q.y, msg.q.z, msg.q.w]

    def drone_pose_callback(self, msg: PoseStamped):
        self.drone_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.drone_q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

    def publish_loop(self):
        # Use current timestamp
        stamp = self.get_clock().now().to_msg()

        # Broadcast TF from base_link → gimbal_frame with current orientation and position
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "base_link"
        tf_msg.child_frame_id = "gimbal_frame"
        tf_msg.transform.translation.x = self.drone_pos[0]
        tf_msg.transform.translation.y = self.drone_pos[1]
        tf_msg.transform.translation.z = self.drone_pos[2]
        tf_msg.transform.rotation.x = self.gimbal_q[0]
        tf_msg.transform.rotation.y = self.gimbal_q[1]
        tf_msg.transform.rotation.z = self.gimbal_q[2]
        tf_msg.transform.rotation.w = self.gimbal_q[3]
        self.tf_broadcaster.sendTransform(tf_msg)

        # Create a marker in gimbal_frame pointing forward in local +X
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "gimbal_frame"
        marker.ns = "gimbal_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Fixed arrow shape (relative to gimbal_frame)
        start_point = Point(x=0.0, y=0.0, z=0.0)
        end_point = Point(x=1.0, y=0.0, z=0.0)
        marker.points = [start_point, end_point]
        
        # Scale for arrow size
        marker.scale.x = 1.0  # arrow length
        marker.scale.y = 0.1  # arrow width
        marker.scale.z = 0.1  # arrow height

        # Color (solid red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalVisualizerNode()
    node.get_logger().info("UROC Foxglove Gimbal TF Node started")

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
