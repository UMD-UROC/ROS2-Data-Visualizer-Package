"""Minimal Foxglove 3D Gimbal Visualization Node for UROC drone operations."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Point
from tf2_ros import TransformBroadcaster
from mavros_msgs.msg import GimbalDeviceSetAttitude
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from visualization_msgs.msg import Marker
# from mavros_msgs.msg import GimbalDeviceAttitudeStatus



class GimbalVisualizerNode(Node):
    def __init__(self):
        super().__init__('gimbal_visualizer_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.drone_q = [0.0, 0.0, 0.0, 1.0]

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            GimbalDeviceSetAttitude,
            '/mavros/gimbal_control/device/set_attitude',
            self.mavros_pose_callback,
            qos
        )

        self.marker_pub = self.create_publisher(Marker, '/drone/gimbal/marker', 1)
        self.timer = self.create_timer(1.0 / 1.0, self.publish_loop)

    def mavros_pose_callback(self, msg: GimbalDeviceSetAttitude):
        self.drone_q = [msg.q.x, msg.q.y, msg.q.z, msg.q.w]

    def publish_loop(self):
        # Use current timestamp
        stamp = self.get_clock().now().to_msg()

        # Broadcast TF from drone_frame -> gimbal_frame with current orientation
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = 'drone_frame'
        tf_msg.child_frame_id = 'gimbal_frame'
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = self.drone_q[0]
        tf_msg.transform.rotation.y = self.drone_q[1]
        tf_msg.transform.rotation.z = self.drone_q[2]
        tf_msg.transform.rotation.w = self.drone_q[3]
        self.tf_broadcaster.sendTransform(tf_msg)

        # Create a marker in gimbal_frame pointing forward in local +X
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = 'gimbal_frame'
        marker.ns = 'gimbal_arrow'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Fixed arrow shape (relative to gimbal_frame)
        start_point = Point(x=0.0, y=0.0, z=0.0)
        end_point = Point(x=1.0, y=0.0, z=0.0)
        marker.points = [start_point, end_point]

        # Size of the arrow
        marker.scale.x = 0.1  # shaft diameter
        marker.scale.y = 0.2  # head diameter
        marker.scale.z = 0.2  # head length

        # Color (solid red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalVisualizerNode()
    node.get_logger().info('UROC Foxglove Gimbal TF Node started')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
