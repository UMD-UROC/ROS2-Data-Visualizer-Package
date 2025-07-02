"""Foxglove 3D Path Visualization Node for UROC drone operations.

This module provides a ROS2 node that subscribes to MAVROS pose data and
publishes visualization data for Foxglove including drone pose, path, and
transforms.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class PathVisualizerNode(Node):
    """ROS2 node for visualizing drone path in Foxglove.

    Subscribes to MAVROS pose data and publishes drone pose, flight path,
    and TF transforms for visualization in Foxglove.
    """

    def __init__(self):
        super().__init__('path_visualizer_node')
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers
        self.drone_pose_pub = self.create_publisher(PoseStamped, '/drone/pose', 1)
        self.path = Path()
        self.path.header.frame_id = 'map'  # match your TF root
        self.path_pub = self.create_publisher(Path, '/drone/flight_path', 1)

        # State
        self.drone_pos = [0.0, 0.0, 0.0]
        self.drone_q = [0.0, 0.0, 0.0, 1.0]

        # QoS matching MAVROS publisher (BEST_EFFORT)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to MAVROS pose with matching QoS
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.mavros_pose_callback,
            qos
        )

        # Timer for publishing transforms, pose, and path
        self.timer = self.create_timer(1.0 / 1.0, self.publish_loop)

    def mavros_pose_callback(self, msg: PoseStamped):
        # Receive ENU pose
        self.drone_pos = [msg.pose.position.x,
                          msg.pose.position.y,
                          msg.pose.position.z]
        self.drone_q = [msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w]
        self.latest_header = msg.header

    def publish_loop(self):
        # Use latest header timestamp, fallback to now
        if hasattr(self, 'latest_header'):
            stamp = self.latest_header.stamp
        else:
            stamp = self.get_clock().now().to_msg()

        # Broadcast TF: map->drone_frame
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.path.header.frame_id
        tf_msg.child_frame_id = 'drone_frame'
        tf_msg.transform.translation.x = self.drone_pos[0]
        tf_msg.transform.translation.y = self.drone_pos[1]
        tf_msg.transform.translation.z = self.drone_pos[2]
        tf_msg.transform.rotation.x = self.drone_q[0]
        tf_msg.transform.rotation.y = self.drone_q[1]
        tf_msg.transform.rotation.z = self.drone_q[2]
        tf_msg.transform.rotation.w = self.drone_q[3]
        self.tf_broadcaster.sendTransform(tf_msg)

        # Current PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.path.header.frame_id
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
    """Main entry point for the path visualization node."""
    rclpy.init(args=args)
    node = PathVisualizerNode()

    node.get_logger().info('UROC Foxglove 3D Path Visualization Node started')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
