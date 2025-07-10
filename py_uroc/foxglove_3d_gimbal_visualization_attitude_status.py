#!/usr/bin/env python3
"""Visualize actual gimbal attitude status as a blue yaw-invariant arrow."""

import rclpy
from geometry_msgs.msg import TransformStamped , Point , PoseStamped
from mavros_msgs.msg import GimbalDeviceAttitudeStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile , QoSReliabilityPolicy , QoSHistoryPolicy
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from .qos_profile import BEST_EFFORT_QOS


def quat_inverse(q):
    x, y, z, w = q
    return [-x, -y, -z, w]


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


class GimbalStatusVisualizer(Node):
    def __init__(self):
        super().__init__("gimbal_visualizer_attitude_status")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.status_q = [0.0, 0.0, 0.0, 1.0]
        self.drone_q = [0.0, 0.0, 0.0, 1.0]
        self.flags = None

        # Subscriptions
        self.create_subscription(
            GimbalDeviceAttitudeStatus,
            "/mavros/gimbal_control/device/attitude_status",
            self.on_status,
            BEST_EFFORT_QOS,
        )
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.on_drone_pose,
            BEST_EFFORT_QOS,
        )

        self.marker_pub = self.create_publisher(
            Marker, "/drone/attitude_status/gimbal/marker", 1
        )
        self.timer = self.create_timer(0.1, self.publish_loop)  # 10 Hz

    def on_status(self, msg: GimbalDeviceAttitudeStatus):
        self.status_q = [msg.q.x, msg.q.y, msg.q.z, msg.q.w]
        self.flags = msg.flags

    def on_drone_pose(self, msg: PoseStamped):
        self.drone_q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]

    def publish_loop(self):
        if self.flags is None:
            return
        if self.flags != 0:
            self.get_logger().warn("Gimbal not supported, skipping visualization")
            return

        stamp = self.get_clock().now().to_msg()
        # Relative orientation: inv(drone_q) * status_q
        q_rel = quat_multiply(quat_inverse(self.drone_q), self.status_q)

        # 2) Draw blue arrow along local -X
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "gimbal_frame"
        marker.ns = "gimbal_attitude_status"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=-1.0, y=0.0, z=0.0),
        ]
        marker.scale.x = 0.1
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalStatusVisualizer()
    node.get_logger().info("Started Gimbal Attitude-Status Visualizer")
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
