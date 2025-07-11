import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from geometry_msgs.msg import Point , PoseStamped
from mavros_msgs.msg import GimbalDeviceAttitudeStatus , GimbalDeviceSetAttitude
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from .qos_profile import BEST_EFFORT_QOS

# Load .env file from the package share directory
package_share_directory = get_package_share_directory("py_uroc")
load_dotenv(os.path.join(package_share_directory, ".env"))
REFRESH_RATE_HZ = float(os.getenv("REFRESH_RATE_HZ"))


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


class GimbalVisualizer(Node):
    def __init__(self):
        super().__init__("gimbal_visualizer")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.status_q = [0.0, 0.0, 0.0, 1.0]
        self.drone_q = [0.0, 0.0, 0.0, 1.0]
        self.flags = None

        # Declare parameters
        self.declare_parameter("visualizer_topic", "PARAMETER WASN'T SET")

        # Get parameters
        self.visualizer_topic = self.get_parameter("visualizer_topic").value

        # Subscriptions
        if self.visualizer_topic == "/mavros/gimbal_control/device/set_attitude":
            self.create_subscription(
                GimbalDeviceSetAttitude,
                "/mavros/gimbal_control/device/set_attitude",
                self.on_gimbal_cmd,
                BEST_EFFORT_QOS,
            )
        elif self.visualizer_topic == "/mavros/gimbal_control/device/attitude_status":
            self.create_subscription(
                GimbalDeviceAttitudeStatus,
                "/mavros/gimbal_control/device/attitude_status",
                self.on_status,
                BEST_EFFORT_QOS,
            )
        else:
            self.get_logger.info("Unsupported Parameter!")
            exit(1)

        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.on_drone_pose,
            BEST_EFFORT_QOS,
        )

        if self.visualizer_topic == "/mavros/gimbal_control/device/set_attitude":
            self.marker_pub = self.create_publisher(Marker, "/drone/set_attitude/gimbal/marker", 1)
        elif self.visualizer_topic == "/mavros/gimbal_control/device/attitude_status":
            self.marker_pub = self.create_publisher(
                Marker, "/drone/attitude_status/gimbal/marker", 1
            )
        else:
            self.get_logger.info("Unsupported Parameter!")
            exit(1)

        self.timer = self.create_timer(REFRESH_RATE_HZ, self.publish_loop)  # 10 Hz

    def on_status(self, msg: GimbalDeviceAttitudeStatus):
        self.status_q = [msg.q.x, msg.q.y, msg.q.z, msg.q.w]
        self.flags = msg.flags

    def on_gimbal_cmd(self, msg: GimbalDeviceSetAttitude):
        # msg.q is already in ENU order [x,y,z,w]
        self.cmd_q = [msg.q.x, msg.q.y, msg.q.z, msg.q.w]

    def on_drone_pose(self, msg: PoseStamped):
        self.drone_q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]

    def publish_loop(self):

        stamp = self.get_clock().now().to_msg()

        if self.visualizer_topic == "/mavros/gimbal_control/device/set_attitude":

            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = "gimbal_frame"
            marker.ns = "gimbal_set_attitude"
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
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            self.marker_pub.publish(marker)

        elif self.visualizer_topic == "/mavros/gimbal_control/device/attitude_status":

            if self.flags is None:
                return
            if self.flags != 0:
                self.get_logger().warn("Gimbal not supported, skipping visualization")
                return

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

        else:
            self.get_logger.info("Unsupported Parameter!")
            exit(1)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalVisualizer()
    node.get_logger().info("Started Gimbal Visualizer")
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
