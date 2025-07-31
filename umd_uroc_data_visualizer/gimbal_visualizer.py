# gimbal_visualizer.py

"""
Gimbal visualization for UROC drone system.
"""

import os
import rclpy
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from geometry_msgs.msg import Point
from mavros_msgs.msg import GimbalDeviceAttitudeStatus, GimbalDeviceSetAttitude
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from .qos_profile import BEST_EFFORT_QOS

package_share_directory = get_package_share_directory("umd_uroc_data_visualizer")
load_dotenv(os.path.join(package_share_directory, ".env"))
REFRESH_RATE_HZ = float(os.getenv("REFRESH_RATE_HZ", 0.1))

class GimbalVisualizer(Node):
    def __init__(self):
        super().__init__("gimbal_visualizer")
        self.tf_broadcaster = TransformBroadcaster(self)

        self.flags = None

        self.declare_parameter("visualizer_topic", "PARAMETER WASN'T SET")
        self.visualizer_topic = self.get_parameter("visualizer_topic").value

        if self.visualizer_topic == "/mavros/gimbal_control/device/set_attitude":
            self.marker_pub = self.create_publisher(
                Marker, "/drone/set_attitude/gimbal/marker", 1
            )
        elif self.visualizer_topic == "/mavros/gimbal_control/device/attitude_status":
            self.marker_pub = self.create_publisher(
                Marker, "/drone/attitude_status/gimbal/marker", 1
            )
        else:
            self.get_logger().info("Unsupported Parameter!")
            exit(1)

        self.timer = self.create_timer(REFRESH_RATE_HZ, self.publish_loop)

    def publish_loop(self):
        stamp = self.get_clock().now().to_msg()

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "gimbal_frame"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=1.0, y=0.0, z=0.0),
        ]
        marker.scale.x = 0.1
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0

        if self.visualizer_topic == "/mavros/gimbal_control/device/set_attitude":
            marker.ns = "gimbal_set_attitude"
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

        elif self.visualizer_topic == "/mavros/gimbal_control/device/attitude_status":
            marker.ns = "gimbal_attitude_status"
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0

        else:
            self.get_logger().info("Unsupported Parameter!")
            exit(1)

        self.marker_pub.publish(marker)

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

