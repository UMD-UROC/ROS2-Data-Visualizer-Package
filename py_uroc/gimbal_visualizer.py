import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from geometry_msgs.msg import Point , PoseStamped
from mavros_msgs.msg import GimbalDeviceSetAttitude , GimbalDeviceAttitudeStatus
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

# Load .env file from the package share directory
package_share_directory = get_package_share_directory("py_uroc")
load_dotenv(os.path.join(package_share_directory, ".env"))
REFRESH_RATE_HZ = float(os.getenv("REFRESH_RATE_HZ"))


class GimbalVisualizer(Node):
    def __init__(self):
        print("Hello World")


def main(args=None):
    rclpy.init(args=args)
    node = GimbalVisualizer()
    node.get_logger().info("Gimbal Visualizer Started")
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
