"""

Gimbal visualization for UROC drone system.

This module provides visualization capabilities for gimbal orientation in 3D space.
It can visualize either gimbal set attitude commands or actual gimbal status,
depending on configuration. The visualization appears as colored arrows in the
gimbal reference frame for use in tools like Foxglove.
"""

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import GimbalDeviceAttitudeStatus, GimbalDeviceSetAttitude
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from .qos_profile import BEST_EFFORT_QOS

# Load environment configuration from the package share directory
package_share_directory = get_package_share_directory("umd_uroc_data_visualizer")
load_dotenv(os.path.join(package_share_directory, ".env"))
# Refresh rate for gimbal visualization updates (Hz)
REFRESH_RATE_HZ = float(os.getenv("REFRESH_RATE_HZ"))


def quat_inverse(q):
    """
    Compute the inverse of a quaternion.

    Parameters
    ----------
    q : array-like
        Quaternion as [x, y, z, w]

    Returns
    -------
    list
        Inverse quaternion as [x, y, z, w]

    """
    x, y, z, w = q
    return [-x, -y, -z, w]


def quat_multiply(a, b):
    """
    Multiply two quaternions.

    Performs quaternion multiplication: result = a * b

    Parameters
    ----------
    a : array-like
        First quaternion as [x, y, z, w]
    b : array-like
        Second quaternion as [x, y, z, w]

    Returns
    -------
    list
        Product quaternion as [x, y, z, w]

    """
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


class GimbalVisualizer(Node):
    """
    ROS2 node for visualizing gimbal orientation and commands.

    This node can visualize either gimbal set attitude commands or actual
    gimbal status, depending on the visualizer_topic parameter. It creates
    arrow markers in the gimbal frame to show gimbal orientation in 3D
    visualization tools.

    Attributes
    ----------
    tf_broadcaster : TransformBroadcaster
        Transform broadcaster for coordinate frames
    status_q : list
        Current gimbal status quaternion [x, y, z, w]
    drone_q : list
        Current drone orientation quaternion [x, y, z, w]
    flags : int
        Gimbal status flags
    visualizer_topic : str
        Topic name determining visualization mode
    marker_pub : Publisher
        Publisher for visualization markers
    timer : Timer
        Timer for periodic visualization updates

    """

    def __init__(self):
        """Initialize the GimbalVisualizer node."""
        super().__init__("gimbal_visualizer")
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize gimbal and drone orientation quaternions (identity)
        self.status_q = [0.0, 0.0, 0.0, 1.0]
        self.drone_q = [0.0, 0.0, 0.0, 1.0]
        self.flags = None

        # Declare and get configuration parameter for visualization topic
        self.declare_parameter("visualizer_topic", "PARAMETER WASN'T SET")
        self.visualizer_topic = self.get_parameter("visualizer_topic").value

        # Subscribe to appropriate gimbal topic based on configuration
        if self.visualizer_topic == "/mavros/gimbal_control/device/set_attitude":
            # Subscribe to gimbal command messages (what gimbal should do)
            self.create_subscription(
                GimbalDeviceSetAttitude,
                "/mavros/gimbal_control/device/set_attitude",
                self.on_gimbal_cmd,
                BEST_EFFORT_QOS,
            )
        elif self.visualizer_topic == "/mavros/gimbal_control/device/attitude_status":
            # Subscribe to gimbal status messages (actual gimbal state)
            self.create_subscription(
                GimbalDeviceAttitudeStatus,
                "/mavros/gimbal_control/device/attitude_status",
                self.on_status,
                BEST_EFFORT_QOS,
            )
        else:
            self.get_logger().info("Unsupported Parameter!")
            exit(1)

        # Subscribe to drone pose for coordinate frame reference
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.on_drone_pose,
            BEST_EFFORT_QOS,
        )

        # Create appropriate marker publisher based on visualization mode
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

        # Create timer for periodic visualization updates
        self.timer = self.create_timer(REFRESH_RATE_HZ, self.publish_loop)

    def on_status(self, msg: GimbalDeviceAttitudeStatus):
        """
        Handle incoming gimbal attitude status messages.

        Updates the current gimbal orientation and status flags from
        the actual gimbal hardware feedback.

        Parameters
        ----------
        msg : GimbalDeviceAttitudeStatus
            Gimbal device attitude status message

        """
        self.status_q = [msg.q.x, msg.q.y, msg.q.z, msg.q.w]
        self.flags = msg.flags

    def on_gimbal_cmd(self, msg: GimbalDeviceSetAttitude):
        """
        Handle incoming gimbal set attitude command messages.

        Updates the commanded gimbal orientation from gimbal control commands.
        The quaternion is already in ENU coordinate frame.

        Parameters
        ----------
        msg : GimbalDeviceSetAttitude
            Gimbal device set attitude command message

        """
        # Store commanded quaternion (already in ENU order [x,y,z,w])
        self.cmd_q = [msg.q.x, msg.q.y, msg.q.z, msg.q.w]

    def on_drone_pose(self, msg: PoseStamped):
        """
        Handle incoming drone pose messages.

        Updates the current drone orientation for coordinate frame reference.

        Parameters
        ----------
        msg : PoseStamped
            Drone pose message from MAVROS

        """
        self.drone_q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]

    def publish_loop(self):
        """
        Publish gimbal visualization markers.

        Creates and publishes arrow markers showing gimbal orientation based on
        the configured visualization mode (command vs status).
        """
        # Get current timestamp for marker messages
        stamp = self.get_clock().now().to_msg()

        if self.visualizer_topic == "/mavros/gimbal_control/device/set_attitude":
            # Visualize gimbal set attitude commands (red arrows)
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = "gimbal_frame"
            marker.ns = "gimbal_set_attitude"
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            # Arrow points from origin to (-1, 0, 0) in gimbal frame
            marker.points = [
                Point(x=0.0, y=0.0, z=0.0),
                Point(x=-1.0, y=0.0, z=0.0),
            ]

            # Set arrow geometry and red color for commands
            marker.scale.x = 0.1  # Arrow shaft diameter
            marker.scale.y = 0.2  # Arrow head width
            marker.scale.z = 0.2  # Arrow head height
            marker.color.r = 1.0  # Red color for set attitude
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Fully opaque
            self.marker_pub.publish(marker)

        elif self.visualizer_topic == "/mavros/gimbal_control/device/attitude_status":
            # Only visualize if gimbal status is available and supported
            if self.flags is None:
                return
            if self.flags != 0:
                self.get_logger().warn("Gimbal not supported, skipping visualization")
                return

            # Visualize actual gimbal attitude status (blue arrows)
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = "gimbal_frame"
            marker.ns = "gimbal_attitude_status"
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            # Arrow points from origin to (-1, 0, 0) in gimbal frame
            marker.points = [
                Point(x=0.0, y=0.0, z=0.0),
                Point(x=-1.0, y=0.0, z=0.0),
            ]

            # Set arrow geometry and blue color for status
            marker.scale.x = 0.1  # Arrow shaft diameter
            marker.scale.y = 0.2  # Arrow head width
            marker.scale.z = 0.2  # Arrow head height
            marker.color.r = 0.0  # Blue color for attitude status
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0  # Fully opaque
            self.marker_pub.publish(marker)

        else:
            self.get_logger().info("Unsupported Parameter!")
            exit(1)


def main(args=None):
    """
    Execute the gimbal visualizer node.

    Parameters
    ----------
    args : list, optional
        Command line arguments

    """
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
