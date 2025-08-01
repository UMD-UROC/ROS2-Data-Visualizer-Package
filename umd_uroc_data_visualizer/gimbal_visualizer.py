# gimbal_visualizer.py

"""
Gimbal orientation visualization for UROC drone system.

This module creates 3D arrow markers to visualize gimbal orientation in real-time
for the UROC drone visualization system. It publishes arrow markers that can be
displayed in 3D visualization tools like Foxglove to show either commanded gimbal
orientation (red arrows) or actual gimbal status (blue arrows).

The node operates in two modes:
1. Set Attitude Mode: Shows commanded gimbal orientations as red arrows
2. Attitude Status Mode: Shows actual gimbal orientations as blue arrows

The arrows are published as ROS visualization markers in the gimbal_frame coordinate
system, which is established by the gimbal_frame node's transform publishers.
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

# Load environment configuration from package share directory
package_share_directory = get_package_share_directory("umd_uroc_data_visualizer")
load_dotenv(os.path.join(package_share_directory, ".env"))
# Refresh rate for gimbal marker publishing (Hz) - controls visualization update frequency
REFRESH_RATE_HZ = float(os.getenv("REFRESH_RATE_HZ", 10.0))

class GimbalVisualizer(Node):
    """
    ROS2 node for visualizing gimbal orientation using 3D arrow markers.

    This node creates and publishes arrow-shaped visualization markers that represent
    gimbal orientation in 3D space. The arrows are displayed in the gimbal_frame
    coordinate system and can show either commanded gimbal attitudes (red arrows)
    or actual gimbal status (blue arrows) depending on configuration.

    Key Features:
    - Configurable visualization mode (set_attitude vs attitude_status)
    - Color-coded arrows (red for commands, blue for actual status)
    - Real-time marker updates at configurable refresh rate
    - Compatible with 3D visualization tools like Foxglove

    Marker Specifications:
    - Type: Arrow (pointing in +X direction of gimbal frame)
    - Scale: 0.1m shaft, 0.2m head width/height
    - Colors: Red (set_attitude) or Blue (attitude_status)
    - Frame: gimbal_frame (established by gimbal_frame node)

    Published Topics:
    - /drone/set_attitude/gimbal/marker: Red arrows for commanded orientations
    - /drone/attitude_status/gimbal/marker: Blue arrows for actual orientations

    Parameters:
    - visualizer_topic: Selects gimbal data source and visualization mode
    """

    def __init__(self):
        """
        Initialize the GimbalVisualizer node with publishers and configuration.

        Sets up the appropriate marker publisher based on the configured visualizer
        topic and initializes the timer for periodic marker publishing.
        """
        super().__init__("gimbal_visualizer")

        # Initialize transform broadcaster (currently unused but available for future features)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Storage for gimbal device flags (reserved for future use with status messages)
        self.flags = None

        # Parameter to select which gimbal topic/mode to visualize
        # Default set to trigger error if not properly configured
        self.declare_parameter("visualizer_topic", "PARAMETER WASN'T SET")
        self.visualizer_topic = self.get_parameter("visualizer_topic").value

        # Create appropriate marker publisher based on visualizer topic configuration
        if self.visualizer_topic == "/mavros/gimbal_control/device/set_attitude":
            # Publisher for commanded gimbal attitude visualization (red arrows)
            self.marker_pub = self.create_publisher(
                Marker, "/drone/set_attitude/gimbal/marker", 1
            )
        elif self.visualizer_topic == "/mavros/gimbal_control/device/attitude_status":
            # Publisher for actual gimbal attitude visualization (blue arrows)
            self.marker_pub = self.create_publisher(
                Marker, "/drone/attitude_status/gimbal/marker", 1
            )
        else:
            # Invalid configuration - log error and exit
            self.get_logger().info("Unsupported Parameter!")
            exit(1)

        # Timer for periodic marker publishing at configured refresh rate
        self.timer = self.create_timer(1.0 / REFRESH_RATE_HZ, self.publish_loop)

    def publish_loop(self):
        """
        Timer callback that creates and publishes gimbal orientation visualization markers.

        Creates an arrow marker representing gimbal orientation and publishes it to the
        appropriate topic. The arrow points in the +X direction of the gimbal_frame,
        showing the gimbal's pointing direction. Color and namespace are determined
        by the configured visualizer topic.

        Marker Properties:
        - Type: ARROW (3D arrow from point A to point B)
        - Points: [origin, +X direction] - 1 meter arrow in gimbal frame +X
        - Frame: gimbal_frame (orientation provided by gimbal_frame node)
        - Scale: 0.1m shaft diameter, 0.2m arrowhead dimensions
        - Colors: Red (commands) or Blue (status) with full opacity

        Notes
        -----
        Called periodically at REFRESH_RATE_HZ to maintain real-time visualization.
        """
        # Get current timestamp for the marker message
        stamp = self.get_clock().now().to_msg()

        # Create arrow marker for gimbal orientation visualization
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "gimbal_frame"  # Arrow expressed in gimbal coordinate frame
        marker.id = 0                            # Unique marker ID
        marker.type = Marker.ARROW               # 3D arrow marker type
        marker.action = Marker.ADD               # Add/update the marker

        # Define arrow geometry: from origin to +X direction (1 meter)
        # This shows the gimbal's pointing direction in its local coordinate frame
        marker.points = [
            Point(x=0.0, y=0.0, z=0.0),    # Arrow start point (gimbal center)
            Point(x=1.0, y=0.0, z=0.0),    # Arrow end point (+X direction)
        ]

        # Set arrow scale dimensions
        marker.scale.x = 0.1    # Arrow shaft diameter (meters)
        marker.scale.y = 0.2    # Arrow head width (meters)
        marker.scale.z = 0.2    # Arrow head height (meters)

        # Set opacity to fully opaque
        marker.color.a = 1.0

        # Configure marker appearance based on visualizer topic mode
        if self.visualizer_topic == "/mavros/gimbal_control/device/set_attitude":
            # Red arrows for commanded gimbal attitudes
            marker.ns = "gimbal_set_attitude"
            marker.color.r = 1.0    # Full red
            marker.color.g = 0.0    # No green
            marker.color.b = 0.0    # No blue

        elif self.visualizer_topic == "/mavros/gimbal_control/device/attitude_status":
            # Blue arrows for actual gimbal status
            marker.ns = "gimbal_attitude_status"
            marker.color.r = 0.0    # No red
            marker.color.g = 0.0    # No green
            marker.color.b = 1.0    # Full blue

        else:
            # Should never reach here due to constructor validation
            self.get_logger().info("Unsupported Parameter!")
            exit(1)

        # Publish the marker for visualization
        self.marker_pub.publish(marker)

def main(args=None):
    """
    Main entry point for the gimbal visualizer node.

    Initializes ROS2, creates the GimbalVisualizer node, and runs the main event loop.
    Handles graceful shutdown on keyboard interrupt and ensures proper cleanup.

    Parameters
    ----------
    args : list, optional
        Command line arguments passed to rclpy.init()

    Notes
    -----
    This function serves as the console script entry point defined in setup.py.
    """
    # Initialize ROS2 Python client library
    rclpy.init(args=args)

    # Create and start the gimbal visualizer node
    node = GimbalVisualizer()
    node.get_logger().info("Started Gimbal Visualizer")

    try:
        # Run the node until shutdown is requested
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully without error messages
        pass
    finally:
        # Cleanup: destroy node and shutdown ROS2 if still active
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()

