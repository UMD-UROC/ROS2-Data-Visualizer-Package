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
from .node_utils import NodeShutdownHandler, setup_node_logging, log_periodic_status

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

    def __init__(self, debug: bool = False):
        """
        Initialize the GimbalVisualizer node with publishers and configuration.

        Sets up the appropriate marker publisher based on the configured visualizer
        topic and initializes the timer for periodic marker publishing.
        """
        super().__init__("gimbal_visualizer")

        # Setup logging
        self.logger = setup_node_logging(self, debug)
        self.debug = debug
        
        # Setup graceful shutdown handling
        self.shutdown_handler = NodeShutdownHandler(self)
        
        # Initialize counters for periodic status reporting
        self.publish_loop_count = 0

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
            self.marker_color = "red"
        elif self.visualizer_topic == "/mavros/gimbal_control/device/attitude_status":
            # Publisher for actual gimbal attitude visualization (blue arrows)
            self.marker_pub = self.create_publisher(
                Marker, "/drone/attitude_status/gimbal/marker", 1
            )
            self.marker_color = "blue"
        else:
            # Invalid configuration - log error and exit
            self.logger.error(f"Unsupported Parameter: {self.visualizer_topic}")
            raise ValueError(f"Unsupported visualizer_topic: {self.visualizer_topic}")

        # Timer for periodic marker publishing at configured refresh rate
        self.timer = self.create_timer(1.0 / REFRESH_RATE_HZ, self.publish_loop)
        
        self.logger.info(f"Gimbal visualizer initialized (debug={'enabled' if debug else 'disabled'})")
        if debug:
            self.logger.info(f"Visualizing {self.visualizer_topic} as {self.marker_color} arrows at {REFRESH_RATE_HZ} Hz")

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
        self.publish_loop_count += 1
        
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
            self.logger.error("Unsupported Parameter!")
            return

        # Publish the marker for visualization
        self.marker_pub.publish(marker)
        
        # Debug-only status reporting (control center doesn't need marker publication details)
        if self.debug:
            log_periodic_status(
                self,
                f"Published {self.marker_color} gimbal arrow marker",
                self.publish_loop_count,
                200  # Log every 200 publications
            )
            self.logger.debug(f"Published {marker.ns} marker with {self.marker_color} color")

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
    rclpy.init(args=args)
    
    # Check for debug flag in arguments
    debug = '--debug' in (args or [])

    try:
        # Create and start the gimbal visualizer node
        node = GimbalVisualizer(debug=debug)
        node.get_logger().info("Started Gimbal Visualizer")
        # Use the new shutdown-aware spin method
        node.shutdown_handler.spin_with_shutdown()
    except KeyboardInterrupt:
        # Graceful shutdown is handled by NodeShutdownHandler
        pass
    except Exception as e:
        print(f"Unexpected error in gimbal visualizer: {e}")
    finally:
        # Cleanup is handled by NodeShutdownHandler
        pass

if __name__ == "__main__":
    main()

