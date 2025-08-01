import os
import rclpy
import rclpy.logging
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from .qos_profile import BEST_EFFORT_QOS
from visualization_msgs.msg import Marker

# Load environment configuration from package share directory to get refresh rate
package_share_directory = get_package_share_directory("umd_uroc_data_visualizer")
load_dotenv(os.path.join(package_share_directory, ".env"))
# Refresh rate for transform publishing (Hz) - controls how often gimbal transforms are updated
REFRESH_RATE_HZ = float(os.getenv("REFRESH_RATE_HZ", 0.1))

class RangefinderLocalizationVisualizer(Node):
    """
    ROS2 node for visualizing rangefinder localization data in 3D.

    This node creates and publishes visualization markers that represent rangefinder
    localization data in 3D space. The markers are displayed in the rangefinder_frame
    coordinate system and can show either raw rangefinder data or processed localization
    results depending on configuration.

    Key Features:
    - Configurable visualization mode (raw vs processed)
    - Color-coded markers (green for raw, yellow for processed)
    - Real-time marker updates at configurable refresh rate
    - Compatible with 3D visualization tools like Foxglove

    Marker Specifications:
    - Type: Sphere (representing rangefinder points)
    - Scale: 0.1m radius
    - Colors: Green (raw data) or Yellow (processed data)
    - Frame: rangefinder_frame (established by rangefinder_frame node)

    Published Topics:
    - /drone/rangefinder/localization/marker: Markers for rangefinder localization data
    """

    def __init__(self):
        super().__init__("rangefinder_localization_visualizer")
        self.logger = rclpy.logging.get_logger("RangefinderLocalizationVisualizer")
        self.logger.info("Initializing Rangefinder Localization Visualizer Node")

        # Initialize transform broadcaster for publishing transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        # Set up publisher for rangefinder localization markers
        self.marker_publisher = self.create_publisher(
            Marker, "/drone/rangefinder/localization/marker", BEST_EFFORT_QOS
        )

        # Timer to control marker publishing frequency
        self.timer = self.create_timer(1.0 / REFRESH_RATE_HZ, self.publish_markers)

    def publish_markers(self):
        """
        Publish rangefinder localization markers at configured refresh rate.

        This method creates and publishes markers representing rangefinder localization
        data in 3D space. The markers are color-coded and scaled appropriately for visualization.
        """
        # Create a marker message (this is a placeholder, actual implementation needed)
        marker = Marker()
        marker.header.frame_id = "rangefinder_frame"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Radius of the sphere
        marker.scale.y = 0.1  # Radius of the sphere
        marker.scale.z = 0.1  # Radius of the sphere
        marker.color.a = 1.0  # Fully opaque
        marker.color.r = 0.0  # No red
        marker.color.g = 1.0  # Green for raw data
        marker.color.b = 0.0  # No blue
        marker.pose.position.x = 0.0  # Placeholder position
        marker.pose.position.y = 0.0  # Placeholder position
        marker.pose.position.z = 0.0  # Placeholder position
        marker.pose.orientation.w = 1.0  # No rotation
        marker.pose.orientation.x = 0.0  # No rotation
        marker.pose.orientation.y = 0.0  # No rotation
        marker.pose.orientation.z = 0.0  # No rotation
        marker.id = 0  # Unique ID for the marker

        # Publish the marker
        self.marker_publisher.publish(marker)
        self.logger.info("Published rangefinder localization marker")

def main(args=None):
    """
    Main entry point for the Rangefinder Localization Visualizer node.

    Initializes the ROS2 Python client library, creates the node, and spins it to
    process incoming messages and publish markers.
    """
    rclpy.init(args=args)
    node = RangefinderLocalizationVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down Rangefinder Localization Visualizer Node")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
