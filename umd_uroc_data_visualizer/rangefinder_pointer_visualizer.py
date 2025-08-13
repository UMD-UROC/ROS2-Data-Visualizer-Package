import os
import rclpy
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from rclpy.node import Node
from .qos_profile import BEST_EFFORT_QOS
import math
from visualization_msgs.msg import Marker
from sensor_msgs.msg import NavSatFix
from .node_utils import NodeShutdownHandler, setup_node_logging, log_periodic_status

# Load environment configuration from package share directory to get refresh rate
package_share_directory = get_package_share_directory("umd_uroc_data_visualizer")
load_dotenv(os.path.join(package_share_directory, ".env"))
# Refresh rate for transform publishing (Hz) - controls how often gimbal transforms are updated
REFRESH_RATE_HZ = float(os.getenv("REFRESH_RATE_HZ", 10.0))

class RangefinderPointerVisualizer(Node):
    """
    ROS2 node for visualizing rangefinder pointer data in 3D.

    This node creates and publishes visualization markers that represent rangefinder
    pointer data in 3D space. The markers are displayed in the rangefinder_frame
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
    - /drone/rangefinder/marker: Markers for rangefinder pointer data
    """

    def __init__(self, debug: bool = False):
        super().__init__("rangefinder_pointer_visualizer")

        # Setup logging
        self.logger = setup_node_logging(self, debug)
        self.debug = debug

        # Setup graceful shutdown handling
        self.shutdown_handler = NodeShutdownHandler(self)

        # Initialize counters for periodic status reporting
        self.publish_count = 0

        # Declare parameter for visualizer topic
        self.declare_parameter("visualizer_topic", "/mavros/gimbal_control/device/attitude_status")
        self.topic = self.get_parameter("visualizer_topic").value

        # Initialize marker and reference position for GPS conversion
        self.marker = Marker()
        self.reference_lat = None
        self.reference_lon = None
        self.has_valid_data = False

        # Setup subscriber for rangefinder pointer data
        self.create_subscription(
            NavSatFix,
            self.topic,
            self.rangefinder_pointer_location_callback,
            BEST_EFFORT_QOS,
        )

        # Set up publisher for rangefinder localization markers
        self.marker_publisher = self.create_publisher(
            Marker, "/drone/rangefinder/marker", BEST_EFFORT_QOS
        )

        # Timer to control marker publishing frequency
        self.timer = self.create_timer(1.0 / REFRESH_RATE_HZ, self.publish_markers)

        self.logger.info(f"Rangefinder localization visualizer initialized (debug={'enabled' if debug else 'disabled'})")
        if debug:
            self.logger.info(f"Publishing at {REFRESH_RATE_HZ} Hz")

    def rangefinder_pointer_location_callback(self, msg: NavSatFix):
        """
        Callback function for rangefinder pointer location data.

        This method receives NavSatFix messages containing rangefinder pointer location data.
        It converts GPS coordinates to local ENU coordinates and updates the marker.
        """

        # Skip invalid GPS data
        if msg.status.status < 0:  # Invalid GPS fix
            if self.debug:
                self.logger.debug("Received invalid GPS data, skipping")
            return

        # Set reference point on first valid message
        if self.reference_lat is None or self.reference_lon is None:
            self.reference_lat = msg.latitude
            self.reference_lon = msg.longitude
            self.logger.info(f"Set GPS reference point: lat={self.reference_lat:.6f}, lon={self.reference_lon:.6f}")

        # Convert GPS to local ENU coordinates
        x, y = self.gps_to_local_enu(msg.latitude, msg.longitude)
        z = msg.altitude  # Use altitude directly

        # Update marker position with converted coordinates
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        self.marker.pose.orientation.w = 1.0  # No rotation
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0

        # Set up marker properties
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.header.frame_id = "map"
        self.marker.id = 0
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0  # Green for rangefinder data
        self.marker.color.b = 0.0

        self.has_valid_data = True

        if self.debug:
            self.logger.debug(f"Updated marker position: x={x:.2f}, y={y:.2f}, z={z:.2f}")

    def gps_to_local_enu(self, lat, lon):
        """
        Convert GPS coordinates to local East-North-Up (ENU) coordinates.

        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees

        Returns:
            tuple: (x, y) coordinates in meters relative to reference point
        """
        # Check if reference coordinates are set
        if self.reference_lat is None or self.reference_lon is None:
            raise ValueError("Reference coordinates not set")

        # Convert degrees to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        ref_lat_rad = math.radians(self.reference_lat)
        ref_lon_rad = math.radians(self.reference_lon)

        # Earth radius in meters
        R = 6378137.0

        # Calculate differences
        dlat = lat_rad - ref_lat_rad
        dlon = lon_rad - ref_lon_rad

        # Convert to local ENU coordinates
        x = R * dlon * math.cos(ref_lat_rad)  # East
        y = R * dlat                          # North

        return x, y

    def publish_markers(self):
        """
        Publish rangefinder pointer markers at configured refresh rate.

        This method publishes the current marker data if valid GPS data has been received.
        """
        # Only publish if we have valid data
        if not self.has_valid_data:
            if self.debug and self.publish_count % 100 == 0:  # Log occasionally when waiting for data
                self.logger.debug("Waiting for valid GPS data to publish markers")
            return

        self.publish_count += 1

        # Update timestamp for current publication
        self.marker.header.stamp = self.get_clock().now().to_msg()

        # Publish the marker
        self.marker_publisher.publish(self.marker)

        # Debug-only status reporting
        if self.debug:
            log_periodic_status(
                self,
                f"Published rangefinder pointer marker at ({self.marker.pose.position.x:.2f}, {self.marker.pose.position.y:.2f}, {self.marker.pose.position.z:.2f})",
                self.publish_count,
                200  # Log every 200 publications
            )

def main(args=None):
    """
    Main entry point for the Rangefinder Localization Visualizer node.

    Initializes the ROS2 Python client library, creates the node, and spins it to
    process incoming messages and publish markers.
    """
    rclpy.init(args=args)

    # Check for debug flag in arguments
    debug = '--debug' in (args or [])

    try:
        node = RangefinderPointerVisualizer(debug=debug)
        # Use the new shutdown-aware spin method
        node.shutdown_handler.spin_with_shutdown()
    except KeyboardInterrupt:
        # Graceful shutdown is handled by NodeShutdownHandler
        pass
    except Exception as e:
        print(f"Unexpected error in rangefinder localization visualizer: {e}")
    finally:
        # Cleanup is handled by NodeShutdownHandler
        pass

if __name__ == "__main__":
    main()
