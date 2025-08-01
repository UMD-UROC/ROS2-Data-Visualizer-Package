"""
Velocity vector visualization for UROC drone system.

This module creates real-time visualization of drone velocity vectors as 3D arrows
in the map frame. It subscribes to drone position and velocity data, then publishes
arrow markers that show the direction and magnitude of drone movement for use in
visualization tools like Foxglove.
"""

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from visualization_msgs.msg import Marker

from .qos_profile import BEST_EFFORT_QOS
from .node_utils import NodeShutdownHandler, setup_node_logging, log_periodic_status

# Load environment configuration from the package share directory
package_share_directory = get_package_share_directory("umd_uroc_data_visualizer")
load_dotenv(os.path.join(package_share_directory, ".env"))
# Refresh rate for velocity vector visualization updates (Hz)
REFRESH_RATE_HZ = float(os.getenv("REFRESH_RATE_HZ", 10.0))


class VelocityVectorVisualizer(Node):
    """
    ROS2 node that visualizes drone velocity vectors as 3D arrows.

    This node creates real-time visualization of drone velocity by drawing green
    arrows from the drone's current position in the direction of its velocity
    vector. The arrows are displayed in the map frame for global reference.

    Attributes
    ----------
    drone_velocity : list
        Current drone velocity in map frame [x, y, z] (m/s)
    drone_pos : list
        Current drone position in map frame [x, y, z] (m)
    target_velocity : list
        Target velocity from position commands [x, y, z] (m/s)
    target_pos : list
        Target position from position commands [x, y, z] (m)
    marker_pub : Publisher
        Publisher for velocity vector visualization markers
    timer : Timer
        Timer for periodic visualization updates

    """

    def __init__(self, debug: bool = False):
        """Initialize the VelocityVectorVisualizer node."""
        super().__init__("vector_visualizer_node")

        # Setup logging
        self.logger = setup_node_logging(self, debug)
        self.debug = debug
        
        # Setup graceful shutdown handling
        self.shutdown_handler = NodeShutdownHandler(self)
        
        # Initialize counters for periodic status reporting
        self.position_callback_count = 0
        self.pose_callback_count = 0
        self.publish_loop_count = 0

        # Initialize state variables for velocity and position tracking
        self.drone_velocity = [0.0, 0.0, 0.0]  # Current velocity (m/s)
        self.drone_pos = [0.0, 0.0, 0.0]        # Current position (m)
        self.target_velocity = [0.0, 0.0, 0.0]  # Target velocity (m/s)
        self.target_pos = [0.0, 0.0, 0.0]       # Target position (m)

        # Subscribe to position target messages for velocity data
        # Note: Velocity comes from MAVLink position targets in NED frame
        self.create_subscription(
            PositionTarget,
            "/mavros/setpoint_raw/local",
            self.on_local_position,
            BEST_EFFORT_QOS,
        )

        # Subscribe to drone pose for current position
        self.create_subscription(
            PoseStamped, "/drone/pose", self.on_drone_pos, BEST_EFFORT_QOS
        )

        # Publisher for velocity vector visualization markers
        self.marker_pub = self.create_publisher(
            Marker, "/drone/velocity_vector/marker", 1
        )

        # Timer for periodic visualization updates
        self.timer = self.create_timer(1.0 / REFRESH_RATE_HZ, self.publish_loop)
        
        self.logger.info(f"Velocity vector visualizer initialized (debug={'enabled' if debug else 'disabled'})")
        if debug:
            self.logger.info(f"Publishing at {REFRESH_RATE_HZ} Hz")

    def mavV_to_rosV(self, mavV):
        """
        Convert MAVLink NED velocity vector to ROS ENU velocity vector.

        MAVLink uses North-East-Down (NED) coordinate system while ROS uses
        East-North-Up (ENU). This function performs the coordinate transformation.

        Parameters
        ----------
        mavV : list
            Velocity vector in MAVLink NED frame [north, east, down]

        Returns
        -------
        list
            Velocity vector in ROS ENU frame [east, north, up]

        """
        # NED to ENU conversion: [N,E,D] -> [E,N,-D]
        return [mavV[1], mavV[0], -mavV[2]]

    def on_local_position(self, target_msg: PositionTarget):
        """
        Handle incoming position target messages containing velocity data.

        Extracts velocity information from MAVLink position targets and converts
        from NED to ENU coordinate frame for ROS visualization.

        Parameters
        ----------
        target_msg : PositionTarget
            Position target message with velocity data

        """
        self.position_callback_count += 1
        
        # Extract velocity from position target and convert NED to ENU
        self.drone_velocity = self.mavV_to_rosV(
            [target_msg.velocity.x, target_msg.velocity.y, target_msg.velocity.z]
        )
        
        # Report data received for status dashboard
        if hasattr(self, 'shutdown_handler'):
            self.shutdown_handler.report_data_received()
        
        # Debug-only status reporting (control center doesn't need velocity details)
        if self.debug:
            log_periodic_status(
                self,
                f"Received velocity [{self.drone_velocity[0]:.2f}, {self.drone_velocity[1]:.2f}, {self.drone_velocity[2]:.2f}] m/s",
                self.position_callback_count,
                50  # Log every 50 messages
            )
            self.logger.debug(f"Updated velocity: {self.drone_velocity}")

    def on_drone_pos(self, drone_pose_msg: PoseStamped):
        """
        Handle incoming drone pose messages.

        Updates the current drone position for velocity vector visualization
        starting point.

        Parameters
        ----------
        drone_pose_msg : PoseStamped
            Current drone pose in map frame

        """
        self.pose_callback_count += 1
        
        self.drone_pos = [
            drone_pose_msg.pose.position.x,
            drone_pose_msg.pose.position.y,
            drone_pose_msg.pose.position.z,
        ]
        
        # Report data received for status dashboard
        if hasattr(self, 'shutdown_handler'):
            self.shutdown_handler.report_data_received()
        
        # Debug-only status reporting (control center doesn't need position details)
        if self.debug:
            log_periodic_status(
                self,
                f"Received pose at [{self.drone_pos[0]:.2f}, {self.drone_pos[1]:.2f}, {self.drone_pos[2]:.2f}]",
                self.pose_callback_count,
                50  # Log every 50 messages
            )
            self.logger.debug(f"Updated position: {self.drone_pos}")

    def publish_loop(self):
        """
        Publish velocity vector markers.

        Creates and publishes arrow markers showing the drone's velocity vector
        as a green arrow pointing from current position in the direction of motion.
        The arrow length represents velocity magnitude.
        """
        self.publish_loop_count += 1
        
        # Use latest message timestamp if available, otherwise current time
        if hasattr(self, "latest_header"):
            stamp = self.latest_header.stamp
        else:
            stamp = self.get_clock().now().to_msg()

        # Calculate target position for velocity vector visualization
        # End point = current position + velocity vector
        target_pos = [
            self.drone_pos[0] + self.drone_velocity[0],
            self.drone_pos[1] + self.drone_velocity[1],
            self.drone_pos[2] + self.drone_velocity[2],
        ]

        # Create arrow marker for velocity vector visualization
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "map"  # Global ENU reference frame
        marker.ns = "velocity_vector_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Define arrow start and end points in global ENU coordinates
        start_point = Point(
            x=self.drone_pos[0], y=self.drone_pos[1], z=self.drone_pos[2]
        )
        end_point = Point(
            x=target_pos[0], y=target_pos[1], z=target_pos[2]
        )
        marker.points = [start_point, end_point]

        # Set arrow visual properties
        marker.scale.x = 0.1  # Arrow shaft diameter
        marker.scale.y = 0.2  # Arrow head width
        marker.scale.z = 0.2  # Arrow head height

        # Green color for velocity vectors
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque

        # Publish the velocity vector marker
        self.marker_pub.publish(marker)
        
        # Debug-only status reporting (control center doesn't need velocity magnitude details)
        if self.debug:
            # Calculate velocity magnitude for debug reporting
            velocity_magnitude = (self.drone_velocity[0]**2 + self.drone_velocity[1]**2 + self.drone_velocity[2]**2)**0.5
            log_periodic_status(
                self,
                f"Published velocity vector (magnitude: {velocity_magnitude:.2f} m/s)",
                self.publish_loop_count,
                100  # Log every 100 publications
            )
            self.logger.debug(f"Published velocity arrow from {self.drone_pos} to {target_pos}")


def main(args=None):
    """
    Execute the velocity vector visualizer node.

    Parameters
    ----------
    args : list, optional
        Command line arguments

    """
    rclpy.init(args=args)
    
    # Check for debug flag in arguments
    debug = '--debug' in (args or [])
    
    try:
        node = VelocityVectorVisualizer(debug=debug)
        node.get_logger().info("UROC Vector Visualizer Node started")
        # Use the new shutdown-aware spin method
        node.shutdown_handler.spin_with_shutdown()
    except KeyboardInterrupt:
        # Graceful shutdown is handled by NodeShutdownHandler
        pass
    except Exception as e:
        print(f"Unexpected error in velocity vector visualizer: {e}")
    finally:
        # Cleanup is handled by NodeShutdownHandler
        pass


if __name__ == "__main__":
    main()
