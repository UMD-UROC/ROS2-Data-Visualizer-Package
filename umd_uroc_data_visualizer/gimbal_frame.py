# gimbal_frame.py

"""
Gimbal frame transform publisher for UROC visualization system.

This module creates and publishes coordinate frame transforms for gimbal visualization
in the UROC drone system. It handles coordinate conversions between different frame
conventions (ENU/NED/FLU) and publishes transforms that allow 3D visualization tools
like Foxglove to properly display gimbal orientation relative to the drone body.

The node subscribes to both vehicle IMU data and gimbal attitude messages, computes
relative gimbal orientation, and publishes the appropriate transforms to the /tf topic.
Includes comprehensive debug logging to trace quaternion transformations.
"""

import os
import numpy as np
import rclpy
import rclpy.logging
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from .qos_profile import BEST_EFFORT_QOS
from mavros_msgs.msg import GimbalDeviceAttitudeStatus, GimbalDeviceSetAttitude
from scipy.spatial.transform import Rotation as R

# Load environment configuration from package share directory to get refresh rate
package_share_directory = get_package_share_directory("umd_uroc_data_visualizer")
load_dotenv(os.path.join(package_share_directory, ".env"))
# Refresh rate for transform publishing (Hz) - controls how often gimbal transforms are updated
REFRESH_RATE_HZ = float(os.getenv("REFRESH_RATE_HZ", 0.1))

def ned_to_flu_quat(q_enu):
    """
    Convert quaternion from ENU (East-North-Up) to FLU (Front-Left-Up) coordinate frame.
    
    This function performs coordinate frame conversion for gimbal orientation data.
    The input quaternion represents rotation in ENU frame (typical for geographic/mapping)
    and converts it to FLU frame (typical for vehicle body coordinates).
    
    The conversion involves rotating the coordinate system by flipping Y and Z axes:
    - X (East/Front) remains the same
    - Y (North) becomes -Y (Left) 
    - Z (Up) becomes -Z (Up in vehicle frame)
    
    Parameters
    ----------
    q_enu : array-like
        Input quaternion [x, y, z, w] in ENU coordinate frame
        
    Returns
    -------
    numpy.ndarray
        Output quaternion [x, y, z, w] in FLU coordinate frame
        
    Notes
    -----
    Uses scipy.spatial.transform.Rotation for robust quaternion operations.
    The conversion matrix R_conv represents the coordinate frame transformation.
    """
    # Conversion matrix from ENU to NED/FLU conventions
    # This matrix performs the axis flipping: X->X, Y->-Y, Z->-Z
    R_conv = np.array([
        [1, 0, 0],    # X axis unchanged (East -> Front)
        [0, -1, 0],   # Y axis flipped (North -> -Left)
        [0, 0, -1]    # Z axis flipped (Up -> -Up)
    ])
    # Convert input quaternion to rotation matrix in ENU frame
    r_enu = R.from_quat(q_enu)
    # Apply coordinate transformation: R_conv * R_enu * R_conv^T
    # This transforms the rotation matrix from ENU to FLU coordinate system
    R_transformed = R_conv @ r_enu.as_matrix() @ R_conv.T
    # Convert back to quaternion representation in FLU frame
    r_flu = R.from_matrix(R_transformed)
    return r_flu.as_quat()

class GimbalFrame(Node):
    """
    ROS2 node for publishing gimbal coordinate frame transforms.
    
    This node creates and maintains the transform relationship between the vehicle
    body frame (base_link) and the gimbal frame. It subscribes to vehicle IMU data
    and gimbal attitude messages, then computes and publishes the appropriate
    transform to enable 3D visualization of gimbal orientation.
    
    The node supports two modes of operation based on configuration:
    1. Set Attitude Mode: Visualizes commanded gimbal orientation
    2. Attitude Status Mode: Visualizes actual gimbal orientation with
       compensation for vehicle body movement
       
    Key Features:
    - Real-time coordinate frame conversion (ENU ↔ FLU)
    - Vehicle body motion compensation for relative gimbal orientation
    - Configurable topic selection for different gimbal data sources
    - Debug logging for quaternion value tracing
    
    Subscribed Topics:
    - /mavros/imu/data: Vehicle body attitude (IMU data)
    - /mavros/gimbal_control/device/set_attitude: Commanded gimbal attitude
    - /mavros/gimbal_control/device/attitude_status: Actual gimbal attitude
    
    Published Topics:
    - /tf: Transform from base_link to gimbal_frame
    
    Parameters:
    - visualizer_topic: Selects which gimbal topic to use for transform calculation
    """
    
    def __init__(self):
        """
        Initialize the GimbalFrame node with subscribers, publishers, and parameters.
        
        Sets up subscriptions to vehicle IMU and gimbal attitude topics,
        configures transform broadcasting, and initializes quaternion storage
        for coordinate frame calculations.
        """
        super().__init__("gimbal_frame")
        # Enable debug-level logging for detailed quaternion tracing
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Initialize gimbal orientation quaternions for both attitude modes
        # These store the most recent gimbal orientations in FLU coordinate frame
        self.gimbal_q_set_attitude = [0.0, 0.0, 0.0, 1.0]      # Commanded gimbal orientation
        self.gimbal_q_attitude_status = [0.0, 0.0, 0.0, 1.0]   # Actual gimbal orientation (relative to vehicle)
        # Vehicle (drone) body quaternion in FLU frame from IMU data
        self.vehicle_q = [0.0, 0.0, 0.0, 1.0]

        # Initialize transform broadcaster for publishing gimbal frame transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to vehicle IMU data to track body attitude for relative gimbal calculations
        self.create_subscription(
            Imu,
            "/mavros/imu/data",
            self._vehicle_imu_cb,
            BEST_EFFORT_QOS,
        )

        # Parameter to select which gimbal topic to visualize
        # Options: set_attitude (commands) or attitude_status (actual orientation)
        self.declare_parameter("visualizer_topic", "/mavros/gimbal_control/device/attitude_status")
        self.visualizer_topic = self.get_parameter("visualizer_topic").value

        # Subscribe to the appropriate gimbal topic based on configuration
        if self.visualizer_topic == "/mavros/gimbal_control/device/set_attitude":
            # Subscribe to commanded gimbal attitudes (what the gimbal should do)
            self.create_subscription(
                GimbalDeviceSetAttitude,
                self.visualizer_topic,
                self.gimbal_callback_set_attitude,
                BEST_EFFORT_QOS,
            )
        elif self.visualizer_topic == "/mavros/gimbal_control/device/attitude_status":
            # Subscribe to actual gimbal attitudes (what the gimbal is doing)
            self.create_subscription(
                GimbalDeviceAttitudeStatus,
                self.visualizer_topic,
                self.gimbal_callback_attitude_status,
                BEST_EFFORT_QOS,
            )
        else:
            # Invalid topic configuration - log error and shutdown
            self.get_logger().error(f"Unsupported visualizer_topic: {self.visualizer_topic}")
            rclpy.shutdown()
            return

        # Timer for periodic transform publishing at configured refresh rate
        self.timer = self.create_timer(1.0 / REFRESH_RATE_HZ, self.publish_loop)

    def _vehicle_imu_cb(self, msg):
        """
        Callback for vehicle IMU data - updates vehicle body attitude.
        
        Processes incoming IMU data to track the vehicle's body orientation,
        which is needed for computing relative gimbal orientation in attitude_status mode.
        The IMU data is converted from ENU to FLU coordinate frame for consistency.
        
        Parameters
        ----------
        msg : sensor_msgs.msg.Imu
            IMU message containing vehicle orientation quaternion in ENU frame
            
        Notes
        -----
        Commented debug logs are available for troubleshooting coordinate transformations.
        """
        # Extract raw IMU orientation quaternion (typically in ENU coordinate frame)
        raw = msg.orientation
        # Uncomment for debugging vehicle orientation values:
        # self.get_logger().debug(
        #     f"Vehicle IMU raw orientation: x={raw.x:.3f}, y={raw.y:.3f}, z={raw.z:.3f}, w={raw.w:.3f}"
        # )
        
        # Convert vehicle orientation from ENU to FLU coordinate frame
        self.vehicle_q = ned_to_flu_quat([
            float(raw.x), float(raw.y), float(raw.z), float(raw.w)
        ])
        # Uncomment for debugging converted vehicle quaternion:
        # self.get_logger().debug(f"Vehicle quaternion (FLU): {self.vehicle_q}")

    def gimbal_callback_set_attitude(self, msg):
        """
        Callback for gimbal set attitude commands - updates commanded gimbal orientation.
        
        Processes incoming gimbal attitude command messages and converts the orientation
        to FLU coordinate frame. This represents the desired/commanded gimbal orientation
        rather than the actual orientation.
        
        Parameters
        ----------
        msg : mavros_msgs.msg.GimbalDeviceSetAttitude
            Gimbal set attitude message containing commanded orientation quaternion
            
        Notes
        -----
        Used when visualizer_topic is set to "/mavros/gimbal_control/device/set_attitude"
        """
        # Extract commanded gimbal orientation quaternion
        raw = msg.q
        # Convert commanded gimbal orientation from ENU to FLU coordinate frame
        self.gimbal_q_set_attitude = ned_to_flu_quat([
            float(raw.x), float(raw.y), float(raw.z), float(raw.w)
        ])
        # Uncomment for debugging commanded gimbal quaternion:
        # self.get_logger().debug(f"Gimbal set_attitude quaternion (FLU): {self.gimbal_q_set_attitude}")

    def gimbal_callback_attitude_status(self, msg):
        """
        Callback for gimbal attitude status - updates actual gimbal orientation relative to vehicle.
        
        Processes incoming gimbal attitude status messages and computes the relative gimbal
        orientation by removing the vehicle body motion. This provides the gimbal's orientation
        relative to the vehicle body frame rather than the global frame.
        
        Parameters
        ----------
        msg : mavros_msgs.msg.GimbalDeviceAttitudeStatus
            Gimbal attitude status message containing actual orientation quaternion
            
        Notes
        -----
        This callback performs vehicle body motion compensation by computing:
        relative_gimbal_orientation = vehicle_orientation^(-1) * absolute_gimbal_orientation
        
        Used when visualizer_topic is set to "/mavros/gimbal_control/device/attitude_status"
        """
        # Extract actual gimbal orientation quaternion from status message
        raw = msg.q
        # Uncomment for debugging raw gimbal orientation values:
        # self.get_logger().debug(
        #     f"Gimbal status raw orientation: x={raw.x:.3f}, y={raw.y:.3f}, z={raw.z:.3f}, w={raw.w:.3f}"
        # )
        
        # Convert absolute gimbal orientation from ENU to FLU coordinate frame
        q_gimbal = ned_to_flu_quat([
            float(raw.x), float(raw.y), float(raw.z), float(raw.w)
        ])
        # Uncomment for debugging converted gimbal quaternion:
        # self.get_logger().debug(f"Gimbal quaternion (FLU): {q_gimbal}")

        # Compute relative gimbal orientation by removing vehicle body rotation
        # This gives us the gimbal orientation relative to the vehicle body frame
        r_vehicle = R.from_quat(self.vehicle_q)      # Vehicle body rotation
        r_gimbal = R.from_quat(q_gimbal)             # Absolute gimbal rotation
        r_rel = r_vehicle.inv() * r_gimbal           # Relative rotation: gimbal w.r.t. vehicle
        self.gimbal_q_attitude_status = r_rel.as_quat()
        # Uncomment for debugging relative quaternion:
        # self.get_logger().debug(f"Relative quaternion: {self.gimbal_q_attitude_status}")

    def publish_loop(self):
        """
        Timer callback that publishes gimbal coordinate frame transforms.
        
        Creates and broadcasts a transform from 'base_link' to 'gimbal_frame' using
        the appropriate gimbal quaternion based on the configured visualizer topic.
        This transform enables 3D visualization tools to properly display gimbal
        orientation relative to the vehicle body.
        
        Transform Details:
        - Parent frame: 'base_link' (vehicle body frame)
        - Child frame: 'gimbal_frame' (gimbal coordinate frame)  
        - Translation: Zero (gimbal rotates at vehicle center)
        - Rotation: Gimbal orientation quaternion (set_attitude or attitude_status)
        
        Notes
        -----
        Called periodically at REFRESH_RATE_HZ to maintain up-to-date transforms.
        """
        # Get current timestamp for the transform
        stamp = self.get_clock().now().to_msg()
        
        # Create transform message from base_link to gimbal_frame
        gimbal_tf = TransformStamped()
        gimbal_tf.header.stamp = stamp
        gimbal_tf.header.frame_id = 'base_link'     # Parent frame (vehicle body)
        gimbal_tf.child_frame_id = 'gimbal_frame'   # Child frame (gimbal)
        
        # Set translation to zero - gimbal rotates at vehicle center point
        gimbal_tf.transform.translation.x = 0.0
        gimbal_tf.transform.translation.y = 0.0
        gimbal_tf.transform.translation.z = 0.0

        # Select appropriate quaternion based on configured topic
        if self.visualizer_topic == "/mavros/gimbal_control/device/set_attitude":
            # Use commanded gimbal orientation for set_attitude mode
            q = self.gimbal_q_set_attitude
        else:
            # Use actual relative gimbal orientation for attitude_status mode
            q = self.gimbal_q_attitude_status

        # Set rotation quaternion for the transform
        gimbal_tf.transform.rotation.x = q[0]
        gimbal_tf.transform.rotation.y = q[1]
        gimbal_tf.transform.rotation.z = q[2]
        gimbal_tf.transform.rotation.w = q[3]

        # Broadcast the transform to the /tf topic
        self.tf_broadcaster.sendTransform(gimbal_tf)


def main(args=None):
    """
    Main entry point for the gimbal frame publisher node.
    
    Initializes ROS2, creates the GimbalFrame node, and runs the main event loop.
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
    
    # Create and start the gimbal frame publisher node
    node = GimbalFrame()
    node.get_logger().info("Gimbal Frame Publisher Started")
    
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

