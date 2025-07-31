# gimbal_frame.py

"""
Gimbal frame transform publisher for UROC visualization.
Includes debug logs to trace quaternion values.
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

# Load refresh rate from environment
package_share_directory = get_package_share_directory("umd_uroc_data_visualizer")
load_dotenv(os.path.join(package_share_directory, ".env"))
REFRESH_RATE_HZ = float(os.getenv("REFRESH_RATE_HZ", 0.1))

def ned_to_flu_quat(q_enu):
    """
    Convert ENU quaternion to FLU by flipping Y and Z axes.
    """
    # Conversion matrix from ENU to NED/FLU conventions
    R_conv = np.array([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, -1]
    ])
    # Interpret input as quaternion in ENU
    r_enu = R.from_quat(q_enu)
    # Transform into equivalent in FLU/NED frame
    R_transformed = R_conv @ r_enu.as_matrix() @ R_conv.T
    r_flu = R.from_matrix(R_transformed)
    return r_flu.as_quat()

class GimbalFrame(Node):
    def __init__(self):
        super().__init__("gimbal_frame")
        # Enable debug logs
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Gimbal orientation quaternions
        self.gimbal_q_set_attitude = [0.0, 0.0, 0.0, 1.0]
        self.gimbal_q_attitude_status = [0.0, 0.0, 0.0, 1.0]
        # Vehicle (drone) body quaternion in FLU
        self.vehicle_q = [0.0, 0.0, 0.0, 1.0]

        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to vehicle IMU to get body attitude
        self.create_subscription(
            Imu,
            "/mavros/imu/data",
            self._vehicle_imu_cb,
            BEST_EFFORT_QOS,
        )

        # Parameter to select which gimbal topic to visualize
        self.declare_parameter("visualizer_topic", "/mavros/gimbal_control/device/attitude_status")
        self.visualizer_topic = self.get_parameter("visualizer_topic").value

        # Subscribe to the appropriate gimbal topic
        if self.visualizer_topic == "/mavros/gimbal_control/device/set_attitude":
            self.create_subscription(
                GimbalDeviceSetAttitude,
                self.visualizer_topic,
                self.gimbal_callback_set_attitude,
                BEST_EFFORT_QOS,
            )
        elif self.visualizer_topic == "/mavros/gimbal_control/device/attitude_status":
            self.create_subscription(
                GimbalDeviceAttitudeStatus,
                self.visualizer_topic,
                self.gimbal_callback_attitude_status,
                BEST_EFFORT_QOS,
            )
        else:
            self.get_logger().error(f"Unsupported visualizer_topic: {self.visualizer_topic}")
            rclpy.shutdown()
            return

        # Timer for publishing transforms
        self.timer = self.create_timer(1.0 / REFRESH_RATE_HZ, self.publish_loop)

    def _vehicle_imu_cb(self, msg):
        # Raw IMU orientation (should be in ENU)
        raw = msg.orientation
        """Log the raw IMU orientation for debugging
        self.get_logger().debug(
            f"Vehicle IMU raw orientation: x={raw.x:.3f}, y={raw.y:.3f}, z={raw.z:.3f}, w={raw.w:.3f}"
        )
        """
        # Convert to FLU frame
        self.vehicle_q = ned_to_flu_quat([
            float(raw.x), float(raw.y), float(raw.z), float(raw.w)
        ])
        #self.get_logger().debug(f"Vehicle quaternion (FLU): {self.vehicle_q}")

    def gimbal_callback_set_attitude(self, msg):
        raw = msg.q
        self.gimbal_q_set_attitude = ned_to_flu_quat([
            float(raw.x), float(raw.y), float(raw.z), float(raw.w)
        ])
        # self.get_logger().debug(f"Gimbal set_attitude quaternion (FLU): {self.gimbal_q_set_attitude}")

    def gimbal_callback_attitude_status(self, msg):
        raw = msg.q
        """Log the raw gimbal orientation for debugging
        self.get_logger().debug(
            f"Gimbal status raw orientation: x={raw.x:.3f}, y={raw.y:.3f}, z={raw.z:.3f}, w={raw.w:.3f}"
        )
        """
        # Convert absolute gimbal orientation to FLU
        q_gimbal = ned_to_flu_quat([
            float(raw.x), float(raw.y), float(raw.z), float(raw.w)
        ])
        #self.get_logger().debug(f"Gimbal quaternion (FLU): {q_gimbal}")

        # Compute relative orientation: remove vehicle body rotation
        r_vehicle = R.from_quat(self.vehicle_q)
        r_gimbal = R.from_quat(q_gimbal)
        r_rel = r_vehicle.inv() * r_gimbal
        self.gimbal_q_attitude_status = r_rel.as_quat()
        #self.get_logger().debug(f"Relative quaternion: {self.gimbal_q_attitude_status}")

    def publish_loop(self):
        # Create and broadcast the transform
        stamp = self.get_clock().now().to_msg()
        gimbal_tf = TransformStamped()
        gimbal_tf.header.stamp = stamp
        gimbal_tf.header.frame_id = 'base_link'
        gimbal_tf.child_frame_id = 'gimbal_frame'
        gimbal_tf.transform.translation.x = 0.0
        gimbal_tf.transform.translation.y = 0.0
        gimbal_tf.transform.translation.z = 0.0

        # Select quaternion based on topic
        if self.visualizer_topic == "/mavros/gimbal_control/device/set_attitude":
            q = self.gimbal_q_set_attitude
        else:
            q = self.gimbal_q_attitude_status

        gimbal_tf.transform.rotation.x = q[0]
        gimbal_tf.transform.rotation.y = q[1]
        gimbal_tf.transform.rotation.z = q[2]
        gimbal_tf.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(gimbal_tf)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalFrame()
    node.get_logger().info("Gimbal Frame Publisher Started")
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

