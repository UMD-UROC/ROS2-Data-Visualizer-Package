# gimbal_frame.py

"""
Gimbal frame transform publisher for UROC visualization system.

Publishes transforms to show gimbal orientation relative to the vehicle body
in 3D viewers (e.g. Foxglove). Includes robust quaternion handling to avoid
runtime crashes on startup or malformed inputs.
"""

import os
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from .qos_profile import BEST_EFFORT_QOS
from mavros_msgs.msg import GimbalDeviceAttitudeStatus, GimbalDeviceSetAttitude
from scipy.spatial.transform import Rotation as R

# Load refresh rate from .env
package_share = get_package_share_directory("umd_uroc_data_visualizer")
load_dotenv(os.path.join(package_share, ".env"))
REFRESH_RATE_HZ = float(os.getenv("REFRESH_RATE_HZ", 10.0))

class GimbalFrame(Node):
    """ROS2 node that broadcasts the gimbal_frame relative to base_link."""

    def __init__(self):
        super().__init__("gimbal_frame")
        self.vehicle_q = [0.0, 0.0, 0.0, 1.0]
        self.gimbal_q_set_attitude    = [0.0, 0.0, 0.0, 1.0]
        self.gimbal_q_attitude_status = [0.0, 0.0, 0.0, 1.0]

        self.tf_broadcaster = TransformBroadcaster(self)
        self.declare_parameter("visualizer_topic", "/mavros/gimbal_control/device/attitude_status")
        topic = self.get_parameter("visualizer_topic").value

        # Subscribe based on chosen topic
        if topic.endswith("set_attitude"):
            self.create_subscription(
                GimbalDeviceSetAttitude, topic,
                self.gimbal_callback_set_attitude, BEST_EFFORT_QOS
            )
        elif topic.endswith("attitude_status"):
            self.create_subscription(
                GimbalDeviceAttitudeStatus, topic,
                self.gimbal_callback_attitude_status, BEST_EFFORT_QOS
            )
        else:
            self.get_logger().error(f"Unsupported visualizer_topic: {topic}")
            rclpy.shutdown()

        # IMU gives us vehicle body orientation
        self.create_subscription(
            Imu, "/mavros/imu/data",
            self._vehicle_imu_cb, BEST_EFFORT_QOS
        )

        # Publish at fixed rate
        self.create_timer(1.0 / REFRESH_RATE_HZ, self.publish_loop)
        self.get_logger().info("Gimbal Frame Publisher Started")

    def ned_to_flu_quat(self, q_enu):
        """
        Convert a quaternion from ENU (East-North-Up) to FLU (Front-Left-Up).

        - Guards against zero-norm quaternions by defaulting to identity [0,0,0,1].
        - Normalizes all valid inputs to unit length before conversion.
        """
        # --- Hardening: guard & normalize ---
        q = np.array(q_enu, dtype=float)
        norm = np.linalg.norm(q)
        if norm < 1e-8:
            # Fallback to no rotation rather than crashing
            return np.array([0.0, 0.0, 0.0, 1.0])
        q = norm

        # Axis-flip matrix (ENU → FLU)
        R_conv = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1],
        ])

        # Try - Except (Fixes issue if a null quaternion is received)
        # This is mainly a dev fix to prevent crashes on startup if PX4 is not ready
        try:
            # Build rotation object from unit ENU quaternion
            r_enu = R.from_quat(q)
            # Transform into FLU frame
            R_flu = R_conv @ r_enu.as_matrix() @ R_conv.T

            self.get_logger().debug(f"Received quaternion: {q_enu}")
            self.get_logger().debug(f"Norm: {norm}")
            self.get_logger().debug(f"r_enu: {r_enu}")
            # Return FLU quaternion
            return R.from_matrix(R_flu).as_quat()
        except:
            # If any error occurs, return identity quaternion
            self.get_logger().error("Invalid quaternion received, returning identity quaternion.")
            return np.array([0.0, 0.0, 0.0, 1.0])


    def _vehicle_imu_cb(self, msg: Imu):
        raw = msg.orientation
        self.vehicle_q = self.ned_to_flu_quat([
            raw.x, raw.y, raw.z, raw.w
        ])

    def gimbal_callback_set_attitude(self, msg: GimbalDeviceSetAttitude):
        raw = msg.q
        self.gimbal_q_set_attitude = self.ned_to_flu_quat([
            raw.x, raw.y, raw.z, raw.w
        ])

    def gimbal_callback_attitude_status(self, msg: GimbalDeviceAttitudeStatus):
        raw = msg.q
        # Absolute gimbal → FLU
        q_abs = self.ned_to_flu_quat([raw.x, raw.y, raw.z, raw.w])
        # Compute relative: vehicle⁻¹ * gimbal
        r_vehicle = R.from_quat(self.vehicle_q)
        r_gimbal  = R.from_quat(q_abs)
        self.gimbal_q_attitude_status = (r_vehicle.inv() * r_gimbal).as_quat()

    def publish_loop(self):
        # Choose which quaternion to broadcast
        q = self.gimbal_q_set_attitude if self.get_parameter("visualizer_topic").value.endswith("set_attitude") \
            else self.gimbal_q_attitude_status

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id  = "gimbal_frame"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalFrame()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()

