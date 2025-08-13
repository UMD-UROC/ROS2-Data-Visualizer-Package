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
from mavros_msgs.msg import GimbalManagerSetAttitude
from scipy.spatial.transform import Rotation as R
import traceback

from .node_utils import NodeShutdownHandler, setup_node_logging, log_periodic_status

# Load refresh rate from .env
package_share = get_package_share_directory("umd_uroc_data_visualizer")
load_dotenv(os.path.join(package_share, ".env"))
REFRESH_RATE_HZ = float(os.getenv("REFRESH_RATE_HZ", 10.0))

class GimbalFrame(Node):
    """ROS2 node that broadcasts the gimbal_frame relative to base_link."""

    def __init__(self, debug: bool = False):
        super().__init__("gimbal_frame")

        # Setup logging
        self.logger = setup_node_logging(self, debug)
        self.debug = debug

        # Setup graceful shutdown handling
        self.shutdown_handler = NodeShutdownHandler(self)

        # Initialize counters for periodic status reporting
        self.imu_callback_count = 0
        self.gimbal_callback_count = 0
        self.publish_loop_count = 0

        self.vehicle_q = [0.0, 0.0, 0.0, 1.0]
        self.gimbal_q_set_attitude = [0.0, 0.0, 0.0, 1.0]
        self.gimbal_q_attitude_status = [0.0, 0.0, 0.0, 1.0]

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            GimbalManagerSetAttitude,
            "/uas4/gimbal_control/manager/set_attitude",
            self.gimbal_callback_set_attitude,
            BEST_EFFORT_QOS
        )

        # Subscribe to vehicle IMU for body orientation
        self.create_subscription(
            Imu,
            "/uas4/imu/data",
            self._vehicle_imu_cb,
            BEST_EFFORT_QOS
        )

        # Timer for publishing transforms
        self.create_timer(1.0 / REFRESH_RATE_HZ, self.publish_loop)

        self.logger.info(f"Gimbal Frame publisher initialized (debug={'enabled' if debug else 'disabled'})")

    def ned_to_flu_quat(self, q_enu):
        """
        Convert a quaternion from ENU (East-North-Up) to FLU (Front-Left-Up).

        - Guards against zero-norm quaternions by defaulting to identity [0,0,0,1].
        - Normalizes all valid inputs to unit length before conversion.
        """
        # Debug logging for the conversion process
        if self.debug:
            self.logger.debug(f"Converting quaternion: {q_enu}")

        q = np.array(q_enu, dtype=float)
        norm = np.linalg.norm(q)

        if self.debug:
            self.logger.debug(f"Quaternion norm: {norm}")

        # Fallback on zero or invalid quaternion
        if norm < 1e-8 or not np.all(np.isfinite(q)):
            if self.debug:
                self.logger.debug("Invalid quaternion, returning identity")
            return np.array([0.0, 0.0, 0.0, 1.0])

        # Normalize quaternion
        q_normed = q / norm

        # Axis-flip matrix (ENU → FLU)
        R_conv = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1],
        ])

        try:
            # Build rotation object from unit ENU quaternion
            r_enu = R.from_quat(q_normed)
            # Transform into FLU frame
            R_flu = R_conv @ r_enu.as_matrix() @ R_conv.T
            result = R.from_matrix(R_flu).as_quat()

            if self.debug:
                self.logger.debug(f"Converted to FLU quaternion: {result}")

            return result
        except Exception as e:
            # If any error occurs, return identity quaternion
            self.logger.error(
                f"Invalid quaternion received, returning identity quaternion, error {e}"
            )
            if self.debug:
                traceback.print_exc()
            return np.array([0.0, 0.0, 0.0, 1.0])

    def _vehicle_imu_cb(self, msg: Imu):
        self.imu_callback_count += 1

        raw = msg.orientation
        self.vehicle_q = self.ned_to_flu_quat([
            raw.x, raw.y, raw.z, raw.w
        ])

        # Report data received for status dashboard
        if hasattr(self, 'shutdown_handler'):
            self.shutdown_handler.report_data_received()

        # Debug-only periodic status reporting (removed from normal operation)
        if self.debug:
            log_periodic_status(
                self,
                "Received vehicle IMU data",
                self.imu_callback_count,
                200  # Log every 200 messages in debug mode only
            )

    def gimbal_callback_set_attitude(self, msg: GimbalManagerSetAttitude):
        self.gimbal_callback_count += 1

        raw = msg.q
        self.gimbal_q_set_attitude = self.ned_to_flu_quat([
            raw.x, raw.y, raw.z, raw.w
        ])

        # Report data received for status dashboard
        if hasattr(self, 'shutdown_handler'):
            self.shutdown_handler.report_data_received()

        # Debug-only periodic status reporting (removed from normal operation)
        if self.debug:
            log_periodic_status(
                self,
                f"Received gimbal set attitude command",
                self.gimbal_callback_count,
                50  # Log every 50 messages in debug mode only
            )

    def publish_loop(self):
        self.publish_loop_count += 1

        # Choose which quaternion to broadcast
        q = (
            self.gimbal_q_set_attitude
        )

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

    # Check for debug flag in arguments
    debug = '--debug' in (args or [])

    try:
        node = GimbalFrame(debug=debug)
        # Use the new shutdown-aware spin method
        node.shutdown_handler.spin_with_shutdown()
    except KeyboardInterrupt:
        # Graceful shutdown is handled by NodeShutdownHandler
        pass
    except Exception as e:
        print(f"Unexpected error in gimbal frame: {e}")
    finally:
        # Cleanup is handled by NodeShutdownHandler
        pass


if __name__ == "__main__":
    main()
