"""PyMAVLink to ROS2 bridge for gimbal and position messages."""

import threading

import rclpy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import GimbalDeviceSetAttitude, PositionTarget
from pymavlink import mavutil
from rclpy.node import Node
from std_msgs.msg import Header

from .qos_profile import BEST_EFFORT_QOS, RELIABLE_QOS


class MAVLinkGimbalBridge(Node):
    """Bridge that converts MAVLink messages to ROS2 messages."""

    def __init__(self):
        """Initialize the MAVLink bridge node."""
        super().__init__("mavlink_gimbal_bridge")

        # Declare configuration parameters with defaults
        self.declare_parameter("mavlink_connection", "udp:localhost:14445")
        self.declare_parameter("system_id", 1)
        self.declare_parameter("component_id", 1)

        # Retrieve configuration parameters
        self.mavlink_connection = self.get_parameter("mavlink_connection").value
        self.system_id = self.get_parameter("system_id").value
        self.component_id = self.get_parameter("component_id").value

        # Initialize ROS2 publishers for converted messages
        self.gimbal_pub = self.create_publisher(
            GimbalDeviceSetAttitude,
            "/mavros/gimbal_control/device/set_attitude",
            RELIABLE_QOS,  # Use reliable QoS for critical gimbal commands
        )
        self.vector_pub = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", BEST_EFFORT_QOS
        )

        # Establish MAVLink connection with error handling
        try:
            self.mavlink_connection_obj = mavutil.mavlink_connection(
                self.mavlink_connection,
                source_system=self.system_id,
                source_component=self.component_id,
            )
            self.get_logger().info(f"Connected to MAVLink on {self.mavlink_connection}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MAVLink: {e}")
            return

        # Start background thread for MAVLink message processing
        self.mavlink_thread = threading.Thread(
            target=self.mavlink_listener, daemon=True
        )
        self.mavlink_thread.start()

        self.get_logger().info("MAVLink Bridge started")

    def mavlink_listener(self):  # noqa: C901
        """
        Listen to MAVLink messages in background thread.

        Continuously monitors the MAVLink connection for incoming messages
        and dispatches them to appropriate processing functions. Runs in
        a separate thread to avoid blocking ROS2 operations.

        Note: C901 complexity warning suppressed as this is a message
        dispatcher that naturally has multiple conditional branches.
        """
        while rclpy.ok():
            try:
                # Wait for incoming MAVLink messages with timeout
                msg = self.mavlink_connection_obj.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue  # Timeout, try again

                # Get message type for dispatch
                mtype = msg.get_type()

                # Dispatch to appropriate handler based on message type
                if mtype == "GIMBAL_DEVICE_SET_ATTITUDE":
                    self.process_gimbal_message(msg)
                elif mtype == "POSITION_TARGET_LOCAL_NED":
                    self.process_position_target(msg)

            except Exception as e:
                self.get_logger().error(
                    f"Error receiving or processing MAVLink message: {e}"
                )
                continue

    def process_gimbal_message(self, mavlink_msg):
        """
        Convert MAVLink gimbal attitude message to ROS2 format.

        Processes GIMBAL_DEVICE_SET_ATTITUDE messages from MAVLink and converts
        them to ROS2 GimbalDeviceSetAttitude messages with proper coordinate
        frame transformations.

        Parameters
        ----------
        mavlink_msg : MAVLink message
            MAVLink GIMBAL_DEVICE_SET_ATTITUDE message

        """
        try:
            ros_msg = GimbalDeviceSetAttitude()

            # Copy target identification and control flags
            ros_msg.target_system = mavlink_msg.target_system
            ros_msg.target_component = mavlink_msg.target_component
            ros_msg.flags = mavlink_msg.flags

            # Convert quaternion from MAVLink [w, x, y, z] to ROS [x, y, z, w]
            # Also apply coordinate frame transformation for gimbal orientation
            ros_msg.q = Quaternion(
                x=float(mavlink_msg.q[1]),   # MAVLink x -> ROS x
                y=-float(mavlink_msg.q[2]),  # MAVLink y -> ROS -y (coord transform)
                z=-float(mavlink_msg.q[3]),  # MAVLink z -> ROS -z (coord transform)
                w=float(mavlink_msg.q[0]),   # MAVLink w -> ROS w
            )

            # Copy angular velocity commands (rad/s)
            ros_msg.angular_velocity_x = float(mavlink_msg.angular_velocity_x)
            ros_msg.angular_velocity_y = float(mavlink_msg.angular_velocity_y)
            ros_msg.angular_velocity_z = float(mavlink_msg.angular_velocity_z)

            # Publish converted message to ROS2 topic
            self.gimbal_pub.publish(ros_msg)

            # Debug logging for gimbal attitude commands
            self.get_logger().debug(
                f"Published gimbal attitude: q=[{ros_msg.q.x:.3f}, {ros_msg.q.y:.3f}, "
                f"{ros_msg.q.z:.3f}, {ros_msg.q.w:.3f}]"
            )

        except Exception as e:
            self.get_logger().error(f"Error processing gimbal message: {e}")

    def process_position_target(self, mavlink_msg):
        """
        Convert MAVLink position target to ROS2 format.

        Processes POSITION_TARGET_LOCAL_NED messages from MAVLink and converts
        them to ROS2 PositionTarget messages with NED to ENU coordinate
        frame transformation.

        Parameters
        ----------
        mavlink_msg : MAVLink message
            MAVLink POSITION_TARGET_LOCAL_NED message

        """
        try:
            ros_msg = PositionTarget()

            # Create header with current timestamp and map frame
            ros_msg.header = Header(
                stamp=self.get_clock().now().to_msg(), frame_id="map"
            )

            # Copy control parameters
            ros_msg.coordinate_frame = mavlink_msg.coordinate_frame
            ros_msg.type_mask = mavlink_msg.type_mask

            # Convert position from NED to ENU coordinate frame
            # NED: North(x), East(y), Down(z) -> ENU: East(x), North(y), Up(z)
            ros_msg.position.x = float(mavlink_msg.y)   # East
            ros_msg.position.y = float(mavlink_msg.x)   # North
            ros_msg.position.z = -float(mavlink_msg.z)  # Up (negative down)

            # Convert velocity from NED to ENU coordinate frame
            ros_msg.velocity.x = float(mavlink_msg.vx)
            ros_msg.velocity.y = float(mavlink_msg.vy)
            ros_msg.velocity.z = float(mavlink_msg.vz)

            # Convert acceleration/force from NED to ENU coordinate frame
            ros_msg.acceleration_or_force.x = float(mavlink_msg.afx)
            ros_msg.acceleration_or_force.y = float(mavlink_msg.afy)
            ros_msg.acceleration_or_force.z = float(mavlink_msg.afz)

            # Copy yaw angle and yaw rate (rotation about vertical axis)
            ros_msg.yaw = float(mavlink_msg.yaw)
            ros_msg.yaw_rate = float(mavlink_msg.yaw_rate)

            # Publish converted message to ROS2 topic
            self.vector_pub.publish(ros_msg)

            # Debug logging for position targets
            self.get_logger().debug(
                f"Published POSITION_TARGET_LOCAL_NED: pos=[{ros_msg.position.x:.2f}, "
                f"{ros_msg.position.y:.2f}, {ros_msg.position.z:.2f}], "
                f"vel=[{ros_msg.velocity.x:.2f}, {ros_msg.velocity.y:.2f}, "
                f"{ros_msg.velocity.z:.2f}]"
            )

        except Exception as e:
            self.get_logger().error(f"Error processing position target message: {e}")


def main(args=None):
    """
    Execute the MAVLink bridge node.

    Parameters
    ----------
    args : list, optional
        Command line arguments

    """
    rclpy.init(args=args)

    try:
        node = MAVLinkGimbalBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if "node" in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
