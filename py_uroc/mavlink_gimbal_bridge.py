"""PyMAVLink to ROS2 bridge for GIMBAL_DEVICE_SET_ATTITUDE and POSITION_TARGET_LOCAL_NED messages."""

import threading
import rclpy
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, PoseStamped
from mavros_msgs.msg import GimbalDeviceSetAttitude, PositionTarget
from pymavlink import mavutil
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class MAVLinkGimbalBridge(Node):
    """Bridge that converts MAVLink messages to ROS2 messages."""

    def __init__(self):
        super().__init__('mavlink_gimbal_bridge')

        # Declare parameters
        self.declare_parameter('mavlink_connection', 'udp:localhost:14445')
        self.declare_parameter('system_id', 1)
        self.declare_parameter('component_id', 1)

        # Get parameters
        self.mavlink_connection = self.get_parameter('mavlink_connection').value
        self.system_id = self.get_parameter('system_id').value
        self.component_id = self.get_parameter('component_id').value

        # QoS matching MAVROS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ROS2 publishers
        self.gimbal_pub = self.create_publisher(
            GimbalDeviceSetAttitude,
            '/mavros/gimbal_control/device/set_attitude',
            qos
        )
        self.vector_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            qos
        )

        # Initialize MAVLink connection
        try:
            self.mavlink_connection_obj = mavutil.mavlink_connection(
                self.mavlink_connection,
                source_system=self.system_id,
                source_component=self.component_id
            )
            self.get_logger().info(f'Connected to MAVLink on {self.mavlink_connection}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MAVLink: {e}')
            return

        # Start MAVLink listener thread
        self.mavlink_thread = threading.Thread(
            target=self.mavlink_listener,
            daemon=True
        )
        self.mavlink_thread.start()

        self.get_logger().info('MAVLink Bridge started')

    def mavlink_listener(self):  # noqa: C901
        """Listen for MAVLink messages in a separate thread and dispatch to handlers."""
        while rclpy.ok():
            try:
                # Wait for any MAVLink message
                msg = self.mavlink_connection_obj.recv_match(
                    blocking=True,
                    timeout=1.0
                )
                if msg is None:
                    continue

                mtype = msg.get_type()

                if mtype == 'GIMBAL_DEVICE_SET_ATTITUDE':
                    self.process_gimbal_message(msg)
                elif mtype == 'POSITION_TARGET_LOCAL_NED':
                    self.process_position_target(msg)

            except Exception as e:
                self.get_logger().error(f'Error receiving or processing MAVLink message: {e}')
                continue

    def process_gimbal_message(self, mavlink_msg):
        """Convert MAVLink GIMBAL_DEVICE_SET_ATTITUDE to ROS2 message."""
        try:
            ros_msg = GimbalDeviceSetAttitude()

            ros_msg.target_system = mavlink_msg.target_system
            ros_msg.target_component = mavlink_msg.target_component
            ros_msg.flags = mavlink_msg.flags

            # Convert quaternion: MAVLink [w, x, y, z] -> ROS [x, y, z, w]
            ros_msg.q = Quaternion(
                x=float(mavlink_msg.q[1]),
                y=-float(mavlink_msg.q[2]),
                z=-float(mavlink_msg.q[3]),
                w=float(mavlink_msg.q[0])
            )

            # Angular velocities
            ros_msg.angular_velocity_x = float(mavlink_msg.angular_velocity_x)
            ros_msg.angular_velocity_y = float(mavlink_msg.angular_velocity_y)
            ros_msg.angular_velocity_z = float(mavlink_msg.angular_velocity_z)

            # Publish ROS2 message
            self.gimbal_pub.publish(ros_msg)

            self.get_logger().debug(
                f'Published gimbal attitude: q=[{ros_msg.q.x:.3f}, {ros_msg.q.y:.3f}, '
                f'{ros_msg.q.z:.3f}, {ros_msg.q.w:.3f}]'
            )

        except Exception as e:
            self.get_logger().error(f'Error processing gimbal message: {e}')

    def process_position_target(self, mavlink_msg):
        """Convert MAVLink POSITION_TARGET_LOCAL_NED to ROS2 PositionTarget message."""
        try:
            ros_msg = PositionTarget()

            # Header
            ros_msg.header = Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id='map'
            )

            # Copy coordinate frame and type mask
            ros_msg.coordinate_frame = mavlink_msg.coordinate_frame
            ros_msg.type_mask = mavlink_msg.type_mask

            # Position
            ros_msg.position.x = float(mavlink_msg.y)
            ros_msg.position.y = float(mavlink_msg.x)
            ros_msg.position.z = -float(mavlink_msg.z)

            # Velocity
            ros_msg.velocity.x = float(mavlink_msg.vx)
            ros_msg.velocity.y = float(mavlink_msg.vy)
            ros_msg.velocity.z = float(mavlink_msg.vz)

            # Acceleration or force
            ros_msg.acceleration_or_force.x = float(mavlink_msg.afx)
            ros_msg.acceleration_or_force.y = float(mavlink_msg.afy)
            ros_msg.acceleration_or_force.z = float(mavlink_msg.afz)

            # Yaw and yaw rate
            ros_msg.yaw = float(mavlink_msg.yaw)
            ros_msg.yaw_rate = float(mavlink_msg.yaw_rate)

            # Publish ROS2 message
            self.vector_pub.publish(ros_msg)

            self.get_logger().debug(
                f'Published POSITION_TARGET_LOCAL_NED: pos=[{ros_msg.position.x:.2f}, '
                f'{ros_msg.position.y:.2f}, {ros_msg.position.z:.2f}], '
                f'vel=[{ros_msg.velocity.x:.2f}, {ros_msg.velocity.y:.2f}, {ros_msg.velocity.z:.2f}]'
            )

        except Exception as e:
            self.get_logger().error(f'Error processing position target message: {e}')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = MAVLinkGimbalBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
