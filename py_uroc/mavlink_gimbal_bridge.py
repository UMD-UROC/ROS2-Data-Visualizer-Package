"""PyMAVLink to ROS2 bridge for GIMBAL_DEVICE_SET_ATTITUDE messages."""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import GimbalDeviceSetAttitude
from geometry_msgs.msg import Quaternion
from pymavlink import mavutil
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import threading


class MAVLinkGimbalBridge(Node):
    """Bridge that converts MAVLink GIMBAL_DEVICE_SET_ATTITUDE to ROS2 messages."""

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

        # ROS2 publisher
        self.gimbal_pub = self.create_publisher(
            GimbalDeviceSetAttitude,
            '/mavros/gimbal_control/device/set_attitude',
            qos
        )

        # Initialize MAVLink connection
        try:
            self.mavlink_connection_obj = mavutil.mavlink_connection(self.mavlink_connection)
            self.get_logger().info(f'Connected to MAVLink on {self.mavlink_connection}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MAVLink: {e}')
            return

        # Start MAVLink listener thread
        self.mavlink_thread = threading.Thread(target=self.mavlink_listener, daemon=True)
        self.mavlink_thread.start()

        self.get_logger().info('MAVLink Gimbal Bridge started')

    def mavlink_listener(self):
        """Listen for MAVLink messages in a separate thread."""
        while rclpy.ok():
            try:
                # Wait for MAVLink message
                msg = self.mavlink_connection_obj.recv_match(
                    type='GIMBAL_DEVICE_SET_ATTITUDE',
                    blocking=True,
                    timeout=1.0
                )

                if msg is not None:
                    self.process_gimbal_message(msg)

            except Exception as e:
                self.get_logger().error(f'Error receiving MAVLink message: {e}')
                continue

    def process_gimbal_message(self, mavlink_msg):
        """Convert MAVLink GIMBAL_DEVICE_SET_ATTITUDE to ROS2 message."""
        try:
            # Create ROS2 message
            ros_msg = GimbalDeviceSetAttitude()

            # Fill in the fields from MAVLink message
            ros_msg.target_system = mavlink_msg.target_system
            ros_msg.target_component = mavlink_msg.target_component
            ros_msg.flags = mavlink_msg.flags

            # Convert quaternion
            ros_msg.q = Quaternion()
            # MAVLink quaternion is array [w,x,y,z], ROS quaternion is [x,y,z,w]
            # WORKING
            ros_msg.q.x = float(mavlink_msg.q[1])  # x component
            ros_msg.q.y = -float(mavlink_msg.q[2])  # y component
            ros_msg.q.z = -float(mavlink_msg.q[3])  # z component
            ros_msg.q.w = float(mavlink_msg.q[0])  # w component (scalar)

            #ros_msg.q.x = float(mavlink_msg.q[1])  # Wrong: negative values
            #ros_msg.q.y = -float(mavlink_msg.q[2])  # Wrong: negative values
            #ros_msg.q.z = float(mavlink_msg.q[3])
            #ros_msg.q.w = float(mavlink_msg.q[0])

            # Convert angular velocities
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
