from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = get_package_share_directory("py_uroc")
    attitude_status_config = config_dir + "/config/attitude_status.yaml"
    set_attitude_config = config_dir + "/config/set_attitude.yaml"
    mavlink_bridge_config = config_dir + "/config/mavlink_bridge.yaml"

    return LaunchDescription(
        [
            # TF bridge: map → base_link
            Node(
                package="py_uroc",
                executable="map_tf_publisher",
                name="map_tf_publisher",
                output="screen",
            ),
            Node(
                package="py_uroc",
                executable="gimbal_frame",
                name="gimbal_frame",
                output="screen",
            ),
            Node(
                package="py_uroc",
                executable="path_visualizer",
                name="path_visualizer",
                output="screen",
            ),
            Node(
                package="py_uroc",
                executable="gimbal_visualizer",
                name="gimbal_visualizer_attitude_status",
                output="screen",
                parameters=[attitude_status_config],
            ),
            Node(
                package="py_uroc",
                executable="gimbal_visualizer",
                name="gimbal_visualizer_set_attitude",
                output="screen",
                parameters=[set_attitude_config],
            ),
            # Target vector visualization (green arrow)
            Node(
                package="py_uroc",
                executable="velocity_vector_visualizer",
                name="velocity_vector_visualizer",
                output="screen",
            ),
            # MAVLink ↔ ROS2 bridge
            Node(
                package="py_uroc",
                executable="mavlink_bridge",
                name="mavlink_bridge",
                output="screen",
                parameters=[mavlink_bridge_config],
            ),
        ]
    )
