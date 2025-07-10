"""Unified launch file for UROC visualization and gimbal command bridge."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate unified launch description for all UROC nodes."""
    return LaunchDescription(
        [
            # TF bridge: map → base_link
            Node(
                package="py_uroc",
                executable="map_tf_publisher",
                name="map_tf_publisher",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package="py_uroc",
                executable="gimbal_frame",
                name="gimbal_frame",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            # Path visualization in global map frame
            Node(
                package="py_uroc",
                executable="foxglove_3d_path_visualization",
                name="path_visualizer_node",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            # Gimbal actual-status visualization (red arrow)
            Node(
                package="py_uroc",
                executable="foxglove_3d_gimbal_visualization_attitude_status",
                name="gimbal_visualizer_node_attitude_status",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            # Gimbal set-attitude visualization (red arrow)
            Node(
                package="py_uroc",
                executable="foxglove_3d_gimbal_visualization_set_attitude",
                name="gimbal_visualizer_node_set_attitude",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            # Target vector visualization (green arrow)
            Node(
                package="py_uroc",
                executable="foxglove_3d_velocity_vector_visualization",
                name="velocity_vector_visualizer_node",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            # MAVLink ↔ ROS2 bridge
            Node(
                package="py_uroc",
                executable="mavlink_bridge",
                name="mavlink_bridge",
                output="screen",
                parameters=[
                    {
                        "mavlink_connection": "udp:localhost:14445",
                        "system_id": 1,
                        "component_id": 1,
                    }
                ],
            ),
        ]
    )
