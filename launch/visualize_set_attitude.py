"""Unified launch file for UROC visualization and gimbal command bridge."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate unified launch description for all UROC nodes."""
    return LaunchDescription(
        [
            Node(
                package="py_uroc",
                executable="foxglove_3d_path_visualization",
                name="path_visualizer_node",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package="py_uroc",
                executable="foxglove_3d_gimbal_visualization_set_attitude",
                name="gimbal_visualizer_node",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package="py_uroc",
                executable="foxglove_3d_vector_visualization",
                name="vector_visualizer_node",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package="py_uroc",
                executable="mavlink_gimbal_bridge",
                name="mavlink_gimbal_bridge",
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
