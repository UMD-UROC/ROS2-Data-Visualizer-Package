"""
Launch file for UROC drone visualization system.

This launch file starts all the necessary nodes for comprehensive drone
visualization in tools like Foxglove, including:
- Transform frame publishers
- Path and velocity vector visualization
- Gimbal attitude visualization (both commands and status)
- MAVLink to ROS2 message bridging

The system provides real-time 3D visualization of drone flight data,
gimbal orientation, and velocity vectors for the MAVInsight project.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate the launch description for UROC visualization nodes.

    Returns
    -------
    LaunchDescription
        Complete launch configuration with all visualization nodes

    """
    # Get package configuration directory for parameter files
    config_dir = get_package_share_directory("umd_uroc_data_visualizer")
    set_attitude_config = config_dir + "/config/set_attitude.yaml"
    mavlink_bridge_config = config_dir + "/config/mavlink_bridge.yaml"

    return LaunchDescription(
        [
            # Transform frame publishers - establish coordinate system relationships
            Node(
                package="umd_uroc_data_visualizer",
                executable="map_tf_publisher",
                name="map_tf_publisher",
                output="screen",
            ),
            Node(
                package="umd_uroc_data_visualizer",
                executable="gimbal_frame",
                name="gimbal_frame",
                output="screen",
                parameters=[set_attitude_config],
            ),

            # Flight path visualization - creates trail showing drone movement
            Node(
                package="umd_uroc_data_visualizer",
                executable="path_visualizer",
                name="path_visualizer",
                output="screen",
            ),

            # Gimbal set attitude visualization (red arrows showing commanded gimbal state)
            Node(
                package="umd_uroc_data_visualizer",
                executable="gimbal_visualizer",
                name="gimbal_visualizer_set_attitude",
                output="screen",
                parameters=[set_attitude_config],
            ),

            # Velocity vector visualization (green arrows showing drone velocity direction)
            Node(
                package="umd_uroc_data_visualizer",
                executable="velocity_vector_visualizer",
                name="velocity_vector_visualizer",
                output="screen",
            ),

            # MAVLink to ROS2 bridge for external MAVLink message sources
            Node(
                package="umd_uroc_data_visualizer",
                executable="mavlink_bridge",
                name="mavlink_bridge",
                output="screen",
                parameters=[mavlink_bridge_config],
            ),

            # Range finder pointer visualization (red arrows showing range finder direction)
            Node(
                package="umd_uroc_data_visualizer",
                executable="rangefinder_pointer_visualizer",
                name="rangefinder_pointer_visualizer",
                output="screen",
            ),
        ]
    )
