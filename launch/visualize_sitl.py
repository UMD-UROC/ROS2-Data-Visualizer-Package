# file: visualize_gimbal.py
"""
Unified launch file for UROC drone visualization.
Select which gimbal pipeline to visualize via: gimbal_topic:=attitude_status | set_attitude
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    gimbal_topic = LaunchConfiguration("gimbal_topic")

    config_dir = get_package_share_directory("umd_uroc_data_visualizer")
    attitude_status_config = config_dir + "/config/sitl/attitude_status.yaml"
    set_attitude_config = config_dir + "/config/sitl/set_attitude.yaml"
    mavlink_bridge_config = config_dir + "/config/sitl/mavlink_bridge.yaml"
    map_tf_publisher_config = config_dir + "/config/sitl/map_tf_publisher.yaml"

    is_status = IfCondition(PythonExpression(["'", gimbal_topic, "' == 'attitude_status'"]))
    is_set    = IfCondition(PythonExpression(["'", gimbal_topic, "' == 'set_attitude'"]))

    return LaunchDescription([
        # Which pipeline to use: attitude_status (actual) vs set_attitude (commanded)
        DeclareLaunchArgument(
            "gimbal_topic",
            default_value="attitude_status",
            description="Select gimbal visualization topic: 'attitude_status' or 'set_attitude'"
        ),

        # Shared nodes ---------------------------------------------------------
        Node(
            package="umd_uroc_data_visualizer",
            executable="map_tf_publisher",
            name="map_tf_publisher",
            output="screen",
            parameters=[map_tf_publisher_config],
        ),
        Node(
            package="umd_uroc_data_visualizer",
            executable="path_visualizer",
            name="path_visualizer",
            output="screen",
        ),
        Node(
            package="umd_uroc_data_visualizer",
            executable="velocity_vector_visualizer",
            name="velocity_vector_visualizer",
            output="screen",
        ),
        Node(
            package="umd_uroc_data_visualizer",
            executable="mavlink_bridge",
            name="mavlink_bridge",
            output="screen",
            parameters=[mavlink_bridge_config],
        ),
        Node(
            package="umd_uroc_data_visualizer",
            executable="rangefinder_pointer_visualizer",
            name="rangefinder_pointer_visualizer",
            output="screen",
        ),

        # Conditional: attitude_status branch ---------------------------------
        Node(
            package="umd_uroc_data_visualizer",
            executable="gimbal_frame",
            name="gimbal_frame",
            output="screen",
            parameters=[attitude_status_config],
            condition=is_status,
        ),
        Node(
            package="umd_uroc_data_visualizer",
            executable="gimbal_visualizer",
            name="gimbal_visualizer_attitude_status",
            output="screen",
            parameters=[attitude_status_config],
            condition=is_status,
        ),

        # Conditional: set_attitude branch ------------------------------------
        Node(
            package="umd_uroc_data_visualizer",
            executable="gimbal_frame",
            name="gimbal_frame",
            output="screen",
            parameters=[set_attitude_config],
            condition=is_set,
        ),
        Node(
            package="umd_uroc_data_visualizer",
            executable="gimbal_visualizer",
            name="gimbal_visualizer_set_attitude",
            output="screen",
            parameters=[set_attitude_config],
            condition=is_set,
        ),
    ])
