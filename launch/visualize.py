"""Launch file for UROC visualization nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for visualization nodes."""
    return LaunchDescription([
        Node(
            package='py_uroc',
            executable='foxglove_3d_path_visualization',
            name='path_visualizer_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        Node(
            package='py_uroc',
            executable='foxglove_3d_gimbal_visualization',
            name='gimbal_visualizer_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        )
    ])
