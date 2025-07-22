"""
Setup configuration for UROC ROS2 Python Package.

This package provides drone visualization capabilities for the University of
Maryland Robotics Club (UROC) as part of the MAVInsight project. It includes
nodes for real-time visualization of drone flight paths, velocity vectors,
gimbal orientation, and coordinate frame transforms.
"""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "umd_uroc_data_visualizer"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Package index registration
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # Package metadata
        ("share/" + package_name, ["package.xml"]),
        # Launch files for starting visualization nodes
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        # Configuration files for node parameters
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # Environment configuration
        ("share/" + package_name, [".env"]),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer="cdenihan",
    maintainer_email="cdenihan@proton.me",
    description="The University of Maryland's UROC lab's live drone data visualizer node.",
    license="MIT",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            # Flight path and pose visualization
            "path_visualizer = umd_uroc_data_visualizer.path_visualizer:main",
            # Velocity vector visualization
            "velocity_vector_visualizer = umd_uroc_data_visualizer.velocity_vector_visualizer:main",
            # Coordinate frame publishers
            "gimbal_frame = umd_uroc_data_visualizer.gimbal_frame:main",
            "map_tf_publisher = umd_uroc_data_visualizer.map_tf_publisher:main",
            # Gimbal attitude visualization
            "gimbal_visualizer = umd_uroc_data_visualizer.gimbal_visualizer:main",
            # MAVLink message bridge
            "mavlink_bridge = umd_uroc_data_visualizer.mavlink_bridge:main",
        ],
    },
)
