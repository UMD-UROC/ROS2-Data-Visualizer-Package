import os
from glob import glob

from setuptools import find_packages, setup

package_name = "umd_uroc_data_visualizer"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
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
            "path_visualizer = umd_uroc_data_visualizer.path_visualizer:main",
            "velocity_vector_visualizer = umd_uroc_data_visualizer.velocity_vector_visualizer:main",
            "gimbal_frame = umd_uroc_data_visualizer.gimbal_frame:main",
            "gimbal_visualizer = umd_uroc_data_visualizer.gimbal_visualizer:main",
            "map_tf_publisher = umd_uroc_data_visualizer.map_tf_publisher:main",
            "mavlink_bridge = umd_uroc_data_visualizer.mavlink_bridge:main",
        ],
    },
)
