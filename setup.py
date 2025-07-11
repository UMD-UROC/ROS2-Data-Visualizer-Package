import os
from glob import glob

from setuptools import find_packages , setup

package_name = "py_uroc"

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
    description="UMD UROC ROS2 Python Package",
    license="MIT",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "path_visualizer = py_uroc.path_visualizer:main",
            "velocity_vector_visualizer = py_uroc.velocity_vector_visualizer:main",
            "gimbal_frame = py_uroc.gimbal_frame:main",
            "gimbal_visualizer = py_uroc.gimbal_visualizer:main",
            "map_tf_publisher = py_uroc.map_tf_publisher:main",
            "mavlink_bridge = py_uroc.mavlink_bridge:main",
        ],
    },
)
