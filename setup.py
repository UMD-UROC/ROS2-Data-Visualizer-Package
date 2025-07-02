from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'py_uroc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer='cdenihan',
    maintainer_email='cdenihan@proton.me',
    description='UMD UROC ROS2 Python Package',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'foxglove_3d_gimbal_visualization = py_uroc.foxglove_3d_gimbal_visualization:main',
            'foxglove_3d_path_visualization = py_uroc.foxglove_3d_path_visualization:main',
            'mavlink_gimbal_bridge = py_uroc.mavlink_gimbal_bridge:main',
        ],
    },
)
