from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'motor_control_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seoyoung',
    maintainer_email='seoyoung@todo.todo',
    description='iRoi ROS2 RS485 motor control with absolute-encoder startup homing',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_control_node = motor_control_pkg.motor_control_node:main',
            'scan_ids = motor_control_pkg.scan_ids:main',
            'check_home = motor_control_pkg.check_home:main',
            'arm_cli = motor_control_pkg.arm_cli:main',
            'arm_pose_cli = motor_control_pkg.arm_pose_cli:main',
            'arm_startup_pose = motor_control_pkg.arm_startup_pose:main',
            'probe_motors = motor_control_pkg.probe_motors:main',
        ],
    },
)
