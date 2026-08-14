import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Development launch for pose/reference framework on verified IDs 1,2,4.

    Difference from three_motor_real.launch.py:
    - does NOT physically home to encoder zero on startup
    - reconstructs the calibrated coordinate reference only
    - then arm_startup_pose attempts pose 0
    - current poses.json is all null, so no physical startup movement occurs yet

    Keep three_motor_real.launch.py unchanged as the hardware-verified fallback.
    """
    package_share = get_package_share_directory('motor_control_pkg')
    zero_config = os.path.join(
        package_share,
        'config',
        'zero_config_i10_verified.json',
    )

    motor_node = Node(
        package='motor_control_pkg',
        executable='motor_control_node',
        name='motor_control',
        namespace='test_arm',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'baudrate': 115200,
            'arm_name': 'test_arm',
            'motor_ids': [1, 2, 4],
            'joint_names': ['joint1', 'joint2', 'joint3'],
            'polling_hz': 20.0,
            'mock_mode': False,
            'auto_home': False,
            'startup_mode': 'reference_only',
            'zero_config_path': zero_config,
            'homing_speed_dps': 30.0,
            'max_speed_dps': 60.0,
            'default_ratio': 10.0,
            'loop_period_deg': 3600.0,
            'teach_hold_speed_dps': 10.0,
        }],
    )

    startup_pose = Node(
        package='motor_control_pkg',
        executable='arm_startup_pose',
        name='arm_startup_pose',
        output='screen',
        parameters=[{
            'mode': 'test',
            'pose_path': '~/.ros/arm_poses.json',
            'startup_pose_id': 0,
            'startup_speed_dps': 20.0,
            # Development only. Final 8-axis launch must switch this to False.
            'allow_partial_pose': True,
        }],
    )

    return LaunchDescription([motor_node, startup_pose])
