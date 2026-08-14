from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """8-axis pose-framework architecture test in mock mode.

    IDs:
      left  = 1,2,3,4
      right = 5,6,7,8

    This is intentionally mock-only until every final motor has its own verified
    encoder calibration (zero_single_deg / ratio / loop_period_deg).
    """

    common = {
        'polling_hz': 20.0,
        'mock_mode': True,
        'auto_home': False,
        'startup_mode': 'reference_only',
        'max_speed_dps': 60.0,
        'teach_hold_speed_dps': 10.0,
    }

    left_arm_node = Node(
        package='motor_control_pkg',
        executable='motor_control_node',
        name='left_arm_motor_control',
        namespace='left_arm',
        output='screen',
        parameters=[{
            **common,
            'serial_port': '/dev/ttyUSB0',
            'arm_name': 'left_arm',
            'motor_ids': [1, 2, 3, 4],
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4'],
        }],
    )

    right_arm_node = Node(
        package='motor_control_pkg',
        executable='motor_control_node',
        name='right_arm_motor_control',
        namespace='right_arm',
        output='screen',
        parameters=[{
            **common,
            'serial_port': '/dev/ttyUSB1',
            'arm_name': 'right_arm',
            'motor_ids': [5, 6, 7, 8],
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4'],
        }],
    )

    startup_pose = Node(
        package='motor_control_pkg',
        executable='arm_startup_pose',
        name='arm_startup_pose',
        output='screen',
        parameters=[{
            'mode': 'dual',
            'pose_path': '~/.ros/arm_poses.json',
            'startup_pose_id': 0,
            'startup_speed_dps': 20.0,
            # Development/mock phase only. Final hardware: False.
            'allow_partial_pose': True,
        }],
    )

    return LaunchDescription([
        left_arm_node,
        right_arm_node,
        startup_pose,
    ])
