"""Start one physical iROI arm with reference-only sync and optional Pose 0."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool_argument(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in {'true', '1', 'yes', 'on'}:
        return True
    if value in {'false', '0', 'no', 'off'}:
        return False
    raise ValueError(f'{name}은 true/false여야 합니다: {value!r}')


def _launch_setup(context):
    arm = LaunchConfiguration('arm').perform(context).strip().lower()
    if arm == 'right':
        motor_ids = [1, 2, 3, 4]
    elif arm == 'left':
        motor_ids = [5, 6, 7, 8]
    else:
        raise ValueError("arm은 'right' 또는 'left'여야 합니다.")

    namespace = f'{arm}_arm'
    common_parameters = {
        'serial_port': LaunchConfiguration('serial_port').perform(context),
        'baudrate': 115200,
        'arm_name': namespace,
        'motor_ids': motor_ids,
        'joint_names': ['joint1', 'joint2', 'joint3', 'joint4'],
        'polling_hz': 20.0,
        'mock_mode': False,
        'auto_home': False,
        'startup_mode': 'reference_only',
        'zero_config_path': LaunchConfiguration('zero_config').perform(context),
        'max_speed_dps': 60.0,
        'teach_hold_speed_dps': 10.0,
    }

    actions = [
        Node(
            package='motor_control_pkg',
            executable='motor_control_node',
            name=f'{arm}_arm_motor_control',
            namespace=namespace,
            output='screen',
            parameters=[common_parameters],
        )
    ]

    if _bool_argument(context, 'start_pose'):
        actions.append(
            Node(
                package='motor_control_pkg',
                executable='arm_startup_pose',
                name=f'{arm}_arm_startup_pose',
                output='screen',
                parameters=[{
                    'mode': arm,
                    'pose_path': LaunchConfiguration('pose_path').perform(context),
                    'startup_pose_id': int(
                        LaunchConfiguration('startup_pose_id').perform(context)
                    ),
                    'startup_speed_dps': float(
                        LaunchConfiguration('startup_speed_dps').perform(context)
                    ),
                    'allow_partial_pose': _bool_argument(
                        context, 'allow_partial_pose'
                    ),
                }],
            )
        )

    return actions


def generate_launch_description():
    package_share = get_package_share_directory('motor_control_pkg')
    default_zero_config = os.path.join(
        package_share,
        'config',
        'zero_config_i10_verified.json',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm',
            default_value='right',
            description='Active physical arm: right (IDs 1-4) or left (IDs 5-8)',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='RS485 adapter for the selected arm',
        ),
        DeclareLaunchArgument('zero_config', default_value=default_zero_config),
        DeclareLaunchArgument('pose_path', default_value='~/.ros/arm_poses.json'),
        DeclareLaunchArgument('startup_pose_id', default_value='0'),
        DeclareLaunchArgument('startup_speed_dps', default_value='20.0'),
        DeclareLaunchArgument('allow_partial_pose', default_value='true'),
        DeclareLaunchArgument(
            'start_pose',
            default_value='true',
            description='Run startup Pose after reference sync and HOLD',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
