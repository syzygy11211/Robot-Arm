"""Start both physical iROI arms with reference-only sync and optional Pose 0."""

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


def _motor_node(arm, serial_port, motor_ids, zero_config):
    namespace = f'{arm}_arm'
    return Node(
        package='motor_control_pkg',
        executable='motor_control_node',
        name=f'{arm}_arm_motor_control',
        namespace=namespace,
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baudrate': 115200,
            'arm_name': namespace,
            'motor_ids': motor_ids,
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4'],
            'polling_hz': 20.0,
            'mock_mode': False,
            'auto_home': False,
            'startup_mode': 'reference_only',
            'zero_config_path': zero_config,
            'max_speed_dps': 60.0,
            'teach_hold_speed_dps': 10.0,
        }],
    )


def _launch_setup(context):
    zero_config = LaunchConfiguration('zero_config').perform(context)
    actions = [
        _motor_node(
            'right',
            LaunchConfiguration('right_port').perform(context),
            [1, 2, 3, 4],
            zero_config,
        ),
        _motor_node(
            'left',
            LaunchConfiguration('left_port').perform(context),
            [5, 6, 7, 8],
            zero_config,
        ),
    ]

    if _bool_argument(context, 'start_pose'):
        actions.append(
            Node(
                package='motor_control_pkg',
                executable='arm_startup_pose',
                name='dual_arm_startup_pose',
                output='screen',
                parameters=[{
                    'mode': 'dual',
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
            'right_port',
            default_value='/dev/ttyUSB0',
            description='RS485 adapter for right arm IDs 1-4',
        ),
        DeclareLaunchArgument(
            'left_port',
            default_value='/dev/ttyUSB1',
            description='RS485 adapter for left arm IDs 5-8',
        ),
        DeclareLaunchArgument('zero_config', default_value=default_zero_config),
        DeclareLaunchArgument('pose_path', default_value='~/.ros/arm_poses.json'),
        DeclareLaunchArgument('startup_pose_id', default_value='0'),
        DeclareLaunchArgument('startup_speed_dps', default_value='20.0'),
        DeclareLaunchArgument('allow_partial_pose', default_value='true'),
        DeclareLaunchArgument(
            'start_pose',
            default_value='true',
            description='Run startup Pose after both arms finish reference sync and HOLD',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
