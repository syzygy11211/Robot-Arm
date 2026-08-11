from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    left_arm_node = Node(
        package='motor_control_pkg',
        executable='motor_control_node',
        name='left_arm_motor_control',
        namespace='left_arm',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'arm_name': 'left_arm',
            'motor_ids': [1, 2, 3, 4],
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4'],
            'mock_mode': True,
        }],
    )

    right_arm_node = Node(
        package='motor_control_pkg',
        executable='motor_control_node',
        name='right_arm_motor_control',
        namespace='right_arm',
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            'arm_name': 'right_arm',
            'motor_ids': [5, 6, 7, 8],
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4'],
            'mock_mode': True,
        }],
    )

    return LaunchDescription([
        left_arm_node,
        right_arm_node,
    ])