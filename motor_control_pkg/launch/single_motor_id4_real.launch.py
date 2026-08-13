import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """MG5010E-i10 / ID 4 실물 1개 검증용 launch.

    주의: 이 launch는 mock이 아니며 시작 직후 저장된 0x94 절대 영점(3599.98 deg)으로
    자동 homing한다. 모터 주변을 비운 뒤 실행할 것.
    """
    package_share = get_package_share_directory('motor_control_pkg')
    zero_config = os.path.join(package_share, 'config', 'zero_config_i10_verified.json')

    node = Node(
        package='motor_control_pkg',
        executable='motor_control_node',
        name='motor_control',
        namespace='test_arm',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'baudrate': 115200,
            'arm_name': 'test_arm',
            'motor_ids': [4],
            'joint_names': ['joint1'],
            'polling_hz': 20.0,
            'mock_mode': False,
            'auto_home': True,
            'zero_config_path': zero_config,
            'homing_speed_dps': 30.0,
            'max_speed_dps': 60.0,
            'default_ratio': 10.0,
            'loop_period_deg': 3600.0,
        }],
    )

    return LaunchDescription([node])
