import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """MG4010E-i10 ID 1, 2 + MG5010E-i10 ID 4 실물 3축 테스트용 launch.

    시작 직후 zero_config_i10_verified.json에 저장된 각 모터의
    0x94 절대 영점으로 자동 homing한다.

    현재 테스트 모터는 모두 i10이므로:
    - 감속비: 10:1
    - loop period: 3600 deg

    주의:
    auto_home=True이므로 launch 즉시 모터가 움직일 수 있다.
    """

    package_share = get_package_share_directory('motor_control_pkg')

    zero_config = os.path.join(
        package_share,
        'config',
        'zero_config_i10_verified.json'
    )

    node = Node(
        package='motor_control_pkg',
        executable='motor_control_node',
        name='motor_control',
        namespace='test_arm',
        output='screen',

        parameters=[{
            # -----------------------------
            # RS485 통신
            # -----------------------------
            'serial_port': '/dev/ttyUSB0',
            'baudrate': 115200,

            # -----------------------------
            # Arm 설정
            # -----------------------------
            'arm_name': 'test_arm',

            # 순서 중요:
            # target_angles도 이 순서와 대응됨
            'motor_ids': [1, 2, 4],

            'joint_names': [
                'joint1',   # ID 1
                'joint2',   # ID 2
                'joint3',   # ID 4
            ],

            # -----------------------------
            # 상태 읽기
            # -----------------------------
            'polling_hz': 20.0,

            # 실제 모터 사용
            'mock_mode': False,

            # 시작 시 저장된 절대 영점으로 자동 복귀
            'auto_home': True,

            # Python 실물 테스트 당시 검증한 영점값
            'zero_config_path': zero_config,

            # -----------------------------
            # 속도
            # -----------------------------
            'homing_speed_dps': 30.0,
            'max_speed_dps': 60.0,

            # -----------------------------
            # 현재 테스트 모터는 전부 i10
            # -----------------------------
            'default_ratio': 10.0,
            'loop_period_deg': 3600.0,
        }],
    )

    return LaunchDescription([node])
