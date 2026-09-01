#!/usr/bin/env python3
"""ROS 2 motor-control node for the iRoi robot arm.

핵심 좌표계
-----------
* 0x94 / read_single_angle(): 전원 재인가 후에도 유지되는 절대 엔코더 기준값.
  저장된 ``zero_single_deg`` 와 비교해서 현재 0x92 좌표의 기준을 복원하는 데 사용한다.
* 0x92 / read_multi_angle(): 현재 전원 세션의 다회전 좌표계.
  Homing이 끝난 뒤 계산한 ``zero_92`` 를 기준으로 위치 추적과 모든 이동 명령을 처리한다.
* 0xA4 / move_to_frame_angle(): 0x92와 같은 좌표계의 목표를 받는다.

따라서 real mode에서는 기본적으로 시작 위치를 유지한 채 reference를 복원한 뒤,
    output_angle = (current_92 - zero_92) / ratio
로 출력축 각도를 계산한다.

이 구조는 기존 Python 실물 테스트의 ``MotorRuntime.start_homing()`` / ``current_output_angle()`` /
``apply_synced_targets()`` 동작을 ROS 2 노드로 이관한 것이다.
"""

import json
import math
import os
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from iroi_interfaces.action import MoveJoint


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # ------------------------------------------------------------------
        # ROS parameter
        # ------------------------------------------------------------------
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('arm_name', 'right_arm')
        self.declare_parameter('motor_ids', [1, 2, 3, 4])
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4'])
        self.declare_parameter('polling_hz', 30.0)

        # config에 모터별 max_speed_dps가 없을 때 쓰는 출력축 기준 기본 상한.
        self.declare_parameter('max_speed_dps', 60.0)

        # mock_mode=True에서는 serial을 열지 않고 homing도 실제로 수행하지 않는다.
        self.declare_parameter('mock_mode', False)

        # 기존 검증된 zero_config.json 형식의 파일 경로.
        # real + auto_home에서는 이 파일이 반드시 필요하다.
        self.declare_parameter(
            'zero_config_path',
            '/home/young/iroi_ws/src/motor_control_pkg/config/zero_config_i10_verified.json'
        )

        # real mode 시작 시 저장된 0x94 절대 영점으로 자동 복귀할지 여부.
        self.declare_parameter('auto_home', True)

        # pose-framework: reference_only startup
        # ''=legacy auto_home, home=physical zero move, reference_only=no-motion sync, disabled=none
        self.declare_parameter('startup_mode', 'reference_only')
        self.declare_parameter('teach_hold_speed_dps', 10.0)

        # Homing 속도는 출력축 기준 deg/s. 실제 0xA4에는 ratio를 곱한 모터축 속도로 보낸다.
        self.declare_parameter('homing_speed_dps', 30.0)
        self.declare_parameter('homing_timeout_sec', 20.0)
        self.declare_parameter('homing_target_tolerance_deg', 0.2)
        self.declare_parameter('homing_settle_count', 3)

        # MoveJoint 완료 판정: 모든 이동 대상 모터가 이 오차 안에 연속 N회 들어와야 성공.
        self.declare_parameter('move_target_tolerance_deg', 0.2)
        self.declare_parameter('move_settle_count', 3)

        # config 항목에 값이 없을 때의 fallback.
        # 현재 i10 실물 테스트에서는 ratio=10, loop_period=3600이 검증값이다.
        self.declare_parameter('default_ratio', 10.0)
        self.declare_parameter('loop_period_deg', 3600.0)

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.arm_name = self.get_parameter('arm_name').value
        self.motor_ids = [int(v) for v in self.get_parameter('motor_ids').value]
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.polling_hz = float(self.get_parameter('polling_hz').value)
        self.default_max_speed_dps = float(self.get_parameter('max_speed_dps').value)
        self.mock_mode = bool(self.get_parameter('mock_mode').value)
        self.auto_home = bool(self.get_parameter('auto_home').value)

        startup_mode = str(self.get_parameter('startup_mode').value).strip().lower()
        if not startup_mode:
            startup_mode = 'home' if self.auto_home else 'disabled'
        if startup_mode not in {'home', 'reference_only', 'disabled'}:
            raise ValueError('startup_mode은 home/reference_only/disabled 중 하나여야 합니다.')
        self.startup_mode = startup_mode
        self.teach_hold_speed_dps = float(self.get_parameter('teach_hold_speed_dps').value)
        self.homing_speed_dps = float(self.get_parameter('homing_speed_dps').value)
        self.homing_timeout_sec = float(self.get_parameter('homing_timeout_sec').value)
        self.homing_target_tolerance_deg = float(
            self.get_parameter('homing_target_tolerance_deg').value
        )
        self.homing_settle_count = int(self.get_parameter('homing_settle_count').value)
        self.move_target_tolerance_deg = float(
            self.get_parameter('move_target_tolerance_deg').value
        )
        self.move_settle_count = int(self.get_parameter('move_settle_count').value)
        self.default_ratio = float(self.get_parameter('default_ratio').value)
        self.default_loop_period_deg = float(self.get_parameter('loop_period_deg').value)

        if self.homing_target_tolerance_deg <= 0.0:
            raise ValueError('homing_target_tolerance_deg는 0보다 커야 합니다.')
        if self.homing_settle_count < 1:
            raise ValueError('homing_settle_count는 1 이상이어야 합니다.')
        if self.move_target_tolerance_deg <= 0.0:
            raise ValueError('move_target_tolerance_deg는 0보다 커야 합니다.')
        if self.move_settle_count < 1:
            raise ValueError('move_settle_count는 1 이상이어야 합니다.')

        if len(self.joint_names) != len(self.motor_ids):
            raise ValueError(
                f'joint_names({len(self.joint_names)})와 motor_ids({len(self.motor_ids)}) 개수가 다릅니다.'
            )

        zero_config_path = self.get_parameter('zero_config_path').value
        if zero_config_path:
            self.zero_config_path = os.path.abspath(os.path.expanduser(zero_config_path))
        else:
            self.zero_config_path = os.path.expanduser('~/.ros/iroi_zero_config.json')

        self.get_logger().info(
            f'[{self.arm_name}] 시작: port={self.serial_port}, motor_ids={self.motor_ids}, '
            f'mock_mode={self.mock_mode}, auto_home={self.auto_home}, '
            f'zero_config_path={self.zero_config_path}'
        )

        # RS485는 반이중이므로 한 버스의 모든 read/write를 직렬화한다.
        self.serial_lock = threading.Lock()

        # Homing, set_zero, MoveJoint 액션이 서로 겹치지 않도록 막는다.
        self.motion_lock = threading.RLock()

        self.motors = {}
        self.ser = None

        # {motor_id: config entry}
        self.motor_cfg = self._load_motor_config()

        # Homing이 끝난 뒤 이번 전원 세션에서 사용하는 0x92 기준 영점.
        # 절대 영점 자체는 zero_single_deg로 파일에 영구 저장되고,
        # zero_92는 전원을 켤 때마다 그 절대 영점을 현재 0x92 frame에 매핑한 임시 기준값이다.
        self.zero_92 = {mid: None for mid in self.motor_ids}
        self.homed = False

        # pose-framework runtime gates
        self.torque_enabled = False
        self.teach_mode = False

        # 읽기 실패 시 마지막 정상 위치를 잠깐 유지하기 위한 cache (출력축 deg).
        self._last_good_output_angle = {}

        # mock mode는 출력축 각도를 직접 저장한다.
        self.mock_angles = {mid: 0.0 for mid in self.motor_ids}

        if self.mock_mode:
            if self.startup_mode == 'disabled':
                self.homed = False
                self.torque_enabled = False
                self.get_logger().warn(
                    f'[{self.arm_name}] mock startup_mode=disabled: 이동 명령을 거부합니다.'
                )
            else:
                self.homed = True
                self.torque_enabled = True
                for mid in self.motor_ids:
                    self.zero_92[mid] = 0.0
                self.get_logger().info(
                    f'[{self.arm_name}] mock_mode=True, 실제 serial 포트를 열지 않습니다.'
                )
        else:
            self._open_real_bus_and_motors()
            if self.startup_mode == 'home':
                self._home_all_motors()
            elif self.startup_mode == 'reference_only':
                self._sync_zero_reference()
            else:
                self._torque_off_all_real_motors()
                self.get_logger().warn(
                    f'[{self.arm_name}] startup_mode=disabled: 전체 torque OFF, '
                    f'좌표 reference가 아직 없습니다. '
                    f'/sync_reference 또는 /home 실행 전에는 이동 명령을 거부합니다.'
                )
        # ------------------------------------------------------------------
        # ROS interface
        # ------------------------------------------------------------------
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        timer_period = 1.0 / max(self.polling_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.polling_callback)

        self.torque_srv = self.create_service(SetBool, 'torque', self.torque_callback)

        # pose-framework hand-guiding and no-motion reference sync
        self.teach_srv = self.create_service(SetBool, 'teach', self.teach_callback)
        self.sync_reference_srv = self.create_service(Trigger, 'sync_reference', self.sync_reference_callback)

        # /set_zero: 현재 물리 위치의 0x94 값을 새로운 절대 영점으로 파일에 저장.
        self.set_zero_srv = self.create_service(Trigger, 'set_zero', self.set_zero_callback)

        # /home: 저장된 절대 영점으로 다시 자동 복귀.
        self.home_srv = self.create_service(Trigger, 'home', self.home_callback)

        self._action_callback_group = ReentrantCallbackGroup()
        self.move_action_server = ActionServer(
            self,
            MoveJoint,
            'move_to',
            execute_callback=self.execute_move_callback,
            cancel_callback=self.cancel_move_callback,
            callback_group=self._action_callback_group,
        )

    # ==================================================================
    # Config
    # ==================================================================

    def _load_motor_config(self):
        """기존 Python 실물 테스트의 zero_config.json 형식을 읽는다.

        기대 형식 예:
        {
          "port": "/dev/ttyUSB0",
          "motors": [
            {
              "name": "motor1",
              "motor_id": 4,
              "ratio": 10.0,
              "max_speed_dps": 60.0,
              "zero_single_deg": 3599.98,
              "loop_period_deg": 3600.0
            }
          ]
        }

        mock mode에서는 파일이 없어도 fallback config로 동작한다.
        real mode에서는 requested motor_id가 파일에 없으면 안전을 위해 시작을 중단한다.
        """
        if not os.path.exists(self.zero_config_path):
            if self.mock_mode:
                self.get_logger().warn(
                    f'[{self.arm_name}] config 없음({self.zero_config_path}); mock fallback을 사용합니다.'
                )
                return {
                    mid: {
                        'name': f'motor{mid}',
                        'motor_id': mid,
                        'ratio': self.default_ratio,
                        'max_speed_dps': self.default_max_speed_dps,
                        'zero_single_deg': 0.0,
                        'loop_period_deg': self.default_loop_period_deg,
                    }
                    for mid in self.motor_ids
                }
            raise RuntimeError(
                f'실물 모드인데 영점 config가 없습니다: {self.zero_config_path}. '
                f'검증된 zero_config.json을 지정해야 합니다.'
            )

        try:
            with open(self.zero_config_path, encoding='utf-8') as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError) as e:
            raise RuntimeError(f'영점 config 읽기 실패: {self.zero_config_path}: {e}') from e

        entries = data.get('motors')
        if not isinstance(entries, list):
            raise RuntimeError(
                f'{self.zero_config_path}에 "motors" 리스트가 없습니다. '
                f'기존 zero_config.json 형식이 필요합니다.'
            )

        by_id = {}
        for raw in entries:
            if not isinstance(raw, dict) or 'motor_id' not in raw:
                continue
            mid = int(raw['motor_id'])
            entry = dict(raw)
            entry['motor_id'] = mid
            entry['name'] = entry.get('name', f'motor{mid}')
            entry['ratio'] = float(entry.get('ratio', self.default_ratio))
            entry['max_speed_dps'] = float(entry.get('max_speed_dps', self.default_max_speed_dps))
            entry['loop_period_deg'] = float(
                entry.get('loop_period_deg', self.default_loop_period_deg)
            )
            by_id[mid] = entry

        selected = {}
        for mid in self.motor_ids:
            if mid not in by_id:
                if self.mock_mode:
                    selected[mid] = {
                        'name': f'motor{mid}',
                        'motor_id': mid,
                        'ratio': self.default_ratio,
                        'max_speed_dps': self.default_max_speed_dps,
                        'zero_single_deg': 0.0,
                        'loop_period_deg': self.default_loop_period_deg,
                    }
                    continue
                raise RuntimeError(
                    f'영점 config에 motor_id={mid} 항목이 없습니다: {self.zero_config_path}'
                )

            entry = by_id[mid]
            if (
                not self.mock_mode
                and entry.get('zero_single_deg') is None
                and self.startup_mode != 'disabled'
            ):
                raise RuntimeError(
                    f"motor_id={mid} ({entry['name']})의 zero_single_deg가 없습니다. "
                    f'물리 영점을 먼저 저장하거나 startup_mode:=disabled로 시작하세요.'
                )
            selected[mid] = entry

        self.get_logger().info(
            f'[{self.arm_name}] motor config 로드 완료: '
            + ', '.join(
                f"ID {mid}: ratio={selected[mid]['ratio']}, "
                f"zero94={selected[mid].get('zero_single_deg')}, "
                f"period={selected[mid]['loop_period_deg']}"
                for mid in self.motor_ids
            )
        )
        return selected

    def _save_motor_config(self):
        """현재 motor_cfg의 절대 영점 정보를 기존 zero_config.json 형식으로 저장한다.

        파일에 현재 노드가 사용하지 않는 다른 모터 항목이 있으면 그대로 보존한다.
        """
        os.makedirs(os.path.dirname(self.zero_config_path), exist_ok=True)

        existing = {'port': self.serial_port, 'motors': []}
        if os.path.exists(self.zero_config_path):
            try:
                with open(self.zero_config_path, encoding='utf-8') as f:
                    loaded = json.load(f)
                if isinstance(loaded, dict):
                    existing = loaded
            except (OSError, json.JSONDecodeError):
                pass

        existing['port'] = existing.get('port') or self.serial_port
        motors = existing.get('motors')
        if not isinstance(motors, list):
            motors = []

        index_by_id = {}
        for idx, entry in enumerate(motors):
            if isinstance(entry, dict) and 'motor_id' in entry:
                index_by_id[int(entry['motor_id'])] = idx

        for mid in self.motor_ids:
            entry = dict(self.motor_cfg[mid])
            if mid in index_by_id:
                motors[index_by_id[mid]] = entry
            else:
                motors.append(entry)

        existing['motors'] = motors

        tmp_path = self.zero_config_path + '.tmp'
        with open(tmp_path, 'w', encoding='utf-8') as f:
            json.dump(existing, f, indent=2, ensure_ascii=False)
        os.replace(tmp_path, self.zero_config_path)

    # ==================================================================
    # Low-level helpers
    # ==================================================================

    @staticmethod
    def _shortest_delta(target, current, period):
        """current에서 target(mod period)까지 최단 signed 각도차."""
        half = period / 2.0
        return (target - current + half) % period - half

    def _motor_delta_to_output(self, mid, motor_delta):
        """모터축 각도차를 감속비로 출력축 각도차로 변환한다."""
        cfg = self.motor_cfg[mid]
        return float(motor_delta) / float(cfg['ratio'])

    def _output_delta_to_motor(self, mid, output_delta):
        """출력축 각도차를 감속비로 모터축 각도차로 변환한다."""
        cfg = self.motor_cfg[mid]
        return float(output_delta) * float(cfg['ratio'])

    def _open_real_bus_and_motors(self):
        import serial
        from motor_control_pkg.lk_motor import LKMotor

        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.2)
        except serial.SerialException as e:
            self.get_logger().error(f'[{self.arm_name}] 시리얼 포트 열기 실패: {e}')
            raise

        # 여기서는 통신과 모델만 확인한다. motor_on은 반드시 현재 위치 목표와 함께
        # HOLD 또는 Homing 경로에서 수행해, 목표 없는 torque ON 구간을 만들지 않는다.
        for mid in self.motor_ids:
            motor = LKMotor(self.ser, motor_id=mid)
            try:
                with self.serial_lock:
                    motor.handshake()
                    info = motor.read_info()
            except Exception as e:
                raise RuntimeError(f'motor {mid} 초기화 실패: {e}') from e

            expected_model = self.motor_cfg[mid].get('model')
            reported_model = info.get('motor', '')
            if expected_model and expected_model not in reported_model and reported_model not in expected_model:
                self.get_logger().warn(
                    f'[{self.arm_name}] ID {mid} 모델 불일치: '
                    f"config='{expected_model}', actual='{reported_model}'"
                )

            self.motors[mid] = motor
            self.get_logger().info(
                f'[{self.arm_name}] motor {mid} 연결: {reported_model} (SN {info.get("sn", "")})'
            )

    def _torque_off_all_real_motors(self):
        """모든 실물 모터의 torque를 best-effort로 해제한다."""
        if self.mock_mode:
            self.torque_enabled = False
            return

        for mid, motor in self.motors.items():
            try:
                with self.serial_lock:
                    motor.motor_off()
            except Exception as e:
                self.get_logger().warn(f'[{self.arm_name}] motor {mid} torque OFF 실패: {e}')

        self.torque_enabled = False

    def _hold_current_positions(self, reason):
        """현재 0x92 위치를 목표로 지정해 torque ON과 HOLD를 원자적으로 수행한다.

        모든 모터의 현재 위치를 먼저 읽은 뒤 각 모터에 motor_on -> 0xA4 current
        target을 보낸다. 하나라도 실패하면 전체 torque OFF를 시도하고 실패를 전파한다.
        """
        if self.mock_mode:
            self.torque_enabled = True
            self.teach_mode = False
            self.get_logger().info(f'[{self.arm_name}] mock 현재 위치 HOLD 완료: {reason}')
            return dict(self.mock_angles)

        with self.motion_lock:
            current_92 = {}
            try:
                for mid, motor in self.motors.items():
                    with self.serial_lock:
                        current_92[mid] = motor.read_multi_angle()

                for mid, motor in self.motors.items():
                    ratio = float(self.motor_cfg[mid]['ratio'])
                    cfg_limit = float(
                        self.motor_cfg[mid].get('max_speed_dps', self.default_max_speed_dps)
                    )
                    hold_speed_out = min(
                        max(self.teach_hold_speed_dps, 0.1),
                        cfg_limit,
                        self.default_max_speed_dps,
                    )
                    with self.serial_lock:
                        motor.motor_on()
                        motor.move_to_frame_angle(current_92[mid], hold_speed_out * ratio)
            except Exception as e:
                self._stop_all_real_motors()
                self._torque_off_all_real_motors()
                self.teach_mode = True
                raise RuntimeError(f'현재 위치 HOLD 실패 ({reason}): {e}') from e

            self.torque_enabled = True
            self.teach_mode = False

            held_output = {}
            for mid, cur_92 in current_92.items():
                zero_92 = self.zero_92.get(mid)
                if zero_92 is None:
                    held_output[mid] = None
                else:
                    held_output[mid] = self._motor_delta_to_output(
                        mid, cur_92 - zero_92
                    )

            self.get_logger().info(
                f'[{self.arm_name}] 현재 위치 HOLD 완료 ({reason}): '
                f'output_deg={held_output}'
            )
            return current_92

    def _sync_zero_reference(self):
        """Map saved 0x94 encoder zeros into the current 0x92 frame without motion."""
        if self.mock_mode:
            self.homed = True
            for mid in self.motor_ids:
                self.zero_92[mid] = 0.0
            return

        with self.motion_lock:
            self.homed = False
            mapped = {}
            for mid in self.motor_ids:
                motor = self.motors[mid]
                cfg = self.motor_cfg[mid]
                zero_single = float(cfg['zero_single_deg'])
                period = float(cfg['loop_period_deg'])
                with self.serial_lock:
                    current_single = motor.read_single_angle()
                    current_92 = motor.read_multi_angle()
                delta = self._shortest_delta(zero_single, current_single, period)
                zero_92 = current_92 + delta
                mapped[mid] = zero_92
                self.get_logger().info(
                    f'[{self.arm_name}] [ID {mid}] reference sync: '
                    f'0x94={current_single:.2f}, saved_zero={zero_single:.2f}, '
                    f'delta={delta:+.2f} motor-deg '
                    f'({self._motor_delta_to_output(mid, delta):+.3f} output-deg), '
                    f'zero_92={zero_92:.2f}'
                )
            self.zero_92.update(mapped)
            self.homed = True
            try:
                self._hold_current_positions('reference_only sync 직후')
                positions, failed = self._read_all_output_angles()
                if failed:
                    raise RuntimeError(
                        f'reference sync 직후 위치 확인 실패: motor {sorted(failed)}'
                    )
            except Exception:
                self.homed = False
                raise
            self.get_logger().info(
                f'[{self.arm_name}] reference sync + 현재 위치 HOLD 완료. '
                f'현재 출력축 위치={positions}'
            )

    def _home_all_motors(self):
        """저장된 0x94 절대 영점으로 모든 모터를 자동 복귀시킨다.

        기존 검증된 start_homing()과 동일한 핵심 변환:
            delta   = shortest_delta(zero_single_deg, current_94, loop_period)
            zero_92 = current_92 + delta
            move_to_frame_angle(zero_92, homing_speed_output * ratio)

        모든 이동 명령을 먼저 순서대로 보내고, 그 뒤 0x92를 라운드로빈 폴링해
        각 모터가 목표 오차 안에 연속 N회 들어왔는지 확인한다.
        """
        if self.mock_mode:
            self.homed = True
            self.torque_enabled = True
            self.teach_mode = False
            for mid in self.motor_ids:
                self.zero_92[mid] = 0.0
                self.mock_angles[mid] = 0.0
            return

        with self.motion_lock:
            self.homed = False
            self.get_logger().warn(
                f'[{self.arm_name}] 자동 Homing 시작 - 모터가 저장된 물리 영점으로 이동합니다.'
            )

            plans = {}
            try:
                # 1) 모든 목표를 먼저 계산한다. 하나라도 읽기 실패하면 움직이지 않는다.
                for mid in self.motor_ids:
                    motor = self.motors[mid]
                    cfg = self.motor_cfg[mid]
                    zero_single = float(cfg['zero_single_deg'])
                    period = float(cfg['loop_period_deg'])
                    ratio = float(cfg['ratio'])
                    cfg_limit = float(
                        cfg.get('max_speed_dps', self.default_max_speed_dps)
                    )
                    speed_output = min(
                        max(self.homing_speed_dps, 0.1),
                        cfg_limit,
                        self.default_max_speed_dps,
                    )

                    with self.serial_lock:
                        current_single = motor.read_single_angle()
                        current_92 = motor.read_multi_angle()

                    delta = self._shortest_delta(zero_single, current_single, period)
                    target_zero_92 = current_92 + delta
                    plans[mid] = {
                        'target_92': target_zero_92,
                        'speed_motor': speed_output * ratio,
                    }

                    self.get_logger().warn(
                        f'[{self.arm_name}] [ID {mid}] Homing: '
                        f'0x94 {current_single:.2f} -> {zero_single:.2f} deg, '
                        f'delta={delta:+.2f} motor-deg '
                        f'({self._motor_delta_to_output(mid, delta):+.3f} output-deg), '
                        f'speed={speed_output:.2f} output-deg/s'
                    )

                # 목표 없는 torque ON 구간을 최소화하기 위해 전부 ON한 직후 명령한다.
                for mid in self.motor_ids:
                    with self.serial_lock:
                        self.motors[mid].motor_on()

                self.torque_enabled = True
                self.teach_mode = False

                for mid in self.motor_ids:
                    plan = plans[mid]
                    with self.serial_lock:
                        self.motors[mid].move_to_frame_angle(
                            plan['target_92'], plan['speed_motor']
                        )
                    self.zero_92[mid] = plan['target_92']
            except Exception as e:
                self._stop_all_real_motors()
                self.homed = False
                self.torque_enabled = False
                raise RuntimeError(f'Homing 명령 준비/전송 실패: {e}') from e

            # 2) 목표 오차가 허용범위 안에 연속 N회 들어오는지 확인.
            deadline = time.time() + self.homing_timeout_sec
            stable = {mid: 0 for mid in self.motor_ids}
            done = {mid: False for mid in self.motor_ids}
            last_errors = {mid: None for mid in self.motor_ids}

            while time.time() < deadline and not all(done.values()):
                for mid in self.motor_ids:
                    if done[mid]:
                        continue

                    try:
                        with self.serial_lock:
                            current_92 = self.motors[mid].read_multi_angle()
                    except Exception as e:
                        self._stop_all_real_motors()
                        self.homed = False
                        raise RuntimeError(f'Homing 중 motor {mid} 0x92 읽기 실패: {e}') from e

                    plan = plans[mid]
                    error_output = self._motor_delta_to_output(
                        mid, plan['target_92'] - current_92
                    )
                    last_errors[mid] = error_output
                    if abs(error_output) <= self.homing_target_tolerance_deg:
                        stable[mid] += 1
                    else:
                        stable[mid] = 0

                    if stable[mid] >= self.homing_settle_count:
                        done[mid] = True
                        self.get_logger().info(
                            f'[{self.arm_name}] [ID {mid}] Homing 목표 도착 확인: '
                            f'error={error_output:+.3f} output-deg, '
                            f'{self.homing_settle_count}회 연속 만족'
                        )

                time.sleep(0.05)

            not_done = [mid for mid in self.motor_ids if not done[mid]]
            if not_done:
                self._stop_all_real_motors()
                self.homed = False
                remaining_errors = {
                    mid: last_errors[mid] for mid in not_done
                }
                raise RuntimeError(
                    f'Homing 타임아웃: motor {not_done}, '
                    f'last_errors(output_deg)={remaining_errors}'
                )

            # 3) Homing 후에는 0x92-zero_92를 유일한 위치 기준으로 사용한다.
            self.homed = True
            final_positions, failed = self._read_all_output_angles()
            if failed:
                self.homed = False
                raise RuntimeError(f'Homing 직후 위치 확인 실패: motor {sorted(failed)}')

            self.get_logger().info(
                f'[{self.arm_name}] Homing 완료. 출력축 기준 위치={final_positions}'
            )

    def _read_all_output_angles(self):
        """모든 모터의 현재 출력축 각도[deg]를 읽는다.

        real mode에서는 반드시 0x92 기준으로 계산한다:
            (read_multi_angle() - zero_92) / ratio
        """
        if self.mock_mode:
            return dict(self.mock_angles), set()

        if not self.homed:
            return {mid: self._last_good_output_angle.get(mid, 0.0) for mid in self.motor_ids}, set(self.motor_ids)

        angles = {}
        failed = set()
        for mid in self.motor_ids:
            try:
                with self.serial_lock:
                    cur_92 = self.motors[mid].read_multi_angle()
                angle_out = self._motor_delta_to_output(
                    mid, cur_92 - self.zero_92[mid]
                )
                angles[mid] = angle_out
                self._last_good_output_angle[mid] = angle_out
            except Exception as e:
                self.get_logger().warn(f'[{self.arm_name}] motor {mid} 0x92 읽기 실패: {e}')
                failed.add(mid)
                angles[mid] = self._last_good_output_angle.get(mid, 0.0)

        return angles, failed

    def _stop_all_real_motors(self):
        """실물 모터에 0x81 stop을 보내 즉시 정지한다. 홀딩 토크는 유지된다."""
        if self.mock_mode:
            return
        for mid, motor in self.motors.items():
            try:
                with self.serial_lock:
                    motor.stop()
            except Exception as e:
                self.get_logger().warn(f'[{self.arm_name}] motor {mid} stop 실패: {e}')

    # ==================================================================
    # ROS callbacks
    # ==================================================================

    def polling_callback(self):
        if not self.homed:
            self.get_logger().warn(
                f'[{self.arm_name}] 아직 homing되지 않아 joint_states publish를 대기합니다.',
                throttle_duration_sec=2.0,
            )
            return

        positions, failed = self._read_all_output_angles()
        if failed:
            self.get_logger().warn(
                f'[{self.arm_name}] 읽기 실패 모터 {sorted(failed)} - 마지막 정상값 사용',
                throttle_duration_sec=1.0,
            )

        self.get_logger().info(f'[{self.arm_name}] positions(output_deg)={positions}', throttle_duration_sec=1.0)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'{self.arm_name}_{jn}' for jn in self.joint_names]
        msg.position = [math.radians(positions.get(mid, 0.0)) for mid in self.motor_ids]
        self.joint_state_pub.publish(msg)

    def torque_callback(self, request, response):
        """Raw arm-level torque ON/OFF. For teaching, prefer /teach."""
        turn_on = bool(request.data)
        if self.mock_mode:
            self.torque_enabled = turn_on
            self.teach_mode = False
            response.success = True
            response.message = f'mock torque {"on" if turn_on else "off"}'
            return response
        try:
            with self.motion_lock:
                if turn_on:
                    self._hold_current_positions('/torque true')
                    response.message = (
                        f'torque ON + current-position HOLD '
                        f'({len(self.motors)}개 모터)'
                    )
                else:
                    self._stop_all_real_motors()
                    self._torque_off_all_real_motors()
                    response.message = f'torque OFF ({len(self.motors)}개 모터)'
                self.teach_mode = False
            response.success = True
            self.get_logger().info(f'[{self.arm_name}] {response.message}')
        except Exception as e:
            response.success = False
            response.message = f'torque 명령 실패: {e}'
            self.get_logger().error(f'[{self.arm_name}] {response.message}')
        return response

    def teach_callback(self, request, response):
        """True: STOP -> torque OFF. False: torque ON -> hold hand-guided current position."""
        enable_teach = bool(request.data)
        if self.mock_mode:
            self.teach_mode = enable_teach
            self.torque_enabled = not enable_teach
            response.success = True
            response.message = ('mock teach ON (torque OFF)' if enable_teach
                                else 'mock teach OFF (torque ON + current HOLD)')
            return response
        try:
            with self.motion_lock:
                if enable_teach:
                    self._stop_all_real_motors()
                    self._torque_off_all_real_motors()
                    self.teach_mode = True
                    response.message = (
                        f'teach ON: torque OFF ({len(self.motors)}개 모터). '
                        '팔을 반드시 지지한 상태에서 직접 움직이세요.'
                    )
                else:
                    self._hold_current_positions('teach OFF')
                    response.message = f'teach OFF: torque ON + current-position HOLD ({len(self.motors)}개 모터)'
            response.success = True
            self.get_logger().warn(f'[{self.arm_name}] {response.message}')
        except Exception as e:
            if not self.mock_mode:
                self._torque_off_all_real_motors()
            self.teach_mode = True
            response.success = False
            response.message = f'teach 전환 실패: {e}; 안전 fallback으로 전체 torque OFF 시도'
            self.get_logger().error(f'[{self.arm_name}] {response.message}')
        return response

    def sync_reference_callback(self, request, response):
        try:
            self._sync_zero_reference()
            response.success = True
            response.message = (
                f'reference sync + current-position HOLD 완료 '
                f'(motor_ids={self.motor_ids})'
            )
        except Exception as e:
            response.success = False
            response.message = f'reference sync 실패: {e}'
            self.get_logger().error(f'[{self.arm_name}] {response.message}')
        return response

    def home_callback(self, request, response):
        """저장된 절대 엔코더 영점으로 다시 자동 homing한다."""
        try:
            if self.teach_mode:
                raise RuntimeError('teach mode에서는 home을 실행할 수 없습니다.')
            self._home_all_motors()
            response.success = True
            response.message = f'home 완료 (motor_ids={self.motor_ids})'
        except Exception as e:
            response.success = False
            response.message = f'home 실패: {e}'
            self.get_logger().error(f'[{self.arm_name}] {response.message}')
        return response

    def set_zero_callback(self, request, response):
        """현재 물리 위치를 새로운 절대 영점으로 지정한다.

        real mode에서는 각 모터의 0x94 절대각을 읽어
        zero_config.json의 zero_single_deg에 저장한다.

        0x90 encoder read는 일부 모터/펌웨어에서 응답하지 않을 수 있으므로
        optional로 처리한다. 실패하면 zero_encoder / zero_raw는 None으로 저장한다.

        이 서비스는 매 부팅마다 호출하는 기능이 아니다.
        조립 위치가 바뀌거나 물리 영점을 다시 캘리브레이션할 때만 호출한다.
        """
        if self.mock_mode:
            for mid in self.motor_ids:
                self.mock_angles[mid] = 0.0
                self.zero_92[mid] = 0.0

            self.homed = True
            response.success = True
            response.message = 'mock set_zero 완료'
            return response
        try:
            with self.motion_lock:
                new_values = {}

                for mid in self.motor_ids:
                    with self.serial_lock:
                        # 모터 모델 확인
                        info = self.motors[mid].read_info()

                        #homing 기준으로 실제 사용하는 절대각
                        zero_single = self.motors[mid].read_single_angle()

                        # 0x90 encoder read는 일부 모터/펌웨어에서 응답하지 않을 수 있으므로 optional로 처리한다.
                        try:
                            enc = self.motors[mid].read_encoder()
                        except Exception as e:
                            self.get_logger().warn(
                               f'ID {mid}: 0x90 encoder read 실패. '
                               f'zero_encoder/zero_raw는 null로 저장합니다: {e}'
                            )
                            enc = {
                                'encoder': None,
                                'raw': None,
                            }

                        # 현재 세션의 0x92 좌표
                        cur_92 = self.motors[mid].read_multi_angle()

                    # config 모델명과 실제 모터 모델 확인
                    expected_model = self.motor_cfg[mid].get('model')
                    reported_model = info.get('motor', '')

                    if (
                        expected_model
                        and expected_model not in reported_model
                        and reported_model not in expected_model
                    ):
                        raise RuntimeError(
                            f'ID {mid}: 모델 불일치: config={expected_model}, actual={reported_model}'
                        )

                    new_values[mid] = (
                        zero_single,
                        enc,
                        cur_92,
                    )

                #모든 모터의 필수값(0x94, 0x92) 읽기가 성공한 뒤에만 저장
                for mid, (zero_single, enc, cur_92) in new_values.items():
                    self.motor_cfg[mid]['zero_single_deg'] = zero_single
                    self.motor_cfg[mid]['zero_encoder'] = enc.get('encoder')
                    self.motor_cfg[mid]['zero_raw'] = enc.get('raw')
                    self.zero_92[mid] = cur_92

                self._save_motor_config()
                self.homed = True

            response.success = True
            response.message = (
                '절대 영점 저장 완료: '
                + ', '.join(
                    f'ID {mid}=0x94 '
                    f'{self.motor_cfg[mid]["zero_single_deg"]:.2f} deg'
                    for mid in self.motor_ids
                )
            )

            self.get_logger().warn(
                f'[{self.arm_name}] {response.message} '
                f'-> {self.zero_config_path}'
            )
        except Exception as e:
            response.success = False
            response.message = f'set_zero 실패: {e}'
            self.get_logger().error(
                f'[{self.arm_name}] {response.message}'
            )
        return response

    def cancel_move_callback(self, goal_handle):
        self.get_logger().info(f'[{self.arm_name}] [Action] 취소 요청 수신')
        return CancelResponse.ACCEPT

    def execute_move_callback(self, goal_handle):
        """MoveJoint: 출력축 기준 목표각[deg]으로 여러 모터를 동기 이동한다.

        Homing 이후에는 0x94를 위치 추적/목표 계산에 사용하지 않는다.
        target_92 = zero_92 + target_output_deg * ratio
        speed_motor = speed_output_dps * ratio
        로 0xA4에 직접 보낸다.
        """
        result = MoveJoint.Result()

        if not self.homed:
            result.success = False
            result.timeout = False
            result.error_message = '아직 homing되지 않았습니다. /home을 먼저 실행하세요.'
            goal_handle.abort()
            return result

        if self.teach_mode or not self.torque_enabled:
            result.success = False
            result.timeout = False
            result.error_message = 'teach mode 또는 torque OFF 상태에서는 이동할 수 없습니다.'
            goal_handle.abort()
            return result

        target_angles = list(goal_handle.request.target_angles)
        requested_speeds = list(goal_handle.request.max_speeds)

        if len(target_angles) != len(self.motor_ids) or len(requested_speeds) != len(self.motor_ids):
            result.success = False
            result.timeout = False
            result.error_message = (
                f'target_angles/max_speeds 길이가 motor_ids 개수({len(self.motor_ids)})와 다릅니다.'
            )
            goal_handle.abort()
            self.get_logger().error(f'[{self.arm_name}] [Action] {result.error_message}')
            return result

        targets = {mid: float(v) for mid, v in zip(self.motor_ids, target_angles)}

        # 사용자가 요청한 속도와 config 상한 중 작은 값을 사용한다. 단위는 출력축 deg/s.
        speed_limits = {}
        for mid, requested in zip(self.motor_ids, requested_speeds):
            config_limit = float(self.motor_cfg[mid].get('max_speed_dps', self.default_max_speed_dps))
            limit = min(max(float(requested), 0.0), config_limit, self.default_max_speed_dps)
            speed_limits[mid] = limit

        with self.motion_lock:
            current, failed = self._read_all_output_angles()
            if failed:
                result.success = False
                result.timeout = False
                result.error_message = f'motor {sorted(failed)} 위치 읽기 실패로 이동 시작 취소'
                goal_handle.abort()
                return result

            distances = {mid: abs(targets[mid] - current[mid]) for mid in self.motor_ids}

            for mid in self.motor_ids:
                if distances[mid] > 1e-6 and speed_limits[mid] <= 1e-6:
                    result.success = False
                    result.timeout = False
                    result.error_message = f'motor {mid}는 이동이 필요한데 max_speed가 0입니다.'
                    goal_handle.abort()
                    return result

            moving = [mid for mid in self.motor_ids if distances[mid] > 1e-6]
            if not moving:
                result.success = True
                result.timeout = False
                result.error_message = ''
                goal_handle.succeed()
                return result

            # 기존 Python 코드와 같은 synchronized arrival 계산.
            duration_t = max(distances[mid] / speed_limits[mid] for mid in moving)
            synced_speeds = {
                mid: (distances[mid] / duration_t if mid in moving else 0.0)
                for mid in self.motor_ids
            }

            self.get_logger().info(
                f'[{self.arm_name}] [Action] 이동 시작: targets(output_deg)={targets}, '
                f'duration≈{duration_t:.2f}s'
            )

            if self.mock_mode:
                pass
            else:
                for mid in self.motor_ids:
                    if distances[mid] <= 1e-6:
                        continue

                    ratio = float(self.motor_cfg[mid]['ratio'])
                    target_92 = self.zero_92[mid] + self._output_delta_to_motor(
                        mid, targets[mid]
                    )
                    speed_motor = synced_speeds[mid] * ratio

                    try:
                        with self.serial_lock:
                            self.motors[mid].move_to_frame_angle(target_92, speed_motor)
                    except Exception as e:
                        self._stop_all_real_motors()
                        result.success = False
                        result.timeout = False
                        result.error_message = f'motor {mid} 이동 명령 실패: {e}'
                        goal_handle.abort()
                        self.get_logger().error(f'[{self.arm_name}] [Action] {result.error_message}')
                        return result

            feedback = MoveJoint.Feedback()
            deadline = time.time() + max(duration_t + 3.0, 8.0)
            stable_count = 0

            while True:
                if goal_handle.is_cancel_requested:
                    self._stop_all_real_motors()
                    goal_handle.canceled()
                    result.success = False
                    result.timeout = False
                    result.error_message = '사용자 취소'
                    return result

                if time.time() > deadline:
                    self._stop_all_real_motors()
                    result.success = False
                    result.timeout = True
                    result.error_message = '동기 이동 타임아웃'
                    goal_handle.abort()
                    return result

                if self.mock_mode:
                    # 0.1초 제어주기 가정으로 출력축 각도를 단순 적분한다.
                    for mid in self.motor_ids:
                        current_angle = self.mock_angles[mid]
                        step = synced_speeds[mid] * 0.1
                        error = targets[mid] - current_angle
                        if abs(error) <= step:
                            self.mock_angles[mid] = targets[mid]
                        elif step > 0.0:
                            self.mock_angles[mid] += step if error > 0.0 else -step
                    current = dict(self.mock_angles)
                    failed = set()
                else:
                    current, failed = self._read_all_output_angles()

                if failed:
                    self._stop_all_real_motors()
                    result.success = False
                    result.timeout = False
                    result.error_message = f'motor {sorted(failed)} 위치 읽기 실패'
                    goal_handle.abort()
                    return result

                errors = {mid: targets[mid] - current[mid] for mid in self.motor_ids}
                all_settled = all(
                    abs(errors[mid]) <= self.move_target_tolerance_deg
                    for mid in moving
                )

                feedback.current_angles = [current[mid] for mid in self.motor_ids]
                feedback.errors = [errors[mid] for mid in self.motor_ids]
                feedback.settled = all_settled
                goal_handle.publish_feedback(feedback)

                stable_count = stable_count + 1 if all_settled else 0
                if stable_count >= self.move_settle_count:
                    result.success = True
                    result.timeout = False
                    result.error_message = ''
                    goal_handle.succeed()
                    self.get_logger().info(f'[{self.arm_name}] [Action] 이동 완료: {current}')
                    return result

                time.sleep(0.1)

    # ==================================================================
    # Shutdown
    # ==================================================================

    def shutdown_safely(self):
        if self.mock_mode:
            return
        for mid, motor in self.motors.items():
            try:
                with self.serial_lock:
                    motor.motor_off()
                self.get_logger().info(f'[{self.arm_name}] motor {mid} 토크 해제 완료')
            except Exception as e:
                self.get_logger().warn(f'[{self.arm_name}] motor {mid} 토크 해제 실패: {e}')

        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = MotorControlNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown_safely()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
