#!/usr/bin/env python3
import json
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

        # --- parameter 선언 ---
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('arm_name', 'left_arm')
        self.declare_parameter('motor_ids', [1, 2, 3, 4])
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4'])
        self.declare_parameter('polling_hz', 30.0)
        self.declare_parameter('max_speed_dps', 60.0)
        self.declare_parameter('mock_mode', True)
        self.declare_parameter('zero_config_path', '')
        # 0x94(read_single_angle)의 랩어라운드 주기. 모터축 기준 deg.
        # zero_config.json 관례상 ratio*360 (기본 10:1 감속비 -> 3600.0).
        self.declare_parameter('loop_period_deg', 3600.0)

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.arm_name = self.get_parameter('arm_name').value
        self.motor_ids = self.get_parameter('motor_ids').value
        self.joint_names = self.get_parameter('joint_names').value
        self.polling_hz = self.get_parameter('polling_hz').value
        self.max_speed_dps = self.get_parameter('max_speed_dps').value
        self.mock_mode = self.get_parameter('mock_mode').value
        self.loop_period_deg = self.get_parameter('loop_period_deg').value

        zero_config_path = self.get_parameter('zero_config_path').value
        if not zero_config_path:
            zero_config_path = os.path.expanduser(f"~/.ros/zero_offset_{self.arm_name}.json")
        self.zero_config_path = zero_config_path

        self.get_logger().info(
            f"[{self.arm_name}] 시작: port={self.serial_port}, "
            f"motor_ids={self.motor_ids}, mock_mode={self.mock_mode}, "
            f"zero_config_path={self.zero_config_path}"
        )

        # --- RS485 버스 하나를 여러 스레드(Timer/Action)가 공유하므로,
        #     모든 serial 통신(모터 read/write)은 반드시 이 lock 안에서만 수행한다.
        #     (동시 접근 시 반이중 버스에서 요청/응답이 서로 섞일 수 있음) ---
        self.serial_lock = threading.Lock()

        self.motors = {}
        self.mock_angles = {}
        self.ser = None
        self._last_good_angle = {}  # {motor_id: 마지막으로 성공했던 raw 각도} - 읽기 실패 시 대체값

        self.zero_offset = self._load_zero_offset()

        if self.mock_mode:
            self.get_logger().info(f"[{self.arm_name}] mock_mode=True, 실제 serial 포트를 열지 않습니다.")
            for mid in self.motor_ids:
                self.mock_angles[mid] = 0.0
        else:
            import serial
            from motor_control_pkg.lk_motor import LKMotor
            try:
                self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.2)
            except serial.SerialException as e:
                self.get_logger().error(f"[{self.arm_name}] 시리얼 포트 열기 실패: {e}")
                raise

            for mid in self.motor_ids:
                motor = LKMotor(self.ser, motor_id=mid)
                motor.handshake()
                motor.motor_on()
                self.motors[mid] = motor

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        timer_period = 1.0 / self.polling_hz
        self.timer = self.create_timer(timer_period, self.polling_callback)

        self.torque_srv = self.create_service(SetBool, 'torque', self.torque_callback)
        self.set_zero_srv = self.create_service(Trigger, 'set_zero', self.set_zero_callback)

        self._action_callback_group = ReentrantCallbackGroup()
        self.move_action_server = ActionServer(
            self,
            MoveJoint,
            'move_to',
            execute_callback=self.execute_move_callback,
            cancel_callback=self.cancel_move_callback,
            callback_group=self._action_callback_group,
        )

    # --- 영점 파일 저장/복원 ---------------------------------------------

    def _load_zero_offset(self):
        default = {mid: 0.0 for mid in self.motor_ids}
        if not os.path.exists(self.zero_config_path):
            self.get_logger().info(
                f"[{self.arm_name}] 저장된 영점 파일이 없습니다({self.zero_config_path}). "
                f"전부 0.0으로 시작합니다. /set_zero 로 영점을 설정하세요."
            )
            return default
        try:
            with open(self.zero_config_path) as f:
                data = json.load(f)
            saved = {int(k): v for k, v in data.get("zero_offset", {}).items()}
            loaded = {}
            for mid in self.motor_ids:
                if mid not in saved:
                    self.get_logger().warn(
                        f"[{self.arm_name}] 저장된 영점에 motor_id {mid} 값이 없습니다. 0.0으로 대체합니다."
                    )
                    loaded[mid] = 0.0
                else:
                    loaded[mid] = saved[mid]
            self.get_logger().info(f"[{self.arm_name}] 영점 복원 완료: {loaded} (from {self.zero_config_path})")
            return loaded
        except (json.JSONDecodeError, OSError, ValueError) as e:
            self.get_logger().error(
                f"[{self.arm_name}] 영점 파일 읽기 실패({e}), 전부 0.0으로 시작합니다. "
                f"반드시 /set_zero 로 영점을 다시 설정하세요."
            )
            return default

    def _save_zero_offset(self):
        os.makedirs(os.path.dirname(self.zero_config_path), exist_ok=True)
        data = {
            "arm_name": self.arm_name,
            "motor_ids": self.motor_ids,
            "zero_offset": {str(mid): val for mid, val in self.zero_offset.items()},
        }
        tmp_path = self.zero_config_path + ".tmp"
        with open(tmp_path, "w") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        os.replace(tmp_path, self.zero_config_path)

    # --- 각도 읽기 / frame 변환 헬퍼 --------------------------------------

    @staticmethod
    def _shortest_delta(target, current, period):
        """current 에서 target(mod period) 으로 가는 최단 signed 각도차."""
        half = period / 2.0
        return (target - current + half) % period - half

    def _read_all_raw_angles(self):
        """motor_ids 전체의 현재 raw(0x94, 영점 보정 전) 각도를 읽는다.
        Returns: (angles: {motor_id: deg}, failed: set(motor_id))
        읽기 실패한 모터는 마지막으로 성공했던 값(없으면 0.0)으로 채우되,
        어떤 모터가 실패했는지 failed 집합으로 반드시 알려준다.
        (호출부가 실패를 무시하고 그냥 0.0을 진짜 값처럼 쓰는 일이 없도록 하기 위함 —
        특히 set_zero 에서 실패값을 영점으로 영구 저장하는 사고를 막는 게 목적)"""
        if self.mock_mode:
            return dict(self.mock_angles), set()

        angles = {}
        failed = set()
        for mid, motor in self.motors.items():
            try:
                with self.serial_lock:
                    angle = motor.read_single_angle()
                angles[mid] = angle
                self._last_good_angle[mid] = angle
            except Exception as e:
                self.get_logger().warn(f"[{self.arm_name}] motor {mid} 읽기 실패: {e}")
                failed.add(mid)
                angles[mid] = self._last_good_angle.get(mid, 0.0)
        return angles, failed

    def _stop_all_real_motors(self):
        """실제 하드웨어라면 즉시 정지 명령(0x81)을 보낸다.
        goal_handle.abort()/canceled() 는 ROS2 쪽 상태만 바꿀 뿐 모터에게 아무 신호도
        안 보내므로, 실제로 모터를 멈추려면 이 호출이 반드시 필요하다."""
        if self.mock_mode:
            return
        for mid, motor in self.motors.items():
            try:
                with self.serial_lock:
                    motor.stop()
            except Exception as e:
                self.get_logger().warn(f"[{self.arm_name}] motor {mid} stop 실패: {e}")

    # -----------------------------------------------------------------

    def polling_callback(self):
        raw_positions, failed = self._read_all_raw_angles()
        if failed:
            self.get_logger().warn(
                f"[{self.arm_name}] 읽기 실패 모터 {sorted(failed)} - 마지막 정상값 사용",
                throttle_duration_sec=1.0,
            )
        positions = {mid: raw_positions.get(mid, 0.0) - self.zero_offset[mid] for mid in self.motor_ids}

        self.get_logger().info(f"[{self.arm_name}] positions={positions}", throttle_duration_sec=1.0)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"{self.arm_name}_{jn}" for jn in self.joint_names]
        msg.position = [
            positions.get(mid, 0.0) * 3.14159265 / 180.0
            for mid in self.motor_ids
        ]
        self.joint_state_pub.publish(msg)

    def torque_callback(self, request, response):
        turn_on = request.data
        if self.mock_mode:
            self.get_logger().info(f"[{self.arm_name}] [mock] torque {'ON' if turn_on else 'OFF'}")
            response.success = True
            response.message = f"mock torque {'on' if turn_on else 'off'}"
            return response
        try:
            for mid, motor in self.motors.items():
                with self.serial_lock:
                    if turn_on:
                        motor.motor_on()
                    else:
                        motor.motor_off()
            response.success = True
            response.message = f"torque {'on' if turn_on else 'off'} 완료 ({len(self.motors)}개 모터)"
            self.get_logger().info(f"[{self.arm_name}] {response.message}")
        except Exception as e:
            response.success = False
            response.message = f"torque 명령 실패: {e}"
            self.get_logger().error(f"[{self.arm_name}] {response.message}")
        return response

    def set_zero_callback(self, request, response):
        """현재 각도를 새로운 영점으로 설정하고 파일에 영구 저장한다.
        읽기 실패한 모터가 하나라도 있으면, 절대 저장하지 않고 실패로 응답한다
        (실패값 0.0을 영점으로 영구히 남기는 사고 방지)."""
        try:
            raw_positions, failed = self._read_all_raw_angles()
            if failed:
                response.success = False
                response.message = f"motor {sorted(failed)} 읽기 실패로 set_zero 중단 (영점 저장 안 함)"
                self.get_logger().error(f"[{self.arm_name}] {response.message}")
                return response

            new_offset = dict(self.zero_offset)
            for mid in self.motor_ids:
                new_offset[mid] = raw_positions[mid]
            self.zero_offset = new_offset
            self._save_zero_offset()

            response.success = True
            response.message = f"set_zero 완료 및 파일 저장됨 (motor_ids={self.motor_ids})"
            self.get_logger().info(
                f"[{self.arm_name}] {response.message}: offset={self.zero_offset} -> {self.zero_config_path}"
            )
        except Exception as e:
            response.success = False
            response.message = f"set_zero 실패: {e}"
            self.get_logger().error(f"[{self.arm_name}] {response.message}")
        return response

    def cancel_move_callback(self, goal_handle):
        self.get_logger().info(f"[{self.arm_name}] [Action] 취소 요청 수신")
        return CancelResponse.ACCEPT

    def execute_move_callback(self, goal_handle):
        """MoveJoint 액션: motor_ids 전체를 target_angles(영점 기준 상대각)까지 동기 이동시킨다."""
        target_angles_rel = list(goal_handle.request.target_angles)
        requested_speeds = list(goal_handle.request.max_speeds)

        if len(target_angles_rel) != len(self.motor_ids) or len(requested_speeds) != len(self.motor_ids):
            result = MoveJoint.Result()
            result.success = False
            result.timeout = False
            result.error_message = (
                f"target_angles/max_speeds 길이가 motor_ids 개수({len(self.motor_ids)})와 다릅니다."
            )
            goal_handle.abort()
            self.get_logger().error(f"[{self.arm_name}] [Action] {result.error_message}")
            return result

        targets_rel = dict(zip(self.motor_ids, target_angles_rel))
        # 안전 상한: 요청 속도가 max_speed_dps 를 넘지 못하게 클램프
        speeds = {
            mid: min(max(v, 0.0), self.max_speed_dps)
            for mid, v in zip(self.motor_ids, requested_speeds)
        }

        current_raw, failed = self._read_all_raw_angles()
        if failed:
            result = MoveJoint.Result()
            result.success = False
            result.timeout = False
            result.error_message = f"motor {sorted(failed)} 읽기 실패로 이동 시작 취소"
            goal_handle.abort()
            self.get_logger().error(f"[{self.arm_name}] [Action] {result.error_message}")
            return result

        targets_raw = {mid: targets_rel[mid] + self.zero_offset[mid] for mid in self.motor_ids}
        self.get_logger().info(f"[{self.arm_name}] [Action] 동기 이동 시작: targets(상대각)={targets_rel}")

        distances = {
            mid: abs(self._shortest_delta(targets_raw[mid], current_raw[mid], self.loop_period_deg))
            for mid in self.motor_ids
        }
        durations = {
            mid: (distances[mid] / speeds[mid] if speeds[mid] > 1e-6 else 0.0)
            for mid in self.motor_ids
        }
        duration_T = max(durations.values()) if durations else 0.0

        synced_speeds = {}
        for mid in self.motor_ids:
            if duration_T > 1e-6 and distances[mid] > 1e-6:
                synced_speeds[mid] = min(distances[mid] / duration_T, self.max_speed_dps)
            else:
                synced_speeds[mid] = 0.0

        result = MoveJoint.Result()

        if self.mock_mode:
            pass  # mock 은 아래 루프에서 mock_angles를 직접 적분
        else:
            # --- 실제 이동 명령 전송: 0x94(single-angle) frame 목표를
            #     0x92/0xA4(multi-angle) frame으로 변환해서 보낸다.
            #     0x94 는 전원 재인가에도 유지되는 절대각이고, 0x92 는 전원 인가 후
            #     0부터 누적되는 값이라 서로 다른 frame이다 — 그대로 섞어 보내면
            #     엉뚱한 위치로 이동 명령이 나갈 수 있다. (기존 motor_control.py의
            #     start_homing() 이 하던 변환을 그대로 재현)
            for mid in self.motor_ids:
                if synced_speeds[mid] <= 1e-6:
                    continue
                try:
                    with self.serial_lock:
                        current_single = self.motors[mid].read_single_angle()
                        current_92 = self.motors[mid].read_multi_angle()
                    delta = self._shortest_delta(targets_raw[mid], current_single, self.loop_period_deg)
                    target_92 = current_92 + delta
                    with self.serial_lock:
                        self.motors[mid].move_to_frame_angle(target_92, synced_speeds[mid])
                except Exception as e:
                    result.success = False
                    result.timeout = False
                    result.error_message = f"motor {mid} 이동 명령 실패: {e}"
                    self._stop_all_real_motors()
                    goal_handle.abort()
                    self.get_logger().error(f"[{self.arm_name}] [Action] {result.error_message}")
                    return result

        feedback = MoveJoint.Feedback()
        deadline = time.time() + max(duration_T + 3.0, 8.0)
        settle_deg = 0.2
        stable_count = 0

        while True:
            if goal_handle.is_cancel_requested:
                self._stop_all_real_motors()
                goal_handle.canceled()
                result.success = False
                result.timeout = False
                result.error_message = "사용자 취소"
                self.get_logger().info(f"[{self.arm_name}] [Action] 취소됨")
                return result

            if time.time() > deadline:
                self._stop_all_real_motors()
                result.success = False
                result.timeout = True
                result.error_message = "동기 이동 타임아웃"
                goal_handle.abort()
                self.get_logger().warn(f"[{self.arm_name}] [Action] {result.error_message}")
                return result

            if self.mock_mode:
                for mid in self.motor_ids:
                    current = self.mock_angles[mid]
                    step = synced_speeds[mid] * 0.1
                    if abs(targets_raw[mid] - current) <= step:
                        self.mock_angles[mid] = targets_raw[mid]
                    else:
                        self.mock_angles[mid] += step if targets_raw[mid] > current else -step
                current_raw = dict(self.mock_angles)
            else:
                current_raw, _ = self._read_all_raw_angles()

            current_rel = {mid: current_raw[mid] - self.zero_offset[mid] for mid in self.motor_ids}
            errors = {
                mid: self._shortest_delta(targets_rel[mid], current_rel[mid], self.loop_period_deg)
                for mid in self.motor_ids
            }
            all_settled = all(abs(errors[mid]) < settle_deg for mid in self.motor_ids)

            feedback.current_angles = [current_rel[mid] for mid in self.motor_ids]
            feedback.errors = [errors[mid] for mid in self.motor_ids]
            feedback.settled = all_settled
            goal_handle.publish_feedback(feedback)

            if all_settled:
                stable_count += 1
            else:
                stable_count = 0

            if stable_count >= 3:
                result.success = True
                result.timeout = False
                result.error_message = ""
                goal_handle.succeed()
                self.get_logger().info(f"[{self.arm_name}] [Action] 동기 이동 완료: {current_rel}")
                return result

            time.sleep(0.1)

    def shutdown_safely(self):
        if self.mock_mode:
            return
        for mid, motor in self.motors.items():
            try:
                with self.serial_lock:
                    motor.motor_off()
                self.get_logger().info(f"[{self.arm_name}] motor {mid} 토크 해제 완료")
            except Exception as e:
                self.get_logger().warn(f"[{self.arm_name}] motor {mid} 토크 해제 실패: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_safely()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()