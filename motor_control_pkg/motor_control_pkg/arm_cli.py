#!/usr/bin/env python3
"""Direct joint-angle CLI for the iROI right, left, or dual arm system.

Motor topology:
  right_arm: IDs 1, 2, 3, 4
  left_arm:  IDs 5, 6, 7, 8

Each active motor accepts a numeric output-axis angle or ``null``. A null target
is replaced with that motor's latest calibrated joint angle, which keeps the
current position when the MoveJoint Action requires a full target array.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from iroi_interfaces.action import MoveJoint


@dataclass(frozen=True)
class ArmSpec:
    key: str
    namespace: str
    motor_ids: tuple[int, ...]


RIGHT_SPEC = ArmSpec("right", "/right_arm", (1, 2, 3, 4))
LEFT_SPEC = ArmSpec("left", "/left_arm", (5, 6, 7, 8))
MODE_SPECS = {
    "right": (RIGHT_SPEC,),
    "left": (LEFT_SPEC,),
    "dual": (RIGHT_SPEC, LEFT_SPEC),
}


class ArmCLI(Node):
    def __init__(self):
        super().__init__("arm_cli")

        self.declare_parameter("mode", "right")
        self.declare_parameter("max_speed_dps", 60.0)
        self.declare_parameter("state_timeout_sec", 5.0)
        self.declare_parameter("state_freshness_sec", 2.0)

        self.mode = str(self.get_parameter("mode").value).strip().lower()
        if self.mode not in MODE_SPECS:
            raise ValueError("mode는 right/left/dual 중 하나여야 합니다.")
        self.specs = MODE_SPECS[self.mode]
        self.motor_ids = tuple(
            mid for spec in self.specs for mid in spec.motor_ids
        )

        self.max_speed_dps = float(self.get_parameter("max_speed_dps").value)
        self.state_timeout_sec = float(
            self.get_parameter("state_timeout_sec").value
        )
        self.state_freshness_sec = float(
            self.get_parameter("state_freshness_sec").value
        )
        if self.max_speed_dps <= 0.0:
            raise ValueError("max_speed_dps는 0보다 커야 합니다.")
        if self.state_timeout_sec <= 0.0:
            raise ValueError("state_timeout_sec는 0보다 커야 합니다.")
        if self.state_freshness_sec <= 0.0:
            raise ValueError("state_freshness_sec는 0보다 커야 합니다.")

        self._state_lock = threading.Lock()
        self.current_angles: Dict[int, float] = {}
        self.last_state_time: Dict[str, float] = {}
        self.action_clients: Dict[str, ActionClient] = {}

        for spec in self.specs:
            self.create_subscription(
                JointState,
                f"{spec.namespace}/joint_states",
                self._make_joint_state_callback(spec),
                10,
            )
            self.action_clients[spec.key] = ActionClient(
                self,
                MoveJoint,
                f"{spec.namespace}/move_to",
            )

    def _make_joint_state_callback(self, spec: ArmSpec):
        def callback(msg: JointState):
            if len(msg.position) < len(spec.motor_ids):
                self.get_logger().warn(
                    f"[{spec.key}] joint_states 길이 부족: "
                    f"{len(msg.position)} < {len(spec.motor_ids)}",
                    throttle_duration_sec=2.0,
                )
                return

            with self._state_lock:
                for mid, radians in zip(spec.motor_ids, msg.position):
                    self.current_angles[mid] = math.degrees(float(radians))
                self.last_state_time[spec.key] = time.time()

        return callback

    @staticmethod
    def _wait_future(future, timeout_sec: float):
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.time() > deadline:
                raise TimeoutError("ROS 응답 대기 timeout")
            time.sleep(0.02)
        return future.result()

    def _current_for(self, spec: ArmSpec) -> Dict[int, float]:
        with self._state_lock:
            missing = [
                mid for mid in spec.motor_ids if mid not in self.current_angles
            ]
            if missing:
                raise RuntimeError(
                    f"[{spec.key}] 현재 각도 미수신 motor: {missing}"
                )

            state_time = self.last_state_time.get(spec.key)
            age = None if state_time is None else time.time() - state_time
            if age is None or age > self.state_freshness_sec:
                age_text = "없음" if age is None else f"{age:.2f}초"
                raise RuntimeError(
                    f"[{spec.key}] joint_states가 오래되었습니다(age={age_text})."
                )

            return {mid: self.current_angles[mid] for mid in spec.motor_ids}

    def _wait_for_initial_states(self) -> None:
        deadline = time.time() + self.state_timeout_sec
        last_error = "joint_states 미수신"
        while rclpy.ok() and time.time() < deadline:
            try:
                for spec in self.specs:
                    self._current_for(spec)
                return
            except RuntimeError as exc:
                last_error = str(exc)
                time.sleep(0.05)
        raise RuntimeError(f"초기 joint_states 준비 실패: {last_error}")

    def connect(self) -> None:
        print(f"[arm] mode={self.mode}, motor order={list(self.motor_ids)}")
        print("[arm] Action 연결 확인 중...")
        for spec in self.specs:
            if not self.action_clients[spec.key].wait_for_server(timeout_sec=5.0):
                raise RuntimeError(f"{spec.namespace}/move_to Action이 없습니다.")

        print("[arm] 최신 joint_states 확인 중...")
        self._wait_for_initial_states()
        print("[arm] 연결 완료")
        print()
        self.print_help()

    def _validate_speed(self, speed: float) -> float:
        speed = float(speed)
        if not math.isfinite(speed):
            raise ValueError("속도는 유한한 숫자여야 합니다.")
        if speed <= 0.0:
            raise ValueError("속도는 0보다 커야 합니다.")
        if speed > self.max_speed_dps:
            raise ValueError(
                f"최대 속도는 {self.max_speed_dps:.1f} deg/s입니다."
            )
        return speed

    @staticmethod
    def _parse_angle(value: str) -> Optional[float]:
        if value.strip().lower() == "null":
            return None
        angle = float(value)
        if not math.isfinite(angle):
            raise ValueError("각도는 유한한 숫자 또는 null이어야 합니다.")
        # CLI는 한 바퀴 표시 체계(0 <= angle < 360)만 노출한다. 실제 목표의
        # 연속 좌표 선택은 motor_control_node가 현재 위치 기준으로 처리한다.
        return angle % 360.0

    @staticmethod
    def _display_angle(value: float) -> float:
        """연속 관절 좌표를 사람이 읽는 0~360도 표기로 바꾼다."""
        return float(value) % 360.0

    def move(self, requested: Dict[int, Optional[float]], speed: float) -> None:
        speed = self._validate_speed(speed)

        # 모든 활성 팔의 최신 상태와 target을 먼저 검사한다. 이 단계에서는
        # Action을 보내지 않아 입력/상태 오류로 인한 부분 이동을 막는다.
        plans = []
        held_ids = []
        for spec in self.specs:
            current = self._current_for(spec)
            targets = {}
            has_numeric_target = False
            for mid in spec.motor_ids:
                value = requested[mid]
                if value is None:
                    targets[mid] = current[mid]
                    held_ids.append(mid)
                else:
                    targets[mid] = float(value)
                    has_numeric_target = True

            if has_numeric_target:
                plans.append((spec, targets))

        if not plans:
            print("[arm] 모든 입력이 null입니다. 이동 명령을 보내지 않습니다.")
            return

        pending = []
        for spec, targets in plans:
            goal = MoveJoint.Goal()
            goal.target_angles = [targets[mid] for mid in spec.motor_ids]
            goal.max_speeds = [speed] * len(spec.motor_ids)
            pending.append(
                (spec, goal, self.action_clients[spec.key].send_goal_async(goal))
            )

        handles = []
        for spec, goal, future in pending:
            handle = self._wait_future(future, timeout_sec=10.0)
            if handle is None or not handle.accepted:
                raise RuntimeError(f"[{spec.key}] MoveJoint goal 거절")
            handles.append((spec, goal, handle))

        print(
            f"[arm] 이동 시작: speed={speed:.2f} deg/s, "
            f"null HOLD IDs={sorted(held_ids)}"
        )
        for spec, goal, _ in handles:
            print(
                f"      {spec.key}: IDs={list(spec.motor_ids)}, "
                f"targets={list(goal.target_angles)}"
            )

        for spec, _, handle in handles:
            response = self._wait_future(
                handle.get_result_async(),
                timeout_sec=120.0,
            )
            if response is None:
                raise RuntimeError(f"[{spec.key}] 이동 결과 없음")
            if not response.result.success:
                raise RuntimeError(
                    f"[{spec.key}] 이동 실패: {response.result.error_message}"
                )

        print("[arm] 이동 완료")

    def status(self) -> None:
        print("CURRENT OUTPUT ANGLES")
        with self._state_lock:
            for spec in self.specs:
                state_time = self.last_state_time.get(spec.key)
                age = None if state_time is None else time.time() - state_time
                age_text = "no state" if age is None else f"age={age:.2f}s"
                print(f"  {spec.key}: {age_text}")
                for mid in spec.motor_ids:
                    value = self.current_angles.get(mid)
                    text = "NULL" if value is None else f"{self._display_angle(value):.3f}°"
                    print(f"    ID {mid}: {text}")

    def print_help(self) -> None:
        count = len(self.motor_ids)
        print("입력 형식:")
        print(f"  각도 {count}개(ID 순서) + 속도 1개")
        print("  각도 대신 null을 쓰면 해당 모터는 현재 위치를 유지합니다.")
        print(f"  모터 순서: {list(self.motor_ids)}")
        print()
        if self.mode == "right":
            print("  예: 10 355 20 0 15")
        elif self.mode == "left":
            print("  예: 0 10 355 20 15")
        else:
            print("  예: 10 355 20 0 null null null null 15")
        print("  status")
        print("  help")
        print("  q")
        print()

    def run_cli(self) -> None:
        required_count = len(self.motor_ids) + 1

        while rclpy.ok():
            try:
                command = input("arm> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not command:
                continue
            if command.lower() in {"q", "quit", "exit"}:
                break
            if command.lower() == "help":
                self.print_help()
                continue
            if command.lower() == "status":
                self.status()
                continue

            parts = command.split()
            if len(parts) != required_count:
                print(
                    f"[arm] 각도 {len(self.motor_ids)}개와 속도 1개를 "
                    "입력해야 합니다."
                )
                continue

            try:
                angles = [self._parse_angle(value) for value in parts[:-1]]
                speed = float(parts[-1])
                requested = dict(zip(self.motor_ids, angles))
                self.move(requested, speed)
            except (ValueError, RuntimeError, TimeoutError) as exc:
                print(f"[arm] ERROR: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = ArmCLI()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.connect()
        node.run_cli()
    except KeyboardInterrupt:
        print("\n[arm] 종료")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
