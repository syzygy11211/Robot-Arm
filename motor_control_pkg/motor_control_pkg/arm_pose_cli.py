#!/usr/bin/env python3
"""Persistent pose/teach CLI for the iROI 8-motor dual-arm system.

Motor topology is fixed as follows:
  right_arm: IDs 1, 2, 3, 4
  left_arm:  IDs 5, 6, 7, 8

``mode`` selects which installed arm endpoints are active: right, left, or dual.
Pose records always keep IDs 1..8; motors outside the active mode remain null.
"""

from __future__ import annotations

import math
import os
import shlex
import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterable, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from iroi_interfaces.action import MoveJoint
from motor_control_pkg.pose_manager import ALL_MOTOR_IDS, PoseManager


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


class ArmPoseCLI(Node):
    def __init__(self):
        super().__init__("arm_pose_cli")

        self.declare_parameter("mode", "right")
        self.declare_parameter("pose_path", "~/.ros/arm_poses.json")
        self.declare_parameter("max_speed_dps", 60.0)
        self.declare_parameter("default_pose_speed_dps", 20.0)
        self.declare_parameter("state_freshness_sec", 2.0)
        self.declare_parameter("teach_save_timeout_sec", 5.0)

        self.mode = str(self.get_parameter("mode").value).strip().lower()
        if self.mode not in MODE_SPECS:
            raise ValueError("mode는 right/left/dual 중 하나여야 합니다.")

        self.specs = MODE_SPECS[self.mode]
        self.pose_path = os.path.abspath(
            os.path.expanduser(str(self.get_parameter("pose_path").value))
        )
        self.max_speed_dps = float(self.get_parameter("max_speed_dps").value)
        self.default_pose_speed_dps = float(
            self.get_parameter("default_pose_speed_dps").value
        )
        self.state_freshness_sec = float(
            self.get_parameter("state_freshness_sec").value
        )
        self.teach_save_timeout_sec = float(
            self.get_parameter("teach_save_timeout_sec").value
        )
        if self.state_freshness_sec <= 0.0:
            raise ValueError("state_freshness_sec는 0보다 커야 합니다.")
        if self.teach_save_timeout_sec <= 0.0:
            raise ValueError("teach_save_timeout_sec는 0보다 커야 합니다.")

        self.pose_manager = PoseManager(self.pose_path)

        self._angles_lock = threading.Lock()
        self.current_angles: Dict[int, float] = {}
        self.last_state_time: Dict[str, float] = {}
        self.teach_state: Dict[str, bool] = {spec.key: False for spec in self.specs}

        self.action_clients: Dict[str, ActionClient] = {}
        self.torque_clients = {}
        self.teach_clients = {}

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
            self.torque_clients[spec.key] = self.create_client(
                SetBool,
                f"{spec.namespace}/torque",
            )
            self.teach_clients[spec.key] = self.create_client(
                SetBool,
                f"{spec.namespace}/teach",
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

            with self._angles_lock:
                for mid, rad in zip(spec.motor_ids, msg.position):
                    self.current_angles[mid] = math.degrees(float(rad))
                self.last_state_time[spec.key] = time.time()

        return callback

    @staticmethod
    def _display_angle(value: float) -> float:
        """연속 관절 좌표를 CLI용 0~360도 표기로 변환한다.

        Pose JSON에는 연속 좌표를 그대로 보관해 상위 제어와 경로 선택에 필요한
        정보를 잃지 않는다.
        """
        return float(value) % 360.0

    @staticmethod
    def _wait_future(future, timeout_sec: float = 15.0):
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.time() > deadline:
                raise TimeoutError("ROS 응답 대기 timeout")
            time.sleep(0.02)
        return future.result()

    def connect(self) -> None:
        print(f"[arm] pose CLI mode={self.mode}")
        print(f"[arm] pose file: {self.pose_path}")
        print("[arm] Action/Service 연결 확인 중...")

        for spec in self.specs:
            if not self.action_clients[spec.key].wait_for_server(timeout_sec=5.0):
                raise RuntimeError(f"{spec.namespace}/move_to Action이 없습니다.")
            if not self.torque_clients[spec.key].wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"{spec.namespace}/torque Service가 없습니다.")
            if not self.teach_clients[spec.key].wait_for_service(timeout_sec=5.0):
                raise RuntimeError(
                    f"{spec.namespace}/teach Service가 없습니다. "
                    "motor_control_node pose-framework patch를 먼저 적용하세요."
                )

        print("[arm] 연결 완료.")
        print()
        self.print_help()

    def _selected_specs(self, target: str) -> list[ArmSpec]:
        target = target.lower()
        if target in {"all", "active"}:
            return list(self.specs)

        aliases = {spec.key: spec for spec in self.specs}

        if target not in aliases:
            valid = ", ".join(sorted(set(aliases) | {"all"}))
            raise ValueError(f"대상은 {valid} 중 하나여야 합니다.")
        return [aliases[target]]

    def _snapshot_angles(self) -> Dict[int, Optional[float]]:
        with self._angles_lock:
            return {
                mid: self.current_angles.get(mid)
                for mid in ALL_MOTOR_IDS
            }

    def _require_current_for(self, spec: ArmSpec) -> Dict[int, float]:
        with self._angles_lock:
            missing = [mid for mid in spec.motor_ids if mid not in self.current_angles]
            if missing:
                raise RuntimeError(
                    f"현재 각도를 아직 받지 못한 motor: {missing}. "
                    "joint_states 수신 후 다시 실행하세요."
                )
            state_time = self.last_state_time.get(spec.key)
            age = None if state_time is None else time.time() - state_time
            if age is None or age > self.state_freshness_sec:
                age_text = "없음" if age is None else f"{age:.2f}초"
                raise RuntimeError(
                    f"[{spec.key}] joint_states가 오래되었습니다(age={age_text}). "
                    "실행 중인 motor_control_node 연결을 확인하세요."
                )
            return {mid: self.current_angles[mid] for mid in spec.motor_ids}

    def _active_snapshot_angles(self) -> Dict[int, Optional[float]]:
        """활성 팔은 최신 실측값, 비활성 팔은 null인 8축 snapshot을 만든다."""
        snapshot: Dict[int, Optional[float]] = {
            mid: None for mid in ALL_MOTOR_IDS
        }
        for spec in self.specs:
            snapshot.update(self._require_current_for(spec))
        return snapshot

    def _wait_for_fresh_states(
        self,
        specs: Iterable[ArmSpec],
        after: Dict[str, float],
    ) -> None:
        """HOLD 전환 이후 각 활성 팔에서 새 joint_states가 올 때까지 기다린다."""
        specs = list(specs)
        deadline = time.time() + self.teach_save_timeout_sec
        while rclpy.ok() and time.time() < deadline:
            with self._angles_lock:
                ready = all(
                    self.last_state_time.get(spec.key, 0.0)
                    > after.get(spec.key, 0.0)
                    and all(mid in self.current_angles for mid in spec.motor_ids)
                    for spec in specs
                )
            if ready:
                return
            time.sleep(0.02)
        raise TimeoutError(
            "teach OFF/HOLD 이후 새로운 joint_states를 받지 못했습니다. "
            "Pose는 저장하지 않았습니다."
        )

    def _validate_speed(self, speed: float) -> float:
        speed = float(speed)
        if speed <= 0.0:
            raise ValueError("speed는 0보다 커야 합니다.")
        if speed > self.max_speed_dps:
            raise ValueError(
                f"speed는 현재 CLI 상한 {self.max_speed_dps:.1f} deg/s 이하여야 합니다."
            )
        return speed

    def _send_arm_goal(
        self,
        spec: ArmSpec,
        targets: Dict[int, float],
        speed: float,
    ):
        goal = MoveJoint.Goal()
        goal.target_angles = [float(targets[mid]) for mid in spec.motor_ids]
        goal.max_speeds = [float(speed)] * len(spec.motor_ids)

        send_future = self.action_clients[spec.key].send_goal_async(goal)
        goal_handle = self._wait_future(send_future, timeout_sec=10.0)
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"[{spec.key}] MoveJoint goal 거절")
        return goal_handle

    def _wait_goal_result(self, spec: ArmSpec, goal_handle) -> None:
        result_future = goal_handle.get_result_async()
        response = self._wait_future(result_future, timeout_sec=120.0)
        if response is None:
            raise RuntimeError(f"[{spec.key}] MoveJoint 결과 없음")
        result = response.result
        if not result.success:
            raise RuntimeError(f"[{spec.key}] 이동 실패: {result.error_message}")

    def move_pose(self, pose_id: int, speed: float) -> None:
        speed = self._validate_speed(speed)
        pose = self.pose_manager.get_pose(pose_id)
        pose_angles = pose["angles"]

        # 모든 활성 팔의 최신 상태와 target을 먼저 검증한다. 이 단계에서는
        # Action을 전송하지 않아, 두 팔 중 하나의 입력 오류로 부분 이동하지 않는다.
        plans = []
        skipped = []
        for spec in self.specs:
            current = self._require_current_for(spec)
            targets: Dict[int, float] = {}
            has_real_target = False

            for mid in spec.motor_ids:
                stored = pose_angles[str(mid)]
                if stored is None:
                    # null means: do not move this motor. The existing MoveJoint
                    # Action requires one target per configured motor, so hold it
                    # at its current calibrated angle.
                    targets[mid] = current[mid]
                    skipped.append(mid)
                else:
                    targets[mid] = float(stored)
                    has_real_target = True

            if not has_real_target:
                continue

            plans.append((spec, targets))

        if not plans:
            print(f"[arm] pose {pose_id}: 이동할 값이 없습니다. (모두 null)")
            return

        # 입력과 현재 상태를 전부 검증한 뒤에만 Action을 전송한다.
        goal_handles = [
            (spec, self._send_arm_goal(spec, targets, speed))
            for spec, targets in plans
        ]

        print(
            f"[arm] pose {pose_id} '{pose['name']}' 이동 시작, "
            f"speed={speed:.1f} deg/s"
        )
        if skipped:
            print(f"[arm] null → 현재 위치 유지: ID {sorted(set(skipped))}")

        for spec, handle in goal_handles:
            self._wait_goal_result(spec, handle)
        print(f"[arm] pose {pose_id} 이동 완료")

    def move_sequence(self, pose_ids: Iterable[int], speed: float) -> None:
        pose_ids = [int(pose_id) for pose_id in pose_ids]

        if not pose_ids:
            raise ValueError("sequence에는 최소 1개의 pose ID가 필요합니다.")

        speed = self._validate_speed(speed)

        # Validate every pose before motion so a missing ID cannot leave the
        # sequence partially executed.
        for pose_id in pose_ids:
            self.pose_manager.get_pose(pose_id)

        print(
            f"[arm] sequence 시작: {pose_ids}, "
            f"speed={speed:.1f} deg/s"
        )

        total = len(pose_ids)

        for index, pose_id in enumerate(pose_ids, start=1):
            print(
                f"[arm] sequence {index}/{total}:"
                f" pose {pose_id} 실행"
            )

            self.move_pose(pose_id, speed)

        print("[arm] sequence 완료")

    def save_pose(self, pose_id: int, name: Optional[str]) -> None:
        snapshot = self._active_snapshot_angles()
        saved = self.pose_manager.save_pose(pose_id, snapshot, name=name)
        print(f"[arm] pose {pose_id} '{saved['name']}' 저장 완료")
        for mid in ALL_MOTOR_IDS:
            value = saved["angles"][str(mid)]
            if value is None:
                print(f"      ID {mid}: NULL")
            else:
                print(f"      ID {mid}: {self._display_angle(value):.3f}°")

    def teach_save_pose(self, pose_id: int, name: Optional[str]) -> None:
        """활성 팔을 Teach OFF/HOLD한 뒤 새 실측 상태를 Pose로 저장한다."""
        specs = list(self.specs)
        print("[arm] teach-save: 활성 팔 Teach OFF + 현재 위치 HOLD 요청")
        self.set_teach("all", enabled=False)
        # Service 성공 응답보다 뒤에 발행된 상태만 저장해야 HOLD 이전 값이
        # 섞이지 않는다.
        with self._angles_lock:
            after_hold = {
                spec.key: self.last_state_time.get(spec.key, 0.0)
                for spec in specs
            }
        self._wait_for_fresh_states(specs, after=after_hold)
        self.save_pose(pose_id, name)

    def _call_set_bool(self, client, value: bool, label: str) -> None:
        req = SetBool.Request()
        req.data = bool(value)
        future = client.call_async(req)
        response = self._wait_future(future, timeout_sec=15.0)
        if response is None or not response.success:
            message = "응답 없음" if response is None else response.message
            raise RuntimeError(f"{label} 실패: {message}")

    def set_teach(self, target: str, enabled: bool) -> None:
        specs = self._selected_specs(target)
        for spec in specs:
            self._call_set_bool(
                self.teach_clients[spec.key],
                enabled,
                f"[{spec.key}] teach {'ON' if enabled else 'OFF'}",
            )
            self.teach_state[spec.key] = enabled
            print(
                f"[arm] {spec.key}: TEACH {'ON' if enabled else 'OFF'} "
                f"(torque {'OFF' if enabled else 'ON + current-position hold'})"
            )

    def set_raw_torque(self, target: str, enabled: bool) -> None:
        specs = self._selected_specs(target)
        for spec in specs:
            self._call_set_bool(
                self.torque_clients[spec.key],
                enabled,
                f"[{spec.key}] torque {'ON' if enabled else 'OFF'}",
            )
            print(f"[arm] {spec.key}: torque {'ON' if enabled else 'OFF'}")

    def list_poses(self) -> None:
        rows = self.pose_manager.list_poses()
        print("POSES")
        for row in rows:
            print(
                f"  {row['pose_id']:>3}  {row['name']:<20} "
                f"{row['measured']}/{row['total']} values"
            )

    def show_pose(self, pose_id: int) -> None:
        pose = self.pose_manager.get_pose(pose_id)
        print(f"Pose {pose_id}: {pose['name']}")
        for mid in ALL_MOTOR_IDS:
            value = pose["angles"][str(mid)]
            text = "NULL" if value is None else f"{self._display_angle(value):.3f}°"
            print(f"  ID {mid}: {text}")

    def status(self) -> None:
        snapshot = self._snapshot_angles()
        print("=" * 44)
        print(" iROI ARM STATUS")
        print("=" * 44)
        for mid in ALL_MOTOR_IDS:
            value = snapshot[mid]
            text = "NULL" if value is None else f"{self._display_angle(value):.3f}°"
            print(f" ID {mid}: {text}")
        print("-" * 44)
        for spec in self.specs:
            state = "ON" if self.teach_state[spec.key] else "OFF"
            age = None
            if spec.key in self.last_state_time:
                age = max(0.0, time.time() - self.last_state_time[spec.key])
            age_text = "no state" if age is None else f"state age={age:.2f}s"
            print(f" {spec.key:>5}: teach={state}, {age_text}")
        print(f" pose file: {self.pose_path}")
        print("=" * 44)

    @staticmethod
    def print_help() -> None:
        print("Commands:")
        print("  pose <ID> [speed]       저장 pose로 이동")
        print("  sequence <ID...>        여러 pose를 입력 순서대로 실행")
        print("  save <ID> [name]        활성 팔 최신값 저장; 비활성 ID는 NULL")
        print("  teach-save <ID> [name]  Teach OFF/HOLD 후 새 상태를 저장")
        print("  list                    pose 목록")
        print("  show <ID>               pose 상세")
        print("  delete <ID>             pose 삭제 (pose 0 삭제 불가)")
        print("  teach <target> on       teach mode ON = torque OFF")
        print("  teach <target> off      현재 위치 기준으로 torque ON + HOLD")
        print("  torque <target> on/off  raw torque 명령 (teach보다 저수준)")
        print("  status                  현재 각도/teach 상태")
        print("  help")
        print("  q | quit | exit")
        print()
        print("mode: right(ID 1-4), left(ID 5-8), dual(ID 1-8)")
        print("target: right/left/active/all (mode에 따라 사용 가능)")
        print("예: pose 0 20")
        print("    sequence 0 1 2 3 2 1 0")
        print("    save 1 wave")
        print("    teach-save 0 attention")
        print("    teach active on")
        print("    teach active off")
        print()

    def run_cli(self) -> None:
        while rclpy.ok():
            try:
                raw = input("arm> ").strip()
            except EOFError:
                break
            if not raw:
                continue

            try:
                parts = shlex.split(raw)
            except ValueError as exc:
                print(f"[arm] 입력 파싱 실패: {exc}")
                continue

            cmd = parts[0].lower()
            if cmd in {"q", "quit", "exit"}:
                break

            try:
                if cmd == "help":
                    self.print_help()

                elif cmd == "list":
                    self.list_poses()

                elif cmd == "show" and len(parts) == 2:
                    self.show_pose(int(parts[1]))

                elif cmd == "delete" and len(parts) == 2:
                    pose_id = int(parts[1])
                    self.pose_manager.delete_pose(pose_id)
                    print(f"[arm] pose {pose_id} 삭제 완료")

                elif cmd == "save" and len(parts) >= 2:
                    pose_id = int(parts[1])
                    name = " ".join(parts[2:]) if len(parts) > 2 else None
                    self.save_pose(pose_id, name)

                elif cmd == "teach-save" and len(parts) >= 2:
                    pose_id = int(parts[1])
                    name = " ".join(parts[2:]) if len(parts) > 2 else None
                    self.teach_save_pose(pose_id, name)

                elif cmd == "pose" and len(parts) in {2, 3}:
                    pose_id = int(parts[1])
                    speed = (
                        float(parts[2])
                        if len(parts) == 3
                        else self.default_pose_speed_dps
                    )
                    self.move_pose(pose_id, speed)

                elif cmd == "sequence" and len(parts) >= 2:
                    pose_ids = [int(value) for value in parts[1:]]
                    self.move_sequence(pose_ids, self.default_pose_speed_dps)

                elif cmd == "teach" and len(parts) == 3:
                    value = parts[2].lower()
                    if value not in {"on", "off"}:
                        raise ValueError("teach 값은 on/off")
                    self.set_teach(parts[1], enabled=(value == "on"))

                elif cmd == "torque" and len(parts) == 3:
                    value = parts[2].lower()
                    if value not in {"on", "off"}:
                        raise ValueError("torque 값은 on/off")
                    self.set_raw_torque(parts[1], enabled=(value == "on"))

                elif cmd == "status" and len(parts) == 1:
                    self.status()

                else:
                    print("[arm] 명령 형식이 맞지 않습니다. 'help'를 입력하세요.")

            except (ValueError, KeyError, RuntimeError, TimeoutError) as exc:
                print(f"[arm] ERROR: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = ArmPoseCLI()
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
