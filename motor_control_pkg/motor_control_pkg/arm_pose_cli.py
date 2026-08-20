#!/usr/bin/env python3
"""Persistent pose/teach CLI for the iROI 8-motor arm system.

This tool intentionally lives beside the already hardware-verified ``arm_cli``.
The old CLI remains available for direct ID 1/2/4 bench commands; this CLI adds
pose storage, pose playback, teach mode, and an 8-motor status view.

ROS parameters:
  mode:       "test" (IDs 1,2,4 on /test_arm) or "dual" (IDs 1..8)
  pose_path:  runtime pose DB, default ~/.ros/arm_poses.json
  max_speed_dps: CLI speed ceiling (output-axis deg/s)
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


TEST_SPECS = (
    ArmSpec("test", "/test_arm", (1, 2, 4)),
)

DUAL_SPECS = (
    ArmSpec("left", "/left_arm", (1, 2, 3, 4)),
    ArmSpec("right", "/right_arm", (5, 6, 7, 8)),
)


class ArmPoseCLI(Node):
    def __init__(self):
        super().__init__("arm_pose_cli")

        self.declare_parameter("mode", "test")
        self.declare_parameter("pose_path", "~/.ros/arm_poses.json")
        self.declare_parameter("max_speed_dps", 60.0)
        self.declare_parameter("default_pose_speed_dps", 20.0)

        self.mode = str(self.get_parameter("mode").value).strip().lower()
        if self.mode not in {"test", "dual"}:
            raise ValueError("mode는 'test' 또는 'dual'이어야 합니다.")

        self.specs = TEST_SPECS if self.mode == "test" else DUAL_SPECS
        self.pose_path = os.path.abspath(
            os.path.expanduser(str(self.get_parameter("pose_path").value))
        )
        self.max_speed_dps = float(self.get_parameter("max_speed_dps").value)
        self.default_pose_speed_dps = float(
            self.get_parameter("default_pose_speed_dps").value
        )

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
        if target == "all":
            return list(self.specs)

        aliases = {spec.key: spec for spec in self.specs}
        if self.mode == "test":
            aliases["left"] = self.specs[0]
            aliases["test"] = self.specs[0]

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

    def _require_current_for(self, motor_ids: Iterable[int]) -> Dict[int, float]:
        with self._angles_lock:
            missing = [mid for mid in motor_ids if mid not in self.current_angles]
            if missing:
                raise RuntimeError(
                    f"현재 각도를 아직 받지 못한 motor: {missing}. "
                    "joint_states 수신 후 다시 실행하세요."
                )
            return {mid: self.current_angles[mid] for mid in motor_ids}

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

        goal_handles = []
        skipped = []

        for spec in self.specs:
            current = self._require_current_for(spec.motor_ids)
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

            goal_handles.append((spec, self._send_arm_goal(spec, targets, speed)))

        if not goal_handles:
            print(f"[arm] pose {pose_id}: 이동할 값이 없습니다. (모두 null)")
            return

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
        snapshot = self._snapshot_angles()
        saved = self.pose_manager.save_pose(pose_id, snapshot, name=name)
        print(f"[arm] pose {pose_id} '{saved['name']}' 저장 완료")
        for mid in ALL_MOTOR_IDS:
            value = saved["angles"][str(mid)]
            if value is None:
                print(f"      ID {mid}: NULL")
            else:
                print(f"      ID {mid}: {value:+.3f}°")

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
            print(f"  ID {mid}: {'NULL' if value is None else f'{value:+.3f}°'}")

    def status(self) -> None:
        snapshot = self._snapshot_angles()
        print("=" * 44)
        print(" iROI ARM STATUS")
        print("=" * 44)
        for mid in ALL_MOTOR_IDS:
            value = snapshot[mid]
            print(f" ID {mid}: {'NULL' if value is None else f'{value:+.3f}°'}")
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
        print("  save <ID> [name]        현재 8축 값을 저장; 미수신 ID는 NULL")
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
        print("target: test/left/right/all (mode에 따라 사용 가능)")
        print("예: pose 0 20")
        print("    sequence 0 1 2 3 2 1 0")
        print("    save 1 wave")
        print("    teach left on")
        print("    teach left off")
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
