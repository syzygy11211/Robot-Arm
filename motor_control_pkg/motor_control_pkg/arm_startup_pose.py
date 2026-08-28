#!/usr/bin/env python3
"""One-shot startup pose mover.

Expected startup sequence:
1) each motor_control_node starts with startup_mode=reference_only
2) node reconstructs calibrated 0x92 reference and applies current-position HOLD
3) this process waits for fresh joint_states, then reads pose 0
4) only arms with non-null pose values receive MoveJoint goals

In development, allow_partial_pose=True lets null entries hold their current angle.
For the final 8-motor robot, set allow_partial_pose=False so startup is rejected if
any required pose-0 value is still null.
"""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Dict

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from iroi_interfaces.action import MoveJoint
from motor_control_pkg.pose_manager import PoseManager


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


class StartupPoseNode(Node):
    def __init__(self):
        super().__init__("arm_startup_pose")
        self.declare_parameter("mode", "right")
        self.declare_parameter("pose_path", "~/.ros/arm_poses.json")
        self.declare_parameter("startup_pose_id", 0)
        self.declare_parameter("startup_speed_dps", 20.0)
        self.declare_parameter("allow_partial_pose", True)
        self.declare_parameter("state_timeout_sec", 10.0)
        self.declare_parameter("state_freshness_sec", 2.0)

        self.mode = str(self.get_parameter("mode").value).strip().lower()
        if self.mode not in MODE_SPECS:
            raise ValueError("mode는 right/left/dual 중 하나여야 합니다.")
        self.specs = MODE_SPECS[self.mode]

        pose_path = str(self.get_parameter("pose_path").value)
        self.pose_manager = PoseManager(os.path.expanduser(pose_path))
        self.pose_id = int(self.get_parameter("startup_pose_id").value)
        self.speed = float(self.get_parameter("startup_speed_dps").value)
        self.allow_partial = bool(self.get_parameter("allow_partial_pose").value)
        self.state_timeout = float(self.get_parameter("state_timeout_sec").value)
        self.state_freshness = float(
            self.get_parameter("state_freshness_sec").value
        )
        if self.speed <= 0.0:
            raise ValueError("startup_speed_dps는 0보다 커야 합니다.")
        if self.state_timeout <= 0.0:
            raise ValueError("state_timeout_sec는 0보다 커야 합니다.")
        if self.state_freshness <= 0.0:
            raise ValueError("state_freshness_sec는 0보다 커야 합니다.")

        self._lock = threading.Lock()
        self.current: Dict[int, float] = {}
        self.last_state_time: Dict[str, float] = {}
        self.action_clients = {}

        for spec in self.specs:
            self.create_subscription(
                JointState,
                f"{spec.namespace}/joint_states",
                self._make_callback(spec),
                10,
            )
            self.action_clients[spec.key] = ActionClient(
                self, MoveJoint, f"{spec.namespace}/move_to"
            )

    def _make_callback(self, spec: ArmSpec):
        def cb(msg: JointState):
            if len(msg.position) < len(spec.motor_ids):
                return
            with self._lock:
                for mid, rad in zip(spec.motor_ids, msg.position):
                    self.current[mid] = math.degrees(float(rad))
                self.last_state_time[spec.key] = time.time()
        return cb

    @staticmethod
    def _wait_future(future, timeout_sec: float):
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.time() > deadline:
                raise TimeoutError("future timeout")
            time.sleep(0.02)
        return future.result()

    def _wait_for_state(self) -> None:
        required = {mid for spec in self.specs for mid in spec.motor_ids}
        deadline = time.time() + self.state_timeout
        missing = sorted(required)
        stale = [spec.key for spec in self.specs]
        while time.time() < deadline:
            with self._lock:
                missing = sorted(required - set(self.current))
                now = time.time()
                stale = [
                    spec.key
                    for spec in self.specs
                    if now - self.last_state_time.get(spec.key, 0.0)
                    > self.state_freshness
                ]
            if not missing and not stale:
                self.get_logger().info(
                    "reference_only + 현재 위치 HOLD 이후 최신 joint_states 확인 완료"
                )
                return
            time.sleep(0.05)
        raise RuntimeError(
            f"startup joint_states 준비 실패: missing_motor={missing}, "
            f"stale_arm={stale}"
        )

    def run_once(self) -> None:
        pose = self.pose_manager.get_pose(self.pose_id)
        self.get_logger().info(
            f"startup mode={self.mode}, pose {self.pose_id} '{pose['name']}', "
            f"allow_partial={self.allow_partial}"
        )

        for spec in self.specs:
            if not self.action_clients[spec.key].wait_for_server(timeout_sec=10.0):
                raise RuntimeError(f"{spec.namespace}/move_to Action 없음")

        self._wait_for_state()

        plans = []
        null_ids = []
        for spec in self.specs:
            with self._lock:
                current = {mid: self.current[mid] for mid in spec.motor_ids}

            targets = []
            has_non_null = False
            for mid in spec.motor_ids:
                stored = pose["angles"][str(mid)]
                if stored is None:
                    null_ids.append(mid)
                    if not self.allow_partial:
                        raise RuntimeError(
                            f"startup pose {self.pose_id}의 ID {mid} 값이 null입니다. "
                            "최종 운용에서는 8축 pose 0을 모두 저장해야 합니다."
                        )
                    targets.append(float(current[mid]))
                else:
                    targets.append(float(stored))
                    has_non_null = True

            # If every target for an arm is null, no movement is necessary.
            if not has_non_null:
                self.get_logger().warn(
                    f"[{spec.key}] startup pose 값이 모두 null -> 현재 위치 유지"
                )
                continue

            plans.append((spec, targets))

        # 모든 활성 팔의 null/현재 상태 검사가 끝난 뒤에만 Action을 전송한다.
        handles = []
        for spec, targets in plans:
            goal = MoveJoint.Goal()
            goal.target_angles = targets
            goal.max_speeds = [self.speed] * len(spec.motor_ids)
            send_future = self.action_clients[spec.key].send_goal_async(goal)
            handle = self._wait_future(send_future, 10.0)
            if handle is None or not handle.accepted:
                raise RuntimeError(f"[{spec.key}] startup pose goal 거절")
            handles.append((spec, handle))

        if null_ids:
            self.get_logger().warn(
                f"startup pose null IDs={sorted(set(null_ids))}; 개발 모드에서는 현재 위치 유지"
            )

        for spec, handle in handles:
            response = self._wait_future(handle.get_result_async(), 120.0)
            if response is None or not response.result.success:
                error = "결과 없음" if response is None else response.result.error_message
                raise RuntimeError(f"[{spec.key}] startup pose 실패: {error}")

        if handles:
            self.get_logger().info(f"startup pose {self.pose_id} 이동 완료")
        else:
            self.get_logger().info("startup pose: 이동할 값 없음, reference sync 상태로 READY")

        self.get_logger().info("READY / MoveJoint Action 사용 가능")


def main(args=None):
    rclpy.init(args=args)
    node = StartupPoseNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    exit_code = 0
    try:
        node.run_once()
    except Exception as exc:
        node.get_logger().error(str(exc))
        exit_code = 1
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)

    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
