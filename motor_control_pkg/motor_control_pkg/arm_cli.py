#!/usr/bin/env python3
"""4축 iROI 모터 제어용 대화형 CLI."""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from iroi_interfaces.action import MoveJoint


# motor_control_node의 motor_ids 순서와 동일해야 한다.
# 나중에 8축이 되면 이 목록만 변경하면 된다.
MOTOR_IDS = [1, 2, 3, 4]

MAX_SPEED_DPS = 60.0
ACTION_NAME = "/move_to"


class ArmCLI(Node):
    def __init__(self):
        super().__init__("arm_cli")

        self.client = ActionClient(
            self,
            MoveJoint,
            ACTION_NAME,
        )

    def connect(self):
        print(f"[arm] {ACTION_NAME} 연결을 기다리는 중...")
        self.client.wait_for_server()

        print("[arm] 연결 완료")
        print()
        print(f"모터 순서: {MOTOR_IDS}")
        print()
        print("입력 형식:")
        print("  ID1각도 ID2각도 ID3각도 ID4각도 속도")
        print()
        print("예:")
        print("  5 0 0 0 10    → ID 1을 5도로 이동")
        print("  0 5 0 0 10    → ID 2를 5도로 이동")
        print("  0 0 5 0 10    → ID 3을 5도로 이동")
        print("  0 0 0 5 10    → ID 4를 5도로 이동")
        print("  0 0 0 0 10    → 전부 원점으로 이동")
        print("  q              → 종료")
        print()

    def move(self, angles, speed):
        goal = MoveJoint.Goal()
        goal.target_angles = [float(angle) for angle in angles]
        goal.max_speeds = [float(speed)] * len(MOTOR_IDS)

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if goal_handle is None:
            print("[arm] Goal 전송 실패")
            return

        if not goal_handle.accepted:
            print("[arm] Goal이 거절됐습니다.")
            return

        print()
        print(
            f"[arm] 이동 시작: "
            f"angles={goal.target_angles}, "
            f"speed={speed:.2f} deg/s"
        )

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result_response = result_future.result()

        if result_response is None:
            print("[arm] 이동 결과를 받지 못했습니다.")
            return

        result = result_response.result

        if result.success:
            print("[arm] 이동 완료")
        else:
            print(f"[arm] 이동 실패: {result.error_message}")

        print()

    def run_cli(self):
        required_count = len(MOTOR_IDS) + 1

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

            parts = command.split()

            if len(parts) != required_count:
                print(
                    f"[arm] 각도 {len(MOTOR_IDS)}개와 "
                    f"속도 1개를 입력해야 합니다."
                )
                print("[arm] 예: 5 0 0 0 10")
                continue

            try:
                angles = [
                    float(value)
                    for value in parts[:-1]
                ]
                speed = float(parts[-1])
            except ValueError:
                print("[arm] 모든 값을 숫자로 입력해야 합니다.")
                continue

            if speed <= 0.0:
                print("[arm] 속도는 0보다 커야 합니다.")
                continue

            if speed > MAX_SPEED_DPS:
                print(
                    f"[arm] 최대 속도는 "
                    f"{MAX_SPEED_DPS:.1f} deg/s입니다."
                )
                continue

            self.move(angles, speed)


def main(args=None):
    rclpy.init(args=args)
    node = ArmCLI()

    try:
        node.connect()
        node.run_cli()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
