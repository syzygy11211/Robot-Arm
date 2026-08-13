#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from iroi_interfaces.action import MoveJoint


class ArmCLI(Node):
    def __init__(self):
        super().__init__("arm_cli")

        # 현재 테스트 중인 motor_control_node의 Action 주소
        self.action_name = "/test_arm/move_to"

        # ActionClient를 프로그램 시작할 때 딱 한 번 생성한다.
        self.client = ActionClient(
            self,
            MoveJoint,
            self.action_name,
        )

    def connect(self):
        print(f"[arm] {self.action_name} 연결 기다리는 중...")

        # 서버를 매 명령마다 찾지 않고 여기서 한 번만 연결한다.
        self.client.wait_for_server()

        print("[arm] 연결 완료.")
        print()
        print("사용법:")
        print("  각도 속도")
        print()
        print("예:")
        print("  10 5     → 10도, 5 deg/s")
        print("  -20 30   → -20도, 30 deg/s")
        print("  0 10     → 0도로 복귀")
        print("  q        → 종료")
        print()

    def move(self, angle, speed):
        goal = MoveJoint.Goal()

        # 현재는 모터 1개 테스트이므로 배열에 값 하나씩 넣는다.
        goal.target_angles = [float(angle)]
        goal.max_speeds = [float(speed)]

        # 이미 연결되어 있는 ActionClient를 그대로 재사용한다.
        send_future = self.client.send_goal_async(goal)

        # goal이 서버에 전달될 때까지만 기다린다.
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if goal_handle is None:
            print("[arm] Goal 전송 실패")
            return

        if not goal_handle.accepted:
            print("[arm] Goal 거절됨")
            return

        print(
            f"[arm] 명령 전송 완료 → "
            f"target={angle:.2f}°, speed={speed:.2f}°/s"
        )

        # 이동 완료 결과까지 기다린다.
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result.success:
            print("[arm] 이동 완료")
        else:
            print(
                f"[arm] 이동 실패: "
                f"{result.error_message}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = ArmCLI()

    try:
        node.connect()

        while rclpy.ok():
            command = input("arm> ").strip()

            if not command:
                continue

            if command.lower() in ["q", "quit", "exit"]:
                break

            parts = command.split()

            if len(parts) != 2:
                print("[arm] 형식: 각도 속도")
                print("예: 10 5")
                continue

            try:
                angle = float(parts[0])
                speed = float(parts[1])
            except ValueError:
                print("[arm] 숫자로 입력해야 함. 예: 10 5")
                continue

            if speed <= 0:
                print("[arm] 속도는 0보다 커야 함.")
                continue

            node.move(angle, speed)

    except KeyboardInterrupt:
        print("\n[arm] 종료")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()