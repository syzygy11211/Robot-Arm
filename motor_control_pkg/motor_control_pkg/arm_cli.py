#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from iroi_interfaces.action import MoveJoint


# ============================================================
# 현재 3모터 테스트 구성
#
# motor_control_node의 motor_ids 순서와 반드시 같아야 한다.
#
# [1, 2, 4]
#  ↑  ↑  ↑
#  │  │  └─ MG5010E-i10
#  │  └──── MG4010E-i10
#  └─────── MG4010E-i10
# ============================================================

MOTOR_IDS = [1, 2, 4]

# 현재 launch 파일에서 설정한 최대 속도
MAX_SPEED_DPS = 60.0


class ArmCLI(Node):
    def __init__(self):
        super().__init__("arm_cli")

        # 현재 3축 motor_control_node의 Action 주소
        self.action_name = "/test_arm/move_to"

        # 프로그램 시작 시 ActionClient를 한 번만 생성
        self.client = ActionClient(
            self,
            MoveJoint,
            self.action_name,
        )

    def connect(self):
        print(f"[arm] {self.action_name} 연결 기다리는 중...")

        self.client.wait_for_server()

        print("[arm] 연결 완료.")
        print()
        print("현재 모터 순서:")
        print("  1번째 각도 → ID 1  (MG4010E-i10)")
        print("  2번째 각도 → ID 2  (MG4010E-i10)")
        print("  3번째 각도 → ID 4  (MG5010E-i10)")
        print()
        print("사용법:")
        print("  ID1각도 ID2각도 ID4각도 속도")
        print()
        print("예:")
        print("  10 0 0 10")
        print("      → ID1=10°, ID2=0°, ID4=0°, 속도=10 deg/s")
        print()
        print("  0 10 0 10")
        print("      → ID1=0°, ID2=10°, ID4=0°, 속도=10 deg/s")
        print()
        print("  0 0 10 10")
        print("      → ID1=0°, ID2=0°, ID4=10°, 속도=10 deg/s")
        print()
        print("  10 -10 20 20")
        print("      → 세 모터 동시 이동")
        print()
        print("  0 0 0 10")
        print("      → 세 모터 모두 영점으로 복귀")
        print()
        print("  q")
        print("      → CLI 종료")
        print()

    def move(self, angle_id1, angle_id2, angle_id4, speed):
        """
        motor_ids = [1, 2, 4] 순서로 목표각을 전송한다.

        각도는 각 관절의 영점 기준 출력축 각도이다.
        """

        goal = MoveJoint.Goal()

        # launch의 motor_ids=[1,2,4]와 같은 순서
        goal.target_angles = [
            float(angle_id1),
            float(angle_id2),
            float(angle_id4),
        ]

        # 현재 CLI에서는 세 모터에 동일한 최대 속도를 요청
        goal.max_speeds = [
            float(speed),
            float(speed),
            float(speed),
        ]

        send_future = self.client.send_goal_async(goal)

        # goal이 서버에 전달될 때까지만 기다림
        rclpy.spin_until_future_complete(
            self,
            send_future,
        )

        goal_handle = send_future.result()

        if goal_handle is None:
            print("[arm] Goal 전송 실패")
            return

        if not goal_handle.accepted:
            print("[arm] Goal 거절됨")
            return

        print()
        print("[arm] 명령 전송 완료")
        print(
            f"      ID1={angle_id1:.2f}°, "
            f"ID2={angle_id2:.2f}°, "
            f"ID4={angle_id4:.2f}°"
        )
        print(
            f"      speed={speed:.2f}°/s"
        )

        # 이동 완료 결과까지 대기
        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        result_response = result_future.result()

        if result_response is None:
            print("[arm] 결과 수신 실패")
            return

        result = result_response.result

        if result.success:
            print("[arm] 이동 완료")
        else:
            print(
                f"[arm] 이동 실패: "
                f"{result.error_message}"
            )

    def run_cli(self):
        while rclpy.ok():

            command = input("arm> ").strip()

            if not command:
                continue

            if command.lower() in [
                "q",
                "quit",
                "exit",
            ]:
                break

            parts = command.split()

            # 각도 3개 + 속도 1개 = 총 4개
            if len(parts) != 4:
                print()
                print("[arm] 형식:")
                print("      ID1각도 ID2각도 ID4각도 속도")
                print()
                print("예:")
                print("      10 0 0 10")
                print()
                continue

            try:
                angle_id1 = float(parts[0])
                angle_id2 = float(parts[1])
                angle_id4 = float(parts[2])
                speed = float(parts[3])

            except ValueError:
                print(
                    "[arm] 전부 숫자로 입력해야 함. "
                    "예: 10 0 0 10"
                )
                continue

            if speed <= 0:
                print("[arm] 속도는 0보다 커야 함.")
                continue

            if speed > MAX_SPEED_DPS:
                print(
                    f"[arm] 현재 최대 속도는 "
                    f"{MAX_SPEED_DPS:.1f} deg/s 입니다."
                )
                continue

            self.move(
                angle_id1,
                angle_id2,
                angle_id4,
                speed,
            )


def main(args=None):
    rclpy.init(args=args)

    node = ArmCLI()

    try:
        node.connect()
        node.run_cli()

    except KeyboardInterrupt:
        print("\n[arm] 종료")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
