#!/usr/bin/env python3
"""RS485 버스에 연결된 모터들의 실제 ID를 스캔하는 진단 스크립트.
DIP 스위치 표만 믿지 말고 반드시 이걸로 실측할 것
(DUAL_MOTOR_SUMMARY.md 2단계에서 겪었던 "ID 미스터리" 재발 방지).

사용법:
    ros2 run motor_control_pkg scan_ids --port /dev/ttyUSB0
    ros2 run motor_control_pkg scan_ids --port /dev/ttyUSB0 --start 1 --end 8
"""
import argparse
import time

import serial

from motor_control_pkg.lk_motor import LKMotor, ProtocolError


def main():
    parser = argparse.ArgumentParser(description="RS485 버스의 모터 ID를 스캔한다.")
    parser.add_argument('--port', required=True, help='시리얼 포트 (예: /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--start', type=int, default=1, help='스캔 시작 ID')
    parser.add_argument('--end', type=int, default=32, help='스캔 끝 ID')
    args = parser.parse_args()

    print(f"포트 {args.port} 에서 ID {args.start}~{args.end} 스캔 중...")
    ser = serial.Serial(args.port, args.baudrate, timeout=0.15)

    found = []
    try:
        for mid in range(args.start, args.end + 1):
            motor = LKMotor(ser, motor_id=mid)
            try:
                info = motor.read_info()
                print(f"  ID={mid:2d} 응답함: 모델={info['motor']}, SN={info['sn']}")
                found.append((mid, info['motor'], info['sn']))
            except ProtocolError:
                pass  # 응답 없음 = 이 ID엔 모터 없음, 조용히 다음으로
            time.sleep(0.02)
    finally:
        ser.close()

    print()
    if found:
        print(f"총 {len(found)}개 모터 발견:")
        for mid, model, sn in found:
            print(f"  - ID {mid}: {model} (SN {sn})")
    else:
        print("응답한 모터가 없습니다. 배선/전원/포트를 확인하세요.")


if __name__ == '__main__':
    main()