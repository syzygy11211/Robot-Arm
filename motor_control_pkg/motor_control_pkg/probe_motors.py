#!/usr/bin/env python3
"""여러 LK 모터의 정보를 읽기만 하는 진단용 CLI.

사용하는 명령:
- 0x12: 모터 정보
- 0x94: 절대 엔코더 각도
- 0x90: 엔코더 원시값
- 0x92: 현재 세션 다회전 각도

motor_on/off 및 이동 명령은 전송하지 않는다.
"""

import argparse
import sys

import serial

from motor_control_pkg.lk_motor import LKMotor


def read_and_print(label, callback):
    """읽기 하나가 실패해도 다음 항목을 계속 검사한다."""
    try:
        value = callback()
        print(f'  {label:<14}: {value}')
        return value
    except Exception as exc:
        print(f'  {label:<14}: FAIL ({exc})')
        return None


def main():
    parser = argparse.ArgumentParser(
        description='LK 모터의 info/0x94/0x90/0x92를 읽습니다.'
    )
    parser.add_argument(
        '--port',
        required=True,
        help='RS485 serial port. 예: /dev/ttyUSB0',
    )
    parser.add_argument(
        '--baudrate',
        type=int,
        default=115200,
        help='통신 속도. 기본값: 115200',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=0.2,
        help='serial read timeout(초). 기본값: 0.2',
    )
    parser.add_argument(
        '--ids',
        type=int,
        nargs='+',
        required=True,
        help='읽을 모터 ID 목록. 예: --ids 5 6 7 8',
    )
    args = parser.parse_args()

    print(
        f'포트 {args.port}, baudrate={args.baudrate}, '
        f'IDs={args.ids} 읽기 시작'
    )
    print('※ 읽기 전용입니다. 모터 이동 명령은 전송하지 않습니다.')

    try:
        bus = serial.Serial(
            args.port,
            args.baudrate,
            timeout=args.timeout,
        )
    except Exception as exc:
        print(f'포트를 열 수 없습니다: {exc}', file=sys.stderr)
        return 1

    try:
        for motor_id in args.ids:
            print()
            print(f'===== Motor ID {motor_id} =====')

            motor = LKMotor(
                bus,
                motor_id=motor_id,
            )

            read_and_print('info (0x12)', motor.read_info)
            read_and_print('angle (0x94)', motor.read_single_angle)
            read_and_print('encoder (0x90)', motor.read_encoder)
            read_and_print('multi (0x92)', motor.read_multi_angle)

    finally:
        bus.close()

    print()
    print('읽기 완료. 파일과 모터 설정은 변경하지 않았습니다.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
