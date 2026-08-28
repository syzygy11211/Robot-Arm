#!/usr/bin/env python3
"""저장된 0x94 절대 영점까지의 homing 이동량을 '읽기만' 해서 확인하는 도구.

모터를 움직이지 않는다.
"""

import argparse
import json
import os

import serial
from ament_index_python.packages import get_package_share_directory

from motor_control_pkg.lk_motor import LKMotor


def shortest_delta(target, current, period):
    half = period / 2.0
    return (target - current + half) % period - half


def default_config_path():
    share = get_package_share_directory('motor_control_pkg')
    return os.path.join(share, 'config', 'zero_config_i10_verified.json')


def main():
    parser = argparse.ArgumentParser(
        description='저장된 절대 영점까지의 예상 homing 이동량 확인 (모터 이동 없음)'
    )
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--config', default=None)
    parser.add_argument('--id', type=int, required=True, dest='motor_id')
    args = parser.parse_args()

    config_path = os.path.abspath(os.path.expanduser(args.config or default_config_path()))
    with open(config_path, encoding='utf-8') as f:
        config = json.load(f)

    entry = next(
        (m for m in config.get('motors', []) if int(m.get('motor_id', -1)) == args.motor_id),
        None,
    )
    if entry is None:
        raise SystemExit(f'config에 motor_id={args.motor_id}가 없습니다: {config_path}')

    ratio = float(entry.get('ratio', 10.0))
    direction = float(entry.get('direction', 1))
    if direction not in {-1.0, 1.0}:
        raise SystemExit(
            f'motor_id={args.motor_id} direction은 -1 또는 +1이어야 합니다: '
            f'{entry.get("direction")!r}'
        )
    direction = int(direction)
    period = float(entry.get('loop_period_deg', ratio * 360.0))
    zero_single = entry.get('zero_single_deg')
    if zero_single is None:
        raise SystemExit(f'motor_id={args.motor_id}의 zero_single_deg가 없습니다.')
    zero_single = float(zero_single)

    ser = serial.Serial(args.port, 115200, timeout=0.2)
    motor = LKMotor(ser, motor_id=args.motor_id)
    try:
        info = motor.read_info()
        state = motor.read_state1()
        current_single = motor.read_single_angle()
        current_92 = motor.read_multi_angle()
    finally:
        ser.close()

    delta_motor = shortest_delta(zero_single, current_single, period)
    delta_output = direction * delta_motor / ratio
    target_92 = current_92 + delta_motor

    print(f"model          : {info['motor']} (ID {args.motor_id}, SN {info['sn']})")
    print(f"voltage        : {state['voltage_v']:.2f} V")
    print(f"error_state    : {state['error_state']}")
    print(f"direction      : {direction:+d}")
    print(f"current 0x94   : {current_single:.2f} deg")
    print(f"saved zero 0x94: {zero_single:.2f} deg")
    print(f"current 0x92   : {current_92:.2f} deg")
    print(f"homing delta   : {delta_motor:+.2f} motor-deg = {delta_output:+.3f} output-deg")
    print(f"target zero_92 : {target_92:.2f} deg")
    print('※ 읽기 전용 검사 완료. 이동 명령은 전송하지 않았습니다.')


if __name__ == '__main__':
    main()
