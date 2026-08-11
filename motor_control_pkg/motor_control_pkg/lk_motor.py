#!/usr/bin/env python3
"""LK-TECH MG5010E-i10 (RS485) 저수준 통신 드라이버.

프로토콜: LK-TECH motor control protocol(RS485) V2.35
CLI(Command Line Interface)/대화형 요소 없음 - motor_control.py 등 상위 제어 코드에서 import 해서 쓴다.
"""

import struct
import time

import serial

HEAD = 0x3E

CMD_READ_INFO = 0x12   # 드라이버/모터 정보
CMD_READ_ENCODER = 0x90   # 엔코더 원시 절대값
CMD_READ_MULTI_ANGLE = 0x92   # 다회전 누적각 (전원 인가 후 누적, 0x92 frame)
CMD_READ_SINGLE_ANGLE = 0x94   # 1회전 절대각 0.01deg/LSB (전원 재인가에도 유지되는 절대 기준)
CMD_READ_STATE2 = 0x9C   # 온도/토크전류/속도/엔코더
CMD_READ_STATE1 = 0x9A   # 온도/전압/모터on-off/에러상태
CMD_MOTOR_OFF = 0x80   # 모터 Off (홀딩 토크 해제, 손으로 자유롭게 돌릴 수 있게 됨)
CMD_MOTOR_ON = 0x88   # 모터 On (Off 상태에서 켬)
CMD_MOTOR_STOP = 0x81   # 정지 (상태는 유지 = 홀딩 토크는 유지된 채로 멈춤, 재명령 가능)
CMD_MULTI_ANGLE_CTRL2 = 0xA4   # 다회전 절대각 이동 (angle:int64, maxSpeed:uint32), 0x92와 같은 frame

# 이동 계열 명령(0xA1/0xA2/0xA4 등) 앞에는 항상 이 순서로 핸드셰이크를 보내야
# 정상적으로 동작하는 것이 기존 검증된 스크립트들에서 공통으로 확인됨.
HANDSHAKE_CMDS = (0x1F, 0x12, 0x16, 0x14, 0x10)


class ProtocolError(Exception):
    pass


class LKMotor:
    def __init__(self, port_or_serial, baudrate=115200, motor_id=1, timeout=0.2):
        """port_or_serial: 포트 경로(str)를 주면 새로 연결을 연다.
        이미 연 serial.Serial 인스턴스를 주면 그 연결을 공유한다(RS485에 여러
        모터가 물린 경우, 버스 하나에 시리얼 연결도 하나만 있어야 하므로 이렇게 공유)."""
        if isinstance(port_or_serial, serial.Serial):
            self.ser = port_or_serial
            self._owns_serial = False
        else:
            self.ser = serial.Serial(port_or_serial, baudrate, timeout=timeout)
            self._owns_serial = True
        self.motor_id = motor_id

    def close(self):
        if self._owns_serial:
            self.ser.close()

    def _build(self, cmd, data=b""):
        head = bytes([HEAD, cmd, self.motor_id, len(data)])
        frame = head + bytes([sum(head) & 0xFF])
        if data:
            frame += data + bytes([sum(data) & 0xFF])
        return frame

    def _read_header(self, cmd):
        """반이중 에코나 잔여 바이트가 섞여도 헤드를 재동기화한다."""
        deadline = time.time() + 0.5
        window = b""
        while time.time() < deadline:
            byte = self.ser.read(1)
            if not byte:
                continue
            window = (window + byte)[-5:]
            if len(window) == 5 and window[0] == HEAD and window[4] == sum(window[0:4]) & 0xFF:
                if window[1] == cmd and window[2] == self.motor_id:
                    return window
                window = b""  # 에코 프레임 등 -> 버리고 계속
        raise ProtocolError(f"cmd 0x{cmd:02X} 응답 없음 (타임아웃)")

    def transact(self, cmd, data=b""):
        self.ser.reset_input_buffer()
        self.ser.write(self._build(cmd, data))
        header = self._read_header(cmd)
        length = header[3]
        if length == 0:
            return b""
        body = self.ser.read(length + 1)
        if len(body) != length + 1:
            raise ProtocolError(f"cmd 0x{cmd:02X} 데이터 부족: {len(body)}/{length + 1}")
        payload, checksum = body[:-1], body[-1]
        if checksum != sum(payload) & 0xFF:
            raise ProtocolError(f"cmd 0x{cmd:02X} 데이터 체크섬 불일치")
        return payload

    # --- 읽기 명령 ---------------------------------------------------------

    def read_info(self):
        d = self.transact(CMD_READ_INFO)
        driver = d[0:20].split(b"\x00")[0].decode("ascii", "replace").strip()
        motor = d[20:40].split(b"\x00")[0].decode("ascii", "replace").strip()
        motor_sn = d[40:52].split(b"\x00")[0].decode("ascii", "replace").strip()
        hw, mv, fw = struct.unpack("<HHH", d[52:58])
        return {"driver": driver, "motor": motor, "sn": motor_sn,
                "hw_ver": hw / 10.0, "motor_ver": mv / 10.0, "fw_ver": fw / 10.0}

    def read_encoder(self):
        enc, raw, offset = struct.unpack("<HHH", self.transact(CMD_READ_ENCODER))
        return {"encoder": enc, "raw": raw, "offset": offset}

    def read_single_angle(self):
        """0x94: 1회전 절대각 [deg], 0~359.99. 모터축 기준, 전원 재인가에도 유지되는 절대 기준값.
        주의: 감속비만큼 한 바퀴 안에서 값이 반복되므로(i10이면 출력축 36deg마다 반복),
        이 값 하나만으로는 여러 바퀴에 걸친 절대 출력각을 구분할 수 없다."""
        (v,) = struct.unpack("<I", self.transact(CMD_READ_SINGLE_ANGLE))
        return v / 100.0

    def read_multi_angle(self):
        """0x92: 전원 인가 후 누적된 다회전 각도 [deg], 모터축 기준. 전원 재인가 시 0으로 초기화됨."""
        (v,) = struct.unpack("<q", self.transact(CMD_READ_MULTI_ANGLE))
        return v / 100.0

    def read_state2(self):
        d = self.transact(CMD_READ_STATE2)
        temp = struct.unpack("<b", d[0:1])[0]
        iq, speed, encoder = struct.unpack("<hhH", d[1:7])
        return {"temp_c": temp, "iq": iq, "speed_dps": speed, "encoder": encoder}

    def read_state1(self):
        """0x9A: 온도/전압/모터 on-off 상태/에러상태."""
        d = self.transact(CMD_READ_STATE1)
        temp = struct.unpack("<b", d[0:1])[0]
        voltage = struct.unpack("<H", d[1:3])[0]
        motor_on = (d[5] == 0x00)
        error_state = d[6]
        return {"temp_c": temp, "voltage_v": voltage / 100.0,
                "motor_on": motor_on, "error_state": error_state}

    # --- 제어 명령 ---------------------------------------------------------

    def handshake(self):
        """이동 명령 전에 필요한 연결 시퀀스. 일부 커맨드는 응답이 없을 수 있어 무시하고 진행."""
        for cmd in HANDSHAKE_CMDS:
            try:
                self.transact(cmd)
            except ProtocolError:
                pass

    def motor_on(self):
        """0x88: 모터를 On 상태로 전환."""
        self.transact(CMD_MOTOR_ON)

    def motor_off(self):
        """0x80: 모터를 Off 상태로 전환. 홀딩 토크가 풀려서 손으로 자유롭게 돌릴 수 있게 된다
        (0x81 stop 은 멈추기만 하고 홀딩 토크는 유지하므로 손으로 돌리기 힘든 채로 남는다)."""
        self.transact(CMD_MOTOR_OFF)

    def stop(self):
        """0x81: 정지 (모터 상태는 유지, 다시 명령 보내면 바로 제어 가능)."""
        self.transact(CMD_MOTOR_STOP)

    def move_to_frame_angle(self, motor_frame_deg, speed_dps_motor):
        """0xA4: 0x92 와 같은 frame(모터축, 전원 인가 후 누적각 기준)의 절대 목표각으로 이동.

        motor_frame_deg: 0x92 기준 절대 목표각 (모터축, deg)
        speed_dps_motor: 모터축 기준 속도 제한 (dps)
        목표값 덮어쓰기라 이전 이동이 끝나길 기다릴 필요 없이 바로 갱신된다.
        """
        angle_control = int(round(motor_frame_deg * 100.0))
        max_speed = int(round(max(speed_dps_motor, 1.0) * 100.0))
        data = struct.pack("<qI", angle_control, max_speed)
        self.transact(CMD_MULTI_ANGLE_CTRL2, data)

    def wait_until_settled(self, timeout=8.0, settle_deg=0.05, settle_count=6, poll_interval=0.05):
        """0x94 를 폴링해 값이 더 이상 변하지 않을 때까지 대기 (주로 영점복귀 등 1회성 동작에 사용).

        settle_count 회 연속으로 변화량이 settle_deg 이하면 정지한 것으로 판단.
        timeout 초 안에 안정되지 않으면 마지막으로 읽은 값으로 그냥 반환한다 (예외 X).
        """
        deadline = time.time() + timeout
        prev_angle = None
        stable = 0
        angle = self.read_single_angle()
        while time.time() < deadline:
            angle = self.read_single_angle()
            if prev_angle is not None:
                diff = abs((angle - prev_angle + 180.0) % 360.0 - 180.0)
                stable = stable + 1 if diff < settle_deg else 0
                if stable >= settle_count:
                    break
            prev_angle = angle
            time.sleep(poll_interval)
        return angle