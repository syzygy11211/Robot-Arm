**한국어** | [English](./README.md)

# iRoi — Dual-Arm Robot ROS2 Motor Control

> LK-TECH RS485 서보모터로 구성된 8자유도(양팔 4+4) 로봇팔을, 파이썬 스크립트 기반 제어에서 **ROS2(Humble)** 구조로 전환하는 프로젝트입니다.

**상태: 🚧 In Progress** — mock_mode(시뮬레이션)로 전체 기능 검증 완료, 실물 하드웨어(Raspberry Pi 4 + 8모터) 연결 후 검증 대기 중입니다.

---

## Why ROS2?

기존에는 파이썬 스크립트(`motor_control.py`)로 파일 기반(`target_batch.json` 폴링) 명령 전달을 직접 구현해 사용했습니다. 문제는 모터 개수와 팔 개수가 늘어날수록 동시성·타이밍 관리를 전부 손으로 짜야 한다는 점이었습니다.

- 재시작 안전성, 다중 모터 동기 이동 같은 요구사항은 ROS2의 표준 매커니즘(Topic QoS, Service, Action)이 이미 다루는 영역이라고 판단했습니다.
- 최종 목표(8모터·양팔·Pi 원격 제어·추후 MoveIt2/RViz 연동)의 규모를 고려하면, 노드 단위로 분리되고 표준 통신 방식을 쓰는 구조가 처음부터 맞다고 봤습니다.
- 이미 검증된 저수준 RS485 통신 로직(`lk_motor.py`)은 그대로 유지하고, 그 위에 ROS2를 **상위 wrapper**로 얹는 방식으로 전환했습니다.

---

## System Architecture

```
노트북 (개발/원격 제어)
   │ ROS2 DDS (Ethernet/WiFi)
   ▼
Raspberry Pi 4 Model B (4GB)
   ├── USB-RS485 #1 (LC529) ──▶ 왼팔 4모터  (motor_id 1~4)
   └── USB-RS485 #2 (LC529) ──▶ 오른팔 4모터 (motor_id 5~8)
```

같은 `motor_control_node`를 `arm_name`/`serial_port`/`motor_ids`만 다르게 주어 왼팔·오른팔 각각 독립 실행하고, launch 파일에서 `namespace`로 완전히 분리합니다.

```
[left_arm]  motor_control_node  (motor_ids=[1,2,3,4], /dev/ttyUSB0)
[right_arm] motor_control_node  (motor_ids=[5,6,7,8], /dev/ttyUSB1)
        │
        ├── Topic   /{arm}/joint_states  → 실시간 각도 publish
        ├── Service /{arm}/torque        → 토크 on/off
        ├── Service /{arm}/set_zero      → 소프트웨어 영점 설정 (영구 저장)
        └── Action  /{arm}/move_to       → 다중 모터 동기 목표각 이동
        │
        ▼
   MoveIt2 / RViz  (상위 모션 플래닝, 연동 예정)
```

---

## ROS2 Interfaces

| 종류 | 이름 | 타입 | 설명 |
|---|---|---|---|
| Topic | `/{arm}/joint_states` | `sensor_msgs/JointState` | `polling_hz` 주기로 각 모터 각도 publish (deg→rad 변환) |
| Service | `/{arm}/torque` | `std_srvs/SetBool` | 모터 토크 On/Off |
| Service | `/{arm}/set_zero` | `std_srvs/Trigger` | 현재 위치를 영점으로 설정, 파일로 영구 저장 후 재시작 시 자동 복원 |
| Action | `/{arm}/move_to` | `iroi_interfaces/action/MoveJoint` | 여러 모터를 동시에 목표각으로 동기 이동 (가장 오래 걸리는 모터 기준 시간 T로 나머지 속도 역산) |

`MoveJoint.action`:
```
# Goal
float64[] target_angles
float64[] max_speeds
---
# Result
bool success
bool timeout
string error_message
---
# Feedback
float64[] current_angles
float64[] errors
bool settled
```

모든 파라미터(`serial_port`, `baudrate`, `arm_name`, `motor_ids`, `joint_names`, `polling_hz`, `max_speed_dps`, `mock_mode`, `zero_config_path`)는 하드코딩 없이 ROS2 parameter로 노출됩니다.

---

## Hardware

| 항목 | 내용 |
|---|---|
| 모터 | LK-TECH MG5010E-i10 / MG4010E-i10, 감속비 10:1, 듀얼 마그네틱 절대값 엔코더 (모터축 18bit + 출력축 14bit) |
| 통신 | RS485, 115200bps (최대 4Mbps 설정 가능) |
| 컨버터 | USB-RS485 (Coms LC529) ×2, 팔당 1개 독립 버스 |
| 제어 보드 | Raspberry Pi 4 Model B (4GB), Ubuntu + ROS2 Humble |

---

## Getting Started

```bash
# 빌드
cd ~/ros2_ws
colcon build --packages-select iroi_interfaces motor_control_pkg
source install/setup.bash

# mock 모드로 단일 노드 실행 (하드웨어 없이 구조만 검증)
ros2 run motor_control_pkg motor_control_node --ros-args \
  -p arm_name:=right_arm -p serial_port:=/dev/ttyUSB1 -p motor_ids:="[5,6,7,8]" -p mock_mode:=true

# 왼팔+오른팔 동시 실행 (실물 모드가 launch 기본값)
ros2 launch motor_control_pkg dual_arm.launch.py

# 모터 ID 실측 스캔 (DIP 스위치 표를 맹신하지 않고 실제 응답으로 확인)
ros2 run motor_control_pkg scan_ids --port /dev/ttyUSB0

# 토크 on/off
ros2 service call /left_arm/torque std_srvs/srv/SetBool "{data: true}"

# 영점 설정
ros2 service call /left_arm/set_zero std_srvs/srv/Trigger "{}"

# 다중 모터 동기 이동
ros2 action send_goal /left_arm/move_to iroi_interfaces/action/MoveJoint \
  "{target_angles: [30.0, 60.0, 90.0, 15.0], max_speeds: [20.0, 20.0, 20.0, 20.0]}" --feedback
```

---

## Project Structure

```
ros2_ws/src/
├── motor_control_pkg/
│   ├── motor_control_pkg/
│   │   ├── lk_motor.py           # RS485 저수준 드라이버 (기존 검증 자산 그대로 이관)
│   │   ├── motor_control_node.py # ROS2 노드 본체 (Topic/Service/Action)
│   │   └── scan_ids.py           # 모터 ID 진단 스크립트
│   └── launch/
│       └── dual_arm.launch.py    # 왼팔+오른팔 동시 기동
└── iroi_interfaces/
    └── action/
        └── MoveJoint.action      # 다중 모터 동기이동 커스텀 액션
```

---

## Engineering Notes — 실물 연결 전 코드 리뷰로 발견/수정한 문제들

mock_mode 검증만으로는 드러나지 않는 문제들을 코드 리뷰 단계에서 미리 찾아 수정했습니다. 실물 하드웨어의 물리적 특성 때문에 시뮬레이션으로는 재현이 안 되는 종류의 버그가 있다는 걸 확인한 과정이라, 기록해둡니다.

**⚠️ 최우선 확인 필요 — 각도 frame 불일치 (수정은 했으나 실물 미검증)**
전원 재인가 후에도 유지되는 절대각(0x94)을, 전원 인가 시점부터 0부터 누적되는 별도 frame(0x92)에 그대로 넘기고 있었습니다. 두 frame은 전원 사이클마다 값이 달라질 수 있어 그대로 섞으면 엉뚱한 위치로 이동 명령이 나갈 위험이 있었습니다. `shortest_delta`로 0x94 기준 델타를 구해 현재 0x92 값에 더하는 방식으로 수정했습니다. mock 시뮬레이션은 이 두 frame을 구분하지 않기 때문에, 이 버그는 애초에 mock으로는 재현될 수 없었습니다 — 실물 연결 후 가장 먼저 확인해야 할 항목입니다.

**RS485 동시 접근**
Polling 타이머와 Action이 서로 다른 스레드에서 동일한 `serial.Serial` 객체에 접근할 수 있는 구조였습니다. RS485는 반이중이라 동시 접근 시 요청/응답이 섞일 위험이 있어, `threading.Lock`으로 모든 통신 지점을 감쌌습니다.

**양팔 이름 충돌**
launch에서 `name`만 다르고 `namespace`가 없어 왼팔/오른팔이 전역 토픽·서비스를 공유하고 있었습니다. `namespace='left_arm'`/`'right_arm'`을 추가해 분리했고, `ros2 topic list` / `ros2 service list`로 실측 확인했습니다.

**통신 실패 시 0.0 처리**
모터 읽기 실패 시 조용히 0.0으로 채워 넘어가던 부분을, 실패 모터 ID를 함께 반환하도록 바꿔 `/set_zero`가 실패값을 영점으로 저장하지 않도록 막고, 이동 시작 전 읽기 실패 시 이동을 아예 시작하지 않도록 했습니다.

---

## Roadmap

- [x] ROS2 노드 구조 설계 및 mock_mode 전체 시나리오 검증
- [x] Dual-arm launch, namespace 분리
- [ ] Raspberry Pi 4 + 실물 모터 3개로 1차 실동작 검증
- [ ] `polling_hz` 실측 기반 조정 (현재 30Hz는 이론값, RS485 반이중 특성상 여유 확인 필요)
- [ ] 8모터(양팔) 전체 연동 및 30분 이상 안정성 테스트
- [ ] MoveIt2 / RViz 연동

---

## Tech Stack

`ROS2 Humble` `Python 3` `pyserial` `Raspberry Pi 4` `RS485`
