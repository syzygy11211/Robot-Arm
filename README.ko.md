**한국어** | [English](./README.md)

# iROI — Dual-Arm Robot ROS2 Motor Control

> LK-TECH RS485 서보모터 기반 8자유도 양팔 로봇팔 제어 프로젝트입니다. 기존에 실물 검증한 저수준 모터 프로토콜은 유지하고, 그 위에 ROS2 Humble 기반 다축 제어, pose 저장/재생, startup pose, teach mode 계층을 추가하고 있습니다.

**상태: 🚧 In Progress** — Raspberry Pi 4 + LC529 기반 ID 1, 2, 4 3축 실물 제어는 검증 완료했습니다. 새로 추가한 8축 pose/teach framework는 ROS2 mock mode에서 검증했습니다. 최종 i10/i36 혼합 8모터 실물 통합, 각 모터별 calibration, pose/teach 실물 검증은 아직 진행 전입니다.

---

## 현재 검증 상태

### 실물 하드웨어 — 검증 완료

**2026-08-14 기준** 다음 실물 경로를 검증했습니다.

```text
Raspberry Pi 4
   ↓ USB
LC529 USB-RS485 (/dev/ttyUSB0, 115200 baud)
   ↓ single RS485 bus
├── MG4010E-i10 (ID 1)
├── MG4010E-i10 (ID 2)
└── MG5010E-i10 (ID 4)
   ↓
외부 24 V 모터 전원
```

검증 완료 항목:

- [x] Raspberry Pi 4 + Ubuntu 22.04 + ROS2 Humble
- [x] LC529 `/dev/ttyUSB0` 통신
- [x] ID 1, 2, 4 동시 검색 및 모델 응답 확인
- [x] 실물 모터 상태값/각도 읽기
- [x] ID 1, 2, 4 절대엔코더 기반 자동 Homing
- [x] ROS2 `MoveJoint` Action 목표각 이동
- [x] persistent `arm_cli` 저지연 명령
- [x] 이동 완료 → 다음 명령 순차 제어
- [x] 가장 오래 걸리는 축 기준 3축 synchronized arrival
- [ ] 최종 좌/우 2개 RS485 bus 실물 검증
- [ ] 최종 8모터 실물 통합

현재 검증된 calibration 파일:

```text
motor_control_pkg/config/zero_config_i10_verified.json
```

현재 벤치 모터는 모두 **감속비 10:1**, **논리 주기 3600 motor-deg** 기준입니다. 최종 MG5010은 i36 예정이므로 감속비와 wrap 관련 값은 모터별 config에서 관리합니다.

### 8축 pose/teach framework — mock 검증 완료

Raspberry Pi의 ROS2 mock mode에서 다음 구조를 검증했습니다.

```text
왼팔  : ID 1, 2, 3, 4
오른팔: ID 5, 6, 7, 8
```

mock 검증 완료 항목:

- [x] 좌/우 namespace 기반 8축 startup
- [x] ID 1..8 고정 pose database
- [x] 미측정 값 `null` 처리
- [x] Pose 0 (`attention`) startup 예약 pose
- [x] pose 저장 / 목록 / 상세 조회 / 재생
- [x] 8축 상태 조회
- [x] Teach ON → torque OFF
- [x] Teach mode / torque OFF 상태에서 이동 명령 차단
- [x] Teach OFF → torque ON + 현재 위치 HOLD
- [x] Pose 0이 전부 `null`일 때 이동 없이 현재 위치 유지

**주의:** pose/teach framework는 아직 최종 실물 8모터 로봇팔에서 검증하지 않았습니다.

---

## 최종 하드웨어 구조

```text
노트북 / 개발 PC
   │
   │ SSH / ROS2 tooling
   ▼
Raspberry Pi 4
   ├── USB → LC529 #1 → RS485 → 왼팔 4모터
   └── USB → LC529 #2 → RS485 → 오른팔 4모터

24 V 모터 전원
   ├── 왼팔 4모터 병렬 공급
   └── 오른팔 4모터 병렬 공급
```

최종 모터 구성:

- MG5010E-i36 ×4
- MG4010E-i10 ×4
- 총 8모터
- 팔당 LC529 1개
- 좌/우 독립 RS485 bus 총 2개

Raspberry Pi와 LC529는 통신/제어만 담당합니다. 모터 구동용 24 V 전력은 Pi 전원 계통과 분리합니다.

---

## ROS2 제어 구조

동일한 `motor_control_node`를 왼팔/오른팔에 각각 실행하고 namespace와 motor ID 목록만 다르게 적용합니다.

```text
[left_arm]  motor_control_node ── ID 1,2,3,4
[right_arm] motor_control_node ── ID 5,6,7,8

        │
        ├── Topic    /{arm}/joint_states
        ├── Service  /{arm}/torque
        ├── Service  /{arm}/teach
        ├── Service  /{arm}/sync_reference
        ├── Service  /{arm}/set_zero
        ├── Service  /{arm}/home
        └── Action   /{arm}/move_to
```

저수준 RS485 드라이버 `lk_motor.py`는 그대로 유지합니다. ROS2는 그 위에서 상태 publish, Service, 다축 Action, pose 실행을 담당합니다.

---

## ROS2 Interfaces

| 종류 | 이름 | 타입 | 역할 |
|---|---|---|---|
| Topic | `/{arm}/joint_states` | `sensor_msgs/JointState` | calibration 기준 출력축 관절각 publish |
| Service | `/{arm}/torque` | `std_srvs/SetBool` | 저수준 팔 단위 torque ON/OFF |
| Service | `/{arm}/teach` | `std_srvs/SetBool` | hand-guiding용 teach mode 전환 |
| Service | `/{arm}/sync_reference` | `std_srvs/Trigger` | 물리 이동 없이 현재 세션의 0x92 기준 복원 |
| Service | `/{arm}/set_zero` | `std_srvs/Trigger` | 물리 encoder zero 재캘리브레이션 및 저장 |
| Service | `/{arm}/home` | `std_srvs/Trigger` | 저장된 encoder zero 위치로 실제 이동 |
| Action | `/{arm}/move_to` | `iroi_interfaces/action/MoveJoint` | 다축 목표각 이동 + feedback/result |

`MoveJoint.action`:

```text
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

`max_speeds` 단위는 출력축 기준 deg/s입니다.

---

## Encoder Calibration과 Pose의 분리

이 프로젝트에서는 **모터 calibration**과 **로봇 pose**를 완전히 분리합니다.

### 1. 모터 / encoder calibration

다음과 같은 파일에 저장합니다.

```text
zero_config_i10_verified.json
```

모터별 주요 값:

```text
motor_id
ratio
zero_single_deg
zero_encoder
zero_raw
loop_period_deg
```

이 값은 해당 모터의 물리 좌표계를 정의합니다. 모터를 교체하면 새 모터 기준으로 다시 calibration해야 하며, 기존 모터의 영점값을 그대로 복사해서 사용하면 안 됩니다.

### 2. 로봇 pose

runtime pose는 별도로 저장합니다.

```text
~/.ros/arm_poses.json
```

Pose에는 raw encoder가 아니라 **calibration된 출력축/관절각**을 저장합니다.

모든 pose는 항상 ID 1..8을 가지며, 아직 측정하지 않은 값은 JSON `null`로 저장합니다.

```json
{
  "version": 1,
  "poses": {
    "0": {
      "name": "attention",
      "angles": {
        "1": null,
        "2": null,
        "3": null,
        "4": null,
        "5": null,
        "6": null,
        "7": null,
        "8": null
      }
    }
  }
}
```

Pose 0은 **차렷/startup pose**로 예약되어 있으며 삭제할 수 없습니다.

`version: 1`은 모터 버전이 아니라 pose 파일 형식의 버전입니다.

---

## 절대엔코더 Reference와 Startup Mode

### 주요 각도 frame

- `0x94` — 전원 재인가 후에도 유지되는 절대 encoder angle
- `0x92` — 현재 세션에서 사용하는 누적/다회전 angle
- `0xA4` — `0x92` frame 기준 이동 명령

calibration된 출력축 각도는 개념적으로 다음과 같습니다.

```text
output_angle = (current_0x92 - zero_92) / reduction_ratio
```

### `home` mode

기존 실물 검증 방식:

```text
저장된 0x94 zero 읽기
        ↓
현재 0x94 + 현재 0x92 읽기
        ↓
shortest_delta(...)
        ↓
저장된 zero를 현재 0x92 frame에 매핑
        ↓
실제로 해당 zero 위치까지 이동
```

현재 실물 검증 fallback launch:

```bash
ros2 launch motor_control_pkg three_motor_real.launch.py
```

### `reference_only` mode

새 pose framework에서는 좌표계 복원과 물리 Homing을 분리합니다.

```text
저장된 0x94 zero 읽기
        ↓
현재 0x94 + 현재 0x92 읽기
        ↓
zero_92 계산
        ↓
calibration된 joint 좌표계 복원
        ↓
encoder zero로는 이동하지 않음
        ↓
startup node가 Pose 0 실행 시도
```

따라서 최종적으로는 전원을 켰을 때 encoder zero를 먼저 찍고 오는 대신, 현재 위치에서 좌표계만 복원한 후 **Pose 0(차렷)** 으로 바로 이동하게 됩니다.

개발용 launch:

```bash
ros2 launch motor_control_pkg three_motor_pose_framework.launch.py
```

이 launch는 ID 1, 2, 4 기준으로 틀을 준비해둔 상태이며 **아직 실물 검증하지 않았습니다.** 현재 Pose 0이 전부 `null`이므로 실제 pose 값이 저장되기 전에는 startup pose 이동이 없어야 정상입니다.

---

## Pose Framework

### Pose Manager

`pose_manager.py`가 8축 pose 저장을 담당합니다.

규칙:

- 항상 ID 1..8 존재
- 미측정 값은 `null`
- Pose 0은 항상 존재
- Pose 0 삭제 금지
- pose file version 검사
- atomic write 방식으로 저장

### Pose CLI

8축 CLI 실행:

```bash
ros2 run motor_control_pkg arm_pose_cli --ros-args -p mode:=dual
```

명령:

```text
pose <ID> [speed]       저장된 pose로 이동
sequence <ID...>        입력한 순서대로 저장 pose를 연속 실행
save <ID> [name]        현재 8축 관절값 저장
list                    pose 목록
show <ID>               pose 상세
delete <ID>             pose 삭제 (Pose 0 삭제 불가)
teach <target> on       Teach ON = torque OFF
teach <target> off      Teach OFF = torque ON + 현재 위치 HOLD
torque <target> on/off  raw torque 제어
status                  현재 8축 상태
help
q | quit | exit
```

target:

```text
test / left / right / all
```

예:

```text
arm> save 1 wave
arm> show 1
arm> pose 1 20
arm> sequence 0 1 2 3 2 1 0
arm> teach left on
arm> teach left off
```

`sequence`는 이동을 시작하기 전에 모든 pose ID가 존재하는지 확인합니다.
그다음 각 `MoveJoint` Action이 성공적으로 완료될 때까지 기다린 후 다음
pose를 전송합니다. 오류 또는 timeout이 발생하면 남은 sequence를 중단합니다.
현재는 `default_pose_speed_dps`를 사용하며 sequence별 속도 지정은 아직
지원하지 않습니다.

순차 실행 명령의 코드 구현과 패키지 빌드는 완료했습니다. Mock 및 실물
하드웨어 동작 검증은 아직 진행하지 않았습니다.

pose 값이 `null`인 모터는 새 목표각을 주지 않고 현재 calibration된 위치를 유지합니다.

---

## Teach Mode

Teach mode는 향후 사람이 직접 팔을 움직여 pose를 저장하기 위한 기능입니다.

### Teach ON

```text
STOP
  ↓
Torque OFF
  ↓
사람이 팔을 직접 원하는 자세로 이동
  ↓
encoder / joint 값은 계속 읽기 가능
  ↓
현재 자세를 pose로 저장
```

### Teach OFF

이전 command target로 바로 복귀시키지 않습니다.

```text
현재 0x92 위치 읽기
        ↓
Torque ON
        ↓
방금 읽은 현재 위치를 HOLD target으로 명령
```

즉 사람이 잡아둔 현재 자세를 기준으로 다시 토크를 걸도록 설계했습니다.

teach 전환 중 오류가 발생하면 best-effort 방식으로 전체 torque OFF를 시도합니다.

> **안전 주의:** torque OFF는 24 V 전원을 끄는 것이 아닙니다. 통신/encoder 읽기는 유지되지만 holding torque가 해제됩니다. 실물 로봇팔은 중력으로 떨어질 수 있으므로, 실제 teach mode 테스트 시 반드시 팔을 물리적으로 지지해야 합니다.

Teach mode 또는 torque OFF 상태에서는 `MoveJoint` 이동 명령을 거부합니다.

---

## Getting Started

### 1. Build

현재 Raspberry Pi workspace:

```text
~/iroi_ws
```

반드시 workspace root에서 빌드합니다.

```bash
cd ~/iroi_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`~/iroi_ws/src`에서 build하면 안 됩니다.

저장소를 다른 이름의 workspace에 내려받았다면 해당 workspace root를
사용합니다. 현재 Ubuntu 개발 환경은 `~/ros2_ws`를 사용하므로 빌드할 때
`cd ~/ros2_ws`에서 시작합니다.

### 2. 개발할 때 열어야 하는 폴더와 파일

에디터에서는 workspace root를 엽니다. Raspberry Pi에서는 `~/iroi_ws`, 현재
Ubuntu 개발 PC에서는 `~/ros2_ws`입니다. 주요 파일의 역할은 다음과 같습니다.

| 용도 | 파일 |
| --- | --- |
| Pose/teach/sequence CLI 명령 | `src/motor_control_pkg/motor_control_pkg/arm_pose_cli.py` |
| 한 번의 `MoveJoint` 이동과 모터 안전 차단 | `src/motor_control_pkg/motor_control_pkg/motor_control_node.py` |
| Pose JSON 읽기와 저장 | `src/motor_control_pkg/motor_control_pkg/pose_manager.py` |
| 양팔 mock 실행 | `src/motor_control_pkg/launch/dual_arm_pose_framework.launch.py` |
| 검증된 3모터 실물 fallback | `src/motor_control_pkg/launch/three_motor_real.launch.py` |
| 3모터 pose-framework 개발 launch | `src/motor_control_pkg/launch/three_motor_pose_framework.launch.py` |

실행 중 저장한 pose는 `~/.ros/arm_poses.json`에 기록됩니다. 이 파일은 저장소의
template이 아니라 runtime 데이터이므로 보통 직접 편집하지 않고
`arm_pose_cli`를 통해 변경합니다.

### 3. 모터 없이 mock pose/sequence 실행

빌드가 성공했다면 터미널 2개를 엽니다. 새 터미널을 열 때마다 ROS2와
workspace를 source해야 package 명령을 사용할 수 있습니다.

터미널 1 — 양팔 mock motor-control node와 startup-pose node 실행:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch motor_control_pkg dual_arm_pose_framework.launch.py
```

터미널 2 — 대화형 pose CLI 실행:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run motor_control_pkg arm_pose_cli --ros-args -p mode:=dual
```

`arm_pose_cli` 안에서 다음 순서로 최소 명령 흐름을 확인할 수 있습니다.

```text
arm> save 1 mock_step_1
arm> save 2 mock_step_2
arm> list
arm> sequence 0 1 2 1 0
```

Mock mode에서는 pose 검색, Action 완료, 실행 순서와 오류 처리를 확인할 수
있습니다. 실제 간섭, 중력, calibration 및 모터 안전성은 검증하지 못합니다.

### 4. 모터 ID 스캔

```bash
ros2 run motor_control_pkg scan_ids --port /dev/ttyUSB0
```

### 5. 이동 없이 Homing 계산만 확인

```bash
ros2 run motor_control_pkg check_home --id 4 --port /dev/ttyUSB0
```

`check_home`은 예상 Homing target만 계산하고 이동 명령은 보내지 않습니다.

### 6. 현재 검증된 3모터 실물 launch

```bash
ros2 launch motor_control_pkg three_motor_real.launch.py
```

ID 1, 2, 4를 대상으로 현재 실물 검증된 fallback 경로이며, 저장된 encoder zero로 실제 Homing합니다.

### 7. Persistent 직접 제어 CLI

```bash
ros2 run motor_control_pkg arm_cli
```

입력 형식:

```text
ID1_angle  ID2_angle  ID4_angle  speed
```

예:

```text
arm> 10 -10 20 20
```

---

## Project Structure

```text
iroi_ws/src/
├── motor_control_pkg/
│   ├── config/
│   │   ├── poses.json
│   │   └── zero_config_i10_verified.json
│   ├── launch/
│   │   ├── dual_arm.launch.py
│   │   ├── dual_arm_pose_framework.launch.py
│   │   ├── single_motor_id4_real.launch.py
│   │   ├── three_motor_pose_framework.launch.py
│   │   └── three_motor_real.launch.py
│   ├── motor_control_pkg/
│   │   ├── __init__.py
│   │   ├── lk_motor.py
│   │   ├── motor_control_node.py
│   │   ├── pose_manager.py
│   │   ├── arm_pose_cli.py
│   │   ├── arm_startup_pose.py
│   │   ├── arm_cli.py
│   │   ├── scan_ids.py
│   │   └── check_home.py
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
└── iroi_interfaces/
    └── action/
        └── MoveJoint.action
```

설치되는 console script:

```text
motor_control_node
scan_ids
check_home
arm_cli
arm_pose_cli
arm_startup_pose
```

---

## Engineering Notes

### RS485 동시 접근

LC529 RS485 bus는 half-duplex이므로 polling, Service, Action이 같은 bus를 동시에 사용하지 않도록 serial read/write 구간을 lock으로 보호합니다.

### 통신 실패를 정상 `0.0°`로 처리하지 않음

읽기 실패는 error로 처리합니다. 실패값을 기준으로 이동을 시작하거나 물리 영점으로 저장하면 안 됩니다.

### Calibration은 모터별로 관리

최종 구성은 i10/i36 혼합이므로 `ratio`, `loop_period_deg`, speed limit, zero reference를 모터별 config에서 읽어야 합니다.

### runtime pose와 repository template 분리

repository의:

```text
motor_control_pkg/config/poses.json
```

은 초기 template입니다.

실제 pose 저장은:

```text
~/.ros/arm_poses.json
```

을 사용합니다. 따라서 현장에서 teach한 pose가 실수로 Git에 올라가는 것을 방지합니다.

---

## Roadmap

- [x] ROS2 노드 구조 및 mock 검증
- [x] Dual-arm namespace 설계
- [x] Raspberry Pi 4 + LC529 + 실물 모터 통신
- [x] 절대엔코더 기반 실물 자동 Homing
- [x] 실물 ROS2 Action 목표각 이동
- [x] persistent `arm_cli`
- [x] 한 RS485 bus에서 다중 모터 검증
- [x] 3축 synchronized arrival
- [x] 8축 pose database framework
- [x] mock pose 저장/list/show/playback
- [x] mock Teach mode 이동 차단
- [x] mock Teach OFF current-position hold
- [x] 순차 pose 명령 구현 및 패키지 빌드
- [ ] 순차 pose 실행 mock 검증
- [ ] 순차 pose 실행 실물 하드웨어 검증
- [ ] 최종 모터 ID ↔ 실제 joint 매핑
- [ ] 최종 8모터별 절대 encoder zero calibration
- [ ] MG5010E-i36 ratio / wrap 실물 검증
- [ ] mixed i10/i36 실물 검증
- [ ] `reference_only` startup 실물 검증
- [ ] 8축 Pose 0 (`attention`) 실제 값 저장
- [ ] 기구적으로 지지된 상태에서 Teach mode 실물 검증
- [ ] 한 팔 4축 실물 검증
- [ ] 좌/우 2개 RS485 bus 실물 검증
- [ ] 전체 8모터 통합
- [ ] 최종 startup에서 `allow_partial_pose=False` 적용
- [ ] 장시간 안정성 테스트
- [ ] MoveIt2 / RViz 연동

---

## Tech Stack

`ROS2 Humble` `Python 3` `pyserial` `Raspberry Pi 4` `RS485` `LC529`
