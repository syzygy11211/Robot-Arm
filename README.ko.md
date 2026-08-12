**한국어** | [English](./README.md)

# iROI — Dual-Arm Robot ROS2 Motor Control

> LK-TECH RS485 서보모터 기반 8자유도 양팔 로봇팔을 기존 단독 Python 제어 스크립트에서 **ROS2 Humble** 구조로 전환하는 프로젝트입니다. 이미 검증된 저수준 모터 통신 로직은 유지하고, 그 위에 ROS2 제어 계층을 구축하고 있습니다.

**상태: 🚧 In Progress** — Raspberry Pi 4 + LC529 + MG5010E-i10(ID 4) 단일 실물 모터 기준으로 통신, 절대엔코더 자동 Homing, ROS2 목표각 이동, persistent `arm_cli` 저지연 명령 경로까지 실물 검증 완료했습니다. 다중 모터 및 최종 8축 통합은 아직 진행 전입니다.

---

## 실물 검증 완료 항목

**2026-08-12 기준** 다음 실물 경로를 벤치에서 검증했습니다.

```text
Raspberry Pi 4
   ↓ USB
LC529 USB-RS485
   ↓ RS485
MG5010E-i10 (ID 4)
   ↓
외부 24 V 모터 전원
```

검증 상태:

- [x] Raspberry Pi 4에 Ubuntu 22.04 + ROS2 Humble 구성
- [x] LC529 `/dev/ttyUSB0` 인식
- [x] 모터 ID 스캔 / 모델 응답 확인
- [x] 실제 모터 상태값 및 각도 읽기
- [x] 절대엔코더 기반 자동 Homing
- [x] ROS2 `MoveJoint` Action 목표각 이동
- [x] persistent `arm_cli` ActionClient를 통한 명령 시작 지연 감소
- [x] 이동 완료 → 다음 명령 순차 제어
- [ ] 이전 이동 중 새 목표를 덮어쓰는 preemption
- [ ] 다중 모터 제어
- [ ] 좌/우 독립 RS485 2버스 검증
- [ ] 전체 8모터 통합

현재 실물 검증 모터:

```text
모델        : MG5010E-i10
Motor ID    : 4
Serial      : U34 P06[
포트        : /dev/ttyUSB0
테스트 전압 : 약 23.9 V
Error state : 0
```

---

## Why ROS2?

기존에는 단독 Python 스크립트(`motor_control.py`)에서 파일 기반 명령 전달(`target_batch.json` polling)을 직접 구현했습니다. 하지만 모터와 팔 개수가 늘어날수록 타이밍, 동시성, 재시작 처리, 다중 모터 동기 이동을 모두 직접 관리해야 했습니다.

ROS2에서는 이를 표준 구조로 나눌 수 있습니다.

- **Topic**: 지속적인 관절 상태 publish
- **Service**: 토크 On/Off, 영점 캘리브레이션 같은 단발 명령
- **Action**: 목표각 이동 + feedback + 완료 결과
- Namespace / launch: 왼팔·오른팔 독립 실행
- 향후 MoveIt2 / RViz 연동 가능

이미 실물에서 검증된 저수준 RS485 드라이버(`lk_motor.py`)는 유지하고, ROS2를 그 위의 orchestration/control 계층으로 추가하는 방식입니다.

---

## 최종 목표 시스템 구조

```text
노트북 (개발 / 원격 제어)
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

Raspberry Pi와 LC529는 **통신만 담당**합니다. 모터 구동용 24 V 전력은 Pi 전원 계통과 분리합니다.

현재 최종 하드웨어 계획:

- MG5010E-i36 ×4
- MG4010E-i10 ×4
- 총 8모터
- 팔당 독립 RS485 bus 1개

> 현재 벤치 테스트 모터는 **MG5010E-i10**입니다. 최종 MG5010은 i36 예정이므로 감속비는 코드에서 설정 가능하게 유지해야 합니다.

---

## ROS2 노드 구조

동일한 `motor_control_node`를 왼팔/오른팔에 각각 실행하고 namespace와 parameter만 다르게 적용하는 구조입니다.

```text
[left_arm]  motor_control_node
[right_arm] motor_control_node
        │
        ├── Topic   /{arm}/joint_states
        ├── Service /{arm}/torque
        ├── Service /{arm}/set_zero
        └── Action  /{arm}/move_to
        │
        ▼
   MoveIt2 / RViz  (예정)
```

현재 단일 모터 실물 벤치 테스트에서는 다음 Action을 사용합니다.

```text
/test_arm/move_to
```

---

## ROS2 Interfaces

| 종류 | 이름 | 타입 | 설명 |
|---|---|---|---|
| Topic | `/{arm}/joint_states` | `sensor_msgs/JointState` | 관절 각도 publish |
| Service | `/{arm}/torque` | `std_srvs/SetBool` | 모터 토크 On/Off |
| Service | `/{arm}/set_zero` | `std_srvs/Trigger` | 물리 영점 기준을 캘리브레이션하고 저장. 매 부팅마다 현재 위치를 새 영점으로 만드는 용도가 아님 |
| Action | `/{arm}/move_to` | `iroi_interfaces/action/MoveJoint` | 목표각 이동 + feedback/result |

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

주의: 속도 필드명은 **`max_speeds`** 복수형입니다.

---

## 절대엔코더 기반 Homing

이 프로젝트는 **부팅할 때 현재 위치를 0점으로 잡는 방식이 아닙니다.**

물리적인 0점 위치를 한 번 캘리브레이션해서 저장하고, 이후 전원이 다시 들어오면 절대엔코더를 읽어 **항상 같은 물리 영점으로 자동 복귀**하는 방식입니다.

### 각도 frame

- `0x94` — 저장된 물리 영점을 찾기 위한 절대각 frame
- `0x92` — 현재 전원 세션에서 누적/추적되는 이동 frame
- `0xA4` — 실제 이동 명령. target은 `0x92`와 같은 frame이어야 함

현재 i10 테스트 모터 기준:

```text
감속비        = 10.0
논리 주기     = 3600.0 motor-deg
```

### 부팅 시 Homing 흐름

```text
저장된 zero_single_deg (0x94 frame)
        ↓
현재 0x94 읽기
현재 0x92 읽기
        ↓
shortest_delta(saved_zero, current_0x94, period)
        ↓
zero_92 = current_0x92 + delta
        ↓
move_to_frame_angle(zero_92)
        ↓
이번 전원 세션의 출력축 0점 기준으로 zero_92 유지
```

최단 signed delta:

```python
def shortest_delta(target, current, period):
    half = period / 2.0
    return (target - current + half) % period - half
```

Homing 후 출력축 각도는 개념적으로:

```text
(current_0x92 - zero_92) / reduction_ratio
```

사용자가 요청한 출력축 목표각은:

```text
target_0x92 = zero_92 + target_output_angle * reduction_ratio
```

으로 변환합니다.

### 실물 Homing 검증 결과

현재 ID 4 테스트 모터의 저장된 영점:

```text
saved zero 0x94 = 3599.98 deg
```

눈으로 Homing이 확실히 보이도록 모터 위치를 손으로 일부러 이동한 뒤 측정:

```text
current 0x94    : 728.36 deg
current 0x92    : 728.36 deg
saved zero 0x94 : 3599.98 deg
homing delta    : -728.38 motor-deg
                : -72.838 output-deg
target zero_92  : -0.02 deg
```

실제 Homing launch 실행 후 모터가 저장된 물리 영점으로 정상 복귀했습니다.

즉 **`0x94 → shortest_delta → 0x92` frame 변환 로직을 실물에서 검증 완료**했습니다.

---

## 저지연 명령 경로: `arm_cli`

초기 테스트에서는 매번 다음 명령을 새로 실행했습니다.

```bash
ros2 action send_goal ...
```

이 방식은 명령마다 새로운 ROS2 CLI 프로세스와 Action client를 생성하므로 discovery/연결 과정 때문에 첫 동작 시작까지 체감 지연이 있었습니다.

이를 줄이기 위해 persistent ActionClient를 유지하는 `arm_cli.py`를 추가했습니다.

실행:

```bash
ros2 run motor_control_pkg arm_cli
```

한 번 연결된 뒤에는:

```text
arm> 10 5
arm> -20 30
arm> 0 10
```

처럼 명령만 계속 입력합니다.

입력 형식:

```text
목표각(deg)  최대속도(deg/s)
```

예:

```text
arm> 10 5
```

의 의미:

```text
목표 출력각 = +10°
최대 속도   = 5°/s
```

### 실물 latency 결과

```text
기존: 매번 `ros2 action send_goal` 실행 → 명령 시작 전 체감 지연 존재
현재: persistent `arm_cli`             → 입력 후 바로 모터가 움직임
```

따라서 벤치에서 보였던 큰 지연의 주원인은 모터/LC529가 아니라 **매 명령마다 새로운 ROS2 client를 띄우던 오버헤드**였다고 볼 수 있습니다.

현재 `arm_cli`는 일부러 **blocking 방식**으로 구현되어 있습니다. 한 이동이 끝난 뒤에야 다음 `arm>` 입력을 받습니다. 이 순차 제어는 실물에서 정상 확인했습니다.

아직 하지 않은 것:

- 이전 이동이 끝나기 전에 새로운 목표를 받아 덮어쓰기
- 비동기/non-blocking 명령
- 연속 streaming target 제어

---

## Getting Started

### 1. Build

현재 실물 검증에 사용한 workspace:

```text
~/iroi_ws
```

**반드시 workspace root에서 빌드합니다.**

```bash
cd ~/iroi_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> `~/iroi_ws/src` 안에서 `colcon build`를 실행하지 마세요. 실제로 중첩 workspace가 생성되어 오래된 패키지를 ROS2가 먼저 읽는 문제가 발생했습니다.

### 2. 모터 ID 스캔

```bash
ros2 run motor_control_pkg scan_ids --port /dev/ttyUSB0
```

### 3. 이동 없이 Homing 계산만 확인

```bash
ros2 run motor_control_pkg check_home --id 4 --port /dev/ttyUSB0
```

`check_home`은 Homing 예상 delta/target만 계산하고 **모터 이동 명령은 보내지 않습니다.**

### 4. 현재 단일 실물 모터 실행

```bash
ros2 launch motor_control_pkg single_motor_id4_real.launch.py
```

실행 시 저장된 절대 영점 기준으로 자동 Homing을 수행합니다.

### 5. Persistent CLI 실행

다른 터미널에서:

```bash
source ~/iroi_ws/install/setup.bash
ros2 run motor_control_pkg arm_cli
```

입력:

```text
arm> 10 5
arm> 20 30
arm> 0 10
```

### 6. 직접 Action 테스트

단발 디버깅용:

```bash
ros2 action send_goal /test_arm/move_to iroi_interfaces/action/MoveJoint \
  "{target_angles: [10.0], max_speeds: [5.0]}" --feedback
```

반복 제어 테스트에서는 새 client를 매번 띄우지 않는 `arm_cli` 사용을 권장합니다.

---

## Project Structure

```text
iroi_ws/src/
├── motor_control_pkg/
│   ├── config/
│   ├── launch/
│   │   ├── dual_arm.launch.py
│   │   └── single_motor_id4_real.launch.py
│   ├── motor_control_pkg/
│   │   ├── __init__.py
│   │   ├── lk_motor.py
│   │   ├── motor_control_node.py
│   │   ├── scan_ids.py
│   │   ├── check_home.py
│   │   └── arm_cli.py
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
└── iroi_interfaces/
    └── action/
        └── MoveJoint.action
```

`setup.py` console scripts:

```python
'console_scripts': [
    'motor_control_node = motor_control_pkg.motor_control_node:main',
    'scan_ids = motor_control_pkg.scan_ids:main',
    'check_home = motor_control_pkg.check_home:main',
    'arm_cli = motor_control_pkg.arm_cli:main',
]
```

---

## Engineering Notes

### 1. `0x94` / `0x92` frame 불일치 — 수정 및 실물 검증 완료

초기 구현에서는 전원 재인가 후에도 유지되는 절대각 `0x94` frame과, 현재 세션의 누적 이동 `0x92` frame이 섞일 가능성이 있었습니다. 전원 사이클 이후 엉뚱한 위치로 이동할 위험이 있었습니다.

현재는 `0x94`에서 최단 delta를 계산한 뒤 현재 `0x92` 값 위에 적용하고, 그 target을 이동 명령에 사용합니다. ID 4 실물 모터에서 정상 동작을 확인했습니다.

### 2. RS485 동시 접근

Polling과 Action callback이 동일한 half-duplex serial bus에 동시에 접근하면 요청/응답이 섞일 수 있습니다. 통신 구간은 `threading.Lock`으로 보호합니다.

### 3. 양팔 namespace 충돌

왼팔/오른팔 노드는 독립 namespace를 사용해야 Topic/Service/Action 이름이 충돌하지 않습니다.

### 4. 통신 실패값을 가짜 `0.0°`로 처리하면 안 됨

읽기 실패는 정상값으로 취급하지 않고 error로 처리해야 합니다. 잘못된 0.0을 기준으로 영점을 저장하거나 이동을 시작하지 않도록 방어합니다.

### 5. 중첩 ROS2 workspace 문제

실수로 `~/iroi_ws/src`에서 `colcon build`를 실행하면서:

```text
~/iroi_ws/src/build
~/iroi_ws/src/install
~/iroi_ws/src/log
```

가 생성된 적이 있습니다.

이 stale install이 `AMENT_PREFIX_PATH`에서 정상 workspace보다 먼저 잡혀, 새로 추가한 `arm_cli`가 설치되어 있음에도 ROS2에서는 executable이 없는 것처럼 보였습니다.

해결:

```bash
rm -rf ~/iroi_ws/src/build ~/iroi_ws/src/install ~/iroi_ws/src/log
```

잘못된 `src/install/setup.bash` source 항목을 제거한 뒤:

```bash
cd ~/iroi_ws
colcon build --symlink-install
source install/setup.bash
```

로 다시 빌드했습니다.

**규칙: `colcon build`는 항상 `~/iroi_ws`에서만 실행합니다.**

---

## Roadmap

- [x] ROS2 노드 구조 및 mock-mode 검증
- [x] Dual-arm namespace 설계
- [x] Raspberry Pi 4 + LC529 + 단일 실물 모터 통신
- [x] 실물 절대엔코더 자동 Homing
- [x] 실물 ROS2 Action 목표각 이동
- [x] persistent `arm_cli`를 통한 명령 시작 지연 감소
- [x] 이동 완료 → 다음 명령 순차 동작 확인
- [ ] 이동 중 새 목표를 받는 preemption / non-blocking 제어
- [ ] 한 RS485 bus에서 다중 모터 검증
- [ ] 좌/우 2개 RS485 bus 검증
- [ ] 전체 8모터 통합
- [ ] 실제 RS485 timing 기반 polling rate 조정
- [ ] 30분 이상 안정성 테스트
- [ ] MoveIt2 / RViz 연동

---

## Tech Stack

`ROS2 Humble` `Python 3` `pyserial` `Raspberry Pi 4` `RS485` `LC529`
