[한국어](./README.ko.md) | **English**

# iROI — Dual-Arm Robot ROS2 Motor Control

> ROS2 (Humble) motor-control stack for an 8-DOF dual-arm robot using LK-TECH RS485 servo motors. The project is being migrated from a standalone Python control script to a ROS2 architecture while preserving the already-validated low-level motor protocol layer.

**Status: 🚧 In Progress** — Three-axis operation on one Raspberry Pi 4 + LC529 RS485 bus is hardware-verified for IDs 1, 2, and 4, including simultaneous discovery, absolute-encoder automatic homing, ROS2 target-angle motion, and synchronized arrival. Final mixed-i10/i36 8-axis dual-arm integration is still pending.

---

## Verified on Real Hardware

As of **2026-08-14**, the following path has been verified on a Raspberry Pi:

```text
Raspberry Pi 4
   ↓ USB
LC529 USB-RS485 (`/dev/ttyUSB0`, 115200 baud)
   ↓ single RS485 bus
├── MG4010E-i10 (ID 1)
├── MG4010E-i10 (ID 2)
└── MG5010E-i10 (ID 4)
   ↓
External 24 V motor supply
```

Verified items:

- [x] Ubuntu 22.04 + ROS2 Humble on Raspberry Pi 4
- [x] LC529 detected as `/dev/ttyUSB0`
- [x] Simultaneous ID 1, 2, and 4 discovery and model reads with `scan_ids`
- [x] Real motor state / angle reads
- [x] Absolute-encoder-based automatic homing of IDs 1, 2, and 4
- [x] Three-axis ROS2 `MoveJoint` Action target-angle motion for IDs 1, 2, and 4
- [x] Persistent `arm_cli` ActionClient with substantially reduced command-start latency
- [x] Sequential move-complete → next-command operation
- [x] Three-axis synchronized arrival based on the longest-moving axis
- [ ] Command preemption while a previous move is still in progress
- [ ] Dual RS485 bus validation
- [ ] Full 8-motor integration

Current verified configuration:

```text
Port           : /dev/ttyUSB0
Baudrate       : 115200
ID 1           : MG4010E-i10
ID 2           : MG4010E-i10
ID 4           : MG5010E-i10
Reduction ratio: 10:1 for all three test motors
Logical period : 3600 motor-deg for all three test motors
```

---

## Why ROS2?

The original implementation used a standalone Python script (`motor_control.py`) with file-based command polling (`target_batch.json`). As the number of motors and arms increased, timing, concurrency, restart behavior, and synchronized motion became increasingly difficult to manage manually.

ROS2 provides standard primitives for those requirements:

- **Topic** for continuous state publishing
- **Service** for discrete operations such as torque / calibration
- **Action** for goal-based motion with feedback and completion results
- Namespaces and launch files for two independent arms
- A clean path toward MoveIt2 / RViz integration

The low-level RS485 driver (`lk_motor.py`) is retained as the hardware protocol layer; ROS2 is added above it as the orchestration layer rather than replacing the proven motor protocol implementation.

---

## Target System Architecture

```text
Laptop (development / remote control)
   │
   │ SSH / ROS2 tooling
   ▼
Raspberry Pi 4
   ├── USB → LC529 #1 → RS485 → Left arm, 4 motors
   └── USB → LC529 #2 → RS485 → Right arm, 4 motors

24 V motor power
   ├── Left-arm motors in parallel
   └── Right-arm motors in parallel
```

The Raspberry Pi and LC529 provide **communication only**. Motor drive power is supplied separately from the Pi power path.

Target motor configuration:

- MG5010E-i36 ×4
- MG4010E-i10 ×4
- Total: 8 motors
- One LC529 per arm, for two independent RS485 buses total

> The current bench setup is **two MG4010E-i10 motors plus one MG5010E-i10**. The final MG5010 units are planned to use i36 gearing, so the reduction ratio must remain configurable per motor.

---

## ROS2 Node Architecture

The same `motor_control_node` is intended to run once per arm with different parameters and namespaces.

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
   MoveIt2 / RViz  (planned)
```

The current three-axis bench launch uses:

```text
/test_arm/move_to
```

---

## ROS2 Interfaces

| Type | Name | Message Type | Description |
|---|---|---|---|
| Topic | `/{arm}/joint_states` | `sensor_msgs/JointState` | Publishes joint angles |
| Service | `/{arm}/torque` | `std_srvs/SetBool` | Motor torque on/off |
| Service | `/{arm}/set_zero` | `std_srvs/Trigger` | Calibrates/persists the physical zero reference; not intended to redefine zero on every boot |
| Action | `/{arm}/move_to` | `iroi_interfaces/action/MoveJoint` | Goal-based joint motion with feedback/result |

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

Important: the request field is **`max_speeds`** (plural).

---

## Absolute-Encoder Homing

The robot arm does **not** treat the current boot position as zero.

Instead, a physical zero is calibrated and stored once. On startup, the motor uses its absolute encoder to return to that same physical zero.

### Relevant motor angle frames

- `0x94` — persistent absolute angle used to locate the stored physical zero
- `0x92` — accumulated/session angle used for continuous tracking and motion commands
- `0xA4` — move command; target must be expressed in the `0x92` frame

For the current ID 1, 2, and 4 i10 bench motors:

```text
reduction ratio = 10.0
logical period  = 3600.0 motor-deg
```

### Startup homing logic

```text
saved zero_single_deg (0x94 frame)
        ↓
read current 0x94
read current 0x92
        ↓
shortest_delta(saved_zero, current_0x94, period)
        ↓
zero_92 = current_0x92 + delta
        ↓
move_to_frame_angle(zero_92)
        ↓
use zero_92 as the output-zero reference for this power session
```

Shortest signed delta:

```python
def shortest_delta(target, current, period):
    half = period / 2.0
    return (target - current + half) % period - half
```

Output angle after homing is conceptually:

```text
(current_0x92 - zero_92) / reduction_ratio
```

A requested output target is mapped as:

```text
target_0x92 = zero_92 + target_output_angle * reduction_ratio
```

### Real-hardware homing result

ROS2 successfully reused the per-motor absolute zero values from `zero_config_i10_verified.json`, originally produced during the standalone Python hardware tests, to automatically home IDs 1, 2, and 4. The values below preserve the earlier detailed result for ID 4.

Saved zero for the current ID 4 test motor:

```text
saved zero 0x94 = 3599.98 deg
```

The motor was deliberately displaced by hand before the test:

```text
current 0x94    : 728.36 deg
current 0x92    : 728.36 deg
saved zero 0x94 : 3599.98 deg
homing delta    : -728.38 motor-deg
                : -72.838 output-deg
target zero_92  : -0.02 deg
```

The real motor then returned to the stored physical zero successfully. This verifies the `0x94 → shortest_delta → 0x92` frame conversion on hardware.

---

## Low-Latency Command Path: `arm_cli`

A noticeable delay was observed when repeatedly using:

```bash
ros2 action send_goal ...
```

Each invocation starts a new ROS2 CLI process and Action client, which introduces startup/discovery overhead.

To remove that overhead during testing, a persistent client was added:

```bash
ros2 run motor_control_pkg arm_cli
```

It connects once and accepts targets for IDs 1, 2, and 4 plus a speed limit:

```text
arm> 10 0 0 10
arm> 10 -10 20 20
arm> 0 0 0 10
```

Format:

```text
ID1_angle  ID2_angle  ID4_angle  speed
```

Example:

```text
arm> 10 -10 20 20
```

means:

```text
ID 1 target = +10°
ID 2 target = -10°
ID 4 target = +20°
speed limit = 20°/s
```

The CLI `speed` is an upper bound for each axis, not a speed forced equally on every motor. `motor_control_node` uses the longest predicted axis duration and reduces the other axes' speeds so all three arrive nearly together. This existing synchronized-arrival logic has now been verified on the three-axis hardware setup.

### Real-hardware latency result

```text
Repeated `ros2 action send_goal` : noticeable command-start delay
Persistent `arm_cli`             : motion begins immediately from the user's perspective
```

This indicates that the dominant observed delay came from repeatedly starting a fresh ROS2 client, not from the LC529 or motor itself.

The current CLI is intentionally **blocking**: it waits until the move completes before accepting the next `arm>` command. Sequential operation has been verified. Motion preemption / streaming target updates are a future step.

---

## Getting Started

### 1. Build

The active workspace used during real-hardware validation is:

```text
~/iroi_ws
```

Build **only from the workspace root**:

```bash
cd ~/iroi_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```


### 2. Scan motors

```bash
ros2 run motor_control_pkg scan_ids --port /dev/ttyUSB0
```

### 3. Read-only homing check

```bash
ros2 run motor_control_pkg check_home --id 4 --port /dev/ttyUSB0
```

`check_home` calculates the expected homing target without sending a move command.

### 4. Run the current real three-motor test

```bash
ros2 launch motor_control_pkg three_motor_real.launch.py
```

This starts the real motor node and automatically homes IDs 1, 2, and 4 from their absolute zeros in `zero_config_i10_verified.json`.

### 5. Start the persistent test CLI

In another terminal:

```bash
source ~/iroi_ws/install/setup.bash
ros2 run motor_control_pkg arm_cli
```

Then:

```text
arm> 10 0 0 10
arm> 10 -10 20 20
arm> 0 0 0 10
```

### 6. Direct Action test

For one-off debugging:

```bash
ros2 action send_goal /test_arm/move_to iroi_interfaces/action/MoveJoint \
  "{target_angles: [10.0, -10.0, 20.0], max_speeds: [20.0, 20.0, 20.0]}" --feedback
```

For repeated testing, `arm_cli` is preferred because it avoids repeatedly creating a new ROS2 Action client.

---

## Project Structure

```text
iroi_ws/src/
├── motor_control_pkg/
│   ├── config/
│   ├── launch/
│   │   ├── dual_arm.launch.py
│   │   ├── single_motor_id4_real.launch.py
│   │   └── three_motor_real.launch.py
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

`setup.py` console scripts include:

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

### 1. 0x94 / 0x92 frame mismatch — fixed and hardware-verified

An earlier implementation mixed the persistent absolute angle frame (`0x94`) with the session/accumulated motion frame (`0x92`). That could command an unintended position after a power cycle.

The fix computes the shortest delta in the `0x94` frame and applies it to the current `0x92` value before issuing a move. This has now been verified on the real ID 4 motor.

### 2. RS485 concurrent access

Polling and Action callbacks can reach the same half-duplex serial bus from different execution contexts. Communication points are protected with a `threading.Lock` to prevent interleaved request/response traffic.

### 3. Namespace collision between arms

Left and right control nodes must use independent namespaces so that their topics, services, and actions do not collide.

### 4. Communication-read failures must not become fake `0.0` angles

Failed reads should be treated as errors. Motion should not begin from an invalid baseline, and zero calibration should not persist a failed read as a physical reference.

### 5. Nested ROS2 workspace / stale package issue

A `colcon build` was accidentally executed from `~/iroi_ws/src`, creating:

```text
~/iroi_ws/src/build
~/iroi_ws/src/install
~/iroi_ws/src/log
```

That stale installation appeared before the correct workspace in `AMENT_PREFIX_PATH`, so ROS2 reported an older `motor_control_pkg` and could not see the newly added `arm_cli` executable.

Resolved by:

```bash
rm -rf ~/iroi_ws/src/build ~/iroi_ws/src/install ~/iroi_ws/src/log
```

removing the stale `src/install/setup.bash` source entry, then rebuilding from:

```bash
cd ~/iroi_ws
colcon build --symlink-install
source install/setup.bash
```

Rule: **always build from `~/iroi_ws`, never from `~/iroi_ws/src`.**

---

## Roadmap

- [x] ROS2 node architecture and mock-mode validation
- [x] Dual-arm namespace design
- [x] Raspberry Pi 4 + LC529 + single real-motor communication
- [x] Real-hardware absolute-encoder automatic homing
- [x] Real-hardware ROS2 Action target-angle control
- [x] Persistent `arm_cli` with reduced command-start latency
- [x] Sequential move-complete → next-command validation
- [ ] Motion preemption / non-blocking target updates
- [x] Multi-motor validation on one RS485 bus
- [ ] Map each new motor ID to its joint
- [ ] Recalibrate every new motor's absolute zero (`0x94`, encoder, raw); never reuse an existing motor's zero values for a new motor
- [ ] Validate the MG5010E-i36 ratio and wrap behavior on hardware
- [ ] In mixed i10/i36 operation, verify that ratio-dependent calculations use each motor config's `ratio` and `loop_period_deg`, not a shared fallback
- [ ] Validate in stages: individual motors → one 4-axis arm → both 8-axis arms
- [ ] Left/right dual-bus validation
- [ ] Final left/right arm launches and dual-arm 8-axis integration
- [ ] Tune polling rate from real RS485 timing measurements
- [ ] 30+ minute stability test
- [ ] MoveIt2 / RViz integration

---

## Tech Stack

`ROS2 Humble` `Python 3` `pyserial` `Raspberry Pi 4` `RS485` `LC529`
