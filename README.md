[한국어](./README.ko.md) | **English**

# iROI — Dual-Arm Robot ROS2 Motor Control

> ROS2 Humble control stack for an 8-DOF dual-arm robot using LK-TECH RS485 servo motors. The project keeps the already hardware-validated low-level motor protocol and adds ROS2-based multi-axis control, persistent pose storage, startup-pose handling, and teach-mode support above it.

**Status: 🚧 In Progress** — Three-axis real-hardware control on Raspberry Pi 4 + LC529 is verified for IDs 1, 2, and 4. The new 8-axis pose/teach framework has been validated in ROS2 mock mode. Final mixed-i10/i36 8-motor hardware integration, per-motor calibration, and real-hardware pose/teach validation are still pending.

---

## Current Validation Status

### Real hardware — verified

As of **2026-08-14**, the following real-hardware path has been validated:

```text
Raspberry Pi 4
   ↓ USB
LC529 USB-RS485 (/dev/ttyUSB0, 115200 baud)
   ↓ single RS485 bus
├── MG4010E-i10 (ID 1)
├── MG4010E-i10 (ID 2)
└── MG5010E-i10 (ID 4)
   ↓
External 24 V motor supply
```

Verified items:

- [x] Ubuntu 22.04 + ROS2 Humble on Raspberry Pi 4
- [x] LC529 communication on `/dev/ttyUSB0`
- [x] Simultaneous discovery/model reads for IDs 1, 2, and 4
- [x] Real motor state and angle reads
- [x] Absolute-encoder-based automatic homing for IDs 1, 2, and 4
- [x] ROS2 `MoveJoint` Action target-angle motion
- [x] Persistent low-latency `arm_cli`
- [x] Sequential move-complete → next-command operation
- [x] Three-axis synchronized arrival based on the longest-moving axis
- [ ] Final left/right dual-RS485 hardware validation
- [ ] Final 8-motor hardware integration

Current verified calibration file:

```text
motor_control_pkg/config/zero_config_i10_verified.json
```

The current bench motors all use a **10:1 reduction ratio** and **3600 motor-deg logical period**. Final MG5010 units are planned to use i36 gearing, so ratio-dependent calculations remain configurable per motor.

### 8-axis pose/teach framework — mock verified

The following software path has been validated on Raspberry Pi in ROS2 mock mode:

```text
Left arm  : IDs 1, 2, 3, 4
Right arm : IDs 5, 6, 7, 8
```

Verified in mock mode:

- [x] Dual-arm namespaced 8-axis startup
- [x] Pose database with fixed IDs 1..8
- [x] `null` handling for unknown/unmeasured motor values
- [x] Reserved Pose 0 (`attention` / startup pose)
- [x] Pose save / list / show / playback
- [x] 8-axis status view
- [x] Teach ON → torque OFF
- [x] Motion blocked while teach mode / torque OFF is active
- [x] Teach OFF → torque ON + current-position hold
- [x] Startup Pose 0 with all `null` values → no physical target, current position held

**Important:** the pose/teach framework has not yet been validated on the final physical 8-motor arm.

---

## Target Hardware Architecture

```text
Laptop / development PC
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

Target motor configuration:

- MG5010E-i36 ×4
- MG4010E-i10 ×4
- Total: 8 motors
- One LC529 per arm
- Two independent RS485 buses total

The Raspberry Pi and LC529 are communication/control devices only. Motor drive power is supplied separately from the Pi power path.

---

## Control Architecture

The same `motor_control_node` is intended to run once per arm with different namespaces and motor lists.

```text
[left_arm]  motor_control_node ── IDs 1,2,3,4
[right_arm] motor_control_node ── IDs 5,6,7,8

        │
        ├── Topic    /{arm}/joint_states
        ├── Service  /{arm}/torque
        ├── Service  /{arm}/teach
        ├── Service  /{arm}/sync_reference
        ├── Service  /{arm}/set_zero
        ├── Service  /{arm}/home
        └── Action   /{arm}/move_to
```

The existing low-level RS485 driver (`lk_motor.py`) remains the hardware protocol layer. ROS2 is used above it for state publication, services, multi-axis motion, pose execution, and future MoveIt2/RViz integration.

---

## ROS2 Interfaces

| Type | Name | Message Type | Purpose |
|---|---|---|---|
| Topic | `/{arm}/joint_states` | `sensor_msgs/JointState` | Publish calibrated output-axis joint angles |
| Service | `/{arm}/torque` | `std_srvs/SetBool` | Raw arm-level torque ON/OFF |
| Service | `/{arm}/teach` | `std_srvs/SetBool` | Hand-guiding mode with safe torque transition |
| Service | `/{arm}/sync_reference` | `std_srvs/Trigger` | Restore calibrated 0x92 reference without physical movement |
| Service | `/{arm}/set_zero` | `std_srvs/Trigger` | Recalibrate and persist the physical encoder zero |
| Service | `/{arm}/home` | `std_srvs/Trigger` | Physically return to the saved encoder zero |
| Action | `/{arm}/move_to` | `iroi_interfaces/action/MoveJoint` | Goal-based multi-joint motion with feedback/result |

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

`max_speeds` is expressed in output-axis deg/s.

---

## Encoder Calibration vs Robot Poses

The system intentionally separates **motor calibration** from **robot poses**.

### 1. Motor / encoder calibration

Stored in files such as:

```text
zero_config_i10_verified.json
```

Each motor stores its own calibration information, including:

```text
motor_id
ratio
zero_single_deg
zero_encoder
zero_raw
loop_period_deg
```

This defines the physical coordinate reference for that motor. A replacement motor must be calibrated independently; calibration values must not be copied blindly between motors.

### 2. Robot poses

Runtime poses are stored separately in:

```text
~/.ros/arm_poses.json
```

Pose values are **calibrated output/joint angles**, not raw encoder values.

All pose records always contain IDs 1..8. Unknown values are stored as JSON `null`.

Example:

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

Pose 0 is reserved as the **startup / attention pose** and cannot be deleted.

`version: 1` is the pose-file format version, not a motor version.

---

## Absolute Encoder Reference and Startup Modes

### Relevant motor angle frames

- `0x94` — persistent absolute encoder angle
- `0x92` — accumulated/session angle used for tracking and motion
- `0xA4` — motion command using the `0x92` frame

The calibrated output angle is conceptually:

```text
output_angle = (current_0x92 - zero_92) / reduction_ratio
```

### `home` mode

Legacy hardware-verified behavior:

```text
read saved zero in 0x94
        ↓
read current 0x94 + current 0x92
        ↓
shortest_delta(...)
        ↓
map saved zero into current 0x92 frame
        ↓
physically move to that zero
```

This is still used by the verified fallback launch:

```bash
ros2 launch motor_control_pkg three_motor_real.launch.py
```

### `reference_only` mode

The new pose framework separates coordinate restoration from physical homing:

```text
read saved zero in 0x94
        ↓
read current 0x94 + current 0x92
        ↓
compute zero_92
        ↓
restore calibrated joint coordinates
        ↓
NO physical move to encoder zero
        ↓
startup pose node attempts Pose 0
```

This allows the final robot to start from its calibrated coordinate system and move directly to the defined **Pose 0** instead of first visiting encoder-zero position.

Development launch:

```bash
ros2 launch motor_control_pkg three_motor_pose_framework.launch.py
```

This launch is prepared for IDs 1, 2, and 4 but has **not yet been real-hardware validated**. The current Pose 0 values are still `null`, so no startup pose movement should occur until actual pose values are recorded.

---

## Pose Framework

### Pose manager

`pose_manager.py` provides persistent 8-axis pose storage.

Rules:

- IDs 1..8 are always present
- unknown/unmeasured values are `null`
- Pose 0 always exists
- Pose 0 cannot be deleted
- pose-file version is validated
- saves are written atomically

### Pose CLI

Run the 8-axis CLI:

```bash
ros2 run motor_control_pkg arm_pose_cli --ros-args -p mode:=dual
```

Available commands:

```text
pose <ID> [speed]       move to a stored pose
sequence <ID...>        move through stored poses in the given order
save <ID> [name]        save current 8-axis joint values
list                    list poses
show <ID>               show one pose
delete <ID>             delete pose (Pose 0 is protected)
teach <target> on       enter teach mode: torque OFF
teach <target> off      exit teach mode: torque ON + current-position hold
torque <target> on/off  raw torque control
status                  show current 8-axis state
help
q | quit | exit
```

Targets:

```text
test / left / right / all
```

Example:

```text
arm> save 1 wave
arm> show 1
arm> pose 1 20
arm> sequence 0 1 2 3 2 1 0
arm> teach left on
arm> teach left off
```

`sequence` validates every pose ID before motion starts, then waits for each
`MoveJoint` Action to complete successfully before sending the next pose. The
sequence stops on the first error or timeout. It currently uses
`default_pose_speed_dps`; per-sequence speed selection is not yet supported.

The sequential command has been implemented and passes the package build. Mock
and real-hardware motion validation are still pending.

When a stored pose value is `null`, that motor is not given a new pose target; its current calibrated position is held instead.

---

## Teach Mode

Teach mode is intended for future hand-guided pose recording.

### Teach ON

```text
STOP
  ↓
Torque OFF
  ↓
User manually positions the arm
  ↓
Current encoder/joint values remain readable
  ↓
Pose can be saved
```

### Teach OFF

The controller does not blindly restore the previous command target.

Instead:

```text
read current 0x92 position
        ↓
Torque ON
        ↓
command the current position as HOLD target
```

This is designed to reduce the risk of the arm snapping back toward an old target after hand-guiding.

If teach-mode transition fails, the controller attempts a best-effort fallback to torque OFF.

> **Safety:** torque OFF does not mean motor power is disconnected. Communication and encoder reads remain available, but holding torque is released. A physical robot arm may drop under gravity, so the arm must be mechanically supported during real teach-mode testing.

While teach mode or torque OFF is active, `MoveJoint` motion requests are rejected.

---

## Getting Started

### 1. Build

The active Raspberry Pi workspace is:

```text
~/iroi_ws
```

Always build from the workspace root:

```bash
cd ~/iroi_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Do **not** build from `~/iroi_ws/src`.

If this repository is checked out in a differently named workspace, use that
workspace root instead. For example, the current Ubuntu development checkout
uses `~/ros2_ws`, so its build command starts with `cd ~/ros2_ws`.

### 2. Files to open when developing

Open the workspace root (`~/iroi_ws` on the Raspberry Pi or `~/ros2_ws` on the
current Ubuntu development machine) in your editor. The main files are:

| Purpose | File |
| --- | --- |
| Pose/teach/sequence CLI commands | `src/motor_control_pkg/motor_control_pkg/arm_pose_cli.py` |
| One `MoveJoint` motion and motor safety gates | `src/motor_control_pkg/motor_control_pkg/motor_control_node.py` |
| Pose JSON loading and saving | `src/motor_control_pkg/motor_control_pkg/pose_manager.py` |
| Dual-arm mock startup | `src/motor_control_pkg/launch/dual_arm_pose_framework.launch.py` |
| Verified three-motor real-hardware fallback | `src/motor_control_pkg/launch/three_motor_real.launch.py` |
| Three-motor pose-framework development launch | `src/motor_control_pkg/launch/three_motor_pose_framework.launch.py` |

Runtime poses are stored in `~/.ros/arm_poses.json`. This is runtime data, not
the repository template, and should normally be changed through `arm_pose_cli`
instead of edited by hand.

### 3. Run the mock pose/sequence workflow (no motors required)

After a successful build, open two terminals. Every newly opened terminal must
source ROS2 and the workspace before using package commands.

Terminal 1 — start the two mock motor-control nodes and startup-pose node:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch motor_control_pkg dual_arm_pose_framework.launch.py
```

Terminal 2 — open the interactive pose CLI:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run motor_control_pkg arm_pose_cli --ros-args -p mode:=dual
```

Inside `arm_pose_cli`, a minimal command-flow check is:

```text
arm> save 1 mock_step_1
arm> save 2 mock_step_2
arm> list
arm> sequence 0 1 2 1 0
```

Mock mode checks pose lookup, Action completion, ordering, and error handling.
It does not validate physical clearance, gravity, calibration, or motor safety.

### 4. Scan motors

```bash
ros2 run motor_control_pkg scan_ids --port /dev/ttyUSB0
```

### 5. Read-only homing calculation

```bash
ros2 run motor_control_pkg check_home --id 4 --port /dev/ttyUSB0
```

`check_home` calculates the expected homing target without sending a move command.

### 6. Verified three-motor real-hardware launch

```bash
ros2 launch motor_control_pkg three_motor_real.launch.py
```

This is the current hardware-verified fallback path for IDs 1, 2, and 4 and physically homes them to the saved encoder-zero references.

### 7. Persistent direct bench CLI

```bash
ros2 run motor_control_pkg arm_cli
```

Input format:

```text
ID1_angle  ID2_angle  ID4_angle  speed
```

Example:

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

Installed console scripts include:

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

### RS485 concurrent access

Each LC529 bus is half-duplex. Serial read/write sections are protected with a lock so polling, services, and Action execution do not interleave packets on the same bus.

### Failed reads are not valid `0.0°` measurements

Communication failures are treated as errors. Motion should not start from a failed baseline, and calibration should not persist a failed read as a physical zero.

### Calibration must remain per motor

The final system mixes i10 and i36 reductions. `ratio`, `loop_period_deg`, speed limits, and zero references must come from each motor's configuration rather than a shared global assumption.

### Runtime pose file vs repository template

`motor_control_pkg/config/poses.json` is the repository template. The runtime pose CLI/startup node uses:

```text
~/.ros/arm_poses.json
```

This prevents locally taught poses from being unintentionally committed to Git.

---

## Roadmap

- [x] ROS2 node architecture and mock validation
- [x] Dual-arm namespace design
- [x] Raspberry Pi 4 + LC529 + real motor communication
- [x] Real-hardware absolute-encoder automatic homing
- [x] Real-hardware ROS2 Action target-angle motion
- [x] Persistent `arm_cli`
- [x] Multi-motor validation on one RS485 bus
- [x] Three-axis synchronized arrival
- [x] 8-axis pose database framework
- [x] Pose save/list/show/playback in mock mode
- [x] Teach-mode motion interlock in mock mode
- [x] Teach OFF current-position hold logic in mock mode
- [x] Sequential pose command implementation and package build
- [ ] Validate sequential pose playback in mock mode
- [ ] Validate sequential pose playback on real hardware
- [ ] Map all final motor IDs to physical joints
- [ ] Calibrate every final motor's absolute encoder zero
- [ ] Validate MG5010E-i36 ratio / wrap behavior
- [ ] Validate mixed i10/i36 operation
- [ ] Validate `reference_only` startup on real hardware
- [ ] Record full 8-axis Pose 0 (`attention`)
- [ ] Validate teach mode on a mechanically supported physical arm
- [ ] Validate one complete 4-axis arm
- [ ] Validate left/right dual RS485 buses
- [ ] Integrate and validate all 8 motors
- [ ] Switch final startup to reject incomplete Pose 0 (`allow_partial_pose=False`)
- [ ] Long-duration stability test
- [ ] MoveIt2 / RViz integration

---

## Tech Stack

`ROS2 Humble` `Python 3` `pyserial` `Raspberry Pi 4` `RS485` `LC529`
