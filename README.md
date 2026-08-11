[한국어](./README.ko.md) | **English**

# iRoi — Dual-Arm Robot ROS2 Motor Control

> ROS2 (Humble) control stack for an 8-DOF dual-arm robot built on LK-TECH RS485 servo motors — migrated from a standalone Python control script.

**Status: 🚧 In Progress** — Full functionality verified in `mock_mode` (simulation). Physical hardware validation (Raspberry Pi 4 + 8 motors) pending.

---

## Why ROS2?

The original implementation was a Python script (`motor_control.py`) using file-based command polling (`target_batch.json`). This became hard to manage as the motor and arm count grew — concurrency and timing had to be handled manually for every new feature.

- Requirements like restart-safety and synchronized multi-motor motion are already well-covered by ROS2's standard primitives (Topic QoS, Service, Action).
- Given the end goal — 8 motors across two arms, remote control from a Raspberry Pi, and future MoveIt2/RViz integration — a node-based architecture with standard communication patterns made sense from the start.
- The already-validated low-level RS485 driver (`lk_motor.py`) was kept as-is; ROS2 was layered on top as a **wrapper**, not a rewrite.

---

## System Architecture

```
Laptop (development / remote control)
   │ ROS2 DDS (Ethernet/WiFi)
   ▼
Raspberry Pi 4 Model B (4GB)
   ├── USB-RS485 #1 (LC529) ──▶ Left arm, 4 motors  (motor_id 1-4)
   └── USB-RS485 #2 (LC529) ──▶ Right arm, 4 motors (motor_id 5-8)
```

The same `motor_control_node` runs twice — once per arm — differing only in `arm_name`, `serial_port`, and `motor_ids`, and is fully namespaced in the launch file.

```
[left_arm]  motor_control_node  (motor_ids=[1,2,3,4], /dev/ttyUSB0)
[right_arm] motor_control_node  (motor_ids=[5,6,7,8], /dev/ttyUSB1)
        │
        ├── Topic   /{arm}/joint_states  → real-time joint angle publishing
        ├── Service /{arm}/torque        → torque on/off
        ├── Service /{arm}/set_zero      → software zero-point calibration
        └── Action  /{arm}/move_to       → synchronized multi-motor motion
        │
        ▼
   MoveIt2 / RViz  (motion planning, integration planned)
```

---

## ROS2 Interfaces

| Type | Name | Message Type | Description |
|---|---|---|---|
| Topic | `/{arm}/joint_states` | `sensor_msgs/JointState` | Publishes joint angles at `polling_hz`, deg→rad converted |
| Service | `/{arm}/torque` | `std_srvs/SetBool` | Motor torque on/off |
| Service | `/{arm}/set_zero` | `std_srvs/Trigger` | Sets current position as zero, persisted to disk and auto-restored on restart |
| Action | `/{arm}/move_to` | `iroi_interfaces/action/MoveJoint` | Synchronized multi-motor motion (target duration derived from the slowest motor, other speeds solved backward) |

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

All runtime behavior (`serial_port`, `baudrate`, `arm_name`, `motor_ids`, `joint_names`, `polling_hz`, `max_speed_dps`, `mock_mode`, `zero_config_path`) is exposed as ROS2 parameters — nothing is hardcoded.

---

## Hardware

| Item | Detail |
|---|---|
| Motors | LK-TECH MG5010E-i10 / MG4010E-i10, 10:1 reduction, dual magnetic absolute encoders (18-bit motor-side + 14-bit output-side) |
| Communication | RS485, 115200 bps (configurable up to 4 Mbps) |
| Converter | USB-RS485 (Coms LC529) ×2, one independent bus per arm |
| Controller | Raspberry Pi 4 Model B (4GB), Ubuntu + ROS2 Humble |

---

## Getting Started

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select iroi_interfaces motor_control_pkg
source install/setup.bash

# Run a single node in mock mode (no hardware required)
ros2 run motor_control_pkg motor_control_node --ros-args \
  -p arm_name:=right_arm -p serial_port:=/dev/ttyUSB1 -p motor_ids:="[5,6,7,8]" -p mock_mode:=true

# Launch both arms together (real-hardware mode is the launch default)
ros2 launch motor_control_pkg dual_arm.launch.py

# Scan for actual motor IDs on a bus (verifies DIP-switch labels against real responses)
ros2 run motor_control_pkg scan_ids --port /dev/ttyUSB0

# Torque on/off
ros2 service call /left_arm/torque std_srvs/srv/SetBool "{data: true}"

# Set zero point
ros2 service call /left_arm/set_zero std_srvs/srv/Trigger "{}"

# Synchronized multi-motor move
ros2 action send_goal /left_arm/move_to iroi_interfaces/action/MoveJoint \
  "{target_angles: [30.0, 60.0, 90.0, 15.0], max_speeds: [20.0, 20.0, 20.0, 20.0]}" --feedback
```

---

## Project Structure

```
ros2_ws/src/
├── motor_control_pkg/
│   ├── motor_control_pkg/
│   │   ├── lk_motor.py           # Low-level RS485 driver (ported unchanged)
│   │   ├── motor_control_node.py # Main ROS2 node (Topics/Services/Actions)
│   │   └── scan_ids.py           # Motor ID diagnostic script
│   └── launch/
│       └── dual_arm.launch.py    # Boots both arms together
└── iroi_interfaces/
    └── action/
        └── MoveJoint.action      # Custom action for synchronized motion
```

---

## Engineering Notes — Issues Found and Fixed Before Hardware Connection

A code review before hardware arrival surfaced several issues that `mock_mode` alone could never expose. Documenting them here as evidence of the kind of failure mode that only shows up once physics enters the picture.

**⚠️ Highest priority for hardware validation — angle frame mismatch (fixed, not yet hardware-verified)**
Angle values read via the power-cycle-persistent absolute frame (0x94) were being passed directly into the move command that expects the power-cycle-relative accumulated frame (0x92). These two frames can diverge across power cycles; mixing them risks commanding the motor to an unintended position. Fixed by computing a `shortest_delta` in the 0x94 frame and applying it on top of the current 0x92 value before issuing a move. This class of bug is invisible in `mock_mode`, since the simulation doesn't distinguish between the two frames at all — it must be the first thing verified once real motors are connected.

**RS485 concurrent access**
The polling timer and the action server could reach the same `serial.Serial` object from different threads. Since RS485 is half-duplex, concurrent access risked interleaving requests and responses. Fixed by wrapping every communication point (polling, torque, set_zero, move, stop, shutdown) in a `threading.Lock`.

**Namespace collision between arms**
The launch file distinguished nodes only by `name`, not `namespace`, so left and right arm nodes shared the same global topics/services. Fixed by adding `namespace='left_arm'`/`'right_arm'`; verified with `ros2 topic list` / `ros2 service list` after the fix.

**Silent zero-fill on communication failure**
Failed motor reads were silently replaced with `0.0`. Fixed so failed reads are tracked explicitly: `set_zero` now refuses to save if any motor read fails, and `move_to` aborts before starting rather than moving from a bad baseline.

---

## Roadmap

- [x] ROS2 node architecture designed, full scenario coverage in `mock_mode`
- [x] Dual-arm launch with namespace separation
- [ ] First real-motor validation with 3 motors on Raspberry Pi 4
- [ ] Tune `polling_hz` from measured data (current 30 Hz is a theoretical starting point; RS485 half-duplex timing needs real-world confirmation)
- [ ] Full 8-motor (both arms) integration and 30+ minute stability test
- [ ] MoveIt2 / RViz integration

---

## Tech Stack

`ROS2 Humble` `Python 3` `pyserial` `Raspberry Pi 4` `RS485`
