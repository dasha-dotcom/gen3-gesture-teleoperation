# Human Gesture-Based Robot Telemanipulation

A PIONEER Lab / NJIT research practice project exploring webcam-detected hand motion to teleoperate a KINOVA Gen3 for simulated pick-and-place tasks.

**September 21, 2026 milestone: calibrated 3-DOF translational webcam control of a mock Gen3 with gesture gripper control and automatic singularity recovery.**

The Mac camera tracks a palm with MediaPipe. A UDP message carries normalized image position, an apparent-palm-width depth proxy, and a stabilized hand gesture into an Ubuntu VM. A hold-to-run interface maps that input to MoveIt Servo while RViz displays the mock robot state.

```text
Mac camera → MediaPipe → UDP palm position/depth proxy + gesture
           → Ubuntu combined bridge → MoveIt Servo / gripper action
           → joint controllers → mock Gen3 + Robotiq 2F-85 → RViz
```

## What works so far

- ROS 2 Jazzy fundamentals and the original `hand_status_demo` package.
- Seven-joint Gen3 + Robotiq 2F-85 model with mock controllers, MoveIt planning/execution, and gripper commands.
- Python endpoint-offset exercises in base-frame x/y/z and keyboard Servo controls.
- MediaPipe camera tracking on the Mac with hand-data delivery to Ubuntu over UDP.
- Calibrated **x/y/z translational teleoperation** while holding Space:
  - image horizontal position → robot y,
  - image vertical position → robot z,
  - calibrated apparent palm width → monocular x-depth proxy.
- A neutral dead zone and combined 3D normalization so simultaneous x/y/z requests stay within the configured Servo translational scale.
- Stabilized OPEN/CLOSED gesture classification controlling the Robotiq gripper.
- Stop behavior on Space release, window focus loss, hand loss, stale hand data, and sender/session interruption.
- MoveIt Servo singularity-halt detection with automatic webcam-control disable, Servo pause, MoveIt trajectory to the Gen3 `Home` state, Servo unpause, and mandatory recalibration before control can resume.

**These are mock-hardware results.** No physics-based grasping, physical robot performance, or real-robot stopping behavior has been validated. The monocular x signal is an apparent-size proxy rather than metric depth from a depth camera.

## Start here

- [Environment and focused build](docs/setup.md)
- [Restart, operation, and stopping behavior](docs/mock-teleoperation.md)
- [Architecture and design choices](docs/architecture.md)
- [Script inventory and checks](scripts/README.md)
- [September 19 milestone notes](notes/2026-09-19.md)
- [Earlier research log](notes/2026-09-10.md)

The current scripts are exercises for this specific Mac/VM setup. The restart guide assumes the previously prepared ROS environment; it is not yet a verified clean-machine installer.

## Repository layout

```text
ros2_ws/src/hand_status_demo/  Original ROS publisher/subscriber exercises
scripts/mac/                 Camera sender and deliberate message-gap test
scripts/ubuntu/              Combined bridge, Servo config/launch, earlier exercises
tests/                       Dependency-free hand-mapping and gating checks
docs/                        Setup, architecture, and restart instructions
notes/                       Dated progress and limitations
```

Only source/configuration and documentation belong here. Virtual environments and generated ROS `build/`, `install/`, and `log/` directories are excluded.

## Current control mapping

```text
hand closer / farther → +x / -x depth command
hand right / left     → +/-y
hand up / down        → +/-z
OPEN                   → gripper open
CLOSED                 → partial gripper close
C                      → save neutral hand position and palm width
hold Space             → enable arm + gripper control
release Space          → stop teleoperation
```

The x-axis mapping is calibrated relative to the apparent palm width at the moment `C` is pressed. It is therefore suitable for the current monocular prototype but should not be treated as true metric camera depth.

## Roadmap

- Stages 0–1: environment and ROS foundations exercised.
- Stage 2: Gen3 mock model/controllers/MoveIt exercised; physics simulation remains separate.
- Stage 3: control without vision exercised with offsets and keyboard input.
- Stages 4–7: webcam tracking, transport, hold-to-run safety gating, and calibrated x/y/z mapping integrated.
- Stage 8: OPEN/CLOSED gesture gripper control integrated.
- **Next: task-level simulated pick-and-place and workspace/task constraints.**
- After that: interface evaluation, latency/jitter/stopping measurements, final demo, and documentation cleanup.

Orientation control is optional future work rather than part of the current translational milestone. Planned-motion action cancellation also remains a separate unresolved issue from the Servo teleoperation path.

## Reference project

Dr. Lin's reference example is [FrankaTeleop](https://github.com/gjcliff/FrankaTeleop), with its [project writeup](https://graham-clifford.com/Robot-Arm-Teleoperation-Through-Computer-Vision-Hand-Tracking/). It informs the separation of perception, mapping, and robot control. This repository uses a Gen3, ROS 2 Jazzy, a Mac camera, and an Ubuntu ARM64 VM.
