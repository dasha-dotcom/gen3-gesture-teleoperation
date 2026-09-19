# Human Gesture-Based Robot Telemanipulation

A PIONEER Lab / NJIT research practice project exploring webcam-detected hand motion to teleoperate a KINOVA Gen3 for simulated pick-and-place tasks.

**September 19, 2026 milestone: calibrated webcam control of mock Gen3 motion in y/z.**

The Mac camera tracks a palm with MediaPipe. A small UDP message carries its normalized coordinates into an Ubuntu VM, where a hold-to-run interface feeds MoveIt Servo. RViz displays the resulting robot state.

```text
Mac camera → MediaPipe → UDP palm coordinates → Ubuntu hand bridge
           → MoveIt Servo → joint trajectory controller → mock Gen3 → RViz
```

## What works so far

- ROS 2 fundamentals and the original `hand_status_demo` package.
- Seven-joint Gen3 + Robotiq 2F-85 model, mock controllers, MoveIt planning/execution, and gripper open/partial-close commands.
- Python endpoint-offset exercises in base-frame x/y/z and keyboard Servo controls.
- Camera tracking on the Mac and hand-data delivery into Ubuntu.
- Calibrated y/z velocity requests while holding Space; a neutral rest zone and balanced positive/negative scaling.
- Observed stop on hand loss and on a deliberate message gap; returning hand/data stayed disabled until a fresh enable action.

**These are mock-hardware results.** No physics-based grasping, physical robot performance, or real-robot stopping behavior has been validated. The arm/gripper choice still needs confirmation against the lab setup.

## Start here

- [Environment and focused build](docs/setup.md)
- [Restart, operation, and stopping behavior](docs/mock-teleoperation.md)
- [Architecture and design choices](docs/architecture.md)
- [Script inventory and checks](scripts/README.md)
- [September 19 milestone and remaining work](notes/2026-09-19.md)
- [Earlier research log](notes/2026-09-10.md)

The current scripts are exercises for this specific Mac/VM setup. The restart guide assumes the previously prepared ROS environment; it is not yet a verified clean-machine installer.

## Repository layout

```text
ros2_ws/src/hand_status_demo/  Original ROS publisher/subscriber exercises
scripts/mac/                 Camera sender and deliberate message-gap test
scripts/ubuntu/              Hand bridge, Servo config/launch, earlier exercises
tests/                       Dependency-free hand-mapping and gating checks
docs/                        Setup, architecture, and restart instructions
notes/                       Dated progress and limitations
```

Only source/configuration and documentation belong here. Virtual environments and generated ROS `build/`, `install/`, and `log/` directories are excluded.

## Roadmap

The original stages remain useful, but development now overlaps them:

- Stages 0–1: environment and ROS foundations exercised.
- Stage 2: Gen3 mock model/controllers/MoveIt exercised; physics simulation remains separate.
- Stage 3: control without vision exercised with offsets and keyboard input.
- Stages 4–7: webcam tracking, enable/stop logic, transport, and y/z mapping integrated as an initial prototype.
- Stage 8: direct gripper commands exercised; gesture gripper integration is next.
- Stages 9–11: pick-and-place, interface evaluation, and final demo/documentation remain.

Webcam x control, orientation control, gesture gripper commands, quantified stopping/latency tests, and task-level pick-and-place remain open. Planned-motion action cancellation also remains unresolved; see the [milestone notes](notes/2026-09-19.md).

## Reference project

Dr. Lin's reference example is [FrankaTeleop](https://github.com/gjcliff/FrankaTeleop), with its [project writeup](https://graham-clifford.com/Robot-Arm-Teleoperation-Through-Computer-Vision-Hand-Tracking/). It informs the separation of perception, mapping, and robot control. This repository uses a Gen3, ROS 2 Jazzy, a Mac camera, and an Ubuntu ARM64 VM.
