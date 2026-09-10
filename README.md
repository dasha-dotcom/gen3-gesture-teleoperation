# Human Gesture-Based Robot Telemanipulation

A PIONEER Lab / NJIT research practice project exploring webcam-detected human hand/arm motion to teleoperate a KINOVA Gen3 robotic arm for simulated pick-and-place tasks.

The project is simulation-first. The eventual interface should provide intuitive start/pause behavior and natural gripper control.

**Current status: Stage 0 — Environment and repository setup.**

## Environment

Development machine:
- macOS personal computer

Target robotics environment:
- OS: TBD — awaiting advisor confirmation
- ROS distribution: TBD
- KINOVA ros2_kortex branch: TBD
- MoveIt version/configuration: TBD
- Gen3 configuration: TBD
- Gripper: TBD

ROS/Kinova installation has intentionally NOT been performed on macOS while the lab environment is being confirmed. The decision between a lab Ubuntu workstation/environment and a self-configured Ubuntu 24.04 + ROS 2 Jazzy setup is pending advisor confirmation; neither is an approved target yet.

## Planned Architecture

```text
Webcam
→ Hand tracking / MediaPipe
→ Gesture + motion logic
→ ROS 2 messages
→ MoveIt / controller
→ KINOVA Gen3 simulation
→ Pick-and-place task
```

This is a planned architecture; no robotics or perception dependencies are installed as part of this repository setup. See [architecture](docs/architecture.md) for future subsystem responsibilities.

## Development Strategy

1. Prove each subsystem independently.
2. Do not debug vision, ROS communication, robot control, and gripper logic simultaneously.
3. Build the smallest working version first.
4. Commit known-working milestones to Git.
5. Record non-obvious setup commands and version information.

## Roadmap

- Stage 0 — Environment + project skeleton
- Stage 1 — ROS 2 fundamentals
- Stage 2 — Gen3 simulation
- Stage 3 — Robot control without vision
- Stage 4 — Webcam hand tracking
- Stage 5 — Gesture/interface state machine
- Stage 6 — Publish perception into ROS
- Stage 7 — Map hand motion to arm motion
- Stage 8 — Gripper integration
- Stage 9 — Pick-and-place
- Stage 10 — Interface enhancement/evaluation
- Stage 11 — Documentation/demo

## Project Documentation

- [Setup and environment decisions](docs/setup.md)
- [Planned subsystem architecture](docs/architecture.md)
- [September 10, 2026 research log](notes/2026-09-10.md)
- [Future reproducible scripts](scripts/README.md)

`ros2_ws/src/` is reserved for future ROS 2 packages. It is currently an empty placeholder, not a configured or built ROS workspace.
