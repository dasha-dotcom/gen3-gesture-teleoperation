# Human Gesture-Based Robot Telemanipulation

A PIONEER Lab / NJIT research practice project exploring webcam-based hand tracking for teleoperation of a KINOVA Gen3 robot arm.

**September 22, 2026 milestone: completed end-to-end gesture-controlled pick-and-place with mock Gen3 + Robotiq 2F-85 hardware, ROS 2 Jazzy, MoveIt 2 / MoveIt Servo, MediaPipe, and RViz.**

```text
Mac webcam → MediaPipe palm position + apparent palm width + gesture
           → UDP → Ubuntu combined bridge
           → MoveIt Servo (arm) + gripper action → mock Gen3 + Robotiq
           → planning-scene watcher: attach → transport → detach → evaluate
           → RViz visualization
```

## Completed system

- Calibrated 3-DOF translational control: palm horizontal motion → robot y, vertical motion → z, apparent palm width → x depth proxy.
- Stabilized OPEN/CLOSED gestures → gripper open/partial-close.
- C to calibrate; hold Space to enable arm and gripper commands; release to stop teleoperation input.
- Hand loss, stale data, focus loss, and sender/session interruption disable input. An already-issued gripper action may still complete.
- MoveIt Servo collision checking and automatic singularity Home recovery: disable teleoperation, pause Servo, execute a MoveIt Home trajectory, unpause, then require recalibration.
- Green 5 cm pick cube with gripper-only collision allowances; automatic pickup within 8 cm of the fingertip midpoint while CLOSED.
- Logical planning-scene attachment to carry the cube and detachment on OPEN.
- Blue 14 × 14 cm target with automatic placement evaluation after detachment.

**This is logical simulated grasping in the RViz/MoveIt planning scene, not contact physics or real-robot validation.** There is no gravity, force feedback, or proof that a physical gripper would grasp the object. Success means the cube center is within the target's x/y bounds (±7 cm per axis); height and full-cube containment are not checked.

## Validation results

Five standardized trials used the same Home start, open gripper, cube location, and target location. An earlier run from a different start was excluded. See the [milestone note](notes/2026-09-22.md) for method and debugging decisions.

| Standardized trial | Outcome | Successful pickup → successful placement | Final planar error |
|---|---|---:|---:|
| 1 | Success | 37.15 s | 2.20 cm |
| 2 | Success after one missed placement/recovery | 74.90 s* | 0.60 cm |
| 3 | Success | 79.24 s | 2.20 cm |
| 4 | Success | 42.30 s | 6.62 cm |
| 5 | Success | 45.30 s | 1.12 cm |

*Trial 2: 74.90 s is the successful re-pick → final placement interval; first pickup → final success took 109.30 s. The mean below uses 74.90 s, not the full recovery sequence.

- **5/5** tasks ultimately completed; **4/5** first-placement successes; **1/5** recovery after a miss.
- Successful pickup → successful placement: mean **55.78 s**, median **45.30 s**.
- Final planar error: mean **2.55 cm**, median **2.20 cm**, best **0.60 cm**, worst successful **6.62 cm**.

This is a **small mock-hardware validation set**, not a rigorous performance evaluation. These intervals exclude the initial approach and unsuccessful grasp attempts before the successful pickup; first-placement success does not mean first-grasp success.

## Control mapping

| Input relative to calibration | Command |
|---|---|
| Hand closer / farther (larger / smaller apparent palm width) | +x / −x |
| Hand right / left in mirrored preview | +y / −y |
| Hand up / down | +z / −z |
| OPEN / CLOSED | Gripper 0.0 / 0.4 rad target |
| C with Space released | Save neutral palm position and width |
| Hold / release Space | Enable / disable teleoperation input |

This is displacement-to-velocity mapping, not hand-position-to-tool-position tracking. The monocular x signal is an apparent-size depth proxy, not metric depth; hand rotation can affect it. Combined translation is capped at a requested 20 mm/s with the checked-in Servo configuration; z is scaled lower. Angular commands are zero, without active orientation restoration.

## Start here

- [Environment and focused build](docs/setup.md)
- [Startup, restart, Home moves, and trial reset](docs/mock-teleoperation.md)
- [Architecture and limitations](docs/architecture.md)
- [Script inventory and checks](scripts/README.md)
- [September 22 milestone and results](notes/2026-09-22.md)
- Earlier records: [September 19](notes/2026-09-19.md), [September 10](notes/2026-09-10.md)

The restart guide assumes the prepared Mac/Ubuntu ARM64 VM environment. A clean-machine installation and upstream package revisions are not yet verified/pinned.

## Repository layout

```text
ros2_ws/src/hand_status_demo/  Original ROS publisher/subscriber exercises
scripts/mac/                 Combined camera sender and earlier exercises
scripts/ubuntu/              Combined bridge, Servo config, scene helpers/watcher
tests/                      Earlier bridge mapping/gating checks
docs/                       Setup, architecture, and operation
notes/                      Dated milestones and results
```

Generated ROS build/install/log directories and virtual environments are excluded.

## Milestone scope and future work

ROS foundations, mock controllers/MoveIt, webcam transport, calibrated x/y/z mapping, gesture gripper control, singularity recovery, and planning-scene pick-and-place are integrated. The simulation-first milestone is complete.

Future work includes metric depth sensing, orientation control, physics/contact simulation, larger controlled evaluations, and measured latency/drift/stopping behavior. Planned-motion action cancellation remains an unresolved diagnostic limitation; it is separate from Servo input stopping. Lab robot/gripper compatibility and physical grasping must be established before any real-robot claims. These mock tests do not establish emergency-stop performance or real-robot readiness.

## Reference project

Dr. Lin's reference example is [FrankaTeleop](https://github.com/gjcliff/FrankaTeleop), with its [project writeup](https://graham-clifford.com/Robot-Arm-Teleoperation-Through-Computer-Vision-Hand-Tracking/). It informed the separation of perception, mapping, and robot control; this implementation uses the Gen3, ROS 2 Jazzy, a Mac webcam, and an Ubuntu ARM64 VM.
