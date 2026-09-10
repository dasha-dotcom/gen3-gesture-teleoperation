# Planned Architecture

This simulation-first architecture describes future responsibilities. No subsystem has been implemented yet; dependency versions and robot configuration remain TBD.

```text
Webcam
→ Perception: hand tracking / MediaPipe
→ Gesture/interface logic: start, pause, motion intent, gripper intent
→ ROS communication: publish perception and interface state
→ Teleoperation mapping: convert hand motion to robot targets
→ Robot control: MoveIt / controller
→ Simulation: KINOVA Gen3 and task objects
→ Pick-and-place task
```

## Perception

Capture webcam frames and estimate hand/arm observations for downstream logic. MediaPipe is the planned hand-tracking component; its integration and version will be determined later.

## Gesture/Interface Logic

Interpret observations as operator intent. Define intuitive start/pause behavior and natural gripper control, including how to pause when tracking is lost.

## ROS Communication

Transport perception results, interface state, and commands between future ROS 2 nodes. Message types, topics, and update rates are TBD.

## Teleoperation Mapping

Convert human hand/arm motion into robot motion targets. Define coordinate frames, scaling, calibration, and workspace limits after the robot configuration is confirmed.

## Robot Control

Use MoveIt and the selected controller interface to execute robot targets and gripper commands in simulation. First verify control independently of vision. Versions and configuration are TBD.

## Simulation

Represent the KINOVA Gen3, the selected gripper, and pick-and-place objects in the confirmed simulation environment. Verify arm and gripper behavior before connecting the full perception pipeline. Simulator choice and Gen3 configuration remain TBD.
