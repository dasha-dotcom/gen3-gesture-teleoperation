# Current architecture

## 1. Perception on the Mac

MediaPipe detects one hand in a mirrored camera image. The sender averages landmarks 0, 5, 9, 13, and 17 to estimate a palm center `(u, v)`, normalized to the image dimensions. It sends coordinates and a detection flag, not video or robot targets, at up to about 20 Hz.

This keeps camera access on macOS while ROS runs inside Ubuntu.

## 2. UDP transport into Ubuntu

Each JSON packet has an increasing integer `seq` and boolean `hand`; a visible hand also has numeric `u` and `v` in `[0, 1]`.

The current sender targets `192.168.64.2:5005`. The bridge binds there and filters for Mac source address `192.168.64.1`. These are local setup assumptions, not universal VM addresses. Increasing sequence numbers reject repeated/out-of-order packets. Restart the bridge when restarting the sender, because the sender counter resets.

There is no authentication or capture timestamp. Fresh receipt does not prove fresh camera capture, especially if packets were queued. This remains a local mock exercise protocol.

## 3. Mapping and enable state in Ubuntu

The operator focuses the bridge window, releases Space, and presses C at a comfortable palm position to set neutral. Each axis has a ±0.1 rest zone. Beyond it, hand displacement requests velocity, with symmetric scaling around the chosen center:

- Palm right/left → base-frame +y/−y.
- Palm up/down → base-frame +z/−z.
- x and angular velocity requests stay zero.

The two-axis request is capped to magnitude 0.5 in unitless Servo input. With linear scale 0.02 m/s, that is at most 0.01 m/s (10 mm/s) requested translation. The displayed value is a command, not measured speed. Zero angular input is not an active orientation-restoration controller.

Space is a hold-to-run input. Hand loss or an accepted-message gap over 0.5 seconds clears enable state. Data returning does not enable motion again; the operator must release and press Space. Focus loss, Escape, delayed GUI updates, and an unexpected subscriber count also disable it. See the restart guide for the evidence and timing limitations.

## 4. Servo and robot control

The bridge publishes `geometry_msgs/TwistStamped` to `/servo_node/delta_twist_cmds`, with `base_link` as the command frame and a fresh ROS timestamp. Servo uses the model, current joint states, kinematics, and planning scene to generate short joint trajectories. Collision checking and smoothing are enabled in the exercise config.

Servo publishes `trajectory_msgs/JointTrajectory` to `/joint_trajectory_controller/joint_trajectory`. The active controller drives `mock_components/GenericSystem`; the joint-state broadcaster publishes `/joint_states`, and the robot-state publisher derives link transforms for RViz.

Mock hardware allows command/interface testing without a physical arm. It does not model contact forces or prove grasping behavior. RViz is the visualization, not a physics simulator.

The separate endpoint-offset exercise instead asks MoveIt to plan a path and execute a complete trajectory. A reachable endpoint does not imply a straight tool path. Do not run that exercise or RViz execution while the hand bridge is publishing Servo commands.

## Next integration

Classify open/closed-hand intent as a displayed label first. Connect deliberate gripper commands only after classification behavior is understood. Webcam x motion, orientation control, and pick-and-place objects are later work.
