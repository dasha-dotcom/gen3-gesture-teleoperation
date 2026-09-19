# Gen3 camera teleoperation: working milestone

Recorded September 19, 2026. Dasha's home Mac + Ubuntu VM setup.

For copying the tracked scripts to these exercise directories, see [setup](setup.md). For code provenance and remaining checks, see the [milestone record](../notes/2026-09-19.md).

## What is working

Mac built-in camera → MediaPipe palm coordinates → UDP → Ubuntu hand bridge → MoveIt Servo → Gen3 mock hardware → RViz.

The hand bridge controls translation in base-frame y and z. It sends zero x and angular velocity requests. This is velocity control from hand displacement relative to a calibrated neutral point, not a hand-position-to-tool-position correspondence or an orientation-hold controller.

All robot results below are from mock hardware. The seven-joint Gen3 and Robotiq 2F-85 configuration still needs confirmation against the intended lab setup. No physics-based grasping or physical robot performance has been validated.

## Which computer owns which files?

| Location | Files / purpose |
|---|---|
| **Mac** `~/gen3_camera/` | `hand_sender.py`, `.venv311/`, optional `hand_sender_gap_test.py` |
| **Ubuntu VM** `~/gen3_exercises/` | `gen3_hand_bridge_v3.py`, `gen3_servo.yaml`, `gen3_servo.launch.py`, earlier keyboard/offset exercises |
| **Ubuntu VM** `~/workspace/gen3_viewer_ws/` | Built robot description and MoveIt configuration packages |
| **Ubuntu VM** `~/workspace/ros2_kortex_ws/src/` | Existing Kinova/Robotiq sources used by the focused build |

These paths are on two separate operating systems. Creating a Mac sender under Ubuntu does not give it access to the Mac camera.

## Before starting

- Use one robot launch, one Servo launch, one hand bridge, and one Mac sender.
- Close earlier keyboard controllers and UDP preview receivers. The bridge owns UDP port 5005.
- Do not execute an offset script or an RViz trajectory while the hand bridge controls Servo. It publishes zero requests even while disabled.
- If programs are already running, do not start duplicates.
- The bridge currently binds `192.168.64.2:5005` and accepts packets from `192.168.64.1`. The sender targets `192.168.64.2:5005`.
- In Ubuntu, `hostname -I` should still include `192.168.64.2`. If networking changes, update the addresses consistently before using the scripts.

## Startup after restarting the VM

### 1. Ubuntu terminal A — robot, MoveIt, and RViz

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/gen3_viewer_ws/install/local_setup.bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py \
  robot_ip:=xxx.yyy.zzz.www \
  use_fake_hardware:=true \
  use_sim_time:=false \
  launch_rviz:=true
```

Leave this terminal running. The placeholder IP is used with mock hardware. This launch supplies the robot model, controller manager, joint-state broadcaster, arm trajectory controller, MoveIt, TF, and RViz.

### 2. Ubuntu terminal B — temporary gripper fix

The configuration asks for a gripper-controller plugin that was unavailable in the installed controller types. The working temporary replacement is `position_controllers/GripperActionController`.

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/gen3_viewer_ws/install/local_setup.bash
ros2 param set /controller_manager robotiq_gripper_controller.type position_controllers/GripperActionController
ros2 run controller_manager spawner robotiq_gripper_controller \
  -c /controller_manager \
  -p ~/workspace/gen3_viewer_ws/install/kinova_gen3_7dof_robotiq_2f_85_moveit_config/share/kinova_gen3_7dof_robotiq_2f_85_moveit_config/config/ros2_controllers.yaml
ros2 control list_controllers -c /controller_manager
```

Expected active controllers: `joint_state_broadcaster`, `joint_trajectory_controller`, and `robotiq_gripper_controller`. If the gripper is already active, skip setting/spawning it again. Gesture-based gripper commands are not implemented yet.

### 3. Ubuntu terminal C — Servo

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/gen3_viewer_ws/install/local_setup.bash
ros2 launch ~/gen3_exercises/gen3_servo.launch.py
```

Leave it running. The launch reads `gen3_servo.yaml` beside it. It supplies Servo with the Gen3 model, semantic description, kinematics, and joint limits. It adds Servo to the existing robot setup.

Key exercise settings:

- `move_group_name: manipulator`
- `command_in_type: unitless`
- `scale.linear: 0.02`
- `publish_period: 0.02`
- `incoming_command_timeout: 0.1`
- `command_out_topic: /joint_trajectory_controller/joint_trajectory`
- Position output enabled; velocity and acceleration output disabled.
- Monitored planning scene: `/monitored_planning_scene`; primary monitor false.
- Smoothing and collision checks enabled.

### 4. Ubuntu terminal D — hand bridge

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/gen3_viewer_ws/install/local_setup.bash
python3 ~/gen3_exercises/gen3_hand_bridge_v3.py
```

The bridge checks active mock arm hardware and the expected Servo input scaling, selects Twist mode, and unpauses Servo. It opens a hold-to-run window. Wait for data before enabling motion.

### 5. Mac Terminal — camera sender

```bash
cd ~/gen3_camera
source .venv311/bin/activate
python hand_sender.py
```

The normal sender has no intentional pause. It displays a mirrored camera preview and sends a sequence number, hand-detected flag, and normalized palm center at up to approximately 20 messages/second. Video is not transmitted to Ubuntu. Keep only the intended tracking hand visible.

### 6. Calibrate and operate

1. Focus the Ubuntu bridge window, with Space released.
2. Hold your palm comfortably and tap **C**. Calibration requires the palm coordinates to be between 0.2 and 0.8 on both axes, away from the image edges.
3. Confirm that the hand preview is zero near this resting position.
4. Hold **Space** in the Ubuntu window to enable motion; release to stop requesting motion.
5. Palm right/left of neutral requests +y/−y. Palm above/below neutral requests +z/−z. Moving diagonally requests both.

Neutral means your calibrated palm position, not necessarily the image center. Each axis has its own rest zone (±0.1 normalized image coordinate). Equal offsets from neutral give equal-magnitude positive/negative requests along that axis. The combined requested speed is capped at 10 mm/s with the checked Servo scaling. Actual robot motion can differ because of timing, smoothing, and limits.

There is no hold-duration limit in v3. Calibration is held in memory and must be repeated after restarting the bridge.

## Stopping behavior and restart rules

| Condition | Intended bridge behavior |
|---|---|
| Space released | Send zero; next fresh press can enable |
| Hand leaves camera view | Clear enable state and send zero |
| No new accepted message for over 0.5 s | Clear enable state and send zero |
| Hand/messages return | Stay disabled until release and fresh Space press |
| Window loses focus or Escape pressed | Clear enable state and send zero |
| Window update gap over 0.15 s | Clear enable state and show delay reason |
| Servo subscriber count differs from one | Clear enable state and send zero |
| Window closes normally | Send zero requests briefly before exiting |

Space-release handling includes a 40 ms key-repeat debounce. None of these settings is a measured physical stopping time.

Servo additionally has its own 0.1 s command-age timeout if the bridge stops sending. The bridge's 0.5 s timeout applies to hand-data arrivals and is a separate layer.

The protocol currently checks increasing sequence numbers and receipt freshness. It does not authenticate the sender or measure camera-capture-to-robot latency; a new arrival is not proof of a recently captured frame. This is an exercise protocol for the local mock setup.

**If the Mac sender restarts, restart the bridge before starting it again.** The sender's sequence number resets to zero; the bridge otherwise rejects those lower numbers. Recalibrate afterward. A bridge restart while the sender continues also requires recalibration.

## Shutdown

1. Release Space.
2. Close the Ubuntu hand-bridge window.
3. Press **Q** in the Mac camera window.
4. If ending the session, stop the Servo launch with Ctrl+C, then the robot launch with Ctrl+C.

Closing RViz alone is not the same as stopping the robot/controller launch.

## Verification evidence so far

- Robot model and seven arm joints visible in RViz; mock controllers active.
- MoveIt planning/execution, direct arm trajectories, and gripper open/partial-close commands succeeded.
- Offset helper verified base-frame x/y/z endpoint movements; a straight tool path was not enforced.
- Planned-motion execution-action cancellation remained ineffective in the observed test: cancellation was answered after execution finished.
- A separate `stop` message on `/trajectory_execution_event` interrupted planned motion; the joint controller canceled and MoveIt reported PREEMPTED. A subsequent fresh movement succeeded.
- Servo short velocity input and explicit-zero stop tested.
- Basic Servo input-disappearance test produced a stationary pose afterward; exact stop latency was not measured.
- Keyboard x/y/z movement and focus-loss disabled indication tested.
- Mac camera/MediaPipe hand tracking and UDP connection to Ubuntu tested.
- Calibrated hand y/z control moved the mock robot. Space-release status and stable final TF samples observed.
- Hand removed while Space held: user observed stopping; hand return did not re-enable motion.
- Timed sender stopped messages for three seconds while tracking continued: user observed the specific hand-data-gap stop message; resumed messages did not re-enable motion.

Latest isolated z recording: approximately 17.137 mm downward and 10.533 mm upward; y changed 0.114 mm, x varied 0.003 mm, net orientation changed about 0.029 degrees; final pose stable for about three seconds. Unequal travel does not imply unequal gain without matched input amplitudes and times.

An earlier mixed-axis recording showed about 1.4 degrees net orientation change. Its cause remains unresolved. Zero angular velocity requests do not provide active correction back to a fixed orientation.

## Known limitations and next work

- Latest normal sender preserves the previously exercised sending loop and removes only the test pause; syntax checked, fresh end-to-end run still to be confirmed.
- ROS/GUIs run in the user's Ubuntu VM; local automated checks covered mapping/gating logic, not the complete ROS stack.
- Exact stop latency, packet-delay behavior, long-duration drift, and all fault paths are not quantified.
- Continuous commands currently cover y/z only. Webcam-derived forward/backward x control, orientation control, and gesture gripper control remain to implement.
- No reliable emergency-stop claim, physical grasp/contact validation, or real-robot readiness claim follows from these mock tests.
- Next development step: identify open/closed-hand gestures as displayed labels first, then integrate gripper commands deliberately.

## Mac dependency notes

Python 3.11 virtual environment `.venv311`; MediaPipe 0.10.21; OpenCV contrib 4.11.0.86 (import prints 4.11.0).

Python 3.9 installation failed on newer syntax in a JAX dependency. The replacement environment imports successfully. MediaPipe's installed wheel metadata declares an Intel tag despite the main binary including ARM64 support, so `pip check` reports a platform warning. Blank-image inference and live camera hand tracking both worked on this Mac. The warning has not been hidden or repaired.

Dependencies can be recorded without downloading anything:

```bash
# Mac, with .venv311 active
python -m pip freeze > ~/gen3_camera/requirements-mac-snapshot.txt
```

This snapshot is for reproducing the Mac environment, not an Ubuntu requirements file.
