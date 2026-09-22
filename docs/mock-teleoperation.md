# Mock Gen3 pick-and-place: startup and operation

Current procedure for the September 22, 2026 milestone on Dasha's Mac + Ubuntu ARM64 VM. This assumes the prepared [environment](setup.md), not a verified clean-machine installation. All robot behavior is mock hardware; planning-scene attachment is logical simulated grasping without contact physics.

## Files and prerequisites

Use the current repository checkout at `~/gen3-gesture-teleoperation` on both computers. Commands below run the tracked files directly; the earlier exercise copies under `~/gen3_exercises` are unnecessary for this procedure. Keep `gen3_servo.launch.py` beside `gen3_servo.yaml`. The Mac Python environment remains `~/gen3_camera/.venv311`.

Use one robot launch, one Servo launch, one combined bridge, one watcher, and one Mac sender. Close older keyboard/hand controllers and UDP receivers. The current pair is `hand_combined_sender.py` + `gen3_combined_bridge.py`, using UDP **5007**, not the older port-5005 y/z pair.

The bridge binds `192.168.64.2:5007` and accepts Mac source IP `192.168.64.1`. Check Ubuntu `hostname -I`; if addresses changed, update the sender destination and bridge bind/source filter consistently. Camera access stays on the Mac.

**In every new Ubuntu terminal**, first run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/gen3_viewer_ws/install/local_setup.bash
cd ~/gen3-gesture-teleoperation
```

## Full startup after restarting Ubuntu / MoveIt

### 1. Robot, MoveIt, and RViz — Ubuntu terminal A

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py \
  robot_ip:=xxx.yyy.zzz.www \
  use_fake_hardware:=true \
  use_sim_time:=false \
  launch_rviz:=true
```

Leave running. The placeholder IP is for mock hardware. This supplies the robot model, controllers, MoveIt, TF, and RViz.

### 2. Check controllers — Ubuntu terminal B

```bash
ros2 control list_controllers -c /controller_manager
```

Expected active controllers: `joint_state_broadcaster`, `joint_trajectory_controller`, and `robotiq_gripper_controller`. If the gripper is already active, skip the workaround below.

The installed setup needed a temporary replacement for an unavailable gripper plugin:

```bash
ros2 param set /controller_manager robotiq_gripper_controller.type position_controllers/GripperActionController
ros2 run controller_manager spawner robotiq_gripper_controller \
  -c /controller_manager \
  -p ~/workspace/gen3_viewer_ws/install/kinova_gen3_7dof_robotiq_2f_85_moveit_config/share/kinova_gen3_7dof_robotiq_2f_85_moveit_config/config/ros2_controllers.yaml
ros2 control list_controllers -c /controller_manager
```

Confirm all three are active before continuing.

### 3. Servo — Ubuntu terminal C

```bash
ros2 launch ~/gen3-gesture-teleoperation/scripts/ubuntu/gen3_servo.launch.py
```

Leave running. The launch loads the adjacent YAML and the external robot description/MoveIt configuration. Key settings are `manipulator`, unitless input, linear scale 0.02 m/s, publish period 0.02 s, command timeout 0.1 s, position trajectories to `/joint_trajectory_controller/joint_trajectory`, and enabled smoothing/collision checks.

### 4. Combined bridge — Ubuntu terminal D

```bash
python3 scripts/ubuntu/gen3_combined_bridge.py
```

Leave the hold-to-run window open with Space released. The bridge checks mock arm/gripper hardware and Servo scaling, selects Twist mode, and unpauses Servo. It needs both the gripper action server and MoveIt Home recovery services/action.

### 5. Cube and drop zone — sourced Ubuntu terminal B

```bash
python3 scripts/ubuntu/pick_place_scene.py add
python3 scripts/ubuntu/pick_place_scene.py allow-gripper-contact
python3 scripts/ubuntu/drop_zone_scene.py add
```

Verify the green cube and blue target in RViz. Restarting MoveIt clears the planning scene, so recreate both objects and the gripper contact allowances afterward. Do not add/reset a cube while it is attached; use the reset procedure below.

### 6. Watcher — Ubuntu terminal E

```bash
python3 scripts/ubuntu/pick_place_watch.py
```

Leave running. It monitors the actual gripper joint and planning scene, independently of Space. CLOSED within the 8 cm fingertip-midpoint radius attaches the cube; OPEN while attached detaches it and checks the drop zone. It continues checking proximity while CLOSED, so attachment need not occur exactly at the close transition.

### 7. Camera sender — Mac terminal

```bash
cd ~/gen3-gesture-teleoperation
source ~/gen3_camera/.venv311/bin/activate
python scripts/mac/hand_combined_sender.py --send
```

Without `--send` this script only previews. Keep one tracking hand visible, palm toward the camera. Packets contain session, sequence, hand flag, palm center, pixel palm width, and stabilized gesture at up to about 20 Hz. Q closes the camera preview.

### 8. Calibrate and operate

1. Focus the Ubuntu combined-bridge window with Space released.
2. Hold the palm at a comfortable neutral position, away from image edges (u/v each 0.2–0.8), and press **C**. This saves both position and apparent width.
3. Confirm near-zero motion preview at rest. Hold **Space** to enable arm and gripper input.
4. Move the hand right/left for +y/−y, up/down for +z/−z, closer/farther for +x/−x via apparent width. Release Space to disable input.
5. Approach the cube OPEN, then make a CLOSED gesture while enabled. Wait for `PICK COMPLETE` before transporting it.
6. Move over the blue target, make OPEN while enabled, and inspect `PLACE COMPLETE` followed by `TASK SUCCESS` or `TASK MISSED`.

Neutral dead zones are ±0.1 for u/v and width ratio 0.90–1.10. z is scaled by 2/3 before the 3D normalization. The total requested speed is capped at 20 mm/s with this Servo configuration; actual speed depends on Servo limits/smoothing. Width is not metric depth, and rotating the palm can change x input. Angular requests remain zero.

OPEN targets 0.0 rad and CLOSED targets 0.4 rad (partial close). UNCERTAIN sends no new gripper goal but can still permit arm translation. Space release prevents new teleoperation input; an already-issued gripper action can still finish. These are not physical emergency-stop guarantees.

## Manual Home move and standardized trial reset

Use the same starting configuration for each recorded trial: robot **Home**, gripper **OPEN**, cube center **(0.537, 0.004, 0.327) m**, drop-zone center **(0.50, −0.18, 0.20) m**, with both objects created in `base_link`.

1. If holding a cube, open the gripper using the enabled gesture control with the palm near neutral; wait for detachment, then release Space. Ensure the gripper is OPEN. If the controller is faulted, resolve/restart it before beginning another trial.
2. With Space released, pause Servo in a sourced Ubuntu terminal:

   ```bash
   ros2 service call /servo_node/pause_servo std_srvs/srv/SetBool "{data: true}"
   ```

   Require `success: true`. Releasing Space alone is insufficient for a planned move because the bridge still publishes zero Twist commands.
3. In RViz MotionPlanning, select the `manipulator` group and named goal **Home**, use the current robot state as start, then **Plan & Execute**. Wait for successful completion. Do not start a competing Home move during automatic recovery.
4. Resume Servo only after the planned move completes:

   ```bash
   ros2 service call /servo_node/pause_servo std_srvs/srv/SetBool "{data: false}"
   ```

   Require `success: true`. Pausing leaves the Servo process running; Ctrl+C would terminate it.
5. With gripper OPEN and cube detached, reset the cube:

   ```bash
   python3 scripts/ubuntu/pick_place_scene.py remove
   python3 scripts/ubuntu/pick_place_scene.py add
   python3 scripts/ubuntu/pick_place_scene.py allow-gripper-contact
   ```

   Leave the existing drop zone unchanged; if absent after a scene restart, recreate it with `python3 scripts/ubuntu/drop_zone_scene.py add`. The `remove` operation removes the world object, not an attached object. For a manual attachment cleanup, stop the watcher, use `pick_place_scene.py detach`, then remove/add, ensure the gripper is OPEN, and restart the watcher.
6. Confirm Home/open gripper/cube/target visually, refocus the bridge, release Space, press **C**, and start the next trial with a fresh Space press.

The watcher, sender, bridge, and Servo can stay running during a normal reset with an OPEN gripper. A CLOSED gripper can trigger the independent watcher, so avoid resetting in that state. After any manual scene manipulation, restart the watcher if its holding state no longer matches the scene.

The target is 14 × 14 cm. Success tests the cube center against ±7 cm x/y bounds, not height, full-cube containment, or physical resting contact. Record watcher pick/place timestamps and final dx/dy; planar error is `sqrt(dx² + dy²)`. Record misses/re-grasps separately and distinguish successful re-pick timing from the full recovery interval. See [the five-trial record](../notes/2026-09-22.md).

## Singularity recovery and restarts

On a Servo singularity halt, the bridge automatically disables webcam input, invalidates calibration, pauses Servo, and executes a MoveIt Home trajectory. It then unpauses Servo and stays disabled until **release Space → C → fresh Space press**. Space release does not stop that planned Home motion. On failure, input remains faulted and Servo may remain paused; inspect the reported error and recover/restart manually before resuming.

| Event | Required action |
|---|---|
| Hand loss, >0.5 s accepted-data gap, focus loss, Escape, delayed GUI, unexpected subscriber count | Resolve cause, release Space, then fresh press; recalibrate if neutral changed |
| Mac sender restart / changed session | Restart combined bridge and recalibrate; sender sequence/session are new |
| Bridge restart | Recalibrate; startup selects Twist and unpauses Servo |
| Servo restart | With Space released, restart bridge to recheck/select Twist/unpause, then recalibrate |
| MoveIt / VM restart | Full startup; recreate cube, contact allowances, and drop zone; restart watcher |
| Gripper failure or recovery failure | Resolve reported fault and restart bridge; do not assume action completion |

For `Command type has not been set, cannot accept input`, first check for an unintended Servo restart or duplicate node. With Space released, this selects Twist if needed:

```bash
ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"
```

Normal bridge startup already does this. It is **not** a required step after every Home move. Confirm success and unpause only when no planned move is active, then recalibrate.

## Shutdown and evidence limits

Release Space, close the bridge, press Q in the Mac preview, stop the watcher, then stop Servo and the robot launch with Ctrl+C when ending the session. Closing RViz alone does not stop controllers. If Home/gripper actions are in progress, confirm their status; closing a window or requesting action cancellation is not proof of a stopped trajectory.

The milestone was exercised interactively in the user's VM. This documentation update does not rerun ROS or validate a clean installation. Exact stopping latency, packet delay, drift, physical contact, and real-robot readiness remain unvalidated. Earlier planned-motion action cancellation was ineffective in an observed test; details and the separately observed stop-event mechanism remain in [scripts/README](../scripts/README.md).

Mac environment: Python 3.11, MediaPipe 0.10.21, OpenCV contrib 4.11.0.86. The MediaPipe wheel's platform-tag warning remains documented in [setup](setup.md).
