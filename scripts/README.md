# Script inventory

These scripts support the existing mock Gen3 setup. Start with [setup](../docs/setup.md) and the [restart guide](../docs/mock-teleoperation.md).

| File | Runs on | Purpose |
|---|---|---|
| `mac/hand_combined_sender.py` | Mac | Current palm position, pixel width, and stabilized gesture sender; use `--send`, UDP 5007 |
| `mac/hand_sender.py` | Mac | Earlier y/z-only palm sender, UDP 5005 |
| `mac/hand_sender_gap_test.py` | Mac | One deliberate 3-second packet pause, 20 seconds after the first processed frame |
| `mac/requirements.txt` | Mac | Recorded Python 3.11 direct dependencies |
| `ubuntu/gen3_combined_bridge.py` | Ubuntu | Current calibrated x/y/z + gesture gripper bridge with automatic singularity Home recovery |
| `ubuntu/gen3_hand_bridge_v3.py` | Ubuntu | Earlier calibrated hold-to-run y/z bridge, UDP 5005 |
| `ubuntu/pick_place_scene.py` | Ubuntu | Add/remove the 5 cm cube, manual attach/detach, and gripper-only contact allowances |
| `ubuntu/pick_place_watch.py` | Ubuntu | Watch gripper state/fingertip proximity, attach/detach cube, and evaluate target placement |
| `ubuntu/drop_zone_scene.py` | Ubuntu | Add/remove the blue 14 × 14 cm target |
| `ubuntu/gen3_servo.launch.py` | Ubuntu | Starts Servo alongside the existing mock robot launch |
| `ubuntu/gen3_servo.yaml` | Ubuntu | Matching scale, topics, model group, timeouts, smoothing and collision settings |
| `ubuntu/gen3_keyboard.py` | Ubuntu | Earlier W/S x, A/D y, R/F z exercise; retains its two-second hold cap |
| `ubuntu/gen3_move_offset_v2.py` | Ubuntu | Earlier MoveIt endpoint planning/execution and cancellation diagnostic |

Keep the Servo launch and YAML together. Use only one command source at a time; a disabled bridge still publishes zeros. The combined bridge has no two-second hold cap. Pause Servo around RViz Home moves as described in the restart guide.

## Pick-and-place utilities

From the repository root in a sourced Ubuntu terminal with MoveIt running:

```bash
python3 scripts/ubuntu/pick_place_scene.py add
python3 scripts/ubuntu/pick_place_scene.py allow-gripper-contact
python3 scripts/ubuntu/drop_zone_scene.py add
python3 scripts/ubuntu/pick_place_watch.py
```

The watcher stays running. The cube utility also accepts `remove`, `attach`, and `detach`; the zone utility accepts `remove`. Stop the watcher before manual attach/detach operations to avoid competing scene updates. Detach an attached cube before world removal/reset. See the [trial reset procedure](../docs/mock-teleoperation.md#manual-home-move-and-standardized-trial-reset).

These helpers implement logical planning-scene grasping without contact physics. Success tests released cube-center x/y bounds only, not height or full-cube containment.

## Automated logic checks

From the repository root, with ordinary Python 3 (no ROS/camera dependencies required):

```bash
python3 -m unittest discover -s tests -v
```

The tests load only the pure `Gate` and mapping definitions from the **earlier `gen3_hand_bridge_v3.py`**, without starting its ROS node, socket, or GUI. They cover calibrated symmetry, the diagonal speed cap, hand loss, stale messages, invalid coordinates, and explicit re-enabling. They do not test the current combined bridge, gesture/depth mapping, Home recovery, or pick/place watcher, and do not validate ROS, focus/key events, camera access, network timing, or physical stopping latency.

## Earlier keyboard exercise

Stop the hand bridge before using the keyboard. With the mock robot and Servo running, source ROS/overlay and select Twist mode/unpause:

```bash
ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"
ros2 service call /servo_node/pause_servo std_srvs/srv/SetBool "{data: false}"
python3 ~/gen3_exercises/gen3_keyboard.py
```

Use the supplied Servo YAML: the keyboard does not independently check the configured scaling. Click Enable keys and hold one direction key. Release to stop; Escape/focus loss disables. Its two-second cap requires enabling again.

## Earlier endpoint exercise

Stop the hand bridge/keyboard and Servo before testing planned trajectories. Keep the combined robot/MoveIt launch running. From a sourced Ubuntu terminal:

```bash
python3 ~/gen3_exercises/gen3_move_offset_v2.py x 0.01
python3 ~/gen3_exercises/gen3_move_offset_v2.py x 0.01 --execute
```

The first plans only. The second plans again and asks for `MOVE` before executing that plan. The target is calculated from current reported state; it is an endpoint constraint, not a straight Cartesian-path guarantee.

**Cancellation remains a diagnostic limitation:** Ctrl+C or `--cancel-after` requests action cancellation, but the observed action-cancel response arrived only after execution completed. Do not interpret the helper's exit as proof that motion stopped. The independently tested planned-trajectory stop mechanism was:

```bash
ros2 topic pub --once /trajectory_execution_event std_msgs/msg/String "{data: stop}"
```

That test produced controller cancellation and MoveIt PREEMPTED. This planned-trajectory mechanism is distinct from the hand bridge's zero requests and Servo command timeout. The scripts do not establish emergency-stop behavior.

## Earlier y/z message-gap test

Use `hand_sender_gap_test.py` only with the earlier `gen3_hand_bridge_v3.py` port-5005 pair, with the combined controller stopped. It does not send the combined session/gesture/width protocol. Already observed successfully in the VM; no need to repeat after every documentation change. To reproduce intentionally, replace the normal Mac sender with `hand_sender_gap_test.py` and restart the Ubuntu bridge because sequence numbers reset. Keep a hand visible and hold Space before the displayed pause. Expected: the bridge reports a hand-data gap, disables, and stays disabled when messages resume. Release and press Space to enable again. Close the test sender before returning to the normal sender, and restart/recalibrate the bridge again.
