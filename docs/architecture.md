# Current architecture

This describes the completed September 22, 2026 mock-hardware milestone. Earlier y/z-only exercises remain in the repository as historical examples.

## 1. Mac perception and packet contents

`hand_combined_sender.py` uses MediaPipe on one mirrored camera image. Landmarks 0, 5, 9, 13, and 17 are averaged into normalized palm center `(u, v)`. The pixel distance between landmarks 5 and 17 is `palm_width`.

A four-finger angle/curl heuristic uses MediaPipe world landmarks to classify OPEN/CLOSED; it ignores the thumb and is not a trained gesture recognizer. A label must persist for at least 0.3 seconds and three observations; ambiguous geometry is UNCERTAIN. No visible hand produces NO HAND. Hand orientation, occlusion, and camera conditions affect recognition.

With `--send`, JSON packets are sent at up to about 20 Hz:

| Field | Meaning |
|---|---|
| `session` | 32-character UUID string generated at sender startup |
| `seq` | Increasing integer within the session |
| `hand` | Boolean palm-detection flag |
| `u`, `v` | Normalized image coordinates; checked in [0, 1] when a hand is visible |
| `palm_width` | Apparent width in pixels; must be finite and positive for a visible hand |
| `gesture` | OPEN, CLOSED, UNCERTAIN, or NO HAND |

Only these measurements/labels are transmitted, not video or robot targets. Without `--send`, the sender is preview-only.

## 2. UDP and input gating

The sender targets `192.168.64.2:5007`; `gen3_combined_bridge.py` binds there and filters for source IP `192.168.64.1`. These are local VM assumptions. The older y/z pair uses port 5005 and is not the current path.

Sequence checks reject repeated/out-of-order messages. A changed session latches a fault requiring receiver restart and recalibration. Packets have no authentication or capture timestamp: fresh receipt does not prove fresh capture. Source-IP filtering is not authentication.

C with Space released saves a recent visible palm's neutral u/v and width (u/v must each be between 0.2 and 0.8). Hold Space to enable. Hand loss, a hand-data gap over 0.5 s, focus loss, Escape, GUI delay over 0.15 s, or a Servo subscriber count other than one disables input. Returned hand/data does not automatically re-enable motion. Sender restart and gripper/recovery faults require receiver restart; normal interruptions require release and a fresh press.

## 3. Calibrated translation and gripper commands

Mapping is relative hand displacement to base-frame velocity:

- Horizontal palm displacement → y, with a ±0.1 normalized-coordinate dead zone.
- Inverted vertical displacement → z, with the same dead zone and a 2/3 scale factor.
- `palm_width / calibrated_width` → x. Ratios 0.90–1.10 request zero; larger ratios ramp toward +1 at 1.60, smaller toward −1 at 0.60.

The combined x/y/z vector is normalized to magnitude at most 1. With unitless Servo input and `scale.linear: 0.02`, the maximum requested translational magnitude is 0.02 m/s (20 mm/s). This is not measured speed. Apparent width is a monocular depth proxy affected by hand rotation and shape, not calibrated metric depth. Zero angular input does not actively restore a fixed orientation.

The bridge sends OPEN → 0.0 rad and CLOSED → 0.4 rad (partial close) through `/robotiq_gripper_controller/gripper_cmd` (`control_msgs/action/GripperCommand`). A changed valid label can issue a goal only while enabled, calibrated, and receiving fresh hand data. UNCERTAIN issues no new gripper target but does not itself disable arm translation. Duplicate labels are suppressed and one gripper action runs at a time. Rejection, unsuccessful completion, or a result timeout faults the bridge. Disabling input does not cancel an already-issued gripper action.

## 4. Servo, mock controllers, and Home recovery

Startup checks active `mock_components/GenericSystem` arm and gripper hardware, unitless Servo input, and 0.02 linear scale; it selects Twist command type 1 and unpauses Servo.

The bridge publishes freshly stamped `geometry_msgs/TwistStamped` on `/servo_node/delta_twist_cmds` in `base_link`. Servo uses current joint states, kinematics, smoothing, joint limits, and the monitored planning scene, with collision checking enabled. It outputs position trajectories to `/joint_trajectory_controller/joint_trajectory`. The joint-state broadcaster and robot-state publisher supply state/TF for MoveIt and RViz. Servo's separate command-age timeout is 0.1 s; it is not the bridge's hand-data timeout or a measured stopping time.

On `HALT_FOR_SINGULARITY` from Servo, the bridge disables teleoperation, clears calibration, pauses Servo, and asks MoveIt `/move_action` to plan/execute the Gen3 Home joint state. Successful recovery unpauses Servo but requires Space release, C, and a fresh press. Failure remains latched for manual recovery/restart; Servo may remain paused. This is automatic planned motion, not an emergency stop. Space release gates webcam commands, not the Home trajectory.

Manual RViz Home moves also require pausing Servo: even a disabled bridge publishes zeros. The [restart guide](mock-teleoperation.md) gives the pause/execute/unpause procedure. Do not run competing command sources.

## 5. Planning-scene pick-and-place

`pick_place_scene.py` creates `pick_cube`, a green 0.05 m box at `(0.537, 0.004, 0.327)` m in `base_link`. Its `allow-gripper-contact` action updates the allowed-collision matrix for the listed Robotiq/gripper links (including the mounting end-effector link). It preserves other collision checks rather than disabling Servo collision checking globally.

`pick_place_watch.py` is separate from the teleoperation bridge. It reads `/joint_states`, classifying `robotiq_85_left_knuckle_joint` as OPEN at ≤0.05 rad and CLOSED at ≥0.30 rad. While CLOSED and not holding, it rechecks proximity every 0.25 s. This watcher has no Space gate; it reacts to reported gripper state, including during other robot motion.

The watcher queries `/get_planning_scene`, transforms both fingertip links into the cube's returned frame (which can be `world`), and compares their midpoint with `cube.pose.position`. In the current single-box scene, MoveIt returns the actual object center there; the primitive pose is relative to the object. Distance ≤0.08 m triggers an `AttachedCollisionObject` update via `/apply_planning_scene`, attaching to `end_effector_link` while preserving relative pose. Opening detaches the object back into the world at its current pose. This represents grasping logically; no contact, friction, support surface, or gravity is simulated.

`drop_zone_scene.py` creates the blue `drop_zone`, a 0.14 × 0.14 × 0.01 m box centered at `(0.50, -0.18, 0.20)` m in `base_link`. It is a collision object placed below the working height, not just a non-colliding RViz graphic.

After detachment, the watcher reads cube and zone object poses in the same frame. Success requires `abs(dx) <= 0.07 m` and `abs(dy) <= 0.07 m`, using half of the zone's x/y dimensions. This is an axis-aligned square center test, not a radial-distance threshold, full-cube containment check, or height/contact test. The reported planar error is `sqrt(dx² + dy²)`. A miss leaves the cube at its released world pose for re-grasp; missing objects or mismatched frames produce a warning instead of a valid result.

## 6. Evidence and limits

The [September 22 note](../notes/2026-09-22.md) records five standardized trials, including one miss and recovery. This small mock-hardware validation set establishes the observed end-to-end logical task only. RViz is visualization, not a physics simulator; there is no real-robot validation, force feedback, measured emergency-stop performance, or general performance claim.

Other limits include unpinned upstream versions, provisional lab robot/gripper match, unquantified network/GUI delay and drift, no hand orientation control, and the earlier planned-motion action-cancellation issue. The dependency-free tests cover the earlier y/z bridge, not the combined controller or scene watcher.
