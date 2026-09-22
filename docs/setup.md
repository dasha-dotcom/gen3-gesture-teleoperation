# Environment and setup record

## Verified development environment

| Component | Current setup |
|---|---|
| Host | Apple M3 Mac, macOS |
| Camera Python | Python 3.11 virtual environment in `~/gen3_camera/.venv311` |
| Perception | MediaPipe 0.10.21, OpenCV contrib 4.11.0.86 |
| Virtualization | UTM / QEMU |
| Guest | Ubuntu 24.04.5 LTS, ARM64 / aarch64 |
| Robotics | ROS 2 Jazzy, MoveIt 2, MoveIt Servo, ros2_control |
| Robot model | Gen3 seven joints + Robotiq 2F-85, provisional lab match |
| Hardware backend | `mock_components/GenericSystem` |

Package revisions for the complete ROS installation and upstream source commits have not yet been pinned. This documents the existing working setup, rather than claiming a tested fresh installation.

## External robot workspace

Existing sources live at:

```text
~/workspace/ros2_kortex_ws/src/ros2_kortex
~/workspace/ros2_kortex_ws/src/ros2_robotiq_gripper
```

The successful focused build uses only description/configuration packages:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/workspace/gen3_viewer_ws
cd ~/workspace/gen3_viewer_ws
colcon build --base-paths \
  ~/workspace/ros2_kortex_ws/src/ros2_kortex/kortex_description \
  ~/workspace/ros2_kortex_ws/src/ros2_robotiq_gripper/robotiq_description \
  ~/workspace/ros2_kortex_ws/src/ros2_kortex/kortex_moveit_config/kinova_gen3_7dof_robotiq_2f_85_moveit_config \
  --executor sequential
source install/local_setup.bash
```

`--base-paths` limits discovery to these packages; `--executor sequential` builds one at a time. The workspace overlay makes them discoverable alongside system ROS packages. Earlier broader builds failed; a complete hardware-driver build is not established by this focused success.

Installed tools used along the way include `joint_state_publisher_gui`, `ros2controlcli`, controller manager, joint trajectory controller, joint-state broadcaster, position gripper controller, MoveIt, Servo, and Ubuntu Tkinter. `mock_components` is a plugin supplied by `hardware_interface`, not a separate package to locate with `ros2 pkg prefix`.

## Put the scripts in the exercise directories

From the repository root **on the Mac**, copy only the Mac scripts:

```bash
mkdir -p ~/gen3_camera
cp scripts/mac/hand_combined_sender.py scripts/mac/hand_sender.py scripts/mac/hand_sender_gap_test.py ~/gen3_camera/
```

From the repository root **inside Ubuntu**, copy the Ubuntu scripts/configuration:

```bash
mkdir -p ~/gen3_exercises
cp scripts/ubuntu/*.py scripts/ubuntu/*.yaml ~/gen3_exercises/
```

These copy commands replace matching exercise files. If your working copies have newer edits, compare them first. Download or check out this repository in both systems; their home directories are separate. The current pair is `hand_combined_sender.py --send` on the Mac and `gen3_combined_bridge.py` in Ubuntu (UDP 5007). The `v3` y/z bridge and its port-5005 sender remain earlier exercises; `v2` identifies the offset helper. The restart guide runs tracked files directly from the repository, so copying to these exercise directories is optional.

The Servo configuration and launch are preserved from the exercise instructions. This repository capture has not independently compared them with the VM's current files or relaunched the full stack.

## Mac dependencies

Use the existing `.venv311` environment if it works. For a new environment, these are the recorded direct dependencies (installation on a clean machine has not been re-tested):

```bash
python3.11 -m venv ~/gen3_camera/.venv311
source ~/gen3_camera/.venv311/bin/activate
python -m pip install -r scripts/mac/requirements.txt
python -c "import mediapipe as mp; import cv2; print(mp.__version__, cv2.__version__)"
```

Run these from the repository root. Python 3.9 failed on dependency syntax; Python 3.11 imports and live hand tracking worked. The installed MediaPipe wheel has an inconsistent platform tag, so `pip check` reported that it was unsupported despite working imports/inference on this Mac. This warning remains documented, not repaired. The requirements file pins direct dependencies only, not the whole transitive environment.

## Run the system

Follow [the restart guide](mock-teleoperation.md) for separate robot, controller check/temporary gripper fix, Servo, combined bridge, scene setup, watcher, and camera terminals, plus Home/trial resets. Always source ROS and the viewer overlay in fresh Ubuntu exercise terminals.
