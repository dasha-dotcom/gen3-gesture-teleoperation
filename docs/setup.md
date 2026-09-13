
## Verified Local Development Environment

Host:
- Apple M3 Mac
- macOS
- UTM / QEMU

Guest:
- Ubuntu 24.04.5 LTS
- ARM64 / aarch64

ROS:
- ROS 2 Jazzy
- Installed using official ROS Ubuntu packages

MoveIt:
- MoveIt 2 installed
- MoveIt Setup Assistant launches successfully
- moveit_servo installed and discoverable by ROS

ROS workspace:
- ros2_ws/
- colcon build verified
- hand_status_demo package verified

External Kinova workspace:
- ~/workspace/ros2_kortex_ws
- ros2_kortex Jazzy source obtained
- dependencies imported with vcstool

Known limitation:
- Full ros2_kortex build has not been attempted on this ARM64 VM.
- Kinova integration will be verified on the appropriate x86-64 Ubuntu lab environment.
