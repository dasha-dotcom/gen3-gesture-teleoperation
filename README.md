# Human Gesture-Based Robot Telemanipulation

A PIONEER Lab / NJIT research practice project exploring webcam-detected human hand/arm motion to teleoperate a KINOVA Gen3 robotic arm for simulated pick-and-place tasks.

The project is simulation-first. The eventual interface should provide intuitive start/pause behavior, safe hand-to-robot motion mapping, and natural gripper control.

**Current status: Stage 0 — Environment and repository setup (in progress).**

## Environment

### Development Machine

- Host: Apple Silicon Mac (M3)
- Host OS: macOS
- Virtualization: UTM / QEMU
- Guest OS: Ubuntu 24.04.5 LTS
- Guest architecture: ARM64 / `aarch64`

### ROS Environment

- ROS distribution: ROS 2 Jazzy
- ROS installation: installed and verified inside the Ubuntu VM
- ROS sourcing:
  ```bash
  source /opt/ros/jazzy/setup.bash