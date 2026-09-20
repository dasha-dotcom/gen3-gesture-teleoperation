from pathlib import Path
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    model = (
        MoveItConfigsBuilder(
            "gen3",
            package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config",
        )
        .robot_description(mappings={
            "robot_ip": "xxx.yyy.zzz.www",
            "use_fake_hardware": "true",
            "gripper": "robotiq_2f_85",
            "gripper_joint_name": "robotiq_85_left_knuckle_joint",
            "dof": "7",
            "gripper_max_velocity": "100.0",
            "gripper_max_force": "100.0",
            "use_internal_bus_gripper_comm": "true",
        })
        .to_moveit_configs()
    )

    config_path = Path(__file__).resolve().parent / "gen3_servo.yaml"
    with config_path.open() as config_file:
        servo_settings = yaml.safe_load(config_file)

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            model.robot_description,
            model.robot_description_semantic,
            model.robot_description_kinematics,
            model.joint_limits,
            {"moveit_servo": servo_settings},
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([servo_node])
