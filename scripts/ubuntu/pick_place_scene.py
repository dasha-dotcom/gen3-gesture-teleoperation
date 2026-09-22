"""Manage a simple pick-and-place cube in MoveIt's planning scene.

This is a mock-hardware planning-scene exercise. It does not simulate physics.

Actions:
- add: place the cube into the world
- remove: remove the cube completely
- attach: attach the existing world cube to the Robotiq end effector
- detach: detach the cube and return it to the world at its current pose
- allow-gripper-contact: allow only the Robotiq/gripper links to contact the cube
"""

import argparse

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    ObjectColor,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA


OBJECT_ID = "pick_cube"
FRAME_ID = "base_link"
END_EFFECTOR_LINK = "end_effector_link"

# Links that may legitimately touch the cube during a grasp. Other robot
# links still treat pick_cube as a normal collision object.
TOUCH_LINKS = (
    "end_effector_link",
    "robotiq_85_base_link",
    "robotiq_85_left_inner_knuckle_link",
    "robotiq_85_left_knuckle_link",
    "robotiq_85_left_finger_link",
    "robotiq_85_left_finger_tip_link",
    "robotiq_85_right_inner_knuckle_link",
    "robotiq_85_right_knuckle_link",
    "robotiq_85_right_finger_link",
    "robotiq_85_right_finger_tip_link",
)

# Initial test location chosen from the measured end-effector pose
# (~0.437, 0.004, 0.427 m): about 10 cm forward and 10 cm lower.
CUBE_CENTER = (0.537, 0.004, 0.327)
CUBE_SIZE = 0.05


def make_add_scene():
    scene = PlanningScene()
    scene.is_diff = True

    cube = CollisionObject()
    cube.header.frame_id = FRAME_ID
    cube.id = OBJECT_ID

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]

    pose = Pose()
    pose.position.x = CUBE_CENTER[0]
    pose.position.y = CUBE_CENTER[1]
    pose.position.z = CUBE_CENTER[2]
    pose.orientation.w = 1.0

    cube.primitives.append(primitive)
    cube.primitive_poses.append(pose)
    cube.operation = CollisionObject.ADD

    scene.world.collision_objects.append(cube)

    color = ObjectColor()
    color.id = OBJECT_ID
    color.color = ColorRGBA(r=0.2, g=0.8, b=0.2, a=1.0)
    scene.object_colors.append(color)

    return scene


def make_remove_scene():
    scene = PlanningScene()
    scene.is_diff = True

    cube = CollisionObject()
    cube.header.frame_id = FRAME_ID
    cube.id = OBJECT_ID
    cube.operation = CollisionObject.REMOVE

    scene.world.collision_objects.append(cube)

    return scene


def make_attach_scene():
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True

    attached = AttachedCollisionObject()
    attached.link_name = END_EFFECTOR_LINK
    attached.object.id = OBJECT_ID
    attached.object.operation = CollisionObject.ADD
    attached.touch_links = list(TOUCH_LINKS)

    # No geometry is included here on purpose. MoveIt finds OBJECT_ID in the
    # world, removes it, and preserves its pose relative to END_EFFECTOR_LINK.
    scene.robot_state.attached_collision_objects.append(attached)

    return scene


def make_detach_scene():
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True

    attached = AttachedCollisionObject()
    attached.link_name = END_EFFECTOR_LINK
    attached.object.id = OBJECT_ID
    attached.object.operation = CollisionObject.REMOVE

    scene.robot_state.attached_collision_objects.append(attached)

    return scene


def apply_scene(node, scene):
    client = node.create_client(
        ApplyPlanningScene,
        "/apply_planning_scene",
    )

    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(
            "MoveIt service /apply_planning_scene is unavailable. "
            "Start the normal robot/MoveIt stack first."
        )

    request = ApplyPlanningScene.Request()
    request.scene = scene

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if not future.done() or future.result() is None:
        raise RuntimeError("No response from /apply_planning_scene.")

    if not future.result().success:
        raise RuntimeError("MoveIt rejected the planning-scene update.")

    node.destroy_client(client)


def get_allowed_collision_matrix(node):
    client = node.create_client(
        GetPlanningScene,
        "/get_planning_scene",
    )

    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(
            "MoveIt service /get_planning_scene is unavailable."
        )

    request = GetPlanningScene.Request()
    request.components.components = (
        PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    )

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if not future.done() or future.result() is None:
        raise RuntimeError("No response from /get_planning_scene.")

    matrix = future.result().scene.allowed_collision_matrix
    node.destroy_client(client)
    return matrix


def ensure_acm_name(matrix, name):
    if name in matrix.entry_names:
        return

    old_size = len(matrix.entry_names)
    matrix.entry_names.append(name)

    # Expand every existing row by one column.
    for row in matrix.entry_values:
        row.enabled.append(False)

    # Add the new square-matrix row.
    row = AllowedCollisionEntry()
    row.enabled = [False] * (old_size + 1)
    matrix.entry_values.append(row)


def make_allow_gripper_contact_scene(node):
    matrix = get_allowed_collision_matrix(node)

    for name in (OBJECT_ID, *TOUCH_LINKS):
        ensure_acm_name(matrix, name)

    cube_index = matrix.entry_names.index(OBJECT_ID)

    for link in TOUCH_LINKS:
        link_index = matrix.entry_names.index(link)
        matrix.entry_values[cube_index].enabled[link_index] = True
        matrix.entry_values[link_index].enabled[cube_index] = True

    scene = PlanningScene()
    scene.is_diff = True
    scene.allowed_collision_matrix = matrix

    return scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "action",
        choices=(
            "add",
            "remove",
            "attach",
            "detach",
            "allow-gripper-contact",
        ),
        help="Planning-scene action for the test cube",
    )
    args = parser.parse_args()

    rclpy.init()
    node = Node("pick_place_scene")

    try:
        if args.action == "add":
            apply_scene(node, make_add_scene())
            print(
                f"Added {OBJECT_ID} at "
                f"x={CUBE_CENTER[0]:.3f}, "
                f"y={CUBE_CENTER[1]:.3f}, "
                f"z={CUBE_CENTER[2]:.3f} m"
            )

        elif args.action == "remove":
            apply_scene(node, make_remove_scene())
            print(f"Removed {OBJECT_ID}")

        elif args.action == "attach":
            apply_scene(node, make_attach_scene())
            print(
                f"Attached {OBJECT_ID} to {END_EFFECTOR_LINK}. "
                "Move the mock arm to verify that the cube follows."
            )

        elif args.action == "detach":
            apply_scene(node, make_detach_scene())
            print(
                f"Detached {OBJECT_ID} from {END_EFFECTOR_LINK} "
                "and returned it to the world."
            )

        else:
            apply_scene(
                node,
                make_allow_gripper_contact_scene(node),
            )
            print(
                "Allowed pick_cube contact with Robotiq/gripper links only. "
                "Servo collision checking remains enabled for the rest of the robot."
            )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
