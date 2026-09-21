"""Add or remove a simple pick-and-place cube in MoveIt's planning scene.

This is a mock-hardware planning-scene exercise. It does not simulate physics.
"""

import argparse

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA


OBJECT_ID = "pick_cube"
FRAME_ID = "base_link"

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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "action",
        choices=("add", "remove"),
        help="Add or remove the test cube",
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
        else:
            apply_scene(node, make_remove_scene())
            print(f"Removed {OBJECT_ID}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
