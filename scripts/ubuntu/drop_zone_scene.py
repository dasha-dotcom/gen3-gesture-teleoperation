"""Add or remove a visible drop-zone marker in MoveIt's planning scene.

The zone is placed below the working height so it acts like a visual floor
marker without getting in the gripper's way.
"""

import argparse

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA


OBJECT_ID = "drop_zone"
FRAME_ID = "base_link"

# About 18 cm to the robot's side from the original cube location.
# Kept low so Servo collision checking should not interfere with placement.
ZONE_CENTER = (0.50, -0.18, 0.20)

ZONE_SIZE_X = 0.14
ZONE_SIZE_Y = 0.14
ZONE_HEIGHT = 0.01


def make_scene(add):
    scene = PlanningScene()
    scene.is_diff = True

    zone = CollisionObject()
    zone.header.frame_id = FRAME_ID
    zone.id = OBJECT_ID

    if add:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            ZONE_SIZE_X,
            ZONE_SIZE_Y,
            ZONE_HEIGHT,
        ]

        pose = Pose()
        pose.position.x = ZONE_CENTER[0]
        pose.position.y = ZONE_CENTER[1]
        pose.position.z = ZONE_CENTER[2]
        pose.orientation.w = 1.0

        zone.primitives.append(primitive)
        zone.primitive_poses.append(pose)
        zone.operation = CollisionObject.ADD

        scene.world.collision_objects.append(zone)

        color = ObjectColor()
        color.id = OBJECT_ID
        color.color = ColorRGBA(
            r=0.1,
            g=0.3,
            b=1.0,
            a=0.8,
        )
        scene.object_colors.append(color)

    else:
        zone.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(zone)

    return scene


def apply_scene(node, scene):
    client = node.create_client(
        ApplyPlanningScene,
        "/apply_planning_scene",
    )

    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(
            "/apply_planning_scene is unavailable"
        )

    request = ApplyPlanningScene.Request()
    request.scene = scene

    future = client.call_async(request)

    rclpy.spin_until_future_complete(
        node,
        future,
        timeout_sec=5.0,
    )

    if (
        not future.done()
        or future.result() is None
        or not future.result().success
    ):
        raise RuntimeError(
            "MoveIt rejected the drop-zone update"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "action",
        choices=("add", "remove"),
    )
    args = parser.parse_args()

    rclpy.init()
    node = Node("drop_zone_scene")

    try:
        apply_scene(
            node,
            make_scene(args.action == "add"),
        )

        if args.action == "add":
            print(
                "Added blue drop zone at "
                f"x={ZONE_CENTER[0]:.2f}, "
                f"y={ZONE_CENTER[1]:.2f}"
            )
        else:
            print("Removed drop zone")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
