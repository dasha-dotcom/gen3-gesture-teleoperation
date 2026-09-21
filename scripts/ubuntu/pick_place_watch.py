"""Automatically attach/detach the mock pick cube from Robotiq motion.

This helper is intentionally separate from the webcam teleoperation bridge.
The existing gesture controller already drives the Robotiq gripper. This node
watches the gripper joint state and mirrors a successful simulated grasp into
MoveIt's planning scene:

- gripper closes near pick_cube -> attach pick_cube
- gripper opens while holding pick_cube -> detach pick_cube

Mock-hardware / planning-scene behavior only. No contact physics are simulated.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


OBJECT_ID = "pick_cube"
BASE_FRAME = "base_link"
END_EFFECTOR_LINK = "end_effector_link"
LEFT_TIP_LINK = "robotiq_85_left_finger_tip_link"
RIGHT_TIP_LINK = "robotiq_85_right_finger_tip_link"
GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"

# Existing gesture controller uses 0.0 for OPEN and 0.4 for CLOSED.
OPEN_THRESHOLD = 0.05
CLOSED_THRESHOLD = 0.30

# Measure proximity from the midpoint between the two fingertip links rather
# than from end_effector_link, which is the gripper mounting frame.
GRASP_RADIUS = 0.08
CLOSED_RECHECK_PERIOD = 0.25
DISTANCE_LOG_PERIOD = 1.0

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


class PickPlaceWatch(Node):
    def __init__(self):
        super().__init__("pick_place_watch")

        self.gripper_position = None
        self.last_state = None
        self.operation_in_progress = False
        self.pending_action = None
        self.holding = False

        self.last_closed_check = 0.0
        self.last_distance_log = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.get_scene = self.create_client(
            GetPlanningScene,
            "/get_planning_scene",
        )

        self.apply_scene = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )

        if not self.get_scene.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                "MoveIt service /get_planning_scene is unavailable."
            )

        if not self.apply_scene.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                "MoveIt service /apply_planning_scene is unavailable."
            )

        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.on_joint_state,
            20,
        )

        self.timer = self.create_timer(
            0.05,
            self.tick,
        )

        self.get_logger().info(
            "Watching Robotiq gripper for automatic mock pick/place."
        )
        self.get_logger().info(
            f"Pickup radius: {GRASP_RADIUS * 100:.0f} cm from the "
            "midpoint between the two fingertip links."
        )
        self.get_logger().info(
            "While CLOSED, pickup proximity is rechecked automatically."
        )

    def on_joint_state(self, msg):
        try:
            index = msg.name.index(GRIPPER_JOINT)
        except ValueError:
            return

        if index >= len(msg.position):
            return

        position = msg.position[index]

        if math.isfinite(position):
            self.gripper_position = position

    def classify_gripper(self):
        if self.gripper_position is None:
            return None

        if self.gripper_position <= OPEN_THRESHOLD:
            return "OPEN"

        if self.gripper_position >= CLOSED_THRESHOLD:
            return "CLOSED"

        return None

    def tick(self):
        if self.operation_in_progress:
            return

        state = self.classify_gripper()

        if state is None:
            return

        changed = state != self.last_state

        if changed:
            self.last_state = state
            self.get_logger().info(
                f"Gripper entered {state} state "
                f"({self.gripper_position:.3f} rad)."
            )

        if state == "OPEN":
            # Query on the transition so a restarted watcher can discover and
            # detach an already-attached object too.
            if changed:
                self.request_scene("OPEN")
            return

        # CLOSED: if not already holding the cube, keep checking proximity.
        if self.holding:
            return

        now = time.monotonic()

        if now - self.last_closed_check >= CLOSED_RECHECK_PERIOD:
            self.last_closed_check = now
            self.request_scene("CLOSED")

    def request_scene(self, action):
        self.operation_in_progress = True
        self.pending_action = action

        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )

        future = self.get_scene.call_async(request)
        future.add_done_callback(self.after_scene)

    def fingertip_midpoint(self):
        left_tf = self.tf_buffer.lookup_transform(
            BASE_FRAME,
            LEFT_TIP_LINK,
            Time(),
        )
        right_tf = self.tf_buffer.lookup_transform(
            BASE_FRAME,
            RIGHT_TIP_LINK,
            Time(),
        )

        left = left_tf.transform.translation
        right = right_tf.transform.translation

        return (
            0.5 * (left.x + right.x),
            0.5 * (left.y + right.y),
            0.5 * (left.z + right.z),
        )

    def after_scene(self, future):
        try:
            response = future.result()

            if response is None:
                raise RuntimeError(
                    "No response from /get_planning_scene"
                )

            scene = response.scene
            action = self.pending_action

            attached = next(
                (
                    item
                    for item in scene.robot_state.attached_collision_objects
                    if item.object.id == OBJECT_ID
                ),
                None,
            )

            self.holding = attached is not None

            if action == "OPEN":
                if attached is None:
                    self.get_logger().info(
                        "OPEN: cube is not attached; nothing to place."
                    )
                    self.finish_operation()
                    return

                self.get_logger().info(
                    "OPEN while holding cube -> detaching."
                )
                self.apply_detach()
                return

            # CLOSED
            if attached is not None:
                self.finish_operation()
                return

            cube = next(
                (
                    item
                    for item in scene.world.collision_objects
                    if item.id == OBJECT_ID
                ),
                None,
            )

            if cube is None:
                self.get_logger().warning(
                    "CLOSED: pick_cube is not present in the world."
                )
                self.finish_operation()
                return

            if not cube.primitive_poses:
                self.get_logger().warning(
                    "CLOSED: pick_cube has no primitive pose."
                )
                self.finish_operation()
                return

            if cube.header.frame_id not in ("", BASE_FRAME):
                self.get_logger().warning(
                    "CLOSED: cube frame is "
                    f"{cube.header.frame_id!r}, expected {BASE_FRAME!r}."
                )
                self.finish_operation()
                return

            try:
                grasp_x, grasp_y, grasp_z = self.fingertip_midpoint()
            except Exception as exc:
                self.get_logger().warning(
                    f"Could not read fingertip transforms: {exc}"
                )
                self.finish_operation()
                return

            cube_pose = cube.primitive_poses[0]

            dx = cube_pose.position.x - grasp_x
            dy = cube_pose.position.y - grasp_y
            dz = cube_pose.position.z - grasp_z
            distance = math.sqrt(
                dx * dx + dy * dy + dz * dz
            )

            now = time.monotonic()

            if (
                distance <= GRASP_RADIUS
                or now - self.last_distance_log >= DISTANCE_LOG_PERIOD
            ):
                self.last_distance_log = now
                self.get_logger().info(
                    "CLOSED: cube/fingertip-midpoint distance = "
                    f"{distance * 100:.1f} cm."
                )

            if distance > GRASP_RADIUS:
                self.finish_operation()
                return

            self.get_logger().info(
                "Cube is inside grasp radius -> attaching."
            )
            self.apply_attach()

        except Exception as exc:
            self.get_logger().error(
                f"Planning-scene query failed: {exc}"
            )
            self.finish_operation()

    def apply_attach(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        attached = AttachedCollisionObject()
        attached.link_name = END_EFFECTOR_LINK
        attached.object.id = OBJECT_ID
        attached.object.operation = CollisionObject.ADD
        attached.touch_links = list(TOUCH_LINKS)

        scene.robot_state.attached_collision_objects.append(
            attached
        )

        self.apply_update(scene, "attach")

    def apply_detach(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        attached = AttachedCollisionObject()
        attached.link_name = END_EFFECTOR_LINK
        attached.object.id = OBJECT_ID
        attached.object.operation = CollisionObject.REMOVE

        scene.robot_state.attached_collision_objects.append(
            attached
        )

        self.apply_update(scene, "detach")

    def apply_update(self, scene, action):
        request = ApplyPlanningScene.Request()
        request.scene = scene

        self.pending_action = action

        future = self.apply_scene.call_async(request)
        future.add_done_callback(self.after_apply)

    def after_apply(self, future):
        try:
            response = future.result()

            if response is None or not response.success:
                raise RuntimeError(
                    "MoveIt rejected planning-scene update"
                )

            if self.pending_action == "attach":
                self.holding = True
                self.get_logger().info(
                    "PICK COMPLETE: pick_cube attached to gripper."
                )
            else:
                self.holding = False
                self.get_logger().info(
                    "PLACE COMPLETE: pick_cube detached into world."
                )

        except Exception as exc:
            self.get_logger().error(
                f"Planning-scene update failed: {exc}"
            )

        finally:
            self.finish_operation()

    def finish_operation(self):
        self.operation_in_progress = False
        self.pending_action = None


def main():
    rclpy.init()
    node = None

    try:
        node = PickPlaceWatch()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
