"""Combined hold-to-run arm/gripper exercise on MOCK hardware only.

C calibrates neutral with Space released. Hold Space enables both channels.

If MoveIt Servo halts for a singularity:
1. Webcam control is latched off.
2. Zero velocity commands are sent.
3. Servo is paused.
4. MoveIt plans and executes a trajectory to Home.
5. Servo is unpaused.
6. The latch is cleared, but recalibration and a fresh Space press are required.

Mock hardware only.
"""

import json
import math
import socket
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import ListHardwareComponents
from geometry_msgs.msg import TwistStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    ServoStatus,
)
from moveit_msgs.srv import ServoCommandType
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import SetBool


TARGETS = {
    "OPEN": 0.0,
    "CLOSED": 0.4,
}


# These are the Home joint values defined in the Kinova Gen3 SRDF.
HOME_JOINTS = {
    "joint_1": 0.0,
    "joint_2": 0.26,
    "joint_3": 3.14,
    "joint_4": -2.27,
    "joint_5": 0.0,
    "joint_6": 0.96,
    "joint_7": 1.57,
}


def direction(value, center=0.5):
    offset = value - center

    if abs(offset) <= 0.1:
        return 0.0

    # Same displacement gives the same speed in either direction.
    # Use the nearer image edge to define the full-speed span.
    ramp = min(center, 1.0 - center) - 0.1

    return math.copysign(
        min((abs(offset) - 0.1) / ramp, 1.0),
        offset,
    )


class Gate:
    def __init__(self):
        self.seq = -1
        self.received = None
        self.hand = False

        self.u = 0.5
        self.v = 0.5

        self.center_u = 0.5
        self.center_v = 0.5

        self.started = None
        self.reason = "Not enabled"

        self.session = None
        self.fault = None
        self.gesture = "NO HAND"

        self.calibrated = False
        self.last_gripper = None

    def stop(self, reason):
        self.started = None
        self.reason = reason

    def fail(self, reason):
        self.fault = reason
        self.stop(reason)

    def next_gripper(self, now):
        if (
            self.started is None
            or not self.ready(now)
            or not self.calibrated
        ):
            return None

        if (
            self.gesture in TARGETS
            and self.gesture != self.last_gripper
        ):
            return self.gesture

        return None

    def release(self):
        if self.started is not None:
            self.stop("Space released")

    def receive(self, message, now):
        if not isinstance(message, dict):
            return

        session = message.get("session")

        if not isinstance(session, str) or len(session) != 32:
            return

        if self.session is not None and session != self.session:
            self.fail("Sender restarted — restart receiver")
            return

        seq = message.get("seq")

        if type(seq) is not int or seq <= self.seq:
            return

        hand = message.get("hand")
        gesture = message.get("gesture")

        if (
            type(hand) is not bool
            or gesture
            not in ("OPEN", "CLOSED", "UNCERTAIN", "NO HAND")
        ):
            return

        if hand == (gesture == "NO HAND"):
            self.stop("Inconsistent hand/gesture packet")
            self.hand = False
            return

        if hand:
            values = (
                message.get("u"),
                message.get("v"),
            )

            if not all(
                type(value) in (int, float)
                and 0.0 <= value <= 1.0
                for value in values
            ):
                self.hand = False
                self.stop("Invalid hand coordinates")
                return

            self.u, self.v = values

        # A packet after a data gap must not automatically resume motion.
        if (
            self.received is not None
            and now - self.received > 0.5
        ):
            self.stop("Hand-data gap exceeded 0.5 seconds")

        if not hand:
            self.stop("Hand no longer detected")

        self.seq = seq
        self.received = now
        self.hand = hand
        self.session = session
        self.gesture = gesture

    def ready(self, now):
        return (
            not self.fault
            and self.hand
            and self.received is not None
            and now - self.received <= 0.5
        )

    def press(self, now):
        if not self.calibrated:
            self.stop(
                "Press C with Space released to calibrate first"
            )
            return

        if self.ready(now):
            self.started = now
            self.reason = ""

    def calibrate(self, now):
        self.stop("Calibration requested")

        if not self.ready(now):
            self.reason = (
                "Calibration needs a visible, recent hand"
            )
            return

        if not all(
            0.2 <= value <= 0.8
            for value in (self.u, self.v)
        ):
            self.reason = (
                "Palm too close to image edge — "
                "reposition camera/hand, then C"
            )
            return

        self.center_u = self.u
        self.center_v = self.v

        self.calibrated = True

        self.reason = (
            "Neutral position saved — hold Space when ready"
        )

    def desired(self):
        y = direction(
            self.u,
            self.center_u,
        )

        z = -direction(
            self.v,
            self.center_v,
        )

        scale = max(
            1.0,
            math.hypot(y, z),
        )

        # Current Servo linear scale is 0.02 m/s.
        # Y reaches full scale; Z is intentionally slower.
        return (
            1.0 * y / scale,
            (2.0 / 3.0) * z / scale,
        )

    def command(self, now):
        if self.fault:
            self.stop(self.fault)

            return (
                0.0,
                0.0,
                f"STOP: {self.fault}",
            )

        if not self.ready(now):
            if self.received is None:
                reason = (
                    "Waiting for first hand-data message"
                )

            elif now - self.received > 0.5:
                reason = (
                    "Hand-data gap exceeded 0.5 seconds"
                )

            else:
                reason = (
                    self.reason
                    if self.reason
                    == "Invalid hand coordinates"
                    else "Hand no longer detected"
                )

            self.stop(reason)

            return (
                0.0,
                0.0,
                f"STOP: {self.reason}",
            )

        if self.started is None:
            return (
                0.0,
                0.0,
                f"STOP: {self.reason}",
            )

        y, z = self.desired()

        if y == 0 and z == 0:
            status = "Enabled — hand in rest zone"
        else:
            status = "Enabled — sending motion input"

        return y, z, status


def call(node, service, name, request):
    client = node.create_client(
        service,
        name,
    )

    try:
        if not client.wait_for_service(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                f"Service unavailable: {name}"
            )

        future = client.call_async(request)

        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=5.0,
        )

        if (
            not future.done()
            or future.result() is None
        ):
            raise RuntimeError(
                f"No response: {name}"
            )

        return future.result()

    finally:
        node.destroy_client(client)


def check_setup(node):
    hardware = call(
        node,
        ListHardwareComponents,
        "/controller_manager/list_hardware_components",
        ListHardwareComponents.Request(),
    )

    owners = [
        component
        for component in hardware.component
        if any(
            interface.name == "joint_1/position"
            for interface in component.command_interfaces
        )
    ]

    if (
        len(owners) != 1
        or owners[0].state.id != 3
        or (
            getattr(
                owners[0],
                "plugin_name",
                "",
            )
            or getattr(
                owners[0],
                "class_type",
                "",
            )
        )
        != "mock_components/GenericSystem"
    ):
        raise RuntimeError(
            "Active GenericSystem MOCK arm hardware is required."
        )

    grippers = [
        component
        for component in hardware.component
        if any(
            interface.name
            == "robotiq_85_left_knuckle_joint/position"
            for interface in component.command_interfaces
        )
    ]

    if (
        len(grippers) != 1
        or grippers[0].state.id != 3
        or (
            getattr(
                grippers[0],
                "plugin_name",
                "",
            )
            or getattr(
                grippers[0],
                "class_type",
                "",
            )
        )
        != "mock_components/GenericSystem"
    ):
        raise RuntimeError(
            "Active GenericSystem MOCK gripper hardware is required."
        )

    request = GetParameters.Request()

    request.names = [
        "moveit_servo.command_in_type",
        "moveit_servo.scale.linear",
    ]

    params = call(
        node,
        GetParameters,
        "/servo_node/get_parameters",
        request,
    ).values

    if (
        len(params) != 2
        or params[0].string_value != "unitless"
        or not math.isclose(
            params[1].double_value,
            0.02,
            abs_tol=1e-9,
        )
    ):
        raise RuntimeError(
            "Expected Servo unitless input "
            "with linear scale 0.02."
        )

    request = ServoCommandType.Request()
    request.command_type = 1

    if not call(
        node,
        ServoCommandType,
        "/servo_node/switch_command_type",
        request,
    ).success:
        raise RuntimeError(
            "Could not select Twist mode."
        )

    # Start with Servo unpaused.
    request = SetBool.Request()
    request.data = False

    if not call(
        node,
        SetBool,
        "/servo_node/pause_servo",
        request,
    ).success:
        raise RuntimeError(
            "Could not unpause Servo."
        )


class Window:
    def __init__(
        self,
        node,
        receiver,
        gripper,
    ):
        self.node = node
        self.receiver = receiver
        self.gripper = gripper

        self.gate = Gate()

        self.busy = False
        self.pending_since = None

        self.closing = False
        self.down = False
        self.release_timer = None

        self.last_tick = time.monotonic()

        # Servo status / recovery state.
        self.servo_code = None
        self.servo_message = ""

        self.singularity_latched = False

        self.recovery_state = "IDLE"
        self.recovery_goal_handle = None
        self.servo_paused_for_recovery = False

        self.pub = node.create_publisher(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            1,
        )

        self.servo_status_sub = node.create_subscription(
            ServoStatus,
            "/servo_node/status",
            self.on_servo_status,
            10,
        )

        # Persistent client used by the recovery state machine.
        self.pause_client = node.create_client(
            SetBool,
            "/servo_node/pause_servo",
        )

        if not self.pause_client.wait_for_service(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                "Servo pause service unavailable."
            )

        # Existing move_group action server from robot.launch.py.
        self.move_group = ActionClient(
            node,
            MoveGroup,
            "/move_action",
        )

        if not self.move_group.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                "MoveIt /move_action server unavailable."
            )

        self.root = tk.Tk()

        self.root.title(
            "Gen3 combined mock hand control"
        )

        self.status = tk.StringVar(
            value="Waiting for hand data..."
        )

        self.values = tk.StringVar(
            value="Requested y=0, z=0 mm/s"
        )

        self.preview = tk.StringVar(
            value="Hand preview: waiting"
        )

        self.calibration = tk.StringVar(
            value=(
                "Neutral: u=0.500, v=0.500 "
                "(image center)"
            )
        )

        self.gripper_status = tk.StringVar(
            value=(
                "Gripper: disabled until Space is held"
            )
        )

        self.recovery_status = tk.StringVar(
            value="Recovery: idle"
        )

        tk.Label(
            self.root,
            text=(
                "Click this window, then HOLD SPACE "
                "to control arm + gripper.\n"
                "With Space released: hold palm comfortably, "
                "press C to center.\n"
                "Release Space or switch windows to stop.\n"
                "Hand right/left: +/-y. "
                "Hand up/down: +/-z.\n"
                "OPEN: open gripper. "
                "CLOSED: partial close.\n"
                "A singularity halt automatically triggers "
                "MoveIt Home recovery."
            ),
            padx=20,
            pady=15,
        ).pack()

        tk.Label(
            self.root,
            textvariable=self.status,
            padx=10,
            pady=10,
        ).pack()

        tk.Label(
            self.root,
            textvariable=self.recovery_status,
            padx=10,
            pady=5,
        ).pack()

        tk.Label(
            self.root,
            text=(
                "After an interruption/recovery: "
                "release Space, recalibrate with C, "
                "then press Space again."
            ),
            padx=10,
            pady=5,
        ).pack()

        tk.Label(
            self.root,
            textvariable=self.values,
            padx=10,
            pady=10,
        ).pack()

        tk.Label(
            self.root,
            textvariable=self.preview,
            padx=10,
            pady=5,
        ).pack()

        tk.Label(
            self.root,
            textvariable=self.calibration,
            padx=10,
            pady=5,
        ).pack()

        tk.Label(
            self.root,
            textvariable=self.gripper_status,
            padx=10,
            pady=5,
        ).pack()

        self.root.bind(
            "<KeyPress-space>",
            self.press,
        )

        self.root.bind(
            "<KeyRelease-space>",
            self.release,
        )

        self.root.bind(
            "<Escape>",
            lambda event: self.stop(
                "Escape pressed"
            ),
        )

        self.root.bind(
            "<FocusOut>",
            lambda event: self.stop(
                "Window lost focus"
            ),
        )

        self.root.bind(
            "<KeyPress-c>",
            self.calibrate,
        )

        self.root.bind(
            "<KeyPress-C>",
            self.calibrate,
        )

        self.root.protocol(
            "WM_DELETE_WINDOW",
            self.close,
        )

        self.root.after(
            20,
            self.tick,
        )

    # ---------------------------------------------------------
    # Singularity recovery
    # ---------------------------------------------------------

    def make_home_goal(self):
        goal = MoveGroup.Goal()

        goal.request.group_name = "manipulator"

        # Current state should be used as the planning start state.
        goal.request.start_state.is_diff = True

        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0

        # Intentionally conservative recovery trajectory.
        goal.request.max_velocity_scaling_factor = 0.20
        goal.request.max_acceleration_scaling_factor = 0.20

        home = Constraints()
        home.name = "Home"

        for joint_name, position in HOME_JOINTS.items():
            constraint = JointConstraint()

            constraint.joint_name = joint_name
            constraint.position = position

            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01

            constraint.weight = 1.0

            home.joint_constraints.append(
                constraint
            )

        goal.request.goal_constraints = [home]

        # False means MoveIt should plan AND execute.
        goal.planning_options.plan_only = False

        goal.planning_options.look_around = False

        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.2

        goal.planning_options.planning_scene_diff.is_diff = True

        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        return goal

    def start_singularity_recovery(self):
        if self.recovery_state not in (
            "IDLE",
            "COMPLETE",
        ):
            return

        print(
            "Starting automatic singularity recovery.",
            flush=True,
        )

        # Disable teleoperation immediately.
        self.singularity_latched = True
        self.gate.calibrated = False

        self.gate.stop(
            "Singularity halt — automatic recovery starting"
        )

        self.send()

        self.recovery_state = "PAUSING_SERVO"

        self.recovery_status.set(
            "Recovery: pausing Servo..."
        )

        request = SetBool.Request()
        request.data = True

        future = self.pause_client.call_async(
            request
        )

        future.add_done_callback(
            self.after_servo_paused
        )

    def after_servo_paused(self, future):
        if self.closing:
            return

        try:
            response = future.result()

        except Exception as exc:
            self.recovery_failed(
                f"Could not pause Servo: {exc}"
            )
            return

        if response is None or not response.success:
            self.recovery_failed(
                "Servo refused pause request"
            )
            return

        self.servo_paused_for_recovery = True

        print(
            "Servo paused. Sending MoveIt Home goal.",
            flush=True,
        )

        self.recovery_state = "MOVING_HOME"

        self.recovery_status.set(
            "Recovery: planning/executing Home..."
        )

        goal = self.make_home_goal()

        future = self.move_group.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.home_goal_response
        )

    def home_goal_response(self, future):
        if self.closing:
            return

        try:
            goal_handle = future.result()

        except Exception as exc:
            self.recovery_failed(
                f"MoveIt Home goal error: {exc}"
            )
            return

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            self.recovery_failed(
                "MoveIt rejected Home goal"
            )
            return

        self.recovery_goal_handle = goal_handle

        print(
            "MoveIt accepted Home recovery goal.",
            flush=True,
        )

        future = goal_handle.get_result_async()

        future.add_done_callback(
            self.home_result
        )

    def home_result(self, future):
        if self.closing:
            return

        try:
            wrapped_result = future.result()
            result = wrapped_result.result

        except Exception as exc:
            self.recovery_failed(
                f"Home result error: {exc}"
            )
            return

        self.recovery_goal_handle = None

        if (
            result.error_code.val
            != MoveItErrorCodes.SUCCESS
        ):
            self.recovery_failed(
                "MoveIt Home recovery failed "
                f"with error code {result.error_code.val}"
            )
            return

        print(
            "Home reached successfully.",
            flush=True,
        )

        self.recovery_state = "UNPAUSING_SERVO"

        self.recovery_status.set(
            "Recovery: Home reached; unpausing Servo..."
        )

        request = SetBool.Request()
        request.data = False

        future = self.pause_client.call_async(
            request
        )

        future.add_done_callback(
            self.after_servo_unpaused
        )

    def after_servo_unpaused(self, future):
        if self.closing:
            return

        try:
            response = future.result()

        except Exception as exc:
            self.recovery_failed(
                f"Could not unpause Servo: {exc}"
            )
            return

        if response is None or not response.success:
            self.recovery_failed(
                "Servo refused unpause request"
            )
            return

        self.servo_paused_for_recovery = False

        # Recovery succeeded.
        self.singularity_latched = False
        self.recovery_state = "COMPLETE"

        # IMPORTANT:
        # Going Home must NOT automatically resume webcam control.
        self.gate.calibrated = False

        self.gate.stop(
            "Recovery complete — release Space, "
            "press C to recalibrate, then hold Space"
        )

        self.send()

        self.recovery_status.set(
            "Recovery: COMPLETE — recalibrate before control"
        )

        print(
            "Singularity recovery complete. "
            "Teleoperation remains disabled until recalibration.",
            flush=True,
        )

    def recovery_failed(self, reason):
        print(
            f"RECOVERY FAILED: {reason}",
            flush=True,
        )

        self.recovery_state = "FAILED"
        self.singularity_latched = True

        self.gate.fail(
            "Automatic recovery failed — "
            "manual recovery/restart required"
        )

        self.send()

        if self.servo_paused_for_recovery:
            suffix = " Servo remains paused."
        else:
            suffix = ""

        self.recovery_status.set(
            f"Recovery: FAILED — {reason}.{suffix}"
        )

    def on_servo_status(self, msg):
        self.servo_code = msg.code
        self.servo_message = msg.message

        if (
            msg.code
            == ServoStatus.HALT_FOR_SINGULARITY
        ):
            if not self.singularity_latched:
                print(
                    "Servo singularity halt detected: "
                    f"{msg.message}",
                    flush=True,
                )

                self.start_singularity_recovery()

    # ---------------------------------------------------------
    # Normal arm/gripper control
    # ---------------------------------------------------------

    def close(self):
        self.closing = True

        self.stop(
            "Window closed"
        )

        # If Home execution is active, request cancellation.
        if self.recovery_goal_handle is not None:
            try:
                self.recovery_goal_handle.cancel_goal_async()
            except Exception:
                pass

        self.root.quit()

    def gripper_failed(self, reason):
        self.gate.fail(
            reason + " — restart receiver"
        )

        self.send()

        if not self.closing:
            self.gripper_status.set(
                "Gripper: " + reason
            )

        print(
            reason,
            flush=True,
        )

    def maybe_grip(self, now):
        if self.busy:
            if (
                now - self.pending_since > 10
                and not self.gate.fault
            ):
                self.gripper_failed(
                    "Gripper result timed out; "
                    "completion unknown"
                )

            return

        label = self.gate.next_gripper(
            now
        )

        if (
            label is None
            or self.closing
            or self.singularity_latched
        ):
            return

        goal = GripperCommand.Goal()

        goal.command.position = TARGETS[label]
        goal.command.max_effort = 100.0

        self.busy = True
        self.pending_since = now

        self.gate.last_gripper = label

        self.gripper_status.set(
            f"Gripper: sending {label}"
        )

        print(
            f"Sending {label}: "
            f"target={TARGETS[label]}",
            flush=True,
        )

        try:
            self.gripper.send_goal_async(
                goal
            ).add_done_callback(
                self.gripper_accepted
            )

        except Exception as exc:
            self.busy = False

            self.gripper_failed(
                f"Gripper send failed: {exc}"
            )

    def gripper_accepted(self, future):
        try:
            handle = future.result()

            if not handle.accepted:
                raise RuntimeError(
                    "Gripper goal rejected"
                )

            handle.get_result_async(
            ).add_done_callback(
                self.gripper_finished
            )

        except Exception as exc:
            self.busy = False

            self.gripper_failed(
                str(exc)
            )

    def gripper_finished(self, future):
        self.busy = False

        try:
            reply = future.result()
            result = reply.result

            print(
                f"Gripper result: "
                f"status={reply.status}, "
                f"reached={result.reached_goal}, "
                f"position={result.position:.3f}",
                flush=True,
            )

            if (
                reply.status
                != GoalStatus.STATUS_SUCCEEDED
                or not result.reached_goal
            ):
                self.gripper_failed(
                    "Gripper did not confirm target arrival"
                )

            elif not self.closing:
                self.gripper_status.set(
                    "Gripper: last target reached "
                    f"({result.position:.3f})"
                )

        except Exception as exc:
            self.gripper_failed(
                f"Gripper result error: {exc}"
            )

    def send(
        self,
        y=0.0,
        z=0.0,
    ):
        msg = TwistStamped()

        msg.header.stamp = (
            self.node.get_clock().now().to_msg()
        )

        msg.header.frame_id = "base_link"

        msg.twist.linear.y = y
        msg.twist.linear.z = z

        self.pub.publish(msg)

    def calibrate(self, event):
        if self.singularity_latched:
            self.gate.stop(
                "Wait for singularity recovery to finish"
            )
            return

        if self.down:
            self.stop(
                "Release Space before pressing C to calibrate"
            )
            return

        self.gate.calibrate(
            time.monotonic()
        )

        self.send()

        self.calibration.set(
            f"Neutral: "
            f"u={self.gate.center_u:.3f}, "
            f"v={self.gate.center_v:.3f}"
        )

    def press(self, event):
        if self.release_timer is not None:
            self.root.after_cancel(
                self.release_timer
            )

            self.release_timer = None

        if not self.down:
            self.down = True

            if self.singularity_latched:
                self.gate.stop(
                    "Singularity recovery in progress"
                )
                return

            if (
                self.pub.get_subscription_count()
                == 1
            ):
                self.gate.press(
                    time.monotonic()
                )

    def release(self, event):
        if self.release_timer is not None:
            self.root.after_cancel(
                self.release_timer
            )

        # Avoid treating X11 autorepeat release/press pairs
        # as a new hold.
        self.release_timer = self.root.after(
            40,
            self.finish_release,
        )

    def finish_release(self):
        self.release_timer = None
        self.down = False

        self.gate.release()

        self.send()

    def stop(self, reason):
        self.gate.stop(
            reason
        )

        self.send()

    def tick(self):
        rclpy.spin_once(
            self.node,
            timeout_sec=0.0,
        )

        now = time.monotonic()

        if now - self.last_tick > 0.15:
            self.stop(
                "Window update delayed by "
                f"{now - self.last_tick:.2f} seconds"
            )

        self.last_tick = now

        for _ in range(64):
            try:
                data, address = (
                    self.receiver.recvfrom(4096)
                )

            except BlockingIOError:
                break

            if (
                address[0]
                != "192.168.64.1"
            ):
                continue

            try:
                self.gate.receive(
                    json.loads(data),
                    now,
                )

            except (
                ValueError,
                UnicodeDecodeError,
            ):
                continue

        else:
            self.stop(
                "Too many queued hand packets"
            )

        if (
            self.pub.get_subscription_count()
            != 1
        ):
            self.stop(
                "Expected exactly one connected Servo subscriber"
            )

        if (
            self.busy
            and now - self.pending_since > 10
            and not self.gate.fault
        ):
            self.gripper_failed(
                "Gripper result timed out; "
                "completion unknown"
            )

        # While recovery is active, never send webcam motion.
        if self.singularity_latched:
            y = 0.0
            z = 0.0

            status = (
                "STOP: singularity recovery "
                f"({self.recovery_state})"
            )

        else:
            y, z, status = (
                self.gate.command(now)
            )

        self.send(
            y,
            z,
        )

        self.maybe_grip(
            now
        )

        self.status.set(
            status
        )

        self.values.set(
            "Sent request: "
            f"y={20 * y:+.1f}, "
            f"z={20 * z:+.1f} mm/s\n"
            f"Message {self.gate.seq}"
        )

        if self.gate.ready(now):
            preview_y, preview_z = (
                self.gate.desired()
            )

            self.preview.set(
                "Hand preview: "
                f"y={20 * preview_y:+.1f}, "
                f"z={20 * preview_z:+.1f} mm/s\n"
                f"Palm: "
                f"u={self.gate.u:.3f}, "
                f"v={self.gate.v:.3f}\n"
                f"Gesture: {self.gate.gesture}"
            )

        else:
            self.preview.set(
                "Hand preview: no recent hand"
            )

        self.root.after(
            20,
            self.tick,
        )


def main():
    rclpy.init()

    node = Node(
        "gen3_combined_bridge"
    )

    app = None

    gripper = ActionClient(
        node,
        GripperCommand,
        "/robotiq_gripper_controller/gripper_cmd",
    )

    receiver = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM,
    )

    try:
        receiver.bind(
            ("192.168.64.2", 5007)
        )

        receiver.setblocking(
            False
        )

        check_setup(
            node
        )

        if not gripper.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                "Gripper action server unavailable."
            )

        app = Window(
            node,
            receiver,
            gripper,
        )

        app.root.mainloop()

    finally:
        if app is not None:
            app.closing = True

            app.gate.stop(
                "Receiver ending"
            )

            if app.busy:
                print(
                    "Gripper action pending; "
                    "it may still finish after exit.",
                    flush=True,
                )

        if (
            app is not None
            and rclpy.ok()
        ):
            deadline = (
                time.monotonic() + 0.3
            )

            while (
                time.monotonic()
                < deadline
            ):
                app.send()

                rclpy.spin_once(
                    node,
                    timeout_sec=0.02,
                )

            app.root.destroy()

        receiver.close()
        gripper.destroy()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
