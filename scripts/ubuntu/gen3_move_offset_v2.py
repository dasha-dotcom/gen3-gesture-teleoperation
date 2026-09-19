#!/usr/bin/env python3
"""Teach/test small base-frame pose offsets on Dasha's Gen3 MOCK setup.

Default: calculate and preview only. --execute also offers a confirmation prompt.
Run inside Ubuntu with ROS Jazzy and the viewer workspace sourced.
This plans an endpoint offset, NOT a constrained straight Cartesian path.
"""
import argparse
import copy
import math
import signal
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data,
)
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListHardwareComponents
from sensor_msgs.msg import JointState
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, DisplayTrajectory, JointConstraint, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetPositionIK

JOINTS = [f"joint_{i}" for i in range(1, 8)]
BASE = "base_link"
TIP = "end_effector_link"
GROUP = "manipulator"


class OffsetDemo(Node):
    def __init__(self):
        super().__init__("gen3_offset_demo")
        self.latest = None
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.receive_state, qos_profile_sensor_data)
        self.preview = self.create_publisher(
            DisplayTrajectory, "/display_planned_path",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.execution = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.active_goal = None

    def receive_state(self, msg):
        if len(msg.name) == len(msg.position) and set(JOINTS).issubset(msg.name):
            self.latest = msg

    def wait_result(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError("Request timed out. Inspect the combined-launch terminal.")
        result = future.result()
        if result is None:
            raise RuntimeError("Request returned no result.")
        return result

    def call(self, service_type, name, request):
        client = self.create_client(service_type, name)
        try:
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f"Service unavailable: {name}. Is MoveIt running?")
            return self.wait_result(client.call_async(request), 15.0)
        finally:
            self.destroy_client(client)

    def state(self):
        # Spin for a new sample, rather than reuse a pre-planning sample.
        self.latest = None
        deadline = time.monotonic() + 5.0
        while self.latest is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest is None:
            raise RuntimeError("No complete seven-joint state received on /joint_states.")
        if not all(math.isfinite(x) for x in self.latest.position):
            raise RuntimeError("Joint positions contain non-finite values.")
        result = RobotState()
        result.is_diff = True
        result.joint_state.name = list(self.latest.name)
        result.joint_state.position = list(self.latest.position)
        # Omit mock effort NaNs and velocity fields; positions define this seed.
        return result

    def require_mock(self):
        result = self.call(ListHardwareComponents,
                           "/controller_manager/list_hardware_components",
                           ListHardwareComponents.Request())
        owners = [c for c in result.component
                  if "joint_1/position" in [i.name for i in c.command_interfaces]]
        if len(owners) != 1:
            raise RuntimeError("Cannot identify exactly one arm hardware component.")
        arm = owners[0]
        plugin = getattr(arm, "plugin_name", "") or getattr(arm, "class_type", "")
        if plugin != "mock_components/GenericSystem" or arm.state.id != 3:
            raise RuntimeError("This exercise requires active GenericSystem MOCK arm hardware.")

    @staticmethod
    def check(code, stage):
        if code.val != 1:
            raise RuntimeError(f"{stage} failed: MoveIt error {code.val}. "
                               f"{getattr(code, 'message', '')}")

    def pose(self, state):
        request = GetPositionFK.Request()
        request.header.frame_id = BASE
        request.fk_link_names = [TIP]
        request.robot_state = state
        result = self.call(GetPositionFK, "/compute_fk", request)
        self.check(result.error_code, "Forward kinematics")
        if not result.pose_stamped or result.pose_stamped[0].header.frame_id != BASE:
            raise RuntimeError("Forward kinematics returned an unexpected reference frame.")
        return result.pose_stamped[0]

    def run(self, axis, distance, execute, speed_scale, cancel_after):
        self.require_mock()
        start = self.state()
        before = self.pose(start)
        target = copy.deepcopy(before)
        target.header.stamp.sec = 0
        target.header.stamp.nanosec = 0
        setattr(target.pose.position, axis,
                getattr(target.pose.position, axis) + distance)
        print_pose("Current", before)
        print_pose("Target ", target)

        # 1. Inverse kinematics: same orientation, translated position.
        request = GetPositionIK.Request()
        request.ik_request.group_name = GROUP
        request.ik_request.ik_link_name = TIP
        request.ik_request.robot_state = start
        request.ik_request.pose_stamped = target
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 2
        ik = self.call(GetPositionIK, "/compute_ik", request)
        self.check(ik.error_code, "Inverse kinematics")
        angles = dict(zip(ik.solution.joint_state.name, ik.solution.joint_state.position))
        if not all(j in angles and math.isfinite(angles[j]) for j in JOINTS):
            raise RuntimeError("IK response does not contain valid arm joint angles.")

        # 2. Plan a collision-checked joint-space path to the IK result.
        request = GetMotionPlan.Request()
        plan_request = request.motion_plan_request
        plan_request.group_name = GROUP
        plan_request.start_state = start
        plan_request.num_planning_attempts = 1
        plan_request.allowed_planning_time = 5.0
        plan_request.max_velocity_scaling_factor = speed_scale
        plan_request.max_acceleration_scaling_factor = 0.1
        goal = Constraints()
        for joint in JOINTS:
            constraint = JointConstraint()
            constraint.joint_name = joint
            constraint.position = angles[joint]
            constraint.tolerance_above = 0.0001
            constraint.tolerance_below = 0.0001
            constraint.weight = 1.0
            goal.joint_constraints.append(constraint)
        plan_request.goal_constraints = [goal]
        plan = self.call(GetMotionPlan, "/plan_kinematic_path", request).motion_plan_response
        self.check(plan.error_code, "Planning")
        trajectory = plan.trajectory.joint_trajectory
        if not trajectory.points or set(trajectory.joint_names) != set(JOINTS):
            raise RuntimeError("Planner returned an empty or unexpected arm trajectory.")
        initial = dict(zip(start.joint_state.name, start.joint_state.position))
        excursion = max(abs(p.positions[i] - initial[j])
                        for p in trajectory.points
                        for i, j in enumerate(trajectory.joint_names))
        duration = trajectory.points[-1].time_from_start
        seconds = duration.sec + duration.nanosec / 1e9
        print(f"Plan: {len(trajectory.points)} points, {seconds:.2f} seconds.")
        print(f"Largest sampled joint excursion: {math.degrees(excursion):.2f} degrees.")
        print("The endpoint offset is specified; a straight tool path is NOT enforced.")

        display = DisplayTrajectory()
        display.trajectory_start = plan.trajectory_start
        display.trajectory = [plan.trajectory]
        self.preview.publish(display)
        # Allow discovery/delivery to an already-running RViz display.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not execute:
            print("PLAN ONLY: no motion commanded. See RViz Planned Path preview.")
            return

        if cancel_after is not None and seconds < cancel_after + 3.0:
            raise RuntimeError("Plan is too short for this cancellation test. Reduce --speed-scale.")

        # 3. Execute this exact returned trajectory, not a fresh replan.
        if input("Review RViz. Type MOVE to execute this plan: ").strip() != "MOVE":
            print("Cancelled; no motion commanded.")
            return
        self.require_mock()
        current = self.state()
        current_angles = dict(zip(current.joint_state.name, current.joint_state.position))
        if any(abs(current_angles[j] - initial[j]) > 0.001 for j in JOINTS):
            raise RuntimeError("Arm moved since planning. Run the helper again to replan.")
        if not self.execution.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/execute_trajectory action server unavailable.")
        request = ExecuteTrajectory.Goal()
        request.trajectory = plan.trajectory
        request.controller_names = ["joint_trajectory_controller"]
        result, cancel_requested = self.execute_and_watch(request, seconds, cancel_after)
        if cancel_requested:
            if result.status == GoalStatus.STATUS_CANCELED:
                log("Action ended with CANCELED status. Checking reported position next.")
            else:
                log(f"Cancellation NOT confirmed: final action status {result.status}; "
                    f"MoveIt code {result.result.error_code.val}.")
            first = self.pose(self.state())
            print_pose("After action", first)
            until = time.monotonic() + 2.0
            while time.monotonic() < until:
                rclpy.spin_once(self, timeout_sec=0.1)
            second = self.pose(self.state())
            print_pose("2 sec later", second)
            movement = math.sqrt(sum((getattr(second.pose.position, a) -
                                      getattr(first.pose.position, a)) ** 2 for a in "xyz"))
            remaining = math.sqrt(sum((getattr(second.pose.position, a) -
                                       getattr(target.pose.position, a)) ** 2 for a in "xyz"))
            print(f"Position change between checks: {movement * 1000:.3f} mm.")
            print(f"Distance from original target: {remaining * 1000:.3f} mm.")
            print("These two samples check endpoint stability, not stop latency or full motion history.")
            if result.status != GoalStatus.STATUS_CANCELED:
                raise RuntimeError("Cancellation test did not end with CANCELED status.")
            return
        self.check(result.result.error_code, "Execution")
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"Execution ended with action status {result.status}.")

        # 4. Read the reported endpoint and compare it with the target.
        after = self.pose(self.state())
        print_pose("Reached", after)
        delta = [getattr(after.pose.position, a) - getattr(before.pose.position, a)
                 for a in "xyz"]
        error = math.sqrt(sum((getattr(after.pose.position, a) -
                               getattr(target.pose.position, a)) ** 2 for a in "xyz"))
        print("Measured displacement (mm): " + str([round(v * 1000, 3) for v in delta]))
        print(f"Position error from target: {error * 1000:.3f} mm (mock result).")
        print("Execution succeeded.")

    def execute_and_watch(self, request, seconds, cancel_after):
        # A signal sets a flag; it does NOT unwind a ROS callback mid-processing.
        interrupted = [False]
        previous_handler = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, lambda *_: interrupted.__setitem__(0, True))
        try:
            log("Sending execution goal.")
            self.active_goal = self.wait_result(self.execution.send_goal_async(request), 15.0)
            if not self.active_goal.accepted:
                self.active_goal = None
                raise RuntimeError("Execution goal rejected.")
            goal_id = bytes(self.active_goal.goal_id.uuid).hex()
            log(f"Execution accepted. Goal ID: {goal_id}")
            if cancel_after is not None:
                log(f"Will request cancellation {cancel_after:.1f} seconds after acceptance.")
            else:
                log("Ctrl+C now requests cancellation while keeping the ROS client alive.")
            result_future = self.active_goal.get_result_async()
            start_time = time.monotonic()
            deadline = start_time + max(45.0, seconds + 30.0)
            cancel_future = None
            cancel_reported = False
            while True:
                rclpy.spin_once(self, timeout_sec=0.05)
                elapsed = time.monotonic() - start_time
                if (cancel_future is None and not result_future.done() and
                        (interrupted[0] or (cancel_after is not None and elapsed >= cancel_after))):
                    log(f"Requesting cancellation at +{elapsed:.2f} seconds.")
                    cancel_future = self.active_goal.cancel_goal_async()
                    deadline = max(deadline, time.monotonic() + 30.0)
                if cancel_future is not None and cancel_future.done() and not cancel_reported:
                    reply = cancel_future.result()
                    ids = [bytes(g.goal_id.uuid).hex() for g in reply.goals_canceling]
                    log(f"Cancel response code: {reply.return_code}; "
                        f"our goal listed as canceling: {goal_id in ids}.")
                    cancel_reported = True
                if result_future.done() and (cancel_future is None or cancel_reported):
                    result = result_future.result()
                    self.active_goal = None
                    log(f"Final action status: {result.status}; MoveIt code: {result.result.error_code.val}.")
                    if cancel_after is not None and cancel_future is None:
                        raise RuntimeError("Execution finished before cancellation was sent; test not performed.")
                    return result, cancel_future is not None
                if time.monotonic() >= deadline:
                    if result_future.done():
                        result = result_future.result()
                        self.active_goal = None
                        log("Cancel acknowledgement missing; final action result was received.")
                        log(f"Final action status: {result.status}; MoveIt code: {result.result.error_code.val}.")
                        return result, cancel_future is not None
                    raise RuntimeError("No final execution result received. Stop status is UNKNOWN; "
                                       "inspect the robot state and launch log before another command.")
        finally:
            signal.signal(signal.SIGINT, previous_handler)

    def cancel_if_needed(self):
        # Best-effort cleanup after an unexpected execution error.
        if self.active_goal is not None and self.active_goal.accepted:
            log("Cleanup: requesting cancellation and waiting for a final result.")
            try:
                reply = self.wait_result(self.active_goal.cancel_goal_async(), 15.0)
                log(f"Cleanup cancel response code: {reply.return_code}.")
                result = self.wait_result(self.active_goal.get_result_async(), 15.0)
                log(f"Cleanup final action status: {result.status}.")
            except Exception as exc:
                log(f"STOP STATUS UNKNOWN: {exc}. Inspect the launch terminal and joint states.")


def log(message):
    print(f"[{time.time():.3f}] {message}", flush=True)


def print_pose(label, stamped):
    p = stamped.pose.position
    print(f"{label} {TIP} in {BASE}: [{p.x:.6f}, {p.y:.6f}, {p.z:.6f}] m")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("axis", choices=["x", "y", "z"])
    parser.add_argument("distance", type=float, help="Signed offset in metres, e.g. 0.02")
    parser.add_argument("--execute", action="store_true", help="Offer execution after review")
    parser.add_argument("--speed-scale", type=float, default=0.1,
                        help="Fraction of configured joint-speed limits (default 0.1)")
    parser.add_argument("--cancel-after", type=float,
                        help="Request cancellation this many seconds after goal acceptance")
    args = parser.parse_args()
    if not math.isfinite(args.speed_scale) or not 0 < args.speed_scale <= 0.1:
        parser.error("--speed-scale must be greater than zero and at most 0.1")
    if args.cancel_after is not None:
        if not args.execute or not math.isfinite(args.cancel_after) or args.cancel_after <= 0:
            parser.error("--cancel-after requires --execute and a positive finite duration")
    if not math.isfinite(args.distance) or not 0 < abs(args.distance) <= 0.05:
        parser.error("Use a nonzero offset no larger than 0.05 m for this exercise.")
    # Keep Ctrl+C available to request action cancellation before shutting ROS down.
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = OffsetDemo()
    code = 0
    try:
        node.run(args.axis, args.distance, args.execute, args.speed_scale, args.cancel_after)
    except KeyboardInterrupt:
        print("Interrupted.")
        code = 130
    except Exception as exc:
        print(f"Stopped: {exc}", file=sys.stderr)
        code = 1
    finally:
        node.cancel_if_needed()
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
