"""Continuous hold-to-run hand control on Gen3 MOCK hardware only.

Mac sender: JSON {seq: increasing integer, hand: boolean, u: float, v: float}.
No camera depth is used. Commands are base-frame y/z, with zero x/rotation.
Restart this receiver when restarting the sender (its sequence counter resets).
Press C with Space released to set the current palm position as neutral.
Calibration lasts until this program exits. The Mac sender is unchanged.
"""
import json
import math
import socket
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from controller_manager_msgs.srv import ListHardwareComponents
from rcl_interfaces.srv import GetParameters
from moveit_msgs.srv import ServoCommandType
from std_srvs.srv import SetBool


def direction(value, center=0.5):
    offset = value - center
    if abs(offset) <= 0.1:
        return 0.0
    # The same displacement gives the same speed in either direction.
    # Use the nearer image edge to set a reachable, symmetric full-speed span.
    ramp = min(center, 1.0 - center) - 0.1
    return math.copysign(min((abs(offset) - 0.1) / ramp, 1.0), offset)


class Gate:
    def __init__(self):
        self.seq = -1
        self.received = None
        self.hand = False
        self.u = self.v = 0.5
        self.center_u = self.center_v = 0.5
        self.started = None
        self.reason = "Not enabled"

    def stop(self, reason):
        self.started = None
        self.reason = reason

    def release(self):
        if self.started is not None:
            self.stop("Space released")

    def receive(self, message, now):
        if not isinstance(message, dict):
            return
        seq = message.get("seq")
        if type(seq) is not int or seq <= self.seq:
            return
        hand = message.get("hand")
        if type(hand) is not bool:
            return
        if hand:
            values = (message.get("u"), message.get("v"))
            if not all(type(v) in (int, float) and 0.0 <= v <= 1.0 for v in values):
                self.hand = False
                self.stop("Invalid hand coordinates")
                return
            self.u, self.v = values
        # A packet arriving after a gap must not automatically resume motion.
        if self.received is not None and now - self.received > 0.5:
            self.stop("Hand-data gap exceeded 0.5 seconds")
        if not hand:
            self.stop("Hand no longer detected")
        self.seq, self.received, self.hand = seq, now, hand

    def ready(self, now):
        return self.hand and self.received is not None and now - self.received <= 0.5

    def press(self, now):
        if self.ready(now):
            self.started = now
            self.reason = ""

    def calibrate(self, now):
        self.stop("Calibration requested")
        if not self.ready(now):
            self.reason = "Calibration needs a visible, recent hand"
            return
        if not all(0.2 <= v <= 0.8 for v in (self.u, self.v)):
            self.reason = "Palm too close to image edge — reposition camera/hand, then C"
            return
        self.center_u, self.center_v = self.u, self.v
        self.reason = "Neutral position saved — hold Space when ready"

    def desired(self):
        y = direction(self.u, self.center_u)
        z = -direction(self.v, self.center_v)
        scale = max(1.0, math.hypot(y, z))
        return 0.5*y/scale, 0.5*z/scale

    def command(self, now):
        if not self.ready(now):
            if self.received is None:
                reason = "Waiting for first hand-data message"
            elif now - self.received > 0.5:
                reason = "Hand-data gap exceeded 0.5 seconds"
            else:
                reason = self.reason if self.reason == "Invalid hand coordinates" else "Hand no longer detected"
            self.stop(reason)
            return 0.0, 0.0, f"STOP: {self.reason}"
        if self.started is None:
            return 0.0, 0.0, f"STOP: {self.reason}"
        y, z = self.desired()
        # Servo is checked to use unitless input and linear scale 0.02 m/s.
        status = "Enabled — hand in rest zone" if y == 0 and z == 0 else "Enabled — sending motion input"
        return y, z, status


def call(node, service, name, request):
    client = node.create_client(service, name)
    try:
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"Service unavailable: {name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"No response: {name}")
        return future.result()
    finally:
        node.destroy_client(client)


def check_setup(node):
    hardware = call(node, ListHardwareComponents,
                    "/controller_manager/list_hardware_components",
                    ListHardwareComponents.Request())
    owners = [c for c in hardware.component
              if any(i.name == "joint_1/position" for i in c.command_interfaces)]
    if len(owners) != 1 or owners[0].state.id != 3 or (
        getattr(owners[0], "plugin_name", "") or getattr(owners[0], "class_type", "")
    ) != "mock_components/GenericSystem":
        raise RuntimeError("Active GenericSystem MOCK arm hardware is required.")
    request = GetParameters.Request()
    request.names = ["moveit_servo.command_in_type", "moveit_servo.scale.linear"]
    params = call(node, GetParameters, "/servo_node/get_parameters", request).values
    if len(params) != 2 or params[0].string_value != "unitless" or not math.isclose(
        params[1].double_value, 0.02, abs_tol=1e-9
    ):
        raise RuntimeError("Expected Servo unitless input with linear scale 0.02.")
    request = ServoCommandType.Request()
    request.command_type = 1
    if not call(node, ServoCommandType, "/servo_node/switch_command_type", request).success:
        raise RuntimeError("Could not select Twist mode.")
    request = SetBool.Request()
    request.data = False
    if not call(node, SetBool, "/servo_node/pause_servo", request).success:
        raise RuntimeError("Could not unpause Servo.")


class Window:
    def __init__(self, node, receiver):
        self.node, self.receiver = node, receiver
        self.gate = Gate()
        self.down = False
        self.release_timer = None
        self.last_tick = time.monotonic()
        self.pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 1)
        self.root = tk.Tk()
        self.root.title("Gen3 mock hand control v3")
        self.status = tk.StringVar(value="Waiting for hand data...")
        self.values = tk.StringVar(value="Requested y=0, z=0 mm/s")
        self.preview = tk.StringVar(value="Hand preview: waiting")
        self.calibration = tk.StringVar(value="Neutral: u=0.500, v=0.500 (image center)")
        tk.Label(self.root, text="Click this window, then HOLD SPACE to move.\n"
                 "With Space released: hold palm comfortably, press C to center.\n"
                 "Release Space or switch windows to stop.\n"
                 "No hold-time limit. Escape disables.\n"
                 "Hand right/left: +/-y. Hand up/down: +/-z.",
                 padx=20, pady=15).pack()
        tk.Label(self.root, textvariable=self.status, padx=10, pady=10).pack()
        tk.Label(self.root, text="After an interruption: release Space, then press again.",
                 padx=10, pady=5).pack()
        tk.Label(self.root, textvariable=self.values, padx=10, pady=10).pack()
        tk.Label(self.root, textvariable=self.preview, padx=10, pady=5).pack()
        tk.Label(self.root, textvariable=self.calibration, padx=10, pady=5).pack()
        self.root.bind("<KeyPress-space>", self.press)
        self.root.bind("<KeyRelease-space>", self.release)
        self.root.bind("<Escape>", lambda event: self.stop("Escape pressed"))
        self.root.bind("<FocusOut>", lambda event: self.stop("Window lost focus"))
        self.root.bind("<KeyPress-c>", self.calibrate)
        self.root.bind("<KeyPress-C>", self.calibrate)
        self.root.protocol("WM_DELETE_WINDOW", self.root.quit)
        self.root.after(20, self.tick)

    def send(self, y=0.0, z=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.y, msg.twist.linear.z = y, z
        self.pub.publish(msg)

    def calibrate(self, event):
        if self.down:
            self.stop("Release Space before pressing C to calibrate")
            return
        self.gate.calibrate(time.monotonic())
        self.send()
        self.calibration.set(f"Neutral: u={self.gate.center_u:.3f}, "
                             f"v={self.gate.center_v:.3f}")

    def press(self, event):
        if self.release_timer is not None:
            self.root.after_cancel(self.release_timer)
            self.release_timer = None
        if not self.down:
            self.down = True
            if self.pub.get_subscription_count() == 1:
                self.gate.press(time.monotonic())

    def release(self, event):
        if self.release_timer is not None:
            self.root.after_cancel(self.release_timer)
        # Avoid treating X11 autorepeat release/press pairs as new holds.
        self.release_timer = self.root.after(40, self.finish_release)

    def finish_release(self):
        self.release_timer = None
        self.down = False
        self.gate.release()
        self.send()

    def stop(self, reason):
        self.gate.stop(reason)
        # Keep the key latch: autorepeat cannot re-enable after focus loss.
        self.send()

    def tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        now = time.monotonic()
        if now - self.last_tick > 0.15:
            self.stop(f"Window update delayed by {now - self.last_tick:.2f} seconds")
        self.last_tick = now
        for _ in range(64):
            try:
                data, address = self.receiver.recvfrom(4096)
            except BlockingIOError:
                break
            if address[0] != "192.168.64.1":
                continue
            try:
                self.gate.receive(json.loads(data), now)
            except (ValueError, UnicodeDecodeError):
                continue
        if self.pub.get_subscription_count() != 1:
            self.stop("Expected exactly one connected Servo subscriber")
        y, z, status = self.gate.command(now)
        self.send(y, z)
        self.status.set(status)
        self.values.set(f"Sent request: y={20*y:+.1f}, z={20*z:+.1f} mm/s\n"
                        f"Message {self.gate.seq}")
        if self.gate.ready(now):
            preview_y, preview_z = self.gate.desired()
            self.preview.set(f"Hand preview: y={20*preview_y:+.1f}, "
                             f"z={20*preview_z:+.1f} mm/s\n"
                             f"Palm: u={self.gate.u:.3f}, v={self.gate.v:.3f}")
        else:
            self.preview.set("Hand preview: no recent hand")
        self.root.after(20, self.tick)


def main():
    rclpy.init()
    node = Node("gen3_hand_bridge")
    app = None
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        receiver.bind(("192.168.64.2", 5005))
        receiver.setblocking(False)
        check_setup(node)
        app = Window(node, receiver)
        app.root.mainloop()
    finally:
        if app is not None and rclpy.ok():
            deadline = time.monotonic() + 0.3
            while time.monotonic() < deadline:
                app.send()
                rclpy.spin_once(node, timeout_sec=0.02)
            app.root.destroy()
        receiver.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
