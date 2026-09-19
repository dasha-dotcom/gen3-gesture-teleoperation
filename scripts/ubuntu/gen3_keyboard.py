"""Keyboard exercise for the existing Gen3 MOCK Servo setup."""
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from controller_manager_msgs.srv import ListHardwareComponents


KEYS = {
    "w": (0, 0.5), "s": (0, -0.5),
    "a": (1, 0.5), "d": (1, -0.5),
    "r": (2, 0.5), "f": (2, -0.5),
}


class Keyboard:
    def __init__(self, node):
        self.node = node
        self.pub = node.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 1)
        self.root = tk.Tk()
        self.root.title("Gen3 mock keyboard")
        self.armed = False
        self.held = {}
        self.releases = {}
        self.last_tick = time.monotonic()
        self.status = tk.StringVar(value="Click Enable keys to begin.")
        tk.Label(self.root, text="W / S: +x / -x     A / D: +y / -y\n"
                 "R / F: +z / -z     Escape: disable\n\n"
                 "Hold ONE key. Release to stop.\n"
                 "Two-second hold limit; click Enable keys again afterward.",
                 padx=20, pady=15).pack()
        tk.Button(self.root, text="Enable keys", command=self.arm,
                  takefocus=False).pack()
        tk.Label(self.root, textvariable=self.status, padx=15, pady=15).pack()
        self.root.bind("<KeyPress>", self.press)
        self.root.bind("<KeyRelease>", self.release)
        self.root.bind("<FocusOut>", lambda event: self.stop("Focus lost — disabled."))
        self.root.protocol("WM_DELETE_WINDOW", self.root.quit)
        self.root.after(20, self.tick)

    def send(self, xyz=(0.0, 0.0, 0.0)):
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = xyz
        self.pub.publish(msg)

    def stop(self, reason):
        self.armed = False
        self.held.clear()
        self.send()
        self.status.set(reason)

    def arm(self):
        if self.pub.get_subscription_count() != 1:
            self.stop("Need exactly one connected Servo subscriber.")
            return
        self.held.clear()
        self.armed = True
        self.last_tick = time.monotonic()
        self.root.focus_set()
        self.status.set("Enabled — hold one direction key.")

    def press(self, event):
        key = event.keysym.lower()
        if key == "escape":
            self.stop("Disabled. Release keys before enabling again.")
            return
        if key in self.releases:
            self.root.after_cancel(self.releases.pop(key))
        if self.armed and key in KEYS:
            self.held.setdefault(key, time.monotonic())

    def release(self, event):
        key = event.keysym.lower()
        if key not in KEYS:
            return
        if key in self.releases:
            self.root.after_cancel(self.releases.pop(key))
        # X11 key repeat may generate release/press pairs. Give the paired
        # press a short chance to cancel this release.
        self.releases[key] = self.root.after(40, lambda: self.finish_release(key))

    def finish_release(self, key):
        self.releases.pop(key, None)
        self.held.pop(key, None)
        self.send()

    def tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        now = time.monotonic()
        if self.armed and now - self.last_tick > 0.15:
            self.stop("Update delay — disabled. Enable again to continue.")
        self.last_tick = now
        if self.armed and self.pub.get_subscription_count() != 1:
            self.stop("Servo connection changed — disabled.")
        xyz = [0.0, 0.0, 0.0]
        if self.armed:
            if self.held and now - min(self.held.values()) >= 2.0:
                self.stop("Two-second limit — release keys, then enable again.")
            elif len(self.held) == 1:
                key = next(iter(self.held))
                axis, value = KEYS[key]
                xyz[axis] = value
                self.status.set(f"Moving: {key.upper()} — release to stop.")
            else:
                self.status.set("Holding still — use one direction key at a time.")
        self.send(xyz)
        self.root.after(20, self.tick)


def main():
    rclpy.init()
    node = Node("gen3_keyboard")
    app = None
    try:
        client = node.create_client(
            ListHardwareComponents, "/controller_manager/list_hardware_components")
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Controller manager is unavailable.")
        future = client.call_async(ListHardwareComponents.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("Could not check mock hardware.")
        owners = [c for c in future.result().component
                  if any(i.name == "joint_1/position" for i in c.command_interfaces)]
        if len(owners) != 1 or owners[0].state.id != 3 or (
            getattr(owners[0], "plugin_name", "") or
            getattr(owners[0], "class_type", "")
        ) != "mock_components/GenericSystem":
            raise RuntimeError("This exercise requires active GenericSystem mock arm hardware.")
        node.destroy_client(client)
        app = Keyboard(node)
        app.root.mainloop()
    finally:
        if app is not None and rclpy.ok():
            deadline = time.monotonic() + 0.3
            while time.monotonic() < deadline:
                app.send()
                rclpy.spin_once(node, timeout_sec=0.02)
            app.root.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
