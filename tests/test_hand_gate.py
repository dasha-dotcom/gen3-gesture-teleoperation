"""Exercise pure mapping/gating without importing ROS or opening any devices.

Extract definitions from the preserved standalone script to avoid changing its
tested deployment shape. These checks do not exercise its GUI or ROS callbacks.
"""
import ast
import math
from pathlib import Path
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "scripts/ubuntu/gen3_hand_bridge_v3.py"
tree = ast.parse(SOURCE.read_text())
definitions = [node for node in tree.body
               if isinstance(node, (ast.FunctionDef, ast.ClassDef))
               and node.name in {"direction", "Gate"}]
scope = {"math": math}
exec(compile(ast.Module(body=definitions, type_ignores=[]), str(SOURCE), "exec"), scope)
Gate = scope["Gate"]
direction = scope["direction"]


class GateTests(unittest.TestCase):
    def setUp(self):
        self.gate = Gate()
        self.packet(0, 0.0)

    def packet(self, seq, now, **values):
        message = {"seq": seq, "hand": True, "u": 0.75, "v": 0.5}
        message.update(values)
        self.gate.receive(message, now)

    def assert_still(self, now):
        self.assertEqual(self.gate.command(now)[:2], (0.0, 0.0))

    def test_requires_enable_and_release_stops(self):
        self.assert_still(0.0)
        self.gate.press(0.0)
        self.assertGreater(self.gate.command(0.1)[0], 0)
        self.gate.release()
        self.assert_still(0.1)
        self.assertEqual(self.gate.reason, "Space released")

    def test_hand_return_does_not_reenable(self):
        self.gate.press(0.0)
        self.packet(1, 0.1, hand=False)
        self.assert_still(0.1)
        self.packet(2, 0.2)
        self.assert_still(0.2)
        self.gate.release()
        self.gate.press(0.2)
        self.assertGreater(self.gate.command(0.2)[0], 0)

    def test_timeout_and_resume_stay_disabled(self):
        self.gate.press(0.0)
        self.assert_still(0.501)
        self.assertIn("gap", self.gate.reason)
        self.packet(1, 0.6)
        self.assert_still(0.6)

    def test_late_arrival_disables_even_without_intermediate_tick(self):
        self.gate.press(0.0)
        self.packet(1, 0.6)
        self.assert_still(0.6)

    def test_duplicates_cannot_refresh_freshness(self):
        self.gate.press(0.0)
        self.packet(0, 0.4)
        self.assert_still(0.6)

    def test_invalid_coordinates_disable(self):
        for bad in [float("nan"), float("inf"), -0.1, 1.1, True, "0.5"]:
            with self.subTest(bad=bad):
                self.gate = Gate()
                self.packet(0, 0.0)
                self.gate.press(0.0)
                self.packet(1, 0.1, u=bad)
                self.assert_still(0.1)
                self.assertEqual(self.gate.reason, "Invalid hand coordinates")

    def test_calibration_sets_rest_and_symmetric_gain(self):
        self.packet(1, 0.1, u=0.65, v=0.4)
        self.gate.calibrate(0.1)
        self.gate.press(0.1)
        self.assert_still(0.1)
        for center in [0.2, 0.4, 0.65, 0.8]:
            self.assertAlmostEqual(direction(center + 0.15, center),
                                   -direction(center - 0.15, center))
            self.assertEqual(direction(center + 0.05, center), 0.0)

    def test_edge_calibration_rejected(self):
        self.packet(1, 0.1, u=0.9)
        self.gate.calibrate(0.1)
        self.assertEqual(self.gate.center_u, 0.5)
        self.assertIn("edge", self.gate.reason)
        self.assert_still(0.1)

    def test_diagonal_cap_and_axis_signs(self):
        self.packet(1, 0.1, u=1.0, v=0.0)
        self.gate.press(0.1)
        y, z, _ = self.gate.command(0.1)
        self.assertGreater(y, 0)
        self.assertGreater(z, 0)
        self.assertAlmostEqual(math.hypot(y, z) * 0.02, 0.01)

    def test_fresh_continuous_input_has_no_two_second_cap(self):
        self.gate.press(0.0)
        for seq in range(1, 101):
            now = seq / 10
            self.packet(seq, now)
            self.assertGreater(self.gate.command(now)[0], 0)

    def test_external_stop_remains_latched_on_fresh_data(self):
        self.gate.press(0.0)
        self.gate.stop("Window lost focus")
        self.packet(1, 0.1)
        self.gate.release()
        self.assert_still(0.1)
        self.assertEqual(self.gate.reason, "Window lost focus")


if __name__ == "__main__":
    unittest.main()
