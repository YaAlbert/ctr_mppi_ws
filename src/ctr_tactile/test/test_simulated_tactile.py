import math
import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_mppi_controller"))

from ctr_mppi_controller.curved_lumen import CurvedLumen  # noqa: E402
from ctr_mppi_controller.cylindrical_lumen import CylindricalLumen  # noqa: E402
from ctr_tactile.simulated_tactile import (  # noqa: E402
    REGION_NO_CONTACT,
    REGION_STOP,
    SimulatedTactileParameters,
    simulate_tactile,
)


def parameters(**overrides):
    values = {
        "zero_offset": 2.0,
        "scale": 4.0,
        "contact_stiffness_n_per_m": 100.0,
        "force_saturation_n": 1.0,
        "contact_threshold_n": 0.10,
        "warning_threshold_n": 0.30,
        "stop_threshold_n": 0.50,
    }
    values.update(overrides)
    return SimulatedTactileParameters(**values)


class SimulatedTactileModelTest(unittest.TestCase):
    def test_positive_clearance_is_zero_no_contact(self):
        sample = simulate_tactile(0.01, parameters())
        self.assertEqual((0.0, 0.0), (sample.penetration_m, sample.force_n))
        self.assertFalse(sample.contact)
        self.assertEqual(REGION_NO_CONTACT, sample.region)

    def test_exact_boundary_is_geometric_contact_with_zero_force(self):
        sample = simulate_tactile(0.0, parameters())
        self.assertTrue(sample.contact)
        self.assertEqual(0.0, sample.penetration_m)
        self.assertEqual(0.0, sample.force_n)
        self.assertIn("geometric_contact", sample.diagnostic_status)

    def test_penetration_is_monotonic_and_saturates(self):
        values = [simulate_tactile(-clearance, parameters()).force_n for clearance in (0.001, 0.003, 0.020)]
        self.assertEqual([0.1, 0.3, 1.0], values)
        self.assertEqual(REGION_STOP, simulate_tactile(-0.020, parameters()).region)
        self.assertTrue(math.isfinite(values[-1]))

    def test_raw_and_identity_filter_are_deterministic_and_invertible(self):
        first = simulate_tactile(-0.003, parameters())
        second = simulate_tactile(-0.003, parameters())
        self.assertEqual(first, second)
        self.assertEqual(first.raw_signal, first.filtered_signal)
        self.assertAlmostEqual(first.force_n, (first.raw_signal - 2.0) * 4.0)

    def test_invalid_clearance_is_invalid_not_safe(self):
        for value in (None, float("nan"), float("inf"), "bad"):
            with self.subTest(value=value):
                sample = simulate_tactile(value, parameters())
                self.assertFalse(sample.valid)
                self.assertIn("invalid", sample.diagnostic_status)

    def test_parameters_reject_invalid_values(self):
        for field, value in (("scale", 0.0), ("scale", math.inf), ("contact_stiffness_n_per_m", -1.0), ("force_saturation_n", math.nan)):
            with self.subTest(field=field, value=value):
                values = parameters().__dict__
                values[field] = value
                with self.assertRaises(ValueError):
                    SimulatedTactileParameters(**values)
        with self.assertRaises(ValueError):
            parameters(noise_std=0.1)

    def test_authoritative_cylinder_and_curved_clearance_is_consumed(self):
        cylinder = CylindricalLumen(
            frame_id="base_link",
            axis_origin=[0.0, 0.0, 0.0],
            axis_direction=[0.0, 0.0, 1.0],
            radius=0.03,
            length=0.12,
            ctr_outer_radius=0.0015,
            safety_margin=0.002,
        )
        curved = CurvedLumen(
            frame_id="base_link",
            centerline_points=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.12]],
            lumen_radius=0.03,
            ctr_outer_radius=0.0015,
            safety_margin=0.002,
        )
        for geometry, point in ((cylinder, [0.0, 0.0, 0.06]), (curved, [0.0, 0.0, 0.06])):
            clearance = geometry.point_clearance(point).physical_clearance
            sample = simulate_tactile(clearance, parameters())
            self.assertEqual(clearance, sample.clearance_m)
            self.assertEqual(0.0, sample.force_n)


class Slice7BCompatibilityTest(unittest.TestCase):
    def test_message_contract_and_disabled_defaults(self):
        message = (REPO_ROOT / "src" / "ctr_interfaces" / "msg" / "CtrTactileState.msg").read_text()
        self.assertIn("float64 clearance_m", message)
        self.assertIn("string source", message)
        self.assertIn("uint8 REGION_NO_CONTACT=0", message)
        tactile_config = (REPO_ROOT / "config" / "tactile_params.yaml").read_text()
        mppi_config = (REPO_ROOT / "config" / "mppi_params.yaml").read_text()
        self.assertIn("enabled: false", tactile_config)
        self.assertIn("force: 0.0", mppi_config)

    def test_mppi_cost_and_safety_paths_are_not_changed_by_slice_7b(self):
        cost_source = (REPO_ROOT / "src" / "ctr_mppi_controller" / "ctr_mppi_controller" / "cost_functions.py").read_text()
        safety_source = (REPO_ROOT / "src" / "ctr_safety" / "ctr_safety" / "nodes" / "safety_supervisor_node.py").read_text()
        self.assertIn("NotImplementedError", cost_source)
        self.assertIn("TODO-SAFE-010", safety_source)


if __name__ == "__main__":
    unittest.main()
