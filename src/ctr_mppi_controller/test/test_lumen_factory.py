import copy
import math
import re
import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))

from ctr_bringup.parameter_validation import load_parameter_files  # noqa: E402
from ctr_mppi_controller.curved_lumen import CurvedLumen  # noqa: E402
from ctr_mppi_controller.cylindrical_lumen import (  # noqa: E402
    CylindricalLumen,
    lumen_geometry_from_config as compatibility_lumen_geometry_from_config,
)
from ctr_mppi_controller.lumen_factory import (  # noqa: E402
    config_with_lumen_overrides,
    config_with_mppi_profile,
    curved_lumen_enabled,
    cylindrical_lumen_enabled,
    lumen_cost_weights_from_config,
    lumen_geometry_fingerprint,
    lumen_geometry_fingerprint_payload,
    lumen_geometry_from_config,
    lumen_mode_from_config,
)


CONFIG_FILES = [
    REPO_ROOT / "config" / "robot_params.yaml",
    REPO_ROOT / "config" / "model_params.yaml",
    REPO_ROOT / "config" / "mppi_params.yaml",
    REPO_ROOT / "config" / "simulation_params.yaml",
    REPO_ROOT / "config" / "safety_params.yaml",
    REPO_ROOT / "config" / "tactile_params.yaml",
    REPO_ROOT / "config" / "hardware_params.yaml",
]


def load_config():
    return copy.deepcopy(load_parameter_files(CONFIG_FILES))


def no_lumen_config():
    config = load_config()
    config["cylindrical_lumen"]["enabled"] = False
    config["curved_lumen"]["enabled"] = False
    return config


def cylinder_config():
    config = no_lumen_config()
    config["cylindrical_lumen"]["enabled"] = True
    return config


def curved_config(lumen_type="circular_arc"):
    config = no_lumen_config()
    config["curved_lumen"]["enabled"] = True
    config["curved_lumen"]["type"] = lumen_type
    return config


class LumenFactoryTest(unittest.TestCase):
    def test_none_mode_detection(self):
        self.assertEqual("none", lumen_mode_from_config(no_lumen_config()))

    def test_cylinder_mode_detection(self):
        self.assertEqual("cylindrical", lumen_mode_from_config(cylinder_config()))

    def test_curved_mode_detection(self):
        self.assertEqual("curved", lumen_mode_from_config(curved_config()))

    def test_configured_enabled_flags_accept_exact_bool_values(self):
        cases = (
            ("cylindrical_lumen", True, "cylindrical"),
            ("cylindrical_lumen", False, "none"),
            ("curved_lumen", True, "curved"),
            ("curved_lumen", False, "none"),
        )
        for section, value, expected_mode in cases:
            with self.subTest(section=section, value=value):
                config = no_lumen_config()
                config[section]["enabled"] = value
                self.assertEqual(expected_mode, lumen_mode_from_config(config))

    def test_configured_enabled_flags_reject_non_bool_values(self):
        invalid_values = ("false", "true", 0, 1, 0.0, [], {}, None)
        for section in ("cylindrical_lumen", "curved_lumen"):
            for value in invalid_values:
                with self.subTest(section=section, value=value):
                    config = no_lumen_config()
                    config[section]["enabled"] = value
                    with self.assertRaisesRegex(ValueError, rf"{section}\.enabled.*bool"):
                        lumen_mode_from_config(config)

    def test_missing_enabled_key_defaults_to_disabled(self):
        config = no_lumen_config()
        del config["cylindrical_lumen"]["enabled"]
        del config["curved_lumen"]["enabled"]
        self.assertFalse(cylindrical_lumen_enabled(config))
        self.assertFalse(curved_lumen_enabled(config))
        self.assertEqual("none", lumen_mode_from_config(config))

    def test_simultaneous_mode_conflict_rejected(self):
        config = curved_config()
        config["cylindrical_lumen"]["enabled"] = True
        with self.assertRaisesRegex(ValueError, "exactly one lumen geometry mode"):
            lumen_mode_from_config(config)

    def test_unsupported_curved_type_rejected(self):
        config = curved_config("spiral")
        with self.assertRaisesRegex(ValueError, "curved_lumen.type"):
            lumen_geometry_from_config(config)

    def test_cylinder_construction_returns_cylindrical_lumen(self):
        geometry = lumen_geometry_from_config(cylinder_config())
        self.assertIsInstance(geometry, CylindricalLumen)
        self.assertTrue(np.allclose([0.0, 0.0, 1.0], geometry.axis_direction))

    def test_circular_arc_construction_returns_curved_lumen(self):
        geometry = lumen_geometry_from_config(curved_config("circular_arc"))
        self.assertIsInstance(geometry, CurvedLumen)
        self.assertGreater(geometry.centerline_points.shape[0], 2)

    def test_s_curve_construction_returns_curved_lumen(self):
        geometry = lumen_geometry_from_config(curved_config("s_curve"))
        self.assertIsInstance(geometry, CurvedLumen)
        self.assertGreater(geometry.centerline_points.shape[0], 2)

    def test_circular_arc_centerline_deterministic(self):
        first = lumen_geometry_from_config(curved_config("circular_arc"))
        second = lumen_geometry_from_config(curved_config("circular_arc"))
        self.assertTrue(np.array_equal(first.centerline_points, second.centerline_points))

    def test_s_curve_centerline_deterministic(self):
        first = lumen_geometry_from_config(curved_config("s_curve"))
        second = lumen_geometry_from_config(curved_config("s_curve"))
        self.assertTrue(np.array_equal(first.centerline_points, second.centerline_points))

    def test_frame_id_preserved(self):
        config = curved_config()
        config["curved_lumen"]["frame_id"] = "lumen_frame"
        self.assertEqual("lumen_frame", lumen_geometry_from_config(config).frame_id)

    def test_radius_values_preserved(self):
        config = curved_config()
        config["curved_lumen"]["lumen_radius"] = 0.026
        config["curved_lumen"]["ctr_outer_radius"] = 0.001
        geometry = lumen_geometry_from_config(config)
        self.assertAlmostEqual(0.026, geometry.minimum_lumen_radius)
        self.assertAlmostEqual(0.001, geometry.ctr_outer_radius)

    def test_safety_margin_preserved(self):
        config = curved_config()
        config["curved_lumen"]["safety_margin"] = 0.001
        self.assertAlmostEqual(0.001, lumen_geometry_from_config(config).safety_margin)

    def test_input_configuration_not_mutated_by_construction(self):
        config = curved_config()
        original = copy.deepcopy(config)
        lumen_geometry_from_config(config)
        self.assertEqual(original, config)

    def test_override_helper_does_not_mutate_input(self):
        config = no_lumen_config()
        original = copy.deepcopy(config)
        updated = config_with_lumen_overrides(config, enable_cylindrical_lumen=True)
        self.assertEqual(original, config)
        self.assertTrue(updated["cylindrical_lumen"]["enabled"])

    def test_override_helper_deep_copies_nested_configuration(self):
        config = no_lumen_config()
        original = copy.deepcopy(config)
        updated = config_with_lumen_overrides(
            config,
            enable_curved_lumen=True,
            curved_lumen_type="s_curve",
            target=[0.01, 0.012, 0.095],
        )
        updated["curved_lumen"]["s_curve"]["inlet_position"][0] = 0.123
        updated["goal"]["position"][0] = 0.456
        self.assertEqual(original, config)
        self.assertNotEqual(updated["curved_lumen"]["s_curve"]["inlet_position"], config["curved_lumen"]["s_curve"]["inlet_position"])
        self.assertNotEqual(updated["goal"]["position"], config["goal"]["position"])

    def test_enable_cylinder_override(self):
        updated = config_with_lumen_overrides(no_lumen_config(), enable_cylindrical_lumen=True)
        self.assertTrue(cylindrical_lumen_enabled(updated))
        self.assertEqual("cylindrical", lumen_mode_from_config(updated))

    def test_enable_curved_override(self):
        updated = config_with_lumen_overrides(no_lumen_config(), enable_curved_lumen=True)
        self.assertTrue(curved_lumen_enabled(updated))
        self.assertEqual("curved", lumen_mode_from_config(updated))

    def test_curved_type_override(self):
        updated = config_with_lumen_overrides(no_lumen_config(), enable_curved_lumen=True, curved_lumen_type="s_curve")
        self.assertEqual("s_curve", updated["curved_lumen"]["type"])
        self.assertIsInstance(lumen_geometry_from_config(updated), CurvedLumen)

    def test_conflicting_override_rejected(self):
        with self.assertRaisesRegex(ValueError, "exactly one lumen geometry mode"):
            config_with_lumen_overrides(
                no_lumen_config(),
                enable_cylindrical_lumen=True,
                enable_curved_lumen=True,
            )

    def test_bool_override_arguments_accept_exact_bool_and_none(self):
        unchanged = config_with_lumen_overrides(cylinder_config(), enable_cylindrical_lumen=None)
        self.assertEqual("cylindrical", lumen_mode_from_config(unchanged))

        self.assertEqual(
            "cylindrical",
            lumen_mode_from_config(config_with_lumen_overrides(no_lumen_config(), enable_cylindrical_lumen=True)),
        )
        self.assertEqual(
            "none",
            lumen_mode_from_config(config_with_lumen_overrides(cylinder_config(), enable_cylindrical_lumen=False)),
        )
        self.assertEqual(
            "curved",
            lumen_mode_from_config(config_with_lumen_overrides(no_lumen_config(), enable_curved_lumen=True)),
        )
        self.assertEqual(
            "none",
            lumen_mode_from_config(config_with_lumen_overrides(curved_config(), enable_curved_lumen=False)),
        )

    def test_bool_override_arguments_reject_non_bool_values(self):
        cases = (
            ("enable_cylindrical_lumen", "false"),
            ("enable_cylindrical_lumen", 1),
            ("enable_curved_lumen", "false"),
            ("enable_curved_lumen", 0),
            ("enable_curved_lumen", []),
            ("enable_curved_lumen", {}),
        )
        for argument_name, value in cases:
            with self.subTest(argument_name=argument_name, value=value):
                with self.assertRaisesRegex(ValueError, rf"{argument_name}.*bool"):
                    config_with_lumen_overrides(no_lumen_config(), **{argument_name: value})

    def test_cylinder_compatibility_wrapper_returns_equivalent_geometry(self):
        config = cylinder_config()
        direct = lumen_geometry_from_config(config)
        wrapped = compatibility_lumen_geometry_from_config(config)
        self.assertIsInstance(wrapped, CylindricalLumen)
        self.assertTrue(np.allclose(direct.axis_origin, wrapped.axis_origin))
        self.assertTrue(np.allclose(direct.axis_direction, wrapped.axis_direction))
        self.assertAlmostEqual(direct.radius, wrapped.radius)

    def test_cost_weight_extraction_preserves_values(self):
        config = cylinder_config()
        weights = lumen_cost_weights_from_config(config)
        source = config["cylindrical_lumen_cost"]
        self.assertAlmostEqual(source["safety_margin_weight"], weights.safety_margin_weight)
        self.assertAlmostEqual(source["radial_collision_weight"], weights.radial_collision_weight)
        self.assertAlmostEqual(source["end_cap_weight"], weights.end_cap_weight)
        self.assertAlmostEqual(source["terminal_collision_weight"], weights.terminal_collision_weight)

    def test_none_fingerprint_deterministic(self):
        self.assertEqual(
            lumen_geometry_fingerprint(no_lumen_config()),
            lumen_geometry_fingerprint(no_lumen_config()),
        )

    def test_cylinder_fingerprint_deterministic(self):
        self.assertEqual(
            lumen_geometry_fingerprint(cylinder_config()),
            lumen_geometry_fingerprint(cylinder_config()),
        )

    def test_curved_fingerprint_deterministic(self):
        self.assertEqual(
            lumen_geometry_fingerprint(curved_config()),
            lumen_geometry_fingerprint(curved_config()),
        )

    def test_equivalent_copied_configs_have_identical_fingerprint(self):
        config = curved_config()
        self.assertEqual(lumen_geometry_fingerprint(config), lumen_geometry_fingerprint(copy.deepcopy(config)))

    def test_changed_cylinder_origin_changes_fingerprint(self):
        config = cylinder_config()
        changed = copy.deepcopy(config)
        changed["cylindrical_lumen"]["axis_origin"] = [0.001, 0.0, 0.0]
        self.assertNotEqual(lumen_geometry_fingerprint(config), lumen_geometry_fingerprint(changed))

    def test_changed_cylinder_axis_changes_fingerprint(self):
        config = cylinder_config()
        changed = copy.deepcopy(config)
        changed["cylindrical_lumen"]["axis_direction"] = [1.0, 0.0, 0.0]
        self.assertNotEqual(lumen_geometry_fingerprint(config), lumen_geometry_fingerprint(changed))

    def test_normalized_cylinder_axis_equivalence_in_fingerprint(self):
        unit_axis = cylinder_config()
        scaled_axis = copy.deepcopy(unit_axis)
        scaled_axis["cylindrical_lumen"]["axis_direction"] = [0.0, 0.0, 2.0]

        unit_geometry = lumen_geometry_from_config(unit_axis)
        scaled_geometry = lumen_geometry_from_config(scaled_axis)
        self.assertTrue(np.allclose(unit_geometry.axis_direction, scaled_geometry.axis_direction))
        self.assertEqual(lumen_geometry_fingerprint(unit_axis), lumen_geometry_fingerprint(scaled_axis))

        different_axis = copy.deepcopy(unit_axis)
        different_axis["cylindrical_lumen"]["axis_direction"] = [0.0, 1.0, 0.0]
        self.assertNotEqual(lumen_geometry_fingerprint(unit_axis), lumen_geometry_fingerprint(different_axis))

    def test_changed_circular_arc_angle_changes_fingerprint(self):
        config = curved_config("circular_arc")
        changed = copy.deepcopy(config)
        changed["curved_lumen"]["circular_arc"]["arc_angle"] = 0.71
        self.assertNotEqual(lumen_geometry_fingerprint(config), lumen_geometry_fingerprint(changed))

    def test_changed_s_curve_amplitude_changes_fingerprint(self):
        config = curved_config("s_curve")
        changed = copy.deepcopy(config)
        changed["curved_lumen"]["s_curve"]["lateral_amplitude"] = 0.020
        self.assertNotEqual(lumen_geometry_fingerprint(config), lumen_geometry_fingerprint(changed))

    def test_target_change_does_not_change_geometry_fingerprint(self):
        config = curved_config()
        changed = copy.deepcopy(config)
        changed["goal"]["position"] = [0.01, 0.01, 0.09]
        self.assertEqual(lumen_geometry_fingerprint(config), lumen_geometry_fingerprint(changed))

    def test_mppi_weight_change_does_not_change_geometry_fingerprint(self):
        config = curved_config()
        changed = copy.deepcopy(config)
        changed["mppi"]["weights"]["tip"] = 12345.0
        self.assertEqual(lumen_geometry_fingerprint(config), lumen_geometry_fingerprint(changed))

    def test_fingerprint_is_stable_hex_without_process_behavior(self):
        fingerprint = lumen_geometry_fingerprint(curved_config())
        self.assertRegex(fingerprint, re.compile(r"^[0-9a-f]{64}$"))
        self.assertNotIn("object at", str(lumen_geometry_fingerprint_payload(curved_config())))

    def test_non_finite_fingerprint_input_rejected(self):
        config = cylinder_config()
        config["cylindrical_lumen"]["axis_origin"] = [math.nan, 0.0, 0.0]
        with self.assertRaisesRegex(ValueError, "finite"):
            lumen_geometry_fingerprint(config)

    def test_mppi_profile_override_preserves_cylinder_fast_values(self):
        config = no_lumen_config()
        updated = config_with_mppi_profile(config, "cylinder_fast")
        profile = config["mppi_profiles"]["cylinder_fast"]
        self.assertEqual(profile["samples"], updated["mppi"]["num_samples"])
        self.assertEqual(profile["horizon"], updated["mppi"]["horizon"])
        self.assertAlmostEqual(1.0 / profile["control_period"], updated["mppi"]["control_frequency"])


if __name__ == "__main__":
    unittest.main()
