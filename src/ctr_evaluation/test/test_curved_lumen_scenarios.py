import copy
import dataclasses
import math
import sys
import unittest
from pathlib import Path
from unittest import mock

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_mppi_controller"))

from ctr_bringup.parameter_validation import load_parameter_files  # noqa: E402
import ctr_evaluation.curved_lumen_scenarios as scenario_module  # noqa: E402
from ctr_evaluation.curved_lumen_scenarios import (  # noqa: E402
    CENTERLINE_TARGET,
    CURVED_LUMEN_SCENARIO_IDS,
    CURVED_SCENARIO_POLICY_VERSION,
    LATERAL_OFFSET_TARGET,
    NEAR_SAFETY_BOUNDARY_TARGET,
    SCENARIO_CENTERLINE_FRACTIONS,
    CurvedLumenScenario,
    _centerline_data,
    _sample_centerline,
    resolve_curved_lumen_scenario,
)
from ctr_mppi_controller.curved_lumen import CurvedLumen  # noqa: E402
from ctr_mppi_controller.cylindrical_lumen import CylindricalLumen  # noqa: E402
from ctr_mppi_controller.lumen_factory import (  # noqa: E402
    config_with_lumen_overrides,
    lumen_geometry_fingerprint,
    lumen_geometry_from_config,
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


def curved_config(lumen_type="circular_arc"):
    return config_with_lumen_overrides(
        load_config(),
        enable_cylindrical_lumen=False,
        enable_curved_lumen=True,
        curved_lumen_type=lumen_type,
    )


def curved_geometry(lumen_type="circular_arc"):
    return lumen_geometry_from_config(curved_config(lumen_type))


def cylinder_lumen():
    return CylindricalLumen(
        frame_id="base_link",
        axis_origin=[0.0, 0.0, 0.0],
        axis_direction=[0.0, 0.0, 1.0],
        radius=0.030,
        length=0.120,
        ctr_outer_radius=0.0015,
        safety_margin=0.0020,
    )


def variable_radius_lumen():
    return CurvedLumen(
        frame_id="base_link",
        centerline_points=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.040], [0.0, 0.0, 0.120]],
        lumen_radius=np.asarray([0.020, 0.030, 0.040], dtype=float),
        ctr_outer_radius=0.0015,
        safety_margin=0.0020,
    )


def assert_no_nonfinite(testcase, value):
    if dataclasses.is_dataclass(value):
        for field in dataclasses.fields(value):
            assert_no_nonfinite(testcase, getattr(value, field.name))
    elif isinstance(value, np.ndarray):
        testcase.assertTrue(np.all(np.isfinite(value)))
    elif isinstance(value, tuple):
        for item in value:
            assert_no_nonfinite(testcase, item)
    elif isinstance(value, float):
        testcase.assertTrue(math.isfinite(value))


class CurvedLumenScenarioResolutionTest(unittest.TestCase):
    def test_default_scenarios_are_geometry_relative_and_safety_valid(self):
        for lumen_type in ("circular_arc", "s_curve"):
            geometry = curved_geometry(lumen_type)
            for scenario_id in CURVED_LUMEN_SCENARIO_IDS:
                with self.subTest(lumen_type=lumen_type, scenario_id=scenario_id):
                    scenario = resolve_curved_lumen_scenario(curved_config(lumen_type), scenario_id)
                    validation = geometry.validate_target(
                        scenario.validated_target,
                        frame_id=scenario.geometry_frame,
                        require_safety_margin=True,
                    )
                    self.assertTrue(validation.valid, validation.reasons)
                    self.assertEqual(CURVED_SCENARIO_POLICY_VERSION, scenario.policy_version)
                    self.assertEqual("curved", scenario.geometry_mode)
                    self.assertEqual("base_link", scenario.geometry_frame)
                    self.assertEqual(lumen_geometry_fingerprint(geometry), scenario.geometry_fingerprint)
                    self.assertEqual(lumen_type, scenario.curved_lumen_type)
                    self.assertEqual(SCENARIO_CENTERLINE_FRACTIONS[scenario_id], scenario.centerline_fraction)
                    self.assertAlmostEqual(
                        scenario.centerline_arc_length,
                        scenario.centerline_fraction * geometry.length,
                        places=12,
                    )
                    self.assertTrue(np.allclose(scenario.derived_target, scenario.requested_target))
                    self.assertTrue(np.allclose(scenario.requested_target, scenario.validated_target))
                    self.assertFalse(scenario.override_used)
                    self.assertTrue(scenario.require_safety_margin)
                    self.assertEqual(scenario_id == NEAR_SAFETY_BOUNDARY_TARGET, scenario.near_boundary)
                    self.assertAlmostEqual(np.linalg.norm(scenario.local_tangent), 1.0, places=12)
                    self.assertAlmostEqual(np.linalg.norm(scenario.radial_direction), 1.0, places=12)
                    self.assertAlmostEqual(float(np.dot(scenario.local_tangent, scenario.radial_direction)), 0.0, places=12)
                    self.assertGreater(scenario.preferred_radius, 0.0)
                    if scenario_id == CENTERLINE_TARGET:
                        self.assertEqual(0.0, scenario.radial_offset)
                        self.assertTrue(np.allclose(scenario.centerline_point, scenario.derived_target))
                    elif scenario_id == LATERAL_OFFSET_TARGET:
                        self.assertAlmostEqual(0.5 * scenario.preferred_radius, scenario.radial_offset, places=15)
                    else:
                        self.assertAlmostEqual(
                            min(0.001, 0.10 * scenario.preferred_radius),
                            scenario.boundary_guard,
                            places=15,
                        )
                        self.assertAlmostEqual(
                            scenario.preferred_radius - scenario.boundary_guard,
                            scenario.radial_offset,
                            places=15,
                        )
                        self.assertGreaterEqual(validation.clearance.safety_margin_clearance, 0.0)
                    self.assertFalse(validation.clearance.inlet_violation)
                    self.assertFalse(validation.clearance.outlet_violation)
                    assert_no_nonfinite(self, scenario)

    def test_circular_arc_and_s_curve_scenarios_preserve_policy_fractions(self):
        for lumen_type in ("circular_arc", "s_curve"):
            for scenario_id, expected_fraction in SCENARIO_CENTERLINE_FRACTIONS.items():
                with self.subTest(lumen_type=lumen_type, scenario_id=scenario_id):
                    scenario = resolve_curved_lumen_scenario(curved_config(lumen_type), scenario_id)
                    self.assertEqual(expected_fraction, scenario.centerline_fraction)

    def test_arc_length_sampling_is_not_raw_index_sampling(self):
        geometry = CurvedLumen(
            frame_id="base_link",
            centerline_points=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.010], [0.0, 0.0, 0.110]],
            lumen_radius=0.030,
            ctr_outer_radius=0.0015,
            safety_margin=0.0020,
        )
        scenario = resolve_curved_lumen_scenario(
            curved_config("circular_arc"),
            CENTERLINE_TARGET,
            curved_lumen_type="circular_arc",
            geometry=geometry,
        )
        self.assertEqual(1, scenario.centerline_segment_index)
        self.assertAlmostEqual(0.70 * geometry.length, scenario.centerline_arc_length, places=12)
        self.assertAlmostEqual((scenario.centerline_arc_length - 0.010) / 0.100, scenario.centerline_segment_parameter)
        self.assertGreater(scenario.centerline_segment_parameter, 0.5)

    def test_internal_resolution_uses_one_geometry_for_identity_and_validation(self):
        constructed = []
        validation_objects = []
        real_factory = scenario_module.lumen_geometry_from_config

        def factory_spy(config):
            geometry = real_factory(config)
            original_validate = geometry.validate_target

            def validate_spy(*args, **kwargs):
                validation_objects.append(geometry)
                return original_validate(*args, **kwargs)

            object.__setattr__(geometry, "validate_target", validate_spy)
            constructed.append(geometry)
            return geometry

        with (
            mock.patch.object(scenario_module, "lumen_geometry_from_config", side_effect=factory_spy) as factory,
            mock.patch.object(
                scenario_module,
                "lumen_geometry_fingerprint_payload",
                wraps=scenario_module.lumen_geometry_fingerprint_payload,
            ) as payload,
            mock.patch.object(
                scenario_module,
                "lumen_geometry_fingerprint",
                wraps=scenario_module.lumen_geometry_fingerprint,
            ) as fingerprint,
        ):
            scenario = scenario_module.resolve_curved_lumen_scenario(curved_config("circular_arc"), LATERAL_OFFSET_TARGET)

        self.assertEqual(1, factory.call_count)
        self.assertEqual(1, len(constructed))
        self.assertEqual(1, len(validation_objects))
        geometry = constructed[0]
        self.assertIs(validation_objects[0], geometry)
        payload.assert_called_once_with(geometry)
        fingerprint.assert_called_once_with(geometry)
        self.assertEqual(lumen_geometry_fingerprint(geometry), scenario.geometry_fingerprint)
        self.assertTrue(np.array_equal(geometry.validate_target(scenario.validated_target).target, scenario.validated_target))

    def test_explicit_geometry_path_constructs_no_second_geometry(self):
        geometry = curved_geometry("s_curve")
        validation_objects = []
        original_validate = geometry.validate_target

        def validate_spy(*args, **kwargs):
            validation_objects.append(geometry)
            return original_validate(*args, **kwargs)

        object.__setattr__(geometry, "validate_target", validate_spy)
        with (
            mock.patch.object(scenario_module, "lumen_geometry_from_config") as factory,
            mock.patch.object(
                scenario_module,
                "lumen_geometry_fingerprint_payload",
                wraps=scenario_module.lumen_geometry_fingerprint_payload,
            ) as payload,
            mock.patch.object(
                scenario_module,
                "lumen_geometry_fingerprint",
                wraps=scenario_module.lumen_geometry_fingerprint,
            ) as fingerprint,
        ):
            scenario = scenario_module.resolve_curved_lumen_scenario(
                curved_config("s_curve"),
                NEAR_SAFETY_BOUNDARY_TARGET,
                geometry=geometry,
            )

        factory.assert_not_called()
        self.assertEqual(1, len(validation_objects))
        self.assertIs(validation_objects[0], geometry)
        payload.assert_called_once_with(geometry)
        fingerprint.assert_called_once_with(geometry)
        self.assertEqual(lumen_geometry_fingerprint(geometry), scenario.geometry_fingerprint)

    def test_segment_boundary_tie_uses_lower_segment_except_final_endpoint(self):
        geometry = CurvedLumen(
            frame_id="base_link",
            centerline_points=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.050], [0.0, 0.0, 0.100]],
            lumen_radius=0.030,
            ctr_outer_radius=0.0015,
            safety_margin=0.0020,
        )
        data = _centerline_data(geometry)
        midpoint = _sample_centerline(data, 0.5)
        endpoint = _sample_centerline(data, 1.0)
        self.assertEqual(0, midpoint.segment_index)
        self.assertEqual(1.0, midpoint.segment_parameter)
        self.assertEqual(1, endpoint.segment_index)
        self.assertEqual(1.0, endpoint.segment_parameter)

    def test_parallel_reference_normal_uses_deterministic_cartesian_fallback(self):
        config = curved_config("circular_arc")
        config["curved_lumen"]["circular_arc"]["bend_normal"] = [0.0, 0.0, 1.0]
        geometry = CurvedLumen(
            frame_id="base_link",
            centerline_points=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.120]],
            lumen_radius=0.030,
            ctr_outer_radius=0.0015,
            safety_margin=0.0020,
        )
        first = resolve_curved_lumen_scenario(config, LATERAL_OFFSET_TARGET, geometry=geometry)
        second = resolve_curved_lumen_scenario(config, LATERAL_OFFSET_TARGET, geometry=geometry)
        self.assertTrue(np.allclose([1.0, 0.0, 0.0], first.radial_direction))
        self.assertTrue(np.array_equal(first.radial_direction, second.radial_direction))
        self.assertEqual(first.scenario_fingerprint, second.scenario_fingerprint)

    def test_repeated_resolution_is_identical_and_seed_values_do_not_affect_identity(self):
        config = curved_config("s_curve")
        seeded = copy.deepcopy(config)
        seeded["mppi"]["random_seed"] = 98765
        first = resolve_curved_lumen_scenario(config, NEAR_SAFETY_BOUNDARY_TARGET)
        second = resolve_curved_lumen_scenario(config, NEAR_SAFETY_BOUNDARY_TARGET)
        seeded_result = resolve_curved_lumen_scenario(seeded, NEAR_SAFETY_BOUNDARY_TARGET)
        self.assertEqual(first.geometry_fingerprint, second.geometry_fingerprint)
        self.assertEqual(first.geometry_fingerprint, seeded_result.geometry_fingerprint)
        self.assertEqual(first.scenario_fingerprint, second.scenario_fingerprint)
        self.assertEqual(first.scenario_identity_payload, second.scenario_identity_payload)
        self.assertTrue(np.array_equal(first.derived_target, second.derived_target))
        self.assertEqual(first.scenario_fingerprint, seeded_result.scenario_fingerprint)
        self.assertEqual(first.scenario_identity_payload, seeded_result.scenario_identity_payload)

    def test_geometry_types_and_scenario_ids_produce_distinct_identities(self):
        identities = set()
        for lumen_type in ("circular_arc", "s_curve"):
            for scenario_id in CURVED_LUMEN_SCENARIO_IDS:
                scenario = resolve_curved_lumen_scenario(curved_config(lumen_type), scenario_id)
                identities.add(scenario.scenario_fingerprint)
        self.assertEqual(6, len(identities))
        arc = resolve_curved_lumen_scenario(curved_config("circular_arc"), CENTERLINE_TARGET)
        s_curve = resolve_curved_lumen_scenario(curved_config("s_curve"), CENTERLINE_TARGET)
        self.assertNotEqual(arc.geometry_fingerprint, s_curve.geometry_fingerprint)

    def test_invalid_scenario_type_and_non_curved_geometry_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "unsupported curved scenario"):
            resolve_curved_lumen_scenario(curved_config("circular_arc"), "spiral_target")
        with self.assertRaisesRegex(ValueError, "unsupported curved lumen type"):
            resolve_curved_lumen_scenario(curved_config("circular_arc"), CENTERLINE_TARGET, curved_lumen_type="spiral")
        with self.assertRaisesRegex(ValueError, "CurvedLumen"):
            resolve_curved_lumen_scenario(
                curved_config("circular_arc"),
                CENTERLINE_TARGET,
                geometry=cylinder_lumen(),
            )
        with self.assertRaisesRegex(ValueError, "CurvedLumen"):
            resolve_curved_lumen_scenario(
                curved_config("circular_arc"),
                CENTERLINE_TARGET,
                geometry=None if False else object(),
            )

    def test_malformed_centerline_defensive_validation(self):
        geometry = object.__new__(CurvedLumen)
        object.__setattr__(geometry, "frame_id", "base_link")
        object.__setattr__(geometry, "centerline_points", np.asarray([[0.0, 0.0, 0.0]], dtype=float))
        object.__setattr__(geometry, "segment_vectors", np.empty((0, 3), dtype=float))
        object.__setattr__(geometry, "segment_lengths", np.empty((0,), dtype=float))
        object.__setattr__(geometry, "segment_unit_vectors", np.empty((0, 3), dtype=float))
        object.__setattr__(geometry, "cumulative_arc_lengths", np.asarray([0.0], dtype=float))
        object.__setattr__(geometry, "length", 0.0)
        object.__setattr__(geometry, "radius_profile", np.asarray([0.030], dtype=float))
        object.__setattr__(geometry, "ctr_outer_radius", 0.0015)
        object.__setattr__(geometry, "safety_margin", 0.0020)
        with self.assertRaisesRegex(ValueError, "at least two"):
            resolve_curved_lumen_scenario(curved_config("circular_arc"), CENTERLINE_TARGET, geometry=geometry)

    def test_zero_length_centerline_defensive_validation(self):
        geometry = object.__new__(CurvedLumen)
        object.__setattr__(geometry, "frame_id", "base_link")
        object.__setattr__(
            geometry,
            "centerline_points",
            np.asarray([[0.0, 0.0, 0.0], [0.0, 0.0, 0.010]], dtype=float),
        )
        object.__setattr__(geometry, "segment_vectors", np.asarray([[0.0, 0.0, 0.0]], dtype=float))
        object.__setattr__(geometry, "segment_lengths", np.asarray([0.0], dtype=float))
        object.__setattr__(geometry, "segment_unit_vectors", np.asarray([[0.0, 0.0, 0.0]], dtype=float))
        object.__setattr__(geometry, "cumulative_arc_lengths", np.asarray([0.0, 0.0], dtype=float))
        object.__setattr__(geometry, "length", 0.0)
        object.__setattr__(geometry, "radius_profile", np.asarray([0.030, 0.030], dtype=float))
        object.__setattr__(geometry, "ctr_outer_radius", 0.0015)
        object.__setattr__(geometry, "safety_margin", 0.0020)
        with self.assertRaisesRegex(ValueError, "positive"):
            resolve_curved_lumen_scenario(curved_config("circular_arc"), CENTERLINE_TARGET, geometry=geometry)

    def test_variable_radius_uses_local_selected_radius(self):
        geometry = variable_radius_lumen()
        scenario = resolve_curved_lumen_scenario(
            curved_config("circular_arc"),
            LATERAL_OFFSET_TARGET,
            curved_lumen_type="circular_arc",
            geometry=geometry,
        )
        expected_radius = (1.0 - scenario.centerline_segment_parameter) * geometry.radius_profile[
            scenario.centerline_segment_index
        ] + scenario.centerline_segment_parameter * geometry.radius_profile[scenario.centerline_segment_index + 1]
        self.assertAlmostEqual(float(expected_radius), scenario.local_radius, places=15)
        self.assertNotEqual(float(geometry.radius_profile[0]), scenario.local_radius)
        self.assertAlmostEqual(
            scenario.local_radius - geometry.ctr_outer_radius - geometry.safety_margin,
            scenario.preferred_radius,
            places=15,
        )

    def test_full_target_override_is_validated_and_preserved(self):
        nominal = resolve_curved_lumen_scenario(curved_config("circular_arc"), CENTERLINE_TARGET)
        override = nominal.derived_target + 0.25 * nominal.preferred_radius * nominal.radial_direction
        result = resolve_curved_lumen_scenario(curved_config("circular_arc"), CENTERLINE_TARGET, target_override=override)
        self.assertTrue(result.override_used)
        self.assertTrue(np.array_equal(override, result.requested_target))
        self.assertTrue(np.array_equal(override, result.validated_target))
        self.assertTrue(np.array_equal(nominal.derived_target, result.derived_target))
        self.assertNotEqual(nominal.scenario_fingerprint, result.scenario_fingerprint)

    def test_malformed_partial_nonfinite_and_invalid_overrides_are_rejected(self):
        valid = resolve_curved_lumen_scenario(curved_config("circular_arc"), CENTERLINE_TARGET)
        cases = (
            [valid.derived_target[0], valid.derived_target[1]],
            {"x": 0.0, "y": 0.0},
            [[valid.derived_target[0], valid.derived_target[1], valid.derived_target[2]]],
            ["x", 0.0, 0.0],
            [math.nan, 0.0, 0.080],
            [math.inf, 0.0, 0.080],
            [0.0, 0.0, -math.inf],
        )
        for target in cases:
            with self.subTest(target=target):
                with self.assertRaisesRegex(ValueError, "target_override"):
                    resolve_curved_lumen_scenario(curved_config("circular_arc"), CENTERLINE_TARGET, target_override=target)
        with self.assertRaisesRegex(ValueError, "invalid target"):
            resolve_curved_lumen_scenario(
                curved_config("circular_arc"),
                CENTERLINE_TARGET,
                target_override=[0.200, 0.0, 0.080],
            )
        with self.assertRaisesRegex(ValueError, "invalid target"):
            resolve_curved_lumen_scenario(
                curved_config("circular_arc"),
                CENTERLINE_TARGET,
                target_override=valid.centerline_point + 1.01 * valid.preferred_radius * valid.radial_direction,
            )
        geometry = curved_geometry("circular_arc")
        inlet_invalid = geometry.centerline_points[0] - 0.005 * geometry.inlet_tangent
        outlet_invalid = geometry.centerline_points[-1] + 0.005 * geometry.outlet_tangent
        with self.assertRaisesRegex(ValueError, "inlet"):
            resolve_curved_lumen_scenario(
                curved_config("circular_arc"),
                CENTERLINE_TARGET,
                target_override=inlet_invalid,
            )
        with self.assertRaisesRegex(ValueError, "outlet"):
            resolve_curved_lumen_scenario(
                curved_config("circular_arc"),
                CENTERLINE_TARGET,
                target_override=outlet_invalid,
            )

    def test_retained_arrays_are_read_only_and_do_not_alias_override(self):
        override = np.asarray([0.020, 0.0, 0.080], dtype=float)
        scenario = resolve_curved_lumen_scenario(curved_config("circular_arc"), CENTERLINE_TARGET, target_override=override)
        retained = (
            scenario.centerline_point,
            scenario.local_tangent,
            scenario.radial_direction,
            scenario.derived_target,
            scenario.requested_target,
            scenario.validated_target,
        )
        for array in retained:
            with self.subTest(array=array):
                self.assertFalse(array.flags.writeable)
                with self.assertRaises(ValueError):
                    array[0] = array[0] + 1.0
        override[:] = 0.0
        self.assertFalse(np.array_equal(override, scenario.requested_target))

    def test_source_config_and_geometry_are_not_mutated(self):
        config = curved_config("s_curve")
        geometry = curved_geometry("s_curve")
        config_before = copy.deepcopy(config)
        signatures_before = (
            geometry.centerline_points.copy(),
            geometry.radius_profile.copy(),
            geometry.segment_vectors.copy(),
            geometry.segment_lengths.copy(),
            geometry.segment_unit_vectors.copy(),
            geometry.cumulative_arc_lengths.copy(),
        )
        object_ids_before = (
            id(geometry.centerline_points),
            id(geometry.radius_profile),
            id(geometry.segment_vectors),
            id(geometry.segment_lengths),
            id(geometry.segment_unit_vectors),
            id(geometry.cumulative_arc_lengths),
        )
        writeable_before = (
            geometry.centerline_points.flags.writeable,
            geometry.radius_profile.flags.writeable,
            geometry.segment_vectors.flags.writeable,
            geometry.segment_lengths.flags.writeable,
            geometry.segment_unit_vectors.flags.writeable,
            geometry.cumulative_arc_lengths.flags.writeable,
        )
        resolve_curved_lumen_scenario(config, NEAR_SAFETY_BOUNDARY_TARGET, geometry=geometry)
        self.assertEqual(config_before, config)
        self.assertTrue(np.array_equal(signatures_before[0], geometry.centerline_points))
        self.assertTrue(np.array_equal(signatures_before[1], geometry.radius_profile))
        self.assertTrue(np.array_equal(signatures_before[2], geometry.segment_vectors))
        self.assertTrue(np.array_equal(signatures_before[3], geometry.segment_lengths))
        self.assertTrue(np.array_equal(signatures_before[4], geometry.segment_unit_vectors))
        self.assertTrue(np.array_equal(signatures_before[5], geometry.cumulative_arc_lengths))
        self.assertEqual(object_ids_before[0], id(geometry.centerline_points))
        self.assertEqual(object_ids_before[1], id(geometry.radius_profile))
        self.assertEqual(object_ids_before[2], id(geometry.segment_vectors))
        self.assertEqual(object_ids_before[3], id(geometry.segment_lengths))
        self.assertEqual(object_ids_before[4], id(geometry.segment_unit_vectors))
        self.assertEqual(object_ids_before[5], id(geometry.cumulative_arc_lengths))
        self.assertEqual(writeable_before[0], geometry.centerline_points.flags.writeable)
        self.assertEqual(writeable_before[1], geometry.radius_profile.flags.writeable)
        self.assertEqual(writeable_before[2], geometry.segment_vectors.flags.writeable)
        self.assertEqual(writeable_before[3], geometry.segment_lengths.flags.writeable)
        self.assertEqual(writeable_before[4], geometry.segment_unit_vectors.flags.writeable)
        self.assertEqual(writeable_before[5], geometry.cumulative_arc_lengths.flags.writeable)

    def test_identity_payload_excludes_runtime_state(self):
        scenario = resolve_curved_lumen_scenario(curved_config("circular_arc"), NEAR_SAFETY_BOUNDARY_TARGET)
        identity_text = repr(scenario.scenario_identity_payload)
        for forbidden in ("timestamp", "host", "pid", "/tmp", "evaluation/", "run_id", "seed"):
            self.assertNotIn(forbidden, identity_text)
        self.assertEqual(64, len(scenario.scenario_fingerprint))
        self.assertTrue(all(character in "0123456789abcdef" for character in scenario.scenario_fingerprint))

    def test_result_is_frozen(self):
        scenario = resolve_curved_lumen_scenario(curved_config("circular_arc"), CENTERLINE_TARGET)
        with self.assertRaises(dataclasses.FrozenInstanceError):
            scenario.scenario_id = "other"


if __name__ == "__main__":
    unittest.main()
