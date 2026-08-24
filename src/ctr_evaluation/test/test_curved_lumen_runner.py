import json
import math
import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_model"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_mppi_controller"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_sim"))

import ctr_evaluation.run_evaluation as run_module  # noqa: E402
from ctr_bringup.parameter_validation import load_parameter_files  # noqa: E402
from ctr_evaluation.metrics import sanitize_for_json  # noqa: E402
from ctr_evaluation.run_evaluation import (  # noqa: E402
    CommandAudit,
    EvaluationOrchestrator,
    OrchestrationError,
    StabilityStats,
    build_base_simulation_command,
    build_shared_environment_hash,
    parse_args,
)
from ctr_evaluation.curved_lumen_scenarios import (  # noqa: E402
    CENTERLINE_TARGET,
    LATERAL_OFFSET_TARGET,
    NEAR_SAFETY_BOUNDARY_TARGET,
    S_CURVE_MIDDLE_TARGET,
    S_CURVE_NEAR_OUTLET_TARGET,
)


CONFIG_FILES = [
    REPO_ROOT / "config" / "robot_params.yaml",
    REPO_ROOT / "config" / "model_params.yaml",
    REPO_ROOT / "config" / "mppi_params.yaml",
    REPO_ROOT / "config" / "simulation_params.yaml",
    REPO_ROOT / "config" / "evaluation_params.yaml",
    REPO_ROOT / "config" / "safety_params.yaml",
    REPO_ROOT / "config" / "tactile_params.yaml",
    REPO_ROOT / "config" / "hardware_params.yaml",
]


def curved_args(*extra: str):
    return parse_args(
        [
            "--experiment-group",
            "d2b1_curved",
            "--task",
            "curved_lumen_navigation",
            "--duration",
            "8.0",
            *extra,
        ]
    )


def orchestrator(*extra: str) -> EvaluationOrchestrator:
    return EvaluationOrchestrator(curved_args(*extra))


def stable_state() -> StabilityStats:
    return StabilityStats(
        stable=True,
        reason="ok",
        first_q=[0.0] * 6,
        first_tip=[0.0, 0.0, 0.08],
        mean_q_variation=0.0,
        max_q_variation=0.0,
        mean_tip_variation=0.0,
        max_tip_variation=0.0,
        sample_count=10,
        consecutive_stable_samples=10,
        duration_s=0.5,
    )


class CurvedLumenRunnerTest(unittest.TestCase):
    def test_cli_registration_and_legacy_defaults(self):
        args = parse_args(
            [
                "--experiment-group",
                "curved_cli",
                "--task",
                "curved_lumen_navigation",
                "--curved-lumen-type",
                "s_curve",
                "--scenario",
                NEAR_SAFETY_BOUNDARY_TARGET,
                "--duration",
                "8.0",
            ]
        )
        self.assertEqual("curved_lumen_navigation", args.task)
        self.assertEqual("s_curve", args.curved_lumen_type)
        self.assertEqual(NEAR_SAFETY_BOUNDARY_TARGET, args.scenario)

        trajectory = parse_args(["--experiment-group", "legacy_trajectory", "--duration", "12.0"])
        self.assertEqual("trajectory", trajectory.task)
        self.assertIsNone(trajectory.curved_lumen_type)
        self.assertIsNone(trajectory.scenario)

        cylinder = parse_args(
            [
                "--experiment-group",
                "legacy_cylinder",
                "--task",
                "cylinder_navigation",
                "--duration",
                "8.0",
            ]
        )
        self.assertEqual("cylinder_navigation", cylinder.task)
        self.assertIsNone(cylinder.curved_lumen_type)
        self.assertIsNone(cylinder.scenario)

    def test_parser_rejects_invalid_curved_choices(self):
        invalid_values = (
            ["--curved-lumen-type", "spiral"],
            ["--scenario", "unknown_target"],
        )
        for values in invalid_values:
            with self.subTest(values=values):
                with self.assertRaises(SystemExit):
                    curved_args(*values)

    def test_curved_only_options_are_rejected_for_legacy_tasks(self):
        cases = (
            ["--task", "trajectory", "--curved-lumen-type", "circular_arc"],
            ["--task", "trajectory", "--scenario", CENTERLINE_TARGET],
            ["--task", "cylinder_navigation", "--curved-lumen-type", "s_curve"],
            ["--task", "cylinder_navigation", "--scenario", LATERAL_OFFSET_TARGET],
        )
        for values in cases:
            args = parse_args(["--experiment-group", "misuse", "--duration", "8.0", *values])
            with self.subTest(values=values):
                with self.assertRaisesRegex(OrchestrationError, "only valid with --task curved_lumen_navigation"):
                    EvaluationOrchestrator(args)

    def test_curved_defaults_and_single_scenario_resolution(self):
        calls = []
        original = run_module.resolve_curved_lumen_scenario

        def spy(*args, **kwargs):
            result = original(*args, **kwargs)
            calls.append((args, kwargs, result))
            return result

        run_module.resolve_curved_lumen_scenario = spy
        try:
            orch = orchestrator()
        finally:
            run_module.resolve_curved_lumen_scenario = original

        self.assertEqual(1, len(calls))
        self.assertEqual(CENTERLINE_TARGET, calls[0][0][1])
        self.assertEqual("circular_arc", calls[0][1]["curved_lumen_type"])
        self.assertIsNone(calls[0][2].requested_target.base)
        self.assertEqual("circular_arc", orch.curved_lumen_type)
        self.assertEqual(CENTERLINE_TARGET, orch.curved_scenario_id)
        self.assertEqual(CENTERLINE_TARGET, orch.curved_scenario.scenario_id)
        self.assertFalse(orch.project_config["cylindrical_lumen"]["enabled"])
        self.assertTrue(orch.project_config["curved_lumen"]["enabled"])
        self.assertEqual("circular_arc", orch.project_config["curved_lumen"]["type"])
        self.assertEqual("fixed_target", orch.project_config["reference"]["mode"])

    def test_selected_scenario_and_target_override_are_validated_once(self):
        nominal = orchestrator("--curved-lumen-type", "s_curve", "--scenario", LATERAL_OFFSET_TARGET)
        override = [f"{value:.12f}" for value in nominal.curved_scenario.validated_target]
        calls = []
        original = run_module.resolve_curved_lumen_scenario

        def spy(*args, **kwargs):
            result = original(*args, **kwargs)
            calls.append((args, kwargs, result))
            return result

        run_module.resolve_curved_lumen_scenario = spy
        try:
            overridden = orchestrator(
                "--curved-lumen-type",
                "s_curve",
                "--scenario",
                LATERAL_OFFSET_TARGET,
                "--target",
                *override,
            )
        finally:
            run_module.resolve_curved_lumen_scenario = original

        self.assertEqual(1, len(calls))
        self.assertEqual(LATERAL_OFFSET_TARGET, calls[0][0][1])
        self.assertEqual("s_curve", calls[0][1]["curved_lumen_type"])
        np.testing.assert_allclose(np.asarray(override, dtype=float), calls[0][1]["target_override"], atol=0.0, rtol=0.0)
        self.assertTrue(overridden.curved_scenario.override_used)
        np.testing.assert_allclose(overridden._target_position_for_launch(), nominal.curved_scenario.validated_target, atol=1.0e-12, rtol=0.0)

    def test_s_curve_target_identity_is_resolved_once_and_seed_independent(self):
        for scenario_id in (S_CURVE_MIDDLE_TARGET, S_CURVE_NEAR_OUTLET_TARGET):
            resolved = []
            for seed in (11, 22, 33):
                orch = orchestrator(
                    "--curved-lumen-type", "s_curve", "--scenario", scenario_id, "--seed", str(seed)
                )
                identity = orch._curved_scenario_identity_metadata()
                resolved.append(identity)
                self.assertEqual(scenario_id, identity["scenario_id"])
                self.assertEqual("s_curve", identity["curved_lumen_type"])
                self.assertEqual("fixed_target", identity["target_mode"])
                self.assertEqual(0.003, identity["target_tolerance"])
                self.assertEqual(0.5, identity["required_hold_duration"])
                self.assertEqual(0.50 if scenario_id == S_CURVE_MIDDLE_TARGET else 0.90, identity["centerline_fraction"])
                self.assertEqual(0.0, identity["radial_offset"])
                self.assertEqual(identity["requested_target"], identity["executed_target"])
                self.assertEqual(identity["geometry_fingerprint"], orch.curved_scenario.geometry_fingerprint)
                json.dumps(sanitize_for_json(identity), allow_nan=False)
            self.assertEqual(resolved[0], resolved[1])
            self.assertEqual(resolved[1], resolved[2])

    def test_invalid_curved_target_fails_before_process_start(self):
        calls = []
        original_start = run_module.ProcessManager.start
        run_module.ProcessManager.start = lambda *args, **kwargs: calls.append((args, kwargs))
        try:
            with self.assertRaisesRegex(ValueError, "invalid target"):
                orchestrator("--target", "1.0", "1.0", "1.0")
        finally:
            run_module.ProcessManager.start = original_start
        self.assertEqual([], calls)

    def test_conflicting_reference_mode_fails_before_process_start(self):
        config = load_parameter_files(CONFIG_FILES)
        config["reference"]["mode"] = "trajectory"
        calls = []
        original_loader = run_module.load_parameter_files
        original_start = run_module.ProcessManager.start
        run_module.load_parameter_files = lambda paths: config
        run_module.ProcessManager.start = lambda *args, **kwargs: calls.append((args, kwargs))
        try:
            with self.assertRaisesRegex(OrchestrationError, "reference.mode=fixed_target"):
                orchestrator()
        finally:
            run_module.load_parameter_files = original_loader
            run_module.ProcessManager.start = original_start
        self.assertEqual([], calls)

    def test_curved_base_launch_command_uses_fixed_target_mapping(self):
        target = [0.0123456789, -0.004, 0.101]
        baseline = build_base_simulation_command(
            experiment_group="group",
            controller_label="zero_command",
            baseline_dir=None,
            task="curved_lumen_navigation",
            target_position=target,
            curved_lumen_type="s_curve",
            mppi_profile="cylinder_fast",
            random_seed=11,
            run_role="baseline",
        )
        self.assertIn("start_mppi_controller:=false", baseline)
        self.assertIn("enable_cylindrical_lumen:=false", baseline)
        self.assertIn("enable_curved_lumen:=true", baseline)
        self.assertIn("curved_lumen_type:=s_curve", baseline)
        self.assertIn("reference_mode:=fixed_target", baseline)
        self.assertIn("cylinder_target_x:=0.0123456789", baseline)
        self.assertIn("cylinder_target_y:=-0.0040000000000000001", baseline)
        self.assertIn("cylinder_target_z:=0.10100000000000001", baseline)
        self.assertIn("cylinder_profile:=cylinder_fast", baseline)
        self.assertIn("mppi_random_seed:=11", baseline)
        self.assertIn("run_role:=baseline", baseline)
        self.assertNotIn("enable_cylindrical_lumen:=true", baseline)

    def test_curved_reference_and_controller_commands_are_fixed_target(self):
        orch = orchestrator(
            "--curved-lumen-type",
            "s_curve",
            "--scenario",
            NEAR_SAFETY_BOUNDARY_TARGET,
            "--mppi-profile",
            "cylinder_fast",
            "--seed",
            "7",
        )
        reference = orch._reference_command(12.5)
        controller = orch._controller_command()

        for command in (reference, controller):
            self.assertIn("reference_mode:=fixed_target", command)
            self.assertIn("enable_cylindrical_lumen:=false", command)
            self.assertIn("enable_curved_lumen:=true", command)
            self.assertIn("curved_lumen_type:=s_curve", command)
            self.assertIn("cylinder_profile:=cylinder_fast", command)
            self.assertIn("mppi_random_seed:=7", command)
            for index, axis in enumerate(("x", "y", "z")):
                expected = f"cylinder_target_{axis}:={orch.curved_scenario.validated_target[index]:.17g}"
                self.assertIn(expected, command)
        self.assertEqual("evaluation_reference.launch.py", reference[3])
        self.assertNotIn("trajectory_start_policy:=scheduled_time", reference)
        self.assertTrue(all(not item.startswith("scheduled_reference_epoch:=") for item in reference))
        self.assertEqual("evaluation_mppi_controller.launch.py", controller[3])
        self.assertIn("publish_safe_command_for_simulation:=true", controller)

    def test_start_metadata_records_curved_identity_and_strict_json(self):
        orch = orchestrator("--curved-lumen-type", "circular_arc", "--scenario", NEAR_SAFETY_BOUNDARY_TARGET)
        baseline = orch._start_metadata(
            role="baseline",
            controller_label="zero_command",
            run_id="baseline_run",
            domain_id=123,
            recording_start_time=10.0,
            reference_epoch=11.0,
            evaluation_window_end=19.0,
            stability=stable_state(),
            audit=CommandAudit(events=[], publisher_counts={}),
            publisher_counts={},
            records=[],
        )
        candidate = orch._start_metadata(
            role="candidate",
            controller_label="mppi",
            run_id="candidate_run",
            domain_id=124,
            recording_start_time=20.0,
            reference_epoch=21.0,
            evaluation_window_end=29.0,
            stability=stable_state(),
            audit=CommandAudit(events=[], publisher_counts={}),
            publisher_counts={},
            records=[],
        )

        self.assertEqual("curved_lumen_navigation", baseline["reference_configuration"]["task"])
        self.assertEqual("fixed_target", baseline["reference_configuration"]["reference_mode"])
        self.assertEqual("fixed_target_window_epoch", baseline["reference_start_policy"])
        self.assertEqual("fixed_target_ready", baseline["reference_pre_epoch_behavior"])
        self.assertEqual(NEAR_SAFETY_BOUNDARY_TARGET, baseline["scenario_id"])
        self.assertEqual("curved_scenario_v1", baseline["scenario_policy_version"])
        self.assertEqual(orch.curved_scenario.scenario_fingerprint, baseline["scenario_fingerprint"])
        self.assertEqual(orch.curved_scenario.geometry_fingerprint, baseline["geometry_fingerprint"])
        self.assertFalse(baseline["override_used"])
        self.assertEqual(baseline["derived_target"], baseline["requested_target"])
        self.assertEqual(baseline["requested_target"], baseline["executed_target"])
        self.assertEqual(baseline["shared_environment_hash"], candidate["shared_environment_hash"])
        self.assertEqual(baseline["scenario_fingerprint"], candidate["scenario_fingerprint"])
        self.assertEqual(baseline["geometry_fingerprint"], candidate["geometry_fingerprint"])
        self.assertEqual(baseline["requested_target"], candidate["requested_target"])
        json.dumps(sanitize_for_json(baseline), allow_nan=False)

    def test_curved_shared_hash_semantics(self):
        centerline = orchestrator("--scenario", CENTERLINE_TARGET)
        lateral = orchestrator("--scenario", LATERAL_OFFSET_TARGET)
        s_curve = orchestrator("--curved-lumen-type", "s_curve", "--scenario", CENTERLINE_TARGET)
        seed_a = orchestrator("--seed", "1")
        seed_b = orchestrator("--seed", "2")
        root_a = orchestrator("--output-root", "/tmp/d2b1_a")
        root_b = orchestrator("--output-root", "/tmp/d2b1_b")
        override_values = [f"{value:.12f}" for value in lateral.curved_scenario.validated_target]
        override = orchestrator("--scenario", CENTERLINE_TARGET, "--target", *override_values)

        def shared_hash(orch):
            return build_shared_environment_hash(
                orch.project_config,
                task=orch.args.task,
                trajectory=orch.args.trajectory,
                duration=orch.args.duration,
                reference_lead_time=orch.settings.reference_lead_time,
                curved_scenario=orch.curved_scenario,
            )

        self.assertNotEqual(shared_hash(centerline), shared_hash(lateral))
        self.assertNotEqual(shared_hash(centerline), shared_hash(s_curve))
        self.assertNotEqual(shared_hash(centerline), shared_hash(override))
        self.assertEqual(shared_hash(seed_a), shared_hash(seed_b))
        self.assertEqual(shared_hash(root_a), shared_hash(root_b))
        self.assertEqual(shared_hash(centerline), shared_hash(orchestrator("--scenario", CENTERLINE_TARGET)))

    def test_legacy_commands_and_hashes_remain_unmoved(self):
        config = load_parameter_files(CONFIG_FILES)
        trajectory_hash = build_shared_environment_hash(config, trajectory="circle", duration=12.0, reference_lead_time=1.0)
        self.assertEqual(
            trajectory_hash,
            build_shared_environment_hash(config, trajectory="circle", duration=12.0, reference_lead_time=1.0),
        )
        cylinder_hash = build_shared_environment_hash(
            config,
            task="cylinder_navigation",
            trajectory="circle",
            duration=8.0,
            reference_lead_time=1.0,
        )
        self.assertEqual(
            cylinder_hash,
            build_shared_environment_hash(
                config,
                task="cylinder_navigation",
                trajectory="circle",
                duration=8.0,
                reference_lead_time=1.0,
            ),
        )
        cylinder_command = build_base_simulation_command(
            experiment_group="group",
            controller_label="mppi",
            baseline_dir=None,
            task="cylinder_navigation",
            target_position=[0.015, 0.005, 0.1],
            mppi_profile="cylinder_fast",
            random_seed=11,
            run_role="candidate",
        )
        self.assertIn("enable_cylindrical_lumen:=true", cylinder_command)
        self.assertNotIn("enable_curved_lumen:=true", cylinder_command)
        self.assertNotIn("reference_mode:=fixed_target", cylinder_command)
        trajectory_command = build_base_simulation_command(
            experiment_group="group",
            controller_label="mppi",
            baseline_dir=None,
        )
        self.assertNotIn("enable_cylindrical_lumen:=true", trajectory_command)
        self.assertNotIn("enable_curved_lumen:=true", trajectory_command)

    def test_scenario_resolution_failure_starts_no_process(self):
        calls = []
        original_resolver = run_module.resolve_curved_lumen_scenario
        original_start = run_module.ProcessManager.start
        run_module.resolve_curved_lumen_scenario = lambda *args, **kwargs: (_ for _ in ()).throw(ValueError("resolver failed"))
        run_module.ProcessManager.start = lambda *args, **kwargs: calls.append((args, kwargs))
        try:
            with self.assertRaisesRegex(ValueError, "resolver failed"):
                orchestrator()
        finally:
            run_module.resolve_curved_lumen_scenario = original_resolver
            run_module.ProcessManager.start = original_start
        self.assertEqual([], calls)

    def test_command_guard_primitives_remain_task_independent(self):
        audit = CommandAudit()
        audit.events.append(run_module.CommandEvent("/ctr/mppi_command", 1.0, "command_message_timestamp", 1.0, [0.0] * 6))
        audit.events.append(run_module.CommandEvent("/ctr/safe_command", 2.0, "command_message_timestamp", 2.0, [0.0, 0.0, 1.0e-3, 0.0, 0.0, 0.0]))
        self.assertEqual(1, audit.nonzero_count(1.0e-12))
        self.assertEqual({"/ctr/safe_command": 1}, run_module.unexpected_command_publishers({"/ctr/mppi_command": 0, "/ctr/safe_command": 1}))


if __name__ == "__main__":
    unittest.main()
