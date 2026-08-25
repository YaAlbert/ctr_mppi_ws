import json
import os
from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_model"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_mppi_controller"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_sim"))

from ctr_evaluation import development_simulation as development  # noqa: E402
from ctr_evaluation import run_evaluation  # noqa: E402


def passing_metrics():
    return {
        "comparison_valid": True,
        "readiness_succeeded": True,
        "readiness_time_seconds": 0.5,
        "command_message_count": 10,
        "final_tip_to_target_distance_m": 0.001,
        "trajectory_error_rmse_m": 0.002,
        "minimum_wall_clearance_m": 0.004,
        "collision_count": 0,
        "controller_update_frequency_hz": 19.8,
        "safety_events": 0,
        "tactile_invalid_events": 0,
        "cleanup_clean": True,
        "candidate_plots": [],
    }


def test_development_cli_requires_explicit_opt_in():
    assert development.main(["--skip-smoke", "--seeds", "11"]) == 1


def test_low_level_development_mode_rejects_non_simulation_and_non_curved_task():
    args = run_evaluation.parse_args(
        [
            "--development-simulation",
            "--experiment-group",
            "dev",
            "--duration",
            "1",
            "--runtime-mode",
            "hardware",
        ]
    )
    with pytest.raises(run_evaluation.OrchestrationError, match="runtime-mode simulation"):
        run_evaluation.validate_task_options(args)

    args.runtime_mode = "simulation"
    with pytest.raises(run_evaluation.OrchestrationError, match="curved_lumen_navigation"):
        run_evaluation.validate_task_options(args)


def test_development_output_root_is_narrow_and_user_owned(tmp_path):
    accepted = run_evaluation.validate_development_output_root(tmp_path / "results")
    assert accepted == (tmp_path / "results").resolve()
    with pytest.raises(run_evaluation.OrchestrationError):
        run_evaluation.validate_development_output_root(Path("/"))
    with pytest.raises(run_evaluation.OrchestrationError):
        run_evaluation.validate_development_output_root(Path.home())


def test_development_domain_lease_is_exclusive_and_released(monkeypatch, tmp_path):
    monkeypatch.setattr(run_evaluation, "DEVELOPMENT_DOMAIN_ROOT", tmp_path / "domains")
    monkeypatch.setattr(run_evaluation, "development_ros_domain_in_use", lambda _domain: False)
    monkeypatch.setattr(run_evaluation.uuid, "uuid4", lambda: type("U", (), {"int": 0})())
    first = run_evaluation.acquire_development_ros_domain()
    assert 100 <= first.domain_id <= 199
    second_descriptor = os.open(first.path, os.O_RDWR | os.O_CLOEXEC)
    try:
        with pytest.raises(BlockingIOError):
            run_evaluation.fcntl.flock(
                second_descriptor,
                run_evaluation.fcntl.LOCK_EX | run_evaluation.fcntl.LOCK_NB,
            )
    finally:
        os.close(second_descriptor)
        first.close()


def test_run_one_pair_selects_existing_curved_lumen_pipeline(monkeypatch, tmp_path):
    captured = {}

    class FakeOrchestrator:
        def __init__(self, args):
            captured["args"] = args

        def run_pair(self):
            candidate = tmp_path / "candidate"
            candidate.mkdir(exist_ok=True)
            (candidate / "orchestration.json").write_text(
                json.dumps(
                    {
                        "orchestration_success": True,
                        "requested_target": [0.015, 0.005, 0.100],
                        "validated_target": [0.015, 0.005, 0.100],
                    }
                ),
                encoding="utf-8",
            )
            return {
                "orchestration_success": True,
                "comparison_valid": True,
                "ros_domain_id": 155,
                "baseline_dir": str(tmp_path / "baseline"),
                "candidate_dir": str(tmp_path / "candidate"),
            }

    monkeypatch.setattr(development, "EvaluationOrchestrator", FakeOrchestrator)
    monkeypatch.setattr(development, "collect_metrics", lambda _result: passing_metrics())
    result = development.run_one_pair(root=tmp_path, seed=22, duration=25.0, smoke=False)
    assert result["status"] == "passed"
    assert captured["args"].development_simulation is True
    assert captured["args"].runtime_mode == "simulation"
    assert captured["args"].task == "curved_lumen_navigation"
    assert captured["args"].seed == 22
    assert captured["args"].mppi_profile == "cylinder_fast"


def test_development_cli_target_arguments_reach_existing_target_override(monkeypatch, tmp_path):
    captured = {}

    class FakeOrchestrator:
        def __init__(self, args):
            captured["args"] = args

        def run_pair(self):
            candidate = tmp_path / "candidate_cli"
            candidate.mkdir(exist_ok=True)
            (candidate / "orchestration.json").write_text(
                json.dumps(
                    {
                        "orchestration_success": True,
                        "requested_target": [0.015, 0.005, 0.100],
                        "validated_target": [0.015, 0.005, 0.100],
                    }
                ),
                encoding="utf-8",
            )
            return {
                "orchestration_success": True,
                "comparison_valid": True,
                "ros_domain_id": 155,
                "baseline_dir": str(tmp_path / "baseline_cli"),
                "candidate_dir": str(candidate),
            }

    monkeypatch.setattr(development, "EvaluationOrchestrator", FakeOrchestrator)
    monkeypatch.setattr(development, "collect_metrics", lambda _result: passing_metrics())
    result = development.run_one_pair(
        root=tmp_path,
        seed=11,
        duration=5.0,
        smoke=True,
        target_source="cli",
        target=(0.015, 0.005, 0.100),
    )
    assert captured["args"].target == [0.015, 0.005, 0.1]
    assert result["target_selection"] == {
        "target_source": "cli",
        "raw_input_point": [0.015, 0.005, 0.1],
        "raw_input_frame": "base_link",
        "validated_target": [0.015, 0.005, 0.1],
        "controller_target_frame": "base_link",
        "projection_distance": 0.0,
        "acceptance_status": "target_accepted",
        "orientation_used": False,
        "accepted_target_timestamp": result["target_selection"]["accepted_target_timestamp"],
        "reference_pose_count": 1,
        "seed": 11,
        "result_status": "passed",
    }


def test_cli_target_parser_requires_all_finite_coordinates():
    args = development.parse_args(
        [
            "--development-simulation",
            "--target-source",
            "cli",
            "--target-x",
            "0.015",
            "--target-y",
            "0.005",
            "--target-z",
            "0.100",
        ]
    )
    assert development.development_cli_target(args) == (0.015, 0.005, 0.1)
    args.target_z = None
    with pytest.raises(run_evaluation.OrchestrationError, match="requires"):
        development.development_cli_target(args)
    args.target_z = float("nan")
    with pytest.raises(run_evaluation.OrchestrationError, match="finite"):
        development.development_cli_target(args)


def test_rviz_cli_mode_builds_exact_interactive_launch_command():
    args = development.parse_args(
        [
            "--development-simulation",
            "--target-source",
            "rviz",
            "--seeds",
            "11",
            "--target-selection-timeout",
            "30",
        ]
    )
    development.development_cli_target(args)
    assert development.interactive_rviz_command(args) == [
        "ros2",
        "launch",
        "ctr_bringup",
        "slice_7g_development_visual.launch.py",
        "development_simulation:=true",
        "target_source:=rviz",
        "wait_for_target:=true",
        "target_selection_timeout:=30",
        "seed:=11",
    ]


def test_development_base_command_binds_explicit_mode():
    base = run_evaluation.build_base_simulation_command(
        experiment_group="dev",
        controller_label="zero_command",
        baseline_dir=None,
        slice_7g_profile=True,
        development_simulation=True,
    )
    assert "slice_7g_profile:=true" in base
    assert "development_simulation:=true" in base


def test_functional_result_rejects_missing_commands_and_unclean_children():
    metrics = passing_metrics()
    metrics["command_message_count"] = 0
    assert "no command" in development.validate_functional_result(
        {"orchestration_success": True}, metrics
    )
    metrics["command_message_count"] = 1
    metrics["cleanup_clean"] = False
    assert "cleanup" in development.validate_functional_result(
        {"orchestration_success": True}, metrics
    )


def test_result_report_is_explicitly_nonproduction(tmp_path):
    attempts = [
        {
            "kind": "example",
            "seed": 11,
            "status": "passed",
            "failure_reason": None,
            "ros_domain_id": 144,
            "candidate_dir": str(tmp_path / "seed11"),
            "metrics": passing_metrics(),
            "target_selection": {
                "target_source": "cli",
                "raw_input_point": [0.015, 0.005, 0.100],
                "raw_input_frame": "base_link",
                "validated_target": [0.015, 0.005, 0.100],
                "controller_target_frame": "base_link",
                "projection_distance": 0.0,
                "acceptance_status": "target_accepted",
                "orientation_used": False,
                "accepted_target_timestamp": "2026-08-25T00:00:00+00:00",
                "reference_pose_count": 1,
                "seed": 11,
                "result_status": "passed",
            },
        }
    ]
    (tmp_path / "seed11").mkdir()
    paths = development.write_development_results(tmp_path, attempts)
    report = Path(paths["result_report"]).read_text(encoding="utf-8")
    payload = json.loads(Path(paths["result_json"]).read_text(encoding="utf-8"))
    assert "not production promotion evidence" in report
    assert payload["production_promotion_evidence"] is False
    assert payload["production_attempts_consumed"] == 0
    assert payload["attempts"][0]["target_selection"]["target_source"] == "cli"
    assert payload["attempts"][0]["target_selection"]["reference_pose_count"] == 1
    assert "source=`cli`" in report
    assert Path(paths["comparison_plot"]).is_file()
