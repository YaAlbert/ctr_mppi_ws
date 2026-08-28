import csv
import json
import hashlib
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pytest

from ctr_bringup.parameter_validation import load_parameter_files
from ctr_evaluation.paper_evidence import (
    FORMAL_WINDOW_BOUNDARY_CONVENTION,
    FORMAL_WINDOW_EVIDENCE_VALIDATOR_SCHEMA,
    FORBIDDEN_PRESENTATION_TEXT,
    PAPER_FIGURES,
    PAPER_TABLES,
    REQUIRED_RUN_ARTIFACTS,
    TESTED_TARGET,
    aggregate,
    build_tactile_stress_table,
    ensure_standard_plot_names,
    format_formal_window_evidence_validation,
    forbidden_presentation_findings,
    matrix_specs,
    select_specs,
    target_source_block_reason,
    validate_artifacts,
    validate_formal_window_evidence,
    validate_matrix_contract,
)
from ctr_evaluation import paper_evidence
from ctr_evaluation.run_evaluation import default_config_paths


def test_final_evidence_matrix_is_closed_and_complete():
    specs = matrix_specs()
    assert len(specs) == 42
    assert len({spec.test_id for spec in specs}) == len(specs)
    assert {spec.experiment for spec in specs} == {
        "reference",
        "repeatability",
        "target_source",
        "target_difficulty",
        "lumen_geometry",
        "controller_configuration",
    }
    assert {spec.seed for spec in select_specs("repeatability")} == {11, 22, 33, 44, 55}


def test_target_source_cells_use_one_controller_target():
    rows = select_specs("target_source")
    assert len(rows) == 9
    assert {row.target_source for row in rows} == {"profile", "cli", "rviz"}
    for row in rows:
        assert row.target == TESTED_TARGET


def test_target_source_preflight_uses_final_validator_for_all_three_sources():
    config = load_parameter_files(default_config_paths())
    from ctr_bringup.slice_7g_profile import apply_slice_7g_development_simulation_profile
    from ctr_mppi_controller.lumen_factory import config_with_lumen_overrides

    for seed in (11, 22, 33):
        effective = config_with_lumen_overrides(
            config,
            enable_cylindrical_lumen=False,
            enable_curved_lumen=True,
            curved_lumen_type="circular_arc",
            cylinder_profile="cylinder_fast",
            random_seed=seed,
        )
        effective = apply_slice_7g_development_simulation_profile(effective, enabled=True)
        for source in ("profile", "cli", "rviz"):
            spec = next(
                row for row in select_specs("target_source")
                if row.target_source == source and row.seed == seed
            )
            assert target_source_block_reason(spec, effective) is None


def test_matrix_contract_requires_every_cell_and_exact_target_equivalence():
    selected = select_specs("target_source")
    rows = [
        {
            "test_id": spec.test_id,
            "experiment": spec.experiment,
            "matrix_status": "completed",
            "completion_status": "completed",
            "accepted_target": json.dumps(TESTED_TARGET, separators=(",", ":")),
        }
        for spec in selected
    ]
    assert validate_matrix_contract(rows, selected) == []
    rows[0]["accepted_target"] = "[0.0,0.0,0.0]"
    assert "differs" in validate_matrix_contract(rows, selected)[0]
    assert "missing" in validate_matrix_contract(rows[1:], selected)[0]


def test_controller_tradeoff_is_configuration_not_software_version():
    rows = select_specs("controller")
    assert {row.controller_profile for row in rows} == {
        "paper_economy", "cylinder_fast", "paper_extended",
    }
    assert {row.seed for row in rows} == {11, 22, 33}


def test_straight_geometry_uses_fixed_profile_target_without_curved_selector():
    straight = [spec for spec in select_specs("geometry") if spec.geometry == "straight"]
    assert len(straight) == 3
    assert {spec.target_source for spec in straight} == {"profile"}
    assert all(spec.target == (0.0192, 0.0, 0.084) for spec in straight)


def test_tactile_stress_uses_real_processor_threshold_states():
    config = load_parameter_files(default_config_paths())
    rows = build_tactile_stress_table(config)
    state_set = {
        (row.get("contact"), row.get("warning"), row.get("stop"))
        for row in rows if "contact" in row
    }
    assert (False, False, False) in state_set
    assert (True, False, False) in state_set
    assert (True, True, False) in state_set
    assert (True, True, True) in state_set
    assert all(row.get("evidence_class") == "diagnostic_stress_test" for row in rows)


def test_empty_aggregate_emits_complete_export_with_neutral_titles(tmp_path):
    root = tmp_path / "final_system_test"
    root.mkdir()
    result = aggregate(root, [], Path("unused"))
    assert result["completed"] == 0
    assert result["artifact_validation_failures"] == 0
    for name in PAPER_FIGURES:
        assert (root / "paper_figures" / name).stat().st_size > 0
        assert (root / "overleaf_upload" / name).stat().st_size > 0
    for name in PAPER_TABLES:
        assert (root / "paper_tables" / name).stat().st_size > 0
        assert (root / "overleaf_upload" / name).stat().st_size > 0
    assert forbidden_presentation_findings(root) == []
    visible = (root / "paper_results.md").read_text(encoding="utf-8").lower()
    assert not any(phrase in visible for phrase in FORBIDDEN_PRESENTATION_TEXT)
    assert "non-real-time ubuntu host" in visible
    assert "0.20 s" in visible
    assert "production/hardware freshness contract remains 0.10 s" in visible


def test_failed_rows_are_neutralized_only_at_publication_boundary(tmp_path):
    root = tmp_path / "final_system_test"
    root.mkdir()
    raw = {
        "test_id": "E1-reference",
        "experiment": "reference",
        "matrix_status": "failed",
        "failure_reason": "OrchestrationError: Slice 7G tactile/safety status became stale",
    }
    aggregate(root, [raw], Path("unused"))
    assert raw["failure_reason"].startswith("OrchestrationError: Slice 7G")
    assert "Slice 7G" not in (root / "artifact_validation.md").read_text(encoding="utf-8")
    assert "Slice 7G" not in (root / "paper_tables" / "reference_run.csv").read_text(encoding="utf-8")
    assert forbidden_presentation_findings(root) == []
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    assert "manifest.json" not in {member["path"] for member in manifest["members"]}
    for member in manifest["members"]:
        path = root / member["path"]
        assert path.stat().st_size == member["size"]
        assert hashlib.sha256(path.read_bytes()).hexdigest() == member["sha256"]


def test_straight_lumen_emits_common_centerline_evidence_and_plot_names(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    (run / "aligned_samples.csv").write_text(
        "timestamp,tip_x,tip_y,tip_z\n0,0.01,0,0.02\n1,0.02,0,0.03\n",
        encoding="utf-8",
    )
    (run / "cylinder_navigation.csv").write_text(
        "timestamp,minimum_backbone_clearance,collision,safety_margin_violation\n"
        "0,0.0185,False,False\n1,0.0085,False,False\n",
        encoding="utf-8",
    )
    (run / "metadata.yaml").write_text(
        "configuration:\n  cylindrical_lumen:\n    axis_origin: [0.0, 0.0, 0.0]\n"
        "    axis_direction: [0.0, 0.0, 1.0]\n    radius: 0.03\n",
        encoding="utf-8",
    )
    for name in ("wall_clearance.png", "cylinder_backbone_target_3d.png"):
        (run / name).write_bytes(b"png")
    ensure_standard_plot_names(run)
    assert (run / "lumen_evaluation.csv").stat().st_size > 0
    assert (run / "centerline_tracking_error.png").stat().st_size > 0
    assert (run / "curved_wall_clearance.png").read_bytes() == b"png"
    assert (run / "curved_lumen_trajectory_3d.png").read_bytes() == b"png"


@pytest.mark.parametrize("name", PAPER_FIGURES)
def test_paper_figure_filenames_are_exact(name):
    assert name.endswith(".png")
    assert "/" not in name


def _safety_row(
    stamp: float,
    sequence: int,
    *,
    age: float = 0.01,
    valid: bool = True,
    fault: bool = False,
    emergency_stop: bool = False,
    reason: str = "eligible_no_contact",
    out_of_order: bool = False,
) -> dict[str, object]:
    return {
        "timestamp_s": stamp,
        "event_type": "safety",
        "safety_source_sequence": sequence,
        "safety_source_stamp_s": stamp,
        "safety_receipt_monotonic_s": stamp + age,
        "safety_receipt_gap_s": 0.02,
        "safety_source_stamp_gap_s": 0.02,
        "safety_sequence_gap": 1,
        "safety_duplicate_sequence": False,
        "safety_out_of_order_sequence": out_of_order,
        "safety_queued_age_s": age,
        "safety_valid": valid,
        "safety_fault": fault,
        "safety_emergency_stop": emergency_stop,
        "safety_reason": reason,
    }


def _tactile_row(
    stamp: float,
    sequence: int,
    *,
    age: float = 0.01,
    overwrites: int = 0,
) -> dict[str, object]:
    return {
        "timestamp_s": stamp,
        "event_type": "tactile",
        "received_timestamp_s": stamp + age,
        "data_age_s": age,
        "source_sequence": sequence,
        "source_mailbox_overwrites": overwrites,
        "evaluator_receipt_gap_s": age,
    }


def _write_evidence_fixture(
    root: Path,
    *,
    tactile_rows: list[dict[str, object]] | None = None,
    safety_rows: list[dict[str, object]] | None = None,
) -> tuple[dict[str, object], dict[str, object]]:
    root.mkdir()
    metadata: dict[str, object] = {
        "evaluation_window_start_time_s": 100.0,
        "evaluation_window_end_time_s": 125.0,
        "evaluation_window_duration_s": 25.0,
        "recording_start_time_s": 99.0,
        "recording_stop_time_s": 126.0,
        "configuration": {"simulator_paper_evaluation_profile": True},
        "metadata_override": {
            "evaluation_window_start_time_s": 100.0,
            "evaluation_window_end_time_s": 125.0,
            "evaluation_window_duration_s": 25.0,
            "simulator_paper_evaluation_profile": True,
        },
    }
    summary: dict[str, object] = {
        "run_status": {
            "status": "completed",
            "interrupted": False,
            "completed_evaluation_window": True,
        },
        "navigation": {
            "run_valid": True,
            "physical_safety_pass": True,
            "safety_margin_pass": True,
            "completed_evaluation_window": True,
        },
        "slice_7g_safety": {"fault_count": 0},
        "paper_metrics": {"requested_runtime_s": 25.0},
    }
    (root / "metadata.yaml").write_text(
        __import__("yaml").safe_dump(metadata, sort_keys=True), encoding="utf-8"
    )
    (root / "summary.json").write_text(
        json.dumps(summary, sort_keys=True), encoding="utf-8"
    )
    if tactile_rows is None:
        tactile_rows = [
            _tactile_row(99.0, 125, age=0.331, overwrites=0),
            _tactile_row(99.32, 157, overwrites=31),
            _tactile_row(100.0, 158),
            _tactile_row(125.0, 159),
        ]
    if safety_rows is None:
        safety_rows = [
            _safety_row(100.0, 200),
            _safety_row(112.0, 202),
            _safety_row(125.0, 204),
        ]
    rows = [*tactile_rows, *safety_rows]
    fields = sorted({key for row in rows for key in row})
    with (root / "tactile_safety.csv").open(
        "w", encoding="utf-8", newline=""
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return metadata, summary


def test_formal_validator_keeps_cell3_preroll_ros_anomaly_diagnostic(tmp_path):
    run = tmp_path / "run"
    _write_evidence_fixture(run)

    result = validate_formal_window_evidence(run)

    assert result["schema_version"] == FORMAL_WINDOW_EVIDENCE_VALIDATOR_SCHEMA
    assert result["eligible"] is True
    assert result["failures"] == []
    assert (
        result["formal_window"]["boundary_convention"]
        == FORMAL_WINDOW_BOUNDARY_CONVENTION
    )
    full = result["ros_tactile_delivery"]["full_recording"]
    formal = result["ros_tactile_delivery"]["formal_window"]
    assert full["sequence"]["forward_gaps"][0] == {
        "previous_sequence": 125,
        "current_sequence": 157,
        "missing_sequence_count": 31,
        "previous_source_stamp_s": 99.0,
        "current_source_stamp_s": 99.32,
    }
    assert full["source_mailbox_overwrites"]["total"] == 31.0
    assert full["ros_tactile_delivery_age_s"]["max"] == 0.331
    assert formal["sequence"]["forward_gap_count"] == 0
    assert formal["source_mailbox_overwrites"]["total"] == 0.0
    assert formal["ros_tactile_delivery_age_s"]["max"] == 0.01


def test_ros_anomaly_inside_formal_window_remains_non_authoritative(tmp_path):
    run = tmp_path / "run"
    _write_evidence_fixture(
        run,
        tactile_rows=[
            _tactile_row(100.0, 125, age=0.331),
            _tactile_row(100.32, 157, overwrites=31),
            _tactile_row(125.0, 158),
        ],
    )

    result = validate_formal_window_evidence(run)

    assert result["eligible"] is True
    formal = result["ros_tactile_delivery"]["formal_window"]
    assert formal["sequence"]["missing_sequence_count"] == 31
    assert formal["source_mailbox_overwrites"]["total"] == 31.0
    assert formal["ros_tactile_delivery_age_s"]["max"] == 0.331
    assert result["authoritative_safety"]["formal_window"][
        "safety_direct_age_s"
    ]["max"] == 0.01


@pytest.mark.parametrize(
    ("age", "eligible"),
    ((0.199999999, True), (0.20, False), (0.200000001, False)),
)
def test_formal_safety_freshness_boundary_is_strict(tmp_path, age, eligible):
    run = tmp_path / "run"
    _write_evidence_fixture(
        run,
        safety_rows=[
            _safety_row(100.0, 1, age=age),
            _safety_row(125.0, 2, age=age),
        ],
    )

    result = validate_formal_window_evidence(run)

    assert result["eligible"] is eligible
    assert any("safety_direct_age_s" in failure for failure in result["failures"]) is (
        not eligible
    )


@pytest.mark.parametrize(
    "changes",
    (
        {"valid": False},
        {"fault": True},
        {"emergency_stop": True},
        {"reason": "tactile_stale"},
        {"reason": "state_stale"},
        {"reason": "physical_evidence_integrity_invalid"},
        {"reason": "physical_evidence_authentication_invalid"},
        {"reason": "physical_evidence_producer_disconnected"},
        {"reason": "evaluator_service_unavailable"},
        {"reason": "physical_evidence_torn_read"},
        {"reason": "physical_evidence_future_dated"},
    ),
)
def test_formal_authoritative_safety_fail_closed_conditions(tmp_path, changes):
    run = tmp_path / "run"
    _write_evidence_fixture(
        run,
        safety_rows=[
            _safety_row(100.0, 1, **changes),
            _safety_row(125.0, 2),
        ],
    )

    result = validate_formal_window_evidence(run)

    assert result["eligible"] is False
    assert result["failures"]


def test_latest_sample_forward_sequence_gaps_are_diagnostic(tmp_path):
    run = tmp_path / "run"
    _write_evidence_fixture(
        run,
        safety_rows=[
            _safety_row(100.0, 10),
            _safety_row(112.0, 42),
            _safety_row(125.0, 43),
        ],
    )

    result = validate_formal_window_evidence(run)

    assert result["eligible"] is True
    sequence = result["authoritative_safety"]["formal_window"]["sequence"]
    assert sequence["forward_gap_count"] == 1
    assert sequence["missing_sequence_count"] == 31


@pytest.mark.parametrize(
    "rows",
    (
        [_safety_row(100.0, 10), _safety_row(125.0, 9)],
        [_safety_row(100.0, 10), _safety_row(125.0, 10)],
    ),
)
def test_authoritative_rollback_or_same_sequence_mutation_fails(tmp_path, rows):
    run = tmp_path / "run"
    _write_evidence_fixture(run, safety_rows=rows)

    result = validate_formal_window_evidence(run)

    assert result["eligible"] is False
    assert any(
        "rolls back" in failure or "same sequence" in failure
        for failure in result["failures"]
    )


def test_pre_and_post_window_anomalies_remain_full_recording_only(tmp_path):
    run = tmp_path / "run"
    _write_evidence_fixture(
        run,
        tactile_rows=[
            _tactile_row(99.0, 1, age=0.5, overwrites=7),
            _tactile_row(100.0, 9, age=0.01),
            _tactile_row(125.0, 10, age=0.01),
            _tactile_row(126.0, 20, age=0.6, overwrites=9),
        ],
        safety_rows=[
            _safety_row(
                99.0, 1, age=0.5, valid=False, fault=True, reason="state_stale"
            ),
            _safety_row(100.0, 10),
            _safety_row(125.0, 12),
            _safety_row(
                126.0, 11, age=0.6, valid=False, fault=True, reason="tactile_stale"
            ),
        ],
    )

    result = validate_formal_window_evidence(run)

    assert result["eligible"] is True
    safety = result["authoritative_safety"]
    ros = result["ros_tactile_delivery"]
    assert safety["full_recording"]["safety_direct_age_s"]["max"] == 0.6
    assert safety["formal_window"]["safety_direct_age_s"]["max"] == 0.01
    assert safety["full_recording"]["sequence"]["rollback_count"] == 1
    assert safety["formal_window"]["sequence"]["rollback_count"] == 0
    assert ros["full_recording"]["ros_tactile_delivery_age_s"]["max"] == 0.6
    assert ros["formal_window"]["ros_tactile_delivery_age_s"]["max"] == 0.01
    assert ros["full_recording"]["source_mailbox_overwrites"]["total"] == 16.0
    assert ros["formal_window"]["source_mailbox_overwrites"]["total"] == 0.0


@pytest.mark.parametrize(
    "mutation",
    ("missing_start", "reversed", "inconsistent", "empty_safety"),
)
def test_missing_inconsistent_or_empty_formal_window_fails(tmp_path, mutation):
    run = tmp_path / "run"
    metadata, _summary = _write_evidence_fixture(run)
    if mutation == "missing_start":
        metadata.pop("evaluation_window_start_time_s")
    elif mutation == "reversed":
        metadata["evaluation_window_end_time_s"] = 99.0
    elif mutation == "inconsistent":
        metadata["metadata_override"]["evaluation_window_end_time_s"] = 124.0
    else:
        with (run / "tactile_safety.csv").open(
            "w", encoding="utf-8", newline=""
        ) as handle:
            writer = csv.DictWriter(
                handle, fieldnames=sorted(_tactile_row(99.0, 1))
            )
            writer.writeheader()
            writer.writerow(_tactile_row(99.0, 1))
    if mutation != "empty_safety":
        (run / "metadata.yaml").write_text(
            __import__("yaml").safe_dump(metadata, sort_keys=True), encoding="utf-8"
        )

    result = validate_formal_window_evidence(run)

    assert result["eligible"] is False
    assert result["failures"]


def test_validator_labels_and_structured_output_are_deterministic(tmp_path):
    run = tmp_path / "run"
    _write_evidence_fixture(run)

    first = validate_formal_window_evidence(run)
    second = validate_formal_window_evidence(run)
    first_bytes = json.dumps(
        first, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode()
    second_bytes = json.dumps(
        second, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode()
    markdown = format_formal_window_evidence_validation(first)

    assert first_bytes == second_bytes
    assert b"ros_tactile_delivery_age_s" in first_bytes
    assert b"safety_direct_age_s" in first_bytes
    assert "ros_tactile_delivery_age_s" in markdown
    assert "safety_direct_age_s" in markdown
    assert "direct tactile age" not in markdown.lower()


def _complete_artifact_fixture(run: Path) -> None:
    _write_evidence_fixture(run)
    for name in REQUIRED_RUN_ARTIFACTS:
        path = run / name
        if path.exists():
            continue
        if name.endswith(".png"):
            plt.imsave(path, np.zeros((1, 1, 3), dtype=np.float64))
        else:
            path.write_text("field\nvalue\n", encoding="utf-8")


def test_canonical_artifact_validator_keeps_other_gates_and_decodes_pngs(
    tmp_path, monkeypatch
):
    run = tmp_path / "run"
    _complete_artifact_fixture(run)
    row = {
        "test_id": "synthetic",
        "matrix_status": "completed",
        "completion_status": "completed",
        "geometry": "straight",
        "candidate_dir": str(run),
    }
    monkeypatch.setattr(
        paper_evidence, "independently_recompute_metrics", lambda *_args: ({}, {})
    )

    markdown, failures = validate_artifacts(tmp_path, [row])
    assert failures == 0
    assert "formal_window_evidence" in markdown
    assert "invalid_pngs=[]" in markdown

    (run / "tracking_error.png").write_bytes(b"not-a-png")
    markdown, failures = validate_artifacts(tmp_path, [row])
    assert failures == 1
    assert "tracking_error.png" in markdown

    plt.imsave(run / "tracking_error.png", np.zeros((1, 1, 3)))
    (run / "state.csv").unlink()
    markdown, failures = validate_artifacts(tmp_path, [row])
    assert failures == 1
    assert "state.csv" in markdown


def test_canonical_artifact_metric_reconciliation_is_unchanged(tmp_path, monkeypatch):
    run = tmp_path / "run"
    _complete_artifact_fixture(run)
    row = {
        "test_id": "synthetic",
        "matrix_status": "completed",
        "completion_status": "completed",
        "geometry": "straight",
        "candidate_dir": str(run),
    }
    monkeypatch.setattr(
        paper_evidence,
        "independently_recompute_metrics",
        lambda *_args: ({"metric": 1.0}, {"metric": 1.0e-3}),
    )

    markdown, failures = validate_artifacts(tmp_path, [row])

    assert failures == 1
    assert "'metric': 0.001" in markdown
