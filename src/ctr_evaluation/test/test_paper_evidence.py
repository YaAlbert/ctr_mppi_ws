import json
import hashlib
from pathlib import Path

import pytest

from ctr_bringup.parameter_validation import load_parameter_files
from ctr_evaluation.paper_evidence import (
    FORBIDDEN_PRESENTATION_TEXT,
    PAPER_FIGURES,
    PAPER_TABLES,
    TESTED_TARGET,
    aggregate,
    build_tactile_stress_table,
    ensure_standard_plot_names,
    forbidden_presentation_findings,
    matrix_specs,
    select_specs,
    target_source_block_reason,
    validate_matrix_contract,
)
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
        if row.target_source != "profile":
            assert row.target == TESTED_TARGET


def test_target_source_preflight_uses_final_validator_and_blocks_unreachable_profile_target():
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
        spec = next(
            row for row in select_specs("target_source")
            if row.target_source == "cli" and row.seed == seed
        )
        reason = target_source_block_reason(spec, effective)
        assert reason is not None
        assert "final target validator: target_unreachable" in reason


def test_matrix_contract_requires_every_cell_and_exact_target_equivalence():
    selected = select_specs("target_source")
    rows = [
        {
            "test_id": spec.test_id,
            "experiment": spec.experiment,
            "matrix_status": "completed",
            "completion_status": "completed",
            "accepted_target": "[0.021180966381970152,0.0,0.08471218663414842]",
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
