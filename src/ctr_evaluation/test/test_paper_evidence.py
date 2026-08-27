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


def test_controller_tradeoff_is_configuration_not_software_version():
    rows = select_specs("controller")
    assert {row.controller_profile for row in rows} == {
        "paper_economy", "cylinder_fast", "paper_extended",
    }
    assert {row.seed for row in rows} == {11, 22, 33}


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
    for name in PAPER_FIGURES:
        assert (root / "paper_figures" / name).stat().st_size > 0
        assert (root / "overleaf_upload" / name).stat().st_size > 0
    for name in PAPER_TABLES:
        assert (root / "paper_tables" / name).stat().st_size > 0
        assert (root / "overleaf_upload" / name).stat().st_size > 0
    assert forbidden_presentation_findings(root) == []
    visible = (root / "paper_results.md").read_text(encoding="utf-8").lower()
    assert not any(phrase in visible for phrase in FORBIDDEN_PRESENTATION_TEXT)


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
