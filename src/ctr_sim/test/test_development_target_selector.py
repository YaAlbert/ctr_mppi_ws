import json
import sys
from pathlib import Path

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
for package in ("ctr_bringup", "ctr_model", "ctr_mppi_controller", "ctr_sim"):
    sys.path.insert(0, str(REPO_ROOT / "src" / package))

from ctr_bringup.parameter_validation import load_parameter_files  # noqa: E402
from ctr_bringup.slice_7g_profile import apply_slice_7g_development_simulation_profile  # noqa: E402
from ctr_model.approximate_model import ApproximateCTRModel  # noqa: E402
from ctr_mppi_controller.lumen_factory import (  # noqa: E402
    config_with_lumen_overrides,
    lumen_geometry_from_config,
)
from ctr_sim.nodes import development_target_selector_node as selector_module  # noqa: E402
from ctr_sim.nodes.development_target_selector_node import (  # noqa: E402
    TargetSelectionResult,
    build_sampled_reachability_cloud,
    sampled_reachability_predicate,
    select_development_target,
    target_update_status,
    transform_candidate_to_controller_frame,
    validate_candidate_timestamp,
)


CONFIG_FILES = [
    REPO_ROOT / "config" / name
    for name in (
        "robot_params.yaml",
        "model_params.yaml",
        "mppi_params.yaml",
        "simulation_params.yaml",
        "evaluation_params.yaml",
        "safety_params.yaml",
        "tactile_params.yaml",
        "hardware_params.yaml",
        "slice_7g_runtime_params.yaml",
    )
]


def geometry():
    config = config_with_lumen_overrides(
        load_parameter_files(CONFIG_FILES),
        enable_cylindrical_lumen=False,
        enable_curved_lumen=True,
        curved_lumen_type="circular_arc",
    )
    return lumen_geometry_from_config(config)


def select(point, *, source="rviz", frame="base_link", reachable=lambda _target: True):
    return select_development_target(
        point,
        input_frame=frame,
        target_source=source,
        geometry=geometry(),
        controller_frame="base_link",
        world_frame="world",
        projection_limit=0.035,
        reachable=reachable,
        accepted_target_timestamp=12.5,
        seed=11,
    )


def test_valid_cli_target_is_accepted_without_projection_and_records_exact_reference():
    result = select([0.015, 0.005, 0.100], source="cli")
    assert result.accepted
    assert result.status == "target_accepted"
    assert result.projected is False
    assert result.projection_distance == 0.0
    assert result.validated_target == (0.015, 0.005, 0.100)
    assert result.reference_pose_count == 1
    assert result.orientation_used is False
    assert json.loads(result.to_json())["validated_target"] == [0.015, 0.005, 0.1]


def test_world_candidate_uses_established_identity_transform_to_base_link():
    point = [0.015, 0.005, 0.100]
    transformed = transform_candidate_to_controller_frame(
        point,
        input_frame="world",
        controller_frame="base_link",
        world_frame="world",
    )
    assert np.array_equal(point, transformed)
    assert select(point, frame="world").accepted


@pytest.mark.parametrize("point", ([float("nan"), 0.0, 0.08], [0.0, float("inf"), 0.08]))
def test_nonfinite_target_is_rejected(point):
    with pytest.raises(ValueError, match="finite"):
        select(point)


def test_unknown_frame_is_rejected():
    with pytest.raises(ValueError, match="target_frame_invalid"):
        select([0.015, 0.005, 0.100], frame="map")


def test_nearby_rviz_point_outside_wall_projects_to_centerline():
    center = geometry().centerline_points[40]
    result = select(center + np.array([0.0, 0.031, 0.0]))
    assert result.accepted
    assert result.projected
    assert result.projection_distance == pytest.approx(0.031, abs=1.0e-12)
    assert np.allclose(result.validated_target, center)


def test_cli_point_outside_wall_is_rejected_without_projection():
    center = geometry().centerline_points[40]
    result = select(center + np.array([0.0, 0.031, 0.0]), source="cli")
    assert not result.accepted
    assert result.status == "target_outside_lumen"


def test_excessive_projection_distance_is_rejected():
    center = geometry().centerline_points[40]
    result = select(center + np.array([0.0, 0.040, 0.0]))
    assert not result.accepted
    assert result.status == "target_projection_too_far"


def test_cli_clearance_violation_is_rejected_instead_of_silently_projected():
    center = geometry().centerline_points[40]
    result = select(center + np.array([0.0, 0.027, 0.0]), source="cli")
    assert not result.accepted
    assert result.status == "target_clearance_invalid"


def test_rviz_surface_candidate_projects_to_real_centerline_and_reports_distance():
    lumen = geometry()
    center = lumen.centerline_points[40]
    raw = center + np.array([0.0, float(lumen.lumen_radius), 0.0])
    result = select(raw)
    assert result.accepted
    assert result.projected
    assert result.projection_distance == pytest.approx(float(lumen.lumen_radius), abs=1.0e-12)
    assert np.allclose(result.validated_target, center)


def test_geometrically_valid_but_sampled_unreachable_target_is_rejected():
    result = select([0.015, 0.005, 0.100], reachable=lambda _target: False)
    assert not result.accepted
    assert result.status == "target_unreachable"


def test_real_approximate_model_reachability_path_accepts_tested_cli_coordinate():
    config = config_with_lumen_overrides(
        load_parameter_files(CONFIG_FILES),
        enable_cylindrical_lumen=False,
        enable_curved_lumen=True,
        curved_lumen_type="circular_arc",
        cylinder_profile="cylinder_fast",
        random_seed=11,
    )
    config = apply_slice_7g_development_simulation_profile(config, enabled=True)
    predicate = sampled_reachability_predicate(
        build_sampled_reachability_cloud(ApproximateCTRModel(config), config),
        config["goal"]["tolerance"],
    )
    result = select_development_target(
        [0.0166457424, 0.00397477634, 0.102231139],
        input_frame="base_link",
        target_source="cli",
        geometry=lumen_geometry_from_config(config),
        controller_frame="base_link",
        world_frame="world",
        projection_limit=0.035,
        reachable=predicate,
        accepted_target_timestamp=12.5,
        seed=11,
    )
    assert result.accepted


def test_timestamp_policy_accepts_zero_and_rejects_stale_or_future_values():
    validate_candidate_timestamp(stamp_seconds=0.0, now_seconds=10.0, maximum_age=5.0, future_tolerance=0.5)
    validate_candidate_timestamp(stamp_seconds=9.0, now_seconds=10.0, maximum_age=5.0, future_tolerance=0.5)
    with pytest.raises(ValueError, match="stale"):
        validate_candidate_timestamp(stamp_seconds=4.0, now_seconds=10.0, maximum_age=5.0, future_tolerance=0.5)
    with pytest.raises(ValueError, match="future"):
        validate_candidate_timestamp(stamp_seconds=11.0, now_seconds=10.0, maximum_age=5.0, future_tolerance=0.5)


def test_record_is_frozen_and_contains_reproduction_fields():
    result = select([0.015, 0.005, 0.100], source="cli")
    with pytest.raises(Exception):
        result.status = "changed"
    payload = json.loads(result.to_json())
    assert set(payload) == {
        "accepted",
        "accepted_target_timestamp",
        "controller_target_frame",
        "orientation_used",
        "projected",
        "projection_distance",
        "raw_input_frame",
        "raw_input_point",
        "reference_pose_count",
        "seed",
        "status",
        "target_source",
        "validated_target",
    }


def test_target_update_policy_accepts_only_before_motion_starts():
    assert target_update_status(False) == "target_candidate_received"
    assert target_update_status(True) == "target_update_rejected_motion_started"
    with pytest.raises(ValueError):
        target_update_status(1)


def test_console_entrypoint_uses_shared_owned_node_shutdown_contract(monkeypatch):
    captured = {}

    def fake_runner(rclpy_module, node_factory, *, args=None):
        captured.update(rclpy_module=rclpy_module, node_factory=node_factory, args=args)

    monkeypatch.setattr(selector_module, "run_node_until_shutdown", fake_runner)
    selector_module.main(["--ros-args"])
    assert captured == {
        "rclpy_module": selector_module.rclpy,
        "node_factory": selector_module.DevelopmentTargetSelectorNode,
        "args": ["--ros-args"],
    }
