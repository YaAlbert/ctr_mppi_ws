from copy import deepcopy
from pathlib import Path
import sys

import pytest
import yaml


REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "ctr_bringup"))

from ctr_bringup.parameter_validation import load_parameter_files, validate_project_config  # noqa: E402
from ctr_bringup.slice_7g_profile import (  # noqa: E402
    Slice7GProfileError,
    apply_slice_7g_development_simulation_profile,
    apply_slice_7g_simulation_profile,
    validate_slice_7g_effective_config,
)


CONFIGS = [
    "robot_params.yaml", "model_params.yaml", "mppi_params.yaml", "simulation_params.yaml",
    "evaluation_params.yaml", "safety_params.yaml", "tactile_params.yaml", "hardware_params.yaml",
    "slice_7g_runtime_params.yaml",
]


def _config():
    return load_parameter_files([REPO / "config" / name for name in CONFIGS])


def test_profile_enables_coordinated_simulation_contract_without_mutating_source():
    source = _config()
    original = deepcopy(source)
    result = apply_slice_7g_simulation_profile(source, enabled=True)
    assert source == original
    assert result["runtime"] == {"mode": "simulation", "slice_7g_profile": True}
    assert result["curved_lumen"]["enabled"] is True
    assert result["curved_lumen"]["type"] == "circular_arc"
    assert result["tactile"]["enabled"] is True
    assert result["safety"]["enabled"] is True
    assert result["safety"]["tactile_enabled"] is True
    assert result["mppi"]["tactile"]["enabled"] is True
    assert result["mppi"]["weights"]["force"] == 10.0
    assert {name: result["mppi"]["weights"][name] for name in ("shape", "obstacle", "stability")} == {
        "shape": 0.0, "obstacle": 0.0, "stability": 0.0,
    }
    validate_slice_7g_effective_config(result)
    assert validate_project_config(result) == []


@pytest.mark.parametrize("name,todo", [
    ("obstacle", "TODO-COST-001"),
    ("shape", "TODO-COST-005"),
    ("stability", "TODO-COST-006"),
])
def test_unfinished_cost_is_fail_closed_if_enabled_or_weighted(name, todo):
    config = _config()
    config["slice_7g_runtime"]["unfinished_costs"][name]["enabled"] = True
    with pytest.raises(Slice7GProfileError) as raised:
        apply_slice_7g_simulation_profile(config, enabled=True)
    assert raised.value.code == "unfinished_cost_reachable"
    assert todo in str(raised.value)


def test_contradictory_effective_advanced_weight_is_rejected():
    config = apply_slice_7g_simulation_profile(_config(), enabled=True)
    config["mppi"]["weights"]["shape"] = 1.0
    with pytest.raises(Slice7GProfileError) as raised:
        validate_slice_7g_effective_config(config)
    assert raised.value.code == "unfinished_cost_reachable"


def test_disabled_profile_preserves_legacy_configuration():
    config = _config()
    assert apply_slice_7g_simulation_profile(config, enabled=False) == config


def test_profile_rejects_physical_hardware_runtime_instead_of_overriding_it():
    config = _config()
    config.setdefault("runtime", {})["mode"] = "hardware"
    with pytest.raises(Slice7GProfileError) as raised:
        apply_slice_7g_simulation_profile(config, enabled=True)
    assert raised.value.code == "profile_runtime_mode"


def test_launch_profile_starts_supervisor_and_forbids_controller_bypass():
    text = (REPO / "src/ctr_bringup/launch/simulation.launch.py").read_text(encoding="utf-8")
    assert 'executable="safety_supervisor_node"' in text
    assert 'condition=IfCondition(start_safety_supervisor)' in text
    assert '"slice_7g_profile"' in text
    assert "start_safety_supervisor=true cannot be combined" in text
    assert "start_manual_command_publisher=true" in text
    assert "slice_7g_profile=true requires runtime_mode=simulation" in text


def test_slice_7g_topic_graph_has_one_supervised_command_path():
    controller = (
        REPO / "src/ctr_mppi_controller/ctr_mppi_controller/nodes/mppi_controller_node.py"
    ).read_text(encoding="utf-8")
    safety = (
        REPO / "src/ctr_safety/ctr_safety/nodes/safety_supervisor_node.py"
    ).read_text(encoding="utf-8")
    simulator = (
        REPO / "src/ctr_sim/ctr_sim/nodes/simulator_node.py"
    ).read_text(encoding="utf-8")
    evaluator = (
        REPO / "src/ctr_evaluation/ctr_evaluation/nodes/evaluation_node.py"
    ).read_text(encoding="utf-8")

    assert 'self.create_publisher(CtrJointCommand, "/ctr/mppi_command"' in controller
    assert "slice_7g_controller_bypass" in controller
    assert 'self.create_subscription(CtrJointCommand, "/ctr/mppi_command"' in safety
    assert 'self.create_publisher(CtrJointCommand, "/ctr/safe_command"' in safety
    assert '"/ctr/safe_command",' in simulator
    assert '"/ctr/tactile/state"' in simulator and '"/ctr/tactile/state"' in evaluator
    assert '"/ctr/safety/status"' in safety and '"/ctr/safety/status"' in evaluator


def test_runtime_profile_yaml_is_closed_and_contains_approved_readiness():
    data = yaml.safe_load((REPO / "config/slice_7g_runtime_params.yaml").read_text(encoding="utf-8"))
    readiness = data["slice_7g_runtime"]["readiness"]
    assert readiness == {
        "timeout_seconds": 10.0,
        "minimum_stable_samples": 10,
        "minimum_stable_interval_seconds": 0.5,
        "q_variation_tolerance": 5.0e-5,
        "tip_variation_tolerance_m": 5.0e-5,
        "tactile_max_age_seconds": 0.10,
        "safety_max_age_seconds": 0.10,
    }


def test_development_profile_has_bounded_nonproduction_freshness_without_changing_source():
    source = _config()
    original = deepcopy(source)
    result = apply_slice_7g_development_simulation_profile(source, enabled=True)
    assert result["runtime"]["mode"] == "simulation"
    assert result["runtime"]["development_simulation"] is True
    assert result["safety"]["command_timeout"] == 5.0
    assert result["mppi"]["tactile"]["max_age_s"] == 5.0
    assert source == original
    assert source["safety"]["command_timeout"] == 0.10
    assert source["mppi"]["tactile"]["max_age_s"] == 0.10
