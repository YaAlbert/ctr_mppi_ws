import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
LAUNCH_PATH = REPO_ROOT / "src" / "ctr_bringup" / "launch" / "slice_7g_development_visual.launch.py"


def load_launch_module():
    spec = importlib.util.spec_from_file_location("slice_7g_development_visual_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def context(value):
    launch_context = SimpleNamespace(
        launch_configurations={"development_simulation": value}
    )
    launch_context.perform_substitution = lambda substitution: substitution.perform(
        launch_context
    )
    return launch_context


def test_visual_launch_requires_explicit_development_opt_in():
    module = load_launch_module()
    with pytest.raises(RuntimeError, match="development_simulation:=true"):
        module._require_development_opt_in(context("false"))
    assert module._require_development_opt_in(context("true")) == []


def test_visual_launch_is_simulator_only_and_uses_fixed_profile_contract():
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    assert '"runtime_mode": "simulation"' in source
    assert '"slice_7g_profile": "true"' in source
    assert '"development_simulation": "true"' in source
    assert '"enable_curved_lumen": "true"' in source
    assert '"start_safety_supervisor": "true"' in source
    assert "physical_hardware" not in source
    assert "mock_hardware" not in source


def test_simulation_launch_passes_profile_opt_ins_as_typed_booleans():
    source = (
        REPO_ROOT / "src" / "ctr_bringup" / "launch" / "simulation.launch.py"
    ).read_text(encoding="utf-8")
    validator = source.split('executable="parameter_validator_node"', 1)[1].split(
        'executable="simulator_node"', 1
    )[0]
    assert '"slice_7g_profile": ParameterValue(slice_7g_profile, value_type=bool)' in validator
    assert "development_simulation, value_type=bool" in validator
    assert "development_simulation, value_type=str" not in validator


def test_rviz_configuration_displays_ctr_lumen_reference_and_tip():
    config = (REPO_ROOT / "config" / "slice_7g_development.rviz").read_text(encoding="utf-8")
    assert "/ctr/visualization" in config
    assert "/ctr/reference/path" in config
    assert "/ctr/tip" in config
    assert "base_link" in config
