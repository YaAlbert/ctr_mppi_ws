import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
LAUNCH_PATH = REPO_ROOT / "src" / "ctr_bringup" / "launch" / "slice_7g_development_visual.launch.py"


def load_launch_module():
    spec = importlib.util.spec_from_file_location("slice_7g_development_visual_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def context(value, target_source="profile"):
    launch_context = SimpleNamespace(
        launch_configurations={
            "development_simulation": value,
            "target_source": target_source,
        }
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


def test_visual_launch_target_source_defaults_to_profile_and_rejects_unknown_modes():
    module = load_launch_module()
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    assert '"target_source",\n                default_value="profile"' in source
    assert module._require_development_opt_in(context("true", "cli")) == []
    assert module._require_development_opt_in(context("true", "rviz")) == []
    with pytest.raises(RuntimeError, match="profile, cli, or rviz"):
        module._require_development_opt_in(context("true", "unknown"))


def test_visual_launch_is_simulator_only_and_uses_fixed_profile_contract():
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    assert '"runtime_mode": "simulation"' in source
    assert '"slice_7g_profile": "true"' in source
    assert '"development_simulation": "true"' in source
    assert '"enable_development_visualization": "true"' in source
    assert '"enable_curved_lumen": "true"' in source
    assert '"start_safety_supervisor": "true"' in source
    assert '"safety_supervisor_start_delay": "1.0"' in source
    assert '"target_source": target_source' in source
    assert '"target_x": target_x' in source
    assert '"target_y": target_y' in source
    assert '"target_z": target_z' in source
    assert "external_target" in source
    assert '"start_reference_manager": start_reference_manager' in source
    assert "'true' if '" in source
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
    simulator = source.split('executable="simulator_node"', 1)[1].split(
        'executable="manual_command_publisher"', 1
    )[0]
    assert "enable_development_visualization, value_type=bool" in simulator


def test_rviz_configuration_displays_ctr_lumen_reference_and_tip():
    config = yaml.safe_load(
        (REPO_ROOT / "config" / "slice_7g_development.rviz").read_text(encoding="utf-8")
    )
    manager = config["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == "world"
    assert "base_link" not in manager["Global Options"]["Fixed Frame"]
    displays = {display["Name"]: display for display in manager["Displays"]}
    expected = {
        "Curved lumen surface",
        "Curved lumen wireframe",
        "Lumen centerline",
        "CTR backbone",
        "Reference path",
        "Actual tip trajectory",
        "Tip pose",
        "Target",
        "Target candidate",
        "Invalid target candidate",
        "Target selection status",
    }
    assert expected.issubset(displays)
    for name in ("Curved lumen surface", "Curved lumen wireframe", "Lumen centerline"):
        assert displays[name]["Topic"]["Durability Policy"] == "Transient Local"
    assert displays["Reference path"]["Topic"]["Value"].endswith("/reference_path")
    assert displays["Reference path source poses"]["Topic"] == {
        "Depth": 1,
        "Durability Policy": "Transient Local",
        "History Policy": "Keep Last",
        "Reliability Policy": "Reliable",
        "Value": "/ctr/reference/path",
    }
    assert displays["Tip pose"]["Topic"]["Value"] == "/ctr/tip"
    assert displays["Tip pose"]["Shape"] == "Arrow"
    assert {
        key: displays["Tip pose"][key]
        for key in (
            "Alpha",
            "Color",
            "Head Length",
            "Head Radius",
            "Shaft Length",
            "Shaft Radius",
        )
    } == {
        "Alpha": 1.0,
        "Color": "255; 0; 0",
        "Head Length": 0.015,
        "Head Radius": 0.006,
        "Shaft Length": 0.05,
        "Shaft Radius": 0.0025,
    }
    tools = manager["Tools"]
    publish_point = next(tool for tool in tools if tool["Class"] == "rviz_default_plugins/PublishPoint")
    assert publish_point["Topic"] == "/ctr/target_point_candidate"
    assert publish_point["Single click"] is True


def test_visual_launch_defines_semantic_identity_transform_for_fixed_robot_base():
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    assert 'package="tf2_ros"' in source
    assert 'executable="static_transform_publisher"' in source
    assert '"--frame-id", "world", "--child-frame-id", "base_link"' in source
    assert '"--x", "0", "--y", "0", "--z", "0"' in source
