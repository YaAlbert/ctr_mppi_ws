import importlib.util
import os
import tempfile
import unittest
from pathlib import Path

from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


REPO_ROOT = Path(__file__).resolve().parents[3]
LAUNCH_PATH = REPO_ROOT / "src" / "ctr_bringup" / "launch" / "simulation.launch.py"


def load_launch_module():
    spec = importlib.util.spec_from_file_location("simulation_launch_config_array", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    if spec.loader is None:
        raise AssertionError("launch module loader is unavailable")
    spec.loader.exec_module(module)
    return module


def iter_launch_entities(entities):
    """Walk nested launch actions such as the visual-mode safety startup timer."""

    for entity in entities:
        yield entity
        get_children = getattr(entity, "get_sub_entities", None)
        if get_children is not None:
            children = tuple(get_children())
            if children:
                yield from iter_launch_entities(children)
                continue
        timer_children = getattr(entity, "_TimerAction__actions", ())
        if timer_children:
            yield from iter_launch_entities(timer_children)


class SimulationLaunchConfigPathsTest(unittest.TestCase):
    def test_config_paths_are_independent_string_array_elements_for_all_consumers(self):
        module = load_launch_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            for name in module.CONFIG_NAMES:
                (root / name).write_text("placeholder: true\n", encoding="utf-8")
            previous_log_dir = os.environ.get("ROS_LOG_DIR")
            os.environ["ROS_LOG_DIR"] = str(root / "ros_log")
            module.get_package_share_directory = lambda package_name: str(root)
            try:
                description = module.generate_launch_description()
                context = LaunchContext()
                context.launch_configurations["config_root"] = str(root)
                expected = [str((root / name).resolve()) for name in module.CONFIG_NAMES]
                values = []
                for entity in iter_launch_entities(description.entities):
                    if not isinstance(entity, Node):
                        continue
                    for parameter_map in entity._Node__parameters:
                        for key_substitutions, parameter_value in parameter_map.items():
                            key = perform_substitutions(context, key_substitutions)
                            if key != "config_paths":
                                continue
                            value = parameter_value.evaluate(context)
                            self.assertIsInstance(value, list)
                            self.assertEqual(expected, value)
                            self.assertEqual(len(expected), len(value))
                            self.assertTrue(all(isinstance(path, str) for path in value))
                            self.assertTrue(all(path.endswith(".yaml") for path in value))
                            self.assertTrue(all(".yaml/" not in path for path in value))
                            self.assertTrue(all(Path(path).is_file() for path in value))
                            values.append(value)
                self.assertGreaterEqual(len(values), 5)
                self.assertTrue(all(value == expected for value in values))
            finally:
                if previous_log_dir is None:
                    os.environ.pop("ROS_LOG_DIR", None)
                else:
                    os.environ["ROS_LOG_DIR"] = previous_log_dir

    def test_safety_lumen_flags_are_boolean_parameters(self):
        module = load_launch_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            previous_log_dir = os.environ.get("ROS_LOG_DIR")
            os.environ["ROS_LOG_DIR"] = str(root / "ros_log")
            module.get_package_share_directory = lambda package_name: str(root)
            try:
                description = module.generate_launch_description()
                context = LaunchContext()
                context.launch_configurations.update(
                    {
                        "config_root": str(root),
                        "enable_cylindrical_lumen": "true",
                        "enable_curved_lumen": "false",
                    }
                )
                safety_nodes = [
                    entity
                    for entity in iter_launch_entities(description.entities)
                    if isinstance(entity, Node) and entity._Node__node_name == "safety_supervisor"
                ]
                self.assertEqual(1, len(safety_nodes))
                parameter_map = safety_nodes[0]._Node__parameters[0]
                values = {
                    key: value.evaluate(context)
                    for key_substitutions, value in parameter_map.items()
                    for key in [perform_substitutions(context, key_substitutions)]
                    if key in {"enable_cylindrical_lumen", "enable_curved_lumen"}
                }
                self.assertEqual({"enable_cylindrical_lumen": True, "enable_curved_lumen": False}, values)
                self.assertTrue(all(isinstance(value, bool) for value in values.values()))
            finally:
                if previous_log_dir is None:
                    os.environ.pop("ROS_LOG_DIR", None)
                else:
                    os.environ["ROS_LOG_DIR"] = previous_log_dir


if __name__ == "__main__":
    unittest.main()
