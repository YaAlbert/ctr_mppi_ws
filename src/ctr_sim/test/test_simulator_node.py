import copy
import sys
import types
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_model"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_mppi_controller"))


try:
    import ctr_interfaces.msg as ctr_interfaces_msg_module  # noqa: F401
    for required_name in ("CtrBackbone", "CtrJointCommand", "CtrJointState", "CtrState"):
        if not hasattr(ctr_interfaces_msg_module, required_name):
            raise ImportError(required_name)
except ImportError:
    ctr_interfaces_module = types.ModuleType("ctr_interfaces")
    ctr_interfaces_msg_module = types.ModuleType("ctr_interfaces.msg")
    for name in ("CtrBackbone", "CtrJointCommand", "CtrJointState", "CtrState"):
        setattr(ctr_interfaces_msg_module, name, type(name, (), {}))
    sys.modules["ctr_interfaces"] = ctr_interfaces_module
    sys.modules["ctr_interfaces.msg"] = ctr_interfaces_msg_module


from builtin_interfaces.msg import Time  # noqa: E402
from ctr_bringup.parameter_validation import load_parameter_files  # noqa: E402
from ctr_mppi_controller.curved_lumen import CurvedLumen  # noqa: E402
from ctr_mppi_controller.cylindrical_lumen import CylindricalLumen  # noqa: E402
from ctr_mppi_controller.lumen_factory import (  # noqa: E402
    config_with_lumen_overrides,
    lumen_geometry_from_config,
    lumen_mode_from_config,
)
from ctr_sim.nodes.simulator_node import CTRSimulatorNode, _point_from_array  # noqa: E402


CONFIG_FILES = [
    REPO_ROOT / "config" / "robot_params.yaml",
    REPO_ROOT / "config" / "model_params.yaml",
    REPO_ROOT / "config" / "mppi_params.yaml",
    REPO_ROOT / "config" / "simulation_params.yaml",
    REPO_ROOT / "config" / "safety_params.yaml",
    REPO_ROOT / "config" / "tactile_params.yaml",
    REPO_ROOT / "config" / "hardware_params.yaml",
]


def load_config():
    return copy.deepcopy(load_parameter_files(CONFIG_FILES))


def no_lumen_config():
    return config_with_lumen_overrides(
        load_config(),
        enable_cylindrical_lumen=False,
        enable_curved_lumen=False,
    )


def cylinder_config():
    return config_with_lumen_overrides(
        load_config(),
        enable_cylindrical_lumen=True,
        enable_curved_lumen=False,
    )


def curved_config(lumen_type="circular_arc"):
    return config_with_lumen_overrides(
        load_config(),
        enable_cylindrical_lumen=False,
        enable_curved_lumen=True,
        curved_lumen_type=lumen_type,
    )


def simulator_shell(config):
    node = object.__new__(CTRSimulatorNode)
    node.frame_id = config["robot"]["frames"]["base"]
    node.target_position = np.asarray(config["goal"]["position"], dtype=float)
    node.lumen_mode = lumen_mode_from_config(config)
    node.lumen_geometry = lumen_geometry_from_config(config)
    node.lumen = node.lumen_geometry
    return node


def sample_backbone():
    return np.array(
        [
            [0.0, 0.0, 0.0],
            [0.003, 0.0, 0.040],
            [0.006, 0.0, 0.080],
        ],
        dtype=float,
    )


class SimulatorNodeLumenRuntimeTest(unittest.TestCase):
    def test_simulator_source_uses_shared_lumen_factory(self):
        source = (PACKAGE_ROOT / "ctr_sim" / "nodes" / "simulator_node.py").read_text(encoding="utf-8")
        self.assertIn("config_with_lumen_overrides", source)
        self.assertIn("lumen_geometry_from_config", source)
        self.assertIn("lumen_mode_from_config", source)
        self.assertIn("parse_launch_bool", source)
        self.assertNotIn("config_with_cylinder_overrides", source)

    def test_factory_modes_construct_expected_geometry_for_simulator(self):
        cases = (
            (no_lumen_config(), "none", type(None)),
            (cylinder_config(), "cylindrical", CylindricalLumen),
            (curved_config("circular_arc"), "curved", CurvedLumen),
            (curved_config("s_curve"), "curved", CurvedLumen),
        )
        for config, expected_mode, expected_type in cases:
            with self.subTest(expected_mode=expected_mode, expected_type=expected_type):
                self.assertEqual(expected_mode, lumen_mode_from_config(config))
                self.assertIsInstance(lumen_geometry_from_config(config), expected_type)

    def test_no_lumen_marker_array_keeps_geometry_independent_markers(self):
        node = simulator_shell(no_lumen_config())
        backbone = sample_backbone()
        msg = node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
        self.assertEqual(["ctr_backbone", "ctr_tip", "ctr_target"], [marker.ns for marker in msg.markers])

    def test_curved_marker_array_skips_cylinder_specific_markers(self):
        node = simulator_shell(curved_config("circular_arc"))
        backbone = sample_backbone()
        msg = node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
        self.assertEqual(["ctr_backbone", "ctr_tip", "ctr_target"], [marker.ns for marker in msg.markers])

    def test_cylindrical_marker_array_retains_existing_lumen_markers(self):
        node = simulator_shell(cylinder_config())
        backbone = sample_backbone()
        msg = node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
        namespaces = [marker.ns for marker in msg.markers]
        self.assertEqual(["ctr_backbone", "ctr_tip", "ctr_target"], namespaces[:3])
        self.assertGreater(namespaces.count("cylindrical_lumen"), 0)


if __name__ == "__main__":
    unittest.main()
