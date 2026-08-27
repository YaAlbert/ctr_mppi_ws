"""Static Slice 7F command-authority contract checks."""

from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[3]
MPPI_SOURCE = REPO_ROOT / "src" / "ctr_mppi_controller" / "ctr_mppi_controller" / "nodes" / "mppi_controller_node.py"
SAFETY_SOURCE = REPO_ROOT / "src" / "ctr_safety" / "ctr_safety" / "nodes" / "safety_supervisor_node.py"
SIM_SOURCE = REPO_ROOT / "src" / "ctr_sim" / "ctr_sim" / "nodes" / "simulator_node.py"
LAUNCH_SOURCE = REPO_ROOT / "src" / "ctr_bringup" / "launch" / "simulation.launch.py"


class Slice7FAuthorityContractTest(unittest.TestCase):
    def test_mppi_safe_bypass_is_conditional_and_raw_topic_is_authoritative(self):
        source = MPPI_SOURCE.read_text(encoding="utf-8")
        self.assertIn('self.create_publisher(CtrJointCommand, "/ctr/mppi_command", 10)', source)
        self.assertIn("if self.publish_safe_for_sim:", source)
        self.assertIn('self.create_publisher(CtrJointCommand, "/ctr/safe_command", 10)', source)

    def test_safety_mediates_raw_command_to_safe_command(self):
        source = SAFETY_SOURCE.read_text(encoding="utf-8")
        self.assertIn('CtrJointCommand, "/ctr/mppi_command", self._on_command, 10,', source)
        self.assertIn("callback_group=self._command_callback_group", source)
        self.assertIn('self.create_publisher(CtrJointCommand, "/ctr/safe_command", 10)', source)

    def test_simulator_consumes_safe_command_only(self):
        source = SIM_SOURCE.read_text(encoding="utf-8")
        self.assertIn('"/ctr/safe_command"', source)
        self.assertNotIn('"/ctr/mppi_command"', source)

    def test_launch_contract_disables_manual_and_direct_safe_publishers(self):
        source = LAUNCH_SOURCE.read_text(encoding="utf-8")
        self.assertIn('"start_manual_command_publisher"', source)
        self.assertIn('"mppi_publish_safe_for_simulation"', source)
        self.assertIn('"start_mppi_controller"', source)
        self.assertIn('"start_safety_supervisor"', source)


if __name__ == "__main__":
    unittest.main()
