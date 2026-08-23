import unittest

from ctr_interfaces.msg import CtrJointCommand, CtrTactileState


class SafetyRosContractTest(unittest.TestCase):
    def test_command_contract_is_six_actuators(self):
        self.assertEqual(6, len(CtrJointCommand().q_dot))

    def test_tactile_regions_are_typed(self):
        self.assertEqual(0, CtrTactileState.REGION_NO_CONTACT)
        self.assertEqual(1, CtrTactileState.REGION_CONTACT)
        self.assertEqual(2, CtrTactileState.REGION_WARNING)
        self.assertEqual(3, CtrTactileState.REGION_STOP)


if __name__ == "__main__":
    unittest.main()
