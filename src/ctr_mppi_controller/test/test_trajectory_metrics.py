import copy
import math
import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))

from ctr_bringup.parameter_validation import load_parameter_files, validate_or_raise  # noqa: E402
from ctr_mppi_controller.trajectory_metrics import (  # noqa: E402
    NO_TRANSIENT_REACHED,
    TrajectoryMetricsAccumulator,
    TrajectoryMetricsConfig,
)


CONFIG_FILES = [
    REPO_ROOT / "config" / "robot_params.yaml",
    REPO_ROOT / "config" / "model_params.yaml",
    REPO_ROOT / "config" / "mppi_params.yaml",
    REPO_ROOT / "config" / "simulation_params.yaml",
    REPO_ROOT / "config" / "safety_params.yaml",
    REPO_ROOT / "config" / "tactile_params.yaml",
    REPO_ROOT / "config" / "hardware_params.yaml",
]


def make_config():
    config = load_parameter_files(CONFIG_FILES)
    validate_or_raise(config)
    return copy.deepcopy(config)


def make_accumulator(*, command_dimension=3, tolerance=0.5, stable_cycles=3):
    config = TrajectoryMetricsConfig(
        enabled=True,
        publish_frequency=5.0,
        transient_tolerance=tolerance,
        stable_cycles=stable_cycles,
        reset_on_new_trajectory=True,
    )
    return TrajectoryMetricsAccumulator(
        config=config,
        command_dimension=command_dimension,
        trajectory_type="circle",
    )


def add_sample(
    accumulator,
    *,
    timestamp,
    error,
    command=None,
    dt=0.1,
    solve_time=0.01,
    command_saturated=False,
):
    if command is None:
        command = np.zeros(accumulator.command_dimension)
    accumulator.add_sample(
        timestamp=timestamp,
        tip_position=[error, 0.0, 0.0],
        reference_position=[0.0, 0.0, 0.0],
        command=command,
        dt=dt,
        solve_time=solve_time,
        command_saturated=command_saturated,
        control_period=0.05,
    )


class TrajectoryMetricsAccumulatorTest(unittest.TestCase):
    def test_config_loads_from_project_yaml(self):
        config = make_config()
        metrics_config = TrajectoryMetricsConfig.from_project_config(config)
        self.assertTrue(metrics_config.enabled)
        self.assertGreater(metrics_config.publish_frequency, 0.0)

    def test_zero_error_sequence(self):
        accumulator = make_accumulator()
        add_sample(accumulator, timestamp=0.0, error=0.0)
        add_sample(accumulator, timestamp=0.1, error=0.0)
        snapshot = accumulator.snapshot()
        self.assertEqual(2, snapshot.sample_count)
        self.assertAlmostEqual(0.0, snapshot.rmse)
        self.assertAlmostEqual(0.0, snapshot.mean_error)
        self.assertAlmostEqual(0.0, snapshot.max_error)

    def test_known_rmse_calculation(self):
        accumulator = make_accumulator()
        add_sample(accumulator, timestamp=0.0, error=1.0)
        add_sample(accumulator, timestamp=1.0, error=2.0)
        self.assertAlmostEqual(math.sqrt(2.5), accumulator.snapshot().rmse)

    def test_known_mean_error_calculation(self):
        accumulator = make_accumulator()
        add_sample(accumulator, timestamp=0.0, error=1.0)
        add_sample(accumulator, timestamp=1.0, error=2.0)
        add_sample(accumulator, timestamp=2.0, error=4.0)
        self.assertAlmostEqual(7.0 / 3.0, accumulator.snapshot().mean_error)

    def test_known_maximum_error_calculation(self):
        accumulator = make_accumulator()
        for index, error in enumerate([0.2, 0.7, 0.3]):
            add_sample(accumulator, timestamp=float(index), error=error)
        self.assertAlmostEqual(0.7, accumulator.snapshot().max_error)

    def test_control_effort_integration(self):
        accumulator = make_accumulator(command_dimension=2)
        add_sample(accumulator, timestamp=0.0, error=0.0, command=np.array([1.0, 2.0]), dt=0.1)
        add_sample(accumulator, timestamp=0.1, error=0.0, command=np.array([3.0, 4.0]), dt=0.2)
        self.assertAlmostEqual(5.5, accumulator.snapshot().control_effort)

    def test_transient_duration_detection(self):
        accumulator = make_accumulator(tolerance=0.5, stable_cycles=3)
        for timestamp, error in [(0.0, 1.0), (1.0, 0.4), (2.0, 0.3), (3.0, 0.2)]:
            add_sample(accumulator, timestamp=timestamp, error=error)
        self.assertAlmostEqual(1.0, accumulator.snapshot().transient_duration)

    def test_no_transient_condition(self):
        accumulator = make_accumulator(tolerance=0.5, stable_cycles=3)
        for timestamp, error in [(0.0, 1.0), (1.0, 0.4), (2.0, 0.6), (3.0, 0.2)]:
            add_sample(accumulator, timestamp=timestamp, error=error)
        self.assertAlmostEqual(NO_TRANSIENT_REACHED, accumulator.snapshot().transient_duration)

    def test_saturation_counting(self):
        accumulator = make_accumulator()
        add_sample(accumulator, timestamp=0.0, error=0.0, command_saturated=False)
        add_sample(accumulator, timestamp=0.1, error=0.0, command_saturated=True)
        add_sample(accumulator, timestamp=0.2, error=0.0, command_saturated=True)
        self.assertEqual(2, accumulator.snapshot().command_saturation_count)

    def test_maximum_command_per_joint(self):
        accumulator = make_accumulator(command_dimension=3)
        add_sample(accumulator, timestamp=0.0, error=0.0, command=np.array([1.0, -2.0, 0.5]))
        add_sample(accumulator, timestamp=0.1, error=0.0, command=np.array([-3.0, 1.0, 4.0]))
        self.assertTrue(np.allclose([3.0, 2.0, 4.0], accumulator.snapshot().maximum_command_per_joint))

    def test_reset_behavior(self):
        accumulator = make_accumulator()
        add_sample(accumulator, timestamp=0.0, error=1.0)
        accumulator.reset(trajectory_type="ellipse")
        snapshot = accumulator.snapshot()
        self.assertEqual(0, snapshot.sample_count)
        self.assertEqual("ellipse", snapshot.trajectory_type)
        self.assertEqual("no_valid_samples", snapshot.completion_state)

    def test_invalid_sample_rejection(self):
        accumulator = make_accumulator()
        with self.assertRaises(ValueError):
            accumulator.add_sample(
                timestamp=0.0,
                tip_position=[math.nan, 0.0, 0.0],
                reference_position=[0.0, 0.0, 0.0],
                command=np.zeros(3),
                dt=0.1,
                solve_time=0.01,
                command_saturated=False,
            )
        self.assertEqual(0, accumulator.snapshot().sample_count)

    def test_zero_sample_reporting(self):
        snapshot = make_accumulator().snapshot()
        self.assertEqual(0, snapshot.sample_count)
        self.assertTrue(math.isnan(snapshot.rmse))
        self.assertEqual("no_valid_samples", snapshot.completion_state)

    def test_deterministic_accumulation(self):
        first = make_accumulator()
        second = make_accumulator()
        samples = [
            (0.0, 0.5, np.array([0.1, 0.2, 0.3])),
            (0.1, 0.2, np.array([0.0, -0.2, 0.1])),
            (0.2, 0.1, np.array([0.3, 0.0, -0.1])),
        ]
        for accumulator in (first, second):
            for timestamp, error, command in samples:
                add_sample(accumulator, timestamp=timestamp, error=error, command=command)

        first_snapshot = first.snapshot()
        second_snapshot = second.snapshot()
        self.assertEqual(first_snapshot.sample_count, second_snapshot.sample_count)
        self.assertAlmostEqual(first_snapshot.rmse, second_snapshot.rmse)
        self.assertAlmostEqual(first_snapshot.control_effort, second_snapshot.control_effort)
        self.assertTrue(
            np.array_equal(
                first_snapshot.maximum_command_per_joint,
                second_snapshot.maximum_command_per_joint,
            )
        )


if __name__ == "__main__":
    unittest.main()
