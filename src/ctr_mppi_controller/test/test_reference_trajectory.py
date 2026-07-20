import math
import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ctr_mppi_controller.reference_trajectory import (  # noqa: E402
    ReferenceTrajectory,
    elapsed_time_index,
    generate_circle,
    generate_ellipse,
    generate_helix,
    generate_trajectory,
)


def circle_kwargs(**overrides):
    values = {
        "center": [0.02, -0.01, 0.08],
        "radius": 0.004,
        "angular_velocity": 2.0 * math.pi,
        "phase": 0.0,
        "duration": 1.0,
        "sample_period": 0.25,
        "frame_id": "base_link",
        "completion_behavior": "loop",
    }
    values.update(overrides)
    return values


def ellipse_kwargs(**overrides):
    values = {
        "center": [0.02, -0.01, 0.08],
        "radii": [0.006, 0.002],
        "angular_velocity": 2.0 * math.pi,
        "phase": 0.0,
        "duration": 1.0,
        "sample_period": 0.25,
        "frame_id": "base_link",
        "completion_behavior": "loop",
    }
    values.update(overrides)
    return values


def helix_kwargs(**overrides):
    values = {
        "center": [0.02, -0.01, 0.08],
        "radius": 0.004,
        "height": 0.01,
        "angular_velocity": 2.0 * math.pi,
        "phase": 0.0,
        "duration": 1.0,
        "sample_period": 0.25,
        "frame_id": "base_link",
        "completion_behavior": "loop",
    }
    values.update(overrides)
    return values


class ReferenceTrajectoryGenerationTest(unittest.TestCase):
    def test_circle_output_shape(self):
        trajectory = generate_circle(**circle_kwargs())
        self.assertEqual((5, 3), trajectory.points.shape)
        self.assertEqual("circle", trajectory.trajectory_type)
        self.assertTrue(np.all(np.isfinite(trajectory.points)))

    def test_circle_radius_consistency(self):
        radius = 0.004
        center = np.array(circle_kwargs()["center"], dtype=float)
        trajectory = generate_circle(**circle_kwargs(radius=radius))
        radial_distance = np.linalg.norm(trajectory.points[:, :2] - center[:2], axis=1)
        self.assertTrue(np.allclose(radius, radial_distance))
        self.assertTrue(np.allclose(center[2], trajectory.points[:, 2]))

    def test_ellipse_output_shape(self):
        trajectory = generate_ellipse(**ellipse_kwargs())
        self.assertEqual((5, 3), trajectory.points.shape)
        self.assertEqual("ellipse", trajectory.trajectory_type)
        self.assertTrue(np.all(np.isfinite(trajectory.points)))

    def test_unequal_ellipse_axes(self):
        center = np.array(ellipse_kwargs()["center"], dtype=float)
        trajectory = generate_ellipse(**ellipse_kwargs(radii=[0.006, 0.002]))
        self.assertAlmostEqual(center[0] + 0.006, trajectory.points[0, 0])
        self.assertAlmostEqual(center[1], trajectory.points[0, 1])
        self.assertAlmostEqual(center[0], trajectory.points[1, 0])
        self.assertAlmostEqual(center[1] + 0.002, trajectory.points[1, 1])

    def test_helix_output_shape(self):
        trajectory = generate_helix(**helix_kwargs())
        self.assertEqual((5, 3), trajectory.points.shape)
        self.assertEqual("helix", trajectory.trajectory_type)
        self.assertTrue(np.all(np.isfinite(trajectory.points)))

    def test_positive_height_axial_progression(self):
        trajectory = generate_helix(**helix_kwargs(height=0.012))
        self.assertAlmostEqual(0.08, trajectory.points[0, 2])
        self.assertAlmostEqual(0.092, trajectory.points[-1, 2])
        self.assertTrue(np.all(np.diff(trajectory.points[:, 2]) > 0.0))

    def test_negative_height_axial_progression(self):
        trajectory = generate_helix(**helix_kwargs(height=-0.012))
        self.assertAlmostEqual(0.08, trajectory.points[0, 2])
        self.assertAlmostEqual(0.068, trajectory.points[-1, 2])
        self.assertTrue(np.all(np.diff(trajectory.points[:, 2]) < 0.0))

    def test_invalid_zero_helix_height(self):
        with self.assertRaises(ValueError):
            generate_helix(**helix_kwargs(height=0.0))

    def test_invalid_trajectory_type(self):
        with self.assertRaises(ValueError):
            generate_trajectory(trajectory_type="square", **circle_kwargs())

    def test_invalid_duration_and_sample_period(self):
        for overrides in ({"duration": 0.0}, {"duration": -1.0}, {"sample_period": 0.0}, {"sample_period": -0.1}):
            with self.subTest(overrides=overrides):
                with self.assertRaises(ValueError):
                    generate_circle(**circle_kwargs(**overrides))
        with self.assertRaises(ValueError):
            generate_circle(**circle_kwargs(duration=0.1, sample_period=0.2))

    def test_invalid_dimensions(self):
        with self.assertRaises(ValueError):
            generate_circle(**circle_kwargs(center=[0.0, 0.0]))
        with self.assertRaises(ValueError):
            generate_ellipse(**ellipse_kwargs(radii=[0.004, 0.002, 0.001]))
        with self.assertRaises(ValueError):
            ReferenceTrajectory(
                points=np.zeros((2, 2)),
                sample_period=0.1,
                frame_id="base_link",
                trajectory_type="circle",
                completion_behavior="loop",
            )
        with self.assertRaises(ValueError):
            generate_ellipse(**ellipse_kwargs(radii=[0.004, 0.0]))

    def test_nan_and_inf_rejection(self):
        for overrides in (
            {"center": [math.nan, 0.0, 0.0]},
            {"angular_velocity": math.inf},
            {"phase": math.nan},
            {"radius": math.inf},
        ):
            with self.subTest(overrides=overrides):
                with self.assertRaises(ValueError):
                    generate_circle(**circle_kwargs(**overrides))
        with self.assertRaises(ValueError):
            ReferenceTrajectory(
                points=np.array([[0.0, 0.0, 0.0], [math.inf, 0.0, 0.0]]),
                sample_period=0.1,
                frame_id="base_link",
                trajectory_type="circle",
                completion_behavior="loop",
            )

    def test_loop_horizon_extraction(self):
        trajectory = generate_circle(**circle_kwargs(completion_behavior="loop"))
        horizon = trajectory.horizon_at_index(start_index=4, horizon_length=4)
        self.assertEqual((4, 3), horizon.points.shape)
        self.assertEqual([4, 0, 1, 2], horizon.indices.tolist())
        self.assertFalse(horizon.completed)
        self.assertTrue(np.allclose(trajectory.points[4], horizon.current_point))

    def test_hold_final_horizon_extraction(self):
        trajectory = generate_circle(**circle_kwargs(completion_behavior="hold_final"))
        horizon = trajectory.horizon_at_index(start_index=3, horizon_length=4)
        self.assertEqual((4, 3), horizon.points.shape)
        self.assertEqual([3, 4, 4, 4], horizon.indices.tolist())
        self.assertTrue(horizon.completed)
        self.assertTrue(np.allclose(trajectory.points[-1], horizon.points[-1]))

    def test_exact_horizon_shape(self):
        trajectory = generate_circle(**circle_kwargs())
        horizon = trajectory.horizon_at_index(start_index=1, horizon_length=3)
        self.assertEqual((3, 3), horizon.points.shape)
        self.assertEqual((3,), horizon.indices.shape)

    def test_negative_elapsed_time_maps_to_zero(self):
        self.assertEqual(0, elapsed_time_index(current_time=1.0, start_time=2.0, sample_period=0.1))
        trajectory = generate_circle(**circle_kwargs())
        horizon = trajectory.horizon_at_time(current_time=1.0, start_time=2.0, horizon_length=2)
        self.assertEqual(0, horizon.start_index)
        self.assertEqual([0, 1], horizon.indices.tolist())

    def test_exact_time_boundary_indexing(self):
        self.assertEqual(4, elapsed_time_index(current_time=1.0, start_time=0.0, sample_period=0.25))
        self.assertEqual(3, elapsed_time_index(current_time=0.3, start_time=0.0, sample_period=0.1))

    def test_invalid_completion_behavior(self):
        with self.assertRaises(ValueError):
            generate_circle(**circle_kwargs(completion_behavior="stop"))

    def test_non_positive_horizon_length(self):
        trajectory = generate_circle(**circle_kwargs())
        with self.assertRaises(ValueError):
            trajectory.horizon_at_index(start_index=0, horizon_length=0)

    def test_deterministic_output(self):
        first = generate_helix(**helix_kwargs(completion_behavior="hold_final"))
        second = generate_helix(**helix_kwargs(completion_behavior="hold_final"))
        self.assertTrue(np.array_equal(first.points, second.points))
        self.assertTrue(
            np.array_equal(
                first.horizon_at_index(start_index=3, horizon_length=5).points,
                second.horizon_at_index(start_index=3, horizon_length=5).points,
            )
        )


if __name__ == "__main__":
    unittest.main()
