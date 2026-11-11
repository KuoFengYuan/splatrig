# SPDX-License-Identifier: MIT
import math
import unittest

from splatrig.clock import Clock, fit_clock
from splatrig.geometry import Pose, Trajectory, nanoseconds


class GeometryTests(unittest.TestCase):
    def assertVector(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=9)

    def test_compose_inverse_and_extrinsic(self):
        world_rig = Pose((2, 3, 4), (math.sqrt(0.5), 0, 0, math.sqrt(0.5)))
        rig_camera = Pose((0.1, 0, 0), (1, 0, 0, 0))
        camera = world_rig.compose(rig_camera)
        self.assertVector(camera.translation, (2, 3.1, 4))
        point = (0.3, -0.7, 2)
        self.assertVector(camera.inverse().transform(camera.transform(point)), point)

    def test_interpolate_rotation_and_translation(self):
        trajectory = Trajectory([
            {"timestamp_ns": 0, "translation": [0, 0, 0], "quaternion_wxyz": [1, 0, 0, 0]},
            {"timestamp_ns": 100, "translation": [2, 4, 6], "quaternion_wxyz": [0, 0, 0, 1]},
        ])
        middle = trajectory.at(50)
        self.assertVector(middle.translation, (1, 2, 3))
        self.assertVector(middle.transform((1, 0, 0)), (1, 3, 3))

    def test_antipodal_quaternions_do_not_spin(self):
        trajectory = Trajectory([
            {"timestamp_ns": 0, "translation": [0, 0, 0], "quaternion_wxyz": [1, 0, 0, 0]},
            {"timestamp_ns": 100, "translation": [0, 0, 0], "quaternion_wxyz": [-1, 0, 0, 0]},
        ])
        self.assertVector(trajectory.at(50).transform((1, 2, 3)), (1, 2, 3))

    def test_pose_gaps_and_extrapolation_are_rejected(self):
        records = [{"timestamp_ns": t, **Pose.identity().to_dict()} for t in [10, 1000]]
        trajectory = Trajectory(records, 100)
        self.assertEqual(trajectory.at(10), Pose.identity())
        for timestamp in [0, 11, 1001]:
            with self.assertRaises(ValueError):
                trajectory.at(timestamp)

    def test_duplicate_or_backwards_times_are_rejected(self):
        for timestamps in [[1, 1], [2, 1]]:
            with self.assertRaises(ValueError):
                Trajectory([{"timestamp_ns": t, **Pose.identity().to_dict()}
                            for t in timestamps])

    def test_invalid_geometry_is_rejected(self):
        for q in [(0, 0, 0, 0), (2, 0, 0, 0), (float("nan"), 0, 0, 0)]:
            with self.assertRaises(ValueError):
                Pose((0, 0, 0), q)
        with self.assertRaises(ValueError):
            Pose((math.inf, 0, 0), (1, 0, 0, 0))
        for t in [True, 1.0, -1]:
            with self.assertRaises(ValueError):
                nanoseconds(t)


class ClockTests(unittest.TestCase):
    def test_epoch_nanoseconds_retain_single_nanosecond_steps(self):
        anchor = 1_800_000_000_000_000_000
        clock = Clock(anchor, anchor+123)
        self.assertEqual(clock.map(anchor+1), anchor+124)

    def test_fit_recovers_offset_and_drift(self):
        camera = 1_800_000_000_000_000_000
        reference = camera+70_000_000
        pairs = [(camera+i*1_000_000_000, reference+i*1_000_100_000) for i in range(10)]
        clock, report = fit_clock(pairs)
        self.assertAlmostEqual(report["drift_ppm"], 100, places=6)
        self.assertEqual(report["max_residual_ns"], 0)
        self.assertEqual(clock.map(camera+5_000_000_000), reference+5_000_500_000)

    def test_jitter_cannot_be_marked_calibrated(self):
        pairs = [(0, 1_000_000_000), (1_000_000_000, 2_030_000_000),
                 (2_000_000_000, 3_000_000_000)]
        with self.assertRaisesRegex(ValueError, "residual"):
            fit_clock(pairs)

    def test_clock_unit_errors_and_insufficient_events(self):
        with self.assertRaises(ValueError):
            fit_clock([(0, 0), (1, 1)])
        with self.assertRaises(ValueError):
            fit_clock([(0, 0), (0, 1), (2, 2)])
        with self.assertRaises(ValueError):
            fit_clock([(0, 0), (1, 1_000_000_000), (2, 2_000_000_000)])


if __name__ == "__main__":
    unittest.main()
