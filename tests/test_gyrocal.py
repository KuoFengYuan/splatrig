# SPDX-License-Identifier: MIT
import math
import random
import unittest

from splatrig.geometry import rotate
from splatrig.gyrocal import (GPMF_TO_SPHERE, calibrate, det, matmul, polar_rotation,
                              quaternion, singular_values, transpose)
from splatrig.pipeline import lidar_sphere


def rotation_from_quaternion(q):
    w, x, y, z = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))


def angle_between(a, b):
    r = matmul(a, transpose(b))
    return math.degrees(math.acos(max(-1.0, min(1.0, (r[0][0] + r[1][1] + r[2][2] - 1) / 2))))


def motion(t):
    """Angular velocity (rad/s) exciting all three axes."""
    return (0.8 * math.sin(2.1 * t) + 0.3 * math.sin(7.3 * t),
            0.6 * math.sin(1.3 * t + 1) + 0.2 * math.sin(5.9 * t),
            0.5 * math.sin(3.7 * t + 2) + 0.25 * math.sin(0.7 * t))


def streams(rotation, offset_ns, seconds=20, noise=0.01, axes=3, seed=1):
    rng = random.Random(seed)
    camera_times = [round(i * 1e9 / 800) for i in range(seconds * 800)]
    camera = []
    for t in camera_times:
        w = motion(t / 1e9)
        w = tuple(v if i < axes else 0.0 for i, v in enumerate(w))
        camera.append(tuple(v + rng.gauss(0, noise) for v in w))
    lidar_times = [offset_ns + 500_000_000 + i * 5_000_000 for i in range(int((seconds - 1) * 200))]
    lidar = []
    for t in lidar_times:
        w = motion((t - offset_ns) / 1e9)
        w = tuple(v if i < axes else 0.0 for i, v in enumerate(w))
        lidar.append(tuple(sum(rotation[i][k] * w[k] for k in range(3)) + rng.gauss(0, noise)
                           for i in range(3)))
    return camera_times, camera, lidar_times, lidar


class MathTests(unittest.TestCase):
    def test_polar_singular_and_quaternion(self):
        q = (0.9, 0.2, -0.3, 0.25)
        n = math.sqrt(sum(x * x for x in q))
        r = rotation_from_quaternion(tuple(x / n for x in q))
        scaled = tuple(tuple(v * s for v, s in zip(row, (3.0, 2.0, 0.5))) for row in r)
        self.assertLess(angle_between(polar_rotation(scaled), r), 1e-9)
        self.assertTrue(all(abs(a - b) < 1e-9 for a, b in
                            zip(singular_values(scaled), (3.0, 2.0, 0.5))))
        self.assertTrue(all(abs(a - b) < 1e-12 for a, b in
                            zip(quaternion(r), tuple(x / n for x in q))))
        self.assertAlmostEqual(det(GPMF_TO_SPHERE), 1.0)

    def test_sphere_mapping_matches_measured_max2_axes(self):
        # GPMF stored axes (cam Z, X, Y): forward = -Y, down = -Z, right = -X.
        cols = list(zip(*GPMF_TO_SPHERE))
        self.assertEqual(cols, [(0.0, -1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0)])


class CalibrationTests(unittest.TestCase):
    def test_recovers_offset_and_rotation(self):
        q = (0.7, 0.1, -0.5, 0.5)
        n = math.sqrt(sum(x * x for x in q))
        truth = rotation_from_quaternion(tuple(x / n for x in q))
        offset = 1_794_387_316_738_611_819 + 123_400_000
        camera_times, camera, lidar_times, lidar = streams(truth, offset)
        result = calibrate(camera_times, camera, lidar_times, lidar,
                           guess_ns=offset - 123_400_000)
        self.assertLess(abs(result["offset_ns"] - offset), 1_000_000)
        self.assertGreater(result["magnitude_correlation"], 0.99)
        self.assertLess(angle_between(result["R_lidar_gpmf"], truth), 0.5)
        self.assertLess(result["fit"]["residual_rms_rad_s"], 0.05)
        sphere = rotation_from_quaternion(result["q_lidar_sphere_wxyz"])
        self.assertLess(angle_between(sphere, matmul(result["R_lidar_gpmf"], GPMF_TO_SPHERE)), 1e-6)
        self.assertLess(angle_between(sphere, matmul(truth, GPMF_TO_SPHERE)), 0.5)

    def test_identity_mount_gives_forward_sphere(self):
        # Measured rig: GPMF stored axes (cam Z, X, Y) lie along LiDAR +Z, +Y and -X.
        lidar_from_gpmf = tuple(zip(*((0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0))))
        sphere = matmul(lidar_from_gpmf, GPMF_TO_SPHERE)
        forward = lidar_sphere((0, 0, 0))
        self.assertTrue(all(abs(a - b) < 1e-12 for a, b in
                            zip(tuple(row[2] for row in sphere),
                                rotate(forward.quaternion_wxyz, (0, 0, 1)))))
        self.assertTrue(all(abs(a - b) < 1e-12 for a, b in
                            zip(tuple(row[1] for row in sphere),
                                rotate(forward.quaternion_wxyz, (0, 1, 0)))))

    def test_rejects_reflection_and_too_little_motion(self):
        mirror = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0))
        camera_times, camera, lidar_times, lidar = streams(mirror, 0, seconds=8)
        with self.assertRaisesRegex(ValueError, "reflection"):
            calibrate(camera_times, camera, lidar_times, lidar, guess_ns=0)
        still = [(0.001, -0.002, 0.0)] * len(camera)
        quiet = [(0.0, 0.001, 0.002)] * len(lidar)
        with self.assertRaisesRegex(ValueError, "Too little rotation"):
            calibrate(camera_times, still, lidar_times, quiet, guess_ns=0)


if __name__ == "__main__":
    unittest.main()
