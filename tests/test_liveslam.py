# SPDX-License-Identifier: MIT
import json
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace as NS
import unittest

from splatrig.geometry import Pose, rotate
from splatrig.live import camera_mount
from splatrig.liveslam import MAP_POINT, SlamState, cloud_points


def field(name, offset):
    return NS(name=name, offset=offset, datatype=7)


class CloudPointTests(unittest.TestCase):
    def test_fast_and_generic_layouts(self):
        fast = NS(fields=[field("x", 0), field("y", 4), field("z", 8), field("intensity", 12)],
                  point_step=16, data=MAP_POINT.pack(1, 2, 3, 4) + MAP_POINT.pack(5, 6, 7, 8))
        self.assertEqual(list(cloud_points(fast)), [(1, 2, 3, 4), (5, 6, 7, 8)])
        generic = NS(fields=[field("intensity", 0), field("x", 4), field("y", 8), field("z", 12)],
                     point_step=20, data=struct.pack("<fffff", 9, 1, 2, 3, 0) * 2)
        self.assertEqual(list(cloud_points(generic)), [(1, 2, 3, 9)] * 2)
        no_intensity = NS(fields=[field("x", 0), field("y", 4), field("z", 8)], point_step=12,
                          data=struct.pack("<fff", 1, 2, 3))
        self.assertEqual(list(cloud_points(no_intensity)), [(1, 2, 3, 0.0)])
        with self.assertRaises(ValueError):
            list(cloud_points(NS(fields=[field("x", 0)], point_step=4, data=b"")))


class SlamStateTests(unittest.TestCase):
    def test_voxels_budget_route_and_payloads(self):
        state = SlamState(voxel=0.1, max_voxels=3)
        state.add_scan([(0.01, 0, 0, 1), (0.02, 0, 0, 2), (0.15, 0, 0, 3), (0, 0.5, 0, 4),
                        (0, 0, 0.9, 5)])
        status = state.status(now=0)
        self.assertEqual((status["map_voxels"], status["map_dropped"]), (3, 1))
        payload = state.map_payload()
        self.assertEqual(struct.unpack_from("<I", payload)[0], 3)
        first = list(MAP_POINT.iter_unpack(payload[4:]))[0]  # float32 storage
        self.assertTrue(all(abs(a - b) < 1e-6 for a, b in zip(first, (0.01, 0, 0, 1))))
        self.assertEqual(struct.unpack_from("<I", state.map_payload(max_points=2))[0], 2)
        for i, x in enumerate([0, 0.01, 0.05, 0.051, 0.2]):
            state.add_odometry((x, 0, 0), (1, 0, 0, 0), received=10 + i * 0.1)
        route = state.route_payload()
        self.assertEqual(struct.unpack_from("<I", route)[0], 3)  # 2 cm spacing
        status = state.status(now=10.45)
        self.assertAlmostEqual(status["odom_hz"], 10.0)
        self.assertEqual(status["pose"]["translation"], [0.2, 0.0, 0.0])
        self.assertNotIn("camera", status)

    def test_camera_pose_from_calibration(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "calibration.json"
            path.write_text(json.dumps({"q_lidar_sphere_wxyz": [0.5, -0.5, 0.5, -0.5]}))
            mount = camera_mount(path, (0, 0, 0.083))
        state = SlamState(camera=mount)
        state.add_odometry((1, 2, 0), (1, 0, 0, 0), received=0)
        camera = Pose.from_dict(state.status(now=0)["camera"])
        self.assertTrue(all(abs(a - b) < 1e-9 for a, b in
                            zip(rotate(camera.quaternion_wxyz, (0, 0, 1)), (1, 0, 0))))
        # rig = IMU; camera center = IMU pose * T_imu_lidar * offset
        expected = (1 - 0.011, 2 - 0.02329, 0.04412 + 0.083)
        self.assertTrue(all(abs(a - b) < 1e-9 for a, b in zip(camera.translation, expected)))
        self.assertIsNone(camera_mount(None))


if __name__ == "__main__":
    unittest.main()
