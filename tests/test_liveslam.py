# SPDX-License-Identifier: MIT
import json
from pathlib import Path
import os
import struct
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest

from splatrig.geometry import Pose, rotate
from splatrig.live import camera_mount
from splatrig.liveslam import (LIVE_PREFIX, MAP_POINT, OWNER_FILE, SlamState, cloud_points,
                               reap_orphans, voxel_key)


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
        scan = [(0.01, 0, 0, 1), (0.02, 0, 0, 2), (0.15, 0, 0, 3), (0, 0.5, 0, 4),
                (0, 0, 0.9, 5)]
        state.add_scan(scan, received=0)
        self.assertEqual(state.status(now=0)["map_voxels"], 0)  # not yet confirmed
        state.add_scan(scan, received=1)
        status = state.status(now=1)
        self.assertEqual((status["map_voxels"], status["map_dropped"]), (3, 1))
        payload = state.map_payload()
        self.assertEqual(struct.unpack_from("<I", payload)[0], 3)
        first = list(MAP_POINT.iter_unpack(payload[4:]))[0]  # float32 storage
        self.assertTrue(all(abs(a - b) < 1e-6 for a, b in zip(first, (0.01, 0, 0, 1))))
        self.assertEqual(struct.unpack_from("<I", state.map_payload(max_points=2))[0], 2)
        scan = state.scan_payload()  # every point of the newest scan, budget or not
        self.assertEqual(struct.unpack_from("<I", scan)[0], 5)
        self.assertEqual(len(scan), 4 + 5 * MAP_POINT.size)
        for i, x in enumerate([0, 0.01, 0.05, 0.051, 0.2]):
            state.add_odometry((x, 0, 0), (1, 0, 0, 0), received=10 + i * 0.1)
        route = state.route_payload()
        self.assertEqual(struct.unpack_from("<I", route)[0], 3)  # 2 cm spacing
        status = state.status(now=10.45)
        self.assertAlmostEqual(status["odom_hz"], 10.0)
        self.assertEqual(status["pose"]["translation"], [0.2, 0.0, 0.0])
        self.assertNotIn("camera", status)
        self.assertIsNone(status["odom_publishers"])
        self.assertEqual(state.pose_status(now=10.5), {"odom_age_s": 0.1, "pose": status["pose"]})

    def test_transient_and_near_returns_stay_out_of_the_map(self):
        state = SlamState(voxel=0.1, near_m=0.6, confirm_s=0.5, pending_s=10)
        state.add_odometry((10, 0, 0), (1, 0, 0, 0), received=0)
        wall, walker, operator = (13, 0, 0, 1), (11.5, 0, 0, 2), (10.3, 0, 0, 3)
        state.add_scan([wall, walker, operator], received=0.0)
        state.add_scan([wall, walker, operator], received=0.3)  # walker still there
        state.add_scan([wall, operator], received=0.6)  # walker gone after 0.3 s
        self.assertEqual(state.status(now=0.6)["map_voxels"], 1)
        payload = state.map_payload()
        self.assertEqual(list(MAP_POINT.iter_unpack(payload[4:])), [wall])
        # The newest-scan view still shows everything the sensor returned.
        self.assertEqual(struct.unpack_from("<I", state.scan_payload())[0], 2)
        self.assertEqual(state.status(now=0.6)["map_pending"], 1)  # the walker
        state.add_scan([wall], received=11)  # the walker's voxel expires unconfirmed
        state.add_scan([walker], received=11.2)
        state.add_scan([walker], received=11.4)
        self.assertEqual(state.status(now=11.4)["map_voxels"], 1)
        # A bounded pending table drops its oldest entries first.
        small = SlamState(voxel=0.1, max_pending=2)
        small.add_scan([(1, 0, 0, 0), (2, 0, 0, 0), (3, 0, 0, 0)], received=0)
        self.assertEqual(list(small.pending), [voxel_key(2, 0, 0, 0.1), voxel_key(3, 0, 0, 0.1)])

    def test_voxel_keys(self):
        self.assertEqual(voxel_key(0.01, 0.02, 0.03, 0.05), voxel_key(0.04, 0.0, 0.049, 0.05))
        keys = {voxel_key(x, y, z, 0.05) for x in (-0.01, 0.01) for y in (-0.01, 0.01)
                for z in (-0.01, 0.01)}
        self.assertEqual(len(keys), 8)
        self.assertNotEqual(voxel_key(-50000, 0, 0, 0.05), voxel_key(50000, 0, 0, 0.05))

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


class OrphanTests(unittest.TestCase):
    def spawn(self, folder):
        # Same shape as a live GLIM: its own session, config path on the command line.
        return subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)",
                                 str(folder / "config")], start_new_session=True)

    def test_stops_glim_of_dead_owner_only(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dead = subprocess.Popen([sys.executable, "-c", "pass"])
            dead.wait()
            orphan, owned = root / (LIVE_PREFIX + "orphan"), root / (LIVE_PREFIX + "owned")
            for folder, owner in ((orphan, dead.pid), (owned, os.getpid())):
                folder.mkdir()
                (folder / OWNER_FILE).write_text(str(owner))
            (root / (LIVE_PREFIX + "legacy")).mkdir()  # no owner file: never touched
            left, kept = self.spawn(orphan), self.spawn(owned)
            # Reap the orphan as its real parent would; this test's child stays a zombie otherwise.
            threading.Thread(target=left.wait, daemon=True).start()
            try:
                started = time.monotonic()
                self.assertEqual(reap_orphans(root, timeout=5), [left.pid])
                self.assertLess(time.monotonic() - started, 4)
                self.assertIsNotNone(left.wait(5))
                self.assertIsNone(kept.poll())
                self.assertFalse(orphan.exists())
                self.assertTrue(owned.exists() and (root / (LIVE_PREFIX + "legacy")).exists())
            finally:
                for process in (left, kept):
                    process.kill()
                    process.wait()


if __name__ == "__main__":
    unittest.main()


class NewScanTests(unittest.TestCase):
    def test_reset_and_glim_restart(self):
        import sys
        state = SlamState(voxel=0.1)
        state.add_odometry((1, 2, 3), (1, 0, 0, 0), received=0)
        state.add_scan([(5, 0, 0, 1)], received=0)
        state.add_scan([(5, 0, 0, 1)], received=1)
        self.assertEqual(state.status(now=1)["map_voxels"], 1)
        state.reset()
        status = state.status(now=1)
        self.assertEqual((status["map_voxels"], status["scans"], status["pose"]), (0, 0, None))
        self.assertEqual(struct.unpack_from("<I", state.route_payload())[0], 0)
        from unittest import mock
        from splatrig.liveslam import GlimProcess
        # No GLIM installation is needed: its configuration writer is replaced.
        with tempfile.TemporaryDirectory() as temporary, \
                mock.patch("splatrig.glim.write_glim_config", lambda folder, **_: folder.mkdir()):
            os.environ["TMPDIR"] = temporary
            tempfile.tempdir = None
            try:
                glim = GlimProcess(command=[sys.executable, "-c", "import time; time.sleep(60)"])
                first, folder = glim.process.pid, glim.folder
                glim.restart()
                self.assertNotEqual(glim.process.pid, first)
                self.assertFalse(folder.exists())
                self.assertTrue(glim.running())
                self.assertEqual(glim.restarts, 1)
                glim.stop()
                self.assertFalse(glim.running())
            finally:
                os.environ.pop("TMPDIR", None)
                tempfile.tempdir = None
