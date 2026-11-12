# SPDX-License-Identifier: MIT
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

from splatrig.glim import TIMED_POINT, import_trajectory, pack_timed_points, read_tum, \
    seconds_to_ns, write_glim_config
from splatrig.session import read_json, read_jsonl


class GlimTests(unittest.TestCase):
    def test_pack_timed_points(self):
        points = [NS(x=1.0, y=2.0, z=3.0, reflectivity=9, offset_time=99_829_000),
                  NS(x=0.0, y=0.0, z=0.0, reflectivity=0, offset_time=5),
                  NS(x=-1.0, y=0.5, z=0.0, reflectivity=0, offset_time=0)]
        data, count = pack_timed_points(points)
        self.assertEqual(count, 2)
        first, second = TIMED_POINT.iter_unpack(data)
        self.assertEqual(first[:4], (1.0, 2.0, 3.0, 9.0))
        self.assertAlmostEqual(first[4], 0.099829, places=7)
        self.assertEqual(second, (-1.0, 0.5, 0.0, 0.0, 0.0))

    def test_seconds_to_ns_is_exact(self):
        self.assertEqual(seconds_to_ns("1794382198.001229189"), 1_794_382_198_001_229_189)
        self.assertEqual(seconds_to_ns("1794382198.5"), 1_794_382_198_500_000_000)
        for bad in ["-1", "nan", "inf", "abc"]:
            with self.assertRaises(ValueError):
                seconds_to_ns(bad)

    def test_tum_import(self):
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            tum = base / "glim" / "traj_imu.txt"
            tum.parent.mkdir()
            tum.write_text("# t x y z qx qy qz qw\n"
                           "1794382198.000000001 0 0 0 0 0 0 1\n"
                           "1794382198.100000001 1 2 3 0 0 0.7071067811865476 0.7071067811865476\n")
            records = list(read_tum(tum, 3))
            self.assertEqual(records[0]["timestamp_ns"], 1_794_382_198_000_000_001)
            self.assertEqual(records[1]["quaternion_wxyz"][0], records[1]["quaternion_wxyz"][3])
            self.assertEqual(records[1]["translation"], [1.0, 2.0, 3.0])
            report = import_trajectory(tum, base / "out", revision=3, status="final", frame="imu")
            self.assertEqual(report["poses"], 2)
            self.assertEqual(list(read_jsonl(base / "out/trajectory.jsonl")), records)
            self.assertEqual(read_json(base / "out/trajectory-source.json")["tracked_frame"], "imu")
            with self.assertRaisesRegex(ValueError, "outside"):
                import_trajectory(tum, tum.parent / "nested", revision=3, status="final",
                                  frame="imu")
            tum.write_text("1.0 0 0 0 0 0 0 1\n1.0 0 0 0 0 0 0 1\n")
            with self.assertRaisesRegex(ValueError, "increase"):
                list(read_tum(tum, 1))
            tum.write_text("1.0 0 0 0 0 0 1\n")
            with self.assertRaisesRegex(ValueError, "8 TUM columns"):
                list(read_tum(tum, 1))


    def test_glim_config_edits_and_refuses_unknown_layout(self):
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            source = base / "installed"
            source.mkdir()
            (source / "config.json").write_text(
                '{"global": { // comment\n "config_odometry": "config_odometry_gpu.json",\n'
                ' "config_sub_mapping": "config_sub_mapping_gpu.json",\n'
                ' "config_global_mapping": "config_global_mapping_gpu.json"}}')
            (source / "config_ros.json").write_text(
                '{"glim_ros": {"acc_scale": 0.0, // auto\n "extension_modules": ["a.so",\n "b.so"],'
                ' "imu_topic": "/os/imu", "points_topic": "/os/points", "publish_imu2lidar": true}}')
            (source / "config_sensors.json").write_text('{"sensors": {"T_lidar_imu": [1, 2,\n 3, 0, 0, 0, 1]}}')
            write_glim_config(base / "out", source=source)
            out = base / "out"
            self.assertIn('"config_odometry": "config_odometry_cpu.json"', (out / "config.json").read_text())
            ros = (out / "config_ros.json").read_text()
            for text in ['"acc_scale": 9.80665,', '"imu_topic": "/livox/imu"',
                         '"points_topic": "/livox/points"', '"extension_modules": ["libmemory_monitor.so"]',
                         '"publish_imu2lidar": false']:
                self.assertIn(text, ros)
            self.assertIn('"T_lidar_imu": [0.011, 0.02329, -0.04412, 0, 0, 0, 1]',
                          (out / "config_sensors.json").read_text())
            self.assertEqual((source / "config_ros.json").read_text().count("/os/imu"), 1)
            write_glim_config(base / "gpu", source=source, gpu=True)
            self.assertIn("config_global_mapping_gpu.json", (base / "gpu/config.json").read_text())
            write_glim_config(base / "live", source=source, publish=True)
            self.assertIn('"libmemory_monitor.so", "librviz_viewer.so"]',
                          (base / "live/config_ros.json").read_text())
            (source / "config_sensors.json").write_text('{"sensors": {}}')
            with self.assertRaisesRegex(ValueError, "setting not found"):
                write_glim_config(base / "bad", source=source)
            self.assertFalse((base / "bad").exists())


if __name__ == "__main__":
    unittest.main()
