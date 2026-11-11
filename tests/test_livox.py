# SPDX-License-Identifier: MIT
from collections import Counter
from contextlib import redirect_stderr
import io
import json
from pathlib import Path
import random
import tempfile
from types import SimpleNamespace as NS
import unittest

from splatrig.cli import main
from splatrig.livox import assess_clock, concentration, convert
from splatrig.pointlog import PointLogWriter, iter_points, read_header
from splatrig.session import prepare, read_json, read_jsonl
from test_pipeline import fixture

INTERVAL = 4947        # measured Mid-360S point spacing inside one packet (ns)
PER_PACKET = 96
PACKET_PERIOD = 480_000  # 95 * 4947 + 10035, the measured packet boundary step


def driver_frames(count, *, host_arrival=False, seed=3, packets=(10, 11), zero_every=7):
    """Frames built the way livox_ros_driver2 stamps points.

    Synced devices stamp each packet at a fixed spacing. Unsynced packets are
    stamped with host arrival time: device time plus variable network latency.
    """
    rng = random.Random(seed)
    base_time, packet_index = 1_794_382_198_000_000_000, 0
    for frame in range(count):
        points, base = [], None
        for _ in range(packets[frame % len(packets)]):
            stamp = base_time + packet_index * PACKET_PERIOD
            if host_arrival:
                stamp += 300_000 + int(rng.expovariate(1 / 60_000))
            packet_index += 1
            for i in range(PER_PACKET):
                time = stamp + i * INTERVAL
                base = time if base is None else base
                index = len(points)
                xyz = (0.0, 0.0, 0.0) if index % zero_every == 0 else \
                    (1.0 + index * 1e-3, -2.0, 0.5)
                points.append(NS(offset_time=(time - base) % 2**32, x=xyz[0], y=xyz[1],
                                 z=xyz[2], reflectivity=index % 200, tag=0, line=index % 4))
        yield NS(timebase=base, points=points)


def imu_messages(count, start=1_794_382_198_000_000_000, accel=(0.02, 0.05, 0.99)):
    for index in range(count):
        time = start + index * 5_000_000
        yield NS(header=NS(stamp=NS(sec=time // 10**9, nanosec=time % 10**9)),
                 linear_acceleration=NS(x=accel[0], y=accel[1], z=accel[2]),
                 angular_velocity=NS(x=0.001, y=-0.002, z=0.0))


def evidence(frames):
    steps, frame_steps, backwards, previous = Counter(), [], 0, None
    for message in frames:
        offsets = [p.offset_time for p in message.points]
        steps.update(b - a for a, b in zip(offsets, offsets[1:]))
        backwards += any(b < a for a, b in zip(offsets, offsets[1:])) or max(offsets) >= 2**31
        if previous is not None:
            frame_steps.append(message.timebase - previous)
        previous = message.timebase
    return steps, frame_steps, backwards


def recording(frames, imu):
    messages = [("/livox/lidar", f.timebase + 2_000_000, f) for f in frames]
    messages += [("/livox/imu", stamp_ns + 900_000, m) for m in imu
                 for stamp_ns in [m.header.stamp.sec * 10**9 + m.header.stamp.nanosec]]
    return sorted(messages, key=lambda item: item[1])


class PointLogTests(unittest.TestCase):
    def test_round_trip_and_truncation(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "points.bin"
            with PointLogWriter(path, {"frame": "lidar"}) as writer:
                writer.write(1_794_382_198_000_000_001, 1.5, -2.25, 0.125, 200, 16, 3)
                writer.write(2**62, 0.0, 0.0, 1.0)
            header, _, count = read_header(path)
            self.assertEqual((header["frame"], count), ("lidar", 2))
            self.assertEqual(list(iter_points(path)),
                             [(1_794_382_198_000_000_001, 1.5, -2.25, 0.125, 200, 16, 3),
                              (2**62, 0.0, 0.0, 1.0, 0, 0, 0)])
            with self.assertRaises(FileExistsError):
                PointLogWriter(path, {})
            with path.open("ab") as stream:
                stream.write(b"\0" * 5)
            with self.assertRaisesRegex(ValueError, "truncated"):
                read_header(path)
            path.write_bytes(b"NOTPOINTS" + b"\0" * 20)
            with self.assertRaisesRegex(ValueError, "not a SplatRig"):
                read_header(path)


class ClockAssessmentTests(unittest.TestCase):
    def test_concentration_windows(self):
        counts = Counter({100: 5, 105: 5, 500: 8, 900: 2})
        self.assertEqual(concentration(counts, 10), 0.5)
        self.assertEqual(concentration(counts, 10, windows=2), 0.9)
        self.assertEqual(concentration(Counter(), 10), 0.0)

    def test_synced_device_stamps(self):
        result = assess_clock(*evidence(driver_frames(30)))
        self.assertEqual(result["status"], "ptp_inferred")
        self.assertEqual(result["point_interval_ns"], INTERVAL)
        self.assertGreater(result["boundary_concentration"], 0.99)
        self.assertGreater(result["frame_step_concentration"], 0.99)

    def test_host_arrival_stamps_are_rejected(self):
        for seed in range(5):
            result = assess_clock(*evidence(driver_frames(30, host_arrival=True, seed=seed)))
            self.assertEqual(result["status"], "host_arrival_suspected", result)
            self.assertLess(result["boundary_concentration"], 0.5)

    def test_backwards_offsets_and_short_recordings(self):
        steps, frame_steps, _ = evidence(driver_frames(30))
        self.assertEqual(assess_clock(steps, frame_steps, 1)["status"],
                         "host_arrival_suspected")
        self.assertEqual(assess_clock(*evidence(driver_frames(5)))["status"], "indeterminate")


class ConversionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.base = Path(self.temporary.name)
        self.bag = self.base / "bag"
        self.bag.mkdir()
        (self.bag / "recording.mcap").write_bytes(b"original")

    def tearDown(self):
        self.temporary.cleanup()

    def test_synced_recording(self):
        frames = list(driver_frames(30))
        report = convert(recording(frames, list(imu_messages(400))), self.base / "out", self.bag)
        self.assertFalse(report["provisional"])
        self.assertEqual(report["clock"]["status"], "ptp_inferred")
        total = sum(len(f.points) for f in frames)
        zeros = sum(1 for f in frames for p in f.points if p.x == p.y == p.z == 0)
        self.assertEqual(report["lidar"]["no_return_excluded"], zeros)
        self.assertEqual(report["lidar"]["points_written"], total - zeros)
        self.assertEqual(report["lidar"]["frame_gaps"]["count"], 0)
        expected = [(f.timebase + p.offset_time, p.reflectivity, p.line)
                    for f in frames for p in f.points if (p.x, p.y, p.z) != (0, 0, 0)]
        written = [(t, r, line) for t, _, _, _, r, _, line in iter_points(self.base / "out/points.bin")]
        self.assertEqual(written, expected)
        imu = list(read_jsonl(self.base / "out/imu.jsonl"))
        self.assertEqual(len(imu), 400)
        self.assertEqual(report["imu"]["linear_acceleration_unit"], "g")
        self.assertEqual(report["imu"]["gaps"]["count"], 0)
        self.assertAlmostEqual(report["imu"]["rate_hz"], 200.0)
        self.assertAlmostEqual(report["imu"]["receive_latency_ms"]["p50"], 0.9)
        self.assertEqual(read_json(self.base / "out/recording.json"), report)
        self.assertEqual((self.bag / "recording.mcap").read_bytes(), b"original")

    def test_unsynced_recording_requires_explicit_provisional_output(self):
        frames = list(driver_frames(30, host_arrival=True))
        with self.assertRaisesRegex(ValueError, "not ptp_inferred"):
            convert(recording(frames, []), self.base / "out", self.bag)
        self.assertFalse((self.base / "out").exists())
        self.assertEqual([p.name for p in self.base.iterdir()], ["bag"])
        report = convert(recording(frames, []), self.base / "out", self.bag,
                         allow_unsynced=True)
        self.assertTrue(report["provisional"])
        self.assertEqual(report["clock"]["status"], "host_arrival_suspected")

    def test_reports_gaps_units_and_refuses_bad_outputs(self):
        frames = list(driver_frames(30))
        del frames[10:13]
        imu = list(imu_messages(100, accel=(0.2, 0.5, 9.79)))
        del imu[50:60]
        report = convert(recording(frames, imu), self.base / "out", self.bag)
        self.assertEqual(report["lidar"]["frame_gaps"]["count"], 1)
        self.assertEqual(report["imu"]["gaps"]["count"], 1)
        self.assertEqual(report["imu"]["linear_acceleration_unit"], "m/s^2")
        with self.assertRaisesRegex(ValueError, "already exists"):
            convert(recording(frames, imu), self.base / "out", self.bag)
        with self.assertRaisesRegex(ValueError, "outside"):
            convert(recording(frames, imu), self.bag / "nested", self.bag)
        with self.assertRaisesRegex(ValueError, "No LiDAR points"):
            convert(recording([], imu), self.base / "empty", self.bag)

    def test_cli_reports_missing_ros_or_bag(self):
        stderr = io.StringIO()
        with redirect_stderr(stderr):
            code = main(["import-livox", str(self.base / "missing"),
                         "--output", str(self.base / "out")])
        self.assertEqual(code, 2)
        self.assertIn("Bag not found", stderr.getvalue())


class BinarySessionTests(unittest.TestCase):
    def test_prepare_reads_binary_points_like_jsonl(self):
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            text, binary = base / "text", base / "binary"
            fixture(text)
            fixture(binary)
            samples = list(read_jsonl(binary / "points.jsonl"))
            (binary / "points.jsonl").unlink()
            with PointLogWriter(binary / "points.bin", {"frame": "lidar"}) as writer:
                for sample in samples:
                    writer.write(sample["timestamp_ns"], *sample["xyz"])
            a, b = prepare(text), prepare(binary)
            self.assertEqual(a.report["raw_points"], b.report["raw_points"])
            self.assertEqual([p[:3] for p in a.points], [p[:3] for p in b.points])
            (binary / "points.jsonl").write_text(json.dumps(samples[0]) + "\n")
            with self.assertRaisesRegex(ValueError, "not both"):
                prepare(binary)


if __name__ == "__main__":
    unittest.main()
