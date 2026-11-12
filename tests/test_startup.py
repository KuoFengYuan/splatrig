# SPDX-License-Identifier: MIT
import json
from pathlib import Path
import socket
import tempfile
import unittest

import subprocess
import sys

from splatrig.startup import (DRIVER_CONFIG, DRIVER_LAUNCH, Driver, checks, driver_running,
                              format_checks, interface_addresses, livox_addresses, ptp_masters)

IP_OUTPUT = """1: lo    inet 127.0.0.1/8 scope host lo\\       valid_lft forever
3: enx00aa    inet 192.168.1.50/24 brd 192.168.1.255 scope global noprefixroute enx00aa
7: enx0457    inet 172.24.132.55/24 brd 172.24.132.255 scope global dynamic enx0457
"""
PTP = (101, ["/usr/sbin/ptp4l", "-i", "enx00aa", "-S", "-m", "--ptp_minor_version", "0"])
SUDO = (100, ["sudo", "ptp4l", "-i", "enx00aa", "-S", "-m", "--ptp_minor_version", "0"])
DRIVER = (200, ["/ws/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node",
                "--ros-args"])


def workspace(root: Path, glim=True):
    share = root / "share"
    (share / "livox_ros_driver2/config").mkdir(parents=True)
    (share / "livox_ros_driver2/launch_ROS2").mkdir(parents=True)
    (share / "livox_ros_driver2/launch_ROS2" / DRIVER_LAUNCH).write_text("")
    (share / "livox_ros_driver2/config" / DRIVER_CONFIG).write_text(json.dumps({
        "lidar_summary_info": {"lidar_type": 8},
        "Mid360s": {"host_net_info": [{"host_ip": "192.168.1.50"}]},
        "lidar_configs": [{"ip": "192.168.1.102"}]}))
    if glim:
        (share / "glim/config").mkdir(parents=True)
        (share / "glim/config/config.json").write_text("{}")
    return {"AMENT_PREFIX_PATH": str(root)}


def levels(results):
    return {c.name: c.level for c in results}


class ParseTests(unittest.TestCase):
    def test_addresses_processes(self):
        with tempfile.TemporaryDirectory() as temporary:
            workspace(Path(temporary))
            config = Path(temporary) / "share/livox_ros_driver2/config" / DRIVER_CONFIG
            self.assertEqual(livox_addresses(config), (["192.168.1.50"], ["192.168.1.102"]))
        self.assertEqual(interface_addresses(IP_OUTPUT)["192.168.1.50"], "enx00aa")
        self.assertEqual(ptp_masters([SUDO, PTP, DRIVER]),
                         [{"pid": 101, "nic": "enx00aa", "minor": "0", "software": True}])
        self.assertTrue(driver_running([PTP, DRIVER]))
        self.assertFalse(driver_running([PTP, (5, ["ros2", "launch", "livox_ros_driver2"])]))


class CheckTests(unittest.TestCase):
    def run_checks(self, procs, ip_output=IP_OUTPUT, glim=True, ping=lambda ip: True, **options):
        with tempfile.TemporaryDirectory() as temporary:
            environ = workspace(Path(temporary), glim)
            return checks(environ=environ, procs=procs, ip_output=ip_output, ping=ping,
                          ros_python=True, port=self.port, **options)

    def setUp(self):
        with socket.socket() as probe:  # a port that is free now
            probe.bind(("127.0.0.1", 0))
            self.port = probe.getsockname()[1]

    def test_ready_machine(self):
        results = self.run_checks([SUDO, PTP, DRIVER])
        self.assertTrue(all(c.level == "ok" for c in results), format_checks(results))
        found = {c.name: c.detail for c in results}
        self.assertIn("172.24.132.51", found["MAX2"])
        self.assertIn("reused", found["Livox driver process"])

    def test_missing_pieces_name_their_fix(self):
        results = self.run_checks([], ip_output="1: lo    inet 127.0.0.1/8 scope host lo\n",
                                  glim=False)
        state = levels(results)
        self.assertEqual((state["GLIM"], state["LiDAR network"], state["MAX2"]),
                         ("fail", "fail", "warn"))
        self.assertNotIn("PTP time sync", state)  # needs the NIC first
        self.assertIn("nmcli connection up", format_checks(results))
        self.assertEqual(levels(self.run_checks([], glim=False, slam=False)).get("GLIM"), None)

    def test_ptp_states(self):
        nic = "sudo tools/install-ptp-service enx00aa"
        none = self.run_checks([DRIVER])
        self.assertEqual(levels(none)["PTP time sync"], "warn")
        self.assertIn(nic, format_checks(none))
        two = self.run_checks([PTP, (102, PTP[1])])
        self.assertEqual(levels(two)["PTP time sync"], "fail")
        v21 = self.run_checks([(101, PTP[1][:-2])])
        self.assertEqual(levels(v21)["PTP time sync"], "warn")
        other = self.run_checks([(101, ["ptp4l", "-i", "eno1", "-S"])])
        self.assertEqual(levels(other)["PTP time sync"], "fail")
        silent = self.run_checks([PTP], ping=lambda ip: False)
        self.assertEqual(levels(silent)["LiDAR"], "warn")

    def test_busy_port_and_missing_ros(self):
        with socket.socket() as busy:
            busy.bind(("127.0.0.1", self.port))
            busy.listen()
            self.assertEqual(levels(self.run_checks([PTP]))["Console port"], "fail")
        no_ros = checks(environ={}, procs=[], ip_output="", ros_python=False)
        self.assertEqual([(c.name, c.level) for c in no_ros], [("ROS 2", "fail")])
        self.assertIn("tools/splatrig-ros", format_checks(no_ros))


class DriverTests(unittest.TestCase):
    def test_stop_ends_the_process_group_and_removes_the_log(self):
        with tempfile.TemporaryDirectory() as temporary:
            # A launcher with a child, like ros2 launch and the driver node.
            script = ("import subprocess, sys, time; subprocess.Popen([sys.executable, '-c', "
                      "'import time; time.sleep(60)']); print('up', flush=True); time.sleep(60)")
            driver = Driver(log_dir=Path(temporary), command=[sys.executable, "-c", script])
            for _ in range(50):
                if driver.log_path.read_text():
                    break
                subprocess.run([sys.executable, "-c", "import time; time.sleep(0.1)"])
            self.assertTrue(driver.running())
            group = driver.process.pid
            driver.stop()
            self.assertFalse(driver.running())
            self.assertFalse(driver.log_path.exists())
            with self.assertRaises(ProcessLookupError):
                import os
                os.killpg(group, 0)


if __name__ == "__main__":
    unittest.main()
