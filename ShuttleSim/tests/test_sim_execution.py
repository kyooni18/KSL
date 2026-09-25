#!/usr/bin/env python3
"""Exercise simulator process boundaries using private endpoints, never live KSP."""
from __future__ import annotations

import os
import json
import tempfile
import pathlib
import socket
import subprocess
import unittest

BINARY = os.environ.get("SHUTTLESIM_BINARY")
ROOT = pathlib.Path(__file__).resolve().parents[2]


@unittest.skipUnless(BINARY, "SHUTTLESIM_BINARY is supplied by CTest")
class SimulatorExecutionTests(unittest.TestCase):
    def run_simulator(self, *arguments: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [BINARY, "--command-port", "0", "--telemetry-port", "0",
             "--web-telemetry-port", "0", "--max-sim-time", "0.1", "--quiet", *arguments],
            text=True, capture_output=True, timeout=5, cwd=ROOT,
        )

    def test_explicit_offline_execution_is_supported(self):
        result = self.run_simulator()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("SHUTTLESIM_SUMMARY", result.stderr)

    def test_terminal_frame_is_recorded_between_publish_intervals(self):
        with tempfile.TemporaryDirectory() as directory:
            path=pathlib.Path(directory)/"telemetry.jsonl"
            result=self.run_simulator("--max-sim-time","0.11","--record",str(path))
            self.assertEqual(result.returncode,0,result.stderr)
            summary=json.loads(next(line.partition(" ")[2] for line in result.stderr.splitlines()
                                    if line.startswith("SHUTTLESIM_SUMMARY ")))
            samples=[json.loads(line) for line in path.read_text().splitlines()]
            self.assertAlmostEqual(samples[-1]["sim_time"],summary["sim_time_s"],places=6)
            self.assertGreater(samples[-1]["sim_time"],0.1)

    def test_occupied_command_endpoint_fails_without_stealing_it(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as owner:
            owner.bind(("127.0.0.1", 0))
            port = owner.getsockname()[1]
            result = self.run_simulator("--command-port", str(port))
            self.assertEqual(result.returncode, 3, result.stderr)
            self.assertIn("Failed to open requested UDP endpoints", result.stderr)
            self.assertEqual(owner.getsockname()[1], port)

    def test_nonfinite_and_partially_parsed_numbers_are_rejected(self):
        for option in ("--dt", "--rate", "--max-sim-time", "--telemetry-hz", "--fixed-aoa", "--fixed-bank"):
            for value in ("nan", "inf", "0.1garbage"):
                with self.subTest(option=option, value=value):
                    result = self.run_simulator(option, value)
                    self.assertEqual(result.returncode, 2, result.stderr)

    def test_invalid_ports_are_rejected_before_launch(self):
        for option in ("--command-port", "--telemetry-port", "--web-telemetry-port"):
            for value in ("-1", "65536", "garbage", "1.5", "999999999999999999999999"):
                with self.subTest(option=option, value=value):
                    self.assertEqual(self.run_simulator(option, value).returncode, 2)

    def test_scenario_errors_fail_instead_of_silently_changing_the_input(self):
        with tempfile.TemporaryDirectory() as directory:
            path=pathlib.Path(directory)/"scenario.ini"
            for content in ("mass_kg=nan", "mass_kg=0", "mass_kg=10junk",
                            "position_x_m=600000", "runway_width_m=-1",
                            "latitude_deg=91", "typo_heading=180", "missing separator"):
                with self.subTest(content=content):
                    path.write_text(content+"\n")
                    result=self.run_simulator("--scenario",str(path))
                    self.assertEqual(result.returncode,3,result.stderr)

    def test_explicit_heading_is_preserved_and_omitted_heading_is_derived(self):
        with tempfile.TemporaryDirectory() as directory:
            root=pathlib.Path(directory);scenario=root/"scenario.ini";record=root/"telemetry.jsonl"
            base=("latitude_deg=0\nlongitude_deg=0\naltitude_m=5000\nsurface_speed_mps=150\n"
                  "flight_path_angle_deg=-5\ndeorbit_delta_v_mps=0\nrunway_latitude_deg=0\nrunway_longitude_deg=1\n")
            for heading in (0,90,180,270,None):
                with self.subTest(heading=heading):
                    scenario.write_text(base+(f"heading_deg={heading}\n" if heading is not None else ""))
                    result=self.run_simulator("--scenario",str(scenario),"--record",str(record))
                    self.assertEqual(result.returncode,0,result.stderr)
                    first=json.loads(record.read_text().splitlines()[0])
                    actual=first["attitude"]["heading_deg"]
                    expected=90 if heading is None else heading
                    error=(actual-expected+180)%360-180
                    self.assertAlmostEqual(error,0,places=3)

    def test_existing_scenario_library_is_still_accepted(self):
        scenarios=list((ROOT/"ShuttleSim"/"scenarios").rglob("*.ini"))
        self.assertTrue(scenarios)
        for path in scenarios:
            with self.subTest(scenario=path.name):
                result=self.run_simulator("--scenario",str(path))
                self.assertEqual(result.returncode,0,result.stderr)

    def test_interactive_execution_requires_command_input(self):
        for option in ("--lockstep", "--start-paused"):
            with self.subTest(option=option):
                result = self.run_simulator(option)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn("command endpoint", result.stderr)


if __name__ == "__main__":
    unittest.main()
