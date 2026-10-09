#!/usr/bin/env python3
"""Regression tests for the Kerbin orbit scenario generator and its sim-frame consistency."""
import json
import math
import os
import pathlib
import subprocess
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import orbit_scenarios as orb  # noqa: E402

INCS = (0, 15, 30, 45, 60, 75, 90, 120, 150, 180)
ORBITS = ((86, 86), (80, 100), (80, 130), (80, 160))


class OrbitGenerator(unittest.TestCase):
    def test_elements_roundtrip_across_matrix(self):
        for inc in INCS:
            for rp, ra in ORBITS:
                for branch in ("asc", "desc"):
                    _, geo = orb.generate(inc, rp, ra, 37.0, 1300.0, 0.0, branch, 54321.0)
                    self.assertTrue(geo["validation"]["ok"], (inc, rp, ra, branch, geo["validation"]))

    def test_plane_contains_site_at_pass_time(self):
        # Propagate analytically from the generated epoch state to UT_pass: the
        # vehicle must be over the rotating-body runway threshold, on the
        # requested branch (northbound for asc, southbound for desc).
        for inc in (30, 60, 90, 120, 150):
            for branch in ("asc", "desc"):
                sc, geo = orb.generate(inc, 80, 130, 120.0, 1500.0, 0.0, branch, 99999.0)
                el = geo["elements_from_state"]
                M0 = orb.mean_from_true(math.radians(geo["nu0_deg"]), el["e"])
                nu = orb.true_from_mean(M0 + math.sqrt(orb.MU / el["a_m"] ** 3) * 1500.0, el["e"])
                pos, vel = orb.elements_to_state(el["a_m"], el["e"], math.radians(inc),
                                                 math.radians(geo["raan_deg"]), math.radians(120.0), nu)
                site = orb.site_inertial(geo["ut_pass"])
                ang = math.acos(max(-1, min(1, orb.dot(orb.unit(pos), orb.unit(site)))))
                self.assertLess(math.degrees(ang), 1e-5, (inc, branch))
                northbound = vel[2] > 0
                self.assertEqual(northbound, branch == "asc", (inc, branch))

    def test_cross_range_offset_is_honoured(self):
        for cr in (0.5, 2.0, -1.5):
            _, geo = orb.generate(60, 86, 86, 0.0, 1250.0, cr, "desc", 10000.0)
            self.assertAlmostEqual(geo["achieved_pass_crossrange_deg"], cr, places=6)

    @unittest.skipUnless((ROOT / "build" / "shuttlesim").exists(), "shuttlesim not built")
    def test_simulator_frame_matches_generator(self):
        """ShuttleSim's own rotating frame carries the vehicle over the runway at UT_pass."""
        sc, geo = orb.generate(45, 80, 130, 40.0, 1250.0, 0.0, "desc", 77777.0)
        with tempfile.TemporaryDirectory() as d:
            ini = pathlib.Path(d) / "case.ini"
            orb.write_ini(ini, "case", sc)
            p = subprocess.run([str(ROOT / "build" / "shuttlesim"), "--scenario", str(ini),
                                "--atmosphere", str(ROOT / "reference-model/kerbin_atmosphere_identified.csv"),
                                "--aero", str(ROOT / "reference-model/sts_aero_provisional.csv"),
                                "--attitude", str(ROOT / "reference-model/stsn_attitude_identified.ini"),
                                "--rate", "max", "--telemetry-hz", "1", "--max-sim-time", "1251",
                                "--command-port", "0", "--telemetry-port", "0", "--web-telemetry-port", "0",
                                "--quiet"], capture_output=True, text=True, check=True)
        summary = [l for l in p.stderr.splitlines() if l.startswith("SHUTTLESIM_SUMMARY")][-1]
        s = json.loads(summary.partition(" ")[2])
        # Final sample is at UT_pass + ~1 s; allow the 1 s of motion plus integration error.
        self.assertLess(math.hypot(s["final_along_m"], s["final_cross_m"]), 6000.0, s)


if __name__ == "__main__":
    unittest.main()
