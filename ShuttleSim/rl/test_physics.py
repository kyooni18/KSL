"""Offline ABI tests only; no guidance, network, or landing-success claims.

Run: python3 ShuttleSim/rl/test_physics.py /absolute/path/to/shared/library
"""

import ctypes
import json
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
if len(sys.argv) > 1 and Path(sys.argv[1]).suffix in {".dylib", ".so"}:
    LIBRARY = Path(sys.argv.pop(1)).resolve()
else:
    # unittest discovery leaves its own command words in sys.argv.  Do not
    # mistake "discover" or a test selector for the required shared library.
    LIBRARY = None


@unittest.skipUnless(LIBRARY is not None,
                     "pass the built shuttlesim_offline library when running this module directly")
class PhysicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if LIBRARY is None:
            raise RuntimeError("Pass the built shuttlesim_offline library path")
        cls.lib = ctypes.CDLL(str(LIBRARY))
        cls.lib.offline_create.argtypes = [ctypes.c_char_p] * 4
        cls.lib.offline_create.restype = ctypes.c_void_p
        cls.lib.offline_destroy.argtypes = [ctypes.c_void_p]
        cls.lib.offline_destroy.restype = None
        cls.lib.offline_step.argtypes = [
            ctypes.c_void_p,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
        ]
        cls.lib.offline_step.restype = ctypes.c_int
        cls.lib.offline_set_initial_conditions.argtypes = [
            ctypes.c_void_p,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
        ]
        cls.lib.offline_set_initial_conditions.restype = ctypes.c_int
        cls.lib.offline_set_deorbit_orbit.argtypes = [
            ctypes.c_void_p,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
        ]
        cls.lib.offline_set_deorbit_orbit.restype = ctypes.c_int
        cls.lib.offline_runway_contains.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double]
        cls.lib.offline_runway_contains.restype = ctypes.c_int
        for name in ("offline_telemetry", "offline_summary"):
            function = getattr(cls.lib, name)
            function.argtypes = [ctypes.c_void_p]
            function.restype = ctypes.c_char_p

    def create(self):
        paths = (
            "scenarios/orbit86km.ini",
            "data/kerbin_atmosphere_seed.csv",
            "data/stsn_aero_seed.csv",
            "data/stsn_attitude_seed.ini",
        )
        handle = self.lib.offline_create(
            *(str(ROOT / path).encode() for path in paths)
        )
        self.assertTrue(handle)
        self.addCleanup(self.lib.offline_destroy, handle)
        return handle

    def test_recreated_instances_are_deterministic(self):
        first, second = self.create(), self.create()
        for _ in range(10):
            for handle in (first, second):
                self.assertEqual(
                    self.lib.offline_step(handle, 35.0, -20.0, 0, 0, 50), 1
                )
            self.assertEqual(
                self.lib.offline_telemetry(first),
                self.lib.offline_telemetry(second),
            )
        telemetry = json.loads(self.lib.offline_telemetry(first))
        self.assertAlmostEqual(telemetry["sim_time"], 10.0)
        summary = json.loads(self.lib.offline_summary(first))
        self.assertFalse(summary["touchdown"])

    def test_atmosphere_top_ignores_zero_density_table_padding(self):
        handle = self.create()
        telemetry = json.loads(self.lib.offline_telemetry(handle))
        rows = []
        with open("ShuttleSim/data/fitted/kerbin_atmosphere_ksp.csv") as stream:
            for line in stream:
                if not line or line.startswith("#") or "altitude" in line:
                    continue
                altitude, density, *_ = map(float, line.strip().split(","))
                rows.append((altitude, density))
        transition = next(rows[i][0] for i in range(1, len(rows))
                          if rows[i - 1][1] > 0.0 and not rows[i][1] > 0.0)
        self.assertEqual(telemetry["world"]["atmosphere_top_m"], transition)
        self.assertLess(transition, rows[-1][0])

    def test_runway_geometry_uses_threshold_origin_and_physical_width(self):
        handle = self.create()
        telemetry = json.loads(self.lib.offline_telemetry(handle))
        self.assertEqual(telemetry["runway"]["length_m"], 2500.0)
        self.assertEqual(telemetry["runway"]["width_m"], 70.0)
        self.assertEqual(self.lib.offline_runway_contains(handle, 0.0, 0.0), 1)
        self.assertEqual(self.lib.offline_runway_contains(handle, 2500.0, 35.0), 1)
        self.assertEqual(self.lib.offline_runway_contains(handle, -0.01, 0.0), 0)
        self.assertEqual(self.lib.offline_runway_contains(handle, 2500.01, 0.0), 0)
        self.assertEqual(self.lib.offline_runway_contains(handle, 1000.0, 35.01), 0)

    def test_braked_ground_state_reaches_explicit_stopped_state(self):
        handle = self.create()
        self.assertEqual(self.lib.offline_set_initial_conditions(
            handle, 70.0, 1.0, -1.0, 90.0, -0.0486, -74.7240), 1)
        self.assertEqual(self.lib.offline_step(handle, 0.0, 0.0, 1, 1, 50), 1)
        telemetry = json.loads(self.lib.offline_telemetry(handle))
        self.assertTrue(telemetry["ground"]["on_ground"])
        self.assertTrue(telemetry["ground"]["stopped"])
        self.assertEqual(telemetry["velocity"]["surface_mps"], 0.0)

    def test_invalid_input_does_not_advance(self):
        handle = self.create()
        before = self.lib.offline_telemetry(handle)
        for aoa, bank, ticks in (
            (float("nan"), 0, 1),
            (0, float("inf"), 1),
            (35, 0, 0),
            (35, 0, 51),
        ):
            self.assertEqual(
                self.lib.offline_step(handle, aoa, bank, 0, 0, ticks), 0
            )
            self.assertEqual(self.lib.offline_telemetry(handle), before)

    def test_missing_scenario_fails_closed(self):
        self.assertFalse(self.lib.offline_create(b"", b"", b"", b""))

    def test_initial_conditions_can_be_varied_once(self):
        handle = self.create()
        before = json.loads(self.lib.offline_telemetry(handle))
        self.assertEqual(
            self.lib.offline_set_initial_conditions(
                handle, 25000.0, 900.0, -6.0, 2.0, 0.1, -74.9
            ),
            1,
        )
        after = json.loads(self.lib.offline_telemetry(handle))
        self.assertNotEqual(after["position"]["altitude_m"], before["position"]["altitude_m"])
        self.assertNotEqual(after["velocity"]["surface_mps"], before["velocity"]["surface_mps"])
        self.assertEqual(
            self.lib.offline_set_initial_conditions(
                handle, 25000.0, 900.0, -6.0, 2.0, 0.1, -74.9
            ),
            0,
        )

    def test_deorbit_orbit_is_bounded_and_applies_burn(self):
        handle = self.create()
        before = json.loads(self.lib.offline_telemetry(handle))
        self.assertEqual(
            self.lib.offline_set_deorbit_orbit(
                handle, 400000.0, 70000.0, 87.0, -74.7, 0.0, 50000.0
            ),
            1,
        )
        after = json.loads(self.lib.offline_telemetry(handle))
        self.assertAlmostEqual(after["position"]["altitude_m"], 400000.0, delta=1)
        self.assertGreater(after["velocity"]["surface_mps"], 1000)
        self.assertNotEqual(after["velocity"]["surface_mps"], before["velocity"]["surface_mps"])
        self.assertEqual(
            self.lib.offline_set_deorbit_orbit(
                handle, 70000.0, 70000.0, 0.0, 0.0, 0.0, 50000.0
            ),
            0,
        )

    def test_deorbit_orbit_uses_physical_domains_not_test_corridors(self):
        # No arbitrary upper-orbit or 87-degree gate: a high retrograde orbit
        # remains a valid simulator input when its post-burn orbit intersects
        # the atmosphere.
        handle = self.create()
        self.assertEqual(self.lib.offline_set_deorbit_orbit(
            handle, 1000000.0, 70000.0, 120.0, 0.0, 0.0, 50000.0), 1)

        for values in (
            (69999.0, 69999.0, 0.0, 0.0, 0.0, 50000.0),  # pre-orbit in atmosphere
            (130000.0, 70000.0, -0.1, 0.0, 0.0, 60000.0),  # inclination domain
            (130000.0, 70000.0, 180.1, 0.0, 0.0, 60000.0),
            (130000.0, 70000.0, 0.0, 0.0, 0.0, 70000.0),  # no atmosphere crossing
            (130000.0, 70000.0, 0.0, 0.0, 0.0, -600000.0),  # nonphysical radius
        ):
            handle = self.create()
            self.assertEqual(
                self.lib.offline_set_deorbit_orbit(handle, *values), 0)


if __name__ == "__main__":
    unittest.main()
