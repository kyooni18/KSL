"""Offline regressions for the probe's read-only boundary and telemetry admission."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'Tools'))
import probe_sts_terminal_speed_polar as probe


class ProbeAdmissionTests(unittest.TestCase):
    def test_default_mode_never_connects_or_hashes_live_saves(self):
        real_sha = probe.sha
        def offline_sha(path):
            self.assertIn(path, (probe.CONFIG, probe.PROFILE))
            return real_sha(path)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'plan.json'
            with patch.object(sys, 'argv', ['probe', '--output', str(output)]), \
                    patch.object(probe, 'connect', side_effect=AssertionError('live connection')), \
                    patch.object(probe, 'sha', side_effect=offline_sha):
                self.assertEqual(probe.main(), 0)
            self.assertEqual(json.loads(output.read_text())['status'], 'offline-plan')

    def test_nonfinite_preflight_never_admits(self):
        profile = dict(savedUT=1000, mass=26000, dryMass=20000, partCount=22)
        snapshot = dict(vessel='STS', paused=True, ut=1000, altitude_m=72000,
                        mass_kg=26000, thrust_n=0, sas=False, rcs=False,
                        controls=dict(pitch=0, throttle=0))
        actuators = dict(speedbrakeActionGroupEnabled=False,
                         controlSurfaces=[dict(deployed=False, deflection=0)])
        probe.require_preflight(snapshot, actuators, profile, 22, 20000)
        for invalid in (float('nan'), float('inf'), -float('inf')):
            for field in ('ut', 'altitude_m', 'mass_kg', 'thrust_n'):
                state = copy.deepcopy(snapshot)
                state[field] = invalid
                with self.subTest(field=field, invalid=invalid), self.assertRaises(RuntimeError):
                    probe.require_preflight(state, actuators, profile, 22, 20000)
            state = copy.deepcopy(snapshot)
            state['controls']['pitch'] = invalid
            surfaces = copy.deepcopy(actuators)
            surfaces['controlSurfaces'][0]['deflection'] = invalid
            for state_arg, actuator_arg, dry in ((state, actuators, 20000),
                    (snapshot, surfaces, 20000), (snapshot, actuators, invalid)):
                with self.assertRaises(RuntimeError):
                    probe.require_preflight(state_arg, actuator_arg, profile, 22, dry)


if __name__ == '__main__':
    unittest.main()
