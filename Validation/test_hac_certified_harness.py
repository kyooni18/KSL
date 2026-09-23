"""Fast harness-only regressions; no simulator or KSP process is started."""
from __future__ import annotations
import importlib.util
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('hac_legacy_runner', ROOT/'ShuttleSim/scripts/run_guidance.py')
assert spec and spec.loader
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def geometry(radius: float=3000.0, *, arc_degrees: float=270, shift: float=0, lead: float=3049.569) -> str:
    x = -3500.0 + shift
    return (f'MM305 fixed-HAC geometry: variant=B runwayStation=(-3500.000,0.000) '
            f'center=({x:.3f},{-radius:.3f}) upper=({x:.3f},0.000) exit=({x:.3f},0.000) '
            f'exitAlong={x:.3f} exitCross=0.000 entry=({x+radius:.3f},{-radius:.3f}) '
            f'lead={lead:.3f} arc={radius*math.radians(arc_degrees):.3f}.')


class CertifiedHarnessTests(unittest.TestCase):
    def test_dynamic_radius_not_twelve_kilometre_assumption(self):
        for radius in (3000.0,3978.9473684210525,12000.0):
            with self.subTest(radius=radius):
                result = runner.fixed_hac_geometry_evidence(geometry(radius))
                self.assertTrue(result['valid'], result)
                self.assertAlmostEqual(result['radius'],radius,places=3)

    def test_missing_commit_not_success(self):
        self.assertFalse(runner.fixed_hac_geometry_evidence('HAC_EVAL selected=0')['valid'])

    def test_exact_arc_and_dynamic_floor(self):
        for degrees in (90,180,360,630):
            with self.subTest(degrees=degrees):
                self.assertFalse(runner.fixed_hac_geometry_evidence(geometry(arc_degrees=degrees))['valid'])
        self.assertFalse(runner.fixed_hac_geometry_evidence(geometry(2000))['valid'])

    def test_exit_anchor_and_frozen_geometry(self):
        self.assertFalse(runner.fixed_hac_geometry_evidence(geometry(shift=50))['valid'])
        self.assertFalse(runner.fixed_hac_geometry_evidence(geometry()+'\n'+geometry(4000))['valid'])
        self.assertFalse(runner.fixed_hac_geometry_evidence(geometry()+'\n'+geometry(lead=5000))['valid'])
        self.assertTrue(runner.fixed_hac_geometry_evidence(geometry()+'\n'+geometry())['valid'])

    def test_malformed_record_fails_closed(self):
        for text in ('MM305 fixed-HAC geometry:',geometry().replace('center=(-3500.000,-3000.000)','center=(nan,0)'),
                     geometry().replace('center=(-3500.000,-3000.000)','center=(1e-,0)')):
            self.assertFalse(runner.fixed_hac_geometry_evidence(text)['valid'])


if __name__ == '__main__':
    unittest.main()
