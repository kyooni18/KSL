"""Focused vacuum bench guards and inertial-to-body rate conversion."""
import json
import math
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'Tools'))
from probe_sts_preentry_roll import body_rates,validate_sample


class PreentryBenchTests(unittest.TestCase):
    def test_guards_and_body_coordinates(self):
        cfg=json.loads((ROOT/'Configuration/sts.json').read_text())
        safe=dict(ut=9931,altitude=71900,q=0,thrust=0,roll_rate=0,mass=25816,SAS=False,RCS=False)
        validate_sample(safe,9931,cfg)
        for key,value in [('q',1),('altitude',69999),('thrust',2),('roll_rate',5),('ut',9935),('SAS',True),('RCS',True),('mass',math.nan)]:
            with self.subTest(guard=key):
                sample=dict(safe);sample[key]=value
                with self.assertRaises(RuntimeError):validate_sample(sample,9931,cfg)
        rad=math.pi/180
        for axis in range(3):
            with self.subTest(axis=axis):
                angular=[0,0,0];angular[axis]=-rad
                self.assertAlmostEqual(body_rates(angular,(0,0,0,1))[axis],1)
        # A quarter turn about inertial Z maps body +X into inertial +Y.
        q=(0,0,math.sqrt(.5),math.sqrt(.5))
        rates=body_rates((0,-rad,0),q)
        self.assertAlmostEqual(rates[0],1)
        self.assertAlmostEqual(rates[1],0)
        self.assertAlmostEqual(rates[2],0)


if __name__=='__main__':unittest.main()
