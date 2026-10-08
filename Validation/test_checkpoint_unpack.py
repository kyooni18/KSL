import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'Tools'))
from settle_known_checkpoint import resume_placeholder_physics


class CheckpointUnpackTests(unittest.TestCase):
    def test_only_placeholder_state_resumes_at_one_x_without_thrust(self):
        profile=dict(mass=25816.105, dryMass=20348, partCount=22)
        for mass, thrust, resumes, rejected in [
            (22000, 0, True, False), (25816.105, 0, False, False),
            (22000, 10, False, True),
        ]:
            with self.subTest(mass=mass, thrust=thrust):
                conn=NS(space_center=NS(rails_warp_factor=2, physics_warp_factor=2),
                        krpc=NS(paused=True))
                vessel=NS(mass=mass, dry_mass=20348, thrust=thrust,
                          parts=NS(all=list(range(22))))
                if rejected:
                    with self.assertRaises(RuntimeError):
                        resume_placeholder_physics(conn,vessel,profile)
                else:
                    resume_placeholder_physics(conn,vessel,profile)
                self.assertEqual(conn.krpc.paused, not resumes)
                self.assertEqual(conn.space_center.rails_warp_factor,0)
                self.assertEqual(conn.space_center.physics_warp_factor,0)
