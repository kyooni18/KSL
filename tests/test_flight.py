"""Exercise actual craft topology with a small kRPC-shaped test double."""
import unittest
from types import SimpleNamespace
from shuttle.flight import (Shuttle, release_controls, roll_for_heading,
                            set_autopilot_engaged, validate_launch_vessel)


class Part:
    def __init__(self, name, z=0, children=()):
        self.name = name
        self.children = list(children)
        self.position = lambda frame: (0, 0, z)
        self.decoupler = SimpleNamespace(decouple=lambda: None)
        self.engine = SimpleNamespace(part=self, active=False, has_fuel=True,
                                      max_thrust=100, thrust=0, thrust_limit=1)
        self.resources = SimpleNamespace(amount=lambda r: 100,
                                         max=lambda r: 100)


class FlightTests(unittest.TestCase):
    def test_launch_heading_maps_to_double_body_roll(self):
        self.assertEqual(roll_for_heading(90), 180)
        self.assertEqual(roll_for_heading(45), 90)
        self.assertEqual(roll_for_heading(315), -90)

    def test_launch_rejects_non_pad_vessel_before_part_inspection(self):
        vessel = SimpleNamespace(name='STS', situation='flying')
        space_center = SimpleNamespace(
            VesselSituation=SimpleNamespace(pre_launch='prelaunch'))
        with self.assertRaisesRegex(ValueError, 'on the launch pad'):
            validate_launch_vessel(vessel, space_center)

    def test_autopilot_engagement_property_used_by_current_krpc(self):
        class AutoPilot:
            engaged = False

        autopilot = AutoPilot()
        set_autopilot_engaged(autopilot, True)
        self.assertTrue(autopilot.engaged)
        set_autopilot_engaged(autopilot, False)
        self.assertFalse(autopilot.engaged)

    def test_control_release_restores_sas_even_if_autopilot_release_fails(self):
        control = SimpleNamespace(throttle=1, sas=False)
        with self.assertRaises(AttributeError):
            release_controls(control, SimpleNamespace())
        self.assertEqual(control.throttle, 0)
        self.assertTrue(control.sas)

    def test_et_ancestor_is_not_a_booster_separator_and_topoff_is_serial(self):
        boosters = [Part('Pollux'), Part('Pollux')]
        separators = [Part('radialDecoupler1-2', children=[b]) for b in boosters]
        tanks = [Part('Size3LargeTank'), Part('Size3LargeTank')]
        et = Part('radialDecoupler1-2', children=tanks+separators)
        skiff = Part('LiquidEngineRE-I2', -3.8)
        cheetah = Part('LiquidEngineLV-T91')
        bobcats = [Part('LiquidEngineLV-TX87', z) for z in (-.84, -.84, .84, .84)]
        orbiter_tank = Part('mk3FuselageLFO.25')
        all_parts = [et, skiff, cheetah, orbiter_tank]+tanks+separators+boosters+bobcats
        calls = []
        def transfer(*args):
            calls.append(args)
            return SimpleNamespace(complete=False)
        vessel = SimpleNamespace(parts=SimpleNamespace(all=all_parts), reference_frame=None, control=SimpleNamespace(throttle=1))
        sc = SimpleNamespace(active_vessel=vessel,
                             ResourceTransfer=SimpleNamespace(start=transfer))
        shuttle = Shuttle(SimpleNamespace(space_center=sc))
        self.assertIs(shuttle.et_decoupler, et.decoupler)
        self.assertEqual(shuttle.booster_decouplers, [p.decoupler for p in separators])
        self.assertEqual([e.part.position(None)[2] for e in shuttle.groups[1]], [-.84, -.84])
        self.assertEqual([e.part.position(None)[2] for e in shuttle.groups[2]], [.84, .84])
        orbiter_tank.resources.amount = lambda r: 90
        shuttle.top_off()
        shuttle.top_off()
        self.assertEqual(len(calls), 2)  # one in-flight transfer per resource
        self.assertTrue(all(c[0] in tanks and c[1] is orbiter_tank for c in calls))
        self.assertFalse(shuttle.release_et())  # no separation during transfer


if __name__ == '__main__':
    unittest.main()
