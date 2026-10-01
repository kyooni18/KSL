"""Exercise actual craft topology with a small kRPC-shaped test double."""
import unittest
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock, patch
from shuttle.flight import (Shuttle, ascent_pitch, attitude_error_exceeded,
                            meco_ready, g_force_throttle,
                            direction_for_pitch_heading,
                            pitch_heading_for_direction, surface_up_for_pitch_heading,
                            flight_roll_target,
                            release_controls, roll_for_heading, run, set_autopilot_engaged)


class Part:
    ENGINE_NAMES = {'LiquidEngineRE-I2', 'LiquidEngineLV-T91',
                    'LiquidEngineLV-TX87', 'engineLargeSkipper_v2', 'sepMotor1'}

    def __init__(self, name, z=0, children=(), resources=None, is_engine=None):
        self.name = name
        self.title = name
        self.children = list(children)
        self.position = lambda frame: (0, 0, z)
        self.decoupled = False
        self.decoupler = SimpleNamespace(decouple=self._decouple)
        if is_engine is None:
            is_engine = name in self.ENGINE_NAMES or name.startswith('Replacement')
        self.engine = (SimpleNamespace(part=self, active=False, has_fuel=True,
                                       max_thrust=100, thrust=0, thrust_limit=1,
                                       specific_impulse=300,
                                       kerbin_sea_level_specific_impulse=300,
                                       vacuum_specific_impulse=300)
                       if is_engine else None)
        values = dict(resources or {})
        self.resources = SimpleNamespace(
            has_resource=lambda r: r in values,
            amount=lambda r: values.get(r, (0, 0, 1))[0],
            max=lambda r: values.get(r, (0, 0, 1))[1],
            density=lambda r: values.get(r, (0, 0, 1))[2])

    def _decouple(self):
        self.decoupled = True


class FlightTests(unittest.TestCase):
    def test_launch_heading_maps_to_double_body_roll(self):
        self.assertEqual(roll_for_heading(90), 180)
        self.assertEqual(roll_for_heading(45), 90)
        self.assertEqual(roll_for_heading(315), -90)

    def test_direction_vectors_match_surface_heading_axes(self):
        vertical = direction_for_pitch_heading(90, 90)
        self.assertAlmostEqual(vertical[0], 1.0)
        self.assertAlmostEqual(vertical[1], 0.0)
        self.assertAlmostEqual(vertical[2], 0.0)
        near_vertical = direction_for_pitch_heading(89, 90)
        self.assertGreater(near_vertical[2], 0.0)
        self.assertGreater(near_vertical[0], 0.99)
        direction = direction_for_pitch_heading(0, 90)
        self.assertAlmostEqual(direction[0], 0.0)
        self.assertAlmostEqual(direction[1], 0.0)
        self.assertAlmostEqual(direction[2], 1.0)

    def test_surface_direction_round_trips_pitch_and_heading(self):
        direction = direction_for_pitch_heading(58, 90)
        pitch, heading = pitch_heading_for_direction(direction)
        self.assertAlmostEqual(pitch, 58)
        self.assertAlmostEqual(heading, 90)

    def test_surface_roll_reference_is_unit_and_perpendicular_near_vertical(self):
        direction = direction_for_pitch_heading(89, 90)
        up = surface_up_for_pitch_heading(89, 90)
        self.assertAlmostEqual(sum(a*b for a, b in zip(direction, up)), 0)
        self.assertAlmostEqual(sum(x*x for x in up), 1)

    def test_gravity_turn_is_independent_of_delayed_roll_reset(self):
        self.assertEqual(ascent_pitch(1000, 1000, 45000), 89)
        self.assertAlmostEqual(ascent_pitch(23000, 1000, 45000), 44.5)
        self.assertEqual(ascent_pitch(45000, 1000, 45000), 5)

    def test_normal_meco_requires_both_apoapsis_and_periapsis(self):
        self.assertFalse(meco_ready(100000, -100000, 100000, 45000))
        self.assertFalse(meco_ready(90000, 45000, 100000, 45000))
        self.assertTrue(meco_ready(100000, 45000, 100000, 45000))

    def test_g_force_governor_preserves_ratios_with_global_throttle(self):
        self.assertAlmostEqual(g_force_throttle(1.0, 3.3, 3.0), 3.0/3.3)
        self.assertAlmostEqual(g_force_throttle(.7, 3.5, 3.0), .6)
        self.assertAlmostEqual(g_force_throttle(.7, 2.8, 3.0), .72)
        self.assertEqual(g_force_throttle(.7, 2.95, 3.0), .7)

    def test_attitude_error_guard_allows_bounded_roll_reset_maneuver(self):
        self.assertFalse(attitude_error_exceeded(179.5, 45000, 1))
        self.assertFalse(attitude_error_exceeded(179.5, 900, None))
        self.assertTrue(attitude_error_exceeded(46, 45000, 60))

    def test_roll_stays_inverted_until_180_seconds_after_booster_separation(self):
        self.assertEqual(flight_roll_target(90, False, None, 10, 180),
                         (None, False))
        self.assertEqual(flight_roll_target(90, True, None, 10, 180),
                         (180, False))
        self.assertEqual(flight_roll_target(90, True, 1000, 1179.9, 180),
                         (180, False))
        self.assertEqual(flight_roll_target(90, True, 1000, 1180, 180),
                         (0, True))

    def test_launch_rejects_non_pad_vessel_before_part_inspection(self):
        conn = MagicMock()
        conn.__enter__.return_value = conn
        conn.space_center.active_vessel = SimpleNamespace(
            name='STS', situation='flying')
        conn.space_center.VesselSituation.pre_launch = 'prelaunch'
        krpc = SimpleNamespace(connect=lambda **kwargs: conn)
        args = SimpleNamespace(address='127.0.0.1', rpc_port=50000,
                               stream_port=50001, launch=True)
        with patch.dict(sys.modules, {'krpc': krpc}), \
                patch('shuttle.flight.Shuttle') as shuttle:
            with self.assertRaisesRegex(ValueError, 'on the launch pad'):
                run(args)
        shuttle.assert_not_called()

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
        booster_engines = [Part('engineLargeSkipper_v2') for _ in range(4)]
        for i, part in enumerate(booster_engines):
            part.title = f'RE-I5 Skipper {i+1}'
        booster_tanks = [
            Part(f'BoosterTank-{i+1}', resources={
                'LiquidFuel': (10, 10, 1), 'Oxidizer': (10, 10, 1)})
            for i in range(4)]
        sep_motors = [Part('sepMotor1', resources={'SolidFuel': (5, 5, 1)})
                      for _ in range(4)]
        separators = [Part('radialDecoupler1-2', children=[tank, engine, sep])
                      for tank, engine, sep in zip(booster_tanks, booster_engines, sep_motors)]
        tanks = [Part('Size3LargeTank', resources={'LiquidFuel': (100, 100, 1),
                                                   'Oxidizer': (100, 100, 1)}),
                 Part('Size3LargeTank', resources={'LiquidFuel': (100, 100, 1),
                                                   'Oxidizer': (100, 100, 1)})]
        skiff = Part('LiquidEngineRE-I2', -3.8)
        cheetah = Part('LiquidEngineLV-T91')
        bobcats = [Part('LiquidEngineLV-TX87', z) for z in (-.84, -.84, .84, .84)]
        orbiter_tank = Part('mk3FuselageLFO.25',
                            resources={'LiquidFuel': (90, 100, 1),
                                       'Oxidizer': (90, 100, 1)})
        et = Part('radialDecoupler1-2',
                  children=tanks+separators+[cheetah]+bobcats)
        all_parts = ([et, skiff, orbiter_tank]+tanks+separators+booster_tanks+
                     booster_engines+sep_motors+[cheetah]+bobcats)
        calls = []
        def transfer(*args):
            calls.append(args)
            return SimpleNamespace(complete=False)
        vessel = SimpleNamespace(parts=SimpleNamespace(all=all_parts),
                                 reference_frame=None,
                                 control=SimpleNamespace(throttle=1),
                                 mass=1000)
        sc = SimpleNamespace(active_vessel=vessel, ut=1000,
                             ResourceTransfer=SimpleNamespace(start=transfer))
        shuttle = Shuttle(SimpleNamespace(space_center=sc))
        self.assertIs(shuttle.et_decoupler, et.decoupler)
        self.assertEqual(shuttle.booster_decouplers, [p.decoupler for p in separators])
        self.assertEqual(shuttle.booster_kinds, ['liquid'] * 4)
        self.assertEqual([e.part.name for e in shuttle.boosters], ['engineLargeSkipper_v2'] * 4)
        self.assertTrue(all(sep.engine not in shuttle.boosters for sep in sep_motors))
        self.assertEqual(len(shuttle.groups), 5)  # four core groups + one shared booster group
        self.assertEqual(shuttle.groups[-1], [p.engine for p in booster_engines])

        # All four Skippers receive one shared throttle, regardless of position.
        for i, engine_part in enumerate(booster_engines):
            engine_part.position = lambda frame, z=(-3, -1, 1, 3)[i]: (0, 0, z)
            engine_part.engine.max_thrust = 1000
        bounds = shuttle.group_upper_bounds()
        self.assertEqual(len(bounds), 5)
        self.assertEqual(bounds[-1], 1.0)
        self.assertGreater(shuttle.liquid_booster_nominal_burn_time(), 0)
        vessel.mass = 10
        limited = shuttle.allocation(max_g=3)
        self.assertLessEqual(shuttle.commanded_g(limited), 3.000001)
        vessel.mass = 1000
        shuttle.update(max_g=3)
        booster_limits = [p.engine.thrust_limit for p in booster_engines]
        self.assertTrue(all(abs(x-booster_limits[0]) < 1e-12 for x in booster_limits))

        self.assertEqual([e.part.position(None)[2] for e in shuttle.groups[1]], [-.84, -.84])
        self.assertEqual([e.part.position(None)[2] for e in shuttle.groups[2]], [.84, .84])
        shuttle.top_off()
        shuttle.top_off()
        self.assertEqual(len(calls), 2)
        self.assertTrue(all(c[0] in tanks and c[1] is orbiter_tank for c in calls))
        self.assertFalse(shuttle.release_et())

        # The four liquid boosters stay attached while fueled, then cut and separate
        # together as soon as one branch begins to run dry.
        shuttle.transfers = {}
        shuttle.update(max_g=3)
        self.assertFalse(shuttle.separated)
        booster_engines[0].engine.has_fuel = False
        shuttle.update(max_g=3)
        self.assertTrue(shuttle.separated)
        self.assertTrue(all(p.decoupled for p in separators))
        self.assertTrue(all(not e.active and e.thrust_limit == 0
                            for e in [p.engine for p in booster_engines]))
        self.assertEqual(len(shuttle.groups), 4)


if __name__ == '__main__':
    unittest.main()
