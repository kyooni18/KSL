"""Exercise actual craft topology with a small kRPC-shaped test double."""
import unittest
import math
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock, patch
from ksl.ascent.flight import (Shuttle, ascent_pitch, ascent_target_g, attitude_error_exceeded,
                            body_roll_rate_deg_s, desired_orbiter_side,
                            earthward_roll_target, gravity_turn_pitch,
                            meco_ready, g_force_throttle, slew_value, slew_vector,
                            direction_for_pitch_heading, direction_error_deg,
                            pitch_heading_for_direction, surface_axis_errors,
                            surface_up_for_pitch_heading, hold_turn_for_roll,
                            limit_aero_roll, restore_aero_roll,
                            mission_roll_allowed, maneuver_roll_rate, manual_roll_command,
                            roll_attitude_settled, roll_delta, roll_for_roof, slew_roll,
                            ensure_autopilot_engaged, pause_structural_failure,
                            release_controls, roll_for_heading,
                            run, set_autopilot_engaged, validate_launch_vessel,
                            restore_unexpected_ssme_shutdown, control_interval)
from ksl.ascent.guidance import post_booster_master_throttle, separation_thrust_blend
from ksl.ascent.vehicle import delay_booster_separation


class SeparationEnvelopeTests(unittest.TestCase):
    def test_ssme_balances_boosters_through_last_fueled_tick(self):
        # A canted core contributes -1 MN m at full thrust. The SRBs
        # contribute +0.9 MN m, beyond the former 70% trim ceiling.
        frame = object()
        core = [SimpleNamespace(
            thrust_limit=.38, max_thrust=1e6/3, thrust=0., has_fuel=True,
            part=SimpleNamespace(position=lambda _: (0., 0., 1.),
                                 direction=lambda _: (0., 1., 0.))) for _ in range(3)]
        booster = SimpleNamespace(
            active=True, has_fuel=True, thrust=9e5,
            part=SimpleNamespace(position=lambda _: (0., 0., -1.),
                                 direction=lambda _: (0., 1., 0.),
                                 resources=SimpleNamespace(
                                     has_resource=lambda _: True,
                                     amount=lambda _: 1., max=lambda _: 100.)))
        shuttle = Shuttle.__new__(Shuttle)
        shuttle.et_attached = True
        shuttle.top_off = lambda: None
        shuttle.separated = False
        shuttle.vehicle_kind = 'orbiter-shuttle'
        shuttle.solid_boosters = [booster]
        shuttle.groups = shuttle.core_groups = [core]
        shuttle.vessel = SimpleNamespace(reference_frame=frame)
        shuttle._liquid_booster_depleted = lambda: False
        shuttle.allocation = lambda **kwargs: SimpleNamespace(throttles=(0.,))
        shuttle.allocation_for_throttles = lambda throttles: throttles

        # Even with zero measured core thrust during spool-up, derive the
        # correct trim from rated thrust; preserve the normal actuator slew.
        result = shuttle.update(dt=.1, throttle_slew=.2)
        self.assertAlmostEqual(result[0], .40)
        result = shuttle.update(dt=3., throttle_slew=1.)
        self.assertAlmostEqual(result[0], .9)
        self.assertTrue(all(abs(e.thrust_limit-.9) < 1e-9 for e in core))

        # Follow the changing CoM/thrust moment, without a low-fuel taper.
        booster.thrust = 6e5
        self.assertAlmostEqual(shuttle.update(dt=1., throttle_slew=1.)[0], .6)
        booster.thrust = 1.2e6
        self.assertEqual(shuttle.update(dt=1., throttle_slew=1.)[0], 1.)
        self.assertFalse(shuttle.separated)

    def test_srb_separation_caps_core_before_detach_without_waiting_for_slew(self):
        core = SimpleNamespace(thrust_limit=.91, max_thrust=3e6)
        booster = SimpleNamespace(thrust_limit=.9, active=True)
        observed = []
        shuttle = Shuttle.__new__(Shuttle)
        shuttle.vehicle_kind = 'orbiter-shuttle'
        shuttle.vessel = SimpleNamespace(mass=183000.)
        shuttle.sc = SimpleNamespace(ut=100.)
        shuttle.et_attached = True
        shuttle.separated = False
        shuttle.core_groups = shuttle.groups = [[core]]
        shuttle.liquid_boosters = []
        shuttle.solid_boosters = [booster]
        shuttle.sepmotors = []
        shuttle.booster_decouplers = [SimpleNamespace(decouple=lambda: observed.append(core.thrust_limit))]
        shuttle.cruise_control_moment_budget = 1.
        shuttle.top_off = lambda: None
        shuttle._liquid_booster_depleted = lambda: False
        shuttle._solid_boosters_depleted = lambda: True
        shuttle.allocation = lambda **kwargs: SimpleNamespace(throttles=(.9,))
        shuttle.allocation_for_throttles = lambda throttles: throttles
        shuttle.conn = object()
        def stage(conn, groups, boosters, motors, decouplers):
            core.thrust_limit = min(core.thrust_limit, .3)
            for decoupler in decouplers:
                decoupler.decouple()
            return (100., 100.)
        with patch('ksl.ascent.vehicle.separate_sts_boosters_batch', side_effect=stage):
            result = shuttle.update(dt=.1, throttle_slew=.01)
        self.assertEqual(observed, [.3])
        self.assertEqual(result, (.3,))

    def test_thrust_cant_blend_waits_for_hold_then_is_continuous_through_roll(self):
        self.assertEqual(separation_thrust_blend(100., 100.), 0.)
        self.assertEqual(separation_thrust_blend(104.999, 100.), 0.)
        self.assertEqual(separation_thrust_blend(105., 100.), 0.)
        self.assertEqual(separation_thrust_blend(107., 100.), .5)
        self.assertEqual(separation_thrust_blend(109., 100.), 1.)
        self.assertEqual(separation_thrust_blend(107., 100., 107.), .5)
        self.assertEqual(separation_thrust_blend(111., 100., 107.), 0.)
        self.assertEqual(separation_thrust_blend(120., 100., 107., 120.), 0.)
        self.assertEqual(separation_thrust_blend(123., 100., 107., 120.), .5)
        self.assertEqual(separation_thrust_blend(126., 100., 107., 120.), 1.)

    def test_opt_in_post_booster_coast_keeps_core_off_then_restarts(self):
        engine = SimpleNamespace(thrust_limit=0.0, max_thrust=1000.0)
        shuttle = Shuttle.__new__(Shuttle)
        shuttle.et_attached = True
        shuttle.top_off = lambda: None
        shuttle.separated = True
        shuttle.vehicle_kind = 'orbiter-shuttle'
        shuttle.solid_boosters = [object()]
        shuttle.booster_separation_ut = 100.0
        shuttle.sc = SimpleNamespace(ut=102.0)
        shuttle.groups = [[engine]]
        shuttle.core_groups = [[engine]]
        shuttle.vessel = SimpleNamespace(mass=1000.0)
        shuttle.allocation = lambda **kwargs: SimpleNamespace(throttles=(1.0,))
        shuttle.allocation_for_throttles = lambda throttles: throttles
        shuttle.update(post_booster_coast_seconds=5.0, dt=1.0, throttle_slew=1.0)
        self.assertEqual(engine.thrust_limit, 0.0)
        shuttle.sc.ut = 106.0
        shuttle.update(post_booster_coast_seconds=5.0, dt=1.0, throttle_slew=1.0)
        self.assertGreater(engine.thrust_limit, 0.0)

    def test_explicit_pressure_limit_holds_spent_shuttle_boosters(self):
        self.assertTrue(delay_booster_separation('orbiter-shuttle', [object()], True,
                                                  5000.0, 1000.0))
        self.assertFalse(delay_booster_separation('orbiter-shuttle', [object()], True,
                                                   900.0, 1000.0))
        self.assertFalse(delay_booster_separation('orbiter-shuttle', [object()], True,
                                                   5000.0, float('inf')))
        self.assertFalse(delay_booster_separation('nuclear-solid', [object()], True,
                                                   5000.0, 1000.0))


class Part:
    ENGINE_NAMES = {'LiquidEngineRE-I2', 'LiquidEngineRE-J10',
                    'engineLargeSkipper.v2', 'LiquidEngineRK-7',
                    'LiquidEngineRV-1', 'sepMotor1', 'nfa-atomic-multimode-25-1',
                    'Size3AdvancedEngine', 'PC.4Seg.RSRM', 'PC.Nose'}

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
                                       max_thrust=100, max_vacuum_thrust=100,
                                       thrust=0, thrust_limit=1,
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

    def test_nose_error_ignores_roll_and_measures_direction_only(self):
        target = direction_for_pitch_heading(89, 90)
        self.assertAlmostEqual(direction_error_deg(target, target), 0)
        actual = direction_for_pitch_heading(88, 90)
        self.assertAlmostEqual(direction_error_deg(actual, target), 1, places=6)

    def test_surface_axis_errors_do_not_depend_on_autopilot(self):
        pitch_error, heading_error = surface_axis_errors(
            direction_for_pitch_heading(88, 92), 89, 90)
        self.assertAlmostEqual(pitch_error, 1)
        self.assertAlmostEqual(heading_error, 2)

    def test_surface_roll_reference_is_unit_and_perpendicular_near_vertical(self):
        direction = direction_for_pitch_heading(89, 90)
        up = surface_up_for_pitch_heading(89, 90)
        self.assertAlmostEqual(sum(a*b for a, b in zip(direction, up)), 0)
        self.assertAlmostEqual(sum(x*x for x in up), 1)

    def test_pad_roof_is_converted_to_krpc_roll_sign(self):
        self.assertAlmostEqual(
            roll_for_roof((1, 0, 0), (0, 0, -1), (0, -1, 0)), 90)

    def test_physical_orbiter_side_target_flips_exactly_earthward_to_spaceward(self):
        earthward = desired_orbiter_side(45, 90, True)
        spaceward = desired_orbiter_side(45, 90, False)
        direction = direction_for_pitch_heading(45, 90)
        self.assertAlmostEqual(sum(a*b for a, b in zip(direction, earthward)), 0)
        self.assertAlmostEqual(sum(a*b for a, b in zip(direction, spaceward)), 0)
        for earth, space in zip(earthward, spaceward):
            self.assertAlmostEqual(earth, -space)
        reference = surface_up_for_pitch_heading(45, 90)
        earth_roll = roll_for_roof(direction, reference, earthward)
        space_roll = roll_for_roof(direction, reference, spaceward)
        self.assertAlmostEqual(abs(roll_delta(earth_roll, space_roll)), 180)

    def test_roll_slew_uses_shortest_path_and_rate_limit(self):
        self.assertAlmostEqual(slew_roll(90, 180, 3), 93)
        self.assertAlmostEqual(slew_roll(179, -179, 3), -179)

    def test_upright_maneuver_schedule_respects_roll_rate_cap(self):
        self.assertEqual(maneuver_roll_rate(180, 10, 15), 15)
        self.assertEqual(maneuver_roll_rate(180, 10, 5), 5)
        self.assertEqual(maneuver_roll_rate(20, 10, 15), 3)

    def test_structural_abort_releases_controls_and_pauses_ksp(self):
        conn = SimpleNamespace(krpc=SimpleNamespace(paused=False), space_center=MagicMock())
        control = SimpleNamespace(throttle=1.0, roll=0.5, sas=False)
        autopilot = MagicMock()
        pause_structural_failure(conn, control, autopilot)
        self.assertEqual(control.throttle, 0)
        self.assertEqual(control.roll, 0)
        self.assertTrue(control.sas)
        self.assertTrue(conn.krpc.paused)
        autopilot.disengage.assert_called_once()
        self.assertTrue(conn.space_center.save.call_args.args[0].startswith('ksl-structural-'))

    def test_mission_roll_starts_after_climbing_above_the_pad(self):
        self.assertFalse(mission_roll_allowed(False, 100, 30))
        self.assertFalse(mission_roll_allowed(True, 29.9, 30))
        self.assertTrue(mission_roll_allowed(True, 30, 30))
        self.assertTrue(mission_roll_allowed(True, 100, 30))

    def test_roll_settle_requires_low_rate_and_small_nose_error(self):
        self.assertTrue(roll_attitude_settled(2, .5, 2))
        self.assertFalse(roll_attitude_settled(4, .5, 1))
        self.assertFalse(roll_attitude_settled(1, 1.0, 1))
        self.assertFalse(roll_attitude_settled(1, .5, 4))

    def test_manual_roll_controller_is_rate_damped_bounded_and_brakes_overspeed(self):
        self.assertGreater(manual_roll_command(30, 0, 40, 6), 0)
        self.assertLess(manual_roll_command(30, 8, 40, 6), 0)
        self.assertEqual(manual_roll_command(30, 10, 40, 6), -1.0)
        self.assertLessEqual(abs(manual_roll_command(180, -100, 40, 6)), 1.0)

    def test_body_roll_rate_projects_angular_velocity_onto_forward_axis(self):
        vessel = SimpleNamespace(
            surface_reference_frame=object(),
            direction=lambda frame: (0.0, 2.0, 0.0),
            angular_velocity=lambda frame: (1.0, math.pi/18.0, 3.0),
        )
        self.assertAlmostEqual(body_roll_rate_deg_s(vessel), -10.0)

    def test_body_roll_rate_matches_krpc_roll_angle_sign(self):
        vessel = SimpleNamespace(
            surface_reference_frame=object(),
            direction=lambda frame: (1.0, 0.0, 0.0),
            angular_velocity=lambda frame: (1.0, 0.0, 0.0),
        )
        self.assertAlmostEqual(body_roll_rate_deg_s(vessel), -57.295779513, places=6)

    def test_ascent_target_g_ramps_smoothly_and_honors_booster_floor(self):
        self.assertAlmostEqual(ascent_target_g(0, 70000, 3.0), 1.08)
        self.assertAlmostEqual(ascent_target_g(0, 70000, 3.0, minimum_g=1.2), 1.2)
        self.assertGreater(ascent_target_g(35000, 70000, 3.0, minimum_g=1.2), 1.2)
        self.assertAlmostEqual(ascent_target_g(70000, 70000, 3.0, minimum_g=1.2), 3.0)

    def test_engine_throttle_slew_is_continuous_and_rate_limited(self):
        self.assertAlmostEqual(slew_value(.5, 1.0, .25, .1), .525)
        self.assertAlmostEqual(slew_value(.5, 0.0, .25, .1), .475)
        self.assertAlmostEqual(slew_value(.5, .51, .25, .1), .51)

    def test_ascent_slew_tracks_simulation_time_despite_slow_rpc_or_pause(self):
        # The live 90 km ascent turned at only ~0.2 deg/s with a configured
        # 0.8 deg/s because each slow RPC cycle used a nominal 0.1 s step.
        for times in ([0, .1, .2, .3, .4, .5, 1, 2, 3, 4, 5],
                      [0, .7, 1.9, 1.9, 3.2, 5]):
            pitch, throttle, previous = 89.0, .3, None
            for ut in times:
                dt = control_interval(ut, previous)
                pitch = slew_value(pitch, 45.0, .8, dt)
                throttle = slew_value(throttle, 1.0, .1, dt)
                previous = ut
            self.assertAlmostEqual(pitch, 85.0)
            self.assertAlmostEqual(throttle, .8)
        self.assertEqual(control_interval(99, 100), 0)


    def test_vector_slew_uses_one_interpolation_factor(self):
        current = (.2, .4, .8)
        target = (.8, .1, .6)
        result = slew_vector(current, target, .25, .2)
        deltas = [r-c for r, c in zip(result, current)]
        target_deltas = [t-c for t, c in zip(target, current)]
        ratios = [d/td for d, td in zip(deltas, target_deltas)]
        self.assertAlmostEqual(max(abs(d) for d in deltas), .05)
        self.assertAlmostEqual(ratios[0], ratios[1])
        self.assertAlmostEqual(ratios[1], ratios[2])

    def test_elevon_roll_is_enabled_at_limited_authority_and_restored(self):
        elevon = SimpleNamespace(roll_enabled=False, pitch_enabled=True, authority_limiter=100.0)
        small = SimpleNamespace(roll_enabled=True, pitch_enabled=True, authority_limiter=20.0)
        rudder = SimpleNamespace(roll_enabled=False, pitch_enabled=False, authority_limiter=100.0)
        vessel = SimpleNamespace(parts=SimpleNamespace(all=[
            SimpleNamespace(name='benjee10.shuttle.elevon1', control_surface=elevon),
            SimpleNamespace(name='benjee10.shuttle.ELEVON2', control_surface=small),
            SimpleNamespace(name='benjee10.shuttle.rudder', control_surface=rudder),
        ]))
        states = limit_aero_roll(vessel, authority_limit=35.0)
        self.assertEqual(len(states), 2)
        self.assertTrue(elevon.roll_enabled)
        self.assertEqual(elevon.authority_limiter, 35.0)
        self.assertEqual(small.authority_limiter, 20.0)  # never raised above its own setting
        self.assertTrue(elevon.pitch_enabled)
        self.assertFalse(rudder.roll_enabled)  # non-elevon surfaces untouched
        self.assertEqual(rudder.authority_limiter, 100.0)
        restore_aero_roll(states)
        self.assertFalse(elevon.roll_enabled)
        self.assertEqual(elevon.authority_limiter, 100.0)
        self.assertTrue(small.roll_enabled)
        self.assertEqual(small.authority_limiter, 20.0)

    def test_gravity_turn_waits_for_dedicated_roll_phase(self):
        self.assertTrue(hold_turn_for_roll(True, False))
        self.assertFalse(hold_turn_for_roll(False, False))
        self.assertFalse(hold_turn_for_roll(True, True))

    def test_gravity_turn_follows_prograde_with_q_limited_pitch_under(self):
        low_q = gravity_turn_pitch(100, 100, 0)
        high_q = gravity_turn_pitch(100, 100, 30000)
        # Prograde is 45 deg here; bias is 20 deg at low q and 8 deg above 25 kPa
        # (f5d8830 "Bias STS gravity turn toward horizontal suborbit").
        self.assertAlmostEqual(low_q, 25.0, places=1)
        self.assertAlmostEqual(high_q, 37.0, places=1)
        self.assertAlmostEqual(gravity_turn_pitch(100, 100, 15000), 31.0, places=1)
        self.assertLess(low_q, high_q)
        self.assertLess(gravity_turn_pitch(100, .5, 0), 89)

    def test_normal_meco_requires_both_apoapsis_and_periapsis(self):
        self.assertFalse(meco_ready(100000, -100000, 100000, 45000))
        self.assertFalse(meco_ready(90000, 45000, 100000, 45000))
        self.assertTrue(meco_ready(100000, 45000, 100000, 45000))

    def test_ssme_auto_restart_recovers_all_three_without_touching_limiters(self):
        engines = [SimpleNamespace(active=False, has_fuel=True, can_restart=True,
                                   thrust_limit=.4852,
                                   part=SimpleNamespace(title='Vector SSME'))
                   for _ in range(3)]
        self.assertEqual(restore_unexpected_ssme_shutdown(engines), 3)
        self.assertTrue(all(e.active for e in engines))
        self.assertTrue(all(e.thrust_limit == .4852 for e in engines))
        self.assertEqual(restore_unexpected_ssme_shutdown(engines), 0)

    def test_ssme_auto_restart_refuses_engine_with_no_fuel(self):
        engines = [SimpleNamespace(active=False, has_fuel=False, can_restart=True,
                                   part=SimpleNamespace(title='Vector SSME'))]
        with self.assertRaisesRegex(RuntimeError, 'cannot relight'):
            restore_unexpected_ssme_shutdown(engines)
        self.assertFalse(engines[0].active)

    def test_ssme_auto_restart_refuses_non_restartable_engine(self):
        engines = [SimpleNamespace(active=False, has_fuel=True, can_restart=False,
                                   part=SimpleNamespace(title='Vector SSME'))]
        with self.assertRaisesRegex(RuntimeError, 'cannot relight'):
            restore_unexpected_ssme_shutdown(engines)

    def test_post_booster_master_does_not_latch_at_zero(self):
        # Live polar ascent twice idled its SSMEs in thin air although the
        # core thrust-limit allocator still requested positive acceleration.
        self.assertEqual(post_booster_master_throttle(0.0, 3.0, 0.0, 30000.0), 1.0)
        self.assertEqual(post_booster_master_throttle(0.5, 3.0, 0.0, 30000.0), 1.0)
        self.assertAlmostEqual(post_booster_master_throttle(3.5, 3.0, 0.0,
                                                            30000.0), 3.0/3.5)
        self.assertLess(post_booster_master_throttle(1.0, 3.0, 45000.0,
                                                    30000.0), 1.0)

    def test_g_force_governor_preserves_ratios_with_global_throttle(self):
        self.assertAlmostEqual(g_force_throttle(1.0, 3.3, 3.0), 3.0/3.3)
        self.assertAlmostEqual(g_force_throttle(.7, 3.5, 3.0), .6)
        self.assertAlmostEqual(g_force_throttle(.7, 2.8, 3.0), .72)
        self.assertEqual(g_force_throttle(.7, 2.95, 3.0), .7)

    def test_attitude_error_guard_allows_bounded_roll_reset_maneuver(self):
        self.assertFalse(attitude_error_exceeded(179.5, 45000, 1))
        self.assertFalse(attitude_error_exceeded(179.5, 900, None))
        self.assertTrue(attitude_error_exceeded(46, 45000, 60))

    def test_physical_roll_stays_earthward_until_delayed_post_booster_flip(self):
        self.assertEqual(earthward_roll_target(False, None, 10, 180),
                         (None, False))
        self.assertEqual(earthward_roll_target(True, None, 10, 180),
                         (True, False))
        self.assertEqual(earthward_roll_target(True, 1000, 1179.9, 180),
                         (True, False))
        self.assertEqual(earthward_roll_target(True, 1000, 1180, 180),
                         (False, True))

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
                patch('ksl.ascent.flight.Shuttle') as shuttle:
            with self.assertRaisesRegex(ValueError, 'on the launch pad'):
                run(args)
        shuttle.assert_not_called()

    def test_launch_validation_accepts_sts_n_only_on_the_pad(self):
        vessel = SimpleNamespace(name='STS-N', situation='prelaunch')
        space_center = SimpleNamespace(VesselSituation=SimpleNamespace(
            pre_launch='prelaunch'))
        validate_launch_vessel(vessel, space_center)

    def test_launch_accepts_settled_pad_stack_still_on_clamps(self):
        frame = object()
        vessel = SimpleNamespace(
            name='STS', situation='landed',
            parts=SimpleNamespace(launch_clamps=[object(), object()]),
            orbit=SimpleNamespace(body=SimpleNamespace(reference_frame=frame)),
            flight=lambda reference_frame: SimpleNamespace(speed=0.3))
        situations = SimpleNamespace(pre_launch='prelaunch', landed='landed')
        validate_launch_vessel(vessel, SimpleNamespace(VesselSituation=situations))
        vessel.parts.launch_clamps.pop()
        with self.assertRaisesRegex(ValueError, 'on the launch pad'):
            validate_launch_vessel(vessel, SimpleNamespace(VesselSituation=situations))

    def test_sts_nuclear_et_recognizes_twin_rhino_and_solid_booster_stages(self):
        nuke = Part('nfa-atomic-multimode-25-1', -4)
        nuke.title = "J-N500 'Project Eeloo' Multimode Nuclear Engine"
        orbiter_tank = Part('Rockomax8BW', resources={
            'LiquidFuel': (360, 360, 5), 'Oxidizer': (440, 440, 5)})
        rhinos = [Part('Size3AdvancedEngine') for _ in range(2)]
        for engine in rhinos:
            engine.title = 'Kerbodyne KR-2L+ Rhino Liquid Fuel Engine'
        engine_plate = Part('EnginePlate4', children=rhinos)
        nose_tank = Part('DIRECT.STS.nose', resources={
            'LiquidFuel': (40, 40, 5), 'Oxidizer': (50, 50, 5)})
        et_tank = Part('DIRECT.STS.tank.stack', children=[nose_tank, engine_plate],
                       resources={'LiquidFuel': (100, 100, 5),
                                  'Oxidizer': (120, 120, 5)})
        separators, boosters = [], []
        for _ in range(2):
            motor = Part('PC.4Seg.RSRM', resources={'SolidFuel': (10, 10, 5)})
            nose_motor = Part('PC.Nose', resources={'SolidFuel': (1, 1, 5)})
            sep = Part('PC.RSRM.RadialDecoupler', children=[motor, nose_motor])
            et_tank.children.append(sep)
            separators.append(sep)
            boosters.extend([motor, nose_motor])
        et_decoupler = Part('STS.ET.decoupler', children=[et_tank])
        root = Part('benjee10.shuttle.aftFuselage', children=[nuke, orbiter_tank,
                                                              et_decoupler])
        parts = [root, nuke, orbiter_tank, et_decoupler, et_tank, nose_tank,
                 engine_plate, *rhinos, *separators, *boosters]
        vessel = SimpleNamespace(parts=SimpleNamespace(all=parts),
                                 reference_frame=None, mass=1000,
                                 control=SimpleNamespace(throttle=0))
        sc = SimpleNamespace(active_vessel=vessel, ut=1000)
        shuttle = Shuttle(SimpleNamespace(space_center=sc))

        self.assertEqual(shuttle.vehicle_kind, 'nuclear-solid')
        self.assertEqual(shuttle.et_decoupler, et_decoupler.decoupler)
        self.assertEqual(shuttle.booster_decouplers,
                         [p.decoupler for p in separators])
        self.assertEqual(shuttle.booster_kinds, ['solid', 'solid'])
        self.assertEqual(len(shuttle.et_engines), 2)
        self.assertEqual(len(shuttle.solid_boosters), 4)
        self.assertEqual(shuttle.groups, [[nuke.engine],
                                         [p.engine for p in rhinos]])
        self.assertGreater(shuttle.booster_nominal_burn_time(), 0)
        allocation = shuttle.allocation(max_g=3)
        expected = (sum(e.max_thrust for e in shuttle.solid_boosters) +
                    .5 * (allocation.thrust -
                          sum(e.max_thrust for e in shuttle.solid_boosters))) / (1000*9.80665)
        self.assertAlmostEqual(shuttle.commanded_g(allocation, .5), expected)

        shuttle._solid_boosters_depleted = lambda: True
        shuttle.update(max_g=3)
        self.assertTrue(shuttle.separated)
        self.assertTrue(all(p.decoupled for p in separators))
        self.assertTrue(shuttle.release_et())
        self.assertEqual(shuttle.groups, [[nuke.engine]])

    def test_autopilot_engagement_property_used_by_current_krpc(self):
        class AutoPilot:
            engaged = False

        autopilot = AutoPilot()
        set_autopilot_engaged(autopilot, True)
        self.assertTrue(autopilot.engaged)
        set_autopilot_engaged(autopilot, False)
        self.assertFalse(autopilot.engaged)

    def test_autopilot_recovery_disables_sas_and_reengages(self):
        class Control:
            sas = True
        class AutoPilot:
            engaged = False
            def engage(self):
                self.engaged = True

        control = Control()
        autopilot = AutoPilot()
        self.assertTrue(ensure_autopilot_engaged(control, autopilot))
        self.assertFalse(control.sas)
        self.assertTrue(autopilot.engaged)
        self.assertFalse(ensure_autopilot_engaged(control, autopilot))

    def test_control_release_restores_sas_even_if_autopilot_release_fails(self):
        control = SimpleNamespace(throttle=1, roll=.6, sas=False)
        with self.assertRaises(AttributeError):
            release_controls(control, SimpleNamespace())
        self.assertEqual(control.throttle, 0)
        self.assertEqual(control.roll, 0)
        self.assertTrue(control.sas)

    def test_booster_separation_restores_cruise_moment_budget(self):
        class Engine:
            def __init__(self):
                self.thrust_limit = 1.0
                self.active = True

        class Decoupler:
            def __init__(self):
                self.called = False
            def decouple(self):
                self.called = True

        boosters = [Engine() for _ in range(4)]
        sepmotors = [Engine() for _ in range(4)]
        decouplers = [Decoupler() for _ in range(4)]
        fake = SimpleNamespace(
            vehicle_kind="legacy-liquid", solid_boosters=[],
            liquid_boosters=boosters, sepmotors=sepmotors, booster_decouplers=decouplers,
            separated=False, booster_separation_ut=None, sc=SimpleNamespace(ut=123.0),
            core_groups=[[Engine()], [Engine()]], groups=[],
            cruise_control_moment_budget=456.0, control_moment_budget=0.0)
        Shuttle._separate_boosters(fake)
        self.assertTrue(fake.separated)
        self.assertEqual(fake.booster_separation_ut, 123.0)
        self.assertEqual(fake.groups, fake.core_groups)
        self.assertEqual(fake.control_moment_budget, 456.0)
        self.assertTrue(all(e.thrust_limit == 0 and not e.active for e in boosters))
        self.assertTrue(all(d.called for d in decouplers))

    def test_et_ancestor_is_not_a_booster_separator_and_topoff_is_serial(self):
        booster_tanks = [
            Part(f'BoosterTank-{i+1}', resources={
                'LiquidFuel': (10, 10, 1), 'Oxidizer': (10, 10, 1)})
            for i in range(4)]
        sep_motors = [Part('sepMotor1', resources={'SolidFuel': (5, 5, 1)})
                      for _ in range(4)]
        kodiaks = [Part('LiquidEngineRK-7') for _ in range(4)]
        cubs = [[Part('LiquidEngineRV-1') for _ in range(4)] for _ in range(4)]
        for part in kodiaks:
            part.title = 'RK-7 Kodiak'
        for branch in cubs:
            for part in branch:
                part.title = 'RV-1 Cub'
        separators = [
            Part('radialDecoupler1-2',
                 children=[tank, kodiak, *branch_cubs, sep])
            for tank, kodiak, branch_cubs, sep
            in zip(booster_tanks, kodiaks, cubs, sep_motors)]

        tanks = [Part('Size3LargeTank', resources={'LiquidFuel': (100, 100, 1),
                                                   'Oxidizer': (100, 100, 1)}),
                 Part('Size3To2Adapter.v2', resources={'LiquidFuel': (50, 50, 1),
                                                       'Oxidizer': (50, 50, 1)})]
        skiff = Part('LiquidEngineRE-I2', -3.8)
        wolfhound = Part('LiquidEngineRE-J10')
        wolfhound.title = 'RE-J10 Wolfhound'
        et_skippers = [Part('engineLargeSkipper.v2') for _ in range(4)]
        for part in et_skippers:
            part.title = 'RE-I5 Skipper'
        orbiter_tank = Part('mk3FuselageLFO.25',
                            resources={'LiquidFuel': (90, 100, 1),
                                       'Oxidizer': (90, 100, 1)})
        et = Part('radialDecoupler1-2',
                  children=tanks+separators+[wolfhound]+et_skippers)
        booster_engines = kodiaks + [p for branch in cubs for p in branch]
        all_parts = ([et, skiff, orbiter_tank]+tanks+separators+booster_tanks+
                     booster_engines+sep_motors+[wolfhound]+et_skippers)

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
        self.assertEqual(len(shuttle.boosters), 20)
        self.assertEqual(sum(e.part.name == 'LiquidEngineRK-7' for e in shuttle.boosters), 4)
        self.assertEqual(sum(e.part.name == 'LiquidEngineRV-1' for e in shuttle.boosters), 16)
        self.assertTrue(all(sep.engine not in shuttle.boosters for sep in sep_motors))
        self.assertEqual(len(shuttle.groups), 5)
        self.assertEqual(shuttle.groups[1], [wolfhound.engine])
        self.assertEqual(shuttle.groups[2], [p.engine for p in et_skippers[:2]])
        self.assertEqual(shuttle.groups[3], [p.engine for p in et_skippers[2:]])
        self.assertCountEqual(shuttle.groups[-1], [p.engine for p in booster_engines])

        bounds = shuttle.group_upper_bounds()
        self.assertEqual(len(bounds), 5)
        self.assertAlmostEqual(bounds[2], 1.0)
        self.assertAlmostEqual(bounds[3], .45)
        self.assertAlmostEqual(bounds[-1], 1.0)

        # As Wolfhound pressure performance recovers, it replaces Skipper thrust.
        wolfhound.engine.max_thrust = 300
        for part in et_skippers:
            part.engine.max_thrust = 110
        high_alt_bounds = shuttle.group_upper_bounds(20000)
        self.assertLess(high_alt_bounds[2], bounds[2])
        expected_cap = (shuttle.et_reference_thrust - 300) / (4 * 110)
        self.assertAlmostEqual(high_alt_bounds[2], expected_cap)
        self.assertAlmostEqual(high_alt_bounds[3], expected_cap)

        self.assertGreater(shuttle.liquid_booster_nominal_burn_time(), 0)
        vessel.mass = 10
        limited = shuttle.allocation(max_g=3)
        self.assertLessEqual(shuttle.commanded_g(limited), 3.000001)

        vessel.mass = 1000
        shuttle.update(max_g=3)
        booster_limits = [p.engine.thrust_limit for p in booster_engines]
        self.assertTrue(all(abs(x-booster_limits[0]) < 1e-12 for x in booster_limits))
        inner_limits = [p.engine.thrust_limit for p in et_skippers[:2]]
        outer_limits = [p.engine.thrust_limit for p in et_skippers[2:]]
        self.assertTrue(all(abs(x-inner_limits[0]) < 1e-12 for x in inner_limits))
        self.assertTrue(all(abs(x-outer_limits[0]) < 1e-12 for x in outer_limits))

        shuttle.top_off()
        shuttle.top_off()
        self.assertEqual(len(calls), 2)
        self.assertTrue(all(c[0] in tanks and c[1] is orbiter_tank for c in calls))
        self.assertFalse(shuttle.release_et())

        # All four liquid booster branches stay attached while fueled, then cut and
        # separate together as soon as any branch starts to lose propulsion.
        shuttle.transfers = {}
        shuttle.update(max_g=3)
        self.assertFalse(shuttle.separated)
        kodiaks[0].engine.has_fuel = False
        shuttle.update(max_g=3)
        self.assertTrue(shuttle.separated)
        self.assertTrue(all(p.decoupled for p in separators))
        self.assertTrue(all(not e.active and e.thrust_limit == 0
                            for e in [p.engine for p in booster_engines]))
        self.assertEqual(len(shuttle.groups), 4)


if __name__ == '__main__':
    unittest.main()
