"""kRPC adapter. Preflight is read-only; --launch explicitly enables control."""
import argparse
import csv
import math
import signal
import time
from datetime import datetime, timezone
from ..kspguard import GuardError, check_orbiter_integrity
from .allocation import Allocation, allocate
from .events import emit_event, emit_telemetry
from .geometry import (_combine, _cross, _dot, _norm, _unit, cross3, desired_orbiter_side, direction_error_deg, direction_for_pitch_heading, dot3, normalize3, pitch_heading_for_direction, project_perpendicular, roll_delta, roll_error_to_side, roll_for_heading, roll_for_roof, slew_roll, slew_value, slew_vector, smoothstep01, surface_axis_errors, surface_up_for_pitch_heading, wrap_roll)
from .guidance import (ROLL_RESET_GRACE, GUIDANCE_DEBUG, LAUNCH_TARGET_G, MAX_DYNAMIC_PRESSURE_PA, PITCH_SLEW_DEG_PER_SECOND, THROTTLE_SLEW_PER_SECOND, aoa_limit_deg, ascent_pitch, ascent_target_g, attitude_error_exceeded, dynamic_pressure_throttle, earthward_roll_target, final_phase_g, g_force_throttle, post_booster_master_throttle, gravity_turn_pitch, hold_turn_for_roll, maneuver_roll_rate, manual_roll_command, meco_ready, mission_roll_allowed, roll_attitude_settled, scheduled_turn_pitch, terminal_guidance, vertical_profile_pitch)
from .vehicle import (BOOSTER_MIN_G, BOOSTER_PAD_CAP, CONTROL_TORQUE_FRACTION, G0, OUTER_SKIPPER_PAD_CAP, SSME_TRIM_MAX, SSME_TRIM_MIN, Shuttle, body_roll_rate_deg_s, descendants, engine_has_fuel, engine_pitch_moment, engine_thrust, has_resource, is_cub_engine, is_kodiak_engine, is_separator_motor, is_skipper_engine, is_wolfhound_engine, limit_aero_roll, one, part_engine, resource_mass, resource_total, restore_aero_roll, roll_accel_authority_deg_s2, schedule_roll_authority, tanks_empty, thrust_axis_offset)



def validate_launch_vessel(vessel, space_center):
    if vessel.name not in ('STS', 'STS-N'):
        raise ValueError(f'Expected the active vessel to be named STS or STS-N; got {vessel.name!r}')
    if vessel.situation == space_center.VesselSituation.pre_launch:
        return
    # KSP can mark a pad stack LANDED after physics settles even while its
    # launch clamps remain attached. Permit that case only while nearly still.
    if (vessel.situation == space_center.VesselSituation.landed and
            len(vessel.parts.launch_clamps) >= 2 and
            vessel.flight(vessel.orbit.body.reference_frame).speed < 1.0):
        return
    raise ValueError('Launch requires an STS on the launch pad')


def set_autopilot_engaged(autopilot, engaged):
    """Support kRPC clients exposing engagement as either methods or a property."""
    method = getattr(autopilot, 'engage' if engaged else 'disengage', None)
    if callable(method):
        method()
    elif hasattr(autopilot, 'engaged'):
        autopilot.engaged = engaged
    else:
        raise AttributeError('AutoPilot has no engagement control')


def ensure_autopilot_engaged(control, autopilot):
    """Recover if KSP or another controller unexpectedly drops kRPC AutoPilot."""
    if control.sas:
        control.sas = False
    if getattr(autopilot, 'engaged', True):
        return False
    set_autopilot_engaged(autopilot, True)
    return True


def release_controls(control, autopilot):
    """Cut thrust and return attitude control to SAS, even if one write fails."""
    try:
        control.throttle = 0
        control.roll = 0
    finally:
        try:
            set_autopilot_engaged(autopilot, False)
        finally:
            control.sas = True


def pause_structural_failure(conn, control, autopilot):
    """Hand a damaged ascent back to KSP and save a checkpoint; KSP keeps running."""
    try:
        release_controls(control, autopilot)
    except Exception as exc:
        print(f'WARNING: control release during structural failure: {exc}')
    # Let the throttle-zero handoff settle on a physics frame before saving.
    for _ in range(10):
        try:
            if control.throttle <= 0.01:
                break
        except Exception:
            break
        time.sleep(0.1)
    try:
        name = 'ksl-structural-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        conn.space_center.save(name)
        print(f'Structural failure checkpoint: {name}')
    except Exception as exc:
        print(f'WARNING: could not save structural failure checkpoint: {exc}')


def _terminate_as_exception(signum, _frame):
    raise SystemExit(128 + signum)

def run(args):
    import krpc
    with krpc.connect(name='STS Launch Program', address=args.address,
                      rpc_port=args.rpc_port, stream_port=args.stream_port) as conn:
        v = conn.space_center.active_vessel
        if args.launch:
            validate_launch_vessel(v, conn.space_center)
        shuttle = Shuttle(conn)
        if shuttle.vehicle_kind == 'orbiter-shuttle':
            check_orbiter_integrity({"ready": True,
                                     "partNames": [part.name for part in v.parts.all]})
        body = v.orbit.body
        preflight_altitude = v.flight(body.reference_frame).mean_altitude
        preflight_target_g = ascent_target_g(
            preflight_altitude, body.atmosphere_depth, args.max_g, args.launch_g,
            args.booster_min_g)
        result = shuttle.allocation(
            max_g=args.max_g, target_g=preflight_target_g, altitude=preflight_altitude)
        booster_names = ', '.join(sorted({e.part.title for e in shuttle.boosters}))
        booster_max_thrust = sum(e.max_thrust * e.thrust_limit for e in shuttle.boosters)
        kinds = ','.join(shuttle.booster_kinds)
        booster_lf = sum(resource_total(parts, 'LiquidFuel', maximum=True)
                         for parts in shuttle.booster_branch_parts)
        booster_ox = sum(resource_total(parts, 'Oxidizer', maximum=True)
                         for parts in shuttle.booster_branch_parts)
        booster_sf = sum(resource_total([engine.part], 'SolidFuel', maximum=True)
                         for engine in shuttle.solid_boosters)
        if shuttle.vehicle_kind == 'nuclear-solid':
            print(f'{v.name}: {v.mass/1000:.2f} t; engine groups Orbiter-J-N500 / ET-2xRhino / fixed twin-RSRM')
        elif shuttle.vehicle_kind == 'orbiter-shuttle':
            booster_type = ('Clydesdale SRB' if shuttle.solid_boosters else
                            'Size2 liquid booster')
            print(f'{v.name}: {v.mass/1000:.2f} t; engine groups '
                  f'Orbiter-3xSSME / Booster-2x{booster_type}')
            if shuttle.solid_boosters:
                print(f'SRB separation motors detected: {len(shuttle.sepmotors)}')
        else:
            print(f'{v.name}: {v.mass/1000:.2f} t; engine groups '
                  'Orbiter-Skiff / ET-Wolfhound / ET-Skipper-inner2 / ET-Skipper-outer2 / '
                  'Booster-4x(Kodiak+4xCub)')
        print(f'Detected boosters: {len(shuttle.booster_branches)} branch(es), '
              f'{len(shuttle.boosters)} engine(s), kinds={kinds}: {booster_names}; '
              f'{booster_max_thrust/1000:.0f} kN max; LF/Ox/SF={booster_lf:.0f}/{booster_ox:.0f}/{booster_sf:.0f}')
        fixed_thrust, _ = shuttle._solid_booster_state()
        full_thrust = (sum(engine.max_thrust for group in shuttle.groups for engine in group) +
                       fixed_thrust)
        full_throttle_g = full_thrust / (v.mass * G0) if v.mass > 0 else 0.0
        if shuttle.vehicle_kind == 'orbiter-shuttle' and shuttle.solid_boosters:
            ignition_thrust = (sum(engine.max_thrust for engine in shuttle.core_groups[0]) +
                               fixed_thrust)
            ignition_g = ignition_thrust / (v.mass * G0) if v.mass > 0 else 0.0
            print(f'SSMEs at full limiter plus configured SRB thrust: {ignition_g:.2f} G')
        if shuttle.vehicle_kind == 'nuclear-solid':
            print(f"Orbiter J-N500 / ET Rhino thrust: {shuttle.skiff.max_thrust/1000:.0f}/"
                  f"{sum(e.max_thrust for e in shuttle.et_engines)/1000:.0f} kN; "
                  f"fixed SRB thrust {fixed_thrust/1000:.0f} kN")
        elif shuttle.vehicle_kind == 'orbiter-shuttle':
            ssme_thrust = sum(e.max_thrust for e in shuttle.core_groups[0])
            oms_thrust = sum(e.max_thrust for e in shuttle.orbiter_engines)
            print(f"Orbiter SSMEs / paired AJ10 OMS: {ssme_thrust/1000:.0f}/"
                  f"{oms_thrust/1000:.0f} kN; external-tank propellant "
                  f"LF/Ox={shuttle.et_resource_amount('LiquidFuel'):.0f}/"
                  f"{shuttle.et_resource_amount('Oxidizer'):.0f}")
        else:
            skipper_cap = shuttle.et_skipper_upper_bound()
            skipper_thrust = sum(e.max_thrust for e in shuttle.et_skippers)
            print(f'Wolfhound current/vacuum max thrust: {shuttle.wolfhound.max_thrust/1000:.1f}/'
                  f'{shuttle.wolfhound.max_vacuum_thrust/1000:.1f} kN; '
                  f'ET Skipper common cap {skipper_cap*100:.1f}% '
                  f'({skipper_thrust*skipper_cap/1000:.0f} kN of {skipper_thrust/1000:.0f} kN)')
        print(f'Full-throttle physical thrust acceleration at current pressure: {full_throttle_g:.2f} G')
        print(f'Preflight throttles: {result.throttles}; residual {result.residual:.0f} N m; '
              f'commanded {shuttle.commanded_g(result):.2f} G / target {preflight_target_g:.2f} G')
        print(f'Pad group caps: {tuple(round(x, 3) for x in shuttle.group_upper_bounds(preflight_altitude))}; '
              f'booster floor {args.booster_min_g:.2f} G; throttle slew {args.throttle_slew:.2f}/s')
        if shuttle.boosters:
            nominal_burn = shuttle.booster_nominal_burn_time()
            print(f'Booster full-throttle burn estimate at current pressure: {nominal_burn:.1f} s')
        if not args.launch:
            emit_event('preflight', ok=True)
            print('Read-only preflight complete. Use --launch to fly.')
            return
        if body.name != 'Kerbin':
            raise ValueError('Default ascent guidance is designed for Kerbin')
        if args.orbit < body.atmosphere_depth + 5000:
            raise ValueError('Target apoapsis must clear the atmosphere by at least 5 km')
        flight = v.flight(body.reference_frame)
        from .attitude import CustomAutopilot
        # Own attitude controller (not the kRPC AutoPilot): see shuttle/attitude.py.
        ap = CustomAutopilot(conn, v, address=args.address, rpc_port=args.rpc_port,
                             stream_port=args.stream_port)
        control = v.control
        started = conn.space_center.ut
        state = 'ASCENT'
        next_report = 0
        mission_roll = 0.0
        if args.heading is None:
            lat = math.radians(flight.latitude)
            cos_i = math.cos(math.radians(args.inclination))
            if abs(cos_i) > math.cos(lat):
                raise ValueError('Target inclination is below the launch latitude')
            r_mid = body.equatorial_radius + 0.5*(args.orbit+args.meco_periapsis)
            v_orb = math.sqrt(body.gravitational_parameter/r_mid)
            v_rot = body.rotational_speed*body.equatorial_radius*math.cos(lat)
            azimuth = math.asin(cos_i/math.cos(lat))
            args.heading = math.degrees(math.atan2(v_orb*math.sin(azimuth)-v_rot,
                                                   v_orb*math.cos(azimuth))) % 360
            print(f'Launch heading {args.heading:.1f} deg for {args.inclination:.1f} deg inclination')
        # Headings are kept continuous (unwrapped) so slews never go the long way
        # round: a polar launch heads ~356 deg, i.e. -4 deg from the 90 deg pad heading.
        target_heading = 90.0 + wrap_roll(args.heading-90.0)
        args.heading = 90.0  # pad orientation; slewed to target_heading after the roll settles
        post_booster_roll = roll_for_heading(args.heading)
        initial_pitch = 89.0
        commanded_pitch = initial_pitch
        launch_roll = None
        mission_roll_commanded = False
        mission_roll_settled = False
        roll_settle_since = None
        initial_roll_set = False
        liftoff_ut = None
        baseline_part_count = None
        meco_ut = None
        peg_dv = None
        peg_hold = None
        nrf = body.non_rotating_reference_frame
        upper_autotune = False
        roll_reset = False
        roll_reset_ut = None
        upright_since = None
        ap_owns_roll = False
        upright_configured = False
        et_cleared = False
        sep_handled = False
        roll_maneuver = None
        shuttle.post_sep_roll_done_ut = None
        roll_rate = 0.0
        body_roll_rate = 0.0
        last_roll_angle = None
        last_roll_ut = None
        saved_auto_tune = None
        saved_roll_pid = None
        roll_surface_states = []
        roll_accel_reference = 0.0
        turn_start_altitude = args.turn_start
        # Burnout altitude for the post-separation climb profile: low in the target
        # band, where the insertion point of a low-perigee ellipse lies.
        insertion_altitude = args.meco_periapsis + 0.4*(args.orbit-args.meco_periapsis)
        turn_end_altitude = args.turn_end
        old_sigterm_handler = signal.getsignal(signal.SIGTERM)
        signal.signal(signal.SIGTERM, _terminate_as_exception)
        try:
            control.throttle = 0
            control.sas = False
            ap.reference_frame = v.surface_reference_frame
            initial_direction = direction_for_pitch_heading(initial_pitch, args.heading)
            initial_up = surface_up_for_pitch_heading(initial_pitch, args.heading)
            roof_direction = conn.space_center.transform_direction(
                (0, 0, -1), v.reference_frame, v.surface_reference_frame)
            launch_roll = roll_for_roof(initial_direction, initial_up, roof_direction)
            ap.auto_tune = True
            ap.time_to_peak = (1.0, args.roll_time_to_peak, 1.0)
            ap.overshoot = (0.01, args.roll_overshoot, 0.01)
            ap.max_angular_velocity = (1.0, math.radians(args.roll_rate), 1.0)
            ap.roll_attenuation_angle = args.roll_attenuation
            ap.target_smoothing_time = 0.0
            ap.target_direction = initial_direction
            set_autopilot_engaged(ap, True)
            ap.set_direction_and_up(initial_direction, initial_up, launch_roll)
            launch_altitude = flight.mean_altitude
            print(f'Pad roll {launch_roll:.1f} deg; physical heads-down roll starts after '
                  f'+{args.roll_start:.0f} m climb; manual roll cap {args.roll_rate:.1f} deg/s; '
                  f'gravity turn uses prograde-AoA guidance; flip to spaceward {args.roll_reset_delay:.0f} s after booster separation')
            clamps = list(v.parts.launch_clamps)
            # The pad stack is the last intact reference before staging. A wing
            # joint can fail at clamp release, before the first ascent sample.
            prelaunch_part_count = len(v.parts.all)
            if shuttle.vehicle_kind == 'orbiter-shuttle' and shuttle.solid_boosters:
                if control.stage_lock:
                    raise RuntimeError('Staging is locked; cannot ignite Clydesdales and release clamps')
                next_stage = control.current_stage - 1
                if next_stage < 0 or any(engine.part.stage != next_stage
                                         for engine in shuttle.solid_boosters):
                    raise RuntimeError(
                        'Clydesdales are not in the next stage; refusing to release the clamps')
                if any(not engine.can_restart for group in shuttle.core_groups
                       for engine in group):
                    raise RuntimeError('Core engine cannot be ignited independently before booster staging')
            if clamps:
                authority, budget, spool, roll_accel_reference = shuttle.calibrate_control_authority(control)
                launch_target_g = ascent_target_g(
                    flight.mean_altitude, body.atmosphere_depth, args.max_g, args.launch_g,
                    args.booster_min_g)
                result = shuttle.allocation(
                    max_g=args.max_g, target_g=launch_target_g, altitude=flight.mean_altitude)
                print(f'Launch control authority: {authority/1000:.0f} kN m; '
                      f'moment budget {budget/1000:.0f} kN m; spool {spool*100:.0f}%; '
                      f'roll accel {roll_accel_reference:.1f} deg/s^2')
            core_group_count = len(shuttle.core_groups)
            full_ssme_before_srb = (
                shuttle.vehicle_kind == 'orbiter-shuttle' and
                bool(shuttle.solid_boosters))
            for group, u in zip(shuttle.core_groups, result.throttles[:core_group_count]):
                for e in group:
                    e.thrust_limit = args.ssme_limit if full_ssme_before_srb else u
                    e.active = True
            control.throttle = 1
            core_engines = [engine for group in shuttle.core_groups for engine in group]
            expected_core_thrust = sum(engine.max_thrust for engine in core_engines)
            if full_ssme_before_srb:
                expected_core_thrust *= args.ssme_limit
            if not full_ssme_before_srb:
                expected_core_thrust = sum(
                    engine.max_thrust * throttle
                    for group, throttle in zip(shuttle.core_groups,
                                               result.throttles[:core_group_count])
                    for engine in group)
            if expected_core_thrust <= 0:
                raise RuntimeError('Launch allocation leaves the core engines at zero thrust')
            ignition_fraction = .95 if full_ssme_before_srb else .5
            ignition_deadline = time.monotonic() + 8.0
            while time.monotonic() < ignition_deadline:
                if conn.space_center.active_vessel != v:
                    raise RuntimeError('Active vessel changed during core-engine ignition')
                core_thrust = sum(engine_thrust(engine) for engine in core_engines)
                if (all(engine.active for engine in core_engines) and
                        core_thrust >= expected_core_thrust * ignition_fraction):
                    break
                time.sleep(.05)
            else:
                raise RuntimeError(
                    f'Core engines did not reach {ignition_fraction:.0%} of required thrust; '
                    'boosters remain unignited and clamps remain attached')
            if full_ssme_before_srb:
                emit_event('ignition')
                print(f'SSMEs verified at fixed limiter on clamps: '
                      f'{core_thrust/1000:.0f}/{expected_core_thrust/1000:.0f} kN; '
                      f'holding for {args.ssme_lead_time:.1f} s before booster ignition')
            else:
                emit_event('ignition', thrust_kn=core_thrust/1000)
                print(f'Core engines ignited on clamps: {core_thrust/1000:.0f} kN; '
                      f'holding for {args.ssme_lead_time:.1f} s before booster ignition')

            # Make the liquid-engine-first sequence observable and give the
            # SSMEs a real lead interval while the launch clamps still hold.
            lead_deadline = time.monotonic() + args.ssme_lead_time
            while time.monotonic() < lead_deadline:
                if conn.space_center.active_vessel != v:
                    raise RuntimeError('Active vessel changed during SSME lead interval')
                if not all(engine.active for engine in core_engines):
                    raise RuntimeError('SSME shut down during lead interval; boosters remain unignited')
                if (full_ssme_before_srb and
                        sum(engine_thrust(engine) for engine in core_engines) <
                        expected_core_thrust * .95):
                    raise RuntimeError('SSMEs fell below 95% thrust during lead interval; '
                                       'boosters remain unignited')
                time.sleep(min(.05, max(0.0, lead_deadline - time.monotonic())))

            if shuttle.vehicle_kind == 'orbiter-shuttle' and shuttle.solid_boosters:
                for engine in shuttle.sepmotors:
                    engine.thrust_limit = 0.0
                    engine.active = False
                print(f'{len(shuttle.sepmotors)} separator motors inhibited until SRB separation')
                # The next stage ignites the Clydesdales and releases the clamps;
                # Mites share that stage, so they are disabled again immediately.
                control.activate_next_stage()
                if conn.space_center.active_vessel != v:
                    raise RuntimeError('Active vessel changed during booster ignition staging')
                # They share the Clydesdale ignition stage on this craft, so
                # immediately shut them down while the configured zero limiter
                # protects their propellant until booster separation.
                for engine in shuttle.sepmotors:
                    engine.thrust_limit = 0.0
                    engine.active = False
                booster_deadline = time.monotonic() + 5.0
                expected_booster_thrust = sum(engine.max_thrust
                                              for engine in shuttle.solid_boosters)
                while time.monotonic() < booster_deadline:
                    booster_thrust = sum(engine_thrust(engine)
                                         for engine in shuttle.solid_boosters)
                    if (all(engine.active for engine in shuttle.solid_boosters) and
                            booster_thrust >= expected_booster_thrust * .5):
                        break
                    time.sleep(.05)
                else:
                    raise RuntimeError('Clydesdale ignition failed after stage activation')
                expected_after_clamps = prelaunch_part_count - len(clamps)
                actual_after_clamps = len(v.parts.all)
                if actual_after_clamps < expected_after_clamps:
                    control.throttle = 0
                    raise RuntimeError(
                        'Part loss at clamp release: expected at least '
                        f'{expected_after_clamps} parts after {len(clamps)} clamps, '
                        f'found {actual_after_clamps}')
                baseline_part_count = expected_after_clamps
                emit_event('liftoff')
                print(f'Clydesdales ignited; clamps released; booster thrust '
                      f'{booster_thrust/1000:.0f} kN')
            else:
                for group, u in zip(shuttle.groups[core_group_count:],
                                    result.throttles[core_group_count:]):
                    for engine in group:
                        engine.thrust_limit = u
                        engine.active = True
                for engine in shuttle.boosters:
                    if engine not in shuttle.solid_boosters:
                        engine.active = True
                for clamp in clamps:
                    clamp.release()
            with open(args.telemetry, 'w', newline='') as log:
                writer = csv.writer(log)
                writer.writerow(['ut', 'state', 'altitude_m', 'apoapsis_m', 'periapsis_m',
                                 'mass_kg', 'vertical_speed_mps', 'horizontal_speed_mps',
                                 'dynamic_pressure_pa', 'actual_g', 'booster_elapsed_s',
                                 'booster_thrust_N', 'booster_liquid_fuel', 'booster_oxidizer',
                                 'booster_solid_fuel', 'booster_propellant_fraction',
                                 'booster_full_throttle_remaining_s', 'commanded_g',
                                 'et_liquid_fuel', 'et_oxidizer', 'throttles', 'residual_Nm',
                                 'feasible', 'target_roll_deg', 'nose_error_deg',
                                 'autopilot_error_deg', 'actual_roll_deg', 'roll_error_deg', 'roll_rate_deg_s',
                                 'body_roll_rate_deg_s', 'roll_control', 'pitch_error_deg', 'heading_error_deg',
                                 'roll_control_oscillation',
                                 'actual_pitch_deg', 'actual_heading_deg', 'target_pitch_deg',
                                 'control_moment_budget_Nm', 'engine_limit_state',
                                 'engine_gimbal_actuation', 'available_torque_Nm',
                                 'orbit_inclination_deg', 'guidance_dv_mps'])
                while True:
                    tick = time.monotonic()
                    if conn.space_center.active_vessel != v:
                        raise RuntimeError('Active vessel changed')
                    recovered_autopilot = ensure_autopilot_engaged(control, ap)
                    if recovered_autopilot:
                        print('AutoPilot unexpectedly disengaged; re-engaged')
                    if conn.space_center.ut-started > args.timeout:
                        raise RuntimeError('Mission timeout')
                    altitude = flight.mean_altitude
                    altitude_gain = max(0.0, altitude-launch_altitude)
                    if not initial_roll_set and v.situation != conn.space_center.VesselSituation.pre_launch:
                        initial_roll_set = True
                        liftoff_ut = conn.space_center.ut
                        print(f'Liftoff: holding pad roll {launch_roll:.1f} deg; physical heads-down roll deferred')
                    earthward_mode, reset_due = earthward_roll_target(
                        initial_roll_set, shuttle.booster_separation_ut,
                        conn.space_center.ut, args.roll_reset_delay)
                    roll_allowed = mission_roll_allowed(
                        initial_roll_set, altitude_gain, args.roll_start)
                    if roll_allowed and not mission_roll_commanded:
                        saved_auto_tune = ap.auto_tune
                        saved_roll_pid = ap.roll_pid_gains
                        roll_surface_states = limit_aero_roll(v)
                        ap.auto_tune = False
                        ap.roll_pid_gains = (0.0, 0.0, 0.0)
                        ap.target_roll = float('nan')
                        mission_roll_commanded = True
                        # Roll once, straight to the launch-azimuth roof orientation: with
                        # the pad heading kept, the roll stopped short and had to roll
                        # again when the heading slewed (94 deg on a polar launch).
                        args.heading = target_heading
                        print(f'Physical orbiter-side roll started: earthward; '
                              f'elevon roll enabled at 60% authority')
                    roll_active = mission_roll_commanded and state in ('ASCENT', 'ORBITER_ASCENT')
                    if recovered_autopilot and roll_active:
                        ap.auto_tune = False
                        ap.roll_pid_gains = (0.0, 0.0, 0.0)
                        ap.target_roll = float('nan')
                    target_roll = launch_roll if not mission_roll_commanded else None
                    if (reset_due and not roll_reset and
                            flight.dynamic_pressure > args.upright_max_q):
                        # Rolling a winged orbiter in thick air snaps a wing:
                        # wait for the air to thin before the upright roll.
                        reset_due = False
                        earthward_mode = True
                    elif roll_reset:
                        earthward_mode = False
                    if reset_due and not roll_reset:
                        roll_reset = True
                        roll_reset_ut = conn.space_center.ut
                        print(f'Post-booster physical roll: orbiter side now targets spaceward '
                              f'(q {flight.dynamic_pressure:.0f} Pa)')
                    roll_requires_authority = False
                    if state == 'ASCENT':
                        roll_requires_authority = mission_roll_commanded and not mission_roll_settled
                        if mission_roll_settled:
                            heading_goal = target_heading
                            if flight.horizontal_speed > 450.0 and peg_dv is None:
                                # (PEG owns the heading once it is active; two slewed heading
                                # laws cancelled and left ~50 m/s of cross-track uncorrected.)
                                # The velocity vector lags the nose heading by many
                                # degrees at speed, so steer on the measured surface
                                # velocity azimuth: lead the nose past the target by a
                                # gain times the azimuth error.
                                svel = conn.space_center.transform_direction(
                                    v.velocity(body.reference_frame), body.reference_frame,
                                    v.surface_reference_frame)
                                vel_azimuth = math.degrees(math.atan2(svel[2], svel[1])) % 360
                                # Desired surface azimuth at the current speed so the
                                # *inertial* velocity azimuth equals the one that gives
                                # the target inclination.
                                lat_now = math.radians(flight.latitude)
                                v_rot_now = (body.rotational_speed*
                                             (body.equatorial_radius+flight.mean_altitude)*
                                             math.cos(lat_now))
                                east_in = svel[2]+v_rot_now
                                vh_in = math.hypot(east_in, svel[1])
                                orbit_inc = math.degrees(v.orbit.inclination)
                                aim_inclination = args.inclination + max(-1.5, min(
                                    1.5, 0.5*(args.inclination-orbit_inc)))
                                a_inertial = math.asin(max(-1.0, min(1.0,
                                    math.cos(math.radians(aim_inclination))/
                                    max(0.05, math.cos(lat_now)))))
                                desired = math.degrees(math.atan2(
                                    vh_in*math.sin(a_inertial)-v_rot_now,
                                    vh_in*math.cos(a_inertial))) % 360
                                heading_goal = args.heading + wrap_roll(desired-args.heading)
                                azimuth_error = wrap_roll(desired-vel_azimuth)
                                # Keep side loads small while the air is thick.
                                lead_limit = (8.0 if flight.dynamic_pressure > 8000.0
                                              else 15.0)
                                # Runs yy/zz overshot the plane by ~1.5 deg at 55-70 km with
                                # a 1.5x / 30 deg lead; a softer lead converges without it.
                                heading_goal += max(-lead_limit,
                                                    min(lead_limit, 1.0*azimuth_error))
                            if peg_dv is None:
                                args.heading = slew_value(
                                args.heading, heading_goal,
                                3.0 if flight.dynamic_pressure < 1500.0
                                else args.heading_slew if flight.dynamic_pressure < 5000.0
                                else min(args.heading_slew, 0.7), 1/args.hz)
                        if roll_requires_authority or altitude < turn_start_altitude:
                            raw_pitch = initial_pitch
                        else:
                            raw_pitch = scheduled_turn_pitch(
                                altitude, args.orbit, flight.vertical_speed,
                                flight.horizontal_speed, flight.dynamic_pressure,
                                initial_pitch)
                        # Apply the MECO energy-build pitch through the same
                        # slew limiter as the gravity turn. A post-slew clamp
                        # can command an instantaneous 89-to-0 degree jump.
                        pitch_slew_rate = args.pitch_slew
                        if (v.orbit.apoapsis_altitude >= 0.8*args.orbit and
                                v.orbit.periapsis_altitude < args.meco_periapsis):
                            # Apoapsis is approaching target: spend the remaining
                            # energy on horizontal speed. Damp the climb with
                            # vertical speed and keep the burnout altitude low in the
                            # target band (the insertion point of a low-perigee ellipse
                            # lies below the mid-altitude), instead of letting Ap run away.
                            band_center = (args.meco_periapsis +
                                           0.4*(args.orbit-args.meco_periapsis))
                            ap_excess = max(0.0, v.orbit.apoapsis_altitude-args.orbit)
                            hold = args.meco_pitch + 5.0 \
                                - 0.2*flight.vertical_speed \
                                + max(-10.0, min(10.0, (altitude-band_center)/-1500.0)) \
                                - min(15.0, 1.5*ap_excess/1000.0)
                            raw_pitch = min(raw_pitch, max(-45.0, min(5.0, hold)))
                            pitch_slew_rate = max(args.pitch_slew, 3.0)
                        if shuttle.separated and shuttle.post_sep_roll_done_ut is None:
                            # Hold attitude while rolling upright.
                            raw_pitch = commanded_pitch
                        elif shuttle.separated:
                            r_now = body.equatorial_radius+altitude
                            lat_now = math.radians(flight.latitude)
                            v_rot = body.rotational_speed*r_now*math.cos(lat_now)
                            vh_inertial = flight.horizontal_speed+v_rot
                            r_ins = body.equatorial_radius+insertion_altitude
                            ra = body.equatorial_radius+args.orbit
                            rp = body.equatorial_radius+args.meco_periapsis
                            mu = body.gravitational_parameter
                            vh_ins = math.sqrt(2.0*mu*ra*rp/(ra+rp))/r_ins
                            # Radial speed of the target orbit at the insertion radius
                            # (ascending branch): 90x60 needs ~+50 m/s at 72 km, not 0.
                            vr_ins = math.sqrt(max(0.0, 2.0*mu*(1.0/r_ins - 1.0/(ra+rp))
                                                   - vh_ins**2))
                            raw_pitch = vertical_profile_pitch(
                                altitude, flight.vertical_speed, vh_inertial, r_now,
                                mu, max(flight.g_force, 0.3)*G0,
                                insertion_altitude, vh_ins,
                                insertion_vertical_speed=vr_ins)
                            pitch_slew_rate = max(args.pitch_slew, 2.0)
                        peg_dv = None
                        energy_done = False
                        peg_phase_g = None
                        if (shuttle.separated and
                                v.orbit.apoapsis_altitude > args.orbit+90000.0 and
                                v.orbit.periapsis_altitude > 0):
                            control.throttle = 0
                            raise RuntimeError(
                                f'Guidance runaway: Ap {v.orbit.apoapsis_altitude/1000:.0f} km; engines cut')
                        if shuttle.separated and flight.speed > 1400.0:
                            try:
                                sc = conn.space_center
                                pos_n = v.position(nrf)
                                vel_n = v.velocity(nrf)
                                surf_vel = sc.transform_direction(
                                    v.velocity(body.reference_frame), body.reference_frame, nrf)
                                rot_vel = _combine((1.0, vel_n), (-1.0, surf_vel))
                                accel = max(flight.g_force, 0.3)*G0
                                d_n, peg_dv, peg_tgo = terminal_guidance(
                                    pos_n, vel_n, rot_vel, body.gravitational_parameter,
                                    body.equatorial_radius, args.orbit, args.meco_periapsis,
                                    # Steer to the true target plane: an exaggerated plane
                                    # target (target + 2x error) never converges, so the
                                    # dv stalled ~120 m/s and the energy cutoff fired with
                                    # the wrong shape (runs yy/zz 85x65 @34.9).
                                    args.inclination,
                                    accel)
                                d_s = sc.transform_direction(d_n, nrf, v.surface_reference_frame)
                                peg_pitch, peg_heading = pitch_heading_for_direction(d_s)
                                peg_heading = target_heading + wrap_roll(peg_heading-target_heading)
                                if peg_tgo > 4.0 or peg_hold is None:
                                    peg_hold = (max(-60.0, min(25.0, peg_pitch)),
                                                max(target_heading-40.0,
                                                    min(target_heading+40.0, peg_heading)))
                                raw_pitch = peg_hold[0]
                                args.heading = slew_value(args.heading, peg_hold[1],
                                                          3.0, 1/args.hz)
                                pitch_slew_rate = max(pitch_slew_rate, 4.0)
                                target_sma = body.equatorial_radius + 0.5*(args.orbit+args.meco_periapsis)
                                sma_now = v.orbit.semi_major_axis
                                energy_dv = ((target_sma-sma_now)*body.gravitational_parameter
                                             / (2.0*sma_now*sma_now*max(1.0, v.orbit.speed)))
                                energy_done = sma_now >= target_sma
                                if int(conn.space_center.ut*2) % 4 == 0:
                                    g = GUIDANCE_DEBUG
                                    print(f'PEG: dv {peg_dv:.1f} (radial {g["radial"]:.1f}, along '
                                          f'{g["along"]:.1f}, cross {g["cross"]:.1f}); vr {g["vr"]:.1f}/'
                                          f'{g["vr_target"]:.1f}, vh {g["vh"]:.1f}/{g["vh_target"]:.1f}; '
                                          f'sma {(sma_now-target_sma)/1000:+.1f} km')
                                # Taper to 0.15 G for the last metres per second so the
                                # 10 Hz cutoff resolves ~0.15 m/s (~0.2 km of periapsis).
                                peg_phase_g = max(0.15 if peg_dv < 8.0 else 0.5,
                                                  min(args.upper_g,
                                                      min(peg_dv, max(0.0, energy_dv))/5.0/G0))
                            except Exception:
                                peg_dv = None
                        commanded_pitch = slew_value(
                            commanded_pitch, raw_pitch,
                            pitch_slew_rate, 1/args.hz)
                        pitch = commanded_pitch
                        target_pitch = pitch
                        target_direction = direction_for_pitch_heading(pitch, args.heading)
                        # A small PEG residual or target semi-major axis is
                        # insufficient: a 100 km Ap / underground Pe trajectory
                        # can satisfy energy_done while being nowhere near orbit.
                        # Do not release the ET until the measured *two* apsides
                        # reach the executable handoff orbit. Depletion below is
                        # separately reported as a failed/partial insertion.
                        if (shuttle.separated and
                                meco_ready(v.orbit.apoapsis_altitude,
                                           v.orbit.periapsis_altitude,
                                           args.orbit-3000.0,
                                           args.meco_periapsis-300.0)):
                            control.throttle = 0
                            state = 'COAST'
                            emit_event('meco', apoapsis_m=v.orbit.apoapsis_altitude, periapsis_m=v.orbit.periapsis_altitude)
                            print(f'MECO: Ap {v.orbit.apoapsis_altitude/1000:.1f} km; '
                                  f'Pe {v.orbit.periapsis_altitude/1000:.1f} km')
                        elif shuttle.separated and tanks_empty(shuttle):
                            control.throttle = 0
                            state = 'COAST'
                            emit_event('meco', depleted=True, apoapsis_m=v.orbit.apoapsis_altitude, periapsis_m=v.orbit.periapsis_altitude)
                            print(f'MECO (propellant depleted): Ap {v.orbit.apoapsis_altitude/1000:.1f} km; '
                                  f'Pe {v.orbit.periapsis_altitude/1000:.1f} km')
                    elif state == 'ORBITER_ASCENT':
                        target_pitch = 5
                        target_direction = direction_for_pitch_heading(5, args.heading)
                        control.throttle = min(1, max(.05, (args.orbit-v.orbit.apoapsis_altitude)/5000))
                        if v.orbit.apoapsis_altitude >= args.orbit:
                            control.throttle = 0
                            state = 'COAST'
                    elif state == 'COAST':
                        target_pitch = None
                        control.throttle = 0
                        if not shuttle.separated:
                            raise RuntimeError('Target reached before booster separation')
                        if meco_ut is None:
                            meco_ut = conn.space_center.ut
                        if shuttle.et_attached and conn.space_center.ut-meco_ut < args.et_sep_delay:
                            time.sleep(1/args.hz)
                            continue
                        if shuttle.et_attached and not shuttle.release_et():
                            time.sleep(1/args.hz)
                            continue
                        if not et_cleared:
                            # Translate the orbiter roof-ward (control.up = vessel -z, away
                            # from the tank) on RCS so it cannot touch the ET.
                            control.rcs = True
                            control.up = 1.0
                            clear_until = time.monotonic() + args.et_clear_time
                            while time.monotonic() < clear_until:
                                time.sleep(0.05)
                            control.up = 0.0
                            control.rcs = False
                            et_cleared = True
                            print(f'Translated away from the ET for {args.et_clear_time:.0f} s on RCS')
                            if shuttle.vehicle_kind == 'orbiter-shuttle':
                                check_orbiter_integrity({
                                    "ready": True,
                                    "partNames": [part.name for part in v.parts.all],
                                })
                        if args.meco_periapsis < args.orbit-5000:
                            o = v.orbit
                            print(f'Target orbit reached: Ap {o.apoapsis_altitude/1000:.1f} km; '
                                  f'Pe {o.periapsis_altitude/1000:.1f} km; '
                                  f'inclination {math.degrees(o.inclination):.2f} deg')
                            return
                        ap.reference_frame = v.orbital_reference_frame
                        # Orbital-frame x points anti-radially (toward the body).
                        roll_up_reference = (-1, 0, 0)
                        target_direction = (0, 1, 0)
                        radius = body.equatorial_radius+v.orbit.apoapsis_altitude
                        dv = max(0, math.sqrt(body.gravitational_parameter/radius)-math.sqrt(body.gravitational_parameter*(2/radius-1/v.orbit.semi_major_axis)))
                        orbital_thrust = sum(
                            engine.max_thrust for engine in
                            getattr(shuttle, 'orbiter_engines', [shuttle.skiff]))
                        acceleration = orbital_thrust/v.mass
                        if acceleration <= 0:
                            raise RuntimeError('Orbiter engines have no thrust for insertion')
                        if v.orbit.time_to_apoapsis <= dv/acceleration/2+1:
                            state = 'INSERTION'
                    elif state == 'INSERTION':
                        target_pitch = None
                        target_direction = (0, 1, 0)
                        control.throttle = min(1, max(.03, (args.orbit-v.orbit.periapsis_altitude)/10000))
                        if v.orbit.periapsis_altitude >= args.orbit-1000:
                            emit_event('insertion', apoapsis_m=v.orbit.apoapsis_altitude, periapsis_m=v.orbit.periapsis_altitude)
                            print('Orbital insertion complete')
                            return
                        if flight.mean_altitude < body.atmosphere_depth:
                            raise RuntimeError('Insertion trajectory reentered atmosphere')
                    booster_elapsed = (None if liftoff_ut is None else
                                       max(0.0, conn.space_center.ut-liftoff_ut))
                    if shuttle.et_attached:
                        target_g = ascent_target_g(
                            altitude, body.atmosphere_depth, args.max_g, args.launch_g,
                            args.booster_min_g if not shuttle.separated else 0.0)
                        phase_max_g = args.max_g
                        if state == 'ASCENT' and shuttle.separated:
                            phase_max_g = final_phase_g(
                                v.orbit.periapsis_altitude, args.meco_periapsis,
                                min(args.max_g, args.upper_g))
                            if peg_phase_g is not None:
                                phase_max_g = peg_phase_g
                            target_g = min(target_g, phase_max_g)
                        if shuttle.separated and not sep_handled:
                            # New vehicle: the integrator holds the stack's trim and the
                            # disturbance model the old CoM; start both fresh.
                            ap.ctrl.reset_integral()
                            ap.ctrl._last_slow = 0.0
                            sep_handled = True
                        result = shuttle.update(
                            max_g=args.max_g, target_g=target_g, altitude=altitude,
                            dt=1/args.hz, throttle_slew=args.throttle_slew,
                            dynamic_pressure=flight.dynamic_pressure,
                            separation_max_q=args.booster_separation_max_q,
                            post_booster_coast_seconds=args.post_booster_coast_seconds)
                        if state == 'ASCENT':
                            solid_boosters_burning = (
                                bool(shuttle.solid_boosters) and not shuttle.separated and
                                any(engine.active and engine.has_fuel
                                    for engine in shuttle.solid_boosters))
                            if not solid_boosters_burning:
                                # Engine thrust_limit already follows target_g
                                # and PEG's phase_max_g each FCS tick. Reapplying
                                # that *same* low-G target to the master throttle
                                # made it decay to zero irrecoverably as the
                                # vehicle approached the desired orbit. Master
                                # throttle is a hard G/Q guard, not a second
                                # copy of the phase-trim allocator; recompute it
                                # from full each tick to prevent a zero latch.
                                if shuttle.separated:
                                    control.throttle = post_booster_master_throttle(
                                        flight.g_force, args.max_g,
                                        flight.dynamic_pressure, args.max_q)
                                else:
                                    control.throttle = dynamic_pressure_throttle(
                                        g_force_throttle(control.throttle, flight.g_force,
                                                         min(args.max_g, phase_max_g)),
                                        flight.dynamic_pressure, args.max_q)
                    else:
                        shuttle.skiff.thrust_limit = 1
                    actual_roll = ''
                    roll_control = 0.0
                    aim_direction = target_direction
                    if roll_active and state in ('ASCENT', 'ORBITER_ASCENT'):
                        # AutoPilot owns pitch/yaw only. Direct roll control keeps the
                        # real orbiter-mounted side (-Z) on the requested Earth/space side.
                        if roll_surface_states:
                            schedule_roll_authority(
                                roll_surface_states, flight.dynamic_pressure,
                                low_q_limit=60.0 if mission_roll_settled else 35.0,
                                high_q_limit=30.0)
                        aim_direction = target_direction
                        if state == 'ASCENT' and shuttle.separated:
                            # Guidance pitch is a thrust pitch: the SSMEs are canted ~15 deg
                            # off the nose, so aim the resultant thrust (not the nose).
                            try:
                                offset = thrust_axis_offset(conn, v, ap.reference_frame)
                                # The canted thrust axis sweeps a cone about the nose while
                                # rolling, so hold the nose itself on target during the upright
                                # roll and blend the thrust correction back in over 6 s after.
                                if roll_reset and shuttle.post_sep_roll_done_ut is None:
                                    # ramp out over 4 s: an instant switch steps the nose 15 deg
                                    blend = max(0.0, 1.0-(conn.space_center.ut-roll_reset_ut)/4.0)
                                elif shuttle.post_sep_roll_done_ut is not None:
                                    blend = min(1.0, (conn.space_center.ut -
                                                      shuttle.post_sep_roll_done_ut)/6.0)
                                else:
                                    blend = 1.0
                                raw = tuple(target_direction[i]-blend*offset[i] for i in range(3))
                                n = math.sqrt(sum(c*c for c in raw)) or 1.0
                                aim_direction = tuple(c/n for c in raw)
                            except Exception:
                                aim_direction = target_direction
                        if not shuttle.separated:
                            ap.target_direction = aim_direction
                            ap.target_roll = float('nan')
                        surface_frame = v.surface_reference_frame
                        actual_direction = v.direction(surface_frame)
                        roll_up_reference = surface_up_for_pitch_heading(
                            target_pitch, args.heading)
                        actual_side = conn.space_center.transform_direction(
                            (0, 0, -1), v.reference_frame, surface_frame)
                        target_side = desired_orbiter_side(
                            target_pitch, args.heading, bool(earthward_mode))
                        actual_roll = roll_for_roof(
                            actual_direction, roll_up_reference, actual_side)
                        physical_target_roll = roll_for_roof(
                            actual_direction, roll_up_reference, target_side)
                        target_roll = physical_target_roll
                        roll_error_signed = roll_delta(
                            actual_roll, physical_target_roll)
                        roll_error = abs(roll_error_signed)
                        body_roll_rate = body_roll_rate_deg_s(v)
                        roll_rate = body_roll_rate
                        pitch_error, heading_error = surface_axis_errors(
                            actual_direction, target_pitch, args.heading)
                        # Feed the live torque authority and moment of inertia
                        # (centre-of-mass movement, propellant burn, booster
                        # separation, thrust-dependent gimbal torque) into the
                        # roll loop instead of the pad calibration.
                        live_roll_accel = roll_accel_authority_deg_s2(v)
                        if live_roll_accel > 1.0:
                            # available_torque overstates what the roll axis really
                            # delivers (gimbal roll, elevons at low q), so bound the
                            # live estimate between 8 and 40 deg/s^2.
                            roll_accel_reference = max(8.0, min(40.0, live_roll_accel))
                        upright_phase = (shuttle.separated and roll_reset and
                                         shuttle.post_sep_roll_done_ut is None)
                        if shuttle.separated:
                            # After separation the autopilot owns all three axes. Its
                            # output replaces a direct control.roll command, so the
                            # upright roll is flown as an attitude target (nose on the
                            # thrust aim, orbiter roof on the requested side), auto-tuned
                            # continuously from the live inertia, CoM and torque.
                            if not ap_owns_roll:
                                ap.auto_tune = True
                                ap.time_to_peak = (3.0, 2.0, 3.0)
                                ap.overshoot = (0.03, 0.02, 0.03)
                                ap.max_angular_velocity = (
                                    1.0, math.radians(args.upright_roll_rate), 1.0)
                                ap_owns_roll = True
                                print('Autopilot owns pitch/yaw/roll after separation '
                                      '(auto-tuned, 3 s time-to-peak)')
                            if upright_phase and not upright_configured:
                                # Pitch/yaw keep priority during the roll: roll is suppressed
                                # whenever the nose is more than 8 deg off and fully engaged
                                # inside 5 deg (a tighter gate chattered roll on/off and
                                # limit-cycled at +-15 deg/s); stiffer pitch/yaw while rolling.
                                ap.roll_start_angle = 8.0
                                ap.roll_engage_angle = 5.0
                                ap.time_to_peak = (2.0, 2.5, 2.0)
                                ap.overshoot = (0.03, 0.01, 0.03)
                                upright_configured = True
                            if upright_phase:
                                control.rcs = True
                        # Size each roll maneuver to finish in a set time: the rate limit is
                        # chosen from the angle still to go when the maneuver starts.
                        maneuver = ('upright' if upright_phase else
                                    'headsdown' if not mission_roll_settled else None)
                        if maneuver and maneuver != roll_maneuver:
                            roll_maneuver = maneuver
                            duration = (args.upright_roll_time if maneuver == 'upright'
                                        else args.roll_time)
                            limit = (args.upright_roll_rate if maneuver == 'upright'
                                     else args.roll_rate)
                            rate = maneuver_roll_rate(roll_error, duration, limit)
                            p_rate, _, y_rate = ap.ctrl.max_rate
                            ap.ctrl.max_rate = (p_rate, math.radians(rate), y_rate)
                            print(f'{maneuver} roll: {roll_error:.0f} deg in ~{duration:.0f} s '
                                  f'(rate limit {rate:.1f} deg/s)')
                        # The attitude controller owns all three axes; the roll program is
                        # an orbiter-roof target (heads-down before separation, upright after).
                        ap.set_direction_and_up(aim_direction, target_side, 0.0)
                        roll_control = ap.ctrl.inputs[1]
                        settle_nose_error = direction_error_deg(
                            actual_direction, aim_direction)
                        # Pause roll whenever the nose is being pulled away from
                        # the commanded ascent vector; resume once pitch/yaw recover.
                        if settle_nose_error > 5.0:
                            roll_control = 0.0
                        if upright_phase:
                            if roll_error < 6.0 and abs(body_roll_rate) < 3.0:
                                if upright_since is None:
                                    upright_since = conn.space_center.ut
                                elif conn.space_center.ut-upright_since >= 1.0:
                                    shuttle.post_sep_roll_done_ut = conn.space_center.ut
                                    control.rcs = False
                                    ap.time_to_peak = (3.0, 3.0, 3.0)
                                    ap.max_angular_velocity = (1.0, math.radians(3.0), 1.0)
                                    print(f'Upright after {conn.space_center.ut-roll_reset_ut:.1f} s; '
                                          f'SSME limiter ramp released')
                            else:
                                upright_since = None
                            if (shuttle.post_sep_roll_done_ut is None and
                                    conn.space_center.ut-roll_reset_ut > 25.0):
                                shuttle.post_sep_roll_done_ut = conn.space_center.ut
                                control.rcs = False
                                print(f'WARNING: upright roll not settled after 25 s '
                                      f'(error {roll_error:.0f} deg); releasing climb anyway')
                        if not mission_roll_settled:
                            if roll_attitude_settled(
                                    roll_error, body_roll_rate,
                                    settle_nose_error, angle_limit=5.0,
                                    rate_limit=1.5):
                                if roll_settle_since is None:
                                    roll_settle_since = conn.space_center.ut
                                elif (conn.space_center.ut-roll_settle_since >=
                                      args.roll_settle_time):
                                    mission_roll_settled = True
                                    print('Limited elevon roll authority retained for ascent attitude hold')
                                    emit_event('gravity-turn')
                                    print('Physical heads-down roll settled; gravity turn released')
                            else:
                                roll_settle_since = None
                    elif target_roll is not None:
                        if state in ('ASCENT', 'ORBITER_ASCENT'):
                            roll_up_reference = surface_up_for_pitch_heading(
                                target_pitch, args.heading)
                            surface_frame = v.surface_reference_frame
                            actual_direction = v.direction(surface_frame)
                            roof_direction = conn.space_center.transform_direction(
                                (0, 0, -1), v.reference_frame, surface_frame)
                            actual_roll = roll_for_roof(actual_direction, roll_up_reference, roof_direction)
                        else:
                            # Orbital-frame x points anti-radially (toward the body).
                            roll_up_reference = (-1, 0, 0)
                        ap.set_direction_and_up(target_direction, roll_up_reference, target_roll)
                        if state in ('ASCENT', 'ORBITER_ASCENT'):
                            roll_error = abs(roll_delta(actual_roll, target_roll))
                            pitch_error, heading_error = surface_axis_errors(
                                actual_direction, target_pitch, args.heading)
                        else:
                            # Orbital-frame attitude errors are diagnostic only here.
                            try:
                                roll_error = abs(ap.roll_error)
                            except Exception:
                                roll_error = 0.0
                            pitch_error = 0.0
                            heading_error = 0.0
                    else:
                        ap.target_direction = target_direction
                        roll_error = 0.0
                        pitch_error = 0.0
                        heading_error = 0.0
                    nose_error = direction_error_deg(v.direction(ap.reference_frame), aim_direction)
                    try:
                        autopilot_error = ap.error
                    except Exception:
                        autopilot_error = ''
                    if conn.space_center.ut >= next_report:
                        actual_pitch, actual_heading = pitch_heading_for_direction(
                            v.direction(v.surface_reference_frame))
                        booster_thrust, booster_lf, booster_ox, booster_sf, booster_fraction = shuttle.booster_telemetry()
                        commanded_g = (shuttle.commanded_g(result, control.throttle)
                                       if shuttle.et_attached else 0.0)
                        booster_full_time = shuttle.booster_nominal_burn_time()
                        engine_limits = shuttle.engine_limit_telemetry()
                        gimbal_actuation = shuttle.gimbal_telemetry()
                        try:
                            available_torque = tuple(
                                tuple(round(x, 1) for x in side)
                                for side in v.available_torque)
                        except Exception:
                            available_torque = ()
                        emit_telemetry(ut=conn.space_center.ut, state=state, altitude=altitude, apoapsis=v.orbit.apoapsis_altitude, periapsis=v.orbit.periapsis_altitude, mass=v.mass, verticalSpeed=flight.vertical_speed, horizontalSpeed=flight.horizontal_speed, dynamicPressure=flight.dynamic_pressure, gForce=flight.g_force, commandedG=commanded_g, pitch=actual_pitch, heading=actual_heading, roll=actual_roll)
                        writer.writerow([conn.space_center.ut, state, altitude,
                                         v.orbit.apoapsis_altitude, v.orbit.periapsis_altitude,
                                         v.mass, flight.vertical_speed, flight.horizontal_speed,
                                         flight.dynamic_pressure, flight.g_force,
                                         '' if booster_elapsed is None else booster_elapsed,
                                         booster_thrust, booster_lf, booster_ox, booster_sf,
                                         booster_fraction, booster_full_time, commanded_g,
                                         shuttle.et_resource_amount('LiquidFuel'),
                                         shuttle.et_resource_amount('Oxidizer'),
                                         result.throttles if shuttle.et_attached else (1,),
                                         result.residual if shuttle.et_attached else 0,
                                         result.feasible if shuttle.et_attached else True,
                                         '' if target_roll is None else target_roll,
                                         nose_error, autopilot_error, actual_roll, roll_error, roll_rate,
                                         body_roll_rate, roll_control, pitch_error, heading_error,
                                         ap.roll_control_oscillation,
                                         actual_pitch, actual_heading,
                                         '' if target_pitch is None else target_pitch,
                                         shuttle.control_moment_budget, engine_limits,
                                         gimbal_actuation, available_torque,
                                         round(math.degrees(v.orbit.inclination), 3),
                                         '' if peg_dv is None else round(peg_dv, 2)])
                        log.flush()
                        print(f'{state}: altitude {altitude/1000:.1f} km; Ap {v.orbit.apoapsis_altitude/1000:.1f} km; residual {result.residual:.0f} N m; budget {shuttle.control_moment_budget:.0f} N m; cmd {commanded_g:.2f} G; actual {flight.g_force:.2f} G; nose {nose_error:.1f} deg; roll {roll_error:.1f} deg; rate {roll_rate:.1f}/{body_roll_rate:.1f} deg/s; ctl {roll_control:.2f}; gimbal {gimbal_actuation}; torque {available_torque}')
                        next_report = conn.space_center.ut+2
                    since_roll_reset = (None if roll_reset_ut is None else
                                        conn.space_center.ut-roll_reset_ut)
                    if liftoff_ut is not None:
                        part_count = len(v.parts.all)
                        if baseline_part_count is None:
                            baseline_part_count = part_count
                        elif (not shuttle.separated and
                              part_count < baseline_part_count):
                            control.roll = 0
                            pause_structural_failure(conn, control, ap)
                            raise RuntimeError(
                                f'Part loss detected before booster separation: '
                                f'{baseline_part_count} -> {part_count} parts')
                    if (shuttle.vehicle_kind == 'orbiter-shuttle' and
                            shuttle.separated and
                            conn.space_center.ut - shuttle.booster_separation_ut >= 1.0):
                        # A wing can detach several seconds after the impulse,
                        # once the damaged joint sees sustained engine/aero load.
                        try:
                            check_orbiter_integrity({
                                "ready": True,
                                "partNames": [part.name for part in v.parts.all],
                            })
                        except GuardError:
                            pause_structural_failure(conn, control, ap)
                            raise
                    if attitude_error_exceeded(nose_error, altitude, since_roll_reset):
                        raise RuntimeError(f'Nose attitude error {nose_error:.1f} deg exceeds 45 degrees')
                    orbital_engines = getattr(shuttle, 'orbiter_engines', [shuttle.skiff])
                    if not shuttle.et_attached and not any(e.has_fuel for e in orbital_engines):
                        raise RuntimeError('Orbiter engines out of fuel')
                    time.sleep(max(0, 1/args.hz-(time.monotonic()-tick)))
        finally:
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            try:
                try:
                    if saved_roll_pid is not None:
                        ap.roll_pid_gains = saved_roll_pid
                    restore_aero_roll(roll_surface_states)
                    if saved_auto_tune is not None:
                        ap.auto_tune = saved_auto_tune
                finally:
                    release_controls(control, ap)
            finally:
                signal.signal(signal.SIGTERM, old_sigterm_handler)
            print('Control released; main throttle zero. Any attached solid booster continues until burnout.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--launch', action='store_true')
    parser.add_argument('--address', default='127.0.0.1')
    parser.add_argument('--rpc-port', type=int, default=50000)
    parser.add_argument('--stream-port', type=int, default=50001)
    parser.add_argument('--orbit', type=float, default=90000, help='Target apoapsis altitude in m')
    parser.add_argument('--inclination', type=float, default=34.0,
                        help='Target orbital inclination in degrees (sets launch azimuth unless --heading is given)')
    parser.add_argument('--heading-slew', type=float, default=2.0,
                        help='Launch-azimuth turn rate in deg/s after the roll settles')
    parser.add_argument('--heading', type=float, default=None,
                        help='Explicit final launch heading; default derives from --inclination')
    parser.add_argument('--turn-start', type=float, default=1500,
                        help='Altitude in m where the post-roll gravity turn begins')
    parser.add_argument('--turn-end', type=float, default=55000,
                        help='Altitude in m where the smoothed gravity turn reaches minimum pitch')
    parser.add_argument('--pitch-slew', type=float,
                        default=PITCH_SLEW_DEG_PER_SECOND,
                        help='Maximum pitch-command change in deg/s')
    parser.add_argument('--meco-periapsis', type=float, default=60000,
                        help='Minimum periapsis altitude required for normal MECO in m')
    parser.add_argument('--booster-separation-max-q', type=float, default=float('inf'),
                        help='Coast after SRB burnout until dynamic pressure is at or below this Pa value')
    parser.add_argument('--post-booster-coast-seconds', type=float, default=0.0,
                        help='Keep SSMEs off briefly after STS booster separation (default: immediate restart)')
    parser.add_argument('--meco-pitch', type=float, default=0,
                        help='Surface pitch used to build horizontal energy after target apoapsis')
    parser.add_argument('--max-g', type=float, default=3.0,
                        help='Maximum commanded thrust acceleration in Earth g')
    parser.add_argument('--max-q', type=float, default=MAX_DYNAMIC_PRESSURE_PA,
                        help='Maximum dynamic pressure in Pa; reduces liquid throttle above this value')
    parser.add_argument('--launch-g', type=float, default=LAUNCH_TARGET_G,
                        help='Gentle initial thrust-acceleration target in Earth g')
    parser.add_argument('--ssme-limit', type=float, default=0.7,
                        help='Fixed SSME thrust limiter held through SRB burn')
    parser.add_argument('--upper-g', type=float, default=1.4,
                        help='Acceleration cap for the SSME-only phase after booster separation')
    parser.add_argument('--et-clear-time', type=float, default=4.0,
                        help='Seconds of RCS up-translation after ET separation')
    parser.add_argument('--et-sep-delay', type=float, default=5.0,
                        help='Seconds to hold attitude after MECO before external-tank separation')
    parser.add_argument('--ssme-lead-time', type=float, default=3.0,
                        help='Seconds to hold verified SSME thrust before booster ignition')
    parser.add_argument('--booster-min-g', type=float, default=BOOSTER_MIN_G,
                        help='Hard minimum thrust acceleration while boosters are attached and burning')
    parser.add_argument('--throttle-slew', type=float, default=THROTTLE_SLEW_PER_SECOND,
                        help='Maximum per-second change of each engine-group thrust limiter')
    parser.add_argument('--roll-reset-delay', type=float, default=2.0,
                        help='Seconds after booster separation before the delayed 180-degree roll')
    parser.add_argument('--roll-start', type=float, default=30,
                        help='Climb above launch point in m before starting the mission roll')
    parser.add_argument('--upright-max-q', type=float, default=800.0,
                        help='Start the post-separation upright roll only below this dynamic pressure (Pa)')
    parser.add_argument('--upright-roll-rate', type=float, default=15.0,
                        help='Roll rate limit (deg/s) for the upright roll after booster separation')
    parser.add_argument('--roll-time', type=float, default=10.0,
                        help='Duration of the heads-down roll maneuver (s)')
    parser.add_argument('--upright-roll-time', type=float, default=10.0,
                        help='Duration of the post-separation upright roll (s)')
    parser.add_argument('--roll-rate', type=float, default=15.0,
                        help='Direct physical-roll angular-rate cap in deg/s')
    parser.add_argument('--roll-attenuation', type=float, default=5.0,
                        help='Roll-error angle in deg where AutoPilot starts slowing the roll')
    parser.add_argument('--roll-time-to-peak', type=float, default=2.5,
                        help='AutoTune time-to-peak for the roll rate controller in seconds')
    parser.add_argument('--roll-overshoot', type=float, default=0.001,
                        help='AutoTune target roll overshoot fraction')
    parser.add_argument('--roll-settle-time', type=float, default=2.0,
                        help='Seconds roll/pitch/yaw errors must remain settled')
    parser.add_argument('--roll-control-limit', type=float, default=0.5,
                        help='Maximum direct roll-control magnitude during physical roll')
    parser.add_argument('--roll-response-time', type=float, default=.3,
                        help='Rate-controller response horizon in seconds for physical roll')
    parser.add_argument('--hz', type=float, default=10)
    parser.add_argument('--timeout', type=float, default=1800)
    parser.add_argument('--telemetry', default='telemetry.csv')
    args = parser.parse_args()
    if (not 0 < args.hz <= 50 or not 0 <= args.turn_start < args.turn_end or
            args.timeout <= 0 or args.ssme_lead_time < 0 or
            not 0 < args.pitch_slew <= 20 or
            args.roll_reset_delay < 0 or args.roll_start < 0 or
            not 0 < args.roll_rate <= 30 or not 0 < args.roll_attenuation <= 90 or
            args.roll_time_to_peak <= 0 or not 0 <= args.roll_overshoot <= 1 or
            args.roll_settle_time < 0 or not 0 < args.roll_control_limit <= 1 or
            args.roll_response_time <= 0 or not 0 < args.launch_g <= args.max_g <= 10 or
            not 0 < args.booster_min_g <= args.max_g or not 0 < args.throttle_slew <= 1 or
            math.isnan(args.booster_separation_max_q) or args.booster_separation_max_q <= 0 or
            not math.isfinite(args.post_booster_coast_seconds) or
            args.post_booster_coast_seconds < 0 or
            not 0 <= args.meco_periapsis < args.orbit or
            not -10 <= args.meco_pitch <= 20):
        parser.error('Invalid loop rate, ascent/MECO profile, G limit, timeout, or roll profile')
    run(args)


if __name__ == '__main__':
    main()
