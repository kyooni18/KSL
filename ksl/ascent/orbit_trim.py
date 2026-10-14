"""OMS trim: bring a post-MECO orbit to exact apoapsis/periapsis/inclination.

Three small burns planned on kRPC maneuver nodes (each solved against the node's
predicted orbit, so no sign or frame conventions are assumed):

  1. at apoapsis      -- prograde burn that sets the periapsis
  2. at periapsis     -- retrograde burn that sets the apoapsis
  3. at the next node -- normal burn that sets the inclination
"""

import argparse
import math
import time

import krpc

from ..controls import Controls, warp_until
from . import orbit_math as om
from .engine_roles import is_oms_engine
from .events import emit_event


MIN_BURN_ALTITUDE = 74000.0
ALIGN_LEAD = 30.0  # s of attitude alignment before every burn
# Corrections at or below this delta-v fly as RCS forward translation instead of
# an OMS burn: the OMS thrust vector is off the shuttle's attitude axis, so a
# short OMS burn makes it tumble, while forward translation needs no thrust-vector
# alignment.
RCS_TRIM_MAX_DV = 2.0  # m/s
RCS_TRIM_TIMEOUT = 180.0  # s of wall time
RUNTIME = {}  # 'controls': operator runtime controls (auto-warp, trim hold/disable)


def controls():
    if 'controls' not in RUNTIME:
        RUNTIME['controls'] = Controls.from_env()
    return RUNTIME['controls']


def warp_to_burn(sc, ut, lead):
    """Warp toward a burn, honouring the operator's auto-warp setting."""
    def announce(index, remaining):
        print(f'time warp index {index}, {remaining:.0f} s to go'
              + ('' if controls().auto_warp else ' (auto-warp off)'))
    warp_until(sc, ut, controls(), lead=lead, announce=announce)


def oms_propellant_mass(vessel):
    res = vessel.resources
    return 5.0*(res.amount('LiquidFuel')+res.amount('Oxidizer'))


def other_apsis(orbit, burn_ut, body_radius):
    """Altitude of the apsis opposite the burn point (the burn point is an apsis).

    Ap + Pe - r_burn is smooth and monotonic in the burn, unlike picking whichever
    apsis is farther from the burn point, which flips near a circular orbit.
    """
    r_b = orbit.radius_at(burn_ut)-body_radius
    return orbit.apoapsis_altitude+orbit.periapsis_altitude-r_b


def recoverable_periapsis_target(desired_periapsis, burn_altitude,
                                 margin=5000.0):
    """Bound the requested Pe below the altitude of an impulsive prograde burn.

    A 5 km margin is helpful for a rescue *away from an apsis*: the radial
    speed cannot be eliminated with prograde thrust alone. But that margin
    must not apply forever after the orbit has been stabilized, or a 100 km
    Ap can never be circularized above 95 km (live fourth-flight OMS failure).
    """
    return min(desired_periapsis, burn_altitude-margin)


def periapsis_burn_margin(current_periapsis, burn_altitude, apoapsis_altitude):
    """Use a fine Pe target only at a real apsis of an already safe orbit."""
    if (current_periapsis >= MIN_BURN_ALTITUDE and
            abs(burn_altitude-apoapsis_altitude) <= 100.0):
        return 100.0
    return 5000.0


def solve(make_node, error, lo, hi, iterations=26):
    """Bisection on a node component; error(orbit) must change sign over [lo, hi]."""
    def value(x):
        node = make_node(x)
        try:
            return error(node.orbit)
        finally:
            node.remove()
    f_lo, f_hi = value(lo), value(hi)
    if f_lo*f_hi > 0:
        raise RuntimeError(f'No solution in [{lo}, {hi}]: errors {f_lo:.1f}, {f_hi:.1f}')
    for _ in range(iterations):
        mid = 0.5*(lo+hi)
        f_mid = value(mid)
        if f_lo*f_mid <= 0:
            hi, f_hi = mid, f_mid
        else:
            lo, f_lo = mid, f_mid
    return 0.5*(lo+hi)


# OMS resultant thrust axis in the body frame. The pods (x +-1.13, y -5.49,
# z -1.68 from the CoM) are canted so the thrust line passes the CoM: about
# (0, 0.95, +0.30), which matches the part axis and the run-b2 in-flight
# measurement (0.07, 0.94, 0.35). Refined live from >= 2 m/s windows; shorter
# windows are dominated by gravity-integration error (run a1 read the sign wrong).
MEASURED_AXIS = {'axis': (0.0, 0.940, 0.342)}
CUSTOM_AP = {}


def thrust_axis_offset(conn, vessel, engines, frame):
    """Net thrust direction of the active engines minus the vessel's forward axis, in `frame`.

    The OMS pods sit off the centre of mass and are canted, so the resultant force
    (what actually changes the orbit, once RCS/wheels cancel the leftover torque)
    is not along the nose. Commanding nose = burn direction - offset aims the
    resultant force at the burn direction instead.
    """
    total = [0.0, 0.0, 0.0]
    for engine in engines:
        for thruster in engine.thrusters:
            try:
                direction = thruster.thrust_direction(vessel.reference_frame)
            except Exception:
                # kRPC often cannot resolve these thruster transforms; the canted
                # part axis is the (ungimballed) thrust axis.
                direction = engine.part.direction(vessel.reference_frame)
            weight = engine.thrust if engine.thrust > 1.0 else engine.available_thrust
            for i in range(3):
                total[i] += weight/max(len(engine.thrusters), 1)*direction[i]
    norm = math.sqrt(sum(c*c for c in total))
    if norm <= 0.0:
        return (0.0, 0.0, 0.0)
    thrust_body = tuple(c/norm for c in total)
    if MEASURED_AXIS:
        # The modelled axis is only a first guess: once a burn has run, use the
        # thrust axis measured from the vessel's own acceleration.
        thrust_body = MEASURED_AXIS['axis']
    forward_body = (0.0, 1.0, 0.0)
    conn_sc = conn.space_center
    t = conn_sc.transform_direction(thrust_body, vessel.reference_frame, frame)
    f = conn_sc.transform_direction(forward_body, vessel.reference_frame, frame)
    return tuple(t[i]-f[i] for i in range(3))


def aim(conn, vessel, engines, ap, frame, direction):
    offset = thrust_axis_offset(conn, vessel, engines, frame)
    command = [direction[i]-offset[i] for i in range(3)]
    norm = math.sqrt(sum(c*c for c in command)) or 1.0
    ap.target_direction = tuple(c/norm for c in command)


def align_for_burn(conn, vessel, engines, ap, frame, direction, limit,
                   timeout=120.0, rcs_after=5.0):
    """Use RCS briefly if reaction wheels cannot reacquire the OMS vector."""
    start = time.monotonic()
    try:
        while ap.error > limit:
            elapsed = time.monotonic() - start
            if elapsed > timeout:
                raise RuntimeError(f'OMS burn attitude did not align within {timeout:.0f} s')
            if elapsed > rcs_after:
                vessel.control.rcs = True
            aim(conn, vessel, engines, ap, frame, direction)
            time.sleep(0.1)
    finally:
        vessel.control.rcs = False


def rcs_delta_v(vessel):
    """Delta-v (m/s) the RCS monopropellant can deliver as forward translation."""
    try:
        mono = vessel.resources.amount('MonoPropellant')*4.0  # 4 kg per unit
        thrusters = list(vessel.parts.rcs)
        isp = max((t.vacuum_specific_impulse for t in thrusters), default=0.0)
    except Exception:
        return math.inf  # unknown: do not block the RCS path on telemetry gaps
    mass = vessel.mass
    if isp <= 0.0 or mass <= mono:
        return 0.0
    return isp*9.80665*math.log(mass/(mass-mono))


def use_rcs_forward(dv, engines_available=True, rcs_available=math.inf, limit=None):
    """Slight corrections use RCS forward translation; larger ones need the OMS.

    The RCS path is only chosen while the monopropellant covers the burn with a 50%
    margin (alignment and attitude leakage also spend it).
    """
    limit = RCS_TRIM_MAX_DV if limit is None else limit
    if not engines_available:
        return True
    return dv <= limit and 1.5*dv <= rcs_available


def execute_rcs_forward(conn, vessel, node, tolerance=0.03):
    """Fly a slight correction with RCS forward translation only.

    Attitude is acquired with reaction wheels while RCS is off.  Only then are the
    RCS parts configured for forward translation alone (pitch/yaw/roll thrusters
    disabled), so the thrusters cannot fight the attitude hold.  The vessel's
    forward axis is aimed straight at the burn vector: no OMS thrust-axis offset.
    """
    sc = conn.space_center
    ctl = vessel.control
    ap = CUSTOM_AP['ap']
    rcs_parts = list(vessel.parts.rcs)
    if not rcs_parts:
        raise RuntimeError('No RCS thrusters for a forward-translation trim')
    saved = [(r.pitch_enabled, r.yaw_enabled, r.roll_enabled, r.forward_enabled,
              r.right_enabled, r.up_enabled) for r in rcs_parts]
    print(f'RCS forward trim: node dv {node.delta_v:.3f} m/s, T-{node.time_to:.0f} s')
    try:
        ctl.rcs = False
        ctl.throttle = 0.0
        ap.reference_frame = node.reference_frame
        burn_dir = (0.0, 1.0, 0.0)
        ap.target_direction = burn_dir
        ap.target_roll = float('nan')
        ap.engaged = True
        if node.time_to > ALIGN_LEAD+60:
            warp_to_burn(sc, node.ut, ALIGN_LEAD+30)
        while sc.ut < node.ut-0.0:
            ap.target_direction = burn_dir
            time.sleep(0.1)
        start = time.monotonic()
        while ap.error > 2.0:  # reaction wheels only, RCS stays off
            if time.monotonic()-start > 120.0:
                raise RuntimeError('RCS trim attitude did not align within 120 s')
            ap.target_direction = burn_dir
            time.sleep(0.1)
        for r in rcs_parts:
            r.pitch_enabled = r.yaw_enabled = r.roll_enabled = False
            r.right_enabled = r.up_enabled = False
            r.forward_enabled = True
        ctl.rcs = True
        burn_start = time.monotonic()
        timeout = max(RCS_TRIM_TIMEOUT, 120.0*node.delta_v)  # ~0.01-0.05 m/s^2 of RCS accel
        while time.monotonic()-burn_start < timeout:
            try:
                remaining = node.remaining_delta_v
            except RuntimeError:
                break
            if remaining <= tolerance:
                break
            if ap.error > 10.0:
                ctl.forward = 0.0
                break
            ctl.forward = max(0.05, min(1.0, remaining/0.3))
            time.sleep(0.05)
        print(f'RCS forward trim end after {time.monotonic()-burn_start:.1f} s, '
              f'aim error {ap.error:.1f} deg')
    finally:
        ctl.forward = 0.0
        ctl.rcs = False
        for r, flags in zip(rcs_parts, saved):
            (r.pitch_enabled, r.yaw_enabled, r.roll_enabled, r.forward_enabled,
             r.right_enabled, r.up_enabled) = flags
        ap.engaged = False
        try:
            node.remove()
        except RuntimeError:
            pass


def oms_delta_v(vessel):
    """Delta-v (m/s) the active OMS engines can still deliver."""
    engines = [e for e in vessel.parts.engines if e.active and e.available_thrust > 0]
    thrust = sum(e.available_thrust for e in engines)
    if thrust <= 0.0:
        return 0.0
    isp = sum(e.specific_impulse*e.available_thrust for e in engines)/thrust
    mass = vessel.mass
    dry = mass - oms_propellant_mass(vessel)
    return isp*9.80665*math.log(mass/max(1.0, dry)) if mass > dry > 0 else 0.0


def choose_frame_map(r0, v0, mu, krpc_inclination_deg):
    """Pick the kRPC -> right-handed mapping whose inclination matches kRPC's own."""
    best = None
    for name, fn in (('swap', lambda v: (v[0], v[2], v[1])),
                     ('rotate', lambda v: (v[0], v[2], -v[1]))):
        inc = om.elements(fn(r0), fn(v0), mu)['inc']
        error = abs(inc-krpc_inclination_deg)
        if best is None or error < best[0]:
            best = (error, name, fn)
    return best[2], best[1]


def solve3(columns, rhs):
    """Solve  sum_i x_i * columns[i] = rhs  for three 3-vectors (Cramer's rule)."""
    def det(a, b, c):
        return om.dot(a, om.cross(b, c))
    d = det(*columns)
    if abs(d) < 1e-9:
        raise RuntimeError('Degenerate node basis')
    return (det(rhs, columns[1], columns[2])/d,
            det(columns[0], rhs, columns[2])/d,
            det(columns[0], columns[1], rhs)/d)


def from_rh_map(to_rh):
    """Inverse of a kRPC -> right-handed map chosen by choose_frame_map."""
    if to_rh((1.0, 2.0, 3.0)) == (1.0, 3.0, 2.0):
        return to_rh  # the swap is its own inverse
    return lambda v: (v[0], -v[2], v[1])


def node_for_dv(vessel, ut, dv_krpc, frame):
    """Maneuver node at ``ut`` whose burn vector equals ``dv_krpc`` (in ``frame``).

    The node frame is calibrated by probing unit nodes through kRPC's own
    burn_vector, so no prograde/normal/radial sign convention is assumed.
    """
    basis = []
    for kwargs in ({'prograde': 1.0}, {'normal': 1.0}, {'radial': 1.0}):
        probe = vessel.control.add_node(ut, **kwargs)
        try:
            basis.append(probe.burn_vector(frame))
        finally:
            probe.remove()
    prograde, normal, radial = solve3(basis, dv_krpc)
    node = vessel.control.add_node(ut, prograde=prograde, normal=normal, radial=radial)
    node.goal = None
    return node


def rh_state(vessel, frame, mu):
    """(r, v, to_rh) of the vessel in the right-handed orbit_math frame."""
    r_k, v_k = vessel.position(frame), vessel.velocity(frame)
    to_rh, _ = choose_frame_map(r_k, v_k, mu, math.degrees(vessel.orbit.inclination))
    return to_rh(r_k), to_rh(v_k), to_rh


def plan_exact(sc, vessel, args, min_lead, tolerance_alt=100.0, tolerance_inc=0.02):
    """One burn to the exact target apsides and inclination, or None to fall back.

    The burn time is searched analytically (orbit_math.solve_one_burn); the dv
    vector is then converted to node components with the node frame calibrated by
    probing unit nodes through kRPC's own burn_vector, so no frame or sign
    convention is assumed. Returns (nodes, predicted orbit); nodes is empty when
    the orbit is already on target.
    """
    orbit = vessel.orbit
    body = orbit.body
    nrf = body.non_rotating_reference_frame
    mu, radius = body.gravitational_parameter, body.equatorial_radius
    if (abs(orbit.apoapsis_altitude-args.apoapsis) < tolerance_alt and
            abs(orbit.periapsis_altitude-args.periapsis) < tolerance_alt and
            abs(math.degrees(orbit.inclination)-args.inclination) < tolerance_inc):
        return [], orbit
    r_k, v_k = vessel.position(nrf), vessel.velocity(nrf)
    to_rh, _ = choose_frame_map(r_k, v_k, mu, math.degrees(orbit.inclination))
    r0, v0 = to_rh(r_k), to_rh(v_k)
    deadline = None
    if orbit.periapsis_altitude < MIN_BURN_ALTITUDE:
        # never wait through a periapsis pass inside the atmosphere
        deadline = orbit.time_to_periapsis - 20.0
    solution = om.solve_one_burn(
        r0, v0, mu, max(args.apoapsis, args.periapsis), min(args.apoapsis, args.periapsis),
        args.inclination, radius, min_lead=min_lead, deadline=deadline)
    if solution is None:
        return None
    ut = sc.ut + solution['t']
    node = node_for_dv(vessel, ut, from_rh_map(to_rh)(solution['dv_vec']), nrf)
    prograde, normal, radial = node.prograde, node.normal, node.radial
    predicted = node.orbit
    print(f'exact plan: burn in {solution["t"]:.0f} s, dv {solution["dv"]:.3f} m/s '
          f'(prograde {prograde:.3f}, normal {normal:.3f}, radial {radial:.3f}); '
          f'predicted Ap {predicted.apoapsis_altitude/1000:.3f} km, '
          f'Pe {predicted.periapsis_altitude/1000:.3f} km, '
          f'inc {math.degrees(predicted.inclination):.4f} deg; '
          f'{"exact" if solution["exact"] else "closest"} solution')
    return [node], predicted


def execute(conn, vessel, node, done, tolerance=0.03, orbit_cutoff=False):
    if not orbit_cutoff and use_rcs_forward(node.delta_v, rcs_available=rcs_delta_v(vessel),
                                            limit=RUNTIME.get('rcs_max_dv')):
        return execute_rcs_forward(conn, vessel, node, min(tolerance, 0.03))
    sc = conn.space_center
    ctl = vessel.control
    from .attitude import CustomAutopilot
    ap = CUSTOM_AP['ap']
    engines = [e for e in vessel.parts.engines if e.active]
    thrust = sum(e.available_thrust for e in engines)
    isp = sum(e.specific_impulse*e.available_thrust for e in engines)/max(thrust, 1.0)
    if thrust <= 0.0:
        raise RuntimeError('No OMS thrust available (fuel exhausted or engines inactive)')
    dv = node.delta_v
    mass = vessel.mass
    burn_time = isp*9.80665*mass*(1.0-math.exp(-dv/(isp*9.80665)))/thrust
    start = node.ut - 0.5*burn_time
    print(f'Node dv {dv:.2f} m/s, burn {burn_time:.1f} s, T-{node.time_to:.0f} s, '
          f'EC {vessel.resources.amount("ElectricCharge"):.0f}')
    # Whatever happens below, the engines must never be left running.
    try:
        burn_dir = (0.0, 1.0, 0.0)
        ap.reference_frame = node.reference_frame
        aim(conn, vessel, engines, ap, node.reference_frame, burn_dir)
        ap.target_roll = float('nan')
        # Fine corrections use reaction wheels only: RCS thrusters leak net
        # translation into the orbit while they hold attitude.
        ctl.rcs = False
        # Leave at least ALIGN_LEAD s of unwarped time to settle on the burn attitude
        # (the autopilot cannot steer on rails).
        if start-sc.ut > ALIGN_LEAD+60:
            warp_to_burn(sc, start, ALIGN_LEAD+30)
        ap.engaged = True
        ctl.rcs = False  # never RCS during OMS burns (it perturbs the orbit)
        while sc.ut < start-30:
            aim(conn, vessel, engines, ap, node.reference_frame, burn_dir)
            time.sleep(0.2)
        ap.reference_frame = node.reference_frame
        while sc.ut < start:
            aim(conn, vessel, engines, ap, node.reference_frame, burn_dir)
            time.sleep(0.02)
        # Never burn unless the nose is on the burn vector.
        align_limit = 2.0 if dv < 5.0 else 3.0
        align_for_burn(conn, vessel, engines, ap, node.reference_frame,
                       burn_dir, align_limit)
        ctl.throttle = 1.0
        body = vessel.orbit.body
        nrf = body.non_rotating_reference_frame
        mu = body.gravitational_parameter
        v_prev, t_prev = vessel.velocity(nrf), sc.ut
        window = [0.0, 0.0, 0.0]
        mass_start = vessel.mass
        prop_start = oms_propellant_mass(vessel)
        while True:
            # Count only OMS propellant: RCS monopropellant use is not burn delta-v.
            burned = max(0.0, prop_start-oms_propellant_mass(vessel))
            spent = isp*9.80665*math.log(mass_start/max(1.0, mass_start-burned))
            try:
                remaining = node.remaining_delta_v
            except RuntimeError:
                # KSP can drop the node mid-burn; fall back to our own accounting.
                remaining = dv-spent
            if done(vessel.orbit) or (remaining <= tolerance and not orbit_cutoff) or spent > 1.3*dv+0.5:
                print(f'burn end: spent {spent:.2f} of {dv:.2f} m/s, remaining {remaining:.2f}, '
                      f'aim error {ap.error:.1f} deg; measured thrust axis '
                      f'{tuple(round(c, 3) for c in MEASURED_AXIS.get("axis", ()))}')
                break
            if vessel.available_thrust <= 0.0:
                print('OMS thrust lost; stopping burn')
                break
            if ap.error > 10.0:
                ctl.throttle = 0.0
                align_for_burn(conn, vessel, engines, ap, node.reference_frame,
                               burn_dir, 3.0, timeout=60.0)
            # Measure the real thrust axis: velocity change minus gravity, in the body frame.
            v_now, t_now = vessel.velocity(nrf), sc.ut
            r = vessel.position(nrf)
            rn = math.sqrt(sum(c*c for c in r))
            dt = t_now-t_prev
            for i in range(3):
                window[i] += (v_now[i]-v_prev[i]) + mu*r[i]/rn**3*dt
            v_prev, t_prev = v_now, t_now
            wn = math.sqrt(sum(c*c for c in window))
            if wn > 2.0 and ap.error < 2.0:
                axis = sc.transform_direction(tuple(c/wn for c in window), nrf,
                                              vessel.reference_frame)
                old = MEASURED_AXIS.get('axis')
                if old is not None:
                    axis = tuple(0.3*a+0.7*b for a, b in zip(axis, old))
                    an = math.sqrt(sum(c*c for c in axis))
                    axis = tuple(c/an for c in axis)
                MEASURED_AXIS['axis'] = axis
                ap.ctrl.axis_override = {
                    'Terrier': axis, 'Viking': axis, 'AJ10': axis, 'Orbital': axis,
                }
                window = [0.0, 0.0, 0.0]
            elif wn > 2.0:
                window = [0.0, 0.0, 0.0]
            aim(conn, vessel, engines, ap, node.reference_frame, burn_dir)
            accel = thrust/vessel.mass
            # Fine trims fly at a low throttle so cutoff resolution is ~1 cm/s.
            cap = 0.1 if dv < 5.0 else 1.0
            ctl.throttle = max(0.01, min(cap, remaining/max(accel, 0.1)/1.5))
            time.sleep(0.02)
    finally:
        ctl.throttle = 0.0
        ctl.rcs = False
        ap.engaged = False
        try:
            node.remove()
        except RuntimeError:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--apoapsis', type=float, default=90000.0)
    parser.add_argument('--periapsis', type=float, default=90000.0)
    parser.add_argument('--inclination', type=float, default=34.0)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--min-burn-dv', type=float, default=0.0,
                        help='Skip planned burns at or below this delta-v (m/s)')
    parser.add_argument('--max-burns', type=int, default=2,
                        help='Maximum number of burns (1 ideally; a 2nd finishes the residual)')
    parser.add_argument('--legacy-plan', action='store_true',
                        help='Use the staged apsis-by-apsis planner instead of the exact one-burn solution')
    parser.add_argument('--skip-if-short-dv', type=float, default=None, metavar='MARGIN',
                        help='Skip (no burn) unless the OMS has the planned dv plus this margin (m/s)')
    parser.add_argument('--label', default='trim', help='Name used in trim events')
    parser.add_argument('--rcs-max-dv', type=float, default=RCS_TRIM_MAX_DV,
                        help='Burns up to this dv (m/s) fly as RCS forward translation '
                             '(RCS off while aligning on reaction wheels)')
    parser.add_argument('--keep-inclination', action='store_true',
                        help='Trim apsides only; keep the as-flown plane (plane-targeted ascents)')
    parser.add_argument('--address', default='127.0.0.1')
    parser.add_argument('--rpc-port', type=int, default=50000)
    parser.add_argument('--stream-port', type=int, default=50001)
    args = parser.parse_args()
    conn = krpc.connect(name='orbit-trim', address=args.address,
                        rpc_port=args.rpc_port, stream_port=args.stream_port)
    sc, vessel = prepare(conn)
    RUNTIME['rcs_max_dv'] = args.rcs_max_dv
    _trim(conn, sc, vessel, args)


def prepare(conn):
    """Shared OMS setup: custom autopilot, OMS engines only, fuel cells on."""
    from .attitude import CustomAutopilot
    # Own attitude controller (no kRPC AutoPilot). OMS burns use reaction wheels
    # only, so the attitude loop is kept slow and gentle.
    CUSTOM_AP['ap'] = CustomAutopilot(conn, conn.space_center.active_vessel)
    # RCS is off: the reaction wheels are the only actuator, so allow their full
    # rate authority (the wheel torque limit is the real bound), and feed the OMS
    # thrust-line moment forward using the measured thrust axis.
    CUSTOM_AP['ap'].ctrl.tau = (0.8, 0.8, 0.8)
    CUSTOM_AP['ap'].ctrl.rate_tau = 0.4
    CUSTOM_AP['ap'].ctrl.max_rate = (math.radians(10.0),)*3
    CUSTOM_AP['ap'].ctrl.deadband = 0.03
    oms_axis = MEASURED_AXIS['axis']
    CUSTOM_AP['ap'].ctrl.axis_override = {
        'Terrier': oms_axis, 'Viking': oms_axis, 'AJ10': oms_axis, 'Orbital': oms_axis,
    }
    sc = conn.space_center
    vessel = sc.active_vessel
    vessel.control.sas = False
    for engine in vessel.parts.engines:
        engine.active = is_oms_engine(engine)
    # Reaction wheels need power: the orbiter has no panels, only a fuel cell.
    for converter in vessel.parts.resource_converters:
        for i in range(converter.count):
            if not converter.active(i):
                converter.start(i)
    return sc, vessel


def _trim(conn, sc, vessel, args):
    if getattr(args, 'keep_inclination', False):
        args.inclination = math.degrees(vessel.orbit.inclination)

    def report(tag):
        o = vessel.orbit
        print(f'{tag}: Ap {o.apoapsis_altitude/1000:.2f} km, Pe {o.periapsis_altitude/1000:.2f} km, '
              f'inc {math.degrees(o.inclination):.3f} deg')
        emit_event('trim-report', tag=tag, apoapsis_m=o.apoapsis_altitude,
                   periapsis_m=o.periapsis_altitude, inclination_deg=math.degrees(o.inclination))

    for stale in list(vessel.control.nodes):
        stale.remove()
    report('start')

    body_radius = vessel.orbit.body.equatorial_radius

    def high_enough(orbit, ut):
        return orbit.radius_at(ut)-body_radius >= MIN_BURN_ALTITUDE

    target_inc = math.radians(args.inclination)
    TOL = 300.0  # m: below this the OMS minimum impulse cannot improve the orbit

    def plan(min_lead=45.0):
        """Next burns from the measured orbit, in an order that never lowers the periapsis
        into the atmosphere:

          A. soon        -- prograde, only if the apoapsis is below target
          B. at apoapsis -- prograde (periapsis) + normal (inclination), solved together
          C. at the new periapsis -- set the apoapsis (circularization trim)

        Returns the nodes; only the first is flown before re-planning.
        """
        for stale in list(vessel.control.nodes):
            stale.remove()
        if not args.legacy_plan:
            try:
                exact = plan_exact(sc, vessel, args, min_lead)
            except Exception as exc:  # fall back to the staged planner
                print(f'exact planner unavailable ({type(exc).__name__}: {exc}); staged plan')
                for stale in list(vessel.control.nodes):
                    stale.remove()
                exact = None
            if exact is not None:
                return exact
        o0 = vessel.orbit
        now = sc.ut
        flight = vessel.flight(o0.body.reference_frame)
        nodes = []
        base = o0
        if o0.apoapsis_altitude < args.apoapsis-TOL:
            ut_a = now + max(10.0, min_lead, (MIN_BURN_ALTITUDE+1500.0-flight.mean_altitude) /
                             max(flight.vertical_speed, 20.0))
            xa = solve(lambda x: vessel.control.add_node(ut_a, prograde=x),
                       lambda orb: orb.apoapsis_altitude-args.apoapsis, 0.0, 300.0)
            na = vessel.control.add_node(ut_a, prograde=xa)
            na.goal = ('ap', args.apoapsis)
            nodes.append(na)
            base = na.orbit
        pe_ok = abs(base.periapsis_altitude-args.periapsis) < TOL
        inc_ok = abs(math.degrees(base.inclination)-args.inclination) < 0.02
        if not (pe_ok and inc_ok):
            ut_b = base.ut_at_true_anomaly(math.pi)
            earliest = (nodes[-1].ut+60.0) if nodes else now+max(30.0, min_lead)
            pe_unsafe = base.periapsis_altitude < MIN_BURN_ALTITUDE
            if ut_b < earliest or (pe_unsafe and ut_b-earliest > 0.5*base.period):
                # Never wait a whole orbit with the periapsis in the air: a burn a
                # little past the apoapsis costs a few percent, a periapsis pass at
                # 60-70 km costs the orbit (run a1).
                ut_b = earliest if pe_unsafe else ut_b + base.period
            burn_altitude = base.radius_at(ut_b)-body_radius
            margin = periapsis_burn_margin(
                base.periapsis_altitude, burn_altitude, base.apoapsis_altitude)
            pe_goal = recoverable_periapsis_target(
                args.periapsis, burn_altitude, margin=margin)
            if pe_goal <= base.periapsis_altitude+TOL:
                raise RuntimeError(
                    f'No safe periapsis rescue at UT {ut_b:.1f}: '
                    f'current Pe {base.periapsis_altitude:.0f} m, '
                    f'burn altitude {burn_altitude:.0f} m')
            if pe_goal < args.periapsis-TOL:
                print(f'Periapsis rescue stage: burn altitude {burn_altitude/1000:.1f} km, '
                      f'target Pe {pe_goal/1000:.1f} km before final orbit trim')
            x = y = 0.0
            for _ in range(4):
                x = solve(lambda q: vessel.control.add_node(ut_b, prograde=q, normal=y),
                          lambda orb: orb.periapsis_altitude-pe_goal, -150.0, 150.0)
                if not inc_ok:
                    y = solve(lambda q: vessel.control.add_node(ut_b, prograde=x, normal=q),
                              lambda orb: math.degrees(orb.inclination-target_inc), -300.0, 300.0)
            nb = vessel.control.add_node(ut_b, prograde=x, normal=y)
            nb.goal = None  # dv cutoff; the closed loop re-plans the remainder
            nodes.append(nb)
            base = nb.orbit
            # Only execute the first burn, then replan from the measured state.
            # A second burn constructed on the predicted post-rescue orbit
            # cannot safely solve 'other apsis' before that rescue exists:
            # the future ut_c may no longer be an apsis of this post-node orbit,
            # and a physical solution can be absent inside the +/-80 m/s
            # bracket. That rejected the *valid first rescue* prematurely.
            return nodes, base
        if abs(base.apoapsis_altitude-args.apoapsis) > TOL:
            ut_c = base.ut_at_true_anomaly(0.0)
            if ut_c < (nodes[-1].ut if nodes else now)+max(60.0, min_lead):
                ut_c += base.period
            xc = solve(lambda q: vessel.control.add_node(ut_c, prograde=q),
                       lambda orb: other_apsis(orb, ut_c, body_radius)-args.apoapsis, -80.0, 80.0)
            nc = vessel.control.add_node(ut_c, prograde=xc)
            nc.goal = ('other', args.apoapsis, ut_c)
            nodes.append(nc)
            base = nodes[-1].orbit
        return nodes, base

    if args.dry_run:
        nodes, final = plan()
        print(f'plan: final Ap {final.apoapsis_altitude/1000:.2f} Pe {final.periapsis_altitude/1000:.2f} '
              f'inc {math.degrees(final.inclination):.3f} (dv {[round(n.delta_v, 1) for n in nodes]})')
        for stale in list(vessel.control.nodes):
            stale.remove()
        return
    ctl = controls()
    if not ctl.trim_enabled:
        print('Orbit operations disabled by the operator; nothing flown')
        emit_event('trim-skipped', label=args.label, reason='disabled')
        return
    if ctl.hold_trim and not getattr(args, 'no_hold', False):
        print('Holding in orbit: waiting for the operator to trigger orbit trim')
        emit_event('trim-held', label=args.label)
        while not ctl.take('trimNow'):
            if not ctl.hold_trim or not ctl.trim_enabled:
                break
            time.sleep(0.5)
        if not ctl.trim_enabled:
            emit_event('trim-skipped', label=args.label, reason='disabled')
            return
    if args.skip_if_short_dv is not None:
        nodes, _ = plan(ALIGN_LEAD+30.0)
        need = sum(n.delta_v for n in nodes)
        have = oms_delta_v(vessel)
        for stale in list(vessel.control.nodes):
            stale.remove()
        emit_event('trim-budget', label=args.label, needed_mps=need, available_mps=have)
        print(f'OMS delta-v budget: need {need:.1f} m/s, have {have:.1f} m/s '
              f'(margin {args.skip_if_short_dv:.1f})')
        if have < need + args.skip_if_short_dv:
            emit_event('trim-skipped', label=args.label, reason='insufficient-dv')
            print('Insufficient OMS delta-v; leaving the orbit as it is')
            return
    for burn in range(1, args.max_burns+1):
        if not ctl.trim_enabled:
            print('Orbit trim disabled by the operator; stopping')
            emit_event('trim-skipped', label=args.label, reason='disabled')
            break
        min_lead = ALIGN_LEAD+30.0
        for _ in range(4):
            t0 = sc.ut
            nodes, final = plan(min_lead)
            if not nodes or nodes[0].ut-sc.ut >= ALIGN_LEAD+10.0:
                break
            min_lead = (sc.ut-t0)+60.0
        if not nodes:
            print('Orbit on target; no burn needed')
            break
        tiny = [n for n in nodes if n.delta_v <= args.min_burn_dv]
        if tiny:
            print(f'Skipping {len(tiny)} correction(s) at or below {args.min_burn_dv} m/s: '
                  f'{[round(n.delta_v, 3) for n in tiny]}')
            nodes = [n for n in nodes if n.delta_v > args.min_burn_dv]
            for n in tiny:
                n.remove()
            if not nodes:
                print('Remaining corrections are below the minimum burn; orbit accepted')
                break
        emit_event('trim-plan', label=args.label, burn=burn,
                   dv_mps=[n.delta_v for n in nodes])
        print(f'plan {burn}: final Ap {final.apoapsis_altitude/1000:.2f} Pe '
              f'{final.periapsis_altitude/1000:.2f} inc {math.degrees(final.inclination):.3f} '
              f'(dv {[round(n.delta_v, 1) for n in nodes]})')
        node = nodes[0]
        for extra in nodes[1:]:
            extra.remove()
        goal = getattr(node, 'goal', None)
        if goal is None:
            done = lambda orb: False
        else:
            # Cut off on the orbit itself: stop when the targeted apsis crosses its goal.
            if goal[0] == 'ap':
                measure = lambda orb: orb.apoapsis_altitude
            elif goal[0] == 'pe':
                measure = lambda orb: orb.periapsis_altitude
            else:
                measure = lambda orb, ut=goal[2]: other_apsis(orb, ut, body_radius)
            sign = 1.0 if node.prograde >= 0.0 else -1.0
            done = lambda orb, m=measure, g=goal[1], sg=sign: sg*(m(orb)-g) >= 0.0
        execute(conn, vessel, node, done, orbit_cutoff=goal is not None)
        report(f'after burn {burn}')
    report('final')

if __name__ == '__main__':
    main()
