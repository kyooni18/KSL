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


MIN_BURN_ALTITUDE = 74000.0


def other_apsis(orbit, burn_ut, body_radius):
    """Altitude of the apsis opposite the burn point (which may be Ap or Pe after the burn)."""
    r_b = orbit.radius_at(burn_ut)-body_radius
    if abs(orbit.apoapsis_altitude-r_b) > abs(orbit.periapsis_altitude-r_b):
        return orbit.apoapsis_altitude
    return orbit.periapsis_altitude


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


def execute(conn, vessel, node, done, tolerance=0.03):
    sc = conn.space_center
    ctl = vessel.control
    ap = vessel.auto_pilot
    engines = [e for e in vessel.parts.engines if e.active]
    thrust = sum(e.available_thrust for e in engines)
    isp = sum(e.specific_impulse*e.available_thrust for e in engines)/max(thrust, 1.0)
    if thrust <= 0.0:
        raise RuntimeError('No OMS thrust available (fuel exhausted or engines inactive)')
    dv = node.delta_v
    mass = vessel.mass
    burn_time = isp*9.80665*mass*(1.0-math.exp(-dv/(isp*9.80665)))/thrust
    start = node.ut - 0.5*burn_time
    print(f'Node dv {dv:.2f} m/s, burn {burn_time:.1f} s, T-{node.time_to:.0f} s')
    # Whatever happens below, the engines must never be left running.
    try:
        burn_dir = (0.0, 1.0, 0.0)
        ap.reference_frame = node.reference_frame
        aim(conn, vessel, engines, ap, node.reference_frame, burn_dir)
        ap.target_roll = float('nan')
        ap.engaged = True
        if start-sc.ut > 90:
            sc.warp_to(start-60)
        while sc.ut < start-30:
            aim(conn, vessel, engines, ap, node.reference_frame, burn_dir)
            time.sleep(0.2)
        ap.reference_frame = node.reference_frame
        ctl.rcs = True
        while sc.ut < start:
            aim(conn, vessel, engines, ap, node.reference_frame, burn_dir)
            time.sleep(0.02)
        # Never burn unless the nose is on the burn vector.
        aligned_wait = time.time()
        while ap.error > 3.0 and time.time()-aligned_wait < 120:
            aim(conn, vessel, engines, ap, node.reference_frame, burn_dir)
            time.sleep(0.1)
        ctl.throttle = 1.0
        mass_start = vessel.mass
        while True:
            spent = isp*9.80665*math.log(mass_start/vessel.mass)
            try:
                remaining = node.remaining_delta_v
            except RuntimeError:
                # KSP can drop the node mid-burn; fall back to our own accounting.
                remaining = dv-spent
            if remaining <= tolerance or done(vessel.orbit) or spent > 1.1*dv+0.5:
                print(f'burn end: spent {spent:.2f} of {dv:.2f} m/s, remaining {remaining:.2f}, '
                      f'aim error {ap.error:.1f} deg')
                break
            if vessel.available_thrust <= 0.0:
                print('OMS thrust lost; stopping burn')
                break
            if ap.error > 10.0:
                ctl.throttle = 0.0
                while ap.error > 3.0:
                    aim(conn, vessel, engines, ap, node.reference_frame, burn_dir)
                    time.sleep(0.05)
            aim(conn, vessel, engines, ap, node.reference_frame, burn_dir)
            accel = thrust/vessel.mass
            ctl.throttle = max(0.02, min(1.0, remaining/max(accel, 0.1)/1.5))
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
    args = parser.parse_args()
    conn = krpc.connect(name='orbit-trim')
    sc = conn.space_center
    vessel = sc.active_vessel
    vessel.control.sas = False
    for engine in vessel.parts.engines:
        engine.active = 'Viking' in engine.part.title or 'Orbital' in engine.part.title

    def report(tag):
        o = vessel.orbit
        print(f'{tag}: Ap {o.apoapsis_altitude/1000:.2f} km, Pe {o.periapsis_altitude/1000:.2f} km, '
              f'inc {math.degrees(o.inclination):.3f} deg')

    for stale in list(vessel.control.nodes):
        stale.remove()
    report('start')

    body_radius = vessel.orbit.body.equatorial_radius

    def high_enough(orbit, ut):
        return orbit.radius_at(ut)-body_radius >= MIN_BURN_ALTITUDE

    target_inc = math.radians(args.inclination)
    TOL = 150.0  # m

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
            nodes.append(na)
            base = na.orbit
        pe_ok = abs(base.periapsis_altitude-args.periapsis) < TOL
        inc_ok = abs(math.degrees(base.inclination)-args.inclination) < 0.02
        if not (pe_ok and inc_ok):
            ut_b = base.ut_at_true_anomaly(math.pi)
            if ut_b < now+max(30.0, min_lead):
                ut_b += base.period
            pe_goal = min(args.periapsis, base.apoapsis_altitude)
            x = y = 0.0
            for _ in range(4):
                x = solve(lambda q: vessel.control.add_node(ut_b, prograde=q, normal=y),
                          lambda orb: orb.periapsis_altitude-pe_goal, -250.0, 400.0)
                if not inc_ok:
                    y = solve(lambda q: vessel.control.add_node(ut_b, prograde=x, normal=q),
                              lambda orb: math.degrees(orb.inclination-target_inc), -300.0, 300.0)
            nb = vessel.control.add_node(ut_b, prograde=x, normal=y)
            nodes.append(nb)
            base = nb.orbit
        if abs(base.apoapsis_altitude-args.apoapsis) > TOL:
            ut_c = base.ut_at_true_anomaly(0.0)
            if ut_c < (nodes[-1].ut if nodes else now)+max(60.0, min_lead):
                ut_c += base.period
            xc = solve(lambda q: vessel.control.add_node(ut_c, prograde=q),
                       lambda orb: other_apsis(orb, ut_c, body_radius)-args.apoapsis, -80.0, 80.0)
            nodes.append(vessel.control.add_node(ut_c, prograde=xc))
            base = nodes[-1].orbit
        return nodes, base

    if args.dry_run:
        nodes, final = plan()
        print(f'plan: final Ap {final.apoapsis_altitude/1000:.2f} Pe {final.periapsis_altitude/1000:.2f} '
              f'inc {math.degrees(final.inclination):.3f} (dv {[round(n.delta_v, 1) for n in nodes]})')
        for stale in list(vessel.control.nodes):
            stale.remove()
        return
    for burn in range(1, 7):
        min_lead = 45.0
        for _ in range(4):
            t0 = sc.ut
            nodes, final = plan(min_lead)
            if not nodes or nodes[0].ut-sc.ut >= 30.0:
                break
            min_lead = (sc.ut-t0)+60.0
        if not nodes:
            print('Orbit on target; no burn needed')
            break
        print(f'plan {burn}: final Ap {final.apoapsis_altitude/1000:.2f} Pe '
              f'{final.periapsis_altitude/1000:.2f} inc {math.degrees(final.inclination):.3f} '
              f'(dv {[round(n.delta_v, 1) for n in nodes]})')
        node = nodes[0]
        for extra in nodes[1:]:
            extra.remove()
        done = lambda orb: False
        if node.prograde > 0.0 and node.normal == 0.0 and vessel.orbit.apoapsis_altitude < args.apoapsis-TOL:
            ap_goal = node.orbit.apoapsis_altitude
            done = lambda orb: orb.apoapsis_altitude >= ap_goal
        execute(conn, vessel, node, done)
        report(f'after burn {burn}')
    report('final')

if __name__ == '__main__':
    main()
