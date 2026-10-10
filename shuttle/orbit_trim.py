"""OMS trim: bring a post-MECO orbit to exact apoapsis/periapsis/inclination.

Three small burns planned on kRPC maneuver nodes (each solved against the node's
predicted orbit, so no sign or frame conventions are assumed):

  1. at apoapsis      -- prograde burn that sets the periapsis
  2. at the next node -- normal burn that sets the inclination
  3. at periapsis     -- retrograde/prograde burn that sets the apoapsis
"""

import argparse
import math
import time

import krpc


MIN_BURN_ALTITUDE = 74000.0


def solve(make_node, error, lo, hi, iterations=40):
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


def execute(conn, vessel, node, done, tolerance=0.03):
    sc = conn.space_center
    ctl = vessel.control
    ap = vessel.auto_pilot
    engines = [e for e in vessel.parts.engines if e.active]
    thrust = sum(e.available_thrust for e in engines)
    isp = sum(e.specific_impulse*e.available_thrust for e in engines)/max(thrust, 1.0)
    dv = node.delta_v
    mass = vessel.mass
    burn_time = isp*9.80665*mass*(1.0-math.exp(-dv/(isp*9.80665)))/thrust
    start = node.ut - 0.5*burn_time
    print(f'Node dv {dv:.2f} m/s, burn {burn_time:.1f} s, T-{node.time_to:.0f} s')
    ap.reference_frame = node.reference_frame
    ap.target_direction = (0.0, 1.0, 0.0)
    ap.target_roll = float('nan')
    ap.engaged = True
    if start-sc.ut > 90:
        sc.warp_to(start-60)
    while sc.ut < start-30:
        time.sleep(0.2)
    ap.reference_frame = node.reference_frame
    ap.target_direction = (0.0, 1.0, 0.0)
    ctl.rcs = True
    while sc.ut < start:
        time.sleep(0.02)
    # Never burn unless the nose is on the burn vector.
    aligned_wait = time.time()
    while ap.error > 3.0 and time.time()-aligned_wait < 120:
        time.sleep(0.1)
    ctl.throttle = 1.0
    mass_start = vessel.mass
    while True:
        remaining = node.remaining_delta_v
        spent = isp*9.80665*math.log(mass_start/vessel.mass)
        if remaining <= tolerance or done(vessel.orbit) or spent > 1.25*dv+1.0:
            break
        if ap.error > 10.0:
            ctl.throttle = 0.0
            while ap.error > 3.0:
                time.sleep(0.05)
        accel = thrust/vessel.mass
        ctl.throttle = max(0.02, min(1.0, remaining/max(accel, 0.1)/1.5))
        time.sleep(0.02)
    ctl.rcs = False
    ctl.throttle = 0.0
    ap.engaged = False
    node.remove()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--apoapsis', type=float, default=90000.0)
    parser.add_argument('--periapsis', type=float, default=50000.0)
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

    def plan(d_ap, d_pe):
        """Chain three nodes; return (nodes, final orbit) with Ap/Pe targets offset."""
        for stale in list(vessel.control.nodes):
            stale.remove()
        flight = vessel.flight(vessel.orbit.body.reference_frame)
        lead = max(10.0, (MIN_BURN_ALTITUDE+1500.0-flight.mean_altitude) /
                   max(flight.vertical_speed, 20.0))
        ut1 = sc.ut + lead
        x1 = solve(lambda x: vessel.control.add_node(ut1, prograde=x),
                   lambda orb: orb.apoapsis_altitude-(args.apoapsis+d_ap), -250.0, 250.0)
        n1 = vessel.control.add_node(ut1, prograde=x1)
        ut2 = n1.orbit.ut_at_true_anomaly(math.pi)
        x2 = solve(lambda x: vessel.control.add_node(ut2, prograde=x),
                   lambda orb: orb.periapsis_altitude-(args.periapsis+d_pe), -250.0, 250.0)
        n2 = vessel.control.add_node(ut2, prograde=x2)
        o2 = n2.orbit
        omega = o2.argument_of_periapsis
        candidates = [o2.ut_at_true_anomaly((-omega) % (2*math.pi)),
                      o2.ut_at_true_anomaly((math.pi-omega) % (2*math.pi))]
        usable = [c for c in candidates if c > ut2+120 and c < ut2+900 and high_enough(o2, c)]
        if not usable:
            usable = [c for c in candidates if c > ut2+120 and c < ut2+900 and
                      o2.radius_at(c)-body_radius >= 68000.0]
        ut3 = min(usable) if usable else max(candidates, key=lambda c: o2.radius_at(c) if c > ut2+120 else 0)
        target = math.radians(args.inclination)
        x3 = solve(lambda x: vessel.control.add_node(ut3, normal=x),
                   lambda orb: math.degrees(orb.inclination-target), -400.0, 400.0)
        n3 = vessel.control.add_node(ut3, normal=x3)
        return (n1, n2, n3), n3.orbit

    d_ap = d_pe = 0.0
    for iteration in range(6):
        nodes, final = plan(d_ap, d_pe)
        e_ap = final.apoapsis_altitude-args.apoapsis
        e_pe = final.periapsis_altitude-args.periapsis
        print(f'plan {iteration}: final Ap {final.apoapsis_altitude/1000:.2f} Pe '
              f'{final.periapsis_altitude/1000:.2f} inc {math.degrees(final.inclination):.3f} '
              f'(dv {[round(n.delta_v, 1) for n in nodes]})')
        if abs(e_ap) < 100.0 and abs(e_pe) < 100.0:
            break
        d_ap -= e_ap
        d_pe -= e_pe
    if args.dry_run:
        for stale in list(vessel.control.nodes):
            stale.remove()
        return
    for index, node in enumerate(nodes):
        execute(conn, vessel, node, lambda orb: False)
        report(f'after burn {index+1}')


if __name__ == '__main__':
    main()
