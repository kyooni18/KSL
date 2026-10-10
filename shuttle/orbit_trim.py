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

    def plan(d_ap, d_pe, min_lead=45.0):
        """Chain up to three nodes in time order: plane change at the first high node
        after MECO, apoapsis set shortly after, periapsis raised at apoapsis."""
        for stale in list(vessel.control.nodes):
            stale.remove()
        o0 = vessel.orbit
        omega = o0.argument_of_periapsis
        now = sc.ut
        candidates = sorted(
            o0.ut_at_true_anomaly(nu) for nu in ((-omega) % (2*math.pi), (math.pi-omega) % (2*math.pi)))
        usable = [c for c in candidates if c > now+min_lead and c < now+900 and
                  o0.radius_at(c)-body_radius >= MIN_BURN_ALTITUDE+3000.0]
        target = math.radians(args.inclination)
        nodes = []
        if usable:
            ut_p = usable[0]
            xp = solve(lambda x: vessel.control.add_node(ut_p, normal=x),
                       lambda orb: math.degrees(orb.inclination-target), -400.0, 400.0)
            nodes.append(vessel.control.add_node(ut_p, normal=xp))
            ut1 = ut_p + 0.5*abs(xp)/2.7*1.0 + 70.0
        else:
            print('No safe plane-change node; skipping the inclination burn')
            flight = vessel.flight(o0.body.reference_frame)
            ut1 = now + max(10.0, (MIN_BURN_ALTITUDE+1500.0-flight.mean_altitude) /
                            max(flight.vertical_speed, 20.0))
        base_orbit = nodes[-1].orbit if nodes else o0
        ut1 = max(ut1, now + 15.0)
        x1 = solve(lambda x: vessel.control.add_node(ut1, prograde=x),
                   lambda orb: orb.apoapsis_altitude-(args.apoapsis+d_ap), -250.0, 250.0)
        n1 = vessel.control.add_node(ut1, prograde=x1)
        nodes.append(n1)
        ut2 = n1.orbit.ut_at_true_anomaly(math.pi)
        x2 = solve(lambda x: vessel.control.add_node(ut2, prograde=x),
                   lambda orb: orb.periapsis_altitude-(args.periapsis+d_pe), -250.0, 250.0)
        n2 = vessel.control.add_node(ut2, prograde=x2)
        nodes.append(n2)
        return tuple(nodes), n2.orbit

    d_ap = d_pe = 0.0
    min_lead = 45.0
    t_start = sc.ut
    for iteration in range(8):
        nodes, final = plan(d_ap, d_pe, min_lead)
        elapsed = sc.ut-t_start
        if nodes[0].ut-sc.ut < 40.0:
            # Planning takes game time; leave room for it before the first burn.
            min_lead = elapsed+60.0
            print(f'planning took {elapsed:.0f} s; re-planning with lead {min_lead:.0f} s')
            continue
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
