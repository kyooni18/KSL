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

    def step_periapsis():
        o = vessel.orbit
        ut = sc.ut + o.time_to_apoapsis
        make = lambda x: vessel.control.add_node(ut, prograde=x)
        dv = solve(make, lambda orb: orb.periapsis_altitude-args.periapsis, -150.0, 150.0)
        return ut, dict(prograde=dv), lambda orb: orb.periapsis_altitude >= args.periapsis-30.0 if dv > 0 else orb.periapsis_altitude <= args.periapsis+30.0

    def step_inclination():
        o = vessel.orbit
        omega = o.argument_of_periapsis
        candidates = []
        for anomaly in ((-omega) % (2*math.pi), (math.pi-omega) % (2*math.pi)):
            candidates.append(o.ut_at_true_anomaly(anomaly))
        body_radius = o.body.equatorial_radius
        usable = [c for c in candidates if c > sc.ut+120 and
                  o.radius_at(c)-body_radius >= MIN_BURN_ALTITUDE]
        ut = min(usable) if usable else max(candidates, key=lambda c: o.radius_at(c) if c > sc.ut+120 else 0)
        target = math.radians(args.inclination)
        make = lambda x: vessel.control.add_node(ut, normal=x)
        dv = solve(make, lambda orb: math.degrees(orb.inclination-target), -400.0, 400.0)
        target_deg = args.inclination
        return ut, dict(normal=dv), lambda orb: abs(math.degrees(orb.inclination)-target_deg) < 0.01

    def step_apoapsis():
        # Set the apoapsis from the first point above the atmosphere (a burn at the
        # low periapsis would sit in the air at ~2.3 km/s).
        o = vessel.orbit
        body_radius = o.body.equatorial_radius
        flight = vessel.flight(o.body.reference_frame)
        lead = max(10.0, (MIN_BURN_ALTITUDE+1500.0-flight.mean_altitude)/max(flight.vertical_speed, 20.0))
        ut = sc.ut + lead
        make = lambda x: vessel.control.add_node(ut, prograde=x)
        dv = solve(make, lambda orb: orb.apoapsis_altitude-args.apoapsis, -250.0, 250.0)
        return ut, dict(prograde=dv), lambda orb: orb.apoapsis_altitude <= args.apoapsis+30.0 if dv < 0 else orb.apoapsis_altitude >= args.apoapsis-30.0

    for name, planner in (('apoapsis', step_apoapsis),
                          ('periapsis', step_periapsis),
                          ('inclination', step_inclination)):
        ut, comps, done = planner()
        node = vessel.control.add_node(ut, **comps)
        print(f'{name}: node at T+{ut-sc.ut:.0f} s, components {comps}, '
              f'predicted Ap {node.orbit.apoapsis_altitude/1000:.2f} Pe '
              f'{node.orbit.periapsis_altitude/1000:.2f} inc {math.degrees(node.orbit.inclination):.3f}')
        if args.dry_run:
            node.remove()
            continue
        execute(conn, vessel, node, done)
        report(f'after {name}')


if __name__ == '__main__':
    main()
