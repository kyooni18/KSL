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
            direction = thruster.thrust_direction(vessel.reference_frame)
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
    parser.add_argument('--periapsis', type=float, default=60000.0)
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
        """Chain nodes in an atmosphere-safe order: raise the periapsis at the first
        apoapsis, lower the apoapsis at that new periapsis, then change the plane at
        the next node. Lowering the apoapsis first would drop the periapsis into the
        atmosphere for the whole coast to the next burn."""
        for stale in list(vessel.control.nodes):
            stale.remove()
        o0 = vessel.orbit
        now = sc.ut
        ut1 = o0.ut_at_true_anomaly(math.pi)
        if ut1 < now+max(30.0, min_lead):
            ut1 += o0.period
        x1 = solve(lambda x: vessel.control.add_node(ut1, prograde=x),
                   lambda orb: orb.periapsis_altitude-(args.periapsis+d_pe), -250.0, 250.0)
        n1 = vessel.control.add_node(ut1, prograde=x1)
        nodes = [n1]
        ut2 = n1.orbit.ut_at_true_anomaly(0.0)
        if ut2 < ut1+120.0:
            ut2 += n1.orbit.period
        x2 = solve(lambda x: vessel.control.add_node(ut2, prograde=x),
                   lambda orb: orb.apoapsis_altitude-(args.apoapsis+d_ap), -250.0, 250.0)
        n2 = vessel.control.add_node(ut2, prograde=x2)
        nodes.append(n2)
        final = n2.orbit
        target = math.radians(args.inclination)
        if abs(math.degrees(final.inclination)-args.inclination) > 0.15:
            omega = final.argument_of_periapsis
            cands = sorted(final.ut_at_true_anomaly(nu) for nu in
                           ((-omega) % (2*math.pi), (math.pi-omega) % (2*math.pi)))
            ut_p = next((c for c in cands if c > ut2+150.0), cands[0]+final.period)
            xp = solve(lambda x: vessel.control.add_node(ut_p, normal=x),
                       lambda orb: math.degrees(orb.inclination-target), -400.0, 400.0)
            n3 = vessel.control.add_node(ut_p, normal=xp)
            nodes.append(n3)
            final = n3.orbit
        return tuple(nodes), final

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
    if nodes[0].ut-sc.ut < 20.0:
        for stale in list(vessel.control.nodes):
            stale.remove()
        raise RuntimeError('Trim plan arrived too late to burn safely; aborting without burning')
    if args.dry_run:
        for stale in list(vessel.control.nodes):
            stale.remove()
        return
    for index, node in enumerate(nodes):
        execute(conn, vessel, node, lambda orb: False)
        report(f'after burn {index+1}')


if __name__ == '__main__':
    main()
