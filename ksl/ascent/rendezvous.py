"""Rendezvous with a target vessel in orbit around the same body.

  1. transfer: cheapest two-impulse Lambert solution (a Hohmann when the orbits
     are circular and coplanar, better otherwise) to a station ``standoff`` m
     behind the target on its velocity vector
  2. midcourse: re-target the same arrival point from the measured state
  3. arrival: match the target's velocity
  4. final approach: closed-loop RCS translation with a closing speed that shrinks
     with range, stopping at ``stop-distance`` with near-zero relative velocity
"""
import argparse
import math
import time

import krpc

from . import orbit_math as om
from .events import emit_event
from .orbit_trim import (RUNTIME, ALIGN_LEAD, controls, execute, from_rh_map, node_for_dv, prepare, rh_state)


def find_target(sc, name):
    if name:
        for vessel in sc.vessels:
            if vessel.name == name and vessel != sc.active_vessel:
                return vessel
        raise RuntimeError(f'Target vessel {name!r} not found')
    target = sc.target_vessel
    if target is None:
        raise RuntimeError('No target: set one in KSP or pass --target')
    return target


def states(vessel, target, frame, mu):
    r, v, to_rh = rh_state(vessel, frame, mu)
    return (r, v), (to_rh(target.position(frame)), to_rh(target.velocity(frame))), to_rh


def burn(conn, vessel, ut, dv_rh, to_rh, frame, label, min_dv):
    if om.norm(dv_rh) <= min_dv:
        print(f'{label}: {om.norm(dv_rh):.3f} m/s below minimum; skipped')
        return
    node = node_for_dv(vessel, ut, from_rh_map(to_rh)(dv_rh), frame)
    emit_event('rendezvous-burn', label=label, dv_mps=node.delta_v, ut=ut)
    print(f'{label}: {node.delta_v:.2f} m/s at UT {ut:.0f}')
    execute(conn, vessel, node, lambda o: False)


def closing_speed(distance, stop, gain=0.02, maximum=3.0, minimum=0.2):
    """Commanded closing speed (m/s) toward the target for a given range."""
    if distance <= stop:
        return 0.0
    return max(minimum, min(maximum, gain*(distance-stop)))


def translation_command(rel_pos, rel_vel, stop, gain=1.0):
    """RCS translation (right, forward, up) from target-relative state in the vessel frame.

    ``rel_pos``/``rel_vel`` are the target's position/velocity relative to us in
    the kRPC vessel frame (x right, y forward, z down). We want the target to come
    toward us at the scheduled closing speed: thrust along (rel_vel - desired).
    """
    distance = om.norm(rel_pos)
    speed = closing_speed(distance, stop)
    desired = om.scale(om.unit(rel_pos), -speed) if distance > 0 else (0.0, 0.0, 0.0)
    error = om.add(rel_vel, desired, -1.0)
    clip = lambda x: max(-1.0, min(1.0, gain*x))
    return clip(error[0]), clip(error[1]), clip(-error[2])


def final_approach(sc, vessel, target, stop, timeout=1800.0):
    ctl = vessel.control
    nrf = vessel.orbit.body.non_rotating_reference_frame
    ctl.sas = True
    try:
        ctl.sas_mode = sc.SASMode.stability_assist
    except Exception:
        pass
    ctl.rcs = True
    start = time.monotonic()
    try:
        while time.monotonic() - start < timeout:
            rel_pos = target.position(vessel.reference_frame)
            rel_vel_nrf = om.add(target.velocity(nrf), vessel.velocity(nrf), -1.0)
            rel_vel = sc.transform_direction(rel_vel_nrf, nrf, vessel.reference_frame)
            distance = om.norm(rel_pos)
            if distance <= stop and om.norm(rel_vel) < 0.1:
                break
            ctl.right, ctl.forward, ctl.up = translation_command(rel_pos, rel_vel, stop)
            time.sleep(0.1)
    finally:
        ctl.right = ctl.forward = ctl.up = 0.0
        ctl.rcs = False
    return distance, om.norm(rel_vel)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', default='', help='Target vessel name (default: KSP target)')
    parser.add_argument('--standoff', type=float, default=1000.0,
                        help='Transfer arrival point behind the target (m)')
    parser.add_argument('--stop-distance', type=float, default=50.0,
                        help='Final approach stops at this range (m)')
    parser.add_argument('--min-burn-dv', type=float, default=0.1)
    parser.add_argument('--rcs-max-dv', type=float, default=2.0,
                        help='Burns up to this dv (m/s) fly as RCS forward translation')
    parser.add_argument('--max-wait-orbits', type=float, default=6.0,
                        help='Longest departure delay searched, in chaser orbits')
    parser.add_argument('--address', default='127.0.0.1')
    parser.add_argument('--rpc-port', type=int, default=50000)
    parser.add_argument('--stream-port', type=int, default=50001)
    args = parser.parse_args()
    conn = krpc.connect(name='rendezvous', address=args.address,
                        rpc_port=args.rpc_port, stream_port=args.stream_port)
    sc, vessel = prepare(conn)
    RUNTIME['rcs_max_dv'] = args.rcs_max_dv
    if not controls().trim_enabled:
        print('Orbit operations disabled by the operator')
        return
    target = find_target(sc, args.target)
    body = vessel.orbit.body
    if target.orbit.body != body:
        raise RuntimeError('Target orbits a different body')
    frame = body.non_rotating_reference_frame
    mu, radius = body.gravitational_parameter, body.equatorial_radius
    for stale in list(vessel.control.nodes):
        stale.remove()

    chaser, tgt, to_rh = states(vessel, target, frame, mu)
    period = vessel.orbit.period
    t_period = target.orbit.period
    lead = ALIGN_LEAD + 90.0
    plan = om.plan_rendezvous(chaser, tgt, mu, (lead, lead + args.max_wait_orbits*period),
                              (0.2*min(period, t_period), 1.5*max(period, t_period)),
                              standoff=args.standoff,
                              min_departure_alt_radius=radius + body.atmosphere_depth + 5000.0)
    if plan is None:
        raise RuntimeError('No rendezvous transfer found')
    now = sc.ut
    t_depart, t_arrive = now + plan['delay'], now + plan['delay'] + plan['tof']
    emit_event('rendezvous-plan', target=target.name, depart_ut=t_depart, arrive_ut=t_arrive,
               dv1_mps=plan['dv1'], dv2_mps=plan['dv2'])
    print(f'Rendezvous with {target.name}: depart in {plan["delay"]:.0f} s, flight {plan["tof"]:.0f} s, '
          f'dv {plan["dv1"]:.1f} + {plan["dv2"]:.1f} m/s')
    burn(conn, vessel, t_depart, plan['dv1_vec'], to_rh, frame, 'transfer', args.min_burn_dv)

    # midcourse: re-solve to the same arrival point from the measured state
    chaser, tgt, to_rh = states(vessel, target, frame, mu)
    t_mid = max(sc.ut + lead, 0.5*(sc.ut + t_arrive))
    if t_arrive - t_mid > 120.0:
        r_c, v_c = om.propagate(chaser[0], chaser[1], mu, t_mid - sc.ut)
        r_t, v_t = om.propagate(tgt[0], tgt[1], mu, t_arrive - sc.ut)
        aim = om.station_offset(r_t, v_t, args.standoff)
        solution = om.lambert(r_c, aim, t_arrive - t_mid, mu)
        if solution is not None:
            burn(conn, vessel, t_mid, om.add(solution[0], v_c, -1.0), to_rh, frame,
                 'midcourse', max(args.min_burn_dv, 0.2))

    # arrival: match velocity
    chaser, tgt, to_rh = states(vessel, target, frame, mu)
    dt = t_arrive - sc.ut
    _, v_c = om.propagate(chaser[0], chaser[1], mu, dt)
    _, v_t = om.propagate(tgt[0], tgt[1], mu, dt)
    burn(conn, vessel, t_arrive, om.add(v_t, v_c, -1.0), to_rh, frame, 'match', args.min_burn_dv)

    distance, rel_speed = final_approach(sc, vessel, target, args.stop_distance)
    emit_event('rendezvous', target=target.name, distance_m=distance, relative_speed_mps=rel_speed)
    print(f'Rendezvous complete: {distance:.0f} m from {target.name}, '
          f'relative speed {rel_speed:.2f} m/s')


if __name__ == '__main__':
    main()
