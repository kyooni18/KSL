"""GTO/GEO orbit operations after an ET semi-GTO insertion.

  1. (optional) phasing burn at the first perigee so the GTO apogee will sit over
     the requested GEO longitude (ksl.ascent.geo.phasing_plan)
  2. GTO injection at the chosen perigee passage: apoapsis to synchronous altitude
  3. circularize at apogee into GEO with the exact one-burn trim, if the OMS has
     the delta-v (mode auto), always, or never (stay in the GTO)
"""
import argparse
import math
from types import SimpleNamespace

import krpc

from . import geo
from .events import emit_event
from .orbit_trim import (ALIGN_LEAD, _trim, controls, execute, oms_delta_v, prepare, solve)


def perigee_longitude(sc, vessel, t_perigee):
    """Body-fixed longitude (deg) of the perigee point at the moment it is passed."""
    body = vessel.orbit.body
    nrf = body.non_rotating_reference_frame
    position = vessel.orbit.position_at(t_perigee, nrf)
    rotating = sc.transform_position(position, nrf, body.reference_frame)
    lon_now = body.longitude_at_position(rotating, body.reference_frame)
    rate = math.degrees(body.rotational_speed)
    return geo.wrap180(lon_now - rate*(t_perigee - sc.ut))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--perigee', type=float, required=True, help='GTO perigee altitude (m)')
    parser.add_argument('--inclination', type=float, default=0.0)
    parser.add_argument('--geo-altitude', type=float, default=None,
                        help='Apogee/GEO altitude (m); default: synchronous altitude of the body')
    parser.add_argument('--longitude', type=float, default=None,
                        help='Target GEO longitude (deg east); omit for no phasing')
    parser.add_argument('--circularize', choices=('auto', 'always', 'never'), default='auto')
    parser.add_argument('--reserve-dv', type=float, default=0.0,
                        help='OMS dv (m/s) to keep in reserve when deciding to circularize')
    parser.add_argument('--max-revs', type=int, default=6)
    parser.add_argument('--min-burn-dv', type=float, default=0.1)
    parser.add_argument('--address', default='127.0.0.1')
    parser.add_argument('--rpc-port', type=int, default=50000)
    parser.add_argument('--stream-port', type=int, default=50001)
    args = parser.parse_args()
    conn = krpc.connect(name='geo-transfer', address=args.address,
                        rpc_port=args.rpc_port, stream_port=args.stream_port)
    sc, vessel = prepare(conn)
    ctl = controls()
    if not ctl.trim_enabled:
        emit_event('trim-skipped', label='gto', reason='disabled')
        print('Orbit operations disabled by the operator')
        return
    if ctl.hold_trim:
        emit_event('trim-held', label='gto')
        print('Holding: trigger orbit operations to start the GTO sequence')
        while not ctl.take('trimNow') and ctl.hold_trim and ctl.trim_enabled:
            import time
            time.sleep(0.5)
    body = vessel.orbit.body
    mu, radius = body.gravitational_parameter, body.equatorial_radius
    body_info = {'mu': mu, 'radius': radius, 'rotation_period': body.rotational_period}
    geo_alt = args.geo_altitude or geo.synchronous_altitude(body_info)
    rate = geo.rotation_rate_deg(body_info)
    for stale in list(vessel.control.nodes):
        stale.remove()

    orbit = vessel.orbit
    t_p0 = sc.ut + orbit.time_to_periapsis
    if t_p0 - sc.ut < ALIGN_LEAD + 60.0:
        t_p0 += orbit.period
    semi = orbit.period
    perigee_alt = orbit.periapsis_altitude
    gto_period = geo.period_from_apsides(mu, radius, geo_alt, perigee_alt)
    lon_p = perigee_longitude(sc, vessel, t_p0)
    revs = 0
    if args.longitude is not None:
        options, direct = geo.phasing_plan(lon_p, semi, gto_period, args.longitude, rate, mu, radius,
                                           perigee_alt, max_revs=args.max_revs)
        if not options:
            raise RuntimeError('No phasing option reaches the requested GEO longitude')
        best = options[0]
        revs = best['revs']
        emit_event('geo-plan', perigee_lon=lon_p, target_lon=args.longitude, revs=revs,
                   phasing_dv=best['dv'], semi_period=semi, phased_period=best['period'],
                   unphased_lon=direct['longitude'])
        print(f'GEO plan: perigee lon {lon_p:.2f}, target {args.longitude:.2f} deg; '
              f'{revs} rev(s) at period {best["period"]:.0f} s (now {semi:.0f}); '
              f'phasing burn {best["dv"]:+.2f} m/s at perigee')
        if abs(best['dv']) > args.min_burn_dv:
            node = vessel.control.add_node(t_p0, prograde=best['dv'])
            execute(conn, vessel, node, lambda o: False)
        semi = vessel.orbit.period
    t_burn = t_p0 + revs*semi
    if t_burn - sc.ut < ALIGN_LEAD + 60.0:
        t_burn += semi
    x = solve(lambda q: vessel.control.add_node(t_burn, prograde=q),
              lambda o: o.apoapsis_altitude - geo_alt, 0.0, 2000.0)
    node = vessel.control.add_node(t_burn, prograde=x)
    predicted = node.orbit
    t_apo = predicted.ut_at_true_anomaly(math.pi)
    print(f'GTO injection: {x:.1f} m/s at UT {t_burn:.0f}; apogee {geo_alt/1000:.1f} km at UT {t_apo:.0f}')
    execute(conn, vessel, node, lambda o: o.apoapsis_altitude >= geo_alt, orbit_cutoff=True)
    o = vessel.orbit
    emit_event('gto', apoapsis_m=o.apoapsis_altitude, periapsis_m=o.periapsis_altitude,
               inclination_deg=math.degrees(o.inclination))
    t_apo = sc.ut + o.time_to_apoapsis
    position = o.position_at(t_apo, body.non_rotating_reference_frame)
    lon_apo = geo.wrap180(body.longitude_at_position(
        sc.transform_position(position, body.non_rotating_reference_frame, body.reference_frame),
        body.reference_frame) - rate*(t_apo - sc.ut))
    emit_event('gto-apogee', longitude=lon_apo, target=args.longitude, ut=t_apo)
    print(f'GTO: Ap {o.apoapsis_altitude/1000:.1f} km, Pe {o.periapsis_altitude/1000:.1f} km; '
          f'apogee over {lon_apo:.2f} deg' +
          ('' if args.longitude is None else f' (target {args.longitude:.2f})'))

    if args.circularize == 'never':
        print('GTO profile: no circularization')
        return
    r_apo = radius + o.apoapsis_altitude
    need = math.sqrt(mu/r_apo) - math.sqrt(mu*(2.0/r_apo - 1.0/o.semi_major_axis))
    have = oms_delta_v(vessel)
    emit_event('trim-budget', label='geo', needed_mps=need, available_mps=have)
    if args.circularize == 'auto' and have < need + args.reserve_dv:
        print(f'OMS has {have:.0f} m/s, GEO circularization needs {need:.0f}+{args.reserve_dv:.0f}; '
              'staying in GTO')
        emit_event('trim-skipped', label='geo', reason='insufficient-dv')
        return
    _trim(conn, sc, vessel, SimpleNamespace(
        apoapsis=geo_alt, periapsis=geo_alt, inclination=args.inclination, dry_run=False,
        min_burn_dv=args.min_burn_dv, max_burns=2, legacy_plan=False, skip_if_short_dv=None,
        label='geo', no_hold=True))
    emit_event('geo', longitude=lon_apo)


if __name__ == '__main__':
    main()
