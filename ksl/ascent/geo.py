"""GEO/GTO planning: where the apogee lands in longitude, and how to place it.

The ET leaves the stack in a semi-GTO (perigee at MECO, apogee well below GEO).
The OMS then raises it to the final GTO with a burn at a later perigee passage.
Waiting ``n`` revolutions of the semi-GTO before that burn slides the apogee, which
stays over a fixed inertial direction, under a rotating body: the apogee
longitude is

    L = L_perigee(t_p0) + 180 - w * (n * P_semi + P_gto / 2)        (deg, east +)

(w = body rotation rate in deg/s). A small phasing burn at the first perigee
retunes P_semi continuously, so any longitude is reachable for modest dv.
"""
import math

from . import orbit_math as om

KERBIN = {'mu': 3.5316e12, 'radius': 600000.0, 'rotation_period': 21549.425,
          'atmosphere': 70000.0}


def wrap180(angle_deg):
    return (angle_deg + 180.0) % 360.0 - 180.0


def synchronous_altitude(body=KERBIN):
    return om.synchronous_radius(body['mu'], body['rotation_period']) - body['radius']


def rotation_rate_deg(body=KERBIN):
    return 360.0/body['rotation_period']


def apogee_longitude(perigee_lon_deg, revs, semi_period, gto_period, rate_deg):
    """Longitude (deg) of the first apogee after ``revs`` semi-GTO revolutions."""
    return wrap180(perigee_lon_deg + 180.0 - rate_deg*(revs*semi_period + 0.5*gto_period))


def period_from_apsides(mu, radius, apoapsis_alt, periapsis_alt):
    return om.period(mu, radius + 0.5*(apoapsis_alt + periapsis_alt))


def apoapsis_for_period(mu, radius, periapsis_alt, target_period):
    """Apoapsis altitude of the orbit with this periapsis and period."""
    a = (mu*(target_period/om.TAU)**2)**(1.0/3.0)
    return 2.0*a - (radius + periapsis_alt) - radius


def phasing_plan(perigee_lon_deg, semi_period, gto_period, target_lon_deg, rate_deg,
                 mu, radius, periapsis_alt, max_revs=6, min_apoapsis_margin=20000.0,
                 max_period_change=0.6):
    """Choose revolutions and a phasing period so the GTO apogee lands on the target.

    Returns the options sorted by phasing cost (|dP| of the semi-GTO). Each option is
    {revs, period, dperiod, apoapsis_alt, dv}: ``dv`` is the (signed, prograde +)
    perigee burn that retunes the semi-GTO to ``period``; revs == 0 has no phasing
    freedom and reports the unmodified longitude error instead.
    """
    options = []
    rp = radius + periapsis_alt
    v_circ_floor = om.period(mu, rp)  # a period below this would put apoapsis under perigee
    base = (perigee_lon_deg + 180.0 - target_lon_deg - rate_deg*0.5*gto_period)
    for revs in range(1, max_revs+1):
        # need  rate * revs * P' = base (mod 360)  ->  P' = (base + 360 j) / (rate * revs)
        j = math.floor((rate_deg*revs*semi_period*0.5 - base)/360.0)
        for k in (j-1, j, j+1, j+2):
            period = ((base % 360.0) + 360.0*k)/(rate_deg*revs)
            if period <= v_circ_floor*1.001:
                continue
            if abs(period-semi_period) > max_period_change*semi_period:
                continue
            a_new = (mu*(period/om.TAU)**2)**(1.0/3.0)
            apoapsis_alt = 2.0*a_new - rp - radius
            if apoapsis_alt < periapsis_alt + min_apoapsis_margin:
                continue
            a_old = (mu*(semi_period/om.TAU)**2)**(1.0/3.0)
            v_old = math.sqrt(mu*(2.0/rp - 1.0/a_old))
            v_new = math.sqrt(mu*(2.0/rp - 1.0/a_new))
            options.append({'revs': revs, 'period': period, 'dperiod': period-semi_period,
                            'apoapsis_alt': apoapsis_alt, 'dv': v_new-v_old,
                            'longitude': target_lon_deg})
    options.sort(key=lambda o: (abs(o['dv']), o['revs']))
    direct = {'revs': 0, 'period': semi_period, 'dperiod': 0.0, 'dv': 0.0,
              'apoapsis_alt': None,
              'longitude': apogee_longitude(perigee_lon_deg, 0, semi_period, gto_period, rate_deg)}
    return options, direct


def design_semi_gto_apoapsis(mu, radius, periapsis_alt, gto_apoapsis_alt, perigee_lon_deg,
                             target_lon_deg, rate_deg, revs=2):
    """ET target apoapsis whose ``revs``-revolution coast already lines the apogee up.

    The nominal ascent aims the ET's semi-GTO so that no (or only a trim-sized)
    phasing burn is needed afterwards. Returns None if the required period is not
    a bound orbit above the perigee.
    """
    gto_period = period_from_apsides(mu, radius, gto_apoapsis_alt, periapsis_alt)
    rp = radius + periapsis_alt
    base = (perigee_lon_deg + 180.0 - target_lon_deg - rate_deg*0.5*gto_period)
    floor = om.period(mu, rp)
    best = None
    for k in range(-2, 8):
        period = ((base % 360.0) + 360.0*k)/(rate_deg*revs)
        if period <= floor*1.05:
            continue
        a = (mu*(period/om.TAU)**2)**(1.0/3.0)
        apoapsis_alt = 2.0*a - rp - radius
        if apoapsis_alt >= gto_apoapsis_alt:
            continue
        if best is None or apoapsis_alt > best['apoapsis_alt']:
            best = {'apoapsis_alt': apoapsis_alt, 'period': period, 'revs': revs}
    return best
