"""Launch-window geometry: when does the pad pass through a target orbital plane.

Pure functions in the right-handed inertial frame of ``orbit_math`` (+z north).
The target plane is given by its unit normal (the angular-momentum direction of a
prograde orbit), taken from a body's or vessel's actual state vectors so no node
or LAN convention has to be assumed.
"""
import math

from . import orbit_math as om


def crossings(r_site, omega, plane_normal):
    """Times (s from now, ascending) at which the rotating pad lies in the plane.

    ``omega`` is the body's signed angular rate about +z (rad/s, east = positive).
    The pad stays at constant latitude, so the plane is reachable only if its
    inclination is at least the pad latitude; otherwise the list is empty.
    """
    n = om.unit(plane_normal)
    x, y, z = r_site
    a = n[0]*x + n[1]*y
    b = n[1]*x - n[0]*y
    c = n[2]*z
    amp = math.hypot(a, b)
    if amp < 1e-9 or abs(c) > amp:
        return []
    phi0 = math.atan2(b, a)
    spread = math.acos(-c/amp)
    period = om.TAU/abs(omega)
    out = []
    for theta in (phi0 + spread, phi0 - spread):
        # r(t) is rotated by omega*t about +z; theta is the rotation angle needed
        t = (theta/omega) % period if omega > 0 else (-theta/(-omega)) % period
        out.append(t)
    return sorted(out)


def rotate_z(v, angle):
    c, s = math.cos(angle), math.sin(angle)
    return (v[0]*c - v[1]*s, v[0]*s + v[1]*c, v[2])


def launch_direction(r_site, plane_normal):
    """Horizontal direction of motion at the pad for a prograde orbit in the plane."""
    return om.unit(om.cross(plane_normal, om.unit(r_site)))


def azimuth_deg(r_site, direction):
    """Compass azimuth (deg from north, eastward) of a horizontal direction at r_site."""
    rh = om.unit(r_site)
    east = om.unit(om.cross(om.POLE, rh))
    north = om.cross(rh, east)
    return math.degrees(math.atan2(om.dot(direction, east), om.dot(direction, north))) % 360.0


def windows(r_site, omega, plane_normal, horizon, northbound_only=True):
    """Upcoming launch windows within ``horizon`` seconds, each with its azimuth.

    Returns dicts {t, azimuth, northbound}. A window repeats every sidereal day for
    each of the two plane crossings (ascending/descending, i.e. north/southbound).
    """
    day = om.TAU/abs(omega)
    result = []
    for t0 in crossings(r_site, omega, plane_normal):
        k = 0
        while t0 + k*day <= horizon:
            t = t0 + k*day
            r_t = rotate_z(r_site, omega*t)
            d = launch_direction(r_t, plane_normal)
            rh = om.unit(r_t)
            north_component = om.dot(d, om.cross(rh, om.unit(om.cross(om.POLE, rh))))
            window = {'t': t, 'azimuth': azimuth_deg(r_t, d), 'northbound': north_component >= 0.0}
            if window['northbound'] or not northbound_only:
                result.append(window)
            k += 1
    return sorted(result, key=lambda w: w['t'])


def best_rendezvous_window(r_site, omega, plane_normal, chaser_state_after_launch,
                           target_state, mu, ascent_time, horizon, standoff=0.0,
                           tof_range=None, northbound_only=True, orbit_radius=None):
    """Pick the window (within ``horizon``) with the shortest launch-to-rendezvous time.

    ``chaser_state_after_launch(window)`` returns the (r, v) state the shuttle will
    have at insertion for a launch at that window (the caller owns the ascent
    model); ``target_state`` is the target's (r, v) now. The score is the wait plus
    ascent time plus the best Lambert transfer time-of-flight, tie-broken by dv.
    """
    best = None
    for window in windows(r_site, omega, plane_normal, horizon, northbound_only):
        t_ins = window['t'] + ascent_time
        chaser = chaser_state_after_launch(window)
        target = om.propagate(target_state[0], target_state[1], mu, t_ins)
        r_c = om.norm(chaser[0])
        period = om.TAU*math.sqrt((orbit_radius or r_c)**3/mu)
        plan = om.plan_rendezvous(chaser, target, mu, (0.0, period), tof_range or (0.25*period, 1.5*period),
                                  standoff=standoff, delay_samples=24, tof_samples=24)
        if plan is None:
            continue
        total = t_ins + plan['delay'] + plan['tof']
        score = total + 20.0*plan['dv']
        if best is None or score < best['score']:
            best = {'window': window, 'plan': plan, 'score': score, 'total_time': total}
    return best
