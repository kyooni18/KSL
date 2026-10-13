"""Pure two-body orbital mechanics for trim, GTO/GEO and rendezvous planning.

All vectors are plain 3-tuples in a right-handed inertial frame whose +z axis is
the body's north pole. kRPC frames are left-handed with +y north; convert with
``from_krpc`` / ``to_krpc`` (swap y and z) before using these functions.

Nothing here talks to kRPC, so every planner built on it is testable offline.
"""
import math

TAU = 2.0*math.pi


def from_krpc(v):
    """kRPC (x, y=north, z) left-handed -> right-handed (x, y, z=north)."""
    return (v[0], v[2], v[1])


to_krpc = from_krpc  # the swap is its own inverse


def dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    n = norm(a)
    return (a[0]/n, a[1]/n, a[2]/n) if n > 0 else (0.0, 0.0, 0.0)


def add(a, b, k=1.0):
    return (a[0]+k*b[0], a[1]+k*b[1], a[2]+k*b[2])


def scale(a, k):
    return (a[0]*k, a[1]*k, a[2]*k)


POLE = (0.0, 0.0, 1.0)
INEXACT_PENALTY = 20.0  # m/s of extra dv worth paying to finish in a single burn


def elements(r, v, mu):
    """Apsis radii, inclination and period of the orbit through state (r, v)."""
    rn, vn = norm(r), norm(v)
    h = cross(r, v)
    hn = norm(h)
    energy = 0.5*vn*vn - mu/rn
    if energy >= 0.0 or hn <= 0.0:
        return {'a': math.inf, 'e': math.inf, 'ra': math.inf, 'rp': hn*hn/(2*mu) if hn else 0.0,
                'inc': math.degrees(math.acos(max(-1.0, min(1.0, h[2]/hn)))) if hn else 0.0,
                'period': math.inf, 'energy': energy, 'h': hn}
    a = -mu/(2.0*energy)
    e = math.sqrt(max(0.0, 1.0 - hn*hn/(mu*a)))
    return {'a': a, 'e': e, 'ra': a*(1.0+e), 'rp': a*(1.0-e),
            'inc': math.degrees(math.acos(max(-1.0, min(1.0, h[2]/hn)))),
            'period': TAU*math.sqrt(a**3/mu), 'energy': energy, 'h': hn}


def _stumpff(z):
    if z > 1e-8:
        s = math.sqrt(z)
        return (1.0-math.cos(s))/z, (s-math.sin(s))/(s**3)
    if z < -1e-8:
        s = math.sqrt(-z)
        return (math.cosh(s)-1.0)/(-z), (math.sinh(s)-s)/(s**3)
    return 0.5 - z/24.0, 1.0/6.0 - z/120.0


def propagate(r0, v0, mu, dt):
    """Kepler (universal variable) propagation of a state by dt seconds."""
    if dt == 0.0:
        return r0, v0
    r0n = norm(r0)
    alpha = 2.0/r0n - dot(v0, v0)/mu
    if alpha > 1e-12:
        period = TAU/(math.sqrt(mu)*alpha**1.5)
        dt = math.fmod(dt, period)
        if dt == 0.0:
            return r0, v0
    sq = math.sqrt(mu)
    vr0 = dot(r0, v0)/r0n
    chi = sq*abs(alpha)*dt if abs(alpha) > 1e-12 else sq*dt/r0n
    for _ in range(80):
        z = alpha*chi*chi
        c, s = _stumpff(z)
        f = r0n*vr0/sq*chi*chi*c + (1.0-alpha*r0n)*chi**3*s + r0n*chi - sq*dt
        df = r0n*vr0/sq*chi*(1.0-z*s) + (1.0-alpha*r0n)*chi*chi*c + r0n
        step = f/df
        chi -= step
        if abs(step) < 1e-9*max(1.0, abs(chi)):
            break
    z = alpha*chi*chi
    c, s = _stumpff(z)
    fg = 1.0 - chi*chi/r0n*c
    gg = dt - chi**3/sq*s
    r = add(scale(r0, fg), v0, gg)
    rn = norm(r)
    fd = sq/(rn*r0n)*(z*s-1.0)*chi
    gd = 1.0 - chi*chi/rn*c
    return r, add(scale(r0, fd), v0, gd)


def local_frame(r, v):
    """(prograde, normal, radial-out) unit vectors for a burn at state (r, v)."""
    p = unit(v)
    n = unit(cross(r, v))
    return p, n, unit(cross(p, n))


def node_components(r, v, dv):
    """Burn dv vector -> (prograde, normal, radial) components."""
    p, n, rad = local_frame(r, v)
    return dot(dv, p), dot(dv, n), dot(dv, rad)


def horizontal_directions(r, inclination_deg):
    """Horizontal unit vectors at r whose orbital plane has the given inclination.

    Returns up to two (northbound and southbound) plus a flag telling whether the
    inclination is reachable from this latitude (otherwise the closest plane).
    """
    rh = unit(r)
    east_raw = cross(POLE, rh)
    cos_lat = norm(east_raw)
    if cos_lat < 1e-9:  # over a pole: every direction is northbound/southbound
        east = (1.0, 0.0, 0.0)
        cos_lat = 1e-9
    else:
        east = scale(east_raw, 1.0/cos_lat)
    north = cross(rh, east)
    sin_beta = math.cos(math.radians(inclination_deg))/cos_lat
    reachable = abs(sin_beta) <= 1.0
    sin_beta = max(-1.0, min(1.0, sin_beta))
    cos_beta = math.sqrt(1.0 - sin_beta*sin_beta)
    out = []
    for cb in ((cos_beta, -cos_beta) if cos_beta > 1e-9 else (0.0,)):
        out.append(add(scale(north, cb), east, sin_beta))
    return out, reachable


def target_velocities(r, mu, ra, rp, inclination_deg, radial_slack=20.0):
    """Velocities at r that lie on the (ra, rp, inclination) orbit.

    Yields (velocity, exact) pairs: the horizontal direction is set by the
    inclination, the horizontal speed by the angular momentum and the radial speed
    by the energy. Radii slightly outside [rp, ra] are accepted with the radial
    speed clamped to zero (``exact`` False) so near-circular targets stay usable
    when the burn point is not exactly on the target radius.
    """
    rn = norm(r)
    if rn < rp - 1.0 or rn > ra + 1.0:
        slack_only = True
    else:
        slack_only = False
    energy = -mu/(ra+rp)
    h = math.sqrt(2.0*mu*ra*rp/(ra+rp))
    vh = h/rn
    vr2 = 2.0*(energy + mu/rn) - vh*vh
    exact = vr2 >= -0.01 and not slack_only  # (0.1 m/s)^2 of numerical slack at a crossing
    if vr2 < 0.0:
        if vr2 < -radial_slack*radial_slack:
            return []
        vr2 = 0.0
    vr = math.sqrt(vr2)
    rh = unit(r)
    directions, reachable = horizontal_directions(r, inclination_deg)
    exact = exact and reachable  # a plane that does not contain this point is only approximated
    out = []
    for d in directions:
        for sign in ((1.0, -1.0) if vr > 1e-6 else (1.0,)):
            out.append((add(scale(d, vh), rh, sign*vr), exact))
    return out


def solve_one_burn(r0, v0, mu, ra_alt, rp_alt, inclination_deg, body_radius,
                   min_lead=0.0, window=None, samples=720, deadline=None):
    """Cheapest single impulsive burn reaching exact apsides and inclination.

    Scans the burn time over ``window`` seconds (default one period), at each time
    solving the velocity that puts the orbit on the target (apoapsis altitude,
    periapsis altitude, inclination) with a free node and argument of periapsis.
    ``deadline`` (seconds from now) bounds the burn time, e.g. before a periapsis
    pass that would dip into the atmosphere. Returns None if nothing is feasible.
    """
    ra, rp = body_radius + max(ra_alt, rp_alt), body_radius + min(ra_alt, rp_alt)
    el = elements(r0, v0, mu)
    if window is None:
        window = el['period'] if math.isfinite(el['period']) else 6000.0
    end = min(min_lead + window, deadline if deadline is not None else math.inf)
    if end <= min_lead:
        return None

    def evaluate(t):
        r, v = propagate(r0, v0, mu, t)
        best = None
        for v_new, exact in target_velocities(r, mu, ra, rp, inclination_deg):
            dv = add(v_new, v, -1.0)
            cost = norm(dv) + (0.0 if exact else INEXACT_PENALTY)  # one exact burn beats burn + touch-up
            if best is None or cost < best[0]:
                best = (cost, t, r, v, v_new, dv, exact)
        return best

    grid = [min_lead + (end-min_lead)*i/samples for i in range(samples+1)]
    radii = [norm(propagate(r0, v0, mu, t)[0]) for t in grid]
    # A narrow (e.g. circular) target is only reachable where the orbit crosses its
    # radius: add those crossings, found by bisection, to the candidate times.
    for rc in {ra, rp}:
        for i in range(samples):
            a, b = radii[i]-rc, radii[i+1]-rc
            if a == 0.0 or a*b < 0.0:
                lo, hi = grid[i], grid[i+1]
                for _ in range(60):
                    mid = 0.5*(lo+hi)
                    if (norm(propagate(r0, v0, mu, mid)[0])-rc)*a > 0.0:
                        lo = mid
                    else:
                        hi = mid
                grid.append(0.5*(lo+hi))
    grid.sort()
    samples = len(grid)-1
    results = [evaluate(t) for t in grid]
    feasible = [(res[0], i) for i, res in enumerate(results) if res is not None]
    if not feasible:
        return None
    _, index = min(feasible)
    lo = grid[max(0, index-1)]
    hi = grid[min(samples, index+1)]
    best = results[index]
    for _ in range(40):  # ternary refinement of a unimodal local minimum
        m1, m2 = lo + (hi-lo)/3.0, hi - (hi-lo)/3.0
        a, b = evaluate(m1), evaluate(m2)
        ca = a[0] if a else math.inf
        cb = b[0] if b else math.inf
        if a and ca < best[0]:
            best = a
        if b and cb < best[0]:
            best = b
        if ca <= cb:
            hi = m2
        else:
            lo = m1
    cost, t, r, v, v_new, dv, exact = best
    achieved = elements(r, v_new, mu)
    return {'t': t, 'r': r, 'v': v, 'v_new': v_new, 'dv_vec': dv, 'dv': norm(dv),
            'exact': exact, 'components': node_components(r, v, dv),
            'apoapsis_alt': achieved['ra'] - body_radius,
            'periapsis_alt': achieved['rp'] - body_radius,
            'inclination': achieved['inc']}


def apply_dv(r, v, dv):
    return r, add(v, dv)


# --- transfers and phasing ----------------------------------------------------------

def circular_speed(mu, radius):
    return math.sqrt(mu/radius)


def period(mu, sma):
    return TAU*math.sqrt(sma**3/mu)


def synchronous_radius(mu, rotation_period):
    """Radius of the (keo/geo)stationary orbit."""
    return (mu*(rotation_period/TAU)**2)**(1.0/3.0)


def hohmann(mu, r1, r2):
    """(dv1, dv2, transfer time) of a coplanar Hohmann transfer r1 -> r2 (signed)."""
    a = 0.5*(r1+r2)
    v1 = math.sqrt(mu/r1)
    v2 = math.sqrt(mu/r2)
    dv1 = math.sqrt(mu/r1)*(math.sqrt(2.0*r2/(r1+r2))-1.0)
    dv2 = math.sqrt(mu/r2)*(1.0-math.sqrt(2.0*r1/(r1+r2)))
    del v1, v2
    return dv1, dv2, math.pi*math.sqrt(a**3/mu)


def hohmann_phase_angle(mu, r1, r2):
    """Angle (rad) the target must lead the chaser at the first burn, +ve = ahead."""
    _, _, tof = hohmann(mu, r1, r2)
    n_target = math.sqrt(mu/r2**3)
    return (math.pi - n_target*tof) % TAU


def wait_for_phase(mu, r1, r2, phase_now):
    """Seconds until the chaser (r1) is positioned for a Hohmann to the target (r2).

    ``phase_now`` is the angle (rad) by which the target currently leads the chaser.
    Works for either orbit being higher; the wait is bounded by one synodic period.
    """
    n1 = math.sqrt(mu/r1**3)
    n2 = math.sqrt(mu/r2**3)
    if abs(n1-n2) < 1e-12:
        raise ValueError('Chaser and target are co-orbital; no Hohmann phasing exists')
    required = hohmann_phase_angle(mu, r1, r2)
    # the lead angle changes at (n2 - n1) rad/s
    delta = (required - phase_now) / (n2-n1)
    synodic = TAU/abs(n1-n2)
    return delta % synodic


def closest_approach(r_c, v_c, r_t, v_t, mu, t_start, t_end, samples=720):
    """(time, distance) of the closest approach of two Kepler orbits in a window."""
    def separation(t):
        rc, _ = propagate(r_c, v_c, mu, t)
        rt, _ = propagate(r_t, v_t, mu, t)
        return norm(add(rc, rt, -1.0))
    best_t, best_d = t_start, separation(t_start)
    step = (t_end-t_start)/samples
    for i in range(1, samples+1):
        t = t_start + i*step
        d = separation(t)
        if d < best_d:
            best_t, best_d = t, d
    lo, hi = max(t_start, best_t-step), min(t_end, best_t+step)
    for _ in range(40):
        m1, m2 = lo + (hi-lo)/3.0, hi - (hi-lo)/3.0
        if separation(m1) < separation(m2):
            hi = m2
        else:
            lo = m1
    t = 0.5*(lo+hi)
    return t, separation(t)
