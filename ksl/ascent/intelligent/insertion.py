"""Body-relative insertion planning and a conservative recovery direction."""
import math
from .. import orbit_math as om
from ..orbit_trim import choose_frame_map, from_rh_map, node_for_dv


def plan_node(sc, vessel, target, min_lead=5.):
    """Search finite-safe burn opportunities; retain closest feasible solutions.

    Only generic node-frame calibration is shared with orbit trim. Atmosphere and
    target constraints come from the current body, not a Kerbin altitude constant.
    """
    orbit = vessel.orbit
    frame = orbit.body.non_rotating_reference_frame
    rk, vk = vessel.position(frame), vessel.velocity(frame)
    transform, _ = choose_frame_map(rk, vk, target.mu, math.degrees(orbit.inclination))
    r, v = transform(rk), transform(vk)
    deadline = None
    if orbit.periapsis_altitude < target.atmosphere+1000:
        # Search forward for atmospheric return; never coast through that boundary.
        horizon = min(6000., max(60., orbit.period))
        previous = 0.
        for i in range(1, 121):
            t = horizon*i/120
            rp, vp = om.propagate(r, v, target.mu, t)
            if om.norm(rp)-target.radius < target.atmosphere+500 and om.dot(rp,vp) < 0:
                lo, hi = previous, t
                for _ in range(24):
                    mid = (lo+hi)/2
                    point, _ = om.propagate(r, v, target.mu, mid)
                    if om.norm(point)-target.radius > target.atmosphere+500:
                        lo = mid
                    else:
                        hi = mid
                deadline = max(min_lead+1., lo-10.)
                break
            previous = t
    solution = om.solve_one_burn(r, v, target.mu, target.apoapsis, target.periapsis,
                                 abs(target.inclination), target.radius,
                                 min_lead=min_lead, deadline=deadline)
    if solution is None:
        return None
    node = node_for_dv(vessel, sc.ut+solution['t'], from_rh_map(transform)(solution['dv_vec']), frame)
    return node, solution


def recovery_direction(position, velocity, mu):
    """Build orbital energy with small radial support when a burn search is uncertain."""
    radial = om.unit(position)
    vr = om.dot(velocity, radial)
    tangent = om.unit(om.add(velocity, radial, -vr))
    # Keep a descending recovery slightly above horizontal; avoid a large loft.
    return om.unit(om.add(tangent, radial, max(0., min(.3, -vr/500.))))
