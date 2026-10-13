import math
from types import SimpleNamespace

import pytest

from ksl.ascent import orbit_math as om
from ksl.ascent import orbit_trim

MU, R = 3.5316e12, 600000.0


def state(ra_alt, rp_alt, inc_deg, nu_deg=0.0):
    ra, rp = ra_alt+R, rp_alt+R
    a, e = (ra+rp)/2, (ra-rp)/(ra+rp)
    p, nu = a*(1-e*e), math.radians(nu_deg)
    r = p/(1+e*math.cos(nu))
    pos = (r*math.cos(nu), r*math.sin(nu), 0.0)
    vel = (-math.sqrt(MU/p)*math.sin(nu), math.sqrt(MU/p)*(e+math.cos(nu)), 0.0)
    i = math.radians(inc_deg)
    tilt = lambda v: (v[0], v[1]*math.cos(i), v[1]*math.sin(i))
    return tilt(pos), tilt(vel)


def test_propagate_returns_to_start_after_one_period():
    r, v = state(85000, 55000, 10, 40)
    period = om.elements(r, v, MU)['period']
    r2, v2 = om.propagate(r, v, MU, period)
    assert om.norm(om.add(r2, r, -1)) < 1e-3 and om.norm(om.add(v2, v, -1)) < 1e-6


@pytest.mark.parametrize('ra,rp,inc,target', [
    (85000, 55000, 10.0, (85000, 85000, 10.0)),      # post-MECO circularization trim
    (86000, 56000, 10.0, (85000, 85000, 8.0)),       # plane + apsis in one burn
    (85000, 85000, 0.1, (2863000, 85000, 0.1)),      # GTO injection
])
def test_one_burn_reaches_every_target_element_exactly(ra, rp, inc, target):
    r, v = state(ra, rp, inc, 120)
    solution = om.solve_one_burn(r, v, MU, target[0], target[1], target[2], R, min_lead=30)
    assert solution is not None
    assert solution['apoapsis_alt'] == pytest.approx(target[0], abs=5.0)
    assert solution['periapsis_alt'] == pytest.approx(target[1], abs=5.0)
    assert solution['inclination'] == pytest.approx(target[2], abs=0.001)


def test_apoapsis_below_a_circular_target_is_flagged_inexact_but_close():
    # The orbit never reaches the target radius, so one burn cannot be exact; the
    # solver must say so and still land within ~1.5 km so a short 2nd burn finishes.
    r, v = state(84000, 56000, 0.4, 120)
    solution = om.solve_one_burn(r, v, MU, 85000, 85000, 0.0, R, min_lead=30)
    assert not solution['exact']
    assert abs(solution['apoapsis_alt']-85000) < 1500 and abs(solution['periapsis_alt']-85000) < 1500


def test_hohmann_phasing_wait_is_within_one_synodic_period():
    r1, r2 = R+85000, R+2863000
    wait = om.wait_for_phase(MU, r1, r2, phase_now=1.0)
    n1, n2 = math.sqrt(MU/r1**3), math.sqrt(MU/r2**3)
    assert 0 <= wait < om.TAU/abs(n1-n2)
    lead = (1.0 + (n2-n1)*wait) % om.TAU
    assert lead == pytest.approx(om.hohmann_phase_angle(MU, r1, r2), abs=1e-6)


# --- plan_exact against a fake kRPC vessel (left-handed node frame, flipped normal) ---

class FakeNode:
    def __init__(self, vessel, ut, prograde, normal, radial):
        self.vessel, self.ut = vessel, ut
        self.prograde, self.normal, self.radial = prograde, normal, radial

    def _frame(self):
        r, v = om.propagate(self.vessel.r, self.vessel.v, MU, self.ut-self.vessel.ut)
        p, n, rad = om.local_frame(r, v)
        return r, v, p, om.scale(n, -1.0), rad  # kRPC's normal points the other way here

    def burn_vector(self, frame):
        _, _, p, n, rad = self._frame()
        dv = om.add(om.add(om.scale(p, self.prograde), n, self.normal), rad, self.radial)
        return om.to_krpc(dv)

    @property
    def delta_v(self):
        return om.norm(self.burn_vector(None))

    @property
    def orbit(self):
        r, v, p, n, rad = self._frame()
        dv = om.add(om.add(om.scale(p, self.prograde), n, self.normal), rad, self.radial)
        return self.vessel.orbit_of(r, om.add(v, dv))

    def remove(self):
        self.vessel.nodes.remove(self)


def make_vessel(ra, rp, inc, nu):
    r, v = state(ra, rp, inc, nu)
    vessel = SimpleNamespace(r=r, v=v, ut=0.0, nodes=[])

    def orbit_of(rr, vv):
        el = om.elements(rr, vv, MU)
        return SimpleNamespace(apoapsis_altitude=el['ra']-R, periapsis_altitude=el['rp']-R,
                               inclination=math.radians(el['inc']), time_to_periapsis=1e9,
                               body=body)
    body = SimpleNamespace(non_rotating_reference_frame='nrf', gravitational_parameter=MU,
                           equatorial_radius=R)
    vessel.orbit_of = orbit_of
    vessel.orbit = orbit_of(r, v)
    vessel.position = lambda frame: om.to_krpc(vessel.r)
    vessel.velocity = lambda frame: om.to_krpc(vessel.v)

    def add_node(ut, prograde=0.0, normal=0.0, radial=0.0):
        node = FakeNode(vessel, ut, prograde, normal, radial)
        vessel.nodes.append(node)
        return node
    vessel.control = SimpleNamespace(add_node=add_node, nodes=vessel.nodes)
    return vessel


def test_plan_exact_finishes_in_one_burn_whatever_the_node_frame_signs():
    vessel = make_vessel(85000, 55000, 10, 120)
    sc = SimpleNamespace(ut=0.0)
    args = SimpleNamespace(apoapsis=85000.0, periapsis=85000.0, inclination=10.0)
    nodes, predicted = orbit_trim.plan_exact(sc, vessel, args, min_lead=60.0)
    assert len(nodes) == 1
    assert predicted.apoapsis_altitude == pytest.approx(85000, abs=5)
    assert predicted.periapsis_altitude == pytest.approx(85000, abs=5)
    assert math.degrees(predicted.inclination) == pytest.approx(10.0, abs=0.001)
