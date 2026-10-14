"""Receding-horizon point-mass guidance, independent of kRPC and craft identity.

The prediction includes gravity, spherical curvature, propellant mass flow and
measured drag. Each update starts from telemetry; no precomputed time script is
executed. The short horizon optimizes a bounded pitch command around a velocity-based gravity turn, with an apoapsis boundary constraint.
"""
from dataclasses import dataclass, replace
import math
from ..peg import PoweredExplicitGuidance, orbit_state_at


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


@dataclass(frozen=True)
class State:
    altitude: float
    radial: float
    horizontal: float
    mass: float
    thrust: float
    isp: float
    q: float = 0.
    drag: float = 0.  # measured specific drag, m/s²
    fixed_thrust: float = 0.  # solid/unthrottleable contribution
    surface_horizontal: float = -1.  # -1 means nonrotating offline plant
    lift_radial: float = 0.
    lift_horizontal: float = 0.
    mass_flow: float = -1.  # full-throttle kg/s; -1 uses the scalar Isp fallback
    fixed_mass_flow: float = 0.  # unthrottleable kg/s


@dataclass(frozen=True)
class Target:
    radius: float
    mu: float
    atmosphere: float
    apoapsis: float
    periapsis: float
    inclination: float = 0.
    altitude_tolerance: float = 1000.
    inclination_tolerance: float = .2

    @property
    def insertion_speed(self):
        rp, ra = self.radius + self.periapsis, self.radius + self.apoapsis
        return math.sqrt(2*self.mu*ra/(rp*(ra+rp)))


@dataclass(frozen=True)
class Command:
    pitch: float
    throttle: float
    predicted_apoapsis: float
    horizon: float
    status: str = 'powered'


def apsides(s, target):
    r = target.radius + s.altitude
    h = r*s.horizontal
    energy = .5*(s.radial*s.radial+s.horizontal*s.horizontal)-target.mu/r
    if energy >= 0:
        return math.inf, -target.radius
    a = -target.mu/(2*energy)
    e = math.sqrt(max(0., 1-h*h/(target.mu*a)))
    return a*(1+e)-target.radius, a*(1-e)-target.radius


def step(s, target, pitch, throttle, dt):
    """Midpoint integration of planar equations, with a measured local drag model."""
    p = math.radians(pitch)
    force = s.fixed_thrust+(s.thrust-s.fixed_thrust)*throttle
    flow = (s.fixed_mass_flow+(s.mass_flow-s.fixed_mass_flow)*throttle
            if s.mass_flow >= 0 else force/(max(s.isp, 1.)*9.80665))
    def derivative(h, vr, vt, mass):
        r = target.radius+h
        speed = max(1., math.hypot(vr, vt))
        density_ratio = math.exp(clamp((s.altitude-h)/6000., -20., 20.))
        drag = s.drag*density_ratio
        return vr, force/max(mass, 1.)*math.sin(p)-target.mu/r**2+vt*vt/r-drag*vr/speed+s.lift_radial*density_ratio, \
            force/max(mass, 1.)*math.cos(p)-vr*vt/r-drag*vt/speed+s.lift_horizontal*density_ratio
    d = derivative(s.altitude, s.radial, s.horizontal, s.mass)
    mid = derivative(s.altitude+d[0]*dt/2, s.radial+d[1]*dt/2,
                     s.horizontal+d[2]*dt/2, s.mass-flow*dt/2)
    return replace(s, altitude=s.altitude+mid[0]*dt, radial=s.radial+mid[1]*dt,
                   horizontal=s.horizontal+mid[2]*dt, mass=max(1., s.mass-flow*dt))



class Planner:
    peg_altitude, peg_q = .3, .05
    def __init__(self, target, max_q=25000., max_g=3., slew=2., peg=True):
        self.target, self.max_q, self.max_g, self.slew = target, max_q, max_g, slew
        self.pitch = 90.
        self.clock = 0.
        # Closed-loop insertion guidance takes over once the air is thin, so the
        # climb is planned to end at the target state instead of overshooting.
        self.peg = PoweredExplicitGuidance(target.mu) if peg else None
        self.peg_active = False
        self.peg_plan = None

    def peg_ready(self, s):
        t = self.target
        return (self.peg is not None and s.altitude > self.peg_altitude*t.atmosphere and
                s.q < self.peg_q*self.max_q and s.radial > 0. and s.fixed_thrust < 1.)

    def plan_insertion(self, s, accel, ve, solve, constant):
        """Pick the burnout point (radius, ellipse periapsis) of least total delta-v.

        Each candidate is a full PEG solution; it is feasible only if its steering
        stays within bounds. Cost is the PEG burn plus circularization at apoapsis,
        so a high-thrust stage cuts off low and circularizes rather than lofting.
        """
        t = self.target
        ra = t.radius+t.apoapsis
        # Vacuum PEG: keep burnout out of air where drag still matters.
        low = min(t.apoapsis, max(s.altitude, .75*t.atmosphere)+2000.)
        best = None
        for i in range(8):
            r_t = t.radius+low+(t.apoapsis-low)*i/7
            for k in range(9):
                rp = t.radius+t.periapsis-(t.periapsis+.3*t.radius)*k/8
                if rp > r_t:
                    continue
                trial = PoweredExplicitGuidance(t.mu)
                solve((r_t, rp), trial)
                if not trial.converged or trial.tgo <= 1.:
                    continue
                T = trial.tgo
                burn = accel*T if constant else -ve*math.log(1-T*accel/ve)
                v_ap = math.sqrt(2*t.mu*rp/(ra*(ra+rp)))
                rp_goal = t.radius+t.periapsis
                v_goal = math.sqrt(2*t.mu*rp_goal/(ra*(ra+rp_goal)))
                cost = burn+abs(v_goal-v_ap)
                if best is None or cost < best[0]:
                    best = cost, (r_t, rp), trial
        return (best[1], best[2]) if best else (None, self.peg)

    def command(self, s, dt=1.):
        if not all(math.isfinite(x) for x in vars(s).values()) or s.mass <= 0 or s.thrust < 0:
            raise ValueError('Invalid flight state')
        t = self.target
        self.clock += dt
        accel = s.thrust/s.mass
        throttle = clamp((self.max_g*9.80665*s.mass-s.fixed_thrust)/max(s.thrust-s.fixed_thrust, .01), 0., 1.)
        if s.q > self.max_q:
            throttle *= math.sqrt(self.max_q/s.q)
        aero_horizontal = s.horizontal if s.surface_horizontal < 0 else s.surface_horizontal
        speed = math.hypot(s.radial, aero_horizontal)
        # Establish a gravity turn based on velocity and aerodynamic load.
        launch = clamp(90.-max(0., speed-35.)*.08, 0., 90.)
        ap_now, _ = apsides(s, t)
        desired = launch
        if ap_now > t.apoapsis*.8:
            # Predictive apoapsis feedback while building transverse energy.
            desired = clamp(launch-(ap_now-t.apoapsis*.8)/1500., -60., launch)
        prograde = math.degrees(math.atan2(s.radial, max(.1, aero_horizontal)))
        alpha = clamp(50000./max(s.q, 1.), 1.5, 35.)
        lower, upper = -75., 90.
        if s.q > 100.:
            lower, upper = max(lower, prograde-alpha), min(upper, prograde+alpha)
        lower = max(lower, self.pitch-self.slew*dt)
        upper = min(upper, self.pitch+self.slew*dt)
        if lower > upper:  # slew wins when a sudden gust moves prograde
            lower = upper = clamp(prograde, self.pitch-self.slew*dt, self.pitch+self.slew*dt)
        if self.peg_active or self.peg_ready(s):
            force = s.fixed_thrust+(s.thrust-s.fixed_thrust)*throttle
            flow = (s.fixed_mass_flow+(s.mass_flow-s.fixed_mass_flow)*throttle
                    if s.mass_flow >= 0 else force/(max(s.isp, 1.)*9.80665))
            r = t.radius+s.altitude
            ra = t.radius+t.apoapsis
            ve = force/max(flow, 1e-6)
            def solve(plan, guide):
                r_t, rp = plan
                vr_t, vh_t = orbit_state_at(t.mu, ra, rp, r_t)
                return guide.update(self.clock, r, s.radial, s.horizontal, force/s.mass,
                                    ve, r_t, vr_t, vh_t, constant_accel=throttle < .999)
            # Re-plan only when the locked plan has become clearly unflyable.
            # Re-plan only when the locked plan has become clearly unflyable,
            # never in the final seconds where the steering is frozen by design.
            ending = self.peg_plan is not None and (self.peg.tgo or 0.) < 2*self.peg.freeze_tgo
            if not ending and (self.peg_plan is None or not (
                    math.isfinite(self.peg.tgo or math.nan) and self.peg.peak < 1.2)):
                self.peg_plan, self.peg = self.plan_insertion(s, force/s.mass, ve, solve,
                                                              throttle < .999)
            peg_pitch, tgo = (solve(self.peg_plan, self.peg) if self.peg_plan is not None
                              else (None, None))
            ap_now = apsides(s, t)[0]
            if peg_pitch is not None and tgo <= max(.2, 1.5*dt) and ap_now < t.apoapsis-t.altitude_tolerance:
                # Burnout without the apoapsis: drop the plan and let the energy
                # guidance below finish the climb.
                self.peg_plan, peg_pitch = None, None
                self.peg = PoweredExplicitGuidance(t.mu)
            self.peg_active = peg_pitch is not None
            if self.peg_active:
                self.pitch = clamp(peg_pitch, lower, upper)
                status = 'cutoff' if tgo <= max(.2, 1.5*dt) else 'peg'
                return Command(self.pitch, throttle, apsides(s, t)[0], tgo, status)
        nominal = clamp(desired, lower, upper)
        if s.q > self.max_q*.6 and speed > 1.:
            near = step(s, t, nominal, throttle, 2.)
            surface_next = aero_horizontal+(near.horizontal-s.horizontal)
            future_q = s.q*(near.radial**2+surface_next**2)/(speed*speed)*math.exp(
                clamp((s.altitude-near.altitude)/5600., -10., 10.))
            if future_q > self.max_q:
                throttle *= math.sqrt(self.max_q/future_q)
        horizon = clamp((t.insertion_speed-s.horizontal)/max(accel, .1)*.12, 3., 15.)
        candidates = sorted(set(clamp(nominal+d, lower, upper) for d in (-6., -3., 0., 3., 6.)))
        best = None
        for pitch in candidates:
            pred = s
            for _ in range(10):
                pred = step(pred, t, pitch, throttle, horizon/10)
            ap, _ = apsides(pred, t)
            # Only penalize excess apoapsis: insufficient energy is expected early.
            excess = max(0., ap-t.apoapsis) if math.isfinite(ap) else 1e7
            score = ((pitch-nominal)/6.)**2 + (excess/10000.)**2
            if pred.altitude < 0:
                score += 1e6
            if best is None or score < best[0]:
                best = score, pitch, ap
        self.pitch = best[1]
        return Command(self.pitch, throttle, best[2], horizon)


def achieved(apoapsis, periapsis, inclination, target):
    return (all(math.isfinite(x) for x in (apoapsis, periapsis, inclination))
            and abs(apoapsis-target.apoapsis) <= target.altitude_tolerance
            and abs(periapsis-target.periapsis) <= target.altitude_tolerance
            and abs(inclination-abs(target.inclination)) <= target.inclination_tolerance)


def coast_apoapsis(state, target):
    """Predict drag loss until atmospheric exit or apex, with no engine impulse."""
    initial = replace(state, fixed_thrust=0.)
    s = initial
    for _ in range(240):
        if s.radial <= 0 or s.altitude >= target.atmosphere:
            break
        # Rebase the measured atmosphere as the integration advances.
        speed_ratio = (s.radial*s.radial+s.horizontal*s.horizontal)/max(1., initial.radial**2+initial.horizontal**2)
        local_drag = initial.drag*math.exp(clamp((initial.altitude-s.altitude)/5600., -30., 20.))*speed_ratio
        density_ratio = math.exp(clamp((initial.altitude-s.altitude)/5600., -30., 20.))
        s = step(replace(s, drag=local_drag, lift_radial=initial.lift_radial*density_ratio,
                         lift_horizontal=initial.lift_horizontal*density_ratio), target, 0., 0., 1.)
    return apsides(s, target)[0]
