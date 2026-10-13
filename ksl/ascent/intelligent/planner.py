"""Receding-horizon point-mass guidance, independent of kRPC and craft identity.

The prediction includes gravity, spherical curvature, propellant mass flow and
measured drag. Each update starts from telemetry; no precomputed time script is
executed. The short horizon optimizes a bounded pitch command around a velocity-based gravity turn, with an apoapsis boundary constraint.
"""
from dataclasses import dataclass, replace
import math


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
    def __init__(self, target, max_q=25000., max_g=3., slew=2.):
        self.target, self.max_q, self.max_g, self.slew = target, max_q, max_g, slew
        self.pitch = 90.

    def command(self, s, dt=1.):
        if not all(math.isfinite(x) for x in vars(s).values()) or s.mass <= 0 or s.thrust < 0:
            raise ValueError('Invalid flight state')
        t = self.target
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
