"""Receding-horizon point-mass guidance, independent of kRPC and craft identity.

The prediction includes gravity, spherical curvature, propellant mass flow and
measured drag. Each update starts from telemetry; no precomputed time script is
executed. The short horizon optimizes a bounded pitch command around a velocity-based gravity turn, with an apoapsis boundary constraint.
"""
from dataclasses import dataclass
import math
from ..peg import PoweredExplicitGuidance
from .. import native


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
    return native.intelligent_apsides(s, target)


def step(s, target, pitch, throttle, dt):
    return State(**native.intelligent_step(s, target, pitch, throttle, dt))


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

    def command(self, s, dt=1.):
        if not all(math.isfinite(x) for x in vars(s).values()) or s.mass <= 0 or s.thrust < 0:
            raise ValueError('Invalid flight state')
        return Command(*native.intelligent_command(self, s, self.target, dt))



def achieved(apoapsis, periapsis, inclination, target):
    return (all(math.isfinite(x) for x in (apoapsis, periapsis, inclination))
            and abs(apoapsis-target.apoapsis) <= target.altitude_tolerance
            and abs(periapsis-target.periapsis) <= target.altitude_tolerance
            and abs(inclination-abs(target.inclination)) <= target.inclination_tolerance)


def coast_apoapsis(state, target):
    """Predict drag loss until atmospheric exit or apex, with no engine impulse."""
    return native.intelligent_coast(state, target)
