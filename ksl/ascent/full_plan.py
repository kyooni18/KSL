"""Two-stage point-mass ascent forecasts, independent of kRPC.

The live executive may apply a forecast bias after roll handoff, refreshing the
model from telemetry. Its simplified response and atmosphere remain approximate.
"""
from dataclasses import dataclass
import math

from .geometry import smooth_value
from .guidance import scheduled_turn_pitch
from .peg import InsertionGuidance, apsides
from .trajectory import corridor_pitch


def engine_measurements(engines):
    """Return current thrust/flow and pressure-corrected full thrust/flow."""
    thrust = flow = maximum = maximum_flow = 0.
    for engine in engines:
        if not engine.active:
            continue
        force, available, isp = engine.thrust, engine.max_thrust, engine.specific_impulse
        if not all(math.isfinite(x) for x in (force, available, isp)) or isp <= 0:
            raise ValueError('Invalid engine thrust or specific impulse')
        if force < 0 or available < 0:
            raise ValueError('Negative engine thrust')
        thrust += force
        flow += force/(isp*9.80665)
        maximum += available
        maximum_flow += available/(isp*9.80665)
    return thrust, flow, maximum, maximum_flow


@dataclass(frozen=True)
class BoosterState:
    radius: float
    radial_speed: float
    horizontal_speed: float  # inertial
    surface_horizontal_speed: float
    mass: float
    commanded_pitch: float
    flown_pitch: float
    core_thrust: float
    core_mass_flow: float
    booster_thrust: float
    booster_mass_flow: float
    booster_time: float
    booster_dry_mass: float
    dynamic_pressure: float
    drag_acceleration: float
    lift_radial: float = 0.
    lift_horizontal: float = 0.


def predict_booster(state, mu, body_radius, target_altitude, bias,
                    response_seconds=5., attitude_seconds=2., dt=1.):
    """Propagate a smooth gravity turn to booster separation.

    Drag is calibrated at the seed point and extrapolated using airspeed squared
    and an exponential atmosphere. Thrust and mass flow are held at their seed
    values; caller must treat the result as an approximation.
    """
    values = tuple(vars(state).values()) + (mu, body_radius, target_altitude,
                                             response_seconds, attitude_seconds, dt)
    if (not all(math.isfinite(x) for x in values) or state.mass <= 0 or
            state.radius <= 0 or mu <= 0 or dt <= 0 or response_seconds <= 0 or
            attitude_seconds <= 0 or state.booster_time < 0 or
            state.booster_dry_mass < 0 or state.booster_dry_mass >= state.mass or
            min(state.core_thrust, state.core_mass_flow, state.booster_thrust,
                state.booster_mass_flow, state.dynamic_pressure, state.drag_acceleration) < 0):
        raise ValueError('Invalid booster forecast state')
    r, vr, vh, mass = state.radius, state.radial_speed, state.horizontal_speed, state.mass
    command, flown, t = state.commanded_pitch, state.flown_pitch, 0.
    rotation = vh - state.surface_horizontal_speed
    seed_speed = math.hypot(vr, state.surface_horizontal_speed)
    while t < state.booster_time:
        step = min(dt, state.booster_time-t)
        surface_h = vh-rotation
        speed = math.hypot(vr, surface_h)
        density_ratio = math.exp(max(-20., min(20., (state.radius-r)/6000.)))
        ratio = density_ratio*(speed/max(seed_speed, 1.))**2
        q = state.dynamic_pressure*ratio
        drag = state.drag_acceleration*ratio*state.mass/mass
        base = scheduled_turn_pitch(r-body_radius, target_altitude, vr, surface_h, q, 78.)
        ap, _ = apsides(mu, r, vr, vh)
        base = corridor_pitch(base, math.degrees(math.atan2(max(0., vr),
                                                            max(.5, surface_h))),
                              r-body_radius, ap-body_radius, target_altitude, q)
        prograde = math.degrees(math.atan2(max(0., vr), max(.5, surface_h)))
        alpha = max(1.5, min(20., 50000./max(1., q)))
        raw = max(prograde-alpha, min(prograde+1., base+bias))
        command = smooth_value(command, raw, response_seconds, step)
        flown = smooth_value(flown, command, attitude_seconds, step)
        angle = math.radians(flown)
        thrust = state.core_thrust+state.booster_thrust
        acceleration = thrust/mass
        a_r = (acceleration*math.sin(angle)-mu/r**2+vh**2/r-drag*vr/max(speed, 1.)
               + state.lift_radial*ratio*state.mass/mass)
        a_h = (acceleration*math.cos(angle)-vr*vh/r-drag*surface_h/max(speed, 1.)
               + state.lift_horizontal*ratio*state.mass/mass)
        r += vr*step + .5*a_r*step**2
        vr += a_r*step
        vh += a_h*step
        mass -= (state.core_mass_flow+state.booster_mass_flow)*step
        if mass <= state.booster_dry_mass:
            raise ValueError('Predicted propellant consumption exceeds vehicle mass')
        t += step
    return dict(radius=r, radial_speed=vr, horizontal_speed=vh,
                mass=mass-state.booster_dry_mass, commanded_pitch=command,
                flown_pitch=flown, time=t)


def plan_ascent(state, mu, body_radius, target_apoapsis, target_periapsis,
                min_insertion_altitude, upper_thrust, upper_ve, upper_g,
                biases=(-12., -6., 0., 6., 12.)):
    """Compare booster steering candidates through separation and MECO.

    Results are experimental forecasts, not proof of reachability. The caller
    supplies an upper-stage thrust model. Replanning from telemetry reduces
    drift but does not make this approximate model a guarantee of reachability.
    """
    if upper_thrust <= 0 or upper_ve <= 0 or upper_g <= 0 or not biases:
        raise ValueError('Invalid upper-stage model')
    guide = InsertionGuidance(mu, body_radius+target_apoapsis,
                              body_radius+target_periapsis,
                              body_radius+min_insertion_altitude,
                              pitch_bounds=(-90., 90.), pitch_response_seconds=5.)
    candidates = []
    for bias in biases:
        sep = predict_booster(state, mu, body_radius, target_apoapsis, bias)
        prediction = guide.plan(sep['radius'], sep['radial_speed'], sep['horizontal_speed'],
                                sep['mass'], upper_thrust, upper_ve, upper_g,
                                sep['commanded_pitch'], hold_left=25., ramp=15., cap_now=.3,
                                candidates=4, flown_pitch=sep['flown_pitch'], attitude_hold=5.)
        candidates.append(dict(bias=bias, separation=sep, insertion=prediction))
    feasible = [c for c in candidates if
                math.isfinite(c['insertion']['score']) and
                body_radius+target_apoapsis-3000. <= c['insertion']['apoapsis_r'] <= body_radius+target_apoapsis+3000. and
                c['insertion']['periapsis_r'] >= body_radius+target_periapsis-300. and
                c['insertion']['peak_apoapsis_r'] <= body_radius+target_apoapsis+3000.]
    if not feasible:
        raise ValueError('No candidate reaches the target orbit in the current model')
    return min(feasible, key=lambda candidate: candidate['insertion']['score'])
