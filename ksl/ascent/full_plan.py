"""Two-stage timed ascent plans from measured state, independent of kRPC.

Search smooth booster steering programs, upper-stage acceleration and insertion
radius, then expose the selected trajectory for feedforward execution. The
profile bounds the search; it is also an explicit fallback if planning fails.
"""
from dataclasses import dataclass
import math

from .geometry import smooth_value, smoothstep01, slew_value
from .guidance import scheduled_turn_pitch, constrain_turn_pitch, PITCH_SLEW_DEG_PER_SECOND
from .peg import InsertionGuidance, apsides
from .trajectory import corridor_pitch
from .plan_timeline import pitch_maneuvers


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


def attached_booster_dry_mass(branches, attached_parts):
    """Ignore launch clamps and other handles detached since discovery."""
    attached = set(attached_parts)
    boosters = {part for branch in branches for part in branch if part in attached}
    if not boosters:
        raise ValueError('Attached booster parts unavailable')
    return sum(part.dry_mass for part in boosters)


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
                    response_seconds=5., attitude_seconds=2., dt=1.,
                    end_bias=None, record=False, pitch_slew=PITCH_SLEW_DEG_PER_SECOND):
    """Propagate a smooth gravity turn to booster separation.

    Drag is calibrated at the seed point and extrapolated using airspeed squared
    and an exponential atmosphere. Thrust and mass flow are held at their seed
    values; caller must treat the result as an approximation.
    """
    values = tuple(vars(state).values()) + (mu, body_radius, target_altitude,
                                             response_seconds, attitude_seconds, dt, pitch_slew)
    if (not all(math.isfinite(x) for x in values) or state.mass <= 0 or
            state.radius <= 0 or mu <= 0 or dt <= 0 or response_seconds <= 0 or
            attitude_seconds <= 0 or pitch_slew <= 0 or state.booster_time < 0 or
            state.booster_dry_mass < 0 or state.booster_dry_mass >= state.mass or
            min(state.core_thrust, state.core_mass_flow, state.booster_thrust,
                state.booster_mass_flow, state.dynamic_pressure, state.drag_acceleration) < 0):
        raise ValueError('Invalid booster forecast state')
    r, vr, vh, mass = state.radius, state.radial_speed, state.horizontal_speed, state.mass
    command, flown, t = state.commanded_pitch, state.flown_pitch, 0.
    rotation = vh - state.surface_horizontal_speed
    seed_speed = math.hypot(vr, state.surface_horizontal_speed)
    trajectory = []
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
        bias_now = bias if end_bias is None else bias + (end_bias-bias)*smoothstep01(
            t/max(state.booster_time, 1e-6))
        raw = constrain_turn_pitch(base+bias_now, r-body_radius, target_altitude, prograde, q)
        if record:
            trajectory.append(dict(time=t, radius=r, radial_speed=vr, horizontal_speed=vh,
                                   mass=mass, raw_pitch=raw, commanded_pitch=command,
                                   flown_pitch=flown, thrust=state.core_thrust+state.booster_thrust,
                                   core_thrust=state.core_thrust, booster_thrust=state.booster_thrust,
                                   mass_flow=state.core_mass_flow+state.booster_mass_flow,
                                   dynamic_pressure=q))
        command = slew_value(command, smooth_value(command, raw, response_seconds, step),
                             pitch_slew, step)
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
    result = dict(radius=r, radial_speed=vr, horizontal_speed=vh,
                mass=mass-state.booster_dry_mass, commanded_pitch=command,
                flown_pitch=flown, time=t)
    if record:
        endpoint = dict(time=t, radius=r, radial_speed=vr, horizontal_speed=vh, mass=mass,
                        raw_pitch=command, commanded_pitch=command, flown_pitch=flown,
                        thrust=state.core_thrust+state.booster_thrust,
                        core_thrust=state.core_thrust, booster_thrust=state.booster_thrust,
                        mass_flow=state.core_mass_flow+state.booster_mass_flow,
                        dynamic_pressure=state.dynamic_pressure if not trajectory else q)
        trajectory.append(endpoint)
        result['trajectory'] = trajectory
    return result


def plan_ascent(state, mu, body_radius, target_apoapsis, target_periapsis,
                min_insertion_altitude, upper_thrust, upper_ve, upper_g,
                biases=(-6., 0., 6.), *, start_ut=0., core_propellant=None,
                acceleration_fractions=(.8, 1.), pitch_slew=PITCH_SLEW_DEG_PER_SECOND):
    """Plan pitch, thrust, mass and the initial handoff through separation/MECO.

    Booster/core thrust is fixed at the measured trim-balanced value during
    booster flight; changing it freely would violate the pitch-moment budget.
    Upper-stage acceleration and insertion radius are decision variables.
    Optional core propellant is the *usable* remaining liquid propellant, not
    SRB fuel or OMS reserves. An absent budget is labeled unverified.
    """
    if (upper_thrust <= 0 or upper_ve <= 0 or upper_g <= 0 or not biases or
            not acceleration_fractions or any(not 0 < x <= 1 for x in acceleration_fractions) or
            not math.isfinite(start_ut)):
        raise ValueError('Invalid upper-stage model')
    core_used = state.core_mass_flow*state.booster_time
    if core_propellant is not None and (not math.isfinite(core_propellant) or
                                       not core_used <= core_propellant <= state.mass):
        raise ValueError('Insufficient core propellant for booster phase')
    guide = InsertionGuidance(mu, body_radius+target_apoapsis,
                              body_radius+target_periapsis,
                              body_radius+min_insertion_altitude,
                              pitch_bounds=(-90., 90.), pitch_response_seconds=5.)
    candidates = []
    for bias in biases:
        for end_bias in biases:
            sep = predict_booster(state, mu, body_radius, target_apoapsis, bias,
                                  end_bias=end_bias, record=True, pitch_slew=pitch_slew)
            dry_mass = (0. if core_propellant is None else
                        max(0., sep['mass']-(core_propellant-core_used)))
            for fraction in acceleration_fractions:
                acceleration = upper_g*fraction
                prediction = guide.plan(
                    sep['radius'], sep['radial_speed'], sep['horizontal_speed'],
                    sep['mass'], upper_thrust, upper_ve, acceleration,
                    sep['commanded_pitch'], hold_left=25., ramp=15., cap_now=.3,
                    candidates=4, flown_pitch=sep['flown_pitch'], attitude_hold=5.,
                    transition_anchor=sep['commanded_pitch'], since_separation=0.,
                    dry_mass=dry_mass)
                candidates.append(dict(bias=bias, end_bias=end_bias,
                                       upper_acceleration=acceleration,
                                       separation=sep, insertion=prediction))
    feasible = [c for c in candidates if
                math.isfinite(c['insertion']['score']) and
                body_radius+target_apoapsis-3000. <= c['insertion']['apoapsis_r'] <= body_radius+target_apoapsis+3000. and
                c['insertion']['periapsis_r'] >= body_radius+target_periapsis-300. and
                c['insertion']['peak_apoapsis_r'] <= body_radius+target_apoapsis+3000.]
    if not feasible:
        raise ValueError('No candidate reaches the target orbit in the current model')
    selected = min(feasible, key=lambda candidate: candidate['insertion']['score'])
    sep, cutoff = selected['separation'], selected['insertion']
    selected.update(start_ut=start_ut, target_apoapsis=target_apoapsis,
                    target_periapsis=target_periapsis, fuel_budget_verified=core_propellant is not None,
                    core_propellant_used=core_used+cutoff['propellant_used'])
    selected['milestones'] = dict(
        separation=dict(ut=start_ut+sep['time'], altitude=sep['radius']-body_radius,
                        mass=sep['mass'], radial_speed=sep['radial_speed'],
                        horizontal_speed=sep['horizontal_speed']),
        meco=dict(ut=start_ut+sep['time']+cutoff['time'], altitude=cutoff['radius']-body_radius,
                  mass=cutoff['mass'], radial_speed=cutoff['radial_speed'],
                  horizontal_speed=cutoff['horizontal_speed'],
                  apoapsis=cutoff['apoapsis_r']-body_radius,
                  periapsis=cutoff['periapsis_r']-body_radius))
    selected['maneuvers'] = dict(
        booster=pitch_maneuvers(sep['trajectory'], start_ut),
        insertion=pitch_maneuvers(cutoff['trajectory'], start_ut+sep['time']))
    return selected
