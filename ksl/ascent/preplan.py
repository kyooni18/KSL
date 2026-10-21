"""Vehicle-specific prelaunch direct-shooting ascent optimization.

The search minimizes propellant for an executable two-apsis insertion, rather
than choosing the next pitch from altitude alone. All evaluations are local;
this module has no kRPC or control side effects. The model is planar, with
mass/CoM depletion, fixed boosters, pressure-dependent propulsion, attitude lag,
trim and load limits. It is an engineering prediction, not a 6-DOF certificate.
"""
from dataclasses import asdict, dataclass
import bisect
import json
import math
from functools import cached_property
from pathlib import Path

from .geometry import slew_value
from .guidance import aoa_limit_deg, thrust_correction_blend
from .vehicle import G0, SSME_TRIM_MIN, SSME_TRIM_MAX, post_sep_throttle_cap


def interp(table, x):
    i = max(1, min(len(table)-1, bisect.bisect_right(table, x, key=lambda p: p[0])))
    a, b = table[i-1], table[i]
    f = max(0., min(1., (x-a[0])/(b[0]-a[0])))
    return a[1] + f*(b[1]-a[1])


@dataclass(frozen=True)
class MassPoint:
    mass: float
    position: tuple
    pool: str = 'dry'  # dry, liquid, solid, booster-dry

    def __post_init__(self):
        if (self.pool not in ('dry', 'liquid', 'solid', 'booster-dry') or
                self.mass < 0 or len(self.position) != 3 or
                not all(math.isfinite(x) for x in (self.mass, *self.position))):
            raise ValueError('Invalid ascent mass point')


@dataclass(frozen=True)
class Motor:
    thrust_vac: float
    isp_vac: float
    isp_sea: float
    position: tuple
    direction: tuple
    gimbal: float = 0.  # radians of usable gimbal travel

    def __post_init__(self):
        if (self.thrust_vac <= 0 or self.isp_vac <= 0 or self.isp_sea <= 0 or
                not 0 <= self.gimbal < math.pi/2 or len(self.position) != 3 or
                len(self.direction) != 3 or
                not all(math.isfinite(x) for x in (self.thrust_vac, self.isp_vac,
                    self.isp_sea, self.gimbal, *self.position, *self.direction)) or
                abs(sum(x*x for x in self.direction)-1.) > .01):
            raise ValueError('Invalid ascent motor geometry or performance')

    def thrust(self, pressure):
        return self.thrust_vac * max(.01, 1.-pressure*(1.-self.isp_sea/self.isp_vac))


@dataclass(frozen=True)
class VehicleModel:
    points: tuple
    core: tuple
    boosters: tuple
    atmosphere: tuple  # altitude, density, pressure / sea-level pressure
    radius: float = 600000.
    mu: float = 3.5316e12
    atmosphere_depth: float = 70000.
    pad_altitude: float = 80.
    rotation_speed: float = 174.5  # along-track projection of surface rotation
    inertia_pitch: float = 1e8
    drag_area: float = 20.  # Cd*A: measured calibration or explicitly estimated
    drag_source: str = 'estimated'
    booster_limit: float = .9
    initial_core_limit: float = .38
    max_q: float = 25000.
    max_g: float = 3.
    upper_g: float = 1.4
    pitch_slew: float = .8
    hold_slew: float = .5
    throttle_slew: float = .25
    attitude_lag: float = 2.
    turn_start: float = 500.
    roll_start: float = 30.
    roll_duration: float = 25.
    upright_delay: float = 2.
    upright_duration: float = 25.
    upright_max_q: float = 800.
    separation_max_q: float = math.inf
    post_booster_coast: float = 0.
    prelaunch_core_seconds: float = 0.

    def __post_init__(self):
        positive = (self.radius, self.mu, self.inertia_pitch, self.drag_area,
                    self.max_q, self.max_g, self.upper_g, self.pitch_slew,
                    self.throttle_slew, self.attitude_lag, self.upright_duration)
        if (any(not math.isfinite(x) or x <= 0 for x in positive) or
                self.drag_area > 10000. or
                not 0 < self.booster_limit <= 1 or not 0 <= self.initial_core_limit <= 1 or
                len(self.atmosphere) < 2 or
                any(not all(math.isfinite(x) for x in row) or row[1] < 0 or row[2] < 0
                    for row in self.atmosphere) or
                any(a[0] >= b[0] for a, b in zip(self.atmosphere, self.atmosphere[1:]))):
            raise ValueError('Invalid ascent environment or vehicle limits')

    @property
    def initial_mass(self):
        return sum(p.mass for p in self.points)

    @cached_property
    def pools(self):
        result = {}
        for pool in ('dry', 'liquid', 'solid', 'booster-dry'):
            points = [p for p in self.points if p.pool == pool]
            result[pool] = (sum(p.mass for p in points),
                            tuple(sum(p.mass*p.position[i] for p in points) for i in range(3)),
                            sum(p.mass*(p.position[1]**2+p.position[2]**2) for p in points))
        return result

    @cached_property
    def densities(self):
        return tuple((a, rho) for a, rho, _ in self.atmosphere)

    @cached_property
    def pressures(self):
        return tuple((a, p) for a, _, p in self.atmosphere)

    def mass_state(self, liquid, solid, attached):
        weights = [('liquid', liquid), ('solid', solid), ('booster-dry', float(attached)), ('dry', 1.)]
        fractions = dict(weights)
        mass = 0.
        moment = [0., 0., 0.]
        inertia = self.inertia_pitch
        for pool, (initial_mass, first, second) in self.pools.items():
            fraction = fractions[pool]
            mass += initial_mass*fraction
            for i in range(3):
                moment[i] += first[i]*fraction
            inertia -= second*(1.-fraction)
        com = tuple(v/mass for v in moment)
        inertia -= mass*(com[1]**2+com[2]**2)
        return mass, com, max(inertia, .05*self.inertia_pitch)

    def document(self):
        return asdict(self)

    @classmethod
    def from_document(cls, document):
        d = dict(document)
        d['points'] = tuple(MassPoint(**p) for p in d['points'])
        for name in ('core', 'boosters'):
            d[name] = tuple(Motor(**m) for m in d[name])
        d['atmosphere'] = tuple(tuple(p) for p in d['atmosphere'])
        if d.get('separation_max_q') is None:
            d['separation_max_q'] = math.inf
        return cls(**d)


@dataclass(frozen=True)
class Sample:
    time: float
    altitude: float
    radial: float
    horizontal: float
    mass: float
    pitch: float
    throttle: float
    q: float
    apoapsis: float
    periapsis: float
    com: tuple
    separated: bool


def apsides(radius, mu, altitude, radial, horizontal):
    r = radius+altitude
    energy = .5*(radial*radial+horizontal*horizontal)-mu/r
    if energy >= 0:
        return 1e9, -radius
    a = -mu/(2.*energy)
    e = math.sqrt(max(0., 1.-(r*horizontal)**2/(mu*a)))
    return a*(1.+e)-radius, a*(1.-e)-radius


@dataclass
class Trajectory:
    parameters: list
    samples: list
    target_ap: float
    target_pe: float
    propellant: float
    score: float
    feasible: bool
    violations: dict
    evaluations: int = 0

    def reference(self, elapsed):
        times = [s.time for s in self.samples]
        i = max(1, min(len(times)-1, bisect.bisect_right(times, elapsed)))
        a, b = self.samples[i-1], self.samples[i]
        f = max(0., min(1., (elapsed-a.time)/(b.time-a.time)))
        values = {key: getattr(a, key)+f*(getattr(b, key)-getattr(a, key))
                  for key in ('altitude', 'radial', 'horizontal', 'mass', 'pitch', 'throttle')}
        return values

    def tracking_pitch(self, elapsed, altitude, radial, horizontal):
        ref = self.reference(elapsed)
        # Feed-forward trajectory plus bounded vertical/horizontal velocity feedback.
        correction = ((ref['altitude']-altitude)/3000. +
                      (ref['radial']-radial)/35. -
                      (ref['horizontal']-horizontal)/120.)
        return ref['pitch'] + max(-8., min(8., correction))

    def save(self, path, model):
        document = {'version': 1, 'model': model.document(), 'trajectory': asdict(self)}
        # Infinity is a supported in-memory separation setting, not valid JSON.
        if not math.isfinite(document['model']['separation_max_q']):
            document['model']['separation_max_q'] = None
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(document, indent=2, allow_nan=False)+'\n')


def evaluate(model, target_ap, target_pe, parameters, dt=1., retain=True):
    """Integrate an entire candidate through cutoff (midpoint translation).

    Parameters are burn duration and six pitch knots after the roll. Pitch is
    physically slew/load limited; engine trim is recomputed at the depleted CoM.
    """
    if (len(parameters) != 7 or not math.isfinite(dt) or dt <= 0 or
            not all(math.isfinite(x) for x in parameters) or parameters[0] <= 0):
        raise ValueError('Invalid ascent shooting parameters or integration step')
    duration, *knots = map(float, parameters)
    liquid0 = sum(p.mass for p in model.points if p.pool == 'liquid')
    solid0 = sum(p.mass for p in model.points if p.pool == 'solid')
    liquid, solid = liquid0, solid0
    h, vr, vh = model.pad_altitude, 0., model.rotation_speed
    command, flown, limit = 89., 89., model.initial_core_limit
    roll_start, roll_done, sep_time, upright_start, upright_done = None, None, None, None, None
    attached = True
    peak_q = peak_g = peak_ap = trim_excess = peak_alpha_excess = 0.
    samples = []
    used = 0.
    core_flow = sum(m.thrust_vac/(m.isp_vac*G0) for m in model.core)
    solid_flow = sum(m.thrust_vac/(m.isp_vac*G0) for m in model.boosters)*model.booster_limit
    lead_fuel = core_flow*limit*model.prelaunch_core_seconds
    liquid = max(0., liquid-lead_fuel)
    used += lead_fuel
    # Fixed normalized knot locations: the optimizer controls their pitch values
    # and total duration, not the point-mass physics or load envelope.
    fractions = (0., .08, .20, .38, .60, .80, 1.)
    t = 0.
    failed = 0.
    while t < duration-1e-8:
        step_dt = min(dt, duration-t)
        if attached and solid > 1e-5 and solid_flow > 0:
            step_dt = min(step_dt, solid/solid_flow)
        for event in ((None if roll_start is None or roll_done is not None else roll_start+model.roll_duration),
                      (None if upright_start is None or upright_done is not None else upright_start+model.upright_duration)):
            if event is not None and event > t+1e-7:
                step_dt = min(step_dt, event-t)
        mass, com, inertia = model.mass_state(liquid/liquid0, solid/solid0 if solid0 else 0., attached)
        density = interp(model.densities, h)
        pressure = interp(model.pressures, h)
        surface_vh = vh-model.rotation_speed*(model.radius+h)/(model.radius+model.pad_altitude)
        speed = max(.01, math.hypot(vr, surface_vh))
        q = .5*density*speed*speed
        peak_q = max(peak_q, q)
        if roll_start is None and h-model.pad_altitude >= model.roll_start:
            roll_start = t
        if roll_start is not None and t >= roll_start+model.roll_duration:
            roll_done = roll_start+model.roll_duration
        if attached and solid <= 1e-5 and q <= model.separation_max_q:
            attached, sep_time = False, t
            mass, com, inertia = model.mass_state(liquid/liquid0, 0., False)
        if sep_time is not None and upright_start is None and t-sep_time >= model.upright_delay and q <= model.upright_max_q:
            upright_start = t
        if upright_start is not None and t >= upright_start+model.upright_duration:
            upright_done = upright_start+model.upright_duration
        core_thrusts = [m.thrust(pressure) for m in model.core]
        booster_thrusts = [m.thrust(pressure)*model.booster_limit if solid > 1e-5 else 0.
                          for m in model.boosters]
        def moment(m, thrust):
            return thrust*((m.position[1]-com[1])*m.direction[2] -
                           (m.position[2]-com[2])*m.direction[1])
        core_moment = sum(moment(m, f) for m, f in zip(model.core, core_thrusts))
        booster_moment = sum(moment(m, f) for m, f in zip(model.boosters, booster_thrusts))
        if attached and solid > 1e-5:
            desired_limit = max(SSME_TRIM_MIN, min(SSME_TRIM_MAX,
                -booster_moment/core_moment if abs(core_moment) > 1e4 else model.initial_core_limit))
        elif attached:
            desired_limit = 0.
        else:
            cap = post_sep_throttle_cap(t-sep_time, None if upright_done is None else t-upright_done)
            desired_limit = min(cap, model.upper_g*G0*mass/max(sum(core_thrusts), 1.))
            if t-sep_time < model.post_booster_coast:
                desired_limit = 0.
            if q > model.max_q:
                desired_limit *= math.sqrt(model.max_q/q)
        limit_before = limit
        limit = slew_value(limit, desired_limit, model.throttle_slew, step_dt)
        applied_limit = .5*(limit_before+limit)
        fuel_dt = liquid/max(core_flow*applied_limit, 1e-9)
        step_dt = min(step_dt, fuel_dt)
        if step_dt < 1e-7:
            failed = 1.
            break
        core_forces = [f*limit for f in core_thrusts]
        forces = list(zip(model.core, core_forces))+list(zip(model.boosters, booster_thrusts))
        fx = sum(f*m.direction[2] for m, f in forces)
        fy = sum(f*m.direction[1] for m, f in forces)
        force = math.hypot(fx, fy)
        cant = math.degrees(math.atan2(fx, fy))
        trim = abs(core_moment*limit+booster_moment)
        authority = sum(f*math.hypot(m.position[1]-com[1], m.position[2]-com[2])*math.sin(m.gimbal)
                        for m, f in forces)
        trim_excess = max(trim_excess, max(0., trim-authority)/max(authority, 1e5))
        # Reserve half of gimbal authority for attitude, with measured inertia.
        accel_deg = math.degrees(max(0., authority-trim)/inertia)*.5
        slew = min(model.pitch_slew if attached else (model.hold_slew if upright_done is None else 2.),
                   math.sqrt(max(0., 2.*accel_deg*3.)))
        phase = max(0., (t+step_dt-(roll_done or t))/max(1., duration-(roll_done or t)))
        desired_pitch = interp(tuple(zip(fractions, (89., *knots))), phase)
        if roll_done is None or h < model.turn_start:
            desired_pitch = 89.
        prograde = math.degrees(math.atan2(vr, max(.5, surface_vh)))
        roll_fraction = (max(0., min(1., (t-upright_start)/model.upright_duration))
                         if upright_start is not None else 0.)
        projected_cant = cant*math.cos(math.pi*roll_fraction)
        blend = thrust_correction_blend(t, upright_start, upright_done)
        nose_offset = 0. if attached else -projected_cant*blend
        if q > 100.:
            alpha = aoa_limit_deg(q)
            peak_alpha_excess = max(peak_alpha_excess,
                max(0., abs(flown+nose_offset-prograde)-max(1.5, 50000./q)))
            # Pre-separation commands aim the nose; afterward they aim thrust.
            desired_pitch = max(prograde-alpha-nose_offset,
                                min(prograde+alpha-nose_offset, desired_pitch))
        command_before = command
        command = slew_value(command, desired_pitch, slew, step_dt)
        # Exact first-order attitude response to a linearly slewing command.
        # Using end-of-step attitude/throttle for the entire interval creates
        # an O(dt) trajectory bias large enough to fake a good orbital cutoff.
        rate = (command-command_before)/step_dt
        lag = max(model.attitude_lag, 1e-6)
        decay = flown-command_before+rate*lag
        flown_mid = command_before+rate*step_dt/2-rate*lag+decay*math.exp(-step_dt/(2*lag))
        flown = command-rate*lag+decay*math.exp(-step_dt/lag)
        mid_roll = (max(0., min(1., (t+step_dt/2-upright_start)/model.upright_duration))
                    if upright_start is not None else 0.)
        mid_cant = cant*math.cos(math.pi*mid_roll)
        mid_blend = thrust_correction_blend(t+step_dt/2, upright_start, upright_done)
        thrust_pitch = math.radians(flown_mid+cant if attached else
                                    flown_mid+mid_cant*(1.-mid_blend))
        flow = core_flow*applied_limit + (solid_flow if solid > 1e-5 else 0.)
        def derivative(alt, radial, horizontal, m):
            r = model.radius+alt
            svh = horizontal-model.rotation_speed*r/(model.radius+model.pad_altitude)
            v = max(.01, math.hypot(radial, svh))
            rho = interp(model.densities, alt)
            local_pressure = interp(model.pressures, alt)
            fy_local = sum(engine.thrust(local_pressure)*applied_limit*engine.direction[1] for engine in model.core)
            fz_local = sum(engine.thrust(local_pressure)*applied_limit*engine.direction[2] for engine in model.core)
            if solid > 1e-5:
                fy_local += sum(engine.thrust(local_pressure)*model.booster_limit*engine.direction[1] for engine in model.boosters)
                fz_local += sum(engine.thrust(local_pressure)*model.booster_limit*engine.direction[2] for engine in model.boosters)
            local_force = math.hypot(fy_local, fz_local)
            drag = .5*rho*v*v*model.drag_area
            return (radial,
                    local_force/m*math.sin(thrust_pitch)-model.mu/r**2+horizontal**2/r-drag/m*radial/v,
                    local_force/m*math.cos(thrust_pitch)-radial*horizontal/r-drag/m*svh/v)
        d = derivative(h, vr, vh, mass)
        mid = derivative(h+d[0]*step_dt/2, vr+d[1]*step_dt/2, vh+d[2]*step_dt/2, mass-flow*step_dt/2)
        ap, pe = apsides(model.radius, model.mu, h, vr, vh)
        peak_ap, peak_g = max(peak_ap, ap), max(peak_g, force/mass/G0)
        if retain:
            samples.append(Sample(t, h, vr, vh, mass, command_before, limit_before, q, ap, pe, com, not attached))
        previous_h = h
        h += mid[0]*step_dt
        vr += mid[1]*step_dt
        vh += mid[2]*step_dt
        liquid = max(0., liquid-core_flow*applied_limit*step_dt)
        solid = max(0., solid-(solid_flow*step_dt if solid > 1e-5 else 0.))
        used += flow*step_dt
        if roll_start is None and previous_h < model.pad_altitude+model.roll_start <= h:
            roll_start = t+step_dt*(model.pad_altitude+model.roll_start-previous_h)/(h-previous_h)
        t += step_dt
        if h < model.pad_altitude-5. or h > target_ap+100000.:
            failed = 1.
            break
    mass, com, _ = model.mass_state(liquid/liquid0, solid/solid0 if solid0 else 0., attached)
    ap, pe = apsides(model.radius, model.mu, h, vr, vh)
    violations = {'apoapsis_m': ap-target_ap, 'periapsis_m': pe-target_pe,
                  'q_fraction': max(0., peak_q/model.max_q-1.),
                  'g_fraction': max(0., peak_g/model.max_g-1.),
                  'trim_fraction': trim_excess,
                  'aoa_excess_deg': peak_alpha_excess,
                  'peak_apoapsis_m': max(0., peak_ap-target_ap-2000.),
                  'insertion_below_atmosphere_m': max(0., model.atmosphere_depth+1000.-h),
                  'failed': failed}
    errors = ((ap-target_ap)/1000., (pe-target_pe)/1000.,
              violations['q_fraction']*20., violations['g_fraction']*20., trim_excess*20., peak_alpha_excess,
              violations['peak_apoapsis_m']/1000., violations['insertion_below_atmosphere_m']/1000., failed*1000.)
    score = used/1000. + 100.*sum(min(1e6, e*e) for e in errors)
    feasible = (not failed and abs(ap-target_ap) <= 1000. and abs(pe-target_pe) <= 1000. and
                violations['q_fraction'] <= .02 and violations['g_fraction'] <= .02 and
                trim_excess <= .02 and peak_alpha_excess <= 1. and violations['peak_apoapsis_m'] <= 1000. and
                violations['insertion_below_atmosphere_m'] <= 100.)
    if retain:
        samples.append(Sample(t, h, vr, vh, mass, command, limit, 0., ap, pe, com, not attached))
    return Trajectory(list(map(float, parameters)), samples, target_ap, target_pe, used, score, feasible, violations)


def optimize(model, target_ap, target_pe, max_evaluations=5000, dt=1.):
    """Global candidate search, local refinement, then finer-step acceptance.

    Deterministic differential evolution searches the bounded pitch family;
    simplex refinement reduces propellant and endpoint error. This establishes
    the best candidate found within the searched model/family, not proof of global optimality.
    """
    from scipy.optimize import differential_evolution, minimize
    if (not model.core or not model.boosters or model.initial_mass <= 0 or
            not 0 < target_pe < target_ap or model.pitch_slew <= 0 or
            not any(p.pool == 'liquid' and p.mass > 0 for p in model.points) or
            not any(p.pool == 'solid' and p.mass > 0 for p in model.points)):
        raise ValueError('Invalid staged ascent model or target')
    bounds = [(150., 700.)]+[(-30., 89.)]*6
    best, count = None, 0
    integration_dt = dt
    def objective(x):
        nonlocal best, count
        trial = evaluate(model, target_ap, target_pe, x, integration_dt, retain=False)
        count += 1
        if best is None or trial.score < best.score:
            best = trial
        return trial.score
    differential_evolution(objective, bounds, popsize=7,
                           maxiter=max(1, int(max_evaluations*.7)//49-1),
                           tol=.0001, polish=False, rng=7)
    minimize(objective, best.parameters, method='Nelder-Mead', bounds=bounds,
             options={'maxfev': max(100, int(max_evaluations*.15)), 'xatol': .001, 'fatol': .001})
    for level, integration_dt in enumerate((min(.25, dt/2), min(.125, dt/4), min(.0625, dt/8))):
        seed, best = best.parameters, None
        objective(seed)
        minimize(objective, seed, method='Nelder-Mead', bounds=bounds,
                 options={'maxfev': max(250, int(max_evaluations*(.25 if level == 0 else .08))),
                          'xatol': .0005, 'fatol': .0001})
        # The simplex can leave a small terminal error in a flat, load-limited
        # part of the parameter family. A finite-difference refinement also
        # searches directions that require changing several knots together.
        minimize(objective, best.parameters, method='SLSQP', bounds=bounds,
                 options={'maxiter': 35, 'eps': .005, 'ftol': .0001})
        result = evaluate(model, target_ap, target_pe, best.parameters, integration_dt/2)
        if result.feasible:
            result.evaluations = count
            return result
    result.evaluations = count
    return result


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model', help='JSON model or previously saved trajectory document')
    parser.add_argument('--apoapsis', type=float, default=90000.)
    parser.add_argument('--periapsis', type=float, default=60000.)
    parser.add_argument('--evaluations', type=int, default=5000)
    parser.add_argument('--output', default='ascent-trajectory.json')
    args = parser.parse_args()
    document = json.loads(Path(args.model).read_text())
    model = VehicleModel.from_document(document.get('model', document))
    result = optimize(model, args.apoapsis, args.periapsis, args.evaluations)
    result.save(args.output, model)
    print(json.dumps({'feasible': result.feasible, 'propellant_kg': result.propellant,
                      'evaluations': result.evaluations, 'violations': result.violations}))
    return 0 if result.feasible else 2


if __name__ == '__main__':
    raise SystemExit(main())
