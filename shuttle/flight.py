"""kRPC adapter. Preflight is read-only; --launch explicitly enables control."""
import argparse
import csv
import math
import signal
import time
from .allocation import allocate

ROLL_RESET_GRACE = 60.0
G0 = 9.80665


def descendants(part):
    result = [part]
    for child in part.children:
        result.extend(descendants(child))
    return result


def one(parts, name):
    found = [p for p in parts if p.name == name]
    if len(found) != 1:
        raise ValueError(f'Expected one {name}, found {len(found)}')
    return found[0]


def has_resource(part, resource):
    """Return whether a part carries a resource without assuming a specific part name."""
    try:
        return part.resources.has_resource(resource)
    except AttributeError:
        try:
            return part.resources.max(resource) > 0
        except Exception:
            return False


def resource_total(parts, resource, maximum=False):
    total = 0.0
    for part in parts:
        if part.name == 'sepMotor1' or not has_resource(part, resource):
            continue
        getter = part.resources.max if maximum else part.resources.amount
        total += getter(resource)
    return total


def resource_mass(parts, resources, maximum=False):
    total = 0.0
    for part in parts:
        if part.name == 'sepMotor1':
            continue
        for resource in resources:
            if not has_resource(part, resource):
                continue
            getter = part.resources.max if maximum else part.resources.amount
            total += getter(resource) * part.resources.density(resource)
    return total


def part_engine(part):
    try:
        return part.engine
    except Exception:
        return None


def is_skipper_engine(engine):
    name = getattr(engine.part, 'name', '')
    title = getattr(engine.part, 'title', '')
    return name in ('engineLargeSkipper_v2', 'engineLargeSkipper') or 'Skipper' in title


def meco_ready(apoapsis_altitude, periapsis_altitude, target_apoapsis, minimum_periapsis):
    """Normal MECO requires both the target apoapsis and a near-orbital periapsis."""
    return (apoapsis_altitude >= target_apoapsis and
            periapsis_altitude >= minimum_periapsis)


def roll_for_heading(heading):
    """Map launch heading to body roll; 90 degrees maps to 180 degrees."""
    roll = (2 * heading) % 360
    return roll if roll <= 180 else roll - 360


def direction_for_pitch_heading(pitch, heading):
    """Convert pitch/heading to a direction vector in the surface frame."""
    pitch, heading = math.radians(pitch), math.radians(heading)
    return (math.sin(pitch), math.cos(pitch) * math.cos(heading),
            math.cos(pitch) * math.sin(heading))


def pitch_heading_for_direction(direction):
    """Convert a surface-frame direction to pitch and compass heading."""
    x, y, z = direction
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, x))))
    heading = math.degrees(math.atan2(z, y)) % 360
    return pitch, heading


def surface_up_for_pitch_heading(pitch, heading):
    """Return a unit surface-up vector perpendicular to the launch direction."""
    pitch, heading = math.radians(pitch), math.radians(heading)
    return (math.cos(pitch), -math.sin(pitch)*math.cos(heading),
            -math.sin(pitch)*math.sin(heading))


def ascent_pitch(altitude, turn_start_altitude, turn_end_altitude,
                 initial_pitch=89.0):
    """Fly the gravity turn independently of the delayed body-roll reset."""
    progress = min(1, max(0, (altitude-turn_start_altitude) /
                          (turn_end_altitude-turn_start_altitude)))
    return max(5, initial_pitch*(1-progress))


def attitude_error_exceeded(error, altitude, since_roll_reset=None):
    """Preserve the attitude guard while allowing the commanded half-roll."""
    return (error > 45 and altitude > 1000 and
            (since_roll_reset is None or since_roll_reset >= ROLL_RESET_GRACE))


def flight_roll_target(heading, liftoff, booster_separation_ut, current_ut, reset_delay):
    """Return (target roll, reset due), or (None, False) before liftoff."""
    if not liftoff:
        return None, False
    reset_due = (booster_separation_ut is not None and
                 current_ut - booster_separation_ut >= reset_delay)
    return (0 if reset_due else roll_for_heading(heading)), reset_due


def validate_launch_vessel(vessel, space_center):
    if vessel.name != 'STS':
        raise ValueError(f'Expected the active vessel to be named STS; got {vessel.name!r}')
    if vessel.situation != space_center.VesselSituation.pre_launch:
        raise ValueError('Launch requires an STS on the launch pad')


def set_autopilot_engaged(autopilot, engaged):
    """Support kRPC clients exposing engagement as either methods or a property."""
    method = getattr(autopilot, 'engage' if engaged else 'disengage', None)
    if callable(method):
        method()
    elif hasattr(autopilot, 'engaged'):
        autopilot.engaged = engaged
    else:
        raise AttributeError('AutoPilot has no engagement control')


def release_controls(control, autopilot):
    """Cut thrust and return attitude control to SAS, even if one write fails."""
    try:
        control.throttle = 0
    finally:
        try:
            set_autopilot_engaged(autopilot, False)
        finally:
            control.sas = True


def _terminate_as_exception(signum, _frame):
    raise SystemExit(128 + signum)


class Shuttle:
    CORE_GROUPS = 4

    def __init__(self, conn):
        self.conn = conn
        self.sc = conn.space_center
        self.vessel = self.sc.active_vessel
        parts = self.vessel.parts.all
        self.skiff = one(parts, 'LiquidEngineRE-I2').engine
        self.cheetah = one(parts, 'LiquidEngineLV-T91').engine
        bobcats = [p.engine for p in parts if p.name == 'LiquidEngineLV-TX87']
        if len(bobcats) != 4:
            raise ValueError('Expected four Bobcats')

        # Identify the ET attachment from the known ET liquid engines rather than
        # from tank part names, so replacement booster tanks may use any shape.
        et_candidates = []
        for p in parts:
            if p.name != 'radialDecoupler1-2':
                continue
            subtree = descendants(p)
            if (self.cheetah.part in subtree and
                    all(e.part in subtree for e in bobcats) and
                    self.skiff.part not in subtree):
                et_candidates.append((p, subtree))
        if len(et_candidates) != 1:
            raise ValueError(f'Expected one ET separator, found {len(et_candidates)}')
        self.et_decoupler_part, et_subtree = et_candidates[0]
        self.et_decoupler = self.et_decoupler_part.decoupler

        main_engine_parts = {self.skiff.part, self.cheetah.part, *(e.part for e in bobcats)}
        self.booster_decouplers = []
        self.booster_branch_parts = []
        self.booster_branches = []
        self.booster_kinds = []
        for p in parts:
            if p is self.et_decoupler_part or p.name != 'radialDecoupler1-2' or p not in et_subtree:
                continue
            subtree = descendants(p)
            engines = []
            for part in subtree:
                engine = part_engine(part)
                if (engine is not None and part.name != 'sepMotor1' and
                        part not in main_engine_parts):
                    engines.append(engine)
            if not engines:
                continue
            lf = resource_total(subtree, 'LiquidFuel', maximum=True)
            ox = resource_total(subtree, 'Oxidizer', maximum=True)
            sf = resource_total(subtree, 'SolidFuel', maximum=True)
            if lf > 0 and ox > 0:
                kind = 'liquid'
            elif sf > 0:
                kind = 'solid'
            else:
                raise ValueError(f'Booster branch {p.title!r} has no supported propellant')
            self.booster_decouplers.append(p.decoupler)
            self.booster_branch_parts.append(subtree)
            self.booster_branches.append(engines)
            self.booster_kinds.append(kind)
        if not self.booster_branches:
            raise ValueError('Could not identify any booster branches')

        booster_parts = {part for branch in self.booster_branch_parts for part in branch}
        self.tanks = [p for p in et_subtree
                      if p.name == 'Size3LargeTank' and p not in booster_parts]
        self.shuttle_tank = one(parts, 'mk3FuselageLFO.25')
        if len(self.tanks) != 2:
            raise ValueError(f'Expected two ET tanks, found {len(self.tanks)}')

        frame = self.vessel.reference_frame
        s = self.skiff.part.position(frame)
        c = self.cheetah.part.position(frame)
        length = math.hypot(s[0]-c[0], s[2]-c[2])
        if length < 1:
            raise ValueError('Invalid shuttle/ET geometry')
        self.radial = ((s[0]-c[0])/length, (s[2]-c[2])/length)
        bobcats.sort(key=lambda e: self.project(e.part.position(frame)), reverse=True)
        self.core_groups = [[self.skiff], bobcats[:2], bobcats[2:], [self.cheetah]]
        self.boosters = [e for branch in self.booster_branches for e in branch]
        self.liquid_boosters = [e for branch, kind in zip(self.booster_branches, self.booster_kinds)
                                if kind == 'liquid' for e in branch]
        self.solid_boosters = [e for branch, kind in zip(self.booster_branches, self.booster_kinds)
                               if kind == 'solid' for e in branch]
        if self.liquid_boosters:
            if len(self.booster_branches) != 4 or len(self.liquid_boosters) != 4:
                raise ValueError('Liquid booster configuration requires four one-engine branches')
            if not all(is_skipper_engine(e) for e in self.liquid_boosters):
                raise ValueError('Liquid booster configuration requires four Skipper engines')
        # All four liquid boosters share one throttle command. Their collective moment
        # is still modeled, but position never creates differential booster throttles.
        self.liquid_booster_group = list(self.liquid_boosters)
        self.groups = self.core_groups + ([self.liquid_booster_group]
                                          if self.liquid_booster_group else [])
        booster_part_set = {p for branch in self.booster_branch_parts for p in branch}
        self.sepmotors = [p.engine for p in booster_part_set
                          if p.name == 'sepMotor1' and p.engine is not None]
        self.transfers = {}
        self.separated = False
        self.booster_separation_ut = None
        self.et_attached = True

    def project(self, position):
        return position[0]*self.radial[0] + position[2]*self.radial[1]

    def _solid_booster_state(self):
        if self.separated:
            return 0.0, 0.0
        frame = self.vessel.reference_frame
        thrust_total = 0.0
        moment = 0.0
        for engine in self.solid_boosters:
            thrust = engine.thrust if engine.active else (engine.max_thrust if engine.has_fuel else 0.0)
            thrust_total += thrust
            moment += thrust * self.project(engine.part.position(frame))
        return thrust_total, moment

    @staticmethod
    def _engine_isp(engine):
        # Active engines report current pressure-corrected Isp. Before ignition,
        # kRPC reports zero, so use the Kerbin sea-level value for preflight only.
        isp = engine.specific_impulse
        if isp <= 0:
            isp = getattr(engine, 'kerbin_sea_level_specific_impulse', 0.0)
        if isp <= 0:
            isp = getattr(engine, 'vacuum_specific_impulse', 0.0)
        return max(0.0, isp)

    def _engine_mass_flow(self, engine):
        if not engine.has_fuel:
            return 0.0
        isp = self._engine_isp(engine)
        return engine.max_thrust / (isp * G0) if isp > 0 else 0.0

    def liquid_booster_nominal_burn_time(self):
        if not self.liquid_boosters or self.separated:
            return 0.0
        propellant_mass = sum(resource_mass(parts, ('LiquidFuel', 'Oxidizer'))
                              for parts, kind in zip(self.booster_branch_parts, self.booster_kinds)
                              if kind == 'liquid')
        mass_flow = sum(self._engine_mass_flow(engine) for engine in self.liquid_boosters)
        return propellant_mass / mass_flow if mass_flow > 0 else math.inf

    def group_upper_bounds(self):
        # Liquid Skippers are not stretched to a fixed cutoff time. They run at the
        # largest common throttle allowed by moment balance and the stack G limit.
        return [1.0] * len(self.groups)

    def allocation(self, max_g=None):
        frame = self.vessel.reference_frame
        thrusts, arms, mass_flows = [], [], []
        for group in self.groups:
            values = [(e.max_thrust if e.has_fuel else 0.0,
                       self.project(e.part.position(frame))) for e in group]
            thrust = sum(t for t, _ in values)
            thrusts.append(thrust)
            arms.append(sum(t*a for t, a in values)/thrust if thrust else 0.0)
            mass_flows.append(sum(self._engine_mass_flow(e) for e in group))
        fixed_thrust, fixed_moment = self._solid_booster_state()
        thrust_cap = None
        if max_g is not None:
            thrust_cap = max(0.0, max_g * G0 * self.vessel.mass - fixed_thrust)
        return allocate(thrusts, arms, fixed_moment,
                        thrust_cap=thrust_cap,
                        upper_bounds=self.group_upper_bounds(),
                        mass_flows=mass_flows)

    def commanded_g(self, result):
        fixed_thrust, _ = self._solid_booster_state()
        if self.vessel.mass <= 0:
            return 0.0
        return (result.thrust + fixed_thrust) / (self.vessel.mass * G0)

    def booster_telemetry(self):
        if self.separated:
            return 0.0, 0.0, 0.0, 0.0, 0.0
        thrust = sum(e.thrust if e.active else 0.0 for e in self.boosters)
        lf = sum(resource_total(parts, 'LiquidFuel') for parts in self.booster_branch_parts)
        ox = sum(resource_total(parts, 'Oxidizer') for parts in self.booster_branch_parts)
        sf = sum(resource_total(parts, 'SolidFuel') for parts in self.booster_branch_parts)
        current_mass = sum(resource_mass(parts, ('LiquidFuel', 'Oxidizer', 'SolidFuel'))
                           for parts in self.booster_branch_parts)
        max_mass = sum(resource_mass(parts, ('LiquidFuel', 'Oxidizer', 'SolidFuel'), maximum=True)
                       for parts in self.booster_branch_parts)
        return thrust, lf, ox, sf, current_mass/max_mass if max_mass > 0 else 0.0

    def et_resource_amount(self, resource):
        if not self.et_attached:
            return 0.0
        return sum(t.resources.amount(resource) for t in self.tanks)

    def top_off(self):
        for resource in ('LiquidFuel', 'Oxidizer'):
            previous = self.transfers.get(resource)
            if previous is not None and not previous.complete:
                continue
            amount = self.shuttle_tank.resources.max(resource)-self.shuttle_tank.resources.amount(resource)
            if amount < .1:
                continue
            source = max(self.tanks, key=lambda p: p.resources.amount(resource))
            if source.resources.amount(resource) > .1:
                self.transfers[resource] = self.sc.ResourceTransfer.start(
                    source, self.shuttle_tank, resource, amount)

    def _liquid_booster_depleted(self):
        if not self.liquid_boosters:
            return False
        # Shared throttle should keep the four branches synchronized. Cut all four as
        # soon as one branch is effectively dry to avoid asymmetric thrust.
        for parts, kind in zip(self.booster_branch_parts, self.booster_kinds):
            if kind != 'liquid':
                continue
            lf = resource_total(parts, 'LiquidFuel')
            ox = resource_total(parts, 'Oxidizer')
            if lf < .1 or ox < .1:
                return True
        return any(not engine.has_fuel for engine in self.liquid_boosters)

    def _solid_boosters_depleted(self):
        solid = [(parts, kind) for parts, kind in
                 zip(self.booster_branch_parts, self.booster_kinds) if kind == 'solid']
        return bool(solid) and all(resource_total(parts, 'SolidFuel') < .1
                                   for parts, _ in solid)

    def _separate_boosters(self):
        # The four liquid boosters are one stage: common cutoff, then simultaneous
        # decoupler command. No branch is allowed to detach early.
        for engine in self.liquid_boosters:
            engine.thrust_limit = 0.0
            engine.active = False
        for engine in self.sepmotors:
            engine.active = True
        for decoupler in self.booster_decouplers:
            decoupler.decouple()
        self.separated = True
        self.booster_separation_ut = self.sc.ut
        self.groups = list(self.core_groups)
        print('Boosters shut down/burned out and separated')

    def update(self, max_g=3.0):
        if self.et_attached:
            self.top_off()
        if (not self.separated and
                (self._liquid_booster_depleted() or self._solid_boosters_depleted())):
            self._separate_boosters()
        result = self.allocation(max_g=max_g)
        for group, throttle in zip(self.groups, result.throttles):
            for engine in group:
                engine.thrust_limit = throttle
        return result

    def release_et(self):
        self.vessel.control.throttle = 0
        if any(not transfer.complete for transfer in self.transfers.values()):
            return False
        for group in self.core_groups[1:]:
            for engine in group:
                engine.active = False
        self.et_decoupler.decouple()
        self.et_attached = False
        self.core_groups = [[self.skiff]]
        self.groups = [[self.skiff]]
        self.skiff.thrust_limit = 1
        print('External tank separated')
        return True

def run(args):
    import krpc
    with krpc.connect(name='STS Launch Program', address=args.address,
                      rpc_port=args.rpc_port, stream_port=args.stream_port) as conn:
        v = conn.space_center.active_vessel
        if args.launch:
            validate_launch_vessel(v, conn.space_center)
        shuttle = Shuttle(conn)
        result = shuttle.allocation(max_g=args.max_g)
        booster_names = ', '.join(sorted({e.part.title for e in shuttle.boosters}))
        booster_max_thrust = sum(e.max_thrust for e in shuttle.boosters)
        kinds = ','.join(shuttle.booster_kinds)
        booster_lf = sum(resource_total(parts, 'LiquidFuel', maximum=True)
                         for parts in shuttle.booster_branch_parts)
        booster_ox = sum(resource_total(parts, 'Oxidizer', maximum=True)
                         for parts in shuttle.booster_branch_parts)
        booster_sf = sum(resource_total(parts, 'SolidFuel', maximum=True)
                         for parts in shuttle.booster_branch_parts)
        print(f'{v.name}: {v.mass/1000:.2f} t; engine groups Skiff/Near/Far/Cheetah')
        print(f'Detected boosters: {len(shuttle.booster_branches)} branch(es), '
              f'{len(shuttle.boosters)} engine(s), kinds={kinds}: {booster_names}; '
              f'{booster_max_thrust/1000:.0f} kN max; LF/Ox/SF={booster_lf:.0f}/{booster_ox:.0f}/{booster_sf:.0f}')
        print(f'Preflight throttles: {result.throttles}; residual {result.residual:.0f} N m; '
              f'commanded {shuttle.commanded_g(result):.2f} G / {args.max_g:.2f} G cap')
        if shuttle.liquid_boosters:
            nominal_burn = shuttle.liquid_booster_nominal_burn_time()
            print(f'Liquid-booster full-throttle burn estimate at current pressure: {nominal_burn:.1f} s')
        if not args.launch:
            print('Read-only preflight complete. Use --launch to fly.')
            return
        body = v.orbit.body
        if body.name != 'Kerbin':
            raise ValueError('Default ascent guidance is designed for Kerbin')
        if args.orbit <= body.atmosphere_depth + 10000:
            raise ValueError('Target orbit must be at least 10 km above the atmosphere')
        flight = v.flight(body.reference_frame)
        ap = v.auto_pilot
        control = v.control
        started = conn.space_center.ut
        state = 'ASCENT'
        next_report = 0
        initial_roll = roll_for_heading(args.heading)
        initial_pitch = 89.0
        # Surface-frame x is zenith. Project it into the plane perpendicular to
        # the target direction before using it as the roll reference.
        initial_roll_set = False
        liftoff_ut = None
        roll_reset = False
        roll_reset_ut = None
        turn_start_altitude = args.turn_start
        turn_end_altitude = args.turn_end
        old_sigterm_handler = signal.getsignal(signal.SIGTERM)
        signal.signal(signal.SIGTERM, _terminate_as_exception)
        try:
            control.throttle = 0
            control.sas = False
            ap.reference_frame = v.surface_reference_frame
            ap.target_direction = direction_for_pitch_heading(initial_pitch, args.heading)
            set_autopilot_engaged(ap, True)
            for group, u in zip(shuttle.groups, result.throttles):
                for e in group:
                    e.thrust_limit = u
                    e.active = True
            for e in shuttle.boosters:
                e.active = True
            for clamp in v.parts.launch_clamps:
                clamp.release()
            control.throttle = 1
            with open(args.telemetry, 'w', newline='') as log:
                writer = csv.writer(log)
                writer.writerow(['ut', 'state', 'altitude_m', 'apoapsis_m', 'periapsis_m',
                                 'mass_kg', 'vertical_speed_mps', 'horizontal_speed_mps',
                                 'dynamic_pressure_pa', 'actual_g', 'booster_elapsed_s',
                                 'booster_thrust_N', 'booster_liquid_fuel', 'booster_oxidizer',
                                 'booster_solid_fuel', 'booster_propellant_fraction',
                                 'booster_full_throttle_remaining_s', 'commanded_g',
                                 'et_liquid_fuel', 'et_oxidizer', 'throttles', 'residual_Nm',
                                 'feasible', 'target_roll_deg', 'autopilot_error_deg',
                                 'actual_pitch_deg', 'actual_heading_deg', 'target_pitch_deg'])
                while True:
                    tick = time.monotonic()
                    if conn.space_center.active_vessel != v:
                        raise RuntimeError('Active vessel changed')
                    if conn.space_center.ut-started > args.timeout:
                        raise RuntimeError('Mission timeout')
                    altitude = flight.mean_altitude
                    if not initial_roll_set and v.situation != conn.space_center.VesselSituation.pre_launch:
                        initial_roll_set = True
                        liftoff_ut = conn.space_center.ut
                        ap.target_roll = initial_roll
                        print(f'Liftoff: target roll {initial_roll:.1f} deg for heading {args.heading:.1f} deg')
                    target_roll, reset_due = flight_roll_target(
                        args.heading, initial_roll_set,
                        shuttle.booster_separation_ut, conn.space_center.ut,
                        args.roll_reset_delay)
                    if reset_due and not roll_reset:
                        roll_reset = True
                        roll_reset_ut = conn.space_center.ut
                        print('Roll reset: targeting 0 deg')
                    if state == 'ASCENT':
                        pitch = ascent_pitch(altitude, turn_start_altitude,
                                             turn_end_altitude,
                                             initial_pitch)
                        if (v.orbit.apoapsis_altitude >= args.orbit and
                                v.orbit.periapsis_altitude < args.meco_periapsis):
                            pitch = min(pitch, args.meco_pitch)
                        target_pitch = pitch
                        target_direction = direction_for_pitch_heading(pitch, args.heading)
                        if (shuttle.separated and
                                meco_ready(v.orbit.apoapsis_altitude,
                                           v.orbit.periapsis_altitude,
                                           args.orbit, args.meco_periapsis)):
                            control.throttle = 0
                            state = 'COAST'
                            print(f'MECO: Ap {v.orbit.apoapsis_altitude/1000:.1f} km; '
                                  f'Pe {v.orbit.periapsis_altitude/1000:.1f} km')
                        elif shuttle.separated and any(sum(t.resources.amount(r) for t in shuttle.tanks) < 1 for r in ('LiquidFuel', 'Oxidizer')):
                            if shuttle.release_et():
                                state = 'ORBITER_ASCENT'
                    elif state == 'ORBITER_ASCENT':
                        target_pitch = 5
                        target_direction = direction_for_pitch_heading(5, args.heading)
                        control.throttle = min(1, max(.05, (args.orbit-v.orbit.apoapsis_altitude)/5000))
                        if v.orbit.apoapsis_altitude >= args.orbit:
                            control.throttle = 0
                            state = 'COAST'
                    elif state == 'COAST':
                        target_pitch = None
                        control.throttle = 0
                        if not shuttle.separated:
                            raise RuntimeError('Target reached before booster separation')
                        if shuttle.et_attached and not shuttle.release_et():
                            time.sleep(1/args.hz)
                            continue
                        ap.reference_frame = v.orbital_reference_frame
                        # Orbital-frame x points anti-radially (toward the body).
                        roll_up_reference = (-1, 0, 0)
                        target_direction = (0, 1, 0)
                        radius = body.equatorial_radius+v.orbit.apoapsis_altitude
                        dv = max(0, math.sqrt(body.gravitational_parameter/radius)-math.sqrt(body.gravitational_parameter*(2/radius-1/v.orbit.semi_major_axis)))
                        acceleration = shuttle.skiff.max_thrust/v.mass
                        if acceleration <= 0:
                            raise RuntimeError('Skiff has no thrust for insertion')
                        if v.orbit.time_to_apoapsis <= dv/acceleration/2+1:
                            state = 'INSERTION'
                    elif state == 'INSERTION':
                        target_pitch = None
                        target_direction = (0, 1, 0)
                        control.throttle = min(1, max(.03, (args.orbit-v.orbit.periapsis_altitude)/10000))
                        if v.orbit.periapsis_altitude >= args.orbit-1000:
                            print('Orbital insertion complete')
                            return
                        if flight.mean_altitude < body.atmosphere_depth:
                            raise RuntimeError('Insertion trajectory reentered atmosphere')
                    booster_elapsed = (None if liftoff_ut is None else
                                       max(0.0, conn.space_center.ut-liftoff_ut))
                    if shuttle.et_attached:
                        result = shuttle.update(max_g=args.max_g)
                    else:
                        shuttle.skiff.thrust_limit = 1
                    if target_roll is not None:
                        if state in ('ASCENT', 'ORBITER_ASCENT'):
                            roll_up_reference = surface_up_for_pitch_heading(
                                target_pitch, args.heading)
                        else:
                            # Orbital-frame x points anti-radially (toward the body).
                            roll_up_reference = (-1, 0, 0)
                        ap.set_direction_and_up(target_direction, roll_up_reference, target_roll)
                    else:
                        ap.target_direction = target_direction
                    if conn.space_center.ut >= next_report:
                        actual_pitch, actual_heading = pitch_heading_for_direction(
                            v.direction(v.surface_reference_frame))
                        booster_thrust, booster_lf, booster_ox, booster_sf, booster_fraction = shuttle.booster_telemetry()
                        commanded_g = shuttle.commanded_g(result) if shuttle.et_attached else 0.0
                        booster_full_time = shuttle.liquid_booster_nominal_burn_time()
                        writer.writerow([conn.space_center.ut, state, altitude,
                                         v.orbit.apoapsis_altitude, v.orbit.periapsis_altitude,
                                         v.mass, flight.vertical_speed, flight.horizontal_speed,
                                         flight.dynamic_pressure, flight.g_force,
                                         '' if booster_elapsed is None else booster_elapsed,
                                         booster_thrust, booster_lf, booster_ox, booster_sf,
                                         booster_fraction, booster_full_time, commanded_g,
                                         shuttle.et_resource_amount('LiquidFuel'),
                                         shuttle.et_resource_amount('Oxidizer'),
                                         result.throttles if shuttle.et_attached else (1,),
                                         result.residual if shuttle.et_attached else 0,
                                         result.feasible if shuttle.et_attached else True,
                                         0 if roll_reset else initial_roll if initial_roll_set else '',
                                         ap.error, actual_pitch, actual_heading,
                                         '' if target_pitch is None else target_pitch])
                        log.flush()
                        print(f'{state}: altitude {altitude/1000:.1f} km; Ap {v.orbit.apoapsis_altitude/1000:.1f} km; residual {result.residual:.0f} N m; cmd {commanded_g:.2f} G; actual {flight.g_force:.2f} G')
                        next_report = conn.space_center.ut+2
                    since_roll_reset = (None if roll_reset_ut is None else
                                        conn.space_center.ut-roll_reset_ut)
                    if attitude_error_exceeded(ap.error, altitude, since_roll_reset):
                        raise RuntimeError(f'Attitude error {ap.error:.1f} deg exceeds 45 degrees')
                    if not shuttle.skiff.has_fuel:
                        raise RuntimeError('Skiff out of fuel')
                    time.sleep(max(0, 1/args.hz-(time.monotonic()-tick)))
        finally:
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            try:
                release_controls(control, ap)
            finally:
                signal.signal(signal.SIGTERM, old_sigterm_handler)
            print('Control released; main throttle zero. Any attached solid booster continues until burnout.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--launch', action='store_true')
    parser.add_argument('--address', default='127.0.0.1')
    parser.add_argument('--rpc-port', type=int, default=50000)
    parser.add_argument('--stream-port', type=int, default=50001)
    parser.add_argument('--orbit', type=float, default=100000, help='Target altitude in m')
    parser.add_argument('--heading', type=float, default=90)
    parser.add_argument('--turn-start', type=float, default=1000)
    parser.add_argument('--turn-end', type=float, default=45000)
    parser.add_argument('--meco-periapsis', type=float, default=45000,
                        help='Minimum periapsis altitude required for normal MECO in m')
    parser.add_argument('--meco-pitch', type=float, default=0,
                        help='Surface pitch used to build horizontal energy after target apoapsis')
    parser.add_argument('--max-g', type=float, default=3.0,
                        help='Maximum commanded thrust acceleration in Earth g')
    parser.add_argument('--roll-reset-delay', type=float, default=180,
                        help='Seconds after booster separation before rolling to 0 degrees')
    parser.add_argument('--hz', type=float, default=10)
    parser.add_argument('--timeout', type=float, default=1800)
    parser.add_argument('--telemetry', default='telemetry.csv')
    args = parser.parse_args()
    if (not 0 < args.hz <= 50 or not 0 <= args.turn_start < args.turn_end or
            args.timeout <= 0 or args.roll_reset_delay < 0 or
            not 0 < args.max_g <= 10 or
            not 0 <= args.meco_periapsis < args.orbit or
            not -10 <= args.meco_pitch <= 20):
        parser.error('Invalid loop rate, ascent/MECO profile, G limit, timeout, or roll reset delay')
    run(args)


if __name__ == '__main__':
    main()
