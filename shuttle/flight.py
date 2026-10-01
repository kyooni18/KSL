"""kRPC adapter. Preflight is read-only; --launch explicitly enables control."""
import argparse
import csv
import math
import signal
import time
from .allocation import allocate

ROLL_RESET_GRACE = 60.0
G0 = 9.80665
CONTROL_TORQUE_FRACTION = 0.65


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
        try:
            if part.name == 'sepMotor1' or not has_resource(part, resource):
                continue
            getter = part.resources.max if maximum else part.resources.amount
            total += getter(resource)
        except Exception:
            # Detached or destroyed parts leave stale kRPC handles. Treat them as
            # carrying no remaining propellant instead of killing the controller.
            continue
    return total


def resource_mass(parts, resources, maximum=False):
    total = 0.0
    for part in parts:
        try:
            if part.name == 'sepMotor1':
                continue
            for resource in resources:
                if not has_resource(part, resource):
                    continue
                getter = part.resources.max if maximum else part.resources.amount
                total += getter(resource) * part.resources.density(resource)
        except Exception:
            continue
    return total


def engine_has_fuel(engine):
    try:
        return engine.has_fuel
    except Exception:
        return False


def engine_thrust(engine):
    try:
        return engine.thrust if engine.active else 0.0
    except Exception:
        return 0.0


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


def direction_error_deg(actual, target):
    """Angular nose error independent of roll and AutoPilot.error quirks."""
    an = math.sqrt(sum(x*x for x in actual))
    tn = math.sqrt(sum(x*x for x in target))
    if an <= 0 or tn <= 0:
        return 180.0
    dot = sum(a*b for a, b in zip(actual, target)) / (an*tn)
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def surface_up_for_pitch_heading(pitch, heading):
    """Return a unit surface-up vector perpendicular to the launch direction."""
    pitch, heading = math.radians(pitch), math.radians(heading)
    return (math.cos(pitch), -math.sin(pitch)*math.cos(heading),
            -math.sin(pitch)*math.sin(heading))


def wrap_roll(angle):
    wrapped = (angle + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 and angle > 0 else wrapped


def roll_for_roof(direction, up_reference, roof_direction):
    """Return the AutoPilot roll command that preserves the current roof axis."""
    def dot(a, b):
        return sum(x*y for x, y in zip(a, b))

    def cross(a, b):
        return (a[1]*b[2]-a[2]*b[1],
                a[2]*b[0]-a[0]*b[2],
                a[0]*b[1]-a[1]*b[0])

    def normalize(v):
        length = math.sqrt(dot(v, v))
        if length <= 1e-12:
            raise ValueError('Cannot define roll from a zero-length vector')
        return tuple(x/length for x in v)

    axis = normalize(direction)
    up = tuple(u-axis[i]*dot(up_reference, axis) for i, u in enumerate(up_reference))
    roof = tuple(r-axis[i]*dot(roof_direction, axis) for i, r in enumerate(roof_direction))
    up = normalize(up)
    roof = normalize(roof)
    signed = math.degrees(math.atan2(dot(axis, cross(up, roof)), dot(up, roof)))
    # kRPC's positive roll convention is opposite the right-hand signed angle above.
    return wrap_roll(-signed)


def roll_delta(current, target):
    """Shortest signed angular distance from current roll to target roll."""
    return (target-current+180.0) % 360.0 - 180.0


def slew_roll(current, target, max_delta):
    """Move toward target by the shortest roll path, with a per-step rate limit."""
    if max_delta < 0:
        raise ValueError('Roll slew limit must be nonnegative')
    delta = roll_delta(current, target)
    if abs(delta) <= max_delta:
        return target
    return wrap_roll(current + math.copysign(max_delta, delta))


def mission_roll_allowed(liftoff, altitude_gain, roll_start_gain):
    """Start the mission roll only after climbing the requested height above the pad."""
    return liftoff and altitude_gain >= roll_start_gain


def roll_accel_authority_deg_s2(vessel):
    """Conservative roll angular acceleration available from current vessel controls."""
    try:
        positive, negative = vessel.available_torque
        inertia = vessel.moment_of_inertia[1]
    except Exception:
        return 0.0
    if inertia <= 0:
        return 0.0
    torque = min(abs(positive[1]), abs(negative[1]))
    return math.degrees(torque / inertia)


def manual_roll_command(error_deg, rate_deg_s, accel_deg_s2, rate_limit_deg_s,
                        control_limit=.35, response_time=.35):
    """Rate-damped roll command using measured angle/rate and current torque authority."""
    if accel_deg_s2 <= 0 or rate_limit_deg_s <= 0 or control_limit <= 0 or response_time <= 0:
        return 0.0
    desired_rate = max(-rate_limit_deg_s, min(rate_limit_deg_s, error_deg * .5))
    rate_error = desired_rate - rate_deg_s
    command = rate_error / (accel_deg_s2 * response_time)
    return max(-control_limit, min(control_limit, command))


def roll_attitude_settled(roll_error, roll_rate, pitch_error, heading_error,
                          angle_limit=2.0, rate_limit=.75, axis_limit=3.0):
    """Return true when the manual roll and AutoPilot pitch/yaw have all settled."""
    return (abs(roll_error) <= angle_limit and abs(roll_rate) <= rate_limit and
            abs(pitch_error) <= axis_limit and abs(heading_error) <= axis_limit)


def hold_turn_for_roll(roll_active, roll_reset):
    """Keep launch pitch fixed for the entire dedicated roll phase."""
    return roll_active and not roll_reset


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


def g_force_throttle(current_throttle, actual_g, max_g, recovery_step=.02):
    """Limit observed G without disturbing relative engine throttle allocation."""
    current = min(1.0, max(0.0, current_throttle))
    if actual_g > max_g and actual_g > 0:
        return max(0.0, min(current, current * max_g / actual_g))
    if actual_g < max_g - .1:
        return min(1.0, current + recovery_step)
    return current


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


def ensure_autopilot_engaged(control, autopilot):
    """Recover if KSP or another controller unexpectedly drops kRPC AutoPilot."""
    if control.sas:
        control.sas = False
    if getattr(autopilot, 'engaged', True):
        return False
    set_autopilot_engaged(autopilot, True)
    return True


def release_controls(control, autopilot):
    """Cut thrust and return attitude control to SAS, even if one write fails."""
    try:
        control.throttle = 0
        control.roll = 0
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
        self.control_moment_budget = 0.0
        self.cruise_control_moment_budget = 0.0

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
        # largest common throttle allowed by control authority and the stack G limit.
        return [1.0] * len(self.groups)

    def control_moment_authority(self):
        """Return symmetric control torque available about the ET/orbiter moment axis."""
        try:
            positive, negative = self.vessel.available_torque
        except Exception:
            return 0.0
        axis = (-self.radial[1], 0.0, self.radial[0])

        def along(direction):
            total = 0.0
            for i, component in enumerate(direction):
                if component >= 0:
                    total += component * positive[i]
                else:
                    total += component * negative[i]
            return max(0.0, total)

        return min(along(axis), along(tuple(-x for x in axis)))

    def calibrate_control_authority(self, control, timeout=2.0):
        """Measure full-thrust gimbal/control torque while launch clamps still hold."""
        engines = [engine for group in self.groups for engine in group]
        control.throttle = 0
        for engine in engines:
            engine.thrust_limit = 1.0
            engine.active = True
        target_thrust = sum(engine.max_thrust for engine in engines)
        control.throttle = 1.0
        deadline = time.monotonic() + timeout
        actual_thrust = 0.0
        while time.monotonic() < deadline:
            actual_thrust = sum(engine_thrust(engine) for engine in engines)
            if target_thrust <= 0 or actual_thrust >= target_thrust * 0.5:
                break
            time.sleep(.02)
        authority = self.control_moment_authority()
        control.throttle = 0
        self.cruise_control_moment_budget = authority * CONTROL_TORQUE_FRACTION
        self.control_moment_budget = self.cruise_control_moment_budget
        fraction = actual_thrust / target_thrust if target_thrust > 0 else 0.0
        return authority, self.control_moment_budget, fraction

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
                        mass_flows=mass_flows,
                        moment_tolerance=self.control_moment_budget)

    def commanded_g(self, result, main_throttle=1.0):
        fixed_thrust, _ = self._solid_booster_state()
        if self.vessel.mass <= 0:
            return 0.0
        return (result.thrust * main_throttle + fixed_thrust) / (self.vessel.mass * G0)

    def booster_telemetry(self):
        if self.separated:
            return 0.0, 0.0, 0.0, 0.0, 0.0
        thrust = sum(engine_thrust(e) for e in self.boosters)
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
        return any(not engine_has_fuel(engine) for engine in self.liquid_boosters)

    def _solid_boosters_depleted(self):
        solid = [(parts, kind) for parts, kind in
                 zip(self.booster_branch_parts, self.booster_kinds) if kind == 'solid']
        return bool(solid) and all(resource_total(parts, 'SolidFuel') < .1
                                   for parts, _ in solid)

    def _separate_boosters(self):
        # The four liquid boosters are one stage: common cutoff, then simultaneous
        # decoupler command. No branch is allowed to detach early.
        for engine in self.liquid_boosters:
            try:
                engine.thrust_limit = 0.0
                engine.active = False
            except Exception:
                pass
        for engine in self.sepmotors:
            try:
                engine.active = True
            except Exception:
                pass
        for decoupler in self.booster_decouplers:
            try:
                decoupler.decouple()
            except Exception:
                pass
        self.separated = True
        self.booster_separation_ut = self.sc.ut
        self.groups = list(self.core_groups)
        self.control_moment_budget = self.cruise_control_moment_budget
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
        full_thrust = sum(engine.max_thrust for group in shuttle.groups for engine in group)
        full_throttle_g = full_thrust / (v.mass * G0) if v.mass > 0 else 0.0
        print(f'Full-throttle physical thrust acceleration at current pressure: {full_throttle_g:.2f} G')
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
        mission_roll = roll_for_heading(args.heading)
        initial_pitch = 89.0
        # Preserve the craft's actual pad roll, then command the mission roll once.
        launch_roll = None
        mission_roll_commanded = False
        mission_roll_settled = False
        roll_settle_since = None
        initial_roll_set = False
        liftoff_ut = None
        roll_reset = False
        roll_reset_ut = None
        roll_rate = 0.0
        last_roll_angle = None
        last_roll_ut = None
        saved_roll_pid = None
        saved_auto_tune = None
        turn_start_altitude = args.turn_start
        turn_end_altitude = args.turn_end
        old_sigterm_handler = signal.getsignal(signal.SIGTERM)
        signal.signal(signal.SIGTERM, _terminate_as_exception)
        try:
            control.throttle = 0
            control.sas = False
            ap.reference_frame = v.surface_reference_frame
            initial_direction = direction_for_pitch_heading(initial_pitch, args.heading)
            initial_up = surface_up_for_pitch_heading(initial_pitch, args.heading)
            roof_direction = conn.space_center.transform_direction(
                (0, 0, -1), v.reference_frame, v.surface_reference_frame)
            launch_roll = roll_for_roof(initial_direction, initial_up, roof_direction)
            ap.auto_tune = True
            ap.time_to_peak = (1.0, args.roll_time_to_peak, 1.0)
            ap.overshoot = (0.01, args.roll_overshoot, 0.01)
            ap.max_angular_velocity = (1.0, math.radians(args.roll_rate), 1.0)
            ap.roll_attenuation_angle = args.roll_attenuation
            ap.target_smoothing_time = 0.0
            ap.target_direction = initial_direction
            set_autopilot_engaged(ap, True)
            ap.set_direction_and_up(initial_direction, initial_up, launch_roll)
            launch_altitude = flight.mean_altitude
            print(f'Pad roll {launch_roll:.1f} deg; mission roll {mission_roll:.1f} deg starts after '
                  f'+{args.roll_start:.0f} m climb; manual rate cap {args.roll_rate:.1f} deg/s; '
                  'pitch holds 89 deg until settled')
            clamps = list(v.parts.launch_clamps)
            if clamps:
                authority, budget, spool = shuttle.calibrate_control_authority(control)
                result = shuttle.allocation(max_g=args.max_g)
                print(f'Launch control authority: {authority/1000:.0f} kN m; '
                      f'moment budget {budget/1000:.0f} kN m; spool {spool*100:.0f}%')
            for group, u in zip(shuttle.groups, result.throttles):
                for e in group:
                    e.thrust_limit = u
                    e.active = True
            for e in shuttle.boosters:
                e.active = True
            for clamp in clamps:
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
                                 'feasible', 'target_roll_deg', 'nose_error_deg',
                                 'autopilot_error_deg', 'actual_roll_deg', 'roll_error_deg', 'roll_rate_deg_s',
                                 'roll_control', 'pitch_error_deg', 'heading_error_deg',
                                 'roll_control_oscillation',
                                 'actual_pitch_deg', 'actual_heading_deg', 'target_pitch_deg',
                                 'control_moment_budget_Nm'])
                while True:
                    tick = time.monotonic()
                    if conn.space_center.active_vessel != v:
                        raise RuntimeError('Active vessel changed')
                    recovered_autopilot = ensure_autopilot_engaged(control, ap)
                    if recovered_autopilot:
                        print('AutoPilot unexpectedly disengaged; re-engaged')
                    if conn.space_center.ut-started > args.timeout:
                        raise RuntimeError('Mission timeout')
                    altitude = flight.mean_altitude
                    altitude_gain = max(0.0, altitude-launch_altitude)
                    if not initial_roll_set and v.situation != conn.space_center.VesselSituation.pre_launch:
                        initial_roll_set = True
                        liftoff_ut = conn.space_center.ut
                        print(f'Liftoff: holding pad roll {launch_roll:.1f} deg; '
                              f'mission roll {mission_roll:.1f} deg deferred')
                    desired_roll, reset_due = flight_roll_target(
                        args.heading, initial_roll_set,
                        shuttle.booster_separation_ut, conn.space_center.ut,
                        args.roll_reset_delay)
                    roll_allowed = mission_roll_allowed(
                        initial_roll_set, altitude_gain, args.roll_start)
                    if roll_allowed and not reset_due and not mission_roll_commanded:
                        saved_auto_tune = ap.auto_tune
                        saved_roll_pid = ap.roll_pid_gains
                        ap.auto_tune = False
                        ap.roll_pid_gains = (0.0, 0.0, 0.0)
                        ap.target_roll = float('nan')
                        control.roll = 0
                        mission_roll_commanded = True
                        last_roll_angle = None
                        last_roll_ut = None
                        roll_rate = 0.0
                        print(f'Manual mission roll started: {mission_roll:.1f} deg')
                    roll_active = mission_roll_commanded and not mission_roll_settled and not reset_due
                    if recovered_autopilot and roll_active:
                        ap.auto_tune = False
                        ap.roll_pid_gains = (0.0, 0.0, 0.0)
                        ap.target_roll = float('nan')
                    target_roll = (0 if reset_due else
                                   mission_roll if mission_roll_commanded else launch_roll)
                    if reset_due and not roll_reset:
                        roll_reset = True
                        roll_reset_ut = conn.space_center.ut
                        print('Roll reset: targeting 0 deg')
                    roll_requires_authority = False
                    if state == 'ASCENT':
                        roll_requires_authority = hold_turn_for_roll(roll_active, roll_reset)
                        if roll_requires_authority:
                            pitch = initial_pitch
                        else:
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
                        if state == 'ASCENT':
                            control.throttle = g_force_throttle(
                                control.throttle, flight.g_force, args.max_g)
                    else:
                        shuttle.skiff.thrust_limit = 1
                    actual_roll = ''
                    roll_control = 0.0
                    if roll_active and state == 'ASCENT':
                        ap.target_direction = target_direction
                        ap.target_roll = float('nan')
                        surface_frame = v.surface_reference_frame
                        actual_direction = v.direction(surface_frame)
                        roll_up_reference = surface_up_for_pitch_heading(target_pitch, args.heading)
                        roof_direction = conn.space_center.transform_direction(
                            (0, 0, -1), v.reference_frame, surface_frame)
                        actual_roll = roll_for_roof(actual_direction, roll_up_reference, roof_direction)
                        now_ut = conn.space_center.ut
                        if last_roll_angle is not None and last_roll_ut is not None and now_ut > last_roll_ut:
                            instantaneous_rate = roll_delta(last_roll_angle, actual_roll) / (now_ut-last_roll_ut)
                            roll_rate = .7*roll_rate + .3*instantaneous_rate
                        last_roll_angle = actual_roll
                        last_roll_ut = now_ut
                        roll_error_signed = roll_delta(actual_roll, mission_roll)
                        roll_error = abs(roll_error_signed)
                        pitch_error = abs(ap.pitch_error)
                        heading_error = abs(ap.heading_error)
                        roll_control = manual_roll_command(
                            roll_error_signed, roll_rate, roll_accel_authority_deg_s2(v),
                            args.roll_rate, args.roll_control_limit, args.roll_response_time)
                        control.roll = roll_control
                        if roll_attitude_settled(roll_error, roll_rate, pitch_error, heading_error):
                            if roll_settle_since is None:
                                roll_settle_since = now_ut
                            elif now_ut-roll_settle_since >= args.roll_settle_time:
                                mission_roll_settled = True
                                roll_active = False
                                control.roll = 0
                                roll_control = 0.0
                                if saved_roll_pid is not None:
                                    ap.roll_pid_gains = saved_roll_pid
                                if saved_auto_tune is not None:
                                    ap.auto_tune = saved_auto_tune
                                ap.set_direction_and_up(target_direction, roll_up_reference, mission_roll)
                                print('Manual mission roll settled; gravity turn released')
                        else:
                            roll_settle_since = None
                    elif target_roll is not None:
                        control.roll = 0
                        if state in ('ASCENT', 'ORBITER_ASCENT'):
                            roll_up_reference = surface_up_for_pitch_heading(
                                target_pitch, args.heading)
                            surface_frame = v.surface_reference_frame
                            actual_direction = v.direction(surface_frame)
                            roof_direction = conn.space_center.transform_direction(
                                (0, 0, -1), v.reference_frame, surface_frame)
                            actual_roll = roll_for_roof(actual_direction, roll_up_reference, roof_direction)
                        else:
                            # Orbital-frame x points anti-radially (toward the body).
                            roll_up_reference = (-1, 0, 0)
                        ap.set_direction_and_up(target_direction, roll_up_reference, target_roll)
                        try:
                            roll_error = abs(ap.roll_error)
                        except Exception:
                            roll_error = 0.0
                        pitch_error = abs(ap.pitch_error)
                        heading_error = abs(ap.heading_error)
                    else:
                        control.roll = 0
                        ap.target_direction = target_direction
                        roll_error = 0.0
                        pitch_error = abs(ap.pitch_error)
                        heading_error = abs(ap.heading_error)
                    nose_error = direction_error_deg(v.direction(ap.reference_frame), target_direction)
                    if conn.space_center.ut >= next_report:
                        actual_pitch, actual_heading = pitch_heading_for_direction(
                            v.direction(v.surface_reference_frame))
                        booster_thrust, booster_lf, booster_ox, booster_sf, booster_fraction = shuttle.booster_telemetry()
                        commanded_g = (shuttle.commanded_g(result, control.throttle)
                                       if shuttle.et_attached else 0.0)
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
                                         '' if target_roll is None else target_roll,
                                         nose_error, ap.error, actual_roll, roll_error, roll_rate,
                                         roll_control, pitch_error, heading_error,
                                         ap.roll_control_oscillation,
                                         actual_pitch, actual_heading,
                                         '' if target_pitch is None else target_pitch,
                                         shuttle.control_moment_budget])
                        log.flush()
                        print(f'{state}: altitude {altitude/1000:.1f} km; Ap {v.orbit.apoapsis_altitude/1000:.1f} km; residual {result.residual:.0f} N m; budget {shuttle.control_moment_budget:.0f} N m; cmd {commanded_g:.2f} G; actual {flight.g_force:.2f} G; nose {nose_error:.1f} deg; roll {roll_error:.1f} deg; rate {roll_rate:.1f} deg/s; ctl {roll_control:.2f}')
                        next_report = conn.space_center.ut+2
                    since_roll_reset = (None if roll_reset_ut is None else
                                        conn.space_center.ut-roll_reset_ut)
                    if attitude_error_exceeded(nose_error, altitude, since_roll_reset):
                        raise RuntimeError(f'Nose attitude error {nose_error:.1f} deg exceeds 45 degrees')
                    if not shuttle.skiff.has_fuel:
                        raise RuntimeError('Skiff out of fuel')
                    time.sleep(max(0, 1/args.hz-(time.monotonic()-tick)))
        finally:
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            try:
                try:
                    if saved_roll_pid is not None:
                        ap.roll_pid_gains = saved_roll_pid
                    if saved_auto_tune is not None:
                        ap.auto_tune = saved_auto_tune
                finally:
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
    parser.add_argument('--roll-start', type=float, default=30,
                        help='Climb above launch point in m before starting the mission roll')
    parser.add_argument('--roll-rate', type=float, default=6.0,
                        help='AutoPilot roll-axis angular velocity cap in deg/s')
    parser.add_argument('--roll-attenuation', type=float, default=5.0,
                        help='Roll-error angle in deg where AutoPilot starts slowing the roll')
    parser.add_argument('--roll-time-to-peak', type=float, default=2.5,
                        help='AutoTune time-to-peak for the roll rate controller in seconds')
    parser.add_argument('--roll-overshoot', type=float, default=0.001,
                        help='AutoTune target roll overshoot fraction')
    parser.add_argument('--roll-settle-time', type=float, default=2.0,
                        help='Seconds roll/pitch/yaw errors must remain settled')
    parser.add_argument('--roll-control-limit', type=float, default=.35,
                        help='Maximum direct roll control magnitude during the initial mission roll')
    parser.add_argument('--roll-response-time', type=float, default=.35,
                        help='Rate-controller response horizon in seconds for the manual mission roll')
    parser.add_argument('--hz', type=float, default=10)
    parser.add_argument('--timeout', type=float, default=1800)
    parser.add_argument('--telemetry', default='telemetry.csv')
    args = parser.parse_args()
    if (not 0 < args.hz <= 50 or not 0 <= args.turn_start < args.turn_end or
            args.timeout <= 0 or args.roll_reset_delay < 0 or args.roll_start < 0 or
            not 0 < args.roll_rate <= 30 or not 0 < args.roll_attenuation <= 90 or
            args.roll_time_to_peak <= 0 or not 0 <= args.roll_overshoot <= 1 or
            args.roll_settle_time < 0 or not 0 < args.roll_control_limit <= 1 or
            args.roll_response_time <= 0 or not 0 < args.max_g <= 10 or
            not 0 <= args.meco_periapsis < args.orbit or
            not -10 <= args.meco_pitch <= 20):
        parser.error('Invalid loop rate, ascent/MECO profile, G limit, timeout, or roll profile')
    run(args)


if __name__ == '__main__':
    main()
