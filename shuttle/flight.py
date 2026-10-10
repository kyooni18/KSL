"""kRPC adapter. Preflight is read-only; --launch explicitly enables control."""
import argparse
import csv
import math
import signal
import time
from .allocation import Allocation, allocate

ROLL_RESET_GRACE = 60.0
G0 = 9.80665
CONTROL_TORQUE_FRACTION = 0.65

LAUNCH_TARGET_G = 1.08
BOOSTER_MIN_G = 1.20
BOOSTER_PAD_CAP = 1.00
OUTER_SKIPPER_PAD_CAP = 0.45
THROTTLE_SLEW_PER_SECOND = 0.25
PITCH_SLEW_DEG_PER_SECOND = 2.0
MAX_DYNAMIC_PRESSURE_PA = 25000.0


def smoothstep01(value):
    x = min(1.0, max(0.0, value))
    return x*x*(3.0-2.0*x)


def ascent_target_g(altitude, atmosphere_depth, max_g, launch_g=LAUNCH_TARGET_G,
                    minimum_g=0.0):
    """Smooth ascent target with an optional hard lower bound."""
    if atmosphere_depth <= 0:
        target = min(max_g, launch_g)
    else:
        blend = smoothstep01(max(0.0, altitude) / atmosphere_depth)
        target = min(max_g, launch_g + (max_g-launch_g)*blend)
    return min(max_g, max(minimum_g, target))


def slew_value(current, target, rate_per_second, dt):
    if dt <= 0 or rate_per_second <= 0:
        return current
    step = rate_per_second*dt
    return current + max(-step, min(step, target-current))


def slew_vector(current, target, rate_per_second, dt):
    """Move all actuator channels with one interpolation factor."""
    if len(current) != len(target):
        raise ValueError('Current and target vectors must have equal length')
    if dt <= 0 or rate_per_second <= 0:
        return tuple(current)
    delta = [t-c for c, t in zip(current, target)]
    largest = max((abs(x) for x in delta), default=0.0)
    if largest == 0:
        return tuple(target)
    alpha = min(1.0, rate_per_second*dt/largest)
    return tuple(c + alpha*d for c, d in zip(current, delta))


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


def isolate_aero_roll(vessel):
    """Disable only aerodynamic roll channels and return restorable states."""
    states = []
    for part in vessel.parts.all:
        try:
            surface = part.control_surface
            if surface is None:
                continue
            enabled = surface.roll_enabled
            states.append((surface, enabled))
            if enabled:
                surface.roll_enabled = False
        except Exception:
            continue
    return states


def restore_aero_roll(states):
    """Restore control-surface roll-channel states captured by isolate_aero_roll."""
    for surface, enabled in states:
        try:
            surface.roll_enabled = enabled
        except Exception:
            pass


def is_skipper_engine(engine):
    name = getattr(engine.part, 'name', '')
    title = getattr(engine.part, 'title', '')
    return name in ('engineLargeSkipper.v2', 'engineLargeSkipper_v2', 'engineLargeSkipper') or 'Skipper' in title


def is_wolfhound_engine(engine):
    name = getattr(engine.part, 'name', '')
    title = getattr(engine.part, 'title', '')
    return name == 'LiquidEngineRE-J10' or 'Wolfhound' in title


def is_kodiak_engine(engine):
    name = getattr(engine.part, 'name', '')
    title = getattr(engine.part, 'title', '')
    return name == 'LiquidEngineRK-7' or 'Kodiak' in title


def is_cub_engine(engine):
    name = getattr(engine.part, 'name', '')
    title = getattr(engine.part, 'title', '')
    return name == 'LiquidEngineRV-1' or 'Cub' in title


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


def surface_axis_errors(direction, target_pitch, target_heading):
    """Pitch/heading errors from geometry, independent of AutoPilot engagement state."""
    actual_pitch, actual_heading = pitch_heading_for_direction(direction)
    pitch_error = abs(target_pitch-actual_pitch)
    heading_error = abs(wrap_roll(target_heading-actual_heading))
    return pitch_error, heading_error


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


def dot3(a, b):
    return sum(x*y for x, y in zip(a, b))


def cross3(a, b):
    return (a[1]*b[2]-a[2]*b[1],
            a[2]*b[0]-a[0]*b[2],
            a[0]*b[1]-a[1]*b[0])


def normalize3(v):
    length = math.sqrt(dot3(v, v))
    if length <= 1e-12:
        raise ValueError('Cannot normalize a zero-length vector')
    return tuple(x/length for x in v)


def project_perpendicular(v, axis):
    axis = normalize3(axis)
    return tuple(x-axis[i]*dot3(v, axis) for i, x in enumerate(v))


def desired_orbiter_side(pitch, heading, earthward=True):
    """Desired ET->orbiter side axis in the surface frame.

    The orbiter is mounted on vessel -Z. Heads-down means that side axis follows
    local-down projected into the plane normal to the commanded flight direction.
    """
    projected_up = surface_up_for_pitch_heading(pitch, heading)
    return tuple(-x for x in projected_up) if earthward else projected_up


def roll_error_to_side(direction, actual_side, target_side):
    """Signed roll error from the real orbiter-side vector, in control.roll sign."""
    axis = normalize3(direction)
    current = normalize3(project_perpendicular(actual_side, axis))
    target = normalize3(project_perpendicular(target_side, axis))
    signed_rh = math.degrees(math.atan2(dot3(axis, cross3(current, target)),
                                        dot3(current, target)))
    # KSP/kRPC roll-control sign is opposite the right-hand rotation about +forward.
    return wrap_roll(-signed_rh)


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


def body_roll_rate_deg_s(vessel):
    """Physical roll rate in the same sign convention as roll_for_roof/kRPC roll angles."""
    frame = vessel.surface_reference_frame
    direction = vessel.direction(frame)
    omega = vessel.angular_velocity(frame)
    norm = math.sqrt(sum(x*x for x in direction))
    if norm <= 0:
        return 0.0
    # roll_for_roof/kRPC roll angle uses the opposite sign from right-hand angular velocity.
    return -math.degrees(sum(w*d for w, d in zip(omega, direction)) / norm)


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
                        control_limit=1.0, response_time=.20):
    """Rate-damped direct roll with explicit overspeed braking."""
    if accel_deg_s2 <= 0 or rate_limit_deg_s <= 0 or control_limit <= 0 or response_time <= 0:
        return 0.0
    desired_rate = max(-rate_limit_deg_s, min(rate_limit_deg_s, error_deg * .5))
    if abs(rate_deg_s) > rate_limit_deg_s * 1.25:
        return -math.copysign(control_limit, rate_deg_s)
    rate_error = desired_rate - rate_deg_s
    command = rate_error / (accel_deg_s2 * response_time)
    return max(-control_limit, min(control_limit, command))


def roll_attitude_settled(roll_error, roll_rate, nose_error,
                          angle_limit=2.0, rate_limit=.75, nose_limit=3.0):
    return (abs(roll_error) <= angle_limit and abs(roll_rate) <= rate_limit and
            abs(nose_error) <= nose_limit)

def hold_turn_for_roll(roll_active, roll_reset):
    """Keep launch pitch fixed for the entire dedicated roll phase."""
    return roll_active and not roll_reset


def ascent_pitch(altitude, turn_start_altitude, turn_end_altitude,
                 initial_pitch=89.0):
    progress = min(1, max(0, (altitude-turn_start_altitude) /
                          (turn_end_altitude-turn_start_altitude)))
    eased = progress * progress * (3 - 2 * progress)
    return max(5, initial_pitch*(1-eased))


def gravity_turn_pitch(vertical_speed, horizontal_speed, dynamic_pressure,
                       initial_pitch=89.0, minimum_pitch=5.0):
    """Follow surface prograde with a bounded pitch-over bias.

    The bias is larger at low dynamic pressure to establish horizontal velocity,
    then shrinks through max-Q so the stack stays close to prograde.
    """
    vertical = max(0.0, vertical_speed)
    horizontal = max(0.5, horizontal_speed)
    flight_path_pitch = math.degrees(math.atan2(vertical, horizontal))
    q = max(0.0, dynamic_pressure)
    if q <= 5000:
        pitch_under = 7.0
    elif q < 25000:
        pitch_under = 7.0 - 4.0 * (q - 5000.0) / 20000.0
    else:
        pitch_under = 3.0
    return min(initial_pitch, max(minimum_pitch, flight_path_pitch - pitch_under))


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


def dynamic_pressure_throttle(current_throttle, dynamic_pressure, max_dynamic_pressure):
    """Reduce liquid throttle when measured aerodynamic pressure exceeds its cap."""
    if max_dynamic_pressure <= 0 or dynamic_pressure <= max_dynamic_pressure:
        return current_throttle
    ratio = max_dynamic_pressure / max(dynamic_pressure, 1e-9)
    return current_throttle * math.sqrt(ratio)




def earthward_roll_target(liftoff, booster_separation_ut, current_ut, reset_delay):
    """Return whether the orbiter side should face Kerbin, plus delayed-flip state."""
    if not liftoff:
        return None, False
    flipped = (booster_separation_ut is not None and
               current_ut - booster_separation_ut >= reset_delay)
    return (not flipped), flipped


def validate_launch_vessel(vessel, space_center):
    if vessel.name not in ('STS', 'STS-N'):
        raise ValueError(f'Expected the active vessel to be named STS or STS-N; got {vessel.name!r}')
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
        if any(p.name == 'SSME' for p in parts):
            self._init_orbiter_shuttle(parts)
            return
        if any(p.name == 'nfa-atomic-multimode-25-1' for p in parts):
            self._init_nuclear_et(parts)
            return
        self.vehicle_kind = 'skipper-liquid'
        self.skiff = one(parts, 'LiquidEngineRE-I2').engine
        self.wolfhound = one(parts, 'LiquidEngineRE-J10').engine
        et_skippers = [part_engine(p) for p in parts
                       if part_engine(p) is not None and is_skipper_engine(part_engine(p))]
        if len(et_skippers) != 4:
            raise ValueError(f'Expected four ET Skippers, found {len(et_skippers)}')
        self.et_skippers = et_skippers

        # The ET subtree contains Wolfhound + four Skippers and excludes the orbiter Skiff.
        et_candidates = []
        for p in parts:
            if p.name != 'radialDecoupler1-2':
                continue
            subtree = descendants(p)
            if (self.wolfhound.part in subtree and
                    all(e.part in subtree for e in self.et_skippers) and
                    self.skiff.part not in subtree):
                et_candidates.append((p, subtree))
        if len(et_candidates) != 1:
            raise ValueError(f'Expected one ET separator, found {len(et_candidates)}')
        self.et_decoupler_part, et_subtree = et_candidates[0]
        self.et_decoupler = self.et_decoupler_part.decoupler

        main_engine_parts = {self.skiff.part, self.wolfhound.part,
                             *(e.part for e in self.et_skippers)}
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

        if len(self.booster_branches) != 4:
            raise ValueError(f'Expected four booster branches, found {len(self.booster_branches)}')
        if self.booster_kinds != ['liquid'] * 4:
            raise ValueError(f'Expected four liquid booster branches, found {self.booster_kinds}')
        for i, branch in enumerate(self.booster_branches, 1):
            kodiaks = [e for e in branch if is_kodiak_engine(e)]
            cubs = [e for e in branch if is_cub_engine(e)]
            if len(branch) != 5 or len(kodiaks) != 1 or len(cubs) != 4:
                raise ValueError(
                    f'Booster {i} requires one Kodiak and four Cubs; '
                    f'found {len(kodiaks)} Kodiak, {len(cubs)} Cub, {len(branch)} engines')

        booster_parts = {part for branch in self.booster_branch_parts for part in branch}
        self.tanks = [p for p in et_subtree
                      if p not in booster_parts and
                      has_resource(p, 'LiquidFuel') and has_resource(p, 'Oxidizer') and
                      p.resources.max('LiquidFuel') > 0 and p.resources.max('Oxidizer') > 0]
        self.shuttle_tank = one(parts, 'mk3FuselageLFO.25')
        self.transfer_resources = ('LiquidFuel', 'Oxidizer')
        if not self.tanks:
            raise ValueError('Could not identify ET propellant tanks')

        frame = self.vessel.reference_frame
        s = self.skiff.part.position(frame)
        w = self.wolfhound.part.position(frame)
        length = math.hypot(s[0]-w[0], s[2]-w[2])
        if length < 1:
            raise ValueError('Invalid shuttle/ET geometry')
        self.radial = ((s[0]-w[0])/length, (s[2]-w[2])/length)

        skipper_by_arm = sorted(
            self.et_skippers,
            key=lambda e: self.project(e.part.position(frame)),
            reverse=True)
        self.inner_skippers = skipper_by_arm[:2]
        self.outer_skippers = skipper_by_arm[2:]
        self.core_groups = [[self.skiff], [self.wolfhound],
                            list(self.inner_skippers), list(self.outer_skippers)]
        self.boosters = [e for branch in self.booster_branches for e in branch]
        self.liquid_boosters = list(self.boosters)
        self.solid_boosters = []
        self.liquid_booster_group = list(self.liquid_boosters)
        self.groups = self.core_groups + [self.liquid_booster_group]
        booster_part_set = {p for branch in self.booster_branch_parts for p in branch}
        self.sepmotors = [p.engine for p in booster_part_set
                          if p.name == 'sepMotor1' and p.engine is not None]
        self.transfers = {}
        self.separated = False
        self.booster_separation_ut = None
        self.et_attached = True
        self.control_moment_budget = 0.0
        self.cruise_control_moment_budget = 0.0
        self.et_reference_thrust = (
            self._engine_sea_level_max_thrust(self.wolfhound) +
            sum(self._engine_sea_level_max_thrust(e) for e in self.et_skippers))

    def _init_nuclear_et(self, parts):
        """Initialize the STS-N J-N500, twin-Rhino ET, twin-RSRM layout."""
        self.vehicle_kind = 'nuclear-solid'
        engines = [part_engine(p) for p in parts if part_engine(p) is not None]
        self.skiff = one(parts, 'nfa-atomic-multimode-25-1').engine
        self.wolfhound = None
        self.et_skippers = []
        self.et_engines = [e for e in engines
                           if e.part.name == 'Size3AdvancedEngine' and
                           'Rhino' in getattr(e.part, 'title', '')]
        if len(self.et_engines) != 2:
            raise ValueError(f'Expected two ET Rhino engines, found {len(self.et_engines)}')

        et_candidates = []
        for part in parts:
            if part.name != 'STS.ET.decoupler':
                continue
            subtree = descendants(part)
            if (all(e.part in subtree for e in self.et_engines) and
                    self.skiff.part not in subtree):
                et_candidates.append((part, subtree))
        if len(et_candidates) != 1:
            raise ValueError(f'Expected one ET separator around both Rhinos, found {len(et_candidates)}')
        self.et_decoupler_part, et_subtree = et_candidates[0]
        self.et_decoupler = self.et_decoupler_part.decoupler

        self.booster_decouplers = []
        self.booster_branch_parts = []
        self.booster_branches = []
        self.booster_kinds = []
        for part in parts:
            if (part is self.et_decoupler_part or part not in et_subtree or
                    part.name != 'PC.RSRM.RadialDecoupler'):
                continue
            subtree = descendants(part)
            branch_engines = [part_engine(p) for p in subtree
                              if part_engine(p) is not None]
            solid_fuel = resource_total(subtree, 'SolidFuel', maximum=True)
            if not branch_engines or solid_fuel <= 0:
                continue
            if not any(e.part.name == 'PC.4Seg.RSRM' for e in branch_engines):
                continue
            self.booster_decouplers.append(part.decoupler)
            self.booster_branch_parts.append(subtree)
            self.booster_branches.append(branch_engines)
            self.booster_kinds.append('solid')
        if len(self.booster_branches) != 2:
            raise ValueError(f'Expected two RSRM booster branches, found {len(self.booster_branches)}')

        booster_parts = {p for branch in self.booster_branch_parts for p in branch}
        self.tanks = [p for p in et_subtree if p not in booster_parts and
                      has_resource(p, 'LiquidFuel') and has_resource(p, 'Oxidizer') and
                      p.resources.max('LiquidFuel') > 0 and p.resources.max('Oxidizer') > 0]
        if not self.tanks:
            raise ValueError('Could not identify ET propellant tanks')
        self.shuttle_tank = one(parts, 'Rockomax8BW')
        self.transfer_resources = ('LiquidFuel',)

        frame = self.vessel.reference_frame
        orbiter_pos = self.skiff.part.position(frame)
        et_pos = tuple(sum(e.part.position(frame)[i] for e in self.et_engines) /
                       len(self.et_engines) for i in range(3))
        dx, dz = orbiter_pos[0]-et_pos[0], orbiter_pos[2]-et_pos[2]
        length = math.hypot(dx, dz)
        if length < 1:
            raise ValueError('Invalid orbiter/ET geometry')
        self.radial = (dx/length, dz/length)

        self.inner_skippers = []
        self.outer_skippers = []
        self.core_groups = [[self.skiff], self.et_engines]
        self.boosters = [e for branch in self.booster_branches for e in branch]
        self.liquid_boosters = []
        self.solid_boosters = [e for e in self.boosters
                               if resource_total([e.part], 'SolidFuel', maximum=True) > 0]
        self.liquid_booster_group = []
        self.groups = self.core_groups
        self.sepmotors = [p.engine for p in booster_parts
                          if p.name == 'sepMotor1' and p.engine is not None]
        self.transfers = {}
        self.separated = False
        self.booster_separation_ut = None
        self.et_attached = True
        self.control_moment_budget = 0.0
        self.cruise_control_moment_budget = 0.0
        self.et_reference_thrust = sum(self._engine_sea_level_max_thrust(e)
                                       for e in self.et_engines)

    def _init_orbiter_shuttle(self, parts):
        """Initialize the three-SSME orbiter, external tank, twin-LFB craft."""
        self.vehicle_kind = 'orbiter-shuttle'
        engines = [part_engine(p) for p in parts if part_engine(p) is not None]
        ssmes = [e for e in engines if e.part.name == 'SSME']
        self.orbiter_engines = [e for e in engines
                                if e.part.name == 'benjee10.AJ10.v2']
        if len(ssmes) != 3:
            raise ValueError(f'Expected three SSMEs, found {len(ssmes)}')
        if len(self.orbiter_engines) != 2:
            raise ValueError(f'Expected two AJ10 orbital engines, found {len(self.orbiter_engines)}')
        self.skiff = self.orbiter_engines[0]
        self.wolfhound = None
        self.et_skippers = []
        self.et_engines = []

        et_candidates = []
        for part in parts:
            if part.name != 'radialDecoupler1-2':
                continue
            subtree = descendants(part)
            carries_et = any(
                p.name == 'Size4.Tank.04' or
                (has_resource(p, 'LiquidFuel') and has_resource(p, 'Oxidizer'))
                for p in subtree)
            if carries_et:
                et_candidates.append((part, subtree))
        if len(et_candidates) != 1:
            raise ValueError(f'Expected one external-tank separator, found {len(et_candidates)}')
        self.et_decoupler_part, et_subtree = et_candidates[0]
        self.et_decoupler = self.et_decoupler_part.decoupler

        self.booster_decouplers = []
        self.booster_branch_parts = []
        self.booster_branches = []
        self.booster_kinds = []
        for part in et_subtree:
            if part.name != 'radialDecoupler':
                continue
            subtree = descendants(part)
            branch_engines = [part_engine(p) for p in subtree
                              if part_engine(p) is not None]
            lfb_engines = [e for e in branch_engines
                           if e.part.name == 'Size2LFB.v2']
            srb_engines = [e for e in branch_engines
                           if e.part.name == 'Clydesdale']
            lf = resource_total(subtree, 'LiquidFuel', maximum=True)
            ox = resource_total(subtree, 'Oxidizer', maximum=True)
            sf = resource_total(subtree, 'SolidFuel', maximum=True)
            if len(lfb_engines) == 1 and not srb_engines and lf > 0 and ox > 0:
                branch_engines = lfb_engines
                kind = 'liquid'
            elif len(srb_engines) == 1 and not lfb_engines and sf > 0:
                # The attached Mite motors are on the decoupler stage. Keep them
                # under stock staging; only the Clydesdale motors burn on launch.
                branch_engines = srb_engines
                kind = 'solid'
            else:
                continue
            self.booster_decouplers.append(part.decoupler)
            self.booster_branch_parts.append(subtree)
            self.booster_branches.append(branch_engines)
            self.booster_kinds.append(kind)
        if len(self.booster_branches) != 2:
            raise ValueError(f'Expected two supported side boosters, found {len(self.booster_branches)}')
        booster_parts = {p for branch in self.booster_branch_parts for p in branch}
        self.tanks = [p for p in et_subtree if p not in booster_parts and
                      (has_resource(p, 'LiquidFuel') or has_resource(p, 'Oxidizer'))]
        if not self.tanks:
            raise ValueError('Could not identify external-tank propellant storage')
        self.shuttle_tank = None
        self.transfer_resources = ()

        self.boosters = [e for branch in self.booster_branches for e in branch]
        self.liquid_boosters = [e for branch, kind in
                                zip(self.booster_branches, self.booster_kinds)
                                if kind == 'liquid' for e in branch]
        self.solid_boosters = [e for branch, kind in
                               zip(self.booster_branches, self.booster_kinds)
                               if kind == 'solid' for e in branch]
        self.liquid_booster_group = list(self.liquid_boosters)
        self.core_groups = [ssmes]
        self.groups = self.core_groups + ([self.liquid_booster_group]
                                          if self.liquid_booster_group else [])
        self.inner_skippers = []
        self.outer_skippers = []
        self.sepmotors = []
        self.transfers = {}
        self.separated = False
        self.booster_separation_ut = None
        self.et_attached = True
        self.control_moment_budget = 0.0
        self.cruise_control_moment_budget = 0.0
        frame = self.vessel.reference_frame
        engine_pos = [e.part.position(frame) for e in ssmes]
        et_pos = tuple(sum(e.part.position(frame)[i] for e in self.orbiter_engines) /
                       len(self.orbiter_engines) for i in range(3))
        dx = sum(p[0] for p in engine_pos) / len(engine_pos) - et_pos[0]
        dz = sum(p[2] for p in engine_pos) / len(engine_pos) - et_pos[2]
        length = math.hypot(dx, dz)
        if length < 1:
            raise ValueError('Invalid orbiter/external-tank geometry')
        self.radial = (dx/length, dz/length)
        self.et_reference_thrust = sum(self._engine_sea_level_max_thrust(e)
                                       for e in ssmes)
    def project(self, position):
        return position[0]*self.radial[0] + position[2]*self.radial[1]

    def _solid_booster_state(self):
        if self.separated or not self.solid_boosters:
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

    @staticmethod
    def _engine_sea_level_max_thrust(engine):
        vac_thrust = getattr(engine, 'max_vacuum_thrust', 0.0)
        sea_isp = getattr(engine, 'kerbin_sea_level_specific_impulse', 0.0)
        vac_isp = getattr(engine, 'vacuum_specific_impulse', 0.0)
        if vac_thrust > 0 and sea_isp > 0 and vac_isp > 0:
            return vac_thrust * sea_isp / vac_isp
        return max(0.0, engine.max_thrust)

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

    def booster_nominal_burn_time(self):
        if self.liquid_boosters:
            return self.liquid_booster_nominal_burn_time()
        if not self.solid_boosters or self.separated:
            return 0.0
        propellant_mass = sum(resource_mass([engine.part], ('SolidFuel',))
                              for engine in self.solid_boosters)
        mass_flow = sum(self._engine_mass_flow(engine)
                        for engine in self.solid_boosters)
        return propellant_mass/mass_flow if mass_flow > 0 else math.inf

    def et_skipper_upper_bound(self):
        """Let Wolfhound pressure recovery replace ET Skipper thrust continuously."""
        if not self.et_attached:
            return 1.0
        wolfhound_thrust = self.wolfhound.max_thrust if self.wolfhound.has_fuel else 0.0
        skipper_full = sum(e.max_thrust for e in self.et_skippers if e.has_fuel)
        if skipper_full <= 0:
            return 0.0
        required_skipper = max(0.0, self.et_reference_thrust - wolfhound_thrust)
        return min(1.0, required_skipper / skipper_full)

    def group_upper_bounds(self, altitude=0.0):
        bounds = [1.0] * len(self.groups)
        if (not self.et_attached or
                self.vehicle_kind in ('nuclear-solid', 'orbiter-shuttle')):
            return bounds
        skipper_cap = self.et_skipper_upper_bound()
        if len(bounds) >= 4:
            bounds[2] = skipper_cap
            outer_blend = smoothstep01(max(0.0, altitude) / 15000.0)
            bounds[3] = min(skipper_cap, OUTER_SKIPPER_PAD_CAP +
                            (1.0-OUTER_SKIPPER_PAD_CAP)*outer_blend)
        if not self.separated and len(bounds) >= 5:
            booster_blend = smoothstep01(max(0.0, altitude) / 10000.0)
            bounds[4] = BOOSTER_PAD_CAP + (1.0-BOOSTER_PAD_CAP)*booster_blend
        return bounds

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
        """Light only the core engines and measure their control authority on clamps."""
        engines = [engine for group in self.core_groups for engine in group]
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
        roll_accel = roll_accel_authority_deg_s2(self.vessel)
        self.cruise_control_moment_budget = authority * CONTROL_TORQUE_FRACTION
        self.control_moment_budget = self.cruise_control_moment_budget
        fraction = actual_thrust / target_thrust if target_thrust > 0 else 0.0
        return authority, self.control_moment_budget, fraction, roll_accel

    def _group_state(self):
        frame = self.vessel.reference_frame
        thrusts, arms, mass_flows = [], [], []
        for group in self.groups:
            values = [(e.max_thrust if e.has_fuel else 0.0,
                       self.project(e.part.position(frame))) for e in group]
            thrust = sum(t for t, _ in values)
            thrusts.append(thrust)
            arms.append(sum(t*a for t, a in values)/thrust if thrust else 0.0)
            mass_flows.append(sum(self._engine_mass_flow(e) for e in group))
        return thrusts, arms, mass_flows

    def allocation(self, max_g=None, target_g=None, altitude=0.0):
        thrusts, arms, mass_flows = self._group_state()
        fixed_thrust, fixed_moment = self._solid_booster_state()
        requested_g = max_g if target_g is None else min(max_g, target_g)
        thrust_cap = None
        if requested_g is not None:
            thrust_cap = max(0.0, requested_g * G0 * self.vessel.mass - fixed_thrust)
        result = allocate(thrusts, arms, fixed_moment,
                          thrust_cap=thrust_cap,
                          upper_bounds=self.group_upper_bounds(altitude),
                          mass_flows=mass_flows,
                          moment_tolerance=self.control_moment_budget)
        return Allocation(result.throttles, result.thrust+fixed_thrust,
                          result.residual, result.feasible, result.mass_flow)

    def allocation_for_throttles(self, throttles):
        thrusts, arms, mass_flows = self._group_state()
        fixed_thrust, fixed_moment = self._solid_booster_state()
        thrust = fixed_thrust + sum(t*u for t, u in zip(thrusts, throttles))
        residual = fixed_moment + sum(t*a*u for t, a, u in zip(thrusts, arms, throttles))
        mass_flow = sum(m*u for m, u in zip(mass_flows, throttles))
        return Allocation(tuple(throttles), thrust, residual,
                          abs(residual) <= self.control_moment_budget + 1e-5,
                          mass_flow)

    def commanded_g(self, result, main_throttle=1.0):
        if self.vessel.mass <= 0:
            return 0.0
        if self.solid_boosters and not self.separated:
            fixed_thrust, _ = self._solid_booster_state()
            thrust = fixed_thrust + max(0.0, result.thrust-fixed_thrust)*main_throttle
        else:
            thrust = result.thrust*main_throttle
        return thrust / (self.vessel.mass * G0)

    def booster_telemetry(self):
        if self.separated:
            return 0.0, 0.0, 0.0, 0.0, 0.0
        thrust = sum(engine_thrust(e) for e in self.boosters)
        lf = sum(resource_total(parts, 'LiquidFuel') for parts in self.booster_branch_parts)
        ox = sum(resource_total(parts, 'Oxidizer') for parts in self.booster_branch_parts)
        sf = sum(resource_total([engine.part], 'SolidFuel')
                 for engine in self.solid_boosters)
        current_mass = sum(
            resource_mass(parts, ('LiquidFuel', 'Oxidizer')) if kind == 'liquid'
            else resource_mass([engine.part], ('SolidFuel',))
            for parts, branch, kind in zip(self.booster_branch_parts,
                                           self.booster_branches,
                                           self.booster_kinds)
            for engine in branch[:1])
        max_mass = sum(
            resource_mass(parts, ('LiquidFuel', 'Oxidizer'), maximum=True)
            if kind == 'liquid' else
            resource_mass([engine.part], ('SolidFuel',), maximum=True)
            for parts, branch, kind in zip(self.booster_branch_parts,
                                           self.booster_branches,
                                           self.booster_kinds)
            for engine in branch[:1])
        return thrust, lf, ox, sf, current_mass/max_mass if max_mass > 0 else 0.0

    def gimbal_telemetry(self):
        """Return live gimbal actuation for engines that expose it through kRPC."""
        engines = [e for group in self.groups for e in group] + self.boosters
        result = []
        for engine in engines:
            name = engine.part.name
            try:
                if engine.gimballed:
                    position = tuple(round(x, 3) for x in
                                     engine.part.position(self.vessel.reference_frame))
                    result.append((name, position, tuple(engine.gimbal_actuation)))
            except Exception:
                continue
        return tuple(result)

    def engine_limit_telemetry(self):
        engines = [e for group in self.groups for e in group] + self.boosters
        result = []
        for engine in engines:
            try:
                result.append((engine.part.name,
                               tuple(round(x, 3) for x in
                                     engine.part.position(self.vessel.reference_frame)),
                               round(engine.thrust_limit, 4), bool(engine.active)))
            except Exception:
                continue
        return tuple(result)

    def et_resource_amount(self, resource):
        if not self.et_attached:
            return 0.0
        return sum(t.resources.amount(resource) for t in self.tanks)

    def top_off(self):
        if not self.transfer_resources:
            return
        for resource in self.transfer_resources:
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
        return bool(self.solid_boosters) and all(
            not engine_has_fuel(engine) for engine in self.solid_boosters)

    def _separate_boosters(self):
        # Cut every controlled booster motor before detaching the full pair.
        for engine in self.liquid_boosters + getattr(self, 'solid_boosters', []):
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

    def update(self, max_g=3.0, target_g=None, altitude=0.0, dt=0.1,
               throttle_slew=THROTTLE_SLEW_PER_SECOND):
        if self.et_attached:
            self.top_off()
        if (not self.separated and
                (self._liquid_booster_depleted() or self._solid_boosters_depleted())):
            self._separate_boosters()
        target = self.allocation(max_g=max_g, target_g=target_g, altitude=altitude)
        current = tuple(
            sum(engine.thrust_limit for engine in group) / len(group)
            for group in self.groups)
        applied = slew_vector(current, target.throttles, throttle_slew, dt)
        for group, throttle in zip(self.groups, applied):
            for engine in group:
                engine.thrust_limit = throttle
        return self.allocation_for_throttles(applied)

    def release_et(self):
        self.vessel.control.throttle = 0
        if any(not transfer.complete for transfer in self.transfers.values()):
            return False
        if self.vehicle_kind == 'orbiter-shuttle':
            for group in self.core_groups:
                for engine in group:
                    engine.active = False
            self.et_decoupler.decouple()
            self.et_attached = False
            self.core_groups = [self.orbiter_engines]
            self.groups = [self.orbiter_engines]
            for engine in self.orbiter_engines:
                engine.active = True
                engine.thrust_limit = 1.0
            print('External tank separated; AJ10 orbital engines enabled')
            return True
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
        body = v.orbit.body
        preflight_altitude = v.flight(body.reference_frame).mean_altitude
        preflight_target_g = ascent_target_g(
            preflight_altitude, body.atmosphere_depth, args.max_g, args.launch_g,
            args.booster_min_g)
        result = shuttle.allocation(
            max_g=args.max_g, target_g=preflight_target_g, altitude=preflight_altitude)
        booster_names = ', '.join(sorted({e.part.title for e in shuttle.boosters}))
        booster_max_thrust = sum(e.max_thrust for e in shuttle.boosters)
        kinds = ','.join(shuttle.booster_kinds)
        booster_lf = sum(resource_total(parts, 'LiquidFuel', maximum=True)
                         for parts in shuttle.booster_branch_parts)
        booster_ox = sum(resource_total(parts, 'Oxidizer', maximum=True)
                         for parts in shuttle.booster_branch_parts)
        booster_sf = sum(resource_total([engine.part], 'SolidFuel', maximum=True)
                         for engine in shuttle.solid_boosters)
        if shuttle.vehicle_kind == 'nuclear-solid':
            print(f'{v.name}: {v.mass/1000:.2f} t; engine groups Orbiter-J-N500 / ET-2xRhino / fixed twin-RSRM')
        elif shuttle.vehicle_kind == 'orbiter-shuttle':
            booster_type = ('Clydesdale SRB' if shuttle.solid_boosters else
                            'Size2 liquid booster')
            print(f'{v.name}: {v.mass/1000:.2f} t; engine groups '
                  f'Orbiter-3xSSME / Booster-2x{booster_type}')
        else:
            print(f'{v.name}: {v.mass/1000:.2f} t; engine groups '
                  'Orbiter-Skiff / ET-Wolfhound / ET-Skipper-inner2 / ET-Skipper-outer2 / '
                  'Booster-4x(Kodiak+4xCub)')
        print(f'Detected boosters: {len(shuttle.booster_branches)} branch(es), '
              f'{len(shuttle.boosters)} engine(s), kinds={kinds}: {booster_names}; '
              f'{booster_max_thrust/1000:.0f} kN max; LF/Ox/SF={booster_lf:.0f}/{booster_ox:.0f}/{booster_sf:.0f}')
        fixed_thrust, _ = shuttle._solid_booster_state()
        full_thrust = (sum(engine.max_thrust for group in shuttle.groups for engine in group) +
                       fixed_thrust)
        full_throttle_g = full_thrust / (v.mass * G0) if v.mass > 0 else 0.0
        if shuttle.vehicle_kind == 'nuclear-solid':
            print(f"Orbiter J-N500 / ET Rhino thrust: {shuttle.skiff.max_thrust/1000:.0f}/"
                  f"{sum(e.max_thrust for e in shuttle.et_engines)/1000:.0f} kN; "
                  f"fixed SRB thrust {fixed_thrust/1000:.0f} kN")
        elif shuttle.vehicle_kind == 'orbiter-shuttle':
            ssme_thrust = sum(e.max_thrust for e in shuttle.core_groups[0])
            oms_thrust = sum(e.max_thrust for e in shuttle.orbiter_engines)
            print(f"Orbiter SSMEs / paired AJ10 OMS: {ssme_thrust/1000:.0f}/"
                  f"{oms_thrust/1000:.0f} kN; external-tank propellant "
                  f"LF/Ox={shuttle.et_resource_amount('LiquidFuel'):.0f}/"
                  f"{shuttle.et_resource_amount('Oxidizer'):.0f}")
        else:
            skipper_cap = shuttle.et_skipper_upper_bound()
            skipper_thrust = sum(e.max_thrust for e in shuttle.et_skippers)
            print(f'Wolfhound current/vacuum max thrust: {shuttle.wolfhound.max_thrust/1000:.1f}/'
                  f'{shuttle.wolfhound.max_vacuum_thrust/1000:.1f} kN; '
                  f'ET Skipper common cap {skipper_cap*100:.1f}% '
                  f'({skipper_thrust*skipper_cap/1000:.0f} kN of {skipper_thrust/1000:.0f} kN)')
        print(f'Full-throttle physical thrust acceleration at current pressure: {full_throttle_g:.2f} G')
        print(f'Preflight throttles: {result.throttles}; residual {result.residual:.0f} N m; '
              f'commanded {shuttle.commanded_g(result):.2f} G / target {preflight_target_g:.2f} G')
        print(f'Pad group caps: {tuple(round(x, 3) for x in shuttle.group_upper_bounds(preflight_altitude))}; '
              f'booster floor {args.booster_min_g:.2f} G; throttle slew {args.throttle_slew:.2f}/s')
        if shuttle.boosters:
            nominal_burn = shuttle.booster_nominal_burn_time()
            print(f'Booster full-throttle burn estimate at current pressure: {nominal_burn:.1f} s')
        if not args.launch:
            print('Read-only preflight complete. Use --launch to fly.')
            return
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
        mission_roll = 0.0
        post_booster_roll = roll_for_heading(args.heading)
        initial_pitch = 89.0
        commanded_pitch = initial_pitch
        launch_roll = None
        mission_roll_commanded = False
        mission_roll_settled = False
        roll_settle_since = None
        initial_roll_set = False
        liftoff_ut = None
        roll_reset = False
        roll_reset_ut = None
        roll_rate = 0.0
        body_roll_rate = 0.0
        last_roll_angle = None
        last_roll_ut = None
        saved_auto_tune = None
        saved_roll_pid = None
        roll_surface_states = []
        roll_accel_reference = 0.0
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
            print(f'Pad roll {launch_roll:.1f} deg; physical heads-down roll starts after '
                  f'+{args.roll_start:.0f} m climb; manual roll cap {args.roll_rate:.1f} deg/s; '
                  f'gravity turn uses prograde-AoA guidance; flip to spaceward {args.roll_reset_delay:.0f} s after booster separation')
            clamps = list(v.parts.launch_clamps)
            if shuttle.vehicle_kind == 'orbiter-shuttle' and shuttle.solid_boosters:
                if control.stage_lock:
                    raise RuntimeError('Staging is locked; cannot ignite Clydesdales and release clamps')
                next_stage = control.current_stage - 1
                if next_stage < 0 or any(engine.part.stage != next_stage
                                         for engine in shuttle.solid_boosters):
                    raise RuntimeError(
                        'Clydesdales are not in the next stage; refusing to release the clamps')
                if any(not engine.can_restart for group in shuttle.core_groups
                       for engine in group):
                    raise RuntimeError('Core engine cannot be ignited independently before booster staging')
            if clamps:
                authority, budget, spool, roll_accel_reference = shuttle.calibrate_control_authority(control)
                launch_target_g = ascent_target_g(
                    flight.mean_altitude, body.atmosphere_depth, args.max_g, args.launch_g,
                    args.booster_min_g)
                result = shuttle.allocation(
                    max_g=args.max_g, target_g=launch_target_g, altitude=flight.mean_altitude)
                print(f'Launch control authority: {authority/1000:.0f} kN m; '
                      f'moment budget {budget/1000:.0f} kN m; spool {spool*100:.0f}%; '
                      f'roll accel {roll_accel_reference:.1f} deg/s^2')
            core_group_count = len(shuttle.core_groups)
            for group, u in zip(shuttle.core_groups, result.throttles[:core_group_count]):
                for e in group:
                    e.thrust_limit = u
                    e.active = True
            control.throttle = 1
            core_engines = [engine for group in shuttle.core_groups for engine in group]
            expected_core_thrust = sum(
                engine.max_thrust * throttle
                for group, throttle in zip(shuttle.core_groups,
                                           result.throttles[:core_group_count])
                for engine in group)
            if expected_core_thrust <= 0:
                raise RuntimeError('Launch allocation leaves the core engines at zero thrust')
            ignition_deadline = time.monotonic() + 8.0
            while time.monotonic() < ignition_deadline:
                if conn.space_center.active_vessel != v:
                    raise RuntimeError('Active vessel changed during core-engine ignition')
                core_thrust = sum(engine_thrust(engine) for engine in core_engines)
                if (all(engine.active for engine in core_engines) and
                        core_thrust >= expected_core_thrust * .5):
                    break
                time.sleep(.05)
            else:
                raise RuntimeError(
                    'Core engines did not reach 50% of allocated thrust; '
                    'boosters remain unignited and clamps remain attached')
            print(f'Core engines ignited on clamps: {core_thrust/1000:.0f} kN; '
                  'starting boosters only after verified core thrust')

            if shuttle.vehicle_kind == 'orbiter-shuttle' and shuttle.solid_boosters:
                # These solid motors cannot be started by Engine.active. The next
                # stage ignites them and releases the clamps; SSMEs are already lit.
                control.activate_next_stage()
                if conn.space_center.active_vessel != v:
                    raise RuntimeError('Active vessel changed during booster ignition staging')
                booster_deadline = time.monotonic() + 5.0
                expected_booster_thrust = sum(engine.max_thrust
                                              for engine in shuttle.solid_boosters)
                while time.monotonic() < booster_deadline:
                    booster_thrust = sum(engine_thrust(engine)
                                         for engine in shuttle.solid_boosters)
                    if (all(engine.active for engine in shuttle.solid_boosters) and
                            booster_thrust >= expected_booster_thrust * .5):
                        break
                    time.sleep(.05)
                else:
                    raise RuntimeError('Clydesdale ignition failed after stage activation')
                print(f'Clydesdales ignited; clamps released; booster thrust '
                      f'{booster_thrust/1000:.0f} kN')
            else:
                for group, u in zip(shuttle.groups[core_group_count:],
                                    result.throttles[core_group_count:]):
                    for engine in group:
                        engine.thrust_limit = u
                        engine.active = True
                for engine in shuttle.boosters:
                    if engine not in shuttle.solid_boosters:
                        engine.active = True
                for clamp in clamps:
                    clamp.release()
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
                                 'body_roll_rate_deg_s', 'roll_control', 'pitch_error_deg', 'heading_error_deg',
                                 'roll_control_oscillation',
                                 'actual_pitch_deg', 'actual_heading_deg', 'target_pitch_deg',
                                 'control_moment_budget_Nm', 'engine_limit_state',
                                 'engine_gimbal_actuation', 'available_torque_Nm'])
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
                        print(f'Liftoff: holding pad roll {launch_roll:.1f} deg; physical heads-down roll deferred')
                    earthward_mode, reset_due = earthward_roll_target(
                        initial_roll_set, shuttle.booster_separation_ut,
                        conn.space_center.ut, args.roll_reset_delay)
                    roll_allowed = mission_roll_allowed(
                        initial_roll_set, altitude_gain, args.roll_start)
                    if roll_allowed and not mission_roll_commanded:
                        saved_auto_tune = ap.auto_tune
                        saved_roll_pid = ap.roll_pid_gains
                        ap.auto_tune = False
                        # Keep the craft's aerodynamic roll authority. The prior
                        # isolation left direct roll with too little authority once
                        # pitch-over coupling and dynamic pressure increased.
                        ap.roll_pid_gains = (0.0, 0.0, 0.0)
                        ap.target_roll = float('nan')
                        control.roll = 0
                        mission_roll_commanded = True
                        print(f'Physical orbiter-side roll started: earthward; '
                              f'aerodynamic roll surfaces remain enabled')
                    roll_active = mission_roll_commanded and state in ('ASCENT', 'ORBITER_ASCENT')
                    if recovered_autopilot and roll_active:
                        ap.auto_tune = False
                        ap.roll_pid_gains = (0.0, 0.0, 0.0)
                        ap.target_roll = float('nan')
                    target_roll = launch_roll if not mission_roll_commanded else None
                    if reset_due and not roll_reset:
                        roll_reset = True
                        roll_reset_ut = conn.space_center.ut
                        print('Post-booster physical roll: orbiter side now targets spaceward')
                    roll_requires_authority = False
                    if state == 'ASCENT':
                        roll_requires_authority = mission_roll_commanded and not mission_roll_settled
                        if roll_requires_authority or altitude < turn_start_altitude:
                            raw_pitch = initial_pitch
                        else:
                            raw_pitch = gravity_turn_pitch(
                                flight.vertical_speed, flight.horizontal_speed,
                                flight.dynamic_pressure, initial_pitch)
                        commanded_pitch = slew_value(
                            commanded_pitch, raw_pitch,
                            PITCH_SLEW_DEG_PER_SECOND, 1/args.hz)
                        pitch = commanded_pitch
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
                        orbital_thrust = sum(
                            engine.max_thrust for engine in
                            getattr(shuttle, 'orbiter_engines', [shuttle.skiff]))
                        acceleration = orbital_thrust/v.mass
                        if acceleration <= 0:
                            raise RuntimeError('Orbiter engines have no thrust for insertion')
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
                        target_g = ascent_target_g(
                            altitude, body.atmosphere_depth, args.max_g, args.launch_g,
                            args.booster_min_g if not shuttle.separated else 0.0)
                        result = shuttle.update(
                            max_g=args.max_g, target_g=target_g, altitude=altitude,
                            dt=1/args.hz, throttle_slew=args.throttle_slew)
                        if state == 'ASCENT':
                            control.throttle = g_force_throttle(
                                control.throttle, flight.g_force, args.max_g)
                            control.throttle = dynamic_pressure_throttle(
                                control.throttle, flight.dynamic_pressure, args.max_q)
                    else:
                        shuttle.skiff.thrust_limit = 1
                    actual_roll = ''
                    roll_control = 0.0
                    if roll_active and state in ('ASCENT', 'ORBITER_ASCENT'):
                        # AutoPilot owns pitch/yaw only. Direct roll control keeps the
                        # real orbiter-mounted side (-Z) on the requested Earth/space side.
                        ap.target_direction = target_direction
                        ap.target_roll = float('nan')
                        surface_frame = v.surface_reference_frame
                        actual_direction = v.direction(surface_frame)
                        roll_up_reference = surface_up_for_pitch_heading(
                            target_pitch, args.heading)
                        actual_side = conn.space_center.transform_direction(
                            (0, 0, -1), v.reference_frame, surface_frame)
                        target_side = desired_orbiter_side(
                            target_pitch, args.heading, bool(earthward_mode))
                        actual_roll = roll_for_roof(
                            actual_direction, roll_up_reference, actual_side)
                        physical_target_roll = roll_for_roof(
                            actual_direction, roll_up_reference, target_side)
                        target_roll = physical_target_roll
                        roll_error_signed = roll_delta(
                            actual_roll, physical_target_roll)
                        roll_error = abs(roll_error_signed)
                        body_roll_rate = body_roll_rate_deg_s(v)
                        roll_rate = body_roll_rate
                        pitch_error, heading_error = surface_axis_errors(
                            actual_direction, target_pitch, args.heading)
                        roll_control = manual_roll_command(
                            roll_error_signed, body_roll_rate, roll_accel_reference,
                            args.roll_rate, args.roll_control_limit,
                            args.roll_response_time)
                        control.roll = roll_control
                        settle_nose_error = direction_error_deg(
                            actual_direction, target_direction)
                        if not mission_roll_settled:
                            if roll_attitude_settled(
                                    roll_error, body_roll_rate,
                                    settle_nose_error):
                                if roll_settle_since is None:
                                    roll_settle_since = conn.space_center.ut
                                elif (conn.space_center.ut-roll_settle_since >=
                                      args.roll_settle_time):
                                    mission_roll_settled = True
                                    print('Physical heads-down roll settled; gravity turn released')
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
                        if state in ('ASCENT', 'ORBITER_ASCENT'):
                            roll_error = abs(roll_delta(actual_roll, target_roll))
                            pitch_error, heading_error = surface_axis_errors(
                                actual_direction, target_pitch, args.heading)
                        else:
                            # Orbital-frame attitude errors are diagnostic only here.
                            try:
                                roll_error = abs(ap.roll_error)
                            except Exception:
                                roll_error = 0.0
                            pitch_error = 0.0
                            heading_error = 0.0
                    else:
                        ap.target_direction = target_direction
                        roll_error = 0.0
                        pitch_error = 0.0
                        heading_error = 0.0
                    nose_error = direction_error_deg(v.direction(ap.reference_frame), target_direction)
                    try:
                        autopilot_error = ap.error
                    except Exception:
                        autopilot_error = ''
                    if conn.space_center.ut >= next_report:
                        actual_pitch, actual_heading = pitch_heading_for_direction(
                            v.direction(v.surface_reference_frame))
                        booster_thrust, booster_lf, booster_ox, booster_sf, booster_fraction = shuttle.booster_telemetry()
                        commanded_g = (shuttle.commanded_g(result, control.throttle)
                                       if shuttle.et_attached else 0.0)
                        booster_full_time = shuttle.booster_nominal_burn_time()
                        engine_limits = shuttle.engine_limit_telemetry()
                        gimbal_actuation = shuttle.gimbal_telemetry()
                        try:
                            available_torque = tuple(
                                tuple(round(x, 1) for x in side)
                                for side in v.available_torque)
                        except Exception:
                            available_torque = ()
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
                                         nose_error, autopilot_error, actual_roll, roll_error, roll_rate,
                                         body_roll_rate, roll_control, pitch_error, heading_error,
                                         ap.roll_control_oscillation,
                                         actual_pitch, actual_heading,
                                         '' if target_pitch is None else target_pitch,
                                         shuttle.control_moment_budget, engine_limits,
                                         gimbal_actuation, available_torque])
                        log.flush()
                        print(f'{state}: altitude {altitude/1000:.1f} km; Ap {v.orbit.apoapsis_altitude/1000:.1f} km; residual {result.residual:.0f} N m; budget {shuttle.control_moment_budget:.0f} N m; cmd {commanded_g:.2f} G; actual {flight.g_force:.2f} G; nose {nose_error:.1f} deg; roll {roll_error:.1f} deg; rate {roll_rate:.1f}/{body_roll_rate:.1f} deg/s; ctl {roll_control:.2f}; gimbal {gimbal_actuation}; torque {available_torque}')
                        next_report = conn.space_center.ut+2
                    since_roll_reset = (None if roll_reset_ut is None else
                                        conn.space_center.ut-roll_reset_ut)
                    if attitude_error_exceeded(nose_error, altitude, since_roll_reset):
                        raise RuntimeError(f'Nose attitude error {nose_error:.1f} deg exceeds 45 degrees')
                    orbital_engines = getattr(shuttle, 'orbiter_engines', [shuttle.skiff])
                    if not shuttle.et_attached and not any(e.has_fuel for e in orbital_engines):
                        raise RuntimeError('Orbiter engines out of fuel')
                    time.sleep(max(0, 1/args.hz-(time.monotonic()-tick)))
        finally:
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            try:
                try:
                    if saved_roll_pid is not None:
                        ap.roll_pid_gains = saved_roll_pid
                    restore_aero_roll(roll_surface_states)
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
    parser.add_argument('--turn-start', type=float, default=3000,
                        help='Altitude in m where the post-roll gravity turn begins')
    parser.add_argument('--turn-end', type=float, default=55000,
                        help='Altitude in m where the smoothed gravity turn reaches minimum pitch')
    parser.add_argument('--meco-periapsis', type=float, default=45000,
                        help='Minimum periapsis altitude required for normal MECO in m')
    parser.add_argument('--meco-pitch', type=float, default=0,
                        help='Surface pitch used to build horizontal energy after target apoapsis')
    parser.add_argument('--max-g', type=float, default=3.0,
                        help='Maximum commanded thrust acceleration in Earth g')
    parser.add_argument('--max-q', type=float, default=MAX_DYNAMIC_PRESSURE_PA,
                        help='Maximum dynamic pressure in Pa; reduces liquid throttle above this value')
    parser.add_argument('--launch-g', type=float, default=LAUNCH_TARGET_G,
                        help='Gentle initial thrust-acceleration target in Earth g')
    parser.add_argument('--booster-min-g', type=float, default=BOOSTER_MIN_G,
                        help='Hard minimum thrust acceleration while boosters are attached and burning')
    parser.add_argument('--throttle-slew', type=float, default=THROTTLE_SLEW_PER_SECOND,
                        help='Maximum per-second change of each engine-group thrust limiter')
    parser.add_argument('--roll-reset-delay', type=float, default=180,
                        help='Seconds after booster separation before the delayed 180-degree roll')
    parser.add_argument('--roll-start', type=float, default=30,
                        help='Climb above launch point in m before starting the mission roll')
    parser.add_argument('--roll-rate', type=float, default=6.0,
                        help='Direct physical-roll angular-rate cap in deg/s')
    parser.add_argument('--roll-attenuation', type=float, default=5.0,
                        help='Roll-error angle in deg where AutoPilot starts slowing the roll')
    parser.add_argument('--roll-time-to-peak', type=float, default=2.5,
                        help='AutoTune time-to-peak for the roll rate controller in seconds')
    parser.add_argument('--roll-overshoot', type=float, default=0.001,
                        help='AutoTune target roll overshoot fraction')
    parser.add_argument('--roll-settle-time', type=float, default=2.0,
                        help='Seconds roll/pitch/yaw errors must remain settled')
    parser.add_argument('--roll-control-limit', type=float, default=1.0,
                        help='Maximum direct roll-control magnitude during physical roll')
    parser.add_argument('--roll-response-time', type=float, default=.20,
                        help='Rate-controller response horizon in seconds for physical roll')
    parser.add_argument('--hz', type=float, default=10)
    parser.add_argument('--timeout', type=float, default=1800)
    parser.add_argument('--telemetry', default='telemetry.csv')
    args = parser.parse_args()
    if (not 0 < args.hz <= 50 or not 0 <= args.turn_start < args.turn_end or
            args.timeout <= 0 or args.roll_reset_delay < 0 or args.roll_start < 0 or
            not 0 < args.roll_rate <= 30 or not 0 < args.roll_attenuation <= 90 or
            args.roll_time_to_peak <= 0 or not 0 <= args.roll_overshoot <= 1 or
            args.roll_settle_time < 0 or not 0 < args.roll_control_limit <= 1 or
            args.roll_response_time <= 0 or not 0 < args.launch_g <= args.max_g <= 10 or
            not 0 < args.booster_min_g <= args.max_g or not 0 < args.throttle_slew <= 1 or
            not 0 <= args.meco_periapsis < args.orbit or
            not -10 <= args.meco_pitch <= 20):
        parser.error('Invalid loop rate, ascent/MECO profile, G limit, timeout, or roll profile')
    run(args)


if __name__ == '__main__':
    main()
