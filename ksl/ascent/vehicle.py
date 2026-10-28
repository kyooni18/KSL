"""STS vehicle model from kRPC parts: engine groups, boosters, external tank, staging and thrust allocation."""
import math
import time
from krpc.schema import KRPC_pb2
from krpc.decoder import Decoder

from .allocation import Allocation, allocate
from .events import emit_event
from .engine_roles import is_oms_engine
from .geometry import slew_vector, smoothstep01
from .guidance import THROTTLE_SLEW_PER_SECOND


G0 = 9.80665
CONTROL_TORQUE_FRACTION = 0.65
BOOSTER_MIN_G = 1.20
BOOSTER_PAD_CAP = 1.00
OUTER_SKIPPER_PAD_CAP = 0.45



SSME_TRIM_MIN = 0.30
SSME_TRIM_MAX = 1.00


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


def limit_aero_roll(vessel, authority_limit=60.0):
    """Enable shuttle elevon roll at limited authority and return prior states."""
    states = []
    for part in vessel.parts.all:
        if 'elevon' not in getattr(part, 'name', '').lower():
            continue
        try:
            surface = part.control_surface
            if surface is None:
                continue
            enabled = surface.roll_enabled
            authority = surface.authority_limiter
            states.append((surface, enabled, authority))
            surface.roll_enabled = True
            surface.authority_limiter = min(authority, authority_limit)
        except Exception:
            continue
    return states


def schedule_roll_authority(states, dynamic_pressure, low_q_limit=60.0, high_q_limit=30.0):
    """Reduce elevon roll authority as dynamic pressure builds to protect the wings."""
    fraction = min(1.0, max(0.0, (dynamic_pressure-6000.0)/14000.0))
    limit = low_q_limit + (high_q_limit-low_q_limit)*fraction
    for surface, _, authority in states:
        try:
            surface.authority_limiter = min(authority, limit)
        except Exception:
            pass


def restore_aero_roll(states):
    """Restore control-surface states captured by limit_aero_roll."""
    for surface, enabled, authority in states:
        try:
            surface.roll_enabled = enabled
            surface.authority_limiter = authority
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


def is_separator_motor(engine):
    part = engine.part
    return (getattr(part, 'name', '') in ('sepMotor1', 'Mite') or
            'Mite' in getattr(part, 'title', ''))


def separate_sts_boosters_batch(conn, core_groups, boosters, motors, decouplers):
    """Submit the SRB cutoff, separator ignition and detach as one kRPC request.

    The Python client exposes only individual setters, so build the documented
    multi-call protocol request using the same types as its generated setters.
    An error can leave a partially executed request: never retry it blindly.
    """
    types = conn._types
    engine_type = types.class_type('SpaceCenter', 'Engine')
    calls = []
    labels = []

    def setter(engine, name, value, value_type):
        calls.append(conn._build_call('SpaceCenter', 'Engine_set_' + name,
                                      [engine, value], ['self', 'value'],
                                      [engine_type, value_type], None))
        labels.append('engine ' + name)

    for group in core_groups:
        for engine in group:
            setter(engine, 'ThrustLimit', min(engine.thrust_limit, POST_SEP_CAP_FLOOR),
                   types.float_type)
    for engine in boosters:
        setter(engine, 'ThrustLimit', 0.0, types.float_type)
        setter(engine, 'Active', False, types.bool_type)
    # Set every limit before any motor is ignited. Motors then ignite and
    # both branches detach in the same server request, in that order.
    for engine in motors:
        setter(engine, 'ThrustLimit', 1.0, types.float_type)
    start_index = len(calls)
    calls.append(conn.get_call(getattr, conn.space_center, 'ut'))
    labels.append('start time')
    for engine in motors:
        setter(engine, 'Active', True, types.bool_type)
    for decoupler in decouplers:
        calls.append(conn.get_call(decoupler.decouple))
        labels.append('booster decoupler')
    end_index = len(calls)
    calls.append(conn.get_call(getattr, conn.space_center, 'ut'))
    labels.append('end time')
    request = KRPC_pb2.Request()
    request.calls.extend(calls)
    with conn._rpc_connection_lock:
        conn._rpc_connection.send_message(request)
        response = conn._rpc_connection.receive_message(KRPC_pb2.Response)
    if response.HasField('error'):
        raise RuntimeError('Booster separation batch failed; staging may be partial: '
                           + response.error.description)
    failures = [f'{label}: {result.error.description}'
                for label, result in zip(labels, response.results)
                if result.HasField('error')]
    if len(response.results) != len(calls) or failures:
        raise RuntimeError('Booster separation batch incomplete; staging may be partial: '
                           + '; '.join(failures))
    start = Decoder.decode(conn, response.results[start_index].value, types.double_type)
    end = Decoder.decode(conn, response.results[end_index].value, types.double_type)
    return start, end


def delay_booster_separation(vehicle_kind, solid_boosters, depleted,
                             dynamic_pressure, max_pressure):
    """Hold spent STS SRBs until the requested aerodynamic release condition."""
    return (vehicle_kind == 'orbiter-shuttle' and bool(solid_boosters) and depleted
            and dynamic_pressure is not None and dynamic_pressure > max_pressure)


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


def tanks_empty(shuttle):
    """True when the stack propellant is exhausted; tolerant of transient null part handles."""
    try:
        return any(sum(t.resources.amount(r) for t in shuttle.tanks) < 1
                   for r in ('LiquidFuel', 'Oxidizer'))
    except Exception:
        return False


POST_SEP_CAP_FLOOR = 0.3
POST_SEP_CAP_RAMP = 15.0
POST_SEP_CAP_TIMEOUT = 60.0


def post_sep_throttle_cap(since_sep, since_roll_done):
    """SSME limiter cap after booster separation; the insertion planner models the same.

    Hold a low, steady limiter (small thrust-line moment, gimbals free) until the
    upright roll is finished, then ramp over 15 s. Ramping on time since separation
    instead let the SSMEs reach full G mid-roll, at a pitch frozen for the roll, while
    the planner still assumed the low cap: the climb ran on and lofted Ap to ~130 km
    for an 85 km target. The timeout only guards a roll that never starts.
    """
    if since_roll_done is None:
        into_ramp = since_sep-POST_SEP_CAP_TIMEOUT
    else:
        into_ramp = since_roll_done
    return POST_SEP_CAP_FLOOR + (1.0-POST_SEP_CAP_FLOOR)*min(
        1.0, max(0.0, into_ramp/POST_SEP_CAP_RAMP))


def thrust_axis_offset(conn, vessel, frame):
    """Resultant thrust direction of the running engines minus the nose axis, in `frame`.

    Aiming the nose at the guidance direction leaves the force off by this canted
    offset; subtracting it from the command points the resultant force instead.
    """
    total = [0.0, 0.0, 0.0]
    for engine in vessel.parts.engines:
        if not engine.active or engine.thrust <= 1.0:
            continue
        thrusters = engine.thrusters
        for thruster in thrusters:
            try:
                direction = thruster.thrust_direction(vessel.reference_frame)
            except Exception:
                # Some engine transforms are not exposed; the canted part axis is
                # the thrust axis for an ungimballed engine.
                direction = engine.part.direction(vessel.reference_frame)
            for i in range(3):
                total[i] += engine.thrust/len(thrusters)*direction[i]
    norm = math.sqrt(sum(c*c for c in total))
    if norm <= 0.0:
        return (0.0, 0.0, 0.0)
    sc = conn.space_center
    thrust = sc.transform_direction(tuple(c/norm for c in total), vessel.reference_frame, frame)
    nose = sc.transform_direction((0.0, 1.0, 0.0), vessel.reference_frame, frame)
    return tuple(thrust[i]-nose[i] for i in range(3))


def engine_pitch_moment(vessel, engines, full_thrust=False):
    """Pitch moment (N m, vessel x axis) of the engines' current thrust about the CoM.

    Uses each engine's real mounting (position and canted part axis) in the vessel
    frame, whose origin is the centre of mass, so CoM travel from propellant burn and
    booster separation is included. full_thrust uses pressure-corrected maximum
    thrust for fueled engines, independent of limiter and ignition transients.
    Gimbal deflection is excluded on purpose: this
    is the disturbance the gimbals must cancel.
    """
    frame = vessel.reference_frame
    moment = 0.0
    for engine in engines:
        thrust = (engine.max_thrust if engine.has_fuel else 0.0) if full_thrust else engine.thrust
        if thrust <= 1.0:
            continue
        d = engine.part.direction(frame)
        p = engine.part.position(frame)
        moment += thrust*(p[1]*d[2] - p[2]*d[1])
    return moment


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
        self.sepmotors = [engine for part in booster_parts
                          if (engine := part_engine(part)) is not None and
                          is_separator_motor(engine)]
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
        self.orbiter_engines = [e for e in engines if is_oms_engine(e)]
        if len(ssmes) != 3:
            raise ValueError(f'Expected three SSMEs, found {len(ssmes)}')
        if len(self.orbiter_engines) != 2:
            raise ValueError(f'Expected two OMS engines, found {len(self.orbiter_engines)}')
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
        self.sepmotors = [engine for part in booster_parts
                          if (engine := part_engine(part)) is not None and
                          is_separator_motor(engine)]
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
        if self.solid_boosters and (not self.sepmotors or
                                    len(self.sepmotors) % len(self.solid_boosters)):
            raise ValueError(f'Expected an equal number of separation motors per solid booster, '
                             f'found {len(self.sepmotors)} for {len(self.solid_boosters)} boosters')
        self.liquid_booster_group = list(self.liquid_boosters)
        self.core_groups = [ssmes]
        self.groups = self.core_groups + ([self.liquid_booster_group]
                                          if self.liquid_booster_group else [])
        self.inner_skippers = []
        self.outer_skippers = []
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
            try:
                # kRPC's max_thrust ignores the part's thrust limiter. Before
                # SRB ignition, honor that limiter in pad estimates.
                thrust = (engine.thrust if engine.active else
                          engine.max_thrust * engine.thrust_limit
                          if engine.has_fuel else 0.0)
                arm = self.project(engine.part.position(frame))
            except Exception:
                # A separated or destroyed booster invalidates its part handle.
                continue
            thrust_total += thrust
            moment += thrust * arm
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
        try:
            propellant_mass = sum(resource_mass([engine.part], ('SolidFuel',))
                                  for engine in self.solid_boosters)
            mass_flow = sum(self._engine_mass_flow(engine) * engine.thrust_limit
                            for engine in self.solid_boosters)
        except Exception:
            # kRPC can briefly return null engine handles around liftoff;
            # keep the last good estimate instead of aborting the ascent.
            return getattr(self, '_last_solid_burn_time', math.inf)
        burn_time = propellant_mass/mass_flow if mass_flow > 0 else math.inf
        self._last_solid_burn_time = burn_time
        return burn_time

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
            try:
                name = engine.part.name
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
        separation_timing = None
        if self.vehicle_kind == 'orbiter-shuttle' and self.solid_boosters:
            separation_timing = separate_sts_boosters_batch(
                self.conn, self.core_groups, self.liquid_boosters + self.solid_boosters,
                self.sepmotors, self.booster_decouplers)
            print(f'Ignited {len(self.sepmotors)} separator motors and detached '
                  'both boosters in one kRPC request')
            if separation_timing and separation_timing[1] > separation_timing[0]:
                print(f'WARNING: staging batch spanned '
                      f'{separation_timing[1]-separation_timing[0]:.3f} simulation seconds')
        else:
            # Other vehicle families retain their existing staging sequence.
            for engine in self.liquid_boosters + getattr(self, 'solid_boosters', []):
                try:
                    engine.thrust_limit = 0.0
                    engine.active = False
                except Exception:
                    pass
            for engine in self.sepmotors:
                try:
                    engine.thrust_limit = 1.0
                    engine.active = True
                except Exception:
                    pass
            for decoupler in self.booster_decouplers:
                try:
                    decoupler.decouple()
                except Exception:
                    pass
        self.separated = True
        self.booster_separation_ut = separation_timing[1] if separation_timing else self.sc.ut
        self.groups = list(self.core_groups)
        self.control_moment_budget = self.cruise_control_moment_budget
        emit_event('booster-separation', ut=self.booster_separation_ut,
                   batch_start_ut=separation_timing[0] if separation_timing else None)
        print('Boosters shut down/burned out and separated')

    def update(self, max_g=3.0, target_g=None, altitude=0.0, dt=0.1,
               throttle_slew=THROTTLE_SLEW_PER_SECOND,
               dynamic_pressure=None, separation_max_q=float('inf'),
               post_booster_coast_seconds=0.0, before_booster_separation=None):
        if self.et_attached:
            self.top_off()
        depleted = (not self.separated and
                    (self._liquid_booster_depleted() or self._solid_boosters_depleted()))
        hold_separation = delay_booster_separation(
            self.vehicle_kind, self.solid_boosters, depleted,
            dynamic_pressure, separation_max_q)
        if hold_separation:
            if getattr(self, 'booster_separation_hold_ut', None) is None:
                self.booster_separation_hold_ut = self.sc.ut
                emit_event('booster-separation-hold', q_pa=dynamic_pressure,
                           max_q_pa=separation_max_q)
                print(f'Coasting spent SRBs until q <= {separation_max_q:.0f} Pa')
            elif self.sc.ut - self.booster_separation_hold_ut > 30.0:
                raise RuntimeError('Dynamic pressure did not fall enough for SRB separation within 30 s')
        if depleted and not hold_separation:
            if before_booster_separation is not None:
                before_booster_separation()
            self._separate_boosters()
        target = self.allocation(max_g=max_g, target_g=target_g, altitude=altitude)
        current = tuple(
            sum(engine.thrust_limit for engine in group) / len(group)
            for group in self.groups)
        target_throttles = target.throttles
        if (self.vehicle_kind == 'orbiter-shuttle' and self.solid_boosters and
                not self.separated and any(engine.active and engine.has_fuel
                                           for engine in self.solid_boosters)):
            # Prioritize pitch balance while the unthrottleable SRBs burn,
            # even when their thrust alone exceeds the requested G target.
            target_throttles = list(target_throttles)
            target_throttles[0] = current[0]
            ssmes = self.core_groups[0]
            try:
                # Trim the SSME limiter so the canted SSME moment cancels the SRB
                # thrust-line moment as the CoM moves; the gimbals then only fly
                # the vehicle instead of sitting saturated against the offset.
                per_limit = engine_pitch_moment(self.vessel, ssmes, full_thrust=True)
                m_booster = engine_pitch_moment(self.vessel, self.solid_boosters)
                if abs(per_limit) > 1e4:
                    trim = -m_booster/per_limit
                    target_throttles[0] = max(SSME_TRIM_MIN, min(SSME_TRIM_MAX, trim))
                    self.pitch_moment = m_booster + per_limit*current[0]
                # Keep balancing through the last fueled tick. The low-limiter
                # clearance/roll schedule begins only after booster burnout.
            except Exception:
                pass
            target_throttles = tuple(target_throttles)
        elif hold_separation:
            target_throttles = tuple(0.0 for _ in self.groups)
        elif (self.vehicle_kind == 'orbiter-shuttle' and self.solid_boosters and
              self.separated and self.et_attached):
            since_sep = self.sc.ut - self.booster_separation_ut
            if since_sep < post_booster_coast_seconds:
                target_throttles = tuple(0.0 for _ in self.groups)
            else:
                target_throttles = self._post_booster_core_throttles(
                    max_g, target_g, since_sep)
        applied = slew_vector(current, target_throttles, throttle_slew, dt)
        for group, throttle in zip(self.groups, applied):
            for engine in group:
                engine.thrust_limit = throttle
        return self.allocation_for_throttles(applied)

    def _post_booster_core_throttles(self, max_g, target_g, since_sep):
        # Pad control authority is not representative after SRB separation.
        # Do not let that stale moment envelope trap the three SSMEs at a
        # low throttle: use the G target directly and rely on gimbal control.
        available_core_thrust = sum(
            engine.max_thrust for group in self.core_groups for engine in group)
        requested_g = max_g if target_g is None else min(max_g, target_g)
        requested_thrust = requested_g * G0 * self.vessel.mass
        core_throttle = (min(1.0, requested_thrust / available_core_thrust)
                         if available_core_thrust > 0 else 0.0)
        # The CoM jump at booster separation momentarily needs more pitch
        # authority than the gimbals have at a high SSME limiter. Ride a
        # lower limiter (smaller off-axis moment) briefly, then ramp up.
        roll_done = getattr(self, 'post_sep_roll_done_ut', None)
        sep_cap = post_sep_throttle_cap(
            since_sep, None if roll_done is None else self.sc.ut-roll_done)
        core_throttle = min(core_throttle, sep_cap)
        return tuple(core_throttle for _ in self.groups)

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
            try:
                # Orbiter power for the reaction wheels (no solar panels on board).
                for converter in self.vessel.parts.resource_converters:
                    for i in range(converter.count):
                        if not converter.active(i):
                            converter.start(i)
            except Exception:
                pass
            emit_event('et-separation')
            print('External tank separated; OMS engines enabled')
            return True
        for group in self.core_groups[1:]:
            for engine in group:
                engine.active = False
        self.et_decoupler.decouple()
        self.et_attached = False
        self.core_groups = [[self.skiff]]
        self.groups = [[self.skiff]]
        self.skiff.thrust_limit = 1
        emit_event('et-separation')
        print('External tank separated')
        return True

