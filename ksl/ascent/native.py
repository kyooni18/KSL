"""ctypes bridge to the Rust ascent guidance library; kRPC remains in Python."""
import ctypes
import platform
import sys
from pathlib import Path

_suffix = 'dylib' if sys.platform == 'darwin' else 'dll' if sys.platform == 'win32' else 'so'
_prefix = '' if sys.platform == 'win32' else 'lib'
_target = Path(__file__).resolve().parents[2] / 'rust' / 'ascent' / 'target'
if sys.platform == 'darwin' and platform.machine() == 'x86_64':
    _target /= 'x86_64-apple-darwin'
_path = _target / 'release' / f'{_prefix}ksl_ascent.{_suffix}'
try:
    _lib = ctypes.CDLL(str(_path))
except OSError as exc:
    raise RuntimeError(f'Rust ascent library unavailable at {_path}; run make ascent') from exc


def _function(name, arguments, result):
    function = getattr(_lib, name)
    function.argtypes = arguments
    function.restype = result
    return function

_d = ctypes.c_double
_b = ctypes.c_bool
roll_settled = _function('ksl_roll_settled', [_d]*6, _b)
launch_roll_handoff = _function('ksl_launch_roll_handoff', [_d]*6, _b)
maneuver_roll_rate = _function('ksl_maneuver_roll_rate', [_d]*3, _d)
roll_allowed = _function('ksl_roll_allowed', [_b, _d, _d, _d, _d], _b)

turn_reference = _function('ksl_turn_reference', [_d]*4, _d)
constrain_turn = _function('ksl_constrain_turn', [_d]*10, _d)
smoothstep = _function('ksl_smoothstep', [_d], _d)
slew_value = _function('ksl_slew_value', [_d]*4, _d)
smooth_value = _function('ksl_smooth_value', [_d]*4, _d)

_Vec3 = _d * 3
_Vec4 = _d * 4


class AttitudeInput(ctypes.Structure):
    _fields_ = [('rotation', _Vec4), ('angular_velocity', _Vec3),
                ('forward', _Vec3), ('roof', _Vec3), ('has_roof', _b),
                ('torque', _Vec3), ('moi', _Vec3), ('brake', _Vec3),
                ('max_rate', _Vec3), ('tau', _Vec3), ('integral', _Vec3),
                ('disturbance', _Vec3), ('translation', _Vec3),
                ('rate_tau', _d), ('int_limit', _d), ('int_time', _d), ('dt', _d),
                ('roll_authority', _d), ('roll_limit', _d), ('roll_start', _d),
                ('roll_full', _d), ('roll_min', _d), ('deadband', _d),
                ('previous_roll_error', _d)]


class AttitudeOutput(ctypes.Structure):
    _fields_ = [('inputs', _Vec3), ('integral', _Vec3), ('angular_velocity', _Vec3),
                ('error_deg', _d), ('roll_error_deg', _d)]


_attitude_step = _function('ksl_attitude_step',
                           [ctypes.POINTER(AttitudeInput), ctypes.POINTER(AttitudeOutput)], None)


def attitude_step(controller, rotation, angular_velocity, forward, roof, dt):
    p = AttitudeInput(_Vec4(*rotation), _Vec3(*angular_velocity), _Vec3(*forward),
                      _Vec3(*(roof if roof is not None else (0., 0., 0.))), roof is not None,
                      _Vec3(*controller._torque), _Vec3(*controller._moi),
                      _Vec3(*controller.brake_fraction), _Vec3(*controller.max_rate),
                      _Vec3(*controller.tau), _Vec3(*controller._integral),
                      _Vec3(*controller.disturbance), _Vec3(*controller.translation_moment),
                      controller.rate_tau, controller.int_limit, controller.int_time, dt,
                      controller.roll_authority_fraction, controller.roll_input_limit,
                      *controller.roll_gate, getattr(controller, 'roll_min_authority', 0.),
                      controller.deadband, getattr(controller, 'roll_error_deg', 0.))
    output = AttitudeOutput()
    _attitude_step(ctypes.byref(p), ctypes.byref(output))
    return output

_allocate = _function('ksl_allocate', [ctypes.c_size_t, *([ctypes.POINTER(_d)]*4),
                                      _d, _d, _d, _d, ctypes.POINTER(_d)], _b)


def allocate(thrusts, arms, bounds, flows, fixed, target, cap, tolerance):
    n = len(thrusts)
    array = _d*n
    output = array()
    if not _allocate(n, array(*thrusts), array(*arms), array(*bounds), array(*flows),
                     fixed, target, cap, tolerance, output):
        raise ArithmeticError('No allocation vertex found')
    return tuple(output)


class BoosterState(ctypes.Structure):
    _fields_ = [(name, _d) for name in (
        'radius', 'radial_speed', 'horizontal_speed', 'surface_horizontal_speed',
        'mass', 'commanded_pitch', 'flown_pitch', 'core_thrust', 'core_mass_flow',
        'booster_thrust', 'booster_mass_flow', 'booster_time', 'booster_dry_mass',
        'dynamic_pressure', 'drag_acceleration', 'lift_radial', 'lift_horizontal')]


class BoosterRow(ctypes.Structure):
    _fields_ = [(name, _d) for name in (
        'time', 'radius', 'radial_speed', 'horizontal_speed', 'mass', 'raw_pitch',
        'commanded_pitch', 'flown_pitch', 'thrust', 'core_thrust', 'booster_thrust',
        'mass_flow', 'dynamic_pressure')]


_predict_booster = _function('ksl_predict_booster',
    [ctypes.POINTER(BoosterState), _d, _d, _d, _d, _d, _b, _d, _d, _d, _d,
     ctypes.POINTER(BoosterRow), ctypes.c_size_t], ctypes.c_ssize_t)


def predict_booster(state, mu, body_radius, target_altitude, bias, end_bias,
                    response, attitude, dt, pitch_slew):
    import math
    seed = BoosterState(*(getattr(state, name) for name, _ in BoosterState._fields_))
    # A partial final integration step and the terminal sample each need a slot.
    capacity = math.ceil(state.booster_time / dt) + 2
    rows = (BoosterRow * capacity)()
    count = _predict_booster(ctypes.byref(seed), mu, body_radius, target_altitude,
                             bias, 0. if end_bias is None else end_bias,
                             end_bias is not None, response, attitude, dt, pitch_slew,
                             rows, capacity)
    if count == -1:
        raise ValueError('Predicted propellant consumption exceeds vehicle mass')
    if count < 0:
        raise ArithmeticError('Booster forecast exceeded trajectory capacity')
    return [{name: getattr(row, name) for name, _ in BoosterRow._fields_}
            for row in rows[:count]]


class PegState(ctypes.Structure):
    _fields_ = [(name, _d) for name in ('mu', 'freeze_tgo', 'a', 'b', 'tgo',
                                          'last_t', 'hold', 'peak')] + [
                  (name, _b) for name in ('has_solution', 'has_last_t', 'has_hold', 'converged')]


_peg_update = _function('ksl_peg_update',
    [ctypes.POINTER(PegState), *([_d] * 9), _b, ctypes.POINTER(_d), ctypes.POINTER(_d)], _b)


def peg_update(guide, t, r, vr, vh, accel, ve, r_t, vr_t, vh_t, constant_accel):
    state = PegState(guide.mu, guide.freeze_tgo, guide.A or 0., guide.B or 0.,
                     guide.tgo or 0., guide._last_t or 0., guide.hold or 0., guide.peak,
                     guide.A is not None, guide._last_t is not None, guide.hold is not None,
                     guide.converged)
    pitch, time = _d(), _d()
    valid = _peg_update(ctypes.byref(state), t, r, vr, vh, accel, ve, r_t, vr_t, vh_t,
                        constant_accel, ctypes.byref(pitch), ctypes.byref(time))
    guide.A = state.a if state.has_solution else None
    guide.B = state.b if state.has_solution else None
    guide.tgo = state.tgo
    guide._last_t = state.last_t if state.has_last_t else None
    guide.hold = state.hold if state.has_hold else None
    guide.peak = state.peak
    guide.converged = state.converged
    return (pitch.value if valid else None), time.value


class InsertionInput(ctypes.Structure):
    _fields_ = [(name, _d) for name in ('mu', 'ra', 'rp', 'ap_tol', 'pe_tol', 'slew',
                'hold_slew', 'lag', 'lo', 'hi', 'response')] + [('has_response', _b)] + [
                (name, _d) for name in ('rt', 'r', 'vr', 'vh', 'mass', 'thrust', 've',
                 'g_limit', 'pitch', 'hold_left', 'ramp', 'cap_now', 'dt', 't_max',
                 'flown', 'attitude_hold', 'anchor')] + [('has_anchor', _b)] + [
                ('since_separation', _d), ('dry_mass', _d)]


class InsertionRow(ctypes.Structure):
    _fields_ = [(name, _d) for name in ('time', 'radius', 'radial_speed',
                'horizontal_speed', 'mass', 'raw_pitch', 'commanded_pitch',
                'flown_pitch', 'thrust', 'mass_flow', 'throttle')]


class InsertionOutput(ctypes.Structure):
    _fields_ = [(name, _d) for name in ('score', 'apoapsis_r', 'periapsis_r',
                'peak_apoapsis_r', 'time', 'dv', 'saturated', 'radius', 'radial_speed',
                'horizontal_speed', 'mass', 'propellant_used')] + [
                ('fuel_feasible', _b), ('target_feasible', _b)]


_insertion_fly = _function('ksl_insertion_fly',
    [ctypes.POINTER(InsertionInput), ctypes.POINTER(InsertionOutput),
     ctypes.POINTER(InsertionRow), ctypes.c_size_t], ctypes.c_ssize_t)


def insertion_fly(guide, r_t, r, vr, vh, mass, thrust, ve, g_limit, pitch,
                  hold_left, ramp, cap_now, dt, t_max, flown, attitude_hold,
                  transition_anchor, since_separation, record, dry_mass):
    import math
    if not math.isfinite(dt) or dt <= 0 or not math.isfinite(t_max) or t_max < 0:
        raise ValueError('Invalid insertion forecast time step or duration')
    data = InsertionInput(guide.mu, guide.ra, guide.rp, guide.ap_tol, guide.pe_tol,
        guide.slew, guide.hold_slew, guide.lag, guide.lo, guide.hi,
        guide.pitch_response_seconds or 0., guide.pitch_response_seconds is not None,
        r_t, r, vr, vh, mass, thrust, ve, g_limit, pitch, hold_left, ramp, cap_now,
        dt, t_max, pitch if flown is None else flown, attitude_hold,
        transition_anchor or 0., transition_anchor is not None, since_separation, dry_mass)
    capacity = math.ceil(t_max / dt) + 2
    rows = (InsertionRow * capacity)()
    output = InsertionOutput()
    count = _insertion_fly(ctypes.byref(data), ctypes.byref(output), rows, capacity)
    if count < 0:
        raise ArithmeticError('Insertion forecast exceeded trajectory capacity')
    result = {'r_t': r_t, **{name: getattr(output, name) for name, _ in InsertionOutput._fields_}}
    if record:
        if count == 0:
            raise ValueError('Cannot record a zero-duration insertion forecast')
        result['trajectory'] = [{name: getattr(row, name) for name, _ in InsertionRow._fields_}
                                for row in rows[:count]]
    return result


class IntelligentState(ctypes.Structure):
    _fields_ = [(name, _d) for name in ('altitude', 'radial', 'horizontal', 'mass',
                'thrust', 'isp', 'q', 'drag', 'fixed_thrust', 'surface_horizontal',
                'lift_radial', 'lift_horizontal', 'mass_flow', 'fixed_mass_flow')]


class IntelligentTarget(ctypes.Structure):
    _fields_ = [(name, _d) for name in ('radius', 'mu', 'atmosphere', 'apoapsis',
                'periapsis', 'inclination', 'altitude_tolerance', 'inclination_tolerance')]


class IntelligentPlanner(ctypes.Structure):
    _fields_ = [(name, _d) for name in ('max_q', 'max_g', 'slew', 'pitch', 'clock')] + [
        ('peg', PegState), ('has_peg', _b), ('peg_active', _b), ('has_plan', _b),
        ('plan_rt', _d), ('plan_rp', _d)]


class IntelligentCommand(ctypes.Structure):
    _fields_ = [(name, _d) for name in ('pitch', 'throttle', 'predicted_apoapsis', 'horizon')] + [
        ('status', ctypes.c_int)]


_intelligent_command = _function('ksl_intelligent_command',
    [ctypes.POINTER(IntelligentPlanner), ctypes.POINTER(IntelligentState),
     ctypes.POINTER(IntelligentTarget), _d, ctypes.POINTER(IntelligentCommand)], None)
_intelligent_step = _function('ksl_intelligent_step',
    [ctypes.POINTER(IntelligentState), ctypes.POINTER(IntelligentTarget), _d, _d, _d,
     ctypes.POINTER(IntelligentState)], None)
_intelligent_apsides = _function('ksl_intelligent_apsides',
    [ctypes.POINTER(IntelligentState), ctypes.POINTER(IntelligentTarget), ctypes.POINTER(_d)], None)
_intelligent_coast = _function('ksl_intelligent_coast',
    [ctypes.POINTER(IntelligentState), ctypes.POINTER(IntelligentTarget)], _d)


def _native_struct(cls, value):
    return cls(*(getattr(value, name) for name, _ in cls._fields_))


def intelligent_step(state, target, pitch, throttle, dt):
    s, t = _native_struct(IntelligentState, state), _native_struct(IntelligentTarget, target)
    out = IntelligentState()
    _intelligent_step(ctypes.byref(s), ctypes.byref(t), pitch, throttle, dt, ctypes.byref(out))
    return {name: getattr(out, name) for name, _ in IntelligentState._fields_}


def intelligent_apsides(state, target):
    s, t = _native_struct(IntelligentState, state), _native_struct(IntelligentTarget, target)
    out = (_d * 2)()
    _intelligent_apsides(ctypes.byref(s), ctypes.byref(t), out)
    return tuple(out)


def intelligent_coast(state, target):
    s, t = _native_struct(IntelligentState, state), _native_struct(IntelligentTarget, target)
    return _intelligent_coast(ctypes.byref(s), ctypes.byref(t))


def intelligent_command(planner, state, target, dt):
    guide = planner.peg
    if guide is None:
        peg = PegState(target.mu, 8., 0., 0., 0., 0., 0., 0., False, False, False, False)
    else:
        peg = PegState(guide.mu, guide.freeze_tgo, guide.A or 0., guide.B or 0.,
                       guide.tgo or 0., guide._last_t or 0., guide.hold or 0., guide.peak,
                       guide.A is not None, guide._last_t is not None,
                       guide.hold is not None, guide.converged)
    plan = planner.peg_plan
    p = IntelligentPlanner(planner.max_q, planner.max_g, planner.slew, planner.pitch,
                           planner.clock, peg, guide is not None, planner.peg_active,
                           plan is not None, *(plan if plan is not None else (0., 0.)))
    s, t = _native_struct(IntelligentState, state), _native_struct(IntelligentTarget, target)
    out = IntelligentCommand()
    _intelligent_command(ctypes.byref(p), ctypes.byref(s), ctypes.byref(t), dt, ctypes.byref(out))
    planner.pitch, planner.clock, planner.peg_active = p.pitch, p.clock, p.peg_active
    planner.peg_plan = (p.plan_rt, p.plan_rp) if p.has_plan else None
    if guide is not None:
        guide.A, guide.B = (p.peg.a, p.peg.b) if p.peg.has_solution else (None, None)
        guide.tgo = p.peg.tgo if p.peg.has_last_t else None
        guide._last_t = p.peg.last_t if p.peg.has_last_t else None
        guide.hold = p.peg.hold if p.peg.has_hold else None
        guide.peak, guide.converged = p.peg.peak, p.peg.converged
    return out.pitch, out.throttle, out.predicted_apoapsis, out.horizon, ('powered', 'peg', 'cutoff')[out.status]
