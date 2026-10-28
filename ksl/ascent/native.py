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
