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
