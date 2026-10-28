"""Exact vertex search for bounded moment-balanced throttle allocation.

Units: N, m, N m. The solver maximizes throttleable thrust while minimizing
moment residual, optionally under a total throttleable-thrust cap. No optimizer
dependency is required.
"""
from dataclasses import dataclass
import math

from . import native


@dataclass(frozen=True)
class Allocation:
    throttles: tuple[float, ...]
    thrust: float
    residual: float
    feasible: bool
    mass_flow: float


def allocate(thrusts, arms, fixed_moment=0.0, target=0.0, thrust_cap=None,
             upper_bounds=None, mass_flows=None, moment_tolerance=0.0):
    if len(thrusts) != len(arms) or not thrusts:
        raise ValueError('Thrusts and arms must have the same nonzero length')
    if upper_bounds is None:
        upper_bounds = [1.0] * len(thrusts)
    if len(upper_bounds) != len(thrusts):
        raise ValueError('Upper bounds must match thrust groups')
    if any(not 0 <= x <= 1 for x in upper_bounds):
        raise ValueError('Upper bounds must be between zero and one')
    if mass_flows is None:
        mass_flows = [0.0] * len(thrusts)
    if len(mass_flows) != len(thrusts) or any(m < 0 for m in mass_flows):
        raise ValueError('Mass flows must match thrust groups and be nonnegative')
    values = (*thrusts, *arms, *upper_bounds, *mass_flows, fixed_moment, target,
              moment_tolerance)
    if thrust_cap is not None:
        values += (thrust_cap,)
    if any(not math.isfinite(x) for x in values):
        raise ValueError('Inputs must be finite')
    if any(t < 0 for t in thrusts):
        raise ValueError('Thrust must be nonnegative')
    if thrust_cap is not None and thrust_cap < 0:
        raise ValueError('Thrust cap must be nonnegative')
    if moment_tolerance < 0:
        raise ValueError('Moment tolerance must be nonnegative')

    throttles = native.allocate(thrusts, arms, upper_bounds, mass_flows, fixed_moment,
                                target, math.inf if thrust_cap is None else thrust_cap,
                                moment_tolerance)
    thrust = sum(t * x for t, x in zip(thrusts, throttles))
    mass_flow = sum(m * x for m, x in zip(mass_flows, throttles))
    residual = fixed_moment + sum((t * a) * x for t, a, x in zip(thrusts, arms, throttles)) - target
    return Allocation(throttles, thrust, residual,
                      abs(residual) <= moment_tolerance + 1e-5, mass_flow)
