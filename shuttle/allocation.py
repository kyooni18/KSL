"""Exact vertex search for one moment equality and up to four throttle groups.

Units: N, m, N m. Infeasible targets saturate to the closest attainable
moment; ties maximize liquid thrust. No optimizer dependency is required.
"""
from dataclasses import dataclass
from itertools import product
import math


@dataclass(frozen=True)
class Allocation:
    throttles: tuple[float, ...]
    thrust: float
    residual: float
    feasible: bool


def allocate(thrusts, arms, fixed_moment=0.0, target=0.0):
    if len(thrusts) != len(arms) or not thrusts:
        raise ValueError('Thrusts and arms must have the same nonzero length')
    if any(not math.isfinite(x) for x in (*thrusts, *arms, fixed_moment, target)):
        raise ValueError('Inputs must be finite')
    if any(t < 0 for t in thrusts):
        raise ValueError('Thrust must be nonnegative')
    coefficients = [t * a for t, a in zip(thrusts, arms)]
    wanted = target - fixed_moment
    lo = sum(min(0, a) for a in coefficients)
    hi = sum(max(0, a) for a in coefficients)
    attainable = min(hi, max(lo, wanted))
    candidates = []
    for free in range(len(thrusts)):
        others = [i for i in range(len(thrusts)) if i != free]
        for bits in product((0.0, 1.0), repeat=len(others)):
            u = [0.0] * len(thrusts)
            for i, bit in zip(others, bits):
                u[i] = bit
            remainder = attainable - sum(coefficients[i] * u[i] for i in others)
            a = coefficients[free]
            if abs(a) < 1e-9:
                if abs(remainder) > 1e-6:
                    continue
                u[free] = 1.0
            else:
                u[free] = remainder / a
                if not -1e-9 <= u[free] <= 1 + 1e-9:
                    continue
                u[free] = min(1.0, max(0.0, u[free]))
            candidates.append(u)
    if not candidates:
        raise ArithmeticError('No allocation vertex found')
    u = max(candidates, key=lambda v: sum(t * x for t, x in zip(thrusts, v)))
    residual = fixed_moment + sum(a * x for a, x in zip(coefficients, u)) - target
    return Allocation(tuple(u), sum(t * x for t, x in zip(thrusts, u)),
                      residual, abs(residual) <= 1e-5)
