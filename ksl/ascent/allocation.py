"""Exact vertex search for bounded moment-balanced throttle allocation.

Units: N, m, N m. The solver maximizes throttleable thrust while minimizing
moment residual, optionally under a total throttleable-thrust cap. No optimizer
dependency is required.
"""
from dataclasses import dataclass
from itertools import combinations, product
import math


@dataclass(frozen=True)
class Allocation:
    throttles: tuple[float, ...]
    thrust: float
    residual: float
    feasible: bool
    mass_flow: float


def _bounded(value, tol=1e-9):
    return -tol <= value <= 1 + tol


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

    n = len(thrusts)
    effective_thrusts = [t * bound for t, bound in zip(thrusts, upper_bounds)]
    effective_mass_flows = [m * bound for m, bound in zip(mass_flows, upper_bounds)]
    coefficients = [t * a for t, a in zip(effective_thrusts, arms)]
    cap = math.inf if thrust_cap is None else thrust_cap
    wanted = target - fixed_moment
    candidates = []

    def add(u):
        if not all(_bounded(x) for x in u):
            return
        u = [min(1.0, max(0.0, x)) for x in u]
        thrust = sum(t * x for t, x in zip(effective_thrusts, u))
        if thrust <= cap + 1e-6:
            candidates.append(u)

    # Box vertices, including the all-zero fallback.
    for bits in product((0.0, 1.0), repeat=n):
        add(list(bits))

    # Intersections of the admissible moment slab boundaries with box edges.
    # Include the exact target plane too: it remains useful as a best-effort
    # fallback if the requested tolerance cannot be met.
    moment_planes = [wanted]
    if moment_tolerance > 0:
        moment_planes.extend((wanted - moment_tolerance, wanted + moment_tolerance))
    for moment_plane in moment_planes:
        for free in range(n):
            others = [i for i in range(n) if i != free]
            for bits in product((0.0, 1.0), repeat=len(others)):
                u = [0.0] * n
                for i, bit in zip(others, bits):
                    u[i] = bit
                remainder = moment_plane - sum(coefficients[i] * u[i] for i in others)
                a = coefficients[free]
                if abs(a) < 1e-12:
                    if abs(remainder) <= 1e-6:
                        u[free] = 1.0
                        add(u)
                    continue
                u[free] = remainder / a
                add(u)

    if math.isfinite(cap):
        # Intersections of the thrust-cap plane with box edges.
        for free in range(n):
            others = [i for i in range(n) if i != free]
            for bits in product((0.0, 1.0), repeat=len(others)):
                u = [0.0] * n
                for i, bit in zip(others, bits):
                    u[i] = bit
                t = effective_thrusts[free]
                remainder = cap - sum(effective_thrusts[i] * u[i] for i in others)
                if t <= 1e-12:
                    if abs(remainder) <= 1e-6:
                        u[free] = 1.0
                        add(u)
                    continue
                u[free] = remainder / t
                add(u)

        # Intersections of each moment-slab boundary and the thrust-cap
        # plane with box faces.
        for moment_plane in moment_planes:
            for i, j in combinations(range(n), 2):
                others = [k for k in range(n) if k not in (i, j)]
                for bits in product((0.0, 1.0), repeat=len(others)):
                    u = [0.0] * n
                    for k, bit in zip(others, bits):
                        u[k] = bit
                    rhs_m = moment_plane - sum(coefficients[k] * u[k] for k in others)
                    rhs_t = cap - sum(effective_thrusts[k] * u[k] for k in others)
                    det = coefficients[i] * effective_thrusts[j] - coefficients[j] * effective_thrusts[i]
                    if abs(det) < 1e-12:
                        continue
                    u[i] = (rhs_m * effective_thrusts[j] - coefficients[j] * rhs_t) / det
                    u[j] = (coefficients[i] * rhs_t - rhs_m * effective_thrusts[i]) / det
                    add(u)

    if not candidates:
        raise ArithmeticError('No allocation vertex found')

    def score(u):
        moment = fixed_moment + sum(a * x for a, x in zip(coefficients, u))
        residual = abs(moment - target)
        uncovered = max(0.0, residual - moment_tolerance - 1e-5)
        thrust = sum(t * x for t, x in zip(effective_thrusts, u))
        mass_flow = sum(m * x for m, x in zip(effective_mass_flows, u))
        # First keep the residual inside the torque authority envelope, then
        # maximize thrust. Among equally powerful solutions prefer less control
        # effort, then lower propellant flow.
        return (uncovered, -thrust, residual, mass_flow)

    normalized = min(candidates, key=score)
    throttles = tuple(x * bound for x, bound in zip(normalized, upper_bounds))
    thrust = sum(t * x for t, x in zip(thrusts, throttles))
    mass_flow = sum(m * x for m, x in zip(mass_flows, throttles))
    residual = fixed_moment + sum((t * a) * x for t, a, x in zip(thrusts, arms, throttles)) - target
    return Allocation(throttles, thrust, residual,
                      abs(residual) <= moment_tolerance + 1e-5, mass_flow)
