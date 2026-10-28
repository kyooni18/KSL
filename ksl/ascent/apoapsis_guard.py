"""Predict and limit apoapsis growth while the thrust vector pitches down."""
import math

from .peg import apsides
from .geometry import smooth_value

G0 = 9.80665


def projected_apoapsis(mu, radius, radial_speed, horizontal_speed, thrust_pitch,
                       pitch_response_seconds, acceleration, response_lag=2.0, dt=1.0,
                       hold_seconds=0.0):
    """Coast-or-burn prediction while a smooth thrust target becomes horizontal.

    The planar model uses inertial radial/horizontal velocity and constant thrust
    acceleration, neglecting drag and mass change. A small response lag makes
    the predicted pitch follow the commanded response rather than instantly moving.
    """
    if (not all(math.isfinite(x) for x in (mu, radius, radial_speed, horizontal_speed,
                                            thrust_pitch, pitch_response_seconds, acceleration,
                                            response_lag, dt, hold_seconds)) or
            min(mu, radius, pitch_response_seconds, dt) <= 0 or acceleration < 0):
        raise ValueError('Invalid apoapsis prediction input')
    r, vr, vh = radius, radial_speed, horizontal_speed
    pitch = max(0.0, min(90.0, thrust_pitch))
    commanded = pitch
    horizon = hold_seconds+5*pitch_response_seconds+3*response_lag+10.0
    peak = apsides(mu, r, vr, vh)[0]
    elapsed = 0.0
    while elapsed < horizon:
        step = min(dt, horizon-elapsed)
        if elapsed >= hold_seconds:
            commanded = smooth_value(commanded, 0.0, pitch_response_seconds, step)
        pitch += (commanded-pitch)*min(1.0, step/max(response_lag, 0.01))
        angle = math.radians(pitch)
        def derivative(r_, vr_, vh_):
            return (vr_, acceleration*math.sin(angle)-mu/(r_*r_)+vh_*vh_/r_,
                    acceleration*math.cos(angle)-vr_*vh_/r_)
        k = derivative(r, vr, vh)
        k = derivative(r+k[0]*step/2, vr+k[1]*step/2, vh+k[2]*step/2)
        r, vr, vh = r+k[0]*step, vr+k[1]*step, vh+k[2]*step
        peak = max(peak, apsides(mu, r, vr, vh)[0])
        elapsed += step
    return peak


def apoapsis_guard_g(mu, radius, radial_speed, horizontal_speed, thrust_pitch,
                     pitch_response_seconds, target_radius, requested_g, floor_g=0.15,
                     margin_m=1000.0, hold_seconds=0.0):
    """Find a bounded G target whose predicted peak apoapsis stays below target.

    Never request an engine cutoff: if even the minimum G exceeds the target,
    retain that minimum and report the predicted excess to the caller.
    """
    if not all(math.isfinite(x) for x in (target_radius, requested_g, floor_g, margin_m)):
        raise ValueError('Invalid apoapsis guard input')
    ceiling = max(0.0, requested_g)
    floor = min(max(0.0, floor_g), ceiling)
    limit = target_radius-margin_m
    def predict(g):
        return projected_apoapsis(mu, radius, radial_speed, horizontal_speed,
                                  thrust_pitch, pitch_response_seconds, g*G0,
                                  hold_seconds=hold_seconds)
    full_peak = predict(ceiling)
    if full_peak <= limit:
        return ceiling, full_peak, True
    floor_peak = predict(floor)
    if floor_peak > limit:
        return floor, floor_peak, False
    low, high = floor, ceiling
    for _ in range(12):
        mid = (low+high)/2.0
        if predict(mid) <= limit:
            low = mid
        else:
            high = mid
    return low, predict(low), True
