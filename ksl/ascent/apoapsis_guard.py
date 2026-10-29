"""Predict and limit apoapsis growth while the thrust vector pitches down."""
import math

from .peg import apsides
from .geometry import smooth_value

G0 = 9.80665


def reentry_before_meco(mu, radius, radial_speed, horizontal_speed, acceleration,
                        thrust_pitch, target_pitch, response_seconds, atmosphere_radius,
                        horizon, target_apoapsis_radius, target_periapsis_radius,
                        trim_radius=None, trim_coast_seconds=0., dt=2.):
    """Predict reentry before cutoff or failure to reach trim altitude afterward.

    This point-mass forecast assumes constant acceleration, a first-order
    command response and two-second attitude lag. It neglects drag and mass
    change; an ascending vehicle below the atmosphere boundary is not yet
    reentering. A target-orbit cutoff ends the powered forecast early. If a
    trim altitude is provided, the cutoff coast must be able to reach it and
    remain out of reentry through the requested post-cutoff coast interval.
    """
    trim_radius = atmosphere_radius if trim_radius is None else trim_radius
    values = (mu, radius, radial_speed, horizontal_speed, acceleration, thrust_pitch,
              target_pitch, response_seconds, atmosphere_radius, horizon,
              target_apoapsis_radius, target_periapsis_radius, trim_radius,
              trim_coast_seconds, dt)
    if (not all(math.isfinite(x) for x in values) or mu <= 0. or radius <= 0. or
            acceleration < 0. or response_seconds <= 0. or dt <= 0. or
            horizon < 0. or trim_coast_seconds < 0.):
        raise ValueError('Invalid reentry forecast input')
    r, vr, vh = radius, radial_speed, horizontal_speed
    command = flown = thrust_pitch
    elapsed = 0.
    while elapsed < horizon:
        if r <= atmosphere_radius and vr < 0.:
            return True
        ra, rp = apsides(mu, r, vr, vh)
        if ra >= target_apoapsis_radius-3000. and rp >= target_periapsis_radius-300.:
            break
        step = min(dt, horizon-elapsed)
        command = smooth_value(command, target_pitch, response_seconds, step)
        flown += (command-flown)*min(1., step/2.)
        angle = math.radians(flown)
        def derivative(r_, vr_, vh_):
            return (vr_, acceleration*math.sin(angle)-mu/(r_*r_)+vh_*vh_/r_,
                    acceleration*math.cos(angle)-vr_*vh_/r_)
        k = derivative(r, vr, vh)
        k = derivative(r+k[0]*step/2., vr+k[1]*step/2., vh+k[2]*step/2.)
        r, vr, vh = r+k[0]*step, vr+k[1]*step, vh+k[2]*step
        if r <= 0.:
            return True
        elapsed += step
    if r <= atmosphere_radius and vr < 0.:
        return True
    if r < trim_radius:
        # If still below the OMS altitude at cutoff, only an ascending coast
        # whose apoapsis reaches it can deliver the orbiter there.
        if vr <= 0. or apsides(mu, r, vr, vh)[0] < trim_radius:
            return True
    coast_elapsed = 0.
    while coast_elapsed < trim_coast_seconds:
        if r <= atmosphere_radius and vr < 0.:
            return True
        step = min(dt, trim_coast_seconds-coast_elapsed)
        def coast_derivative(r_, vr_, vh_):
            return vr_, -mu/(r_*r_)+vh_*vh_/r_, -vr_*vh_/r_
        k = coast_derivative(r, vr, vh)
        k = coast_derivative(r+k[0]*step/2., vr+k[1]*step/2., vh+k[2]*step/2.)
        r, vr, vh = r+k[0]*step, vr+k[1]*step, vh+k[2]*step
        if r <= 0.:
            return True
        coast_elapsed += step
    if r <= atmosphere_radius and vr < 0.:
        return True
    return False


def recovery_pitch(raw_pitch, guard_active, plan_feasible, apoapsis, target_apoapsis,
                   forecast):
    """Return (pitch, status), preserving a path to the OMS trim altitude.

    A forecast with no pitch that avoids reentry is not evidence that full-up
    thrust will save the flight; retain the horizontal command and report it.
    """
    approaching_target = not plan_feasible and apoapsis >= 0.9*target_apoapsis
    if not (guard_active or approaching_target):
        return raw_pitch, 'inactive'
    if not reentry_before_meco(target_pitch=0., **forecast):
        return min(raw_pitch, 0.), 'horizontal'
    # Choose the smallest upward pitch that avoids reentry in the forecast;
    # the sequence does not require positive vertical velocity at MECO.
    if reentry_before_meco(target_pitch=90., **forecast):
        return min(raw_pitch, 0.), 'unavoidable'
    low, high = 0., 90.
    for _ in range(10):
        mid = (low+high)/2.
        if reentry_before_meco(target_pitch=mid, **forecast):
            low = mid
        else:
            high = mid
    return max(min(raw_pitch, 0.), high), 'climb-for-clearance'


def recovery_guidance(raw_pitch, guard_active, plan_feasible, apoapsis, target_apoapsis,
                      forecast, max_acceleration):
    """Return pitch and, if required, acceleration to preserve the OMS coast.

    If no pitch at the guarded acceleration can preserve the coast, look for
    the least extra horizontal acceleration that does. Only try an upward
    pitch at maximum available acceleration when horizontal thrust cannot.
    """
    pitch, status = recovery_pitch(raw_pitch, guard_active, plan_feasible,
                                   apoapsis, target_apoapsis, forecast)
    if status != 'unavoidable' or max_acceleration <= forecast['acceleration']:
        return pitch, status, None
    maximum = dict(forecast, acceleration=max_acceleration)
    if not reentry_before_meco(target_pitch=0., **maximum):
        low, high = forecast['acceleration'], max_acceleration
        for _ in range(12):
            mid = (low+high)/2.
            if reentry_before_meco(target_pitch=0., **dict(forecast, acceleration=mid)):
                low = mid
            else:
                high = mid
        return min(raw_pitch, 0.), 'horizontal-accelerated', high
    pitch, status = recovery_pitch(raw_pitch, True, plan_feasible,
                                   apoapsis, target_apoapsis, maximum)
    if status != 'unavoidable':
        return pitch, status, max_acceleration
    return min(raw_pitch, 0.), 'unavoidable', None


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
