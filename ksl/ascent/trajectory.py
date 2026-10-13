"""Ascent trajectory corridor: closed-loop following of a reference climb.

The open-loop gravity-turn schedule in ``guidance.scheduled_turn_pitch`` is
altitude-driven and can only turn as fast as the q*alpha load bound allows, so a
heavy (low-TWR) stack or a late pitch-over arrives high and nearly vertical
(live flights lofted to 100-105 km for an 85 km target with 700 m/s of vertical
speed left at 67 km). The corridor closes the loop on the *predicted apoapsis*:
for every altitude there is a reference apoapsis that a well-shaped climb has;
being above it means the trajectory is too lofty (pitch down), below it too flat
(pitch up). Altitude/apoapsis are mass independent, so the same reference holds
for any payload — a heavy stack simply takes longer to follow it.
"""
import math

# (altitude / target apoapsis, reference apoapsis / target apoapsis)
CORRIDOR = (
    (0.00, 0.00),
    (0.12, 0.25),
    (0.22, 0.42),
    (0.34, 0.62),
    (0.45, 0.78),
    (0.57, 0.90),
    (0.70, 0.98),
    (0.82, 1.03),
    (1.00, 1.03),
)

CORRIDOR_MIN_ALTITUDE = 6000.0   # m: below this the pad/low-q turn program owns pitch
CORRIDOR_GAIN_DEG_PER_KM = 0.35
CORRIDOR_MAX_CORRECTION_DEG = 10.0
Q_ALPHA_LIMIT = 50000.0          # Pa*deg, same load bound as guidance.aoa_limit_deg


def reference_apoapsis(altitude, target_apoapsis, table=CORRIDOR):
    """Apoapsis altitude a well-shaped climb has at ``altitude`` (piecewise linear)."""
    target = max(1000.0, target_apoapsis)
    x = max(0.0, altitude)/target
    if x >= table[-1][0]:
        return table[-1][1]*target
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x <= x1:
            return (y0 + (y1-y0)*(x-x0)/(x1-x0))*target
    return table[-1][1]*target


def corridor_correction_deg(altitude, apoapsis, target_apoapsis,
                            gain=CORRIDOR_GAIN_DEG_PER_KM,
                            limit=CORRIDOR_MAX_CORRECTION_DEG):
    """Pitch correction (deg, negative = pitch down) for the apoapsis error."""
    error_km = (apoapsis - reference_apoapsis(altitude, target_apoapsis))/1000.0
    return max(-limit, min(limit, -gain*error_km))


def corridor_pitch(base_pitch, prograde_pitch, altitude, apoapsis, target_apoapsis,
                   dynamic_pressure, min_altitude=CORRIDOR_MIN_ALTITUDE,
                   q_alpha=Q_ALPHA_LIMIT, alpha_floor=1.5, alpha_ceiling=20.0):
    """Apply the corridor correction without breaking the q*alpha load bound.

    The corrected pitch may not trail the surface velocity by more than the
    allowed angle of attack, and may lead it by at most 1 degree.
    """
    if altitude < min_altitude:
        return base_pitch
    pitch = base_pitch + corridor_correction_deg(altitude, apoapsis, target_apoapsis)
    alpha = max(alpha_floor, min(alpha_ceiling, q_alpha/max(1.0, dynamic_pressure)))
    return min(prograde_pitch + 1.0, max(pitch, prograde_pitch - alpha))


def energy_fraction(altitude, inertial_speed, mu, radius, target_altitude, pad_speed):
    """Progress 0..1+ of the climb in specific orbital energy (mass independent)."""
    e = 0.5*inertial_speed**2 - mu/(radius+altitude)
    e0 = 0.5*pad_speed**2 - mu/radius
    e1 = -mu/(2.0*(radius+target_altitude))
    return (e-e0)/(e1-e0)
