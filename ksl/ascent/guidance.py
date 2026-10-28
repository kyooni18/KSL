"""Ascent guidance laws: pitch/heading profiles, G and q throttle feedback, MECO and PEG terminal guidance.

Pure functions of measured state; ksl.ascent.flight applies them through kRPC.
"""
import math

from .geometry import _combine, _cross, _dot, _norm, _unit, smoothstep01

ROLL_RESET_GRACE = 60.0


LAUNCH_TARGET_G = 1.08
THROTTLE_SLEW_PER_SECOND = 0.25
PITCH_SLEW_DEG_PER_SECOND = 0.4
ASCENT_PITCH_RESPONSE_SECONDS = 5.0
MAX_DYNAMIC_PRESSURE_PA = 25000.0
SEPARATION_ATTITUDE_HOLD_SECONDS = 5.0


def attitude_settled(nose_error, roll_error, angular_velocity,
                     angle_limit=3.0, rate_limit_deg_s=0.5):
    """Require all-axis rate damping as well as nose and roll alignment."""
    return (abs(nose_error) <= angle_limit and abs(roll_error) <= angle_limit and
            math.degrees(math.sqrt(sum(x*x for x in angular_velocity))) <= rate_limit_deg_s)


def separation_thrust_blend(now, separation_ut, roll_start=None, roll_done=None):
    """Bring in cant compensation continuously across separation and upright roll."""
    blend = min(1., max(0., (now-separation_ut-SEPARATION_ATTITUDE_HOLD_SECONDS)/4.))
    if roll_done is not None:
        return blend*min(1., max(0., (now-roll_done)/6.))
    if roll_start is not None:
        return blend*max(0., 1.-(now-roll_start)/4.)
    return blend


def maneuver_roll_rate(error_deg, duration_s, limit_deg_s):
    """Bound the maneuver schedule by the operator's physical roll-rate cap."""
    return min(limit_deg_s, max(3.0, min(30.0, 1.4*abs(error_deg)/duration_s)))


def ascent_target_g(altitude, atmosphere_depth, max_g, launch_g=LAUNCH_TARGET_G,
                    minimum_g=0.0):
    """Smooth ascent target with an optional hard lower bound."""
    if atmosphere_depth <= 0:
        target = min(max_g, launch_g)
    else:
        blend = smoothstep01(max(0.0, altitude) / atmosphere_depth)
        target = min(max_g, launch_g + (max_g-launch_g)*blend)
    return min(max_g, max(minimum_g, target))


def meco_ready(apoapsis_altitude, periapsis_altitude, target_apoapsis, minimum_periapsis):
    """Normal MECO requires both the target apoapsis and a near-orbital periapsis."""
    return (apoapsis_altitude >= target_apoapsis and
            periapsis_altitude >= minimum_periapsis)


def mission_roll_allowed(liftoff, altitude_gain, roll_start_gain,
                         surface_speed, minimum_speed=40.0):
    """Wait for both pad clearance and sufficient surface speed before rolling."""
    return (liftoff and altitude_gain >= roll_start_gain and
            math.isfinite(surface_speed) and surface_speed >= minimum_speed)


def manual_roll_command(error_deg, rate_deg_s, accel_deg_s2, rate_limit_deg_s,
                        control_limit=1.0, response_time=.20):
    """Rate-damped direct roll with explicit overspeed braking."""
    if accel_deg_s2 <= 0 or rate_limit_deg_s <= 0 or control_limit <= 0 or response_time <= 0:
        return 0.0
    desired_rate = max(-rate_limit_deg_s, min(rate_limit_deg_s, error_deg * .5))
    if abs(rate_deg_s) > rate_limit_deg_s * 1.25:
        return -math.copysign(control_limit, rate_deg_s)
    rate_error = desired_rate - rate_deg_s
    command = rate_error / (accel_deg_s2 * response_time)
    return max(-control_limit, min(control_limit, command))


def roll_attitude_settled(roll_error, roll_rate, nose_error,
                          angle_limit=2.0, rate_limit=.75, nose_limit=3.0):
    return (abs(roll_error) <= angle_limit and abs(roll_rate) <= rate_limit and
            abs(nose_error) <= nose_limit)


def launch_roll_handoff_ready(commanded_pitch, actual_pitch, target_pitch,
                              roll_error, roll_rate, nose_error):
    """Release the turn near the pitch target once roll and pointing are controlled."""
    return (abs(commanded_pitch-target_pitch) <= 2.5 and
            abs(actual_pitch-target_pitch) <= 8.0 and
            roll_attitude_settled(roll_error, roll_rate, nose_error,
                                  angle_limit=7.0, rate_limit=3.0, nose_limit=8.0))


def progressive_turn_ready(commanded_pitch, actual_pitch, target_pitch,
                           roll_error, roll_rate, nose_error):
    """Allow the smooth turn to begin while the roll loop is still correcting."""
    return (abs(commanded_pitch-target_pitch) <= 2.5 and
            abs(actual_pitch-target_pitch) <= 8.0 and
            abs(roll_error) <= 45.0 and abs(roll_rate) <= 6.0 and
            abs(nose_error) <= 8.0)

def hold_turn_for_roll(roll_active, roll_reset):
    """Keep launch pitch fixed for the entire dedicated roll phase."""
    return roll_active and not roll_reset


def ascent_pitch(altitude, turn_start_altitude, turn_end_altitude,
                 initial_pitch=89.0):
    progress = min(1, max(0, (altitude-turn_start_altitude) /
                          (turn_end_altitude-turn_start_altitude)))
    eased = progress * progress * (3 - 2 * progress)
    return max(5, initial_pitch*(1-eased))


def gravity_turn_pitch(vertical_speed, horizontal_speed, dynamic_pressure,
                       initial_pitch=89.0, minimum_pitch=5.0):
    """Follow surface prograde with a bounded pitch-over bias.

    The bias is larger at low dynamic pressure to establish horizontal velocity,
    then shrinks through max-Q so the stack stays close to prograde.
    """
    vertical = max(0.0, vertical_speed)
    horizontal = max(0.5, horizontal_speed)
    flight_path_pitch = math.degrees(math.atan2(vertical, horizontal))
    q = max(0.0, dynamic_pressure)
    if q <= 5000:
        pitch_under = 20.0
    elif q < 25000:
        pitch_under = 20.0 - 12.0 * (q - 5000.0) / 20000.0
    else:
        pitch_under = 8.0
    return min(initial_pitch, max(minimum_pitch, flight_path_pitch - pitch_under))


def vertical_profile_pitch(altitude, vertical_speed, inertial_horizontal_speed,
                           radius, mu, accel, insertion_altitude, insertion_speed,
                           min_pitch=-20.0, max_pitch=35.0, insertion_vertical_speed=0.0):
    """Pitch that brings the climb to the insertion altitude with ~zero vertical speed
    exactly when horizontal speed reaches the insertion speed.

    Linear-in-time vertical acceleration a(t) = a0 + a1 t meeting h(tgo) = target and
    v(tgo) = 0 gives a0 = (6 dh - 4 v tgo)/tgo^2. Thrust supplies a0 plus the
    effective gravity (gravity minus centrifugal), so the loft and the gravity loss
    are planned instead of being burned off later by pitching nose-down.
    """
    accel = max(accel, 0.5)
    g_eff = max(0.0, mu/(radius*radius) - inertial_horizontal_speed**2/radius)
    dh = insertion_altitude - altitude
    tgo = max(20.0, (insertion_speed - inertial_horizontal_speed)/accel)
    # a(t) = a0 + a1 t with v(tgo) = insertion_vertical_speed and h(tgo) = target.
    a0 = (6.0*(dh - vertical_speed*tgo) -
          2.0*tgo*(insertion_vertical_speed - vertical_speed))/(tgo*tgo)
    s = max(-1.0, min(1.0, (a0 + g_eff)/accel))
    return max(min_pitch, min(max_pitch, math.degrees(math.asin(s))))


def final_phase_g(periapsis_altitude, target_periapsis, max_g, ramp=30000.0, floor=0.5):
    """Taper acceleration as periapsis nears target so cutoff resolution is fine.

    Near orbital speed 1 m/s of delta-v moves apoapsis by roughly 1.2 km, so the
    last part of the burn is flown at low G to keep cutoff timing error small.
    """
    remaining = max(0.0, target_periapsis-periapsis_altitude)
    if remaining >= ramp:
        return max_g
    return max(floor, floor + (max_g-floor)*remaining/ramp)


GUIDANCE_DEBUG = {}


def terminal_guidance(position, velocity, rotation_velocity, mu, body_radius,
                      apoapsis_altitude, periapsis_altitude, inclination_deg,
                      acceleration, plane_normal=None):
    """Velocity-to-be-gained guidance to an exact apoapsis/periapsis/inclination.

    All vectors are in a non-rotating body frame whose +y axis points to the north
    pole. Returns (thrust direction, remaining delta-v magnitude, time-to-go).
    The target velocity at the current radius is the point on the target orbit
    (ascending branch) in the plane of the requested inclination, so apoapsis,
    periapsis and inclination are reached together, not one at a time.
    """
    r = _norm(position)
    rh = _unit(position)
    vr = _dot(velocity, rh)
    vh = _combine((1.0, velocity), (-vr, rh))
    rp = body_radius + periapsis_altitude
    ra = body_radius + apoapsis_altitude
    h_target = math.sqrt(2.0*mu*ra*rp/(ra+rp))
    vh_target = h_target/r
    energy = -mu/(ra+rp)
    vr_target = math.sqrt(max(0.0, 2.0*(energy+mu/r) - vh_target**2))
    north = _unit(_combine((1.0, (0.0, 1.0, 0.0)), (-rh[1], rh)))
    east = _unit(rotation_velocity)
    east = _unit(_combine((1.0, east), (-_dot(east, rh), rh)))
    sin_lat = max(-1.0, min(1.0, rh[1]))
    cos_lat = math.sqrt(1.0 - sin_lat*sin_lat)
    cos_i = math.cos(math.radians(inclination_deg))
    azimuth = math.asin(max(-1.0, min(1.0, cos_i/max(cos_lat, 1e-6))))
    horizontal = _combine((math.cos(azimuth), north), (math.sin(azimuth), east))
    if plane_normal is not None:
        # Steer into an exact plane (inclination *and* node), e.g. a rendezvous
        # target's. h x r is the prograde horizontal in any consistent algebra.
        in_plane = _unit(_cross(plane_normal, rh))
        if _norm(in_plane) > 0.0:
            horizontal = in_plane
    v_target = _combine((vr_target, rh), (vh_target, horizontal))
    dv = _combine((1.0, v_target), (-1.0, velocity))
    dv_mag = _norm(dv)
    cross = _unit(_cross(rh, horizontal))
    GUIDANCE_DEBUG.update(radial=_dot(dv, rh), along=_dot(dv, horizontal),
                          cross=_dot(dv, cross), vr_target=vr_target, vh_target=vh_target,
                          vr=vr, vh=_norm(vh))
    time_to_go = dv_mag/max(acceleration, 0.5)
    g_eff = mu/(r*r) - _dot(vh, vh)/r
    command = _combine((1.0/max(time_to_go, 0.5), dv), (g_eff, rh))
    return _unit(command), dv_mag, time_to_go


def aoa_limit_deg(dynamic_pressure, q_alpha=50000.0, floor=1.5, ceiling=20.0):
    """Bound angle of attack so q*alpha (wing/tank-attach load) stays within q_alpha Pa*deg."""
    return max(floor, min(ceiling, q_alpha/max(1.0, dynamic_pressure)))


def scheduled_turn_pitch(altitude, target_apoapsis, vertical_speed, horizontal_speed,
                         dynamic_pressure=0.0, initial_pitch=89.0, minimum_pitch=5.0):
    """Drag- and load-aware gravity turn scaled to the target orbit.

    The reference passes through 45 deg at 20% of the target apoapsis altitude
    (15 km for a 75 km orbit) and flattens near the target altitude. The nose
    follows the surface velocity vector, i.e. a zero-lift gravity turn, and may
    lead it only by an angle of attack allowed by a q*alpha bound, so the stack
    turns fastest in thin air and nearly zero-lift through max-Q.
    """
    reference = 1.05 * max(1000.0, target_apoapsis)
    exponent = math.log(0.5) / math.log(0.2 / 1.05)
    fraction = min(1.0, max(0.0, altitude) / reference) ** exponent
    scheduled = initial_pitch * (1.0 - fraction)
    x = max(0.0, altitude) / reference
    if x > 0.2:
        # Past the 45-degree point, flatten faster so the climb is spent before the
        # terminal burn instead of arriving with a large vertical speed to cancel.
        scheduled = min(scheduled, 45.0 * max(0.0, 1.0 - (x-0.2)/0.3) ** 1.2)
    prograde = math.degrees(math.atan2(max(0.0, vertical_speed),
                                       max(0.5, horizontal_speed)))
    alpha = aoa_limit_deg(dynamic_pressure)
    pitch = min(prograde + 1.0, max(scheduled, prograde - alpha))
    return min(initial_pitch, max(minimum_pitch, pitch))


def attitude_error_exceeded(error, altitude, since_roll_reset=None):
    """Preserve the attitude guard while allowing the commanded half-roll."""
    return (error > 45 and altitude > 1000 and
            (since_roll_reset is None or since_roll_reset >= ROLL_RESET_GRACE))


def g_force_throttle(current_throttle, actual_g, max_g, recovery_step=.02):
    """Limit observed G without disturbing relative engine throttle allocation."""
    current = min(1.0, max(0.0, current_throttle))
    if actual_g > max_g and actual_g > 0:
        return max(0.0, min(current, current * max_g / actual_g))
    if actual_g < max_g - .1:
        return min(1.0, current + recovery_step)
    return current


def post_booster_master_throttle(actual_g, max_g, dynamic_pressure, max_q):
    """Hard launch envelope only: core engine limiters already set target G.

    Always start from a full master command so a previous reduced/zero master
    throttle cannot permanently disable an otherwise healthy powered ascent.
    """
    return dynamic_pressure_throttle(
        g_force_throttle(1.0, actual_g, max_g),
        dynamic_pressure, max_q)


def dynamic_pressure_throttle(current_throttle, dynamic_pressure, max_dynamic_pressure):
    """Reduce liquid throttle when measured aerodynamic pressure exceeds its cap."""
    if max_dynamic_pressure <= 0 or dynamic_pressure <= max_dynamic_pressure:
        return current_throttle
    ratio = max_dynamic_pressure / max(dynamic_pressure, 1e-9)
    return current_throttle * math.sqrt(ratio)




def earthward_roll_target(liftoff, booster_separation_ut, current_ut, reset_delay):
    """Return whether the orbiter side should face Kerbin, plus delayed-flip state."""
    if not liftoff:
        return None, False
    flipped = (booster_separation_ut is not None and
               current_ut - booster_separation_ut >= reset_delay)
    return (not flipped), flipped

