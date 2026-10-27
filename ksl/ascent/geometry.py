"""Frame, vector and roll geometry for ascent attitude guidance (pure functions)."""
import math


def smoothstep01(value):
    x = min(1.0, max(0.0, value))
    return x*x*(3.0-2.0*x)


def slew_value(current, target, rate_per_second, dt):
    if dt <= 0 or rate_per_second <= 0:
        return current
    step = rate_per_second*dt
    return current + max(-step, min(step, target-current))


def slew_vector(current, target, rate_per_second, dt):
    """Move all actuator channels with one interpolation factor."""
    if len(current) != len(target):
        raise ValueError('Current and target vectors must have equal length')
    if dt <= 0 or rate_per_second <= 0:
        return tuple(current)
    delta = [t-c for c, t in zip(current, target)]
    largest = max((abs(x) for x in delta), default=0.0)
    if largest == 0:
        return tuple(target)
    alpha = min(1.0, rate_per_second*dt/largest)
    return tuple(c + alpha*d for c, d in zip(current, delta))


def roll_for_heading(heading):
    """Map launch heading to body roll; 90 degrees maps to 180 degrees."""
    roll = (2 * heading) % 360
    return roll if roll <= 180 else roll - 360


def direction_for_pitch_heading(pitch, heading):
    """Convert pitch/heading to a direction vector in the surface frame."""
    pitch, heading = math.radians(pitch), math.radians(heading)
    return (math.sin(pitch), math.cos(pitch) * math.cos(heading),
            math.cos(pitch) * math.sin(heading))


def pitch_heading_for_direction(direction):
    """Convert a surface-frame direction to pitch and compass heading."""
    x, y, z = direction
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, x))))
    heading = math.degrees(math.atan2(z, y)) % 360
    return pitch, heading


def surface_axis_errors(direction, target_pitch, target_heading):
    """Pitch/heading errors from geometry, independent of AutoPilot engagement state."""
    actual_pitch, actual_heading = pitch_heading_for_direction(direction)
    pitch_error = abs(target_pitch-actual_pitch)
    heading_error = abs(wrap_roll(target_heading-actual_heading))
    return pitch_error, heading_error


def direction_error_deg(actual, target):
    """Angular nose error independent of roll and AutoPilot.error quirks."""
    an = math.sqrt(sum(x*x for x in actual))
    tn = math.sqrt(sum(x*x for x in target))
    if an <= 0 or tn <= 0:
        return 180.0
    dot = sum(a*b for a, b in zip(actual, target)) / (an*tn)
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def slew_direction(current, target, rate_deg_s, dt):
    """Move a unit direction along the shortest great-circle arc at a bounded rate."""
    a, b = _unit(current), _unit(target)
    if _norm(a) < 1e-12 or _norm(b) < 1e-12:
        raise ValueError('Direction must be nonzero')
    angle = math.acos(max(-1.0, min(1.0, _dot(a, b))))
    step = math.radians(max(0.0, rate_deg_s)*max(0.0, dt))
    if angle <= step or angle < 1e-10:
        return b
    tangent = tuple(b[i]-math.cos(angle)*a[i] for i in range(3))
    if _norm(tangent) < 1e-10:
        basis = min(((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)),
                    key=lambda v: abs(_dot(a, v)))
        tangent = _cross(a, basis)
    tangent = _unit(tangent)
    return tuple(math.cos(step)*a[i] + math.sin(step)*tangent[i] for i in range(3))


def surface_up_for_pitch_heading(pitch, heading):
    """Return a unit surface-up vector perpendicular to the launch direction."""
    pitch, heading = math.radians(pitch), math.radians(heading)
    return (math.cos(pitch), -math.sin(pitch)*math.cos(heading),
            -math.sin(pitch)*math.sin(heading))


def dot3(a, b):
    return sum(x*y for x, y in zip(a, b))


def cross3(a, b):
    return (a[1]*b[2]-a[2]*b[1],
            a[2]*b[0]-a[0]*b[2],
            a[0]*b[1]-a[1]*b[0])


def normalize3(v):
    length = math.sqrt(dot3(v, v))
    if length <= 1e-12:
        raise ValueError('Cannot normalize a zero-length vector')
    return tuple(x/length for x in v)


def project_perpendicular(v, axis):
    axis = normalize3(axis)
    return tuple(x-axis[i]*dot3(v, axis) for i, x in enumerate(v))


def desired_orbiter_side(pitch, heading, earthward=True):
    """Desired ET->orbiter side axis in the surface frame.

    The orbiter is mounted on vessel -Z. Heads-down means that side axis follows
    local-down projected into the plane normal to the commanded flight direction.
    """
    projected_up = surface_up_for_pitch_heading(pitch, heading)
    return tuple(-x for x in projected_up) if earthward else projected_up


def roll_error_to_side(direction, actual_side, target_side):
    """Signed roll error from the real orbiter-side vector, in control.roll sign."""
    axis = normalize3(direction)
    current = normalize3(project_perpendicular(actual_side, axis))
    target = normalize3(project_perpendicular(target_side, axis))
    signed_rh = math.degrees(math.atan2(dot3(axis, cross3(current, target)),
                                        dot3(current, target)))
    # KSP/kRPC roll-control sign is opposite the right-hand rotation about +forward.
    return wrap_roll(-signed_rh)


def wrap_roll(angle):
    wrapped = (angle + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 and angle > 0 else wrapped


def roll_for_roof(direction, up_reference, roof_direction):
    """Return the AutoPilot roll command that preserves the current roof axis."""
    def dot(a, b):
        return sum(x*y for x, y in zip(a, b))

    def cross(a, b):
        return (a[1]*b[2]-a[2]*b[1],
                a[2]*b[0]-a[0]*b[2],
                a[0]*b[1]-a[1]*b[0])

    def normalize(v):
        length = math.sqrt(dot(v, v))
        if length <= 1e-12:
            raise ValueError('Cannot define roll from a zero-length vector')
        return tuple(x/length for x in v)

    axis = normalize(direction)
    up = tuple(u-axis[i]*dot(up_reference, axis) for i, u in enumerate(up_reference))
    roof = tuple(r-axis[i]*dot(roof_direction, axis) for i, r in enumerate(roof_direction))
    up = normalize(up)
    roof = normalize(roof)
    signed = math.degrees(math.atan2(dot(axis, cross(up, roof)), dot(up, roof)))
    # kRPC's positive roll convention is opposite the right-hand signed angle above.
    return wrap_roll(-signed)


def roll_delta(current, target):
    """Shortest signed angular distance from current roll to target roll."""
    return (target-current+180.0) % 360.0 - 180.0


def slew_roll(current, target, max_delta):
    """Move toward target by the shortest roll path, with a per-step rate limit."""
    if max_delta < 0:
        raise ValueError('Roll slew limit must be nonnegative')
    delta = roll_delta(current, target)
    if abs(delta) <= max_delta:
        return target
    return wrap_roll(current + math.copysign(max_delta, delta))


def _dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


def _norm(a):
    return math.sqrt(_dot(a, a))


def _unit(a):
    n = _norm(a)
    return (a[0]/n, a[1]/n, a[2]/n) if n > 0 else (0.0, 0.0, 0.0)


def _cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def _combine(*terms):
    return tuple(sum(c*v[i] for c, v in terms) for i in range(3))

