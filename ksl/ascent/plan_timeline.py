"""Simulation-time trajectory sampling and bounded tracking, independent of kRPC."""
import bisect
import math


def pitch_maneuvers(trajectory, start_ut):
    """Summarize planned pitch-up, pitch-down and hold intervals for review."""
    result = []
    for a, b in zip(trajectory, trajectory[1:]):
        delta = b['commanded_pitch']-a['commanded_pitch']
        direction = 'hold' if abs(delta) < .01 else 'pitch-up' if delta > 0 else 'pitch-down'
        if not result or result[-1]['action'] != direction:
            result.append(dict(action=direction, start_ut=start_ut+a['time'],
                               start_pitch=a['commanded_pitch']))
        result[-1].update(end_ut=start_ut+b['time'], end_pitch=b['commanded_pitch'])
    return result


def sample_trajectory(trajectory, elapsed):
    """Interpolate a single-stage plan; never extrapolate past its horizon."""
    if not trajectory or not math.isfinite(elapsed):
        raise ValueError('Missing trajectory or invalid simulation time')
    if elapsed < trajectory[0]['time'] or elapsed > trajectory[-1]['time']:
        raise ValueError('Simulation time lies outside the planned horizon')
    i = bisect.bisect_right([p['time'] for p in trajectory], elapsed)-1
    a = trajectory[i]
    if i == len(trajectory)-1:
        return dict(a)
    b = trajectory[i+1]
    fraction = (elapsed-a['time'])/(b['time']-a['time'])
    return {key: value + fraction*(b[key]-value) for key, value in a.items()}


def track_trajectory(sample, radius, radial_speed, horizontal_speed, horizon):
    """Feedforward pitch with small position/velocity error correction.

    A nominal state produces exactly the planned command. Feedback corrects
    deviations from that state, not deviations from an arbitrary vertical speed.
    Large errors require replanning; tracking cannot invent a new trajectory.
    """
    tau = max(15., min(40., horizon))
    angle = math.radians(sample['raw_pitch'])
    accel = max(.1, sample['thrust']/sample['mass'])
    radial = (accel*math.sin(angle) + 6*(sample['radius']-radius)/tau**2 +
              4*(sample['radial_speed']-radial_speed)/tau)
    horizontal = (accel*math.cos(angle) +
                  2*(sample['horizontal_speed']-horizontal_speed)/tau)
    correction = math.degrees(math.atan2(radial, horizontal))-sample['raw_pitch']
    correction = (correction+180.) % 360.-180.
    return sample['raw_pitch'] + max(-8., min(8., correction))
