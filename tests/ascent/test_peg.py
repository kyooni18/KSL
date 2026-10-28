import math
from unittest.mock import patch

import pytest

from ksl.ascent.peg import InsertionGuidance, apsides
from ksl.ascent.geometry import inertial_velocity_components, smooth_value
from ksl.ascent.guidance import insertion_prediction_g, separation_pitch_transition

MU, R, G0 = 3.5316e12, 600000., 9.80665


def fly(h, vr, vh, mass, pitch, hold=34., ramp=15., thrust=2811e3, isp=315.,
        slew=2., lag=2., upper_g=1.4, dt=.2, pitch_response_seconds=None,
        pitch_bounds=(-30., 45.), follow_trajectory=False):
    """Point-mass truth plant: slew-limited command, lagged attitude, roll-hold limiter."""
    ve = isp*G0
    guide = InsertionGuidance(MU, R+90000., R+60000., R+72000., slew=slew, lag=lag,
                              pitch_response_seconds=pitch_response_seconds,
                              pitch_bounds=pitch_bounds, follow_trajectory=follow_trajectory)
    r, t, cmd, flown, peak = R+h, 0., pitch, pitch, 0.
    while t < 400.:
        cap = guide._throttle_cap(t, hold, ramp, .3)
        a_avail = thrust*cap/mass
        a = min(a_avail, upper_g*G0)
        ramp_left = ramp if t < hold else max(0., ramp-(t-hold))
        raw = guide.command(t, r, vr, vh, mass, thrust, ve, upper_g*G0, a, a < .99*a_avail,
                            cmd, max(0., hold-t), ramp_left, cap)
        if follow_trajectory:
            a = min(a, guide.planned_sample['thrust']/mass)
        rate = .5 if t < hold else slew
        if pitch_response_seconds is None:
            cmd += max(-rate*dt, min(rate*dt, raw-cmd))
        else:
            cmd = smooth_value(cmd, raw, pitch_response_seconds, dt)
        flown += (cmd-flown)*dt/lag
        p = math.radians(flown)
        vr += (a*math.sin(p)-MU/r**2+vh*vh/r)*dt
        vh += (a*math.cos(p)-vr*vh/r)*dt
        r += vr*dt
        mass -= a*mass/ve*dt
        t += dt
        ra, rp = apsides(MU, r, vr, vh)
        peak = max(peak, ra)
        if guide.done(r, vr, vh):
            break
    return ra-R, rp-R, peak-R


# Separation states of live runs peg-legacy-v2 and the lofted peg-legacy-v3,
# whose old guidance ran apoapsis to 114 km before pulling it back to 96 km.
@pytest.mark.parametrize('state', [
    (23116.8, 665.9, 939.4, 183593., 37.95),
    (24012.0, 779.3, 759.6, 184632., 46.18)])
def test_planned_insertion_reaches_target_without_apoapsis_overshoot(state):
    ap, pe, peak = fly(*state)
    assert 87000. <= ap <= 93000.
    assert pe >= 59700.
    assert peak <= 93000.


@pytest.mark.parametrize('follow_trajectory', [False, True])
def test_smooth_pitch_plan_reaches_recorded_separation_scenario(follow_trajectory):
    ap, pe, peak = fly(24012., 779.3, 759.6, 184632., 46.18,
                       pitch_response_seconds=5., pitch_bounds=(-90., 90.),
                       follow_trajectory=follow_trajectory)
    assert 87000. <= ap <= 93000.
    assert pe >= 59700.
    assert peak <= 93000.


def test_prediction_starts_from_measured_thrust_pitch():
    guide = InsertionGuidance(MU, R+90000., R+60000., R+72000.,
                              pitch_response_seconds=5., pitch_bounds=(-90., 90.))
    args = (R+74000., R+74000., 63., 2140., 112000., 2811000.,
            315.*G0, 1.4*G0, 0.)
    low = guide.fly(*args, dt=1., t_max=10., flown=-35.)
    high = guide.fly(*args, dt=1., t_max=10., flown=40.)
    assert high['peak_apoapsis_r'] > low['peak_apoapsis_r'] + 1000.
    with patch.object(guide, 'plan', wraps=guide.plan) as plan:
        guide.command(0., R+74000., 63., 2140., 112000., 2811000.,
                      315.*G0, 1.4*G0, 5., False, 0., flown_pitch=40.)
        assert plan.call_args.kwargs['flown_pitch'] == 40.


def test_insertion_forecast_preserves_recorded_reference_trajectory():
    guide = InsertionGuidance(MU, R+90000., R+60000., R+72000.,
                              pitch_response_seconds=5., pitch_bounds=(-90., 90.))
    result = guide.fly(R+74000., R+24012., 779.3, 759.6, 184632., 2811000.,
                       315.*G0, 1.4*G0, 46.18, hold_left=25., ramp=15.,
                       cap_now=.3, dt=2., t_max=300., flown=46.18,
                       attitude_hold=5., transition_anchor=46.18, record=True)
    assert result['time'] == 142.
    assert result['radius'] == pytest.approx(674038.2265752547, abs=1e-6)
    assert result['mass'] == pytest.approx(107613.71783463474, abs=1e-6)
    assert result['trajectory'][10]['commanded_pitch'] == pytest.approx(
        37.777787172359105, abs=1e-8)


def test_separation_attitude_hold_is_distinct_from_engine_limiter_hold():
    guide = InsertionGuidance(MU, R+90000., R+60000., R+72000.,
                              pitch_response_seconds=5., pitch_bounds=(-90., 90.))
    args = (R+74000., R+30000., 500., 1000., 180000., 2811000.,
            315.*G0, 1.4*G0, 45.)
    moving = guide.fly(*args, hold_left=25., dt=1., t_max=5., flown=45.)
    held = guide.fly(*args, hold_left=25., dt=1., t_max=5., flown=45., attitude_hold=5.)
    assert held['apoapsis_r'] != pytest.approx(moving['apoapsis_r'], abs=1.)


def test_separation_pitch_handoff_gradually_releases_insertion_command():
    assert separation_pitch_transition(78., -30., 5.) == 78.
    assert separation_pitch_transition(78., -30., 8.) == pytest.approx(74.976)
    assert separation_pitch_transition(78., -30., 35.) == -30.
    guide = InsertionGuidance(MU, R+90000., R+60000., R+72000.,
                              pitch_response_seconds=5., pitch_bounds=(-90., 90.))
    args = (R+74000., R+30000., 500., 1000., 180000., 2811000.,
            315.*G0, 1.4*G0, 45.)
    regular = guide.fly(*args, dt=1., t_max=15., flown=45.)
    blended = guide.fly(*args, dt=1., t_max=15., flown=45.,
                        transition_anchor=45., since_separation=5.)
    assert blended['apoapsis_r'] != pytest.approx(regular['apoapsis_r'], abs=1.)


def test_insertion_forecast_uses_governed_acceleration():
    assert insertion_prediction_g(1.4, .2, True) == .2
    assert insertion_prediction_g(1.4, .2, False) == 1.4
    assert insertion_prediction_g(1.4, 0., True) == 1.4
    assert insertion_prediction_g(1.4, 2., True) == 1.4


def test_inertial_velocity_decomposition_does_not_add_rotation_as_scalar_speed():
    radius, radial, horizontal = inertial_velocity_components(
        (600000., 0., 0.), (10., 100., 100.))
    assert radius == 600000.
    assert radial == 10.
    assert horizontal == pytest.approx(math.sqrt(20000.))
