import pytest
import json

from ksl.ascent.apoapsis_guard import (apoapsis_guard_g, projected_apoapsis,
                                       recovery_pitch, recovery_guidance, reentry_before_meco)


MU = 3.5316e12
R = 600000.0


def test_guard_forecasts_smooth_pitch_response_for_lofted_post_separation_state():
    # UT 155423, about 32 km altitude: 761 m/s radial speed and a
    # steep 54.5-degree nose, with inertial horizontal speed about 780 m/s.
    g, peak, feasible = apoapsis_guard_g(
        MU, R+32300., 761., 780., 54.5, 5., R+90000., 1.4)
    assert g == pytest.approx(1.4)
    assert feasible
    assert peak <= R+89000.
    assert projected_apoapsis(MU, R+32300., 761., 780., 54.5, 5., 1.4*9.80665) <= R+89000.


def test_guard_keeps_minimum_guided_thrust_when_overshoot_is_unavoidable():
    g, peak, feasible = apoapsis_guard_g(
        MU, R+57000., 643., 897., 51.5, 5., R+90000., 1.4)
    assert g == pytest.approx(.15)
    assert peak > R+89000.
    assert not feasible


def test_hold_delays_the_pitch_down_and_zero_response_time_is_rejected():
    free = projected_apoapsis(MU, R+32300., 761., 780., 54.5, 5., 3.)
    held = projected_apoapsis(MU, R+32300., 761., 780., 54.5, 5., 3., hold_seconds=5.)
    assert held > free
    with pytest.raises(ValueError):
        apoapsis_guard_g(MU, R+32300., 761., 780., 54.5, 0., R+90000., 1.4)


def test_unqualified_lofted_insertion_does_not_command_upward_thrust():
    # Recorded 20261029T000625Z: at 57 km the unqualified plan demanded
    # roughly 86 degrees while climbing at 625 m/s toward an 85 km apoapsis.
    forecast = dict(mu=MU, radius=R+57000., radial_speed=625., horizontal_speed=616.,
                    acceleration=.15*9.80665, thrust_pitch=63.34, response_seconds=5.,
                    atmosphere_radius=R+70000., trim_radius=R+74000., horizon=100.,
                    target_apoapsis_radius=R+85000., target_periapsis_radius=R+55000.)
    assert recovery_pitch(85.7, False, False, 84043., 85000., forecast) == (0., 'horizontal')
    assert recovery_pitch(-20., True, False, 84043., 85000., forecast) == (-20., 'horizontal')
    # A qualified plan without a guard, a low-energy climb, or descent is not
    # replaced by this recovery policy.
    assert recovery_pitch(45., False, True, 84043., 85000., forecast) == (45., 'inactive')
    assert recovery_pitch(45., False, False, 60000., 85000., forecast) == (45., 'inactive')


def test_recovery_allows_descent_but_predicts_return_to_atmosphere_and_trim_clearance():
    forecast = dict(mu=MU, radius=R+75000., radial_speed=-100., horizontal_speed=1200.,
                    acceleration=12., thrust_pitch=0., response_seconds=5.,
                    atmosphere_radius=R+70000., trim_radius=R+74000., horizon=120.,
                    target_apoapsis_radius=R+85000., target_periapsis_radius=R+55000.)
    assert reentry_before_meco(target_pitch=0., **forecast)
    pitch, status = recovery_pitch(60., True, False, 84000., 85000., forecast)
    assert status == 'climb-for-clearance'
    assert 0. < pitch < 60.
    assert not reentry_before_meco(target_pitch=pitch, **forecast)
    # A brief descent can be acceptable: after cutoff the coast reaches the
    # OMS altitude without ever returning to the atmosphere before MECO.
    safe = dict(forecast, radius=R+80000., radial_speed=-10., horizontal_speed=2200.,
                acceleration=0., horizon=10.)
    assert not reentry_before_meco(target_pitch=0., **safe)
    assert recovery_pitch(60., True, False, 84000., 85000., safe) == (0., 'horizontal')
    # If cutoff is still below 74 km, a rising coast can reach the OMS
    # altitude; the same speed downward means that opportunity has passed.
    coast = dict(safe, radius=R+73000., radial_speed=200., horizon=0.)
    assert not reentry_before_meco(target_pitch=0., **coast)
    assert reentry_before_meco(target_pitch=0., **dict(coast, radial_speed=-200.))
    # Being above 74 km at cutoff is not enough if the shuttle will reenter
    # before the 30-second OMS alignment lead has elapsed.
    falling = dict(safe, radius=R+75000., radial_speed=-200., horizon=0.,
                   trim_coast_seconds=30.)
    assert reentry_before_meco(target_pitch=0., **falling)


def test_recovery_warns_if_no_pitch_at_current_thrust_reaches_trim():
    forecast = dict(mu=MU, radius=R+57000., radial_speed=625., horizontal_speed=616.,
                    acceleration=.15*9.80665, thrust_pitch=63., response_seconds=5.,
                    atmosphere_radius=R+70000., trim_radius=R+74000., horizon=500.,
                    target_apoapsis_radius=R+85000., target_periapsis_radius=R+55000.)
    assert recovery_pitch(85., True, False, 84000., 85000., forecast) == (0., 'unavoidable')
    pitch, status, acceleration = recovery_guidance(85., True, False, 84000., 85000.,
                                                    forecast, 1.4*9.80665)
    assert pitch == 0.
    assert status == 'horizontal-accelerated'
    assert forecast['acceleration'] < acceleration <= 1.4*9.80665
    assert not reentry_before_meco(target_pitch=0., **dict(forecast, acceleration=acceleration))


def test_recovery_does_not_override_thrust_if_no_available_acceleration_can_clear():
    forecast = dict(mu=MU, radius=R+71000., radial_speed=-500., horizontal_speed=800.,
                    acceleration=1., thrust_pitch=0., response_seconds=5.,
                    atmosphere_radius=R+70000., trim_radius=R+74000., horizon=100.,
                    target_apoapsis_radius=R+85000., target_periapsis_radius=R+55000.)
    pitch, status, acceleration = recovery_guidance(80., True, False, 84000., 85000.,
                                                    forecast, 1.4*9.80665)
    assert (pitch, status, acceleration) == (0., 'unavoidable', None)


def test_nonfinite_plan_diagnostics_do_not_break_json_serialization():
    from ksl.ascent.flight import finite_json
    plan = {'apoapsis_r': float('inf'), 'trajectory': [{'raw_pitch': float('nan'), 'time': 2.}]}
    assert json.loads(json.dumps(finite_json(plan), allow_nan=False)) == {
        'apoapsis_r': None, 'trajectory': [{'raw_pitch': None, 'time': 2.}]}
