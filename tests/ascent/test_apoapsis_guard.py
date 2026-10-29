import pytest
import json

from ksl.ascent.apoapsis_guard import (apoapsis_guard_g, projected_apoapsis,
                                       recovery_pitch, reentry_before_meco)


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
    forecast = dict(mu=MU, radius=R+73142., radial_speed=-41., horizontal_speed=1533.,
                    acceleration=8.04, thrust_pitch=0., response_seconds=5.,
                    atmosphere_radius=R+70000., trim_radius=R+74000., horizon=57.5,
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
    coast = dict(safe, radius=R+73000., radial_speed=120., horizon=0.)
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
    # The former recovery acceleration would clear the atmosphere only by
    # overshooting the target; it is not a viable insertion.
    assert reentry_before_meco(target_pitch=0., **dict(forecast, acceleration=1.4*9.80665))


def test_guard_can_cut_thrust_when_recorded_overshoot_is_already_unavoidable():
    g, peak, feasible = apoapsis_guard_g(
        MU, R+80700., -216., 2066., 0., 5., R+90000., 1.168, floor_g=0.)
    assert g == 0.
    assert peak > R+90000.
    assert not feasible


def test_receding_guard_does_not_starve_the_recorded_near_orbit_state():
    # UT 168290: the long fixed-thrust forecast held this insertion at 0.067 G.
    args = (MU, R+84630.95, -13.69, 2242.5, -.02, 5., R+90000., 1.4)
    long_g, _, _ = apoapsis_guard_g(*args, floor_g=0.)
    short_g, peak, feasible = apoapsis_guard_g(*args, floor_g=0., horizon_seconds=8.)
    assert long_g < .08
    assert short_g > .3
    assert feasible and peak <= R+89000.
    # An already overshot trajectory remains guarded, even on a short horizon.
    g, _, feasible = apoapsis_guard_g(MU, R+80700., -216., 2066., 0., 5.,
                                      R+90000., 1.168, floor_g=0., horizon_seconds=8.)
    assert g == 0. and not feasible


def test_insertion_planner_failure_cuts_throttle_and_propagates_error():
    from types import SimpleNamespace
    from ksl.ascent.flight import insertion_command_or_cut_throttle
    class Guide:
        def command(self):
            raise ValueError('No fuel-feasible insertion trajectory')
    control = SimpleNamespace(throttle=1.)
    with pytest.raises(ValueError, match='No fuel-feasible'):
        insertion_command_or_cut_throttle(Guide(), control)
    assert control.throttle == 0.


def test_nonfinite_plan_diagnostics_do_not_break_json_serialization():
    from ksl.ascent.flight import finite_json
    plan = {'apoapsis_r': float('inf'), 'trajectory': [{'raw_pitch': float('nan'), 'time': 2.}]}
    assert json.loads(json.dumps(finite_json(plan), allow_nan=False)) == {
        'apoapsis_r': None, 'trajectory': [{'raw_pitch': None, 'time': 2.}]}
