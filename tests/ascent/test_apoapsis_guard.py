import pytest

from ksl.ascent.apoapsis_guard import apoapsis_guard_g, projected_apoapsis


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
