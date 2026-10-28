import pytest

from ksl.ascent.full_plan import BoosterState, plan_ascent, predict_booster


@pytest.fixture
def state():
    return BoosterState(610000., 300., 500., 350., 300000., 45., 42.,
                        900000., 300., 5800000., 2300., 35., 20000., 12000., 2.)


def test_forecast_propagates_propellant_and_drops_boosters(state):
    result = predict_booster(state, 3.5316e12, 600000., 90000., 0.)
    assert result['time'] == 35.
    assert result['radius'] > state.radius
    assert result['mass'] == pytest.approx(189000.)
    assert result['radial_speed'] > 0.


def test_full_plan_returns_stage_and_cutoff_forecasts(state):
    plan = plan_ascent(state, 3.5316e12, 600000., 90000., 60000., 72000.,
                       2811000., 315.*9.80665, 1.4*9.80665,
                       biases=(-6., 6.))
    assert plan['bias'] in (-6., 6.)
    assert plan['separation']['time'] == 35.
    assert plan['insertion']['time'] > 0.
    assert plan['insertion']['r_t'] >= 672000.
