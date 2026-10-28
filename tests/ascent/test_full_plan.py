from dataclasses import replace
from types import SimpleNamespace
import pytest

from ksl.ascent.full_plan import (BoosterState, attached_booster_dry_mass,
                                  engine_measurements, plan_ascent, predict_booster)


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
    # Seed a climb on the new 10 km corridor. The old 42 degree / 350 m/s
    # surface-horizontal seed is already shallow and is no longer a reachable
    # recovery case under this profile and its aerodynamic load bound.
    state = replace(state, horizontal_speed=300., surface_horizontal_speed=150.,
                    commanded_pitch=68., flown_pitch=66.)
    plan = plan_ascent(state, 3.5316e12, 600000., 90000., 60000., 72000.,
                       2811000., 315.*9.80665, 1.4*9.80665,
                       biases=(-6., 6.))
    assert plan['bias'] in (-6., 6.)
    assert plan['separation']['time'] == 35.
    assert plan['insertion']['time'] > 0.
    assert plan['insertion']['r_t'] >= 672000.


@pytest.mark.parametrize('radial_speed', [260., 300.])
def test_unreachable_forecasts_are_not_accepted(state, radial_speed):
    with pytest.raises(ValueError, match='No candidate reaches'):
        plan_ascent(replace(state, radial_speed=radial_speed), 3.5316e12, 600000., 90000., 60000., 72000.,
                    2811000., 315.*9.80665, 1.4*9.80665, biases=(-6., 6.))


def test_engine_measurements_follow_each_active_engine():
    engines = [SimpleNamespace(active=True, thrust=1000., max_thrust=2000., specific_impulse=200.),
               SimpleNamespace(active=True, thrust=3000., max_thrust=4000., specific_impulse=400.),
               SimpleNamespace(active=False)]
    thrust, flow, maximum, full_flow = engine_measurements(engines)
    assert thrust == 4000.
    assert maximum == 6000.
    assert flow == pytest.approx((1000./200.+3000./400.)/9.80665)
    assert full_flow == pytest.approx((2000./200.+4000./400.)/9.80665)
    engines[0].thrust = 200.
    assert engine_measurements(engines)[0] == 3200.


def test_booster_mass_skips_detached_parts_and_counts_each_part_once():
    class Part:
        def __init__(self, mass):
            self.dry_mass = mass
    a, b, detached = Part(20.), Part(30.), Part(999.)
    assert attached_booster_dry_mass([[a, detached], [b, a]], [a, b]) == 50.
    with pytest.raises(ValueError, match='Attached booster parts unavailable'):
        attached_booster_dry_mass([[detached]], [a, b])


def test_reseeded_forecast_responds_to_measured_thrust_and_aerodynamics(state):
    baseline = predict_booster(state, 3.5316e12, 600000., 90000., 0.)
    altered = replace(state, booster_thrust=state.booster_thrust*.8,
                      drag_acceleration=state.drag_acceleration*2., lift_radial=-2.)
    revised = predict_booster(altered, 3.5316e12, 600000., 90000., 0.)
    assert revised['radial_speed'] < baseline['radial_speed']
    assert revised['radius'] < baseline['radius']
