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


def test_booster_forecast_obeys_pitch_slew(state):
    result = predict_booster(state, 3.5316e12, 600000., 90000., 0.,
                             record=True, pitch_slew=.4)
    for a, b in zip(result['trajectory'], result['trajectory'][1:]):
        assert abs(b['commanded_pitch'] - a['commanded_pitch']) <= .4 * (b['time'] - a['time']) + 1e-9


def test_booster_forecast_tracks_recorded_reference_and_empty_burn(state):
    result = predict_booster(state, 3.5316e12, 600000., 90000., -6.,
                             end_bias=6., record=True)
    sample = result['trajectory'][20]
    assert sample['radius'] == pytest.approx(617185.9882994395, abs=1e-6)
    assert sample['radial_speed'] == pytest.approx(426.0230435953513, abs=1e-8)
    assert sample['commanded_pitch'] == pytest.approx(38.488459128752744, abs=1e-8)
    assert result['radius'] == pytest.approx(624528.0041481444, abs=1e-6)
    empty = predict_booster(replace(state, booster_time=0.), 3.5316e12,
                            600000., 90000., 0., record=True)
    assert len(empty['trajectory']) == 1
    assert empty['time'] == 0.
    assert empty['mass'] == state.mass - state.booster_dry_mass


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


@pytest.mark.parametrize('radial_speed', [300.])
def test_unreachable_forecasts_are_not_accepted(state, radial_speed):
    with pytest.raises(ValueError, match='No candidate reaches'):
        plan_ascent(replace(state, radial_speed=radial_speed), 3.5316e12, 600000., 90000., 60000., 72000.,
                    2811000., 315.*9.80665, 1.4*9.80665, biases=(-6., 6.),
                    acceleration_fractions=(1.,))


def test_thrust_search_builds_timed_fuel_feasible_handoff(state):
    # Verify the selected acceleration still yields a fuel-feasible target orbit.
    plan = plan_ascent(replace(state, radial_speed=260.), 3.5316e12, 600000.,
                       90000., 60000., 72000., 2811000., 315.*9.80665, 1.4*9.80665,
                       biases=(-6., 6.), start_ut=1000., core_propellant=110000.)
    assert any(plan['upper_acceleration'] == pytest.approx(f*1.4*9.80665)
               for f in (.8, 1.))
    sep, cutoff = plan['milestones']['separation'], plan['milestones']['meco']
    assert sep['ut'] == 1035.
    assert cutoff['ut'] == 1035.+plan['insertion']['time']
    assert 87000. <= cutoff['apoapsis'] <= 93000.
    assert cutoff['periapsis'] >= 59700.
    assert plan['core_propellant_used'] <= 110000.
    assert cutoff['mass'] == pytest.approx(
        state.mass-state.booster_dry_mass-state.booster_mass_flow*35.-plan['core_propellant_used'])
    trajectory = plan['insertion']['trajectory']
    for a, b in zip(trajectory, trajectory[1:]):
        assert b['mass'] == pytest.approx(a['mass']-a['mass_flow']*(b['time']-a['time']))
        assert a['thrust'] <= 2811000.
    with pytest.raises(ValueError, match='Insufficient core propellant'):
        plan_ascent(state, 3.5316e12, 600000., 90000., 60000., 72000.,
                    2811000., 315.*9.80665, 1.4*9.80665, core_propellant=100.)
    with pytest.raises(ValueError, match='No candidate reaches'):
        plan_ascent(state, 3.5316e12, 600000., 90000., 60000., 72000.,
                    2811000., 315.*9.80665, 1.4*9.80665,
                    biases=(-6., 6.), core_propellant=20000.)
    assert any(m['action'] == 'pitch-up' for m in plan['maneuvers']['insertion'])


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
