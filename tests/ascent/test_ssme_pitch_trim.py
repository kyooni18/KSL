"""Throttle balancing must use the available core thrust and measured effort."""
from types import SimpleNamespace as NS
import pytest
from ksl.ascent.vehicle import Shuttle, guarded_ssme_g_target, pitch_trim_feedback, G0


def test_gimbal_feedback_only_integrates_when_pointing_is_settled():
    assert pitch_trim_feedback(0., -.5, 1e6, 10., 1., .2) == pytest.approx(.05)
    assert pitch_trim_feedback(0., -.5, -1e6, 10., 1., .2) == pytest.approx(-.05)
    assert pitch_trim_feedback(.1, -.5, 1e6, 10., 4., .2) == .1
    assert pitch_trim_feedback(.1, -.5, 1e6, 10., 1., 2.) == .1
    assert pitch_trim_feedback(.19, -1., 1e6, 10., 1., 0.) == .2


def test_trim_accounts_for_master_throttle():
    frame = object()
    core = [NS(thrust_limit=.3, max_thrust=1e6, thrust=150000., has_fuel=True,
               part=NS(position=lambda _: (0., 0., 1.),
                       direction=lambda _: (0., 1., 0.)))]
    booster = NS(active=True, has_fuel=True, thrust=300000.,
                 part=NS(position=lambda _: (0., 0., -1.),
                         direction=lambda _: (0., 1., 0.)))
    shuttle = Shuttle.__new__(Shuttle)
    shuttle.et_attached, shuttle.separated = True, False
    shuttle.vehicle_kind, shuttle.solid_boosters = 'orbiter-shuttle', [booster]
    shuttle.groups = shuttle.core_groups = [core]
    shuttle.vessel = NS(reference_frame=frame)
    shuttle.top_off = lambda: None
    shuttle._liquid_booster_depleted = lambda: False
    shuttle._solid_boosters_depleted = lambda: False
    shuttle.allocation = lambda **kwargs: NS(throttles=(0.,))
    shuttle.allocation_for_throttles = lambda values: values
    # 300 kN m from the booster needs 60% limiter at 50% master throttle.
    assert shuttle.update(dt=1., throttle_slew=1., master_throttle=.5)[0] == pytest.approx(.6)
    assert shuttle.pitch_moment == pytest.approx(150000.)

    # Persistent negative pitch-gimbal effort biases the limiter in the
    # direction that relieves that effort; a maneuver must not integrate it.
    for e in core + [booster]:
        e.active, e.gimballed, e.gimbal_actuation = True, True, (-.8, 0., 0.)
    # Here increasing the limiter adds negative x torque, so the correction
    # has the opposite sign from a positive moment arm.
    before = shuttle.update(dt=1., throttle_slew=1., master_throttle=.5,
                            pointing_error=1., pitch_rate_deg_s=.1)[0]
    assert before < .6
    bias = shuttle._pitch_trim_bias
    shuttle.update(dt=1., throttle_slew=1., master_throttle=.5,
                   pointing_error=8., pitch_rate_deg_s=.1)
    assert shuttle._pitch_trim_bias == bias


def test_three_g_governor_accounts_for_master_throttle():
    def engine(thrust):
        return NS(active=True, has_fuel=True, gimballed=True, max_thrust=thrust,
                  thrust=thrust, gimbal_range=10., gimbal_limit=1.,
                  gimbal_actuation=(0., 0., 0.),
                  part=NS(position=lambda _: (0., 10., 1.),
                          direction=lambda _: (0., 1., 0.)))
    core, booster = engine(1e6), engine(4e6)
    vessel = NS(mass=100000., reference_frame=object())
    result, status = guarded_ssme_g_target(vessel, [core], [booster], .8, .9,
                                            3.05, -5e5, master_throttle=.5)
    assert status == 'limited'
    assert result == pytest.approx(.8-.05*vessel.mass*G0/(core.max_thrust*.5))
