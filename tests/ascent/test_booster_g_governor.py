"""The 3 G governor may reduce thrust only with modeled gimbal reserve."""
from types import SimpleNamespace

import pytest

from ksl.ascent.vehicle import G0, guarded_ssme_g_target


def engine(thrust, actuation=(.2, .1, .1), gimbal_range=10.):
    return SimpleNamespace(
        active=True, has_fuel=True, gimballed=True, max_thrust=thrust,
        thrust=thrust, gimbal_range=gimbal_range, gimbal_limit=1.,
        gimbal_actuation=actuation,
        part=SimpleNamespace(position=lambda _: (0., 10., 1.),
                             direction=lambda _: (0., 1., 0.)))


def test_g_governor_reduces_only_when_predicted_gimbal_has_one_degree_reserve():
    vessel = SimpleNamespace(mass=100000., reference_frame=object())
    core, booster = engine(1e6), engine(4e6)
    target, status = guarded_ssme_g_target(vessel, [core], [booster],
                                           .8, .9, 3.3, -1e6)
    assert status == 'limited'
    assert target == pytest.approx(.8 - .3 * vessel.mass * G0 / core.max_thrust)
    core.gimbal_actuation = (.95, 0., 0.)
    assert guarded_ssme_g_target(vessel, [core], [booster], .8, .9, 3.3, -1e6) == (.9, 'gimbal-margin')
    core.gimbal_actuation = (.2, .1, .1)
    core.gimbal_range = 1.
    assert guarded_ssme_g_target(vessel, [core], [booster], .8, .9, 3.3, -1e6) == (.9, 'gimbal-margin')


def test_g_governor_reports_boosters_alone_above_limit_and_missing_telemetry():
    vessel = SimpleNamespace(mass=100000., reference_frame=object())
    core, booster = engine(1e6), engine(4e6)
    assert guarded_ssme_g_target(vessel, [core], [booster], .8, .9, 4.2, -1e6) == (0., 'boosters-exceed-limit')
    del core.gimbal_range
    assert guarded_ssme_g_target(vessel, [core], [booster], .8, .9, 3.3, -1e6) == (.9, 'unavailable')
