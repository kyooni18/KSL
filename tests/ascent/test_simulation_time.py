from unittest.mock import Mock, MagicMock
from types import SimpleNamespace as NS
import threading

import pytest

from ksl.ascent.attitude import AttitudeController
from ksl.ascent.intelligent.flight import Propulsion
from ksl.programs import AscentProgram


@pytest.mark.parametrize('mode', ['legacy', 'intelligent'])
def test_pad_preparation_does_not_wait_for_simulation_time(monkeypatch, mode):
    engine = NS(part=NS(title='Clydesdale'), thrust_limit=0.)
    conn = NS(space_center=NS(active_vessel=NS(parts=NS(engines=[engine]))))
    connection = MagicMock()
    connection.__enter__.return_value = conn
    connect = Mock(return_value=connection)
    monkeypatch.setattr('ksl.programs.connect', connect)
    monkeypatch.setattr('ksl.programs.wait_for_flight_scene', Mock())
    ctx = NS(config={'ascent': {'revertToLaunch': False, 'guidanceMode': mode,
                                'srbThrustLimit': .9, 'padSettleSeconds': 100.}},
             stop_event=threading.Event(), log=Mock())
    AscentProgram()._pad_prep(ctx)
    assert engine.thrust_limit == (.9 if mode == 'legacy' else 0.)
    assert connect.call_count == (1 if mode == 'legacy' else 0)


def test_attitude_loop_uses_full_ut_intervals_and_refreshes_after_rewind(monkeypatch):
    controller = AttitudeController.__new__(AttitudeController)
    controller._running = controller.engaged = True
    controller.forward = (0., 1., 0.)
    controller.dt = .04
    controller._last_step_ut = None
    controller._last_slow = 100.
    controller._dynamics_dirty = threading.Event()
    controller._refresh_slow = Mock()
    controller._integral = [1., 2., 3.]
    samples = iter([100., 100., 100.01, 103., 20., 20.1])
    controller._ut = lambda: next(samples)
    intervals = []
    def step(dt):
        intervals.append(dt)
        if len(intervals) == 6:
            controller._running = False
    controller._step = step
    monkeypatch.setattr('ksl.ascent.attitude.time.sleep', lambda _: None)
    controller._run()
    assert intervals == pytest.approx([0., 0., .01, 2.99, 0., .1])
    assert controller._integral == [0., 0., 0.]
    assert controller._refresh_slow.call_count == 2


def test_intelligent_separation_dwell_uses_ut_and_restarts_on_rewind():
    part = object()
    separator = Mock()
    model = NS(vessel=NS(parts=NS(all=[part])),
               components=[NS(key='booster', separation_stage=1)],
               parts={'booster': part}, separators={'booster': separator},
               approved=set(), exhausted_branch=lambda _: True,
               usable_engines=lambda: {})
    sc = NS(ut=100.)
    propulsion = Propulsion(model, sc)
    for ut in (100., 100., 100.01, 101., 20., 20., 21.99):
        sc.ut = ut
        propulsion.update()
        separator.decouple.assert_not_called()
    sc.ut = 22.
    propulsion.update()
    separator.decouple.assert_called_once()
