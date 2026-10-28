from unittest.mock import Mock
from types import SimpleNamespace as NS
import threading

import pytest

from ksl.programs import wait_simulation_seconds
from ksl.ascent.attitude import AttitudeController
from ksl.ascent.intelligent.flight import Propulsion


def test_pad_settle_uses_simulation_time_and_ignores_pause():
    values = iter([100., 100., 100., 105., 109.9, 110.])
    stop = Mock()
    stop.is_set.return_value = False
    stop.wait.return_value = False
    wait_simulation_seconds(lambda: next(values), stop, 10.)
    assert stop.wait.call_count == 4


def test_pad_settle_restarts_after_clock_rewind_and_can_be_stopped():
    values = iter([100., 105., 20., 29., 30.])
    stop = Mock()
    stop.is_set.return_value = False
    stop.wait.return_value = False
    wait_simulation_seconds(lambda: next(values), stop, 10.)
    assert stop.wait.call_count == 3
    stop.wait.return_value = True
    values = iter([0., 0.])
    with pytest.raises(InterruptedError):
        wait_simulation_seconds(lambda: next(values), stop, 10.)


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
