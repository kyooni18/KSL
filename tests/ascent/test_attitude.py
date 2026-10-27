"""Topology changes must refresh dynamics before the next control step."""
import threading
import math
from types import SimpleNamespace
from unittest.mock import Mock
from ksl.ascent.attitude import AttitudeController
from ksl.ascent.vehicle import Shuttle
from ksl.ascent.geometry import direction_error_deg, slew_direction


def test_separation_invalidation_refreshes_dynamics_without_waiting_for_timer():
    controller = AttitudeController.__new__(AttitudeController)
    controller._dynamics_dirty = threading.Event()
    controller._last_slow = 100.
    controller._integral = [10., 20., 30.]
    controller._refresh_slow = Mock()
    controller._update_dynamics(100.01)
    controller._refresh_slow.assert_not_called()
    controller.invalidate_dynamics()
    controller._update_dynamics(100.02)
    controller._refresh_slow.assert_called_once()
    assert controller._integral == [0., 0., 0.]
    assert not controller._dynamics_dirty.is_set()

    # An event arriving during the RPC reads must survive for the next step.
    controller._refresh_slow.side_effect = controller.invalidate_dynamics
    controller.invalidate_dynamics()
    controller._update_dynamics(100.03)
    assert controller._dynamics_dirty.is_set()


def test_separation_hold_keeps_measured_nose_and_roll_for_five_simulation_seconds():
    controller = AttitudeController.__new__(AttitudeController)
    controller.lock = threading.Lock()
    controller._rotation = lambda: (0., 0., 0., 1.)
    now = [100.]
    controller._ut = lambda: now[0]
    controller.forward = (0., 1., 0.)
    controller.roof = (0., 0., -1.)
    controller._hold_until = None
    controller._held_attitude = None
    controller.hold_attitude(105.)
    controller.set_target((1., 0., 0.), (0., 1., 0.))
    for t in (100., 102., 104.999):
        now[0] = t
        assert controller._target_for_step() == ((0., 1., 0.), (0., 0., -1.))
    now[0] = 105.
    assert controller._target_for_step() == ((1., 0., 0.), (0., 1., 0.))
    assert controller._hold_until is None


def test_preseparation_capture_is_preserved_until_after_actual_staging():
    controller = AttitudeController.__new__(AttitudeController)
    controller.lock = threading.Lock()
    controller._rotation = lambda: (0., 0., 0., 1.)
    now = [96.]
    controller._ut = lambda: now[0]
    controller.forward = (1., 0., 0.)
    controller.roof = None
    controller._hold_until = None
    controller._held_attitude = None
    assert not controller.extend_attitude_hold(105.)
    controller.hold_attitude(float('inf'))
    controller._rotation = lambda: (0., 0., 1., 0.)
    now[0] = 100.
    assert controller.extend_attitude_hold(105.)
    assert controller._target_for_step() == ((0., 1., 0.), (0., 0., -1.))
    now[0] = 105.
    assert controller._target_for_step() == (controller.forward, None)


def test_capture_callback_runs_before_detachment():
    shuttle = Shuttle.__new__(Shuttle)
    shuttle.et_attached = False
    shuttle.separated = False
    shuttle.vehicle_kind = 'other'
    shuttle.solid_boosters = []
    shuttle._liquid_booster_depleted = lambda: True
    shuttle._solid_boosters_depleted = lambda: False
    events = []
    def separate():
        events.append('separate')
        shuttle.separated = True
    shuttle._separate_boosters = separate
    shuttle.groups = [[SimpleNamespace(thrust_limit=.5)]]
    shuttle.allocation = lambda **kwargs: SimpleNamespace(throttles=(.5,))
    shuttle.allocation_for_throttles = lambda values: values
    shuttle.update(before_booster_separation=lambda: events.append('capture'))
    assert events == ['capture', 'separate']


def test_thrust_correction_jump_is_limited_by_simulation_time():
    previous = (0., 1., 0.)
    target = (math.sin(math.radians(27.)), math.cos(math.radians(27.)), 0.)
    first = slew_direction(previous, target, 2., .1)
    assert abs(direction_error_deg(previous, first) - .2) < 1e-8
    assert direction_error_deg(slew_direction(first, target, 2., 20.), target) < 1e-6
    assert direction_error_deg(previous, slew_direction(previous, target, 2., 0.)) < 1e-6
    opposite = slew_direction(previous, (0., -1., 0.), 2., 1.)
    assert abs(direction_error_deg(previous, opposite) - 2.) < 1e-8
