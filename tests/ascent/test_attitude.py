"""Topology changes must refresh dynamics before the next control step."""
import threading
from unittest.mock import Mock
from ksl.ascent.attitude import AttitudeController


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
