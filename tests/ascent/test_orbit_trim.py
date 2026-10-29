import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from ksl.ascent import orbit_trim
import pytest


def test_oms_throttle_ramps_and_never_exceeds_the_gentle_cap():
    throttle = 0.0
    for _ in range(100):
        previous = throttle
        throttle = orbit_trim.oms_throttle_command(throttle, 20., 4.,
                                                  orbit_trim.OMS_THROTTLE_CAP, .1)
        assert abs(throttle-previous) <= orbit_trim.OMS_THROTTLE_SLEW*.1 + 1e-12
        assert throttle <= .2
    assert throttle == pytest.approx(.2)
    assert orbit_trim.oms_throttle_command(.2, .1, 4., .2, 1.) == pytest.approx(.15)


def test_oms_burn_timing_uses_reduced_throttle_and_ramp():
    duration, centroid = orbit_trim.oms_burn_profile(20., 20000., 80000., 300., .2)
    full_throttle_time = 20. / 4.
    assert duration > 5*full_throttle_time
    assert 0. < centroid < duration


def test_oms_burn_cuts_and_stops_when_attitude_slips(monkeypatch):
    class Clock:
        value = 0.

        @property
        def ut(self):
            self.value += .2
            return self.value

    class Controls:
        rcs = False
        _throttle = 0.
        max_throttle = 0.

        @property
        def throttle(self):
            return self._throttle

        @throttle.setter
        def throttle(self, value):
            self._throttle = value
            self.max_throttle = max(self.max_throttle, value)

    ctl = Controls()

    class Autopilot:
        engaged = False
        ctrl = SimpleNamespace()

        @property
        def error(self):
            return 30. if ctl.throttle > 0. else 1.

    ap = Autopilot()
    engine = SimpleNamespace(active=True, available_thrust=80000., specific_impulse=300.)
    body = SimpleNamespace(non_rotating_reference_frame='inertial', gravitational_parameter=1.)
    vessel = SimpleNamespace(control=ctl, parts=SimpleNamespace(engines=[engine]),
                             available_thrust=80000., mass=20000.,
                             orbit=SimpleNamespace(body=body),
                             resources=SimpleNamespace(amount=lambda _: 100.),
                             velocity=lambda _: (0., 0., 0.),
                             position=lambda _: (600000., 0., 0.))
    node = SimpleNamespace(delta_v=20., ut=0., time_to=0., reference_frame='node',
                           remaining_delta_v=20., remove=Mock(),
                           direction=lambda _: (0., 1., 0.),
                           remaining_burn_vector=lambda _: (0., 20., 0.))
    conn = SimpleNamespace(space_center=Clock())
    events = []
    monkeypatch.setitem(orbit_trim.CUSTOM_AP, 'ap', ap)
    monkeypatch.setattr(orbit_trim, 'aim', lambda *args: None)
    monkeypatch.setattr(orbit_trim, 'align_for_burn', lambda *args: None)
    monkeypatch.setattr(orbit_trim, 'oms_propellant_mass', lambda _: 1000.)
    monkeypatch.setattr(orbit_trim.time, 'sleep', lambda _: None)
    monkeypatch.setattr(orbit_trim, 'emit_event', lambda name, **data: events.append((name, data)))

    with pytest.raises(RuntimeError, match='OMS burn stopped: attitude error 30.0'):
        orbit_trim.execute(conn, vessel, node, lambda _: False, orbit_cutoff=True)

    assert ctl.throttle == 0.
    assert ctl.max_throttle > 0.
    assert not ap.engaged
    node.remove.assert_called_once()
    assert any(name == 'trim-burn-cutoff' and data['attitude_error_deg'] == 30.
               for name, data in events)


class RcsBudgetTests(unittest.TestCase):
    def test_rcs_needs_monopropellant_margin_and_respects_configured_limit(self):
        self.assertFalse(orbit_trim.use_rcs_forward(1.0, rcs_available=1.2))
        self.assertTrue(orbit_trim.use_rcs_forward(4.0, rcs_available=10.0, limit=5.0))
        self.assertFalse(orbit_trim.use_rcs_forward(4.0, rcs_available=10.0, limit=2.0))


class RcsTrimSelectionTests(unittest.TestCase):
    def test_slight_corrections_use_rcs_forward_translation(self):
        self.assertTrue(orbit_trim.use_rcs_forward(0.2))
        self.assertTrue(orbit_trim.use_rcs_forward(orbit_trim.RCS_TRIM_MAX_DV))

    def test_larger_corrections_use_the_oms(self):
        self.assertFalse(orbit_trim.use_rcs_forward(orbit_trim.RCS_TRIM_MAX_DV + 0.1))
        self.assertFalse(orbit_trim.use_rcs_forward(25.0))


if __name__ == "__main__":
    unittest.main()


def test_rescue_periapsis_target_respects_burn_altitude():
    # A descending 93 km pass cannot be circularized to 100 km with prograde
    # thrust; stage to an achievable, atmosphere-safe 88 km first.
    assert orbit_trim.recoverable_periapsis_target(100000, 93000) == 88000
    assert orbit_trim.recoverable_periapsis_target(100000, 108000) == 100000
    assert orbit_trim.recoverable_periapsis_target(100000, 102000) == 97000

def test_followup_oms_circularization_can_approach_safe_apoapsis():
    # Fourth live polar flight: first OMS burn left Ap 100089/Pe 95055 m.
    # The old universal 5km margin limited the next node to Pe 95089 m and
    # aborted planning before any second burn. A true apsis allows Pe 99989.
    margin = orbit_trim.periapsis_burn_margin(95055, 100089, 100089)
    assert margin == 100.0
    assert orbit_trim.recoverable_periapsis_target(100000, 100089, margin) == 99989
    assert 99989 > 95055 + 300


def test_oms_rescue_keeps_five_km_margin_when_unsafe_or_not_at_apsis():
    assert orbit_trim.periapsis_burn_margin(69410, 100089, 100089) == 5000
    assert orbit_trim.periapsis_burn_margin(95055, 93000, 100089) == 5000
