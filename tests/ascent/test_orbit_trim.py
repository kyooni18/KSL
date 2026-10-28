import unittest

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
