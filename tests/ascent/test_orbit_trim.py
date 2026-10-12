import unittest

from ksl.ascent import orbit_trim


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
