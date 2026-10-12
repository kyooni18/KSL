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
