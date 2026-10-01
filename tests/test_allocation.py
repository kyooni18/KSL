import unittest
from shuttle.allocation import allocate


class AllocationTests(unittest.TestCase):
    def test_report_sea_level_example(self):
        c = -.4
        result = allocate([240909, 748388, 748388, 52817],
                          [s-c for s in [-3.80000793475, -.84000098525,
                                         .84000098525, -.00000059475]],
                          (-.00000321475-c)*2311112)
        self.assertTrue(result.feasible)
        for actual, expected in zip(result.throttles, [1, 1, .2186, 1]):
            self.assertAlmostEqual(actual, expected, delta=.0001)

    def test_infeasible_booster_moment(self):
        r = allocate([10, 20], [-1, 1], fixed_moment=100)
        self.assertEqual(r.throttles, (1, 0))
        self.assertFalse(r.feasible)
        self.assertEqual(r.residual, 90)

    def test_zero_thrust_and_zero_arms(self):
        r = allocate([0, 10, 10], [5, 0, 1])
        self.assertEqual(r.throttles[1:], (1, 0))
        self.assertTrue(r.feasible)

    def test_post_burnout_and_maximum(self):
        r = allocate([10, 20, 30], [-2, -1, 1])
        self.assertEqual(r.throttles, (0.5, 1, 1))
        self.assertEqual(r.thrust, 55)

    def test_thrust_cap_preserves_moment_balance(self):
        r = allocate([100, 100], [-1, 1], thrust_cap=80)
        self.assertTrue(r.feasible)
        self.assertAlmostEqual(r.thrust, 80)
        self.assertAlmostEqual(r.throttles[0], .4)
        self.assertAlmostEqual(r.throttles[1], .4)

    def test_group_upper_bound_limits_booster_throttle(self):
        r = allocate([100, 100], [-1, 1], upper_bounds=[.25, 1])
        self.assertTrue(r.feasible)
        self.assertAlmostEqual(r.throttles[0], .25)
        self.assertAlmostEqual(r.throttles[1], .25)
        self.assertAlmostEqual(r.thrust, 50)

    def test_efficiency_breaks_equal_thrust_ties(self):
        r = allocate([100, 100], [0, 0], thrust_cap=100,
                     mass_flows=[10, 1])
        self.assertTrue(r.feasible)
        self.assertAlmostEqual(r.thrust, 100)
        self.assertEqual(r.throttles, (0, 1))
        self.assertAlmostEqual(r.mass_flow, 1)

    def test_control_moment_tolerance_keeps_thrust_that_controls_can_trim(self):
        r = allocate([100, 100], [-2, 1], moment_tolerance=100)
        self.assertTrue(r.feasible)
        self.assertEqual(r.throttles, (1, 1))
        self.assertAlmostEqual(r.thrust, 200)
        self.assertAlmostEqual(r.residual, -100)

    def test_moment_tolerance_boundary_combines_with_thrust_cap(self):
        r = allocate([100, 100], [-2, 1], thrust_cap=180, moment_tolerance=80)
        self.assertTrue(r.feasible)
        self.assertAlmostEqual(r.thrust, 180)
        self.assertLessEqual(abs(r.residual), 80.000001)


if __name__ == '__main__':
    unittest.main()
