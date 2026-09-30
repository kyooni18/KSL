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


if __name__ == '__main__':
    unittest.main()
