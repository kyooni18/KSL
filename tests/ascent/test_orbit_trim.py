"""Safety behavior for OMS attitude reacquisition."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from ksl.ascent.orbit_trim import align_for_burn


class _Autopilot:
    def __init__(self, errors):
        self.errors = iter(errors)

    @property
    def error(self):
        return next(self.errors)


class OrbitTrimAlignmentTests(unittest.TestCase):
    def test_rcs_assists_stalled_alignment_then_turns_off(self):
        vessel = SimpleNamespace(control=SimpleNamespace(rcs=False))
        seen = []
        with (patch('ksl.ascent.orbit_trim.time.monotonic', side_effect=[0, 1, 7]),
              patch('ksl.ascent.orbit_trim.time.sleep'),
              patch('ksl.ascent.orbit_trim.aim',
                    side_effect=lambda *args: seen.append(vessel.control.rcs))):
            align_for_burn(None, vessel, [], _Autopilot([12, 12, 2]), None,
                           (0, 1, 0), 3)
        self.assertEqual(seen, [False, True])
        self.assertFalse(vessel.control.rcs)

    def test_alignment_timeout_aborts_with_rcs_off(self):
        vessel = SimpleNamespace(control=SimpleNamespace(rcs=False))
        with (patch('ksl.ascent.orbit_trim.time.monotonic', side_effect=[0, 61]),
              self.assertRaisesRegex(RuntimeError, 'did not align')):
            align_for_burn(None, vessel, [], _Autopilot([12]), None,
                           (0, 1, 0), 3, timeout=60)
        self.assertFalse(vessel.control.rcs)


if __name__ == '__main__':
    unittest.main()
