import math

import pytest

from ksl.ascent.guidance import (TURN_PROFILE, constrain_turn_pitch,
                                scheduled_turn_pitch, turn_reference_pitch)
from ksl.ascent.trajectory import corridor_pitch


def test_turn_reference_is_progressive_and_smooth_at_waypoints():
    for altitude, pitch in TURN_PROFILE:
        assert turn_reference_pitch(altitude, 90000.) == pytest.approx(pitch)
        left = (pitch-turn_reference_pitch(altitude-.1, 90000.))/.1
        right = (turn_reference_pitch(altitude+.1, 90000.)-pitch)/.1
        assert left == pytest.approx(right, abs=1e-6)
    values = [turn_reference_pitch(h, 90000.) for h in range(0, 70001, 100)]
    assert all(a >= b for a, b in zip(values, values[1:]))
    assert all(turn_reference_pitch(h, 90000.) > 45. for h in range(0, 27001, 100))
    assert turn_reference_pitch(5000., 45000.) == pytest.approx(68.)


def test_early_corrections_cannot_erase_profile_but_aerodynamic_loads_take_precedence():
    # A healthy 68 degree flight path at 10 km must not be driven shallow by
    # either the apoapsis corridor or the continuous planner's downward bias.
    prograde = 68.
    vertical, horizontal = 400.*math.sin(math.radians(prograde)), 400.*math.cos(math.radians(prograde))
    for q in (0., 25000., 42000.):
        base = scheduled_turn_pitch(10000., 90000., vertical, horizontal, q)
        assert base == pytest.approx(68.)
        corrected = corridor_pitch(base, prograde, 10000., 90000., 90000., q)
        pitch = constrain_turn_pitch(corrected-10., 10000., 90000., prograde, q)
        assert 65. <= pitch <= 68.
    # An already shallow recorded trajectory cannot jump to 68 degrees at max-Q.
    # It may recover upward within the load envelope, instead of the old +1 cap.
    assert constrain_turn_pitch(68., 10000., 90000., 35., 25000.) == pytest.approx(37.)
    assert constrain_turn_pitch(68., 10000., 90000., 35., 0.) == pytest.approx(55.)
