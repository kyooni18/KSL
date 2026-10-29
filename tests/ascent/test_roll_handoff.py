from ksl.ascent.guidance import (launch_roll_handoff_ready, meco_prepare_ready,
                                 progressive_turn_ready)


def test_recorded_launch_can_release_gravity_turn_without_waiting_for_exact_pitch():
    # Flight 20261028T023710Z at UT 155308: the previous one-degree
    # commanded-pitch gate excluded this otherwise settled sample.
    assert launch_roll_handoff_ready(79.9, 78.4, 78., 1.7, 0., 1.4)
    assert launch_roll_handoff_ready(78.2, 75.7, 78., 1.8, 0., 2.5)


def test_handoff_still_requires_controlled_roll_and_pointing():
    assert not launch_roll_handoff_ready(83., 78., 78., 1., 0., 1.)
    assert not launch_roll_handoff_ready(78., 68., 78., 1., 0., 1.)
    assert not launch_roll_handoff_ready(78., 78., 78., 8., 0., 1.)
    assert not launch_roll_handoff_ready(78., 78., 78., 1., 4., 1.)
    assert not launch_roll_handoff_ready(78., 78., 78., 1., 0., 9.)


def test_large_roll_error_does_not_release_turn():
    assert not launch_roll_handoff_ready(78.1, 73.6, 78., 37.2, 4.3, 4.8)


def test_progressive_turn_can_start_while_recorded_roll_finishes():
    # 20261029T000100Z at 3.4 km: pitch is near target and the remaining
    # 20-degree roll is controlled, although it has not yet settled.
    assert progressive_turn_ready(78.04, 71.91, 78., 20.31, -5.22, 6.62)
    assert not progressive_turn_ready(78.04, 71.91, 78., 46., -5.22, 6.62)
    assert not progressive_turn_ready(78.04, 71.91, 78., 20., -6.1, 6.62)
    assert not progressive_turn_ready(78.04, 71.91, 78., 20., -5., 8.1)


def test_meco_hold_requires_both_apsides_near_target():
    assert not meco_prepare_ready(89000., 52000., 90000., 60000.)
    assert not meco_prepare_ready(96000., 59000., 90000., 60000.)
    assert meco_prepare_ready(89000., 58000., 90000., 60000.)
    assert not meco_prepare_ready(96000., 59000., 90000., 60000., tolerance=5000.)
