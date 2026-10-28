from ksl.ascent.guidance import launch_roll_handoff_ready


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
