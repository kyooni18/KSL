from ksl.ascent.guidance import launch_roll_handoff_ready, progressive_turn_ready


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


def test_gravity_turn_can_progress_while_roll_finishes():
    # 20261028T034252Z at 2 km: pitch had reached the requested 12-degree
    # pitch-over, but waiting for exact roll caused further pointing divergence.
    assert progressive_turn_ready(78.1, 73.6, 78., 37.2, 4.3, 4.8)
    assert not progressive_turn_ready(84., 78., 78., 37., 4., 4.)
    assert not progressive_turn_ready(78., 78., 78., 60., 4., 4.)
    assert not progressive_turn_ready(78., 78., 78., 37., 7., 4.)
