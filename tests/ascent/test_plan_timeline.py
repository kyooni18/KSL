import pytest

from ksl.ascent.plan_timeline import sample_trajectory, track_trajectory


def test_ut_sampling_pause_rewind_and_tracking_do_not_advance_the_plan():
    a = dict(time=0., radius=610000., radial_speed=300., horizontal_speed=500.,
             mass=300000., raw_pitch=68., thrust=6000000.)
    b = dict(a, time=10., radius=613500., radial_speed=400., mass=280000., raw_pitch=65.)
    timeline = [a, b]
    for elapsed in (0., .01, 4.7, 4.7, 9.9, 3., 10.):
        sample = sample_trajectory(timeline, elapsed)
        assert sample['raw_pitch'] == pytest.approx(68.-.3*elapsed)
        assert track_trajectory(sample, sample['radius'], sample['radial_speed'],
                                sample['horizontal_speed'], 30.) == pytest.approx(sample['raw_pitch'])
    assert track_trajectory(a, a['radius']-100., 290., 500., 30.) > a['raw_pitch']
    for outside in (-.01, 10.01):
        with pytest.raises(ValueError, match='outside'):
            sample_trajectory(timeline, outside)
