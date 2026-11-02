"""Focused regressions for recorded-video clock fidelity (no flight simulation)."""
from pathlib import Path
import time
import unicodedata

import numpy as np

from Tools.landing_video import recording_start_epoch
from Tools.mission_video import Timeline


def test_macos_decomposed_korean_recording_name_uses_start_time():
    name = unicodedata.normalize('NFD', '화면 기록 2026-11-02 오후 1.17.37.mov')
    assert recording_start_epoch(Path('/not-a-real-recording') / name) == time.mktime(
        (2026, 11, 2, 13, 17, 37, 0, 0, -1))


def test_clock_uses_simulation_time_and_marks_unobserved_handoff():
    timeline = Timeline.__new__(Timeline)
    timeline.liftoff_ut = 1000.0
    timeline.liftoff_wall = 100.0
    timeline.clock_t = np.array([100., 101., 102., 162.])
    timeline.clock_u = np.array([1000., 1001., 1051., 1651.])
    assert timeline.clock(101.5) == (26.0, False)  # observed 50x warp
    assert timeline.clock(132.) == (351.0, True)  # sparse logger handoff
