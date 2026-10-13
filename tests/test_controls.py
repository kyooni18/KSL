from types import SimpleNamespace

from ksl.controls import Controls, warp_until


def test_triggers_fire_once_per_press_and_stale_presses_are_ignored(tmp_path):
    tui = Controls.for_run(tmp_path)
    tui.trigger("trimNow")                      # pressed before the program started
    program = Controls(tmp_path / "controls.json")
    assert not program.take("trimNow")
    tui.trigger("trimNow")
    assert program.take("trimNow") and not program.take("trimNow")
    assert tui.toggle("autoWarp") is False and not program.auto_warp


def test_warp_stays_at_1x_without_auto_warp_until_warp_now(tmp_path):
    controls = Controls.for_run(tmp_path, {"autoWarp": False})
    sc = SimpleNamespace(ut=0.0, rails_warp_factor=0)
    factors = []

    def tick(_):
        factors.append(sc.rails_warp_factor)
        if len(factors) == 3:
            Controls(tmp_path / "controls.json").trigger("warpNow")
        sc.ut += 100.0 * (1 + 999 * (sc.rails_warp_factor > 0))

    warp_until(sc, 50000.0, controls, sleep=tick)
    assert factors[:3] == [0, 0, 0] and max(factors) > 0 and sc.rails_warp_factor == 0
