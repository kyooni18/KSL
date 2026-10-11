"""Exercise cancellation and exceptional cleanup with actual child processes."""

import os
import sys
import threading
from unittest import mock

import pytest

from ksl.config import load_config
from ksl.controller import MissionController
from ksl.phases import Phase
from ksl.programs import Context, run_child


@pytest.mark.parametrize("close_stdout", [False, True])
def test_abort_reaps_child_even_after_output_closes(tmp_path, close_stdout):
    controller = MissionController(load_config(), runs_root=tmp_path)
    ctx = Context(controller.config, controller.log, "live", controller.stop_event,
                  controller._advance, controller._checkpoint)
    marker = tmp_path / "released"
    script = (
        "import os, signal, time\n"
        "signal.signal(signal.SIGINT, lambda *_: exit(0))\n"
        + ("os.close(1); os.close(2)\n" if close_stdout else "print('ready', flush=True)\n")
        + "try:\n while True: time.sleep(.01)\n"
        + f"finally:\n open({str(marker)!r}, 'w').write('released')\n"
    )
    timer = threading.Timer(0.4, controller.abort)
    timer.start()
    try:
        code, _ = run_child(ctx, "test", [sys.executable, "-u", "-c", script], None, lambda _: None)
        assert code == 0
        assert marker.read_text() == "released"
    finally:
        timer.cancel()
        controller.log.close()


def test_callback_failure_releases_child_and_records_failed_mission(tmp_path):
    controller = MissionController(load_config(), runs_root=tmp_path, stop=Phase.ORBIT)
    marker = tmp_path / "released"
    pidfile = tmp_path / "pid"
    script = (
        "import os, signal, time\n"
        "signal.signal(signal.SIGINT, lambda *_: exit(0))\n"
        f"open({str(pidfile)!r}, 'w').write(str(os.getpid()))\n"
        "try:\n print('@@ksl {\"event\": \"ignition\"}', flush=True)\n"
        " while True: time.sleep(.01)\n"
        f"finally:\n open({str(marker)!r}, 'w').write('released')\n"
    )
    with mock.patch("ksl.programs.AscentProgram._pad_prep"), \
            mock.patch("ksl.programs.AscentProgram._argv", return_value=[sys.executable, "-u", "-c", script]), \
            mock.patch.object(controller, "_advance", side_effect=ValueError("invalid telemetry")):
        status = controller.run()
    assert status.state == "failed"
    assert "invalid telemetry" in status.error
    assert marker.read_text() == "released"
    with pytest.raises(ProcessLookupError):
        os.kill(int(pidfile.read_text()), 0)


def test_abort_before_run_never_starts_a_program(tmp_path):
    controller = MissionController(load_config(), runs_root=tmp_path)
    controller.abort()
    with mock.patch("ksl.programs.AscentProgram.run") as run:
        assert controller.run().state == "aborted"
        run.assert_not_called()


def test_exited_parent_cannot_leave_descendant_holding_output_open(tmp_path):
    controller = MissionController(load_config(), runs_root=tmp_path)
    ctx = Context(controller.config, controller.log, "live", controller.stop_event,
                  controller._advance, controller._checkpoint)
    script = "import subprocess,sys; subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'])"
    try:
        with pytest.raises(Exception, match="descendant kept its output pipe open"):
            run_child(ctx, "test", [sys.executable, "-c", script], None, lambda _: None)
    finally:
        controller.log.close()
