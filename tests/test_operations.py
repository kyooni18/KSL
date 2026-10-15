"""Important operation boundaries and explicit plan engagement (no KSP)."""
import json
import time
import asyncio
from pathlib import Path
from unittest import mock

import pytest

from ksl.commands import Commands
from ksl.config import load_config
from ksl.operations import BY_KEY, OperationRequest, blocked_reason, resume
from ksl.ownership import OwnershipError, claim, owner
from ksl.plan_review import invalid_reason

FAKE = Path(__file__).parent / "fakes" / "fake_backend.py"


def vessel():
    return dict(scene="flight", situation="orbiting", observedAt=time.time(), vessel="STS",
                vesselId=123, body="Kerbin", atmosphereDepth=70000, ut=100,
                apoapsis=85000, periapsis=85000, altitude=85000, inclination=10,
                mass=50000, semiMajorAxis=685000, eccentricity=0,
                longitudeOfAscendingNode=0, argumentOfPeriapsis=0)


def wait_for(predicate):
    deadline = time.monotonic() + 5
    while not predicate():
        assert time.monotonic() < deadline, "Operation did not reach expected state"
        time.sleep(0.02)


def test_directed_launch_and_explicit_orbit_are_independent(tmp_path):
    config = load_config(overrides=["ascent.guidanceMode=intelligent", "orbit.trim=false"])
    launch = OperationRequest("launch", config).build(tmp_path)
    assert [p.name for p, _ in launch.plan()] == ["ascent"]
    launch.run()
    automatic = OperationRequest("launch", config, automatic=True).build(tmp_path)
    assert [p.name for p, _ in automatic.plan()] == ["ascent", "orbit-trim", "landing"]
    automatic.run()
    orbit = OperationRequest("orbit", config).build(tmp_path)
    assert orbit.config["orbit"]["trim"] and not orbit.config["orbit"]["holdForTrigger"]
    orbit.run()
    assert resume(orbit.run_dir)[1].value == "deorbit"


def test_directed_launch_records_an_orbit_checkpoint_without_running_trim(tmp_path):
    import sys
    from ksl import programs
    request = OperationRequest("launch", load_config(), "live")
    with mock.patch("ksl.ownership.DATA_ROOT", tmp_path), \
         mock.patch("ksl.krpc_link.probe", return_value={**vessel(), "situation": "pre_launch"}), \
         mock.patch.object(programs.AscentProgram, "_pad_prep"), \
         mock.patch.object(programs.AscentProgram, "_argv", return_value=[sys.executable, str(FAKE.with_name("fake_ascent.py"))]), \
         mock.patch("ksl.krpc_link.save_game") as save:
        controller = request.build(tmp_path)
        assert controller.run().state == "holding"
        assert controller.status.phase == "orbit"
        save.assert_called_once()
    config, start, checkpoint = resume(controller.run_dir)
    assert start.value == "deorbit"
    assert checkpoint["kspSave"]
    assert not (controller.run_dir / "orbit-trim.log").exists()


def test_context_and_live_ownership_are_checked(tmp_path):
    assert blocked_reason(BY_KEY["launch"], vessel(), "live")
    assert blocked_reason(BY_KEY["deorbit"], vessel(), "live") is None
    assert blocked_reason(BY_KEY["deorbit"], {**vessel(), "periapsis": 55000}, "live")
    with mock.patch("ksl.ownership.DATA_ROOT", tmp_path):
        config = load_config()
        with claim(config, tmp_path / "owner"):
            assert owner(config)["run"].endswith("owner")
            with pytest.raises(OwnershipError):
                with claim(config, tmp_path / "other"):
                    pytest.fail("Two controllers claimed the same endpoint")
            controller = OperationRequest("deorbit", config, "live").build(tmp_path)
            with mock.patch("ksl.krpc_link.probe") as probe:
                assert controller.run().state == "failed"
                probe.assert_not_called()
        assert owner(config) is None


def test_review_validity_rejects_changed_state_and_expired_windows():
    plan, ref = {"burnUT": 5000}, vessel()
    assert invalid_reason(plan, ref, ref, 60) is None
    assert invalid_reason(plan, ref, {**ref, "argumentOfPeriapsis": 2}, 60) is None  # circular orbit
    for delta in ({"vesselId": 999}, {"periapsis": 84000}, {"ut": 4950},
                  {"ut": 0}, {"mass": 40000}, {"scene": "space_center"}):
        assert invalid_reason(plan, ref, {**ref, **delta}, 60)


@pytest.fixture
def review_run(tmp_path):
    requests = tmp_path / "backend-requests.txt"
    info = vessel()
    with mock.patch("ksl.ownership.DATA_ROOT", tmp_path), \
         mock.patch("ksl.programs.BACKEND", FAKE), \
         mock.patch("ksl.krpc_link.probe", side_effect=lambda _: dict(info, observedAt=time.time())), \
         mock.patch("ksl.programs.probe_orbiter_integrity", return_value={"partNames": ["fake"]}), \
         mock.patch("ksl.programs.ensure_payload_bay_closed", return_value={"status": "closed"}), \
         mock.patch.dict("os.environ", {"FAKE_BACKEND_REQUESTS": str(requests)}):
        controller = OperationRequest("deorbit", load_config(), "live").build(tmp_path)
        thread = controller.start_background()
        try:
            wait_for(lambda: controller.status.operation_stage == "review")
            yield controller, Commands(controller.run_dir), info, requests
        finally:
            controller.abort()
            thread.join(5)
            assert not thread.is_alive(), "Review did not release its backend"


def test_review_waits_replans_and_executes_only_the_reviewed_revision(review_run):
    controller, commands, info, requests = review_run
    revision = controller.status.plan_revision
    assert "engage\n" not in requests.read_text()
    with pytest.raises(ValueError, match="current plan"):
        commands.submit("execute-plan", "stale")
    replan = commands.submit("replan", revision)
    wait_for(lambda: controller.status.plan_revision not in (None, revision))
    assert commands.read(replan["id"])["state"] == "completed"
    execute = commands.submit("execute-plan", controller.status.plan_revision)
    wait_for(lambda: controller.status.finished_at is not None)
    assert controller.status.state == "complete"
    assert commands.read(execute["id"])["state"] == "completed"
    assert requests.read_text().count("engage\n") == 1


def test_changed_orbit_rejects_execution_and_external_abort_releases_review(review_run):
    controller, commands, info, requests = review_run
    info["periapsis"] -= 1000
    execute = commands.submit("execute-plan", controller.status.plan_revision)
    wait_for(lambda: commands.read(execute["id"])["state"] == "rejected")
    assert "periapsis" in commands.read(execute["id"])["message"]
    assert "engage\n" not in requests.read_text()
    abort = commands.submit("abort")
    wait_for(lambda: commands.read(abort["id"])["state"] == "completed")
    assert controller.status.state == "aborted"
    assert "shutdown\n" in requests.read_text()


def test_tui_executes_an_external_reviewed_plan(review_run, tmp_path):
    from ksl.tui import KSLApp
    controller, commands, info, requests = review_run

    async def exercise():
        app = KSLApp(runs_root=tmp_path)
        async with app.run_test(size=(120, 45)) as pilot:
            app.vessel_info = info
            app.watch_run(controller.run_dir)
            await pilot.pause()
            assert app.controller is None  # a run started outside this UI
            assert app.displayed_plan_revision == controller.status.plan_revision
            assert "42.0 m/s" in str(app.query_one("#deorbit-plan").render())
            app.query_one("#tabs").active = "dashboard"  # full console holds the review panel
            await pilot.pause()
            button = app.query_one("#execute-plan")
            assert button.display and not button.disabled
            button.press()  # scroll-then-click raced the layout (OutOfBounds)
            for _ in range(50):
                await pilot.pause(0.1)
                if controller.status.finished_at and not app.pending_commands:
                    break
            assert controller.status.state == "complete"
            assert "completed" in str(app.query_one("#command-result").render())
    asyncio.run(exercise())
