"""Integration tests for the unified mission layer (offline; no KSP needed)."""

from __future__ import annotations

import io
import json
import re
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from unittest import mock

from ksl import cli, programs
from ksl.backend import BackendError, validate_plan_qualification
from ksl.config import ConfigError, config_from_snapshot, load_config
from ksl.controller import MissionController, continue_phase
from ksl.kspguard import GuardError, STS_REQUIRED_PARTS, check_orbiter_integrity
from ksl.paths import BACKEND, CLANDING, ROOT
from ksl.phases import BACKEND_PHASES, BACKEND_FAILURE_PHASES, Phase, PhaseMachine, PhaseTransitionError
from ksl.replay import Recording
from ksl.runlog import Event, MissionStatus, RunLog, read_events, read_telemetry
from ksl.scenarios import SCENARIOS

FAKES = Path(__file__).resolve().parent / "fakes"


class OrbiterIntegrityTests(unittest.TestCase):
    def test_intact_sts_is_accepted(self):
        names = [name for name, count in STS_REQUIRED_PARTS.items() for _ in range(count)]
        self.assertEqual(check_orbiter_integrity({"ready": True, "partNames": names})["partNames"], names)

    def test_detached_wing_and_dependents_block_deorbit(self):
        names = [name for name, count in STS_REQUIRED_PARTS.items() for _ in range(count)]
        names.remove("benjee10.shuttle.deltaWing")
        with self.assertRaisesRegex(GuardError, "deltaWing"):
            check_orbiter_integrity({"ready": True, "partNames": names})


class PhaseModelTests(unittest.TestCase):
    def test_machine_is_forward_only_and_allows_skips(self):
        m = PhaseMachine(Phase.MM305)
        self.assertTrue(m.advance(Phase.ROLLOUT))
        self.assertFalse(m.advance(Phase.ROLLOUT))
        with self.assertRaises(PhaseTransitionError):
            m.advance(Phase.FINAL_APPROACH)

    def test_every_backend_phase_string_is_classified(self):
        # Guards drift between CLanding's phase_string() table and the mission mapping.
        source = (CLANDING / "vehicle" / "models.c").read_text()
        table = re.search(r"phase_string\(GuidancePhase p\)\{static const char \*v\[\]=\{([^}]*)\}", source)
        names = re.findall(r'"([^"]+)"', table.group(1))
        status_only = {"Idle", "Paused", "Attitude Recovery"}
        unclassified = [n for n in names if n not in BACKEND_PHASES and n not in BACKEND_FAILURE_PHASES
                        and n not in status_only]
        self.assertEqual(unclassified, [])

    def test_continue_rules(self):
        self.assertEqual(continue_phase(Phase.ORBIT), Phase.DEORBIT)
        self.assertEqual(continue_phase(Phase.MM305), Phase.MM305)
        for phase in (Phase.ASCENT, Phase.ROLLOUT):
            with self.assertRaises(PhaseTransitionError):
                continue_phase(phase)


class ConfigTests(unittest.TestCase):
    def test_polar_landing_attempts_unqualified_guidance_with_warnings(self):
        config = load_config("sts-polar")
        self.assertFalse(config["landing"]["requireStrictPlan"])
        # Flight 4 needed ~35s after nominal burn start for retrograde acquisition.
        self.assertGreaterEqual(config.backend_configuration()["guidance"]
                                ["minimumPlanningLeadTime"], 60.0)
        for degraded in (
                {"executionQualified": True, "targetCaptureAchieved": False},
                {"executionQualified": False, "targetCaptureAchieved": False},
                {"executionQualified": None, "targetCaptureAchieved": None},
        ):
            with self.subTest(plan=degraded):
                warning = validate_plan_qualification(degraded, False)
                self.assertIn("not certified" if not degraded.get("executionQualified")
                              else "no certified", warning)
                self.assertFalse(degraded.get("targetCaptureAchieved"))
        self.assertIsNone(validate_plan_qualification(
            {"executionQualified": True, "targetCaptureAchieved": True}, False))
        # Strict certification is an *explicit* opt-in for other campaigns.
        strict = load_config("sts-polar", ["landing.requireStrictPlan=true"])
        self.assertTrue(strict["landing"]["requireStrictPlan"])
        with self.assertRaisesRegex(BackendError, "strict full-route"):
            validate_plan_qualification(
                {"executionQualified": True, "targetCaptureAchieved": False}, True)
        with self.assertRaisesRegex(BackendError, "no executable candidate"):
            validate_plan_qualification({}, False)

    def test_polar_insertion_periapsis_may_be_inside_atmosphere(self):
        config = load_config('sts-polar')
        # OMS raises periapsis after MECO. A sub-atmosphere-edge insertion
        # is explicitly supported and is not itself a premature-cutoff fault.
        self.assertEqual(config.insertion_periapsis_m, 70000.0)
        self.assertEqual(config.validate(), [])

    def test_profiles_validate_and_overrides_apply(self):
        config = load_config("sts-85km-10deg", ["ascent.inclinationDeg=12.5", "orbit.minBurnDeltaV=0.2"])
        self.assertEqual(config.validate(), [])
        self.assertEqual(config["ascent"]["inclinationDeg"], 12.5)
        self.assertEqual(config.insertion_periapsis_m, 55000.0)
        backend = config.backend_configuration()
        self.assertEqual(backend["connection"]["rpcPort"], 50000)

    def test_bad_profiles_are_rejected(self):
        with self.assertRaises(ConfigError):
            load_config("no-such-profile")
        with self.assertRaises(ConfigError):
            load_config("sts-kerbin", ["ascent.unknownKey=1"])
        self.assertTrue(load_config("sts-kerbin", ["ascent.targetAltitudeKm=60"]).validate())

    def test_invalid_types_and_nonfinite_limits_report_diagnostics(self):
        for override in ("ascent=null", "ascent.targetAltitudeKm=NaN", "connection.rpcPort=1.5",
                         "landing.autoWarp=\"false\"", "orbit.extraArgs=[1]",
                         "landing.flightTimeoutSeconds=-1", "checkpoints.kspSaveOnPhases=\"orbit\""):
            with self.subTest(override=override):
                self.assertTrue(load_config("defaults", [override]).validate())
        config = load_config()
        config.data["landing"]["typoTimeout"] = 10
        self.assertIn("unknown configuration key landing.typoTimeout", config.validate())

    def test_snapshot_restores_resolved_values_and_explicit_overrides(self):
        original = load_config("sts-kerbin", ["ascent.inclinationDeg=12", "connection.rpcPort=50010"])
        restored = config_from_snapshot(original.name, original.data, ["orbit.trim=false"])
        self.assertEqual(restored["ascent"]["inclinationDeg"], 12)
        self.assertEqual(restored["connection"]["rpcPort"], 50010)
        self.assertFalse(restored["orbit"]["trim"])
        self.assertTrue(original["orbit"]["trim"])
        self.assertEqual(restored.validate(), [])


class ControllerTestCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.runs = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def controller(self, **kw):
        kw.setdefault("runs_root", self.runs)
        return MissionController(load_config(kw.pop("profile", "sts-85km-10deg")), **kw)


class ControllerTests(ControllerTestCase):
    def test_parallel_runs_get_separate_directories(self):
        def create(_):
            log = RunLog.create(MissionStatus(run_id="same-second", profile="test", mode="dry-run"), self.runs)
            try:
                return log.directory
            finally:
                log.close()
        with ThreadPoolExecutor(max_workers=8) as pool:
            directories = list(pool.map(create, range(16)))
        self.assertEqual(len(set(directories)), 16)

    def test_dry_run_plans_every_program_without_flying(self):
        c = self.controller(mode="dry-run")
        self.assertEqual([p.name for p, _ in c.plan()], ["ascent", "orbit-trim", "landing"])
        status = c.run()
        self.assertEqual(status.state, "holding")
        log = (c.run_dir / "events.jsonl").read_text()
        self.assertIn("--min-burn-dv 0.1", log)
        self.assertIn("--inclination 10", log)

    def test_unholdable_stop_is_rejected(self):
        with self.assertRaises(PhaseTransitionError):
            self.controller(mode="live", stop=Phase.ASCENT)
        with self.assertRaises(PhaseTransitionError):
            self.controller(mode="live", start=Phase.LAUNCH)

    def test_ascent_milestones_drive_phases(self):
        c = self.controller(mode="live", stop=Phase.ORBIT)
        c.config.data["orbit"]["trim"] = False
        fake = [sys.executable, str(FAKES / "fake_ascent.py")]
        with mock.patch.object(programs.AscentProgram, "_pad_prep"), \
                mock.patch.object(programs.AscentProgram, "_argv", return_value=fake), \
                mock.patch("ksl.krpc_link.save_game"):
            status = c.run()
        self.assertEqual(status.state, "holding", status.status_message)
        self.assertEqual(status.phase_history, ["prelaunch", "launch", "ascent", "orbit"])
        self.assertTrue(read_telemetry(c.run_dir))
        self.assertEqual(json.loads((c.run_dir / "checkpoint.json").read_text())["phase"], "orbit")

    def test_ascent_failure_fails_mission(self):
        c = self.controller(mode="live", stop=Phase.ORBIT)
        fake = [sys.executable, str(FAKES / "fake_ascent.py"), "3"]
        with mock.patch.object(programs.AscentProgram, "_pad_prep"), \
                mock.patch.object(programs.AscentProgram, "_argv", return_value=fake):
            status = c.run()
        self.assertEqual(status.state, "failed")
        self.assertEqual(c.exit_code(), 1)


class LandingProgramTests(ControllerTestCase):
    """Live landing path against a scripted NDJSON backend."""

    def fly(self, mode: str, abort_after: float | None = None):
        c = self.controller(mode="live", start=Phase.MM305)
        c.config.data["checkpoints"]["kspSaveOnPhases"] = ["final-approach"]
        patches = [mock.patch.object(programs, "BACKEND", FAKES / "fake_backend.py"),
                   mock.patch.object(programs, "probe_orbiter_integrity",
                                     return_value={"partNames": ["fake-part"]}),
                   mock.patch.object(programs, "prepare_reentry_direct_control",
                                     return_value={"meanAltitude": 20000.0}),
                   mock.patch.dict("os.environ", {"FAKE_BACKEND_MODE": mode})]
        for p in patches:
            p.start()
        self.addCleanup(lambda: [p.stop() for p in patches])
        if abort_after is not None:
            import threading
            threading.Timer(abort_after, c.abort).start()
        return c, c.run()

    def test_landing_phases_checkpoint_and_completion(self):
        c, status = self.fly("land")
        self.assertEqual(status.state, "complete", status.status_message)
        self.assertEqual(status.phase_history,
                         ["mm305", "final-approach", "touchdown", "rollout", "complete"])
        checkpoint = json.loads((c.run_dir / "checkpoint.json").read_text())
        self.assertEqual(checkpoint["phase"], "final-approach")
        self.assertTrue(checkpoint["kspSave"].endswith("final-approach"))
        self.assertTrue(any(s.get("backendPhase") for s in read_telemetry(c.run_dir)))

    def test_deorbit_proceeds_after_strict_qualification_failure(self):
        for mode in ("unqualified-route", "unqualified-entry"):
            with self.subTest(mode=mode):
                c = self.controller(profile="sts-polar", mode="live", start=Phase.DEORBIT)
                with mock.patch.object(programs, "BACKEND", FAKES / "fake_backend.py"), \
                     mock.patch.object(programs, "probe_orbiter_integrity",
                                       side_effect=programs.GuardError("partial wing inventory")), \
                     mock.patch.dict("os.environ", {"FAKE_BACKEND_MODE": mode}):
                    status = c.run()
                self.assertEqual(status.state, "complete", status.status_message)
                self.assertIn("deorbit", status.phase_history)
                events = list(read_events(c.run_dir))
                self.assertTrue(any("guidance-degraded" == e.kind for e in events))
                self.assertTrue(any("partial wing inventory" in e.message for e in events))
                plan = json.loads((c.run_dir / "deorbit-plan.json").read_text())
                self.assertFalse(plan["targetCaptureAchieved"])
                self.assertEqual(plan["robustnessPassed"], 0)

    def test_backend_fault_fails_mission(self):
        c, status = self.fly("fault")
        self.assertEqual(status.state, "failed")
        self.assertIn("fake actuator fault", status.status_message)

    def test_operator_abort_sends_abort_and_stops(self):
        c, status = self.fly("hold", abort_after=0.5)
        self.assertEqual(status.state, "aborted")
        self.assertEqual(c.exit_code(), 130)
        self.assertTrue(any("Abort requested" in e.message for e in read_events(c.run_dir)))


class CliTests(unittest.TestCase):
    def test_replay_uses_simulation_clock_after_initial_backend_placeholder(self):
        recording = Recording(Path("example"), "run", samples=[
            {"ut": 0, "time": 1791730000.0, "phase": "orbit"},
            {"ut": 5254.0, "time": 1791730001.0, "phase": "orbit"},
            {"ut": 5260.0, "time": 1791730007.0, "phase": "deorbit"},
        ])
        self.assertEqual(recording.summary()["duration_s"], 6.0)
        self.assertEqual(recording.timeline(), [(0.0, "orbit"), (6.0, "deorbit")])
        recording.samples[1]["phase"] = "deorbit"
        recording.samples[2]["phase"] = "mm304"
        self.assertEqual(recording.timeline(), [(0.0, "orbit"), (0.0, "deorbit"),
                                                (6.0, "mm304")])
        recording.events.append(Event("phase", "Phase: Complete", phase="complete"))
        self.assertEqual(recording.timeline()[-1], (6.0, "complete"))

    def run_cli(self, *argv):
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            code = cli.main(list(argv))
        return code, out.getvalue(), err.getvalue()

    def test_usage_errors_exit_2(self):
        self.assertEqual(self.run_cli("run", "-p", "nope", "--dry-run")[0], 2)
        self.assertEqual(self.run_cli("run", "--to", "ascent", "--dry-run")[0], 2)
        self.assertEqual(self.run_cli("land", "--from", "launch", "--dry-run")[0], 2)

    def test_dry_run_json_and_replay_export(self):
        with tempfile.TemporaryDirectory() as tmp, mock.patch.dict("os.environ", {"KSL_RUNS_DIR": tmp}), \
                mock.patch("ksl.runlog.RUNS_DIR", Path(tmp)):
            code, out, _ = self.run_cli("run", "--dry-run", "--json")
            self.assertEqual(code, 0)
            result = json.loads(out.strip().splitlines()[-1])
            self.assertEqual(result["result"]["state"], "holding")
            code, out, _ = self.run_cli("status", "--json")
            self.assertEqual(json.loads(out)["mode"], "dry-run")
            self.assertEqual(self.run_cli("replay", "--json")[0], 0)
        code, out, _ = self.run_cli("simulate", "--list", "--json")
        self.assertIn("l5-mm305", [row["name"] for row in json.loads(out)])

    def test_ascent_csv_replay_and_export(self):
        csv_path = FAKES / "ascent_sample.csv"
        code, out, _ = self.run_cli("replay", str(csv_path), "--json")
        self.assertEqual(code, 0)
        timeline = [phase for _, phase in json.loads(out)["timeline"]]
        self.assertEqual(timeline[:2], ["launch", "ascent"])
        code, out, _ = self.run_cli("export", str(csv_path), "-f", "csv")
        self.assertTrue(out.startswith("ut,state,altitude"))

    def test_continue_uses_saved_overrides(self):
        with tempfile.TemporaryDirectory() as tmp:
            original = MissionController(load_config(overrides=["ascent.inclinationDeg=12"]),
                                         mode="dry-run", start=Phase.ORBIT, runs_root=Path(tmp))
            original.run()
            with mock.patch("ksl.runlog.RUNS_DIR", Path(tmp)):
                code, out, err = self.run_cli("continue", str(original.run_dir), "--dry-run", "--json")
            self.assertEqual(code, 0, err)
            result = json.loads(out.strip().splitlines()[-1])
            saved = json.loads((Path(result["runDirectory"]) / "config.json").read_text())
            self.assertEqual(saved["configuration"]["ascent"]["inclinationDeg"], 12)

    def test_bad_profile_cannot_load_a_save(self):
        with mock.patch("ksl.kspguard.load_save_verified") as load:
            code, _, err = self.run_cli("run", "--from", "mm305", "--load-save", "example",
                                        "--set", "connection.rpcPort=NaN")
        self.assertEqual(code, 2, err)
        load.assert_not_called()

    def test_explicit_cli_operations_use_the_same_directed_boundaries(self):
        with tempfile.TemporaryDirectory() as tmp, mock.patch("ksl.runlog.RUNS_DIR", Path(tmp)):
            code, out, err = self.run_cli("operate", "launch", "--dry-run", "--json")
            self.assertEqual(code, 0, err)
            result = json.loads(out.strip().splitlines()[-1])
            self.assertEqual(result["result"]["stop_phase"], "orbit")
            self.assertEqual(result["result"]["operation"], "launch")
            self.assertNotIn('"message":"orbit:', out)
            code, out, err = self.run_cli("operate", "deorbit", "--dry-run")
            self.assertEqual(code, 0, err)
            self.assertIn("wait for explicit plan execution", out)
            code, out, err = self.run_cli("operate", "simulate", "--dry-run", "--scenario", "l5-mm305")
            self.assertEqual(code, 0, err)
            self.assertIn("simulation: ShuttleSim scenario l5-mm305", out)
            self.assertNotIn("landing: connect to kRPC", out)


class TuiTests(unittest.IsolatedAsyncioTestCase):
    async def test_quit_waits_for_control_release(self):
        from ksl.tui import KSLApp
        app = KSLApp()
        app.controller = mock.Mock()
        app.controller.status.state = "running"
        app.quit_armed_at = time.monotonic()
        app.controller.abort.side_effect = lambda: setattr(app.controller.status, "state", "aborted")
        with mock.patch.object(app, "exit") as exit_app:
            await app.action_quit_safe()
        app.controller.abort.assert_called_once()
        exit_app.assert_called_once_with(0)

    def test_poll_ignores_stopped_app(self):
        from ksl.tui import KSLApp
        app = KSLApp()
        app.watcher = mock.Mock()
        app.watcher.poll.return_value = (None, [])
        self.assertFalse(app.is_running)
        app.poll()
        app.watcher.poll.assert_not_called()

    async def test_inclination_and_altitude_fields_set_the_mission(self):
        from textual.widgets import Input
        from ksl.tui import KSLApp
        app = KSLApp()
        async with app.run_test(size=(120, 40)) as pilot:
            await pilot.press("m")
            app.query_one("#inclination", Input).value = "51.6"
            app.query_one("#altitude", Input).value = "120"
            app.query_one("#overrides", Input).value = "ascent.targetAltitudeKm=400 orbit.trim=true"
            from textual.widgets import Select, Switch
            app.query_one("#orbit-trim", Switch).value = False
            app.query_one("#kind", Select).value = "geo"
            app.query_one("#geo-longitude", Input).value = "-30"
            sel = app._selection()
        config = load_config(sel["profile"], sel["overrides"])
        self.assertEqual(config.data["ascent"]["inclinationDeg"], 51.6)
        self.assertEqual(config.data["ascent"]["targetAltitudeKm"], 120)
        self.assertFalse(config.data["orbit"]["trim"])  # the switch owns it, not the free-text box
        self.assertEqual(config.data["profile"]["kind"], "geo")
        self.assertEqual(config.data["profile"]["geoLongitudeDeg"], -30)
        self.assertEqual(config.validate(), [])

    async def test_mode_and_vessel_context_limit_operations(self):
        from textual.widgets import Select
        from ksl.tui import KSLApp
        app = KSLApp()
        async with app.run_test(size=(120, 40)) as pilot:
            app.query_one("#mode", Select).value = "live"
            await pilot.pause()
            self.assertTrue(app.query_one("#validate").disabled)
            self.assertIn("Connect to KSP", str(app.query_one("#availability").render()))
            app.vessel_info = {"scene": "flight", "situation": "pre_launch", "observedAt": time.time()}
            app.refresh_operations()
            self.assertFalse(app.query_one("#validate").disabled)
            app.query_one("#mode", Select).value = "sim"
            await pilot.pause()
            self.assertEqual(app.selected_operation, "simulate")
            self.assertTrue(app.query_one("#scenario-fields").display)
            self.assertFalse(app.query_one("#launch-fields").display)

    async def test_vessel_monitor_runs_without_a_mission_or_valid_form_fields(self):
        from ksl.tui import KSLApp
        from textual.widgets import Input
        from tests.test_operations import vessel
        with tempfile.TemporaryDirectory() as tmp, mock.patch("ksl.krpc_link.probe", side_effect=lambda _: vessel()) as probe:
            app = KSLApp(runs_root=Path(tmp))
            async with app.run_test(size=(120, 40)) as pilot:
                app.query_one("#altitude", Input).value = "editing"
                await pilot.click("#connect-vessel")
                for _ in range(20):
                    await pilot.pause(0.05)
                    if app.vessel_info:
                        break
                self.assertEqual(app.vessel_info["vessel"], "STS")
                self.assertIn("STS", str(app.query_one("#vehicle-panel").render()))
                self.assertIsNone(app.controller)
                self.assertEqual(list(Path(tmp).iterdir()), [])
                probe.assert_called()

    async def test_checkpoint_continuation_restores_the_recorded_profile(self):
        from ksl.tui import KSLApp
        from textual.widgets import Input
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            original = MissionController(load_config(overrides=["ascent.inclinationDeg=51.6"]),
                                         "dry-run", Phase.ORBIT, Phase.ORBIT, runs_root=root)
            original.run()
            app = KSLApp(runs_root=root)
            async with app.run_test(size=(120, 40)) as pilot:
                app.watch_run(original.run_dir)
                app.choose_operation("continue")
                app.query_one("#altitude", Input).value = "unfinished unrelated edit"
                await pilot.pause()
                app.validate_plan()
                app.action_start()
                for _ in range(30):
                    await pilot.pause(0.1)
                    if app.controller and app.controller.status.finished_at:
                        break
                self.assertEqual(app.controller.config["ascent"]["inclinationDeg"], 51.6)
                self.assertEqual(app.controller.start, Phase.DEORBIT)
                self.assertEqual(app.controller.status.state, "holding")
                self.assertEqual(app._connection_config()["ascent"]["inclinationDeg"], 51.6)

    async def test_quick_start_needs_confirming_second_press(self):
        from ksl.tui import KSLApp
        with tempfile.TemporaryDirectory() as tmp:
            app = KSLApp(runs_root=Path(tmp))
            async with app.run_test(size=(120, 40)) as pilot:
                app.query_one("#q-land-mode").value = "sim"
                app.quick_start("land")
                self.assertIsNone(app.controller)
                app.quick_start("land")
                await pilot.pause(0.5)
                self.assertEqual(app.controller.mode, "sim")

    async def test_reviewed_operation_runs_and_edits_invalidate_review(self):
        from ksl.tui import KSLApp
        from textual.widgets import Input
        with tempfile.TemporaryDirectory() as tmp:
            app = KSLApp(runs_root=Path(tmp))
            async with app.run_test(size=(120, 40)) as pilot:
                await pilot.press("d")
                await pilot.press("s")
                self.assertIsNone(app.controller)  # no unmodified single-key start
                app.query_one("#validate").scroll_visible()
                await pilot.pause()
                await pilot.click("#validate")
                self.assertIn("Launch to orbit", str(app.query_one("#plan").render()))
                self.assertFalse(app.query_one("#start-btn").disabled)
                app.query_one("#altitude", Input).value = "120"
                await pilot.pause()
                self.assertTrue(app.query_one("#start-btn").disabled)
                app.query_one("#validate").scroll_visible()
                await pilot.pause()
                await pilot.click("#validate")
                await pilot.press("ctrl+enter")
                for _ in range(30):
                    await pilot.pause(0.1)
                    if app.controller and app.controller.status.finished_at:
                        break
                await pilot.pause(0.6)
                self.assertEqual(app.controller.status.state, "holding")
                self.assertEqual(app.controller.status.operation, "launch")
                self.assertEqual([p.name for p, _ in app.controller.plan()], ["ascent"])
                self.assertEqual(app.query_one("#tabs").active, "dashboard")
                await pilot.resize_terminal(80, 24)
                await pilot.press("r")
                await pilot.pause(0.2)
                self.assertEqual(app.query_one("#runs").row_count, 1)


@unittest.skipUnless(BACKEND.is_file() and (ROOT / "ShuttleSim" / "build" / "shuttlesim").is_file(),
                     "needs `make backend sim`")
class ClosedLoopSimulationTests(ControllerTestCase):
    def test_l5_mm305_golden_replay(self):
        """Native backend + ShuttleSim from the archived L5 MM305 state (deterministic)."""
        c = self.controller(mode="sim", scenario=SCENARIOS["l5-mm305"], profile="sts-kerbin")
        status = c.run()
        self.assertEqual(status.phase_history[0], "mm305")
        self.assertIn("complete", status.phase_history)
        result = json.loads((c.run_dir / "simulation-result.json").read_text())
        s = result["simulatorSummary"]
        # The closed loop is wall-clock coupled (sim_rate varies by host and
        # checkout), so pin the acceptance envelope rather than 0.01 m/s values.
        # 2026-10-15: 72.35 m/s / 1.53 m/s sink / stop 908 m (71.76/983 m in a
        # fresh worktree).
        self.assertTrue(s["on_runway"] and s["touchdown_gear"] and not s["crashed"])
        self.assertTrue(s["stopped"] and not s["runway_departure"])
        self.assertTrue(64.0 <= s["touchdown_speed_mps"] <= 80.0, s["touchdown_speed_mps"])
        self.assertLessEqual(s["touchdown_sink_mps"], 2.5)
        self.assertLess(s["stop_along_m"], 1800.0)
        self.assertEqual(status.state, "complete")

    def test_l7_low_energy_final_still_flares(self):
        """Live L7 regression: an energy-short TAEM must hand off to Final and flare.

        Before the low-energy delivery, TAEM kept the vehicle and hit the ground
        with no flare (13 m/s sink, gear up). The vehicle cannot reach the runway
        from this state; the requirement is a controlled, flared, gear-down touchdown.
        """
        c = self.controller(mode="sim", scenario=SCENARIOS["l7-mm305"])
        status = c.run()
        self.assertIn("final-approach", status.phase_history)
        s = json.loads((c.run_dir / "simulation-result.json").read_text())["simulatorSummary"]
        self.assertTrue(s["touchdown_gear"] and not s["crashed"])
        # Without a flare this state hits at ~13 m/s. The low-speed flare is still
        # sensitive (0.8-4.5 m/s across stretch tunings), so this guards the flare
        # happening, not its quality.
        self.assertLess(s["touchdown_sink_mps"], 5.0)
        self.assertLess(abs(s["touchdown_cross_m"]), 20.0)


if __name__ == "__main__":
    unittest.main()
