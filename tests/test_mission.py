"""Integration tests for the unified mission layer (offline; no KSP needed)."""

from __future__ import annotations

import io
import json
import re
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from unittest import mock

from ksl import cli, programs
from ksl.config import ConfigError, load_config
from ksl.controller import MissionController, continue_phase
from ksl.paths import BACKEND, CLANDING, ROOT
from ksl.phases import BACKEND_PHASES, BACKEND_FAILURE_PHASES, Phase, PhaseMachine, PhaseTransitionError
from ksl.runlog import read_events, read_telemetry
from ksl.scenarios import SCENARIOS

FAKES = Path(__file__).resolve().parent / "fakes"


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


class TuiTests(unittest.IsolatedAsyncioTestCase):
    async def test_dry_run_from_mission_tab_shows_on_dashboard(self):
        from ksl.tui import KSLApp
        with tempfile.TemporaryDirectory() as tmp:
            app = KSLApp(runs_root=Path(tmp))
            async with app.run_test(size=(120, 40)) as pilot:
                await pilot.press("m")
                await pilot.click("#validate")
                self.assertIn("orbit-trim", str(app.query_one("#plan").render()))
                await pilot.press("s")
                for _ in range(30):
                    await pilot.pause(0.1)
                    if app.controller.status.state != "running":
                        break
                await pilot.pause(0.6)
                self.assertEqual(app.controller.status.state, "holding")
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
        # Baseline at integration (47bada6): on-runway landing and rollout, with
        # touchdown sink above the gate. Any change here is a behavior change.
        self.assertTrue(s["on_runway"] and s["touchdown_gear"] and not s["crashed"])
        self.assertAlmostEqual(s["touchdown_speed_mps"], 60.704, places=2)
        self.assertAlmostEqual(s["touchdown_sink_mps"], 4.944, places=2)
        self.assertAlmostEqual(s["stop_along_m"], 989.217, places=1)
        self.assertEqual(status.state, "failed")  # honest: the sink/speed gates fail at baseline


if __name__ == "__main__":
    unittest.main()
