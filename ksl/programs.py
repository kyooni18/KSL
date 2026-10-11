"""Phase programs: the units of flight software the mission controller sequences.

    AscentProgram      Prelaunch -> Launch -> Ascent -> Orbit insertion
                       (ksl.ascent.flight child process; Python kRPC guidance)
    OrbitTrimProgram   Orbit: OMS trim to the circular target (ksl.ascent.orbit_trim)
    LandingProgram     Deorbit -> MM304 -> MM305 -> Final -> Touchdown -> Rollout
                       (native C backend over NDJSON, live KSP)
    SimulationProgram  the same landing phases closed-loop against ShuttleSim

Programs never contain guidance or control laws; they start the program that
owns those phases, translate its progress into mission phases/telemetry, and
apply the safe-stop path when the operator aborts.
"""

from __future__ import annotations

import json
import math
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable

from .backend import BackendClient, BackendError, snapshot_summary, validate_plan_qualification
from .config import MissionConfig
from .krpc_link import ConnectionFailure, connect, endpoint_args, krpc_env, save_game, wait_for_flight_scene
from .kspguard import prepare_reentry_direct_control, set_ksp_paused
from .paths import BACKEND, ROOT, SHUTTLESIM
from .phases import BACKEND_ENGAGE, BACKEND_FAILURE_PHASES, BACKEND_PHASES, ASCENT_EVENTS, Phase
from .runlog import Event, RunLog
from .scenarios import Scenario

EVENT_PREFIX = "@@ksl "
STOP_GRACE_SECONDS = 20.0


class ProgramUnavailable(RuntimeError):
    """The requested phase cannot be flown in this mode (e.g. no offline ascent plant)."""


@dataclass
class Outcome:
    ok: bool
    message: str = ""
    aborted: bool = False
    holding: bool = False      # stopped deliberately at the requested phase


@dataclass
class Context:
    config: MissionConfig
    log: RunLog
    mode: str                                  # live | sim | dry-run
    stop_event: threading.Event
    advance: Callable[[Phase], None]
    checkpoint: Callable[[Phase, str | None], None]
    scenario: Scenario | None = None

    @property
    def python(self) -> str:
        return sys.executable


class Program:
    name = "program"
    phases: tuple[Phase, ...] = ()

    def describe(self, ctx: Context, start: Phase, stop: Phase) -> list[str]:
        raise NotImplementedError

    def run(self, ctx: Context, start: Phase, stop: Phase) -> Outcome:
        raise NotImplementedError


# --- child-process plumbing ------------------------------------------------------

def run_child(ctx: Context, name: str, argv: list[str], env: dict[str, str] | None,
              on_event: Callable[[dict[str, Any]], None],
              on_line: Callable[[str], None] | None = None) -> tuple[int, list[str]]:
    """Run a phase program, streaming its output into the run log.

    Returns (exit code, last output lines). On operator stop the child gets
    SIGINT (both ascent programs release controls in their finally/except
    paths), then SIGTERM, then SIGKILL.
    """
    process_env = os.environ.copy()
    process_env.update(env or {})
    process_env["PYTHONUNBUFFERED"] = "1"
    log_path = ctx.log.directory / f"{name}.log"
    tail: list[str] = []
    with open(log_path, "a", encoding="utf-8") as raw_log:
        proc = subprocess.Popen(argv, cwd=ROOT, env=process_env, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True)
        lines: queue.Queue[str | None] = queue.Queue()

        def reader() -> None:
            assert proc.stdout is not None
            for line in proc.stdout:
                lines.put(line.rstrip("\n"))
            lines.put(None)

        threading.Thread(target=reader, name=f"{name}-reader", daemon=True).start()
        stop_sent_at: float | None = None
        while True:
            if ctx.stop_event.is_set() and stop_sent_at is None and proc.poll() is None:
                ctx.log.log(f"{name}: stop requested; sending SIGINT for the safe-release path", "warning")
                proc.send_signal(signal.SIGINT)
                stop_sent_at = time.monotonic()
            if stop_sent_at is not None and proc.poll() is None:
                if time.monotonic() - stop_sent_at > STOP_GRACE_SECONDS:
                    proc.terminate()
                if time.monotonic() - stop_sent_at > STOP_GRACE_SECONDS + 5.0:
                    proc.kill()
            try:
                line = lines.get(timeout=0.2)
            except queue.Empty:
                continue
            if line is None:
                break
            raw_log.write(line + "\n")
            raw_log.flush()
            if line.startswith(EVENT_PREFIX):
                try:
                    on_event(json.loads(line[len(EVENT_PREFIX):]))
                except json.JSONDecodeError:
                    pass
                continue
            tail = (tail + [line])[-20:]
            if on_line is not None:
                on_line(line)
            ctx.log.emit(Event("program", line, level="debug", data={"program": name}))
        return proc.wait(), tail


# --- ascent ------------------------------------------------------------------------

class AscentProgram(Program):
    name = "ascent"
    phases = (Phase.PRELAUNCH, Phase.LAUNCH, Phase.ASCENT)

    def _argv(self, ctx: Context, preflight_only: bool) -> list[str]:
        a = ctx.config["ascent"]
        argv = [ctx.python, "-u", "-m", "ksl.ascent.flight", *endpoint_args(ctx.config),
                "--orbit", f"{ctx.config.target_altitude_m:.0f}",
                "--meco-periapsis", f"{ctx.config.insertion_periapsis_m:.0f}",
                "--inclination", str(a["inclinationDeg"]), "--ssme-limit", str(a["ssmeThrustLimit"]),
                "--telemetry", str(ctx.log.directory / "ascent.csv"), *map(str, a["extraArgs"])]
        if not preflight_only:
            argv.insert(4, "--launch")
        return argv

    def describe(self, ctx: Context, start: Phase, stop: Phase) -> list[str]:
        a = ctx.config["ascent"]
        if stop == Phase.PRELAUNCH:
            return ["ascent: read-only preflight of the STS on the pad (no control writes)",
                    "  $ " + " ".join(self._argv(ctx, True))]
        steps = []
        if a["revertToLaunch"]:
            steps.append("ascent: revert the current flight to launch")
        steps += [f"ascent: wait for the flight scene, settle {a['padSettleSeconds']:.0f} s, "
                  f"set Clydesdale limiters to {a['srbThrustLimit']}",
                  f"ascent: fly to {a['targetAltitudeKm']:.0f} x "
                  f"{ctx.config.insertion_periapsis_m / 1000:.0f} km at {a['inclinationDeg']} deg "
                  "(SSME lead, SRB/ET separation, PEG MECO)",
                  "  $ " + " ".join(self._argv(ctx, False))]
        return steps

    def _pad_prep(self, ctx: Context) -> None:
        a = ctx.config["ascent"]
        if a["revertToLaunch"]:
            ctx.log.log("Reverting flight to launch")
            with connect(ctx.config, "ksl-revert") as conn:
                conn.space_center.revert_to_launch()
            time.sleep(3.0)
        wait_for_flight_scene(ctx.config)
        ctx.log.update(connection="connected")
        settle = float(a["padSettleSeconds"])
        ctx.log.log(f"Pad: settling {settle:.0f} s before launch")
        if ctx.stop_event.wait(settle):
            raise InterruptedError("stopped on the pad before ignition")
        with connect(ctx.config, "ksl-pad") as conn:
            count = 0
            for engine in conn.space_center.active_vessel.parts.engines:
                if "Clydesdale" in engine.part.title:
                    engine.thrust_limit = float(a["srbThrustLimit"])
                    count += 1
        ctx.log.log(f"Pad: {count} Clydesdale thrust limiter(s) set to {a['srbThrustLimit']}")

    def run(self, ctx: Context, start: Phase, stop: Phase) -> Outcome:
        if ctx.mode == "sim":
            raise ProgramUnavailable(
                "no offline ascent plant exists; ascent is covered by unit tests and live flights. "
                "Use `ksl replay` on a recorded ascent, or start the simulation at deorbit or later.")
        preflight_only = stop == Phase.PRELAUNCH
        seen: set[str] = set()

        def on_event(event: dict[str, Any]) -> None:
            name = str(event.pop("event", ""))
            if name == "telemetry":
                vs, hs = event.get("verticalSpeed"), event.get("horizontalSpeed")
                if isinstance(vs, (int, float)) and isinstance(hs, (int, float)):
                    event["speed"] = math.hypot(vs, hs)
                ctx.log.telemetry(event, "ascent")
                return
            seen.add(name)
            phase = ASCENT_EVENTS.get(name)
            if phase is not None and not preflight_only:
                ctx.advance(phase)
            ctx.log.emit(Event("milestone", f"ascent: {name}", data=event))

        try:
            if not preflight_only:
                self._pad_prep(ctx)
        except InterruptedError as exc:
            return Outcome(False, str(exc), aborted=True)
        code, tail = run_child(ctx, self.name, self._argv(ctx, preflight_only), krpc_env(ctx.config), on_event)
        if ctx.stop_event.is_set():
            return Outcome(False, "ascent stopped by operator; controls released", aborted=True)
        if code != 0:
            return Outcome(False, f"ascent program exited with code {code}: " + (tail[-1] if tail else ""))
        if preflight_only:
            return Outcome(True, "preflight complete", holding=True)
        if not seen & {"insertion", "et-separation"}:
            return Outcome(False, "ascent program ended without orbital insertion")
        return Outcome(True, "orbital insertion complete")


# --- orbit ---------------------------------------------------------------------------

class OrbitTrimProgram(Program):
    name = "orbit-trim"
    phases = (Phase.ORBIT,)

    def _argv(self, ctx: Context) -> list[str]:
        alt = f"{ctx.config.target_altitude_m:.0f}"
        return [ctx.python, "-u", "-m", "ksl.ascent.orbit_trim", "--apoapsis", alt, "--periapsis", alt,
                "--inclination", str(ctx.config["ascent"]["inclinationDeg"]), *endpoint_args(ctx.config),
                "--min-burn-dv", str(ctx.config["orbit"]["minBurnDeltaV"]),
                *map(str, ctx.config["orbit"]["extraArgs"])]

    def describe(self, ctx: Context, start: Phase, stop: Phase) -> list[str]:
        if not ctx.config["orbit"]["trim"]:
            return ["orbit: OMS trim disabled (orbit.trim=false)"]
        return [f"orbit: OMS trim to {ctx.config['ascent']['targetAltitudeKm']:.0f} km circular "
                f"at {ctx.config['ascent']['inclinationDeg']} deg (reaction wheels, closed-loop re-plan)",
                "  $ " + " ".join(self._argv(ctx))]

    def run(self, ctx: Context, start: Phase, stop: Phase) -> Outcome:
        ctx.advance(Phase.ORBIT)
        if ctx.mode == "sim":
            raise ProgramUnavailable("no offline orbital plant exists for the OMS trim program")
        if ctx.config["orbit"]["trim"]:
            def on_event(event: dict[str, Any]) -> None:
                name = str(event.pop("event", ""))
                if name == "trim-report":
                    ctx.log.telemetry({"altitude": None, "apoapsis": event.get("apoapsis_m"),
                                       "periapsis": event.get("periapsis_m"),
                                       "inclination": event.get("inclination_deg")}, "orbit-trim")
                    ctx.log.emit(Event("milestone", f"orbit trim {event.get('tag')}: "
                                       f"Ap {event.get('apoapsis_m', 0) / 1000:.2f} km, "
                                       f"Pe {event.get('periapsis_m', 0) / 1000:.2f} km, "
                                       f"inc {event.get('inclination_deg', 0):.3f} deg", data=event))

            code, tail = run_child(ctx, self.name, self._argv(ctx), krpc_env(ctx.config), on_event)
            if ctx.stop_event.is_set():
                return Outcome(False, "orbit trim stopped by operator", aborted=True)
            if code != 0:
                return Outcome(False, f"orbit trim exited with code {code}: " + (tail[-1] if tail else ""))
        ctx.checkpoint(Phase.ORBIT, None)
        return Outcome(True, "in target orbit", holding=stop == Phase.ORBIT)


# --- landing (live) -------------------------------------------------------------------

class LandingProgram(Program):
    """Deorbit through rollout on the native C backend against live KSP."""
    name = "landing"
    phases = (Phase.DEORBIT, Phase.MM304, Phase.MM305, Phase.FINAL_APPROACH,
              Phase.TOUCHDOWN, Phase.ROLLOUT, Phase.COMPLETE)

    def describe(self, ctx: Context, start: Phase, stop: Phase) -> list[str]:
        landing = ctx.config["landing"]
        start = max(start, Phase.DEORBIT, key=lambda p: p.index)
        steps = [f"landing: start {BACKEND} (KSP_LANDER_MODEL_PROFILE={landing['modelProfile']}, "
                 f"configuration {landing['backendConfiguration']})",
                 f"landing: connect to kRPC {ctx.config['connection']['address']}:{ctx.config['connection']['rpcPort']}"]
        if start == Phase.DEORBIT:
            steps += ["landing: pause KSP, createPlan (deorbit search), resume; require entry-stage qualification"
                      + (" and strict full-route capture" if landing["requireStrictPlan"] else ""),
                      "landing: engage -> coast, deorbit burn, MM304 entry, MM305 TAEM, final, flare, rollout"]
        else:
            steps += ["landing: verified direct-control handoff (SAS off, zero throttle, 1x warp)",
                      f"landing: {BACKEND_ENGAGE[start]} from {start.title}"]
        return steps

    def run(self, ctx: Context, start: Phase, stop: Phase) -> Outcome:
        if ctx.mode != "live":
            raise ProgramUnavailable("LandingProgram flies live KSP; use SimulationProgram offline")
        landing = ctx.config["landing"]
        start = Phase.DEORBIT if start.index < Phase.DEORBIT.index else start
        env = {"KSP_LANDER_MODEL_PROFILE": str(landing["modelProfile"]), **krpc_env(ctx.config)}
        state = _LandingState(ctx)
        with open(ctx.log.directory / "backend.stderr.log", "ab") as stderr:
            client = BackendClient(BACKEND, snapshot_sink=state.on_snapshot, stderr_target=stderr, env=env)
            try:
                return self._fly(ctx, client, state, start, env)
            except (BackendError, TimeoutError) as exc:
                ctx.log.update(error=str(exc))
                return Outcome(False, str(exc))
            finally:
                if client.process.poll() is None and state.engaged and not state.terminal:
                    try:
                        client.send("abort", timeout=30.0)
                    except Exception:
                        pass
                client.shutdown()
                ctx.log.update(connection="disconnected")

    def _fly(self, ctx: Context, client: BackendClient, state: "_LandingState",
             start: Phase, env: dict[str, str]) -> Outcome:
        landing = ctx.config["landing"]
        if client._read_message(10.0).get("type") != "ready":
            raise BackendError("landing backend did not announce ready")
        client.send("updateConfiguration", configuration=ctx.config.backend_configuration())
        ctx.log.update(connection="connecting")
        client.send("connect", timeout=180.0)
        snap = client.latest_snapshot
        if not snap or snap.get("connectionStatus") not in ("connected", "failed"):
            snap = client.read_until(lambda s: s.get("connectionStatus") in ("connected", "failed"), 180.0)
        if snap.get("connectionStatus") != "connected":
            raise ConnectionFailure(str(snap.get("lastError") or "native kRPC connection failed"))
        ctx.log.log("Landing backend connected to KSP")

        if start == Phase.DEORBIT:
            ctx.advance(Phase.DEORBIT)
            set_ksp_paused(True, env=env)
            ctx.log.log("KSP paused while the deorbit search runs")
            try:
                client.send("createPlan", timeout=float(landing["planTimeoutSeconds"]))
            finally:
                set_ksp_paused(False, env=env)
                ctx.log.log("KSP resumed after deorbit planning")
            planned = client.latest_snapshot
            if not planned or not planned.get("plan"):
                planned = client.read_until(lambda s: bool(s.get("plan")) or s.get("phase") == "Fault", 30.0)
            plan = planned.get("plan") or {}
            if not plan:
                raise BackendError(str(planned.get("lastError") or "no deorbit plan was produced"))
            ctx.log.write_json("deorbit-plan.json", plan)
            ctx.log.emit(Event("milestone", "deorbit plan: burn UT {:.1f}, dv {:.2f} m/s, executable={}, strict={}".format(
                float(plan.get("burnUT") or 0), float(plan.get("deltaV") or 0),
                bool(plan.get("executionQualified")), bool(plan.get("targetCaptureAchieved"))), data=plan))
            warning = validate_plan_qualification(plan, bool(landing["requireStrictPlan"]))
            if warning:
                ctx.log.log(warning, "warning")
            client.engage("engage", timeout=120.0)
        else:
            payload = prepare_reentry_direct_control(env=env)
            ctx.log.log(f"Direct-control handoff verified at {float(payload.get('meanAltitude') or 0) / 1000:.1f} km")
            ctx.advance(start)
            client.engage(BACKEND_ENGAGE[start], timeout=180.0)
        state.engaged = True
        ctx.log.log(f"Guidance engaged ({'engage' if start == Phase.DEORBIT else BACKEND_ENGAGE[start]})")

        deadline = time.monotonic() + float(landing["flightTimeoutSeconds"])
        while True:
            if ctx.stop_event.is_set():
                client.send("abort", timeout=30.0)
                state.terminal = True
                return Outcome(False, "landing aborted by operator; backend safe-control path applied", aborted=True)
            if client.process.poll() is not None:
                raise BackendError(f"landing backend exited with code {client.process.returncode}")
            if time.monotonic() > deadline:
                raise TimeoutError("landing exceeded landing.flightTimeoutSeconds")
            client.pump(0.5)
            state.flush(client)
            phase = (client.latest_snapshot or {}).get("phase")
            if phase == "Complete":
                state.terminal = True
                return Outcome(True, "rollout complete; vehicle stopped")
            if phase in BACKEND_FAILURE_PHASES:
                state.terminal = True
                detail = (client.latest_snapshot or {}).get("lastError") or (client.latest_snapshot or {}).get("statusMessage")
                return Outcome(False, f"backend entered {phase}: {detail}")


class _LandingState:
    """Translate backend snapshots into mission phases, telemetry and checkpoints."""

    def __init__(self, ctx: Context) -> None:
        self.ctx = ctx
        self.engaged = False
        self.terminal = False
        self.last_sample = 0.0
        self.last_backend_phase: str | None = None
        self.last_status = ""
        self.last_warning: str | None = None
        self.pending_saves: list[Phase] = []
        self.save_phases = {Phase.parse(str(p)) for p in ctx.config["checkpoints"]["kspSaveOnPhases"]}

    def on_snapshot(self, snap: dict[str, Any]) -> None:
        ctx = self.ctx
        backend_phase = snap.get("phase")
        if backend_phase != self.last_backend_phase:
            self.last_backend_phase = backend_phase
            ctx.log.emit(Event("backend-phase", f"backend phase: {backend_phase}"))
            mission_phase = BACKEND_PHASES.get(str(backend_phase))
            if mission_phase is not None and self.engaged:
                before = ctx.log.status.phase
                ctx.advance(mission_phase)
                if before != mission_phase.value and mission_phase in self.save_phases:
                    self.pending_saves.append(mission_phase)
        status = str(snap.get("statusMessage") or "")
        if status != self.last_status:
            self.last_status = status
            ctx.log.update(status_message=status)
        warning = snap.get("warningMessage")
        if warning != self.last_warning:
            self.last_warning = warning
            if warning:
                ctx.log.log(str(warning), "warning")
            else:
                ctx.log.update(warning=None)
        conn = snap.get("connectionStatus")
        if conn and conn != ctx.log.status.connection:
            ctx.log.update(connection=conn)
            if conn == "failed":
                ctx.log.log(str(snap.get("lastError") or "kRPC connection lost"), "error")
        now = time.monotonic()
        if now - self.last_sample >= 0.5:
            self.last_sample = now
            ctx.log.telemetry(snapshot_summary(snap), "landing")

    def flush(self, client: BackendClient) -> None:
        """Run deferred backend requests (never re-enter send() from the snapshot sink)."""
        while self.pending_saves:
            phase = self.pending_saves.pop(0)
            name = f"ksl-{self.ctx.log.status.run_id}-{phase.value}"
            try:
                client.send("saveCheckpoint", timeout=30.0, name=name)
                self.ctx.checkpoint(phase, name)
            except (BackendError, TimeoutError) as exc:
                self.ctx.log.log(f"KSP checkpoint {name} failed: {exc}", "warning")


# --- landing (offline ShuttleSim) --------------------------------------------------

# Backend phase that marks the start of each mission phase (for --stop-at-phase).
SIM_STOP_AT = {Phase.DEORBIT: "Deorbit Burn", Phase.MM304: "MM304 Entry", Phase.MM305: "TAEM",
               Phase.FINAL_APPROACH: "Final Approach", Phase.TOUCHDOWN: "Touchdown", Phase.ROLLOUT: "Rollout"}
_PROGRESS = re.compile(
    r"^phase=(?P<phase>.+?) alt=(?P<altitude>\S+) speed=(?P<speed>\S+) range=(?P<rangeToSite>\S+) "
    r"along=(?P<runwayAlongTrack>\S+) cross=(?P<runwayCrossTrack>\S+) aoa=(?P<angleOfAttack>\S+) bank=(?P<roll>\S+)$")


class SimulationProgram(Program):
    """Deorbit/entry/landing closed loop: native backend + ShuttleSim plant."""
    name = "simulation"
    phases = LandingProgram.phases

    def _argv(self, ctx: Context, scenario: Scenario, stop: Phase) -> list[str]:
        sim = ctx.config["simulation"]
        argv = [ctx.python, "-u", str(SHUTTLESIM / "scripts" / "run_guidance.py"),
                "--label", f"ksl-{scenario.name}", "--no-mirror", "--web-telemetry-port", "0",
                "--scenario", str(scenario.paths()[0]), "--configuration", str(scenario.paths()[1]),
                "--engage", scenario.engage, "--telemetry-hz", str(sim["telemetryHz"]),
                "--max-sim-time", str(sim["maxSimTimeSeconds"]), *scenario.extra_args]
        if scenario.attitude:
            argv += ["--attitude", str(scenario.paths()[2])]
        if scenario.direct_control:
            argv.append("--direct-control")
        if stop in SIM_STOP_AT:
            argv += ["--stop-at-phase", SIM_STOP_AT[stop]]
        return argv

    def describe(self, ctx: Context, start: Phase, stop: Phase) -> list[str]:
        scenario = ctx.scenario
        assert scenario is not None
        return [f"simulation: ShuttleSim scenario {scenario.name} ({scenario.description})",
                f"simulation: KSP_LANDER_MODEL_PROFILE={scenario.model_profile}, engage {scenario.engage}",
                "  $ " + " ".join(self._argv(ctx, scenario, stop))]

    def run(self, ctx: Context, start: Phase, stop: Phase) -> Outcome:
        scenario = ctx.scenario
        if scenario is None:
            raise ProgramUnavailable("simulation mode needs a scenario (`ksl simulate --list`)")
        missing = [str(p) for p in scenario.paths() if not p.is_file()]
        if missing:
            raise ProgramUnavailable("scenario inputs missing: " + ", ".join(missing))
        ctx.log.update(connection="simulator")
        ctx.advance(scenario.start)
        last_json: dict[str, Any] = {}

        def on_line(line: str) -> None:
            nonlocal last_json
            match = _PROGRESS.match(line)
            if match:
                fields = match.groupdict()
                backend_phase = fields.pop("phase")
                mission_phase = BACKEND_PHASES.get(backend_phase)
                if mission_phase is not None:
                    ctx.advance(mission_phase)
                sample: dict[str, Any] = {k: float(v) for k, v in fields.items()}
                sample["backendPhase"] = backend_phase
                ctx.log.telemetry(sample, "simulation")
                return
            if line.startswith("{") and line.endswith("}"):
                try:
                    last_json = json.loads(line)
                except json.JSONDecodeError:
                    pass
            elif line.startswith(("plan rejected", "engagement rejected")):
                ctx.log.log(line, "error")

        env = {"KSP_LANDER_MODEL_PROFILE": scenario.model_profile}
        code, tail = run_child(ctx, self.name, self._argv(ctx, scenario, stop), env, lambda _e: None, on_line)
        if ctx.stop_event.is_set():
            return Outcome(False, "simulation stopped by operator", aborted=True)
        if not last_json:
            return Outcome(False, f"simulation produced no result (exit {code}): " + (tail[-1] if tail else ""))
        ctx.log.write_json("simulation-result.json", last_json)
        # A phase shorter than one progress interval can be missed live; the
        # summary's observed phases close any forward gap (never backward).
        observed = sorted({BACKEND_PHASES[n] for n in last_json.get("observedPhases") or [] if n in BACKEND_PHASES},
                          key=lambda p: p.index)
        current = Phase(ctx.log.status.phase)
        for phase in observed:
            if phase.index > current.index and not (last_json.get("stoppedAtPhase") and phase.index > stop.index):
                ctx.advance(phase)
        summary = last_json.get("simulatorSummary") or {}
        if summary.get("touchdown"):
            ctx.log.emit(Event("milestone", "touchdown: {:.1f} m/s, sink {:.2f} m/s, along {:.0f} m, cross {:.1f} m, "
                               "on runway={}, gear={}".format(
                                   summary.get("touchdown_speed_mps", 0), summary.get("touchdown_sink_mps", 0),
                                   summary.get("touchdown_along_m", 0), summary.get("touchdown_cross_m", 0),
                                   summary.get("on_runway"), summary.get("touchdown_gear")), data=summary))
        if last_json.get("stoppedAtPhase"):
            return Outcome(True, f"stopped at {last_json['stoppedAtPhase']} as requested", holding=True)
        if last_json.get("success"):
            return Outcome(True, "landing and rollout passed the simulator acceptance gates")
        failed = [k for k in ("touchdownGearDown", "touchdownSpeedOk", "touchdownSinkOk", "rolloutValid",
                              "rolloutRunwayValid") if last_json.get(k) is False]
        reason = last_json.get("abortReason") or ("failed gates: " + ", ".join(failed) if failed else "not successful")
        return Outcome(False, f"simulation did not pass acceptance ({reason})")
