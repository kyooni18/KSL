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
import uuid
from dataclasses import dataclass
from typing import Any, Callable

from .backend import BackendClient, BackendError, snapshot_summary, validate_plan_qualification
from .config import MissionConfig
from .krpc_link import ConnectionFailure, connect, endpoint_args, krpc_env, wait_for_flight_scene
from .kspguard import GuardError, ensure_payload_bay_closed, prepare_reentry_direct_control, probe_orbiter_integrity
from .paths import BACKEND, DATA_ROOT, ROOT, SHUTTLESIM
from .phases import BACKEND_ENGAGE, BACKEND_FAILURE_PHASES, BACKEND_PHASES, ASCENT_EVENTS, Phase
from .runlog import Event, RunLog
from .scenarios import Scenario
from .commands import Commands
from .plan_review import invalid_reason

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
    review_plan: bool = False

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
            try:
                for line in proc.stdout:
                    lines.put(line.rstrip("\n"))
            finally:
                lines.put(None)

        thread = threading.Thread(target=reader, name=f"{name}-reader", daemon=True)
        thread.start()
        stop_sent_at: float | None = None
        output_closed = False
        parent_exited_at: float | None = None
        try:
            while not output_closed or proc.poll() is None:
                if proc.poll() is not None and parent_exited_at is None:
                    parent_exited_at = time.monotonic()
                if not output_closed and parent_exited_at is not None and \
                        time.monotonic() - parent_exited_at > 5.0:
                    raise BackendError(f"{name}: program exited but a descendant kept its output pipe open")
                if ctx.stop_event.is_set() and stop_sent_at is None:
                    ctx.log.log(f"{name}: stop requested; sending SIGINT for the safe-release path", "warning")
                    _signal_child_group(proc, signal.SIGINT)
                    stop_sent_at = time.monotonic()
                if stop_sent_at is not None:
                    elapsed = time.monotonic() - stop_sent_at
                    if elapsed > STOP_GRACE_SECONDS + 5.0:
                        _signal_child_group(proc, signal.SIGKILL)
                    elif elapsed > STOP_GRACE_SECONDS:
                        _signal_child_group(proc, signal.SIGTERM)
                try:
                    line = lines.get(timeout=0.2)
                except queue.Empty:
                    continue
                if line is None:
                    output_closed = True
                    continue
                raw_log.write(line + "\n")
                raw_log.flush()
                if line.startswith(EVENT_PREFIX):
                    try:
                        event = json.loads(line[len(EVENT_PREFIX):])
                    except json.JSONDecodeError:
                        continue
                    if not isinstance(event, dict):
                        raise BackendError(f"{name}: milestone must be a JSON object")
                    on_event(event)
                    continue
                tail = (tail + [line])[-20:]
                if on_line is not None:
                    on_line(line)
                ctx.log.emit(Event("program", line, level="debug", data={"program": name}))
            return proc.wait(), tail
        finally:
            # Includes callback failures and KeyboardInterrupt: never abandon a
            # detached flight owner. Signal its session so simulator descendants
            # cannot retain pipes or survive an interrupted harness.
            if proc.poll() is None or thread.is_alive():
                _signal_child_group(proc, signal.SIGINT)
                try:
                    proc.wait(timeout=STOP_GRACE_SECONDS)
                except subprocess.TimeoutExpired:
                    _signal_child_group(proc, signal.SIGTERM)
                    try:
                        proc.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        _signal_child_group(proc, signal.SIGKILL)
                        proc.wait(timeout=5.0)
            # A crashed parent may have left descendants holding stdout open.
            thread.join(timeout=0.5)
            if thread.is_alive():
                _signal_child_group(proc, signal.SIGKILL)
                thread.join(timeout=2.0)
            if proc.stdout is not None and not thread.is_alive():
                proc.stdout.close()


def _signal_child_group(proc: subprocess.Popen, sig: int) -> None:
    """Children start their own session; signal only that owned process group."""
    try:
        os.killpg(proc.pid, sig)
    except ProcessLookupError:
        pass


# --- profile targets and runtime controls -------------------------------------------

def controls_env(ctx: Context) -> dict[str, str]:
    """Child programs poll the run's controls.json (TUI warp/trim toggles)."""
    from .controls import ENV_VAR, Controls
    path = Controls.for_run(ctx.log.directory, {
        "autoWarp": bool(ctx.config["operations"]["autoWarp"]),
        "trim": bool(ctx.config["orbit"]["trim"]),
        "holdTrim": bool(ctx.config["orbit"]["holdForTrigger"]),
    }).path
    return {ENV_VAR: str(path)}


def semi_gto_apoapsis_m(config: MissionConfig) -> float:
    """ET target apoapsis of a GTO/GEO ascent (designed for the GEO longitude if asked)."""
    from .ascent import geo
    pr = config["profile"]
    if pr["semiGtoApoapsisKm"] > 0:
        return float(pr["semiGtoApoapsisKm"]) * 1000.0
    body = geo.KERBIN
    perigee = float(pr["gtoPerigeeKm"]) * 1000.0
    if pr["kind"] == "geo" and pr["targetLongitude"]:
        design = geo.design_semi_gto_apoapsis(
            body["mu"], body["radius"], perigee, geo.synchronous_altitude(body),
            float(pr["insertionLongitudeDeg"]), float(pr["geoLongitudeDeg"]), geo.rotation_rate_deg(body))
        if design is not None and design["apoapsis_alt"] <= 1500000.0:
            return design["apoapsis_alt"]
    return 800000.0


def ascent_target_args(config: MissionConfig) -> tuple[str, list[str]]:
    """(description, flight.py target flags) for the profile kind."""
    pr, a = config["profile"], config["ascent"]
    kind = pr["kind"]
    if kind in ("gto", "geo"):
        perigee = float(pr["gtoPerigeeKm"]) * 1000.0
        apo = semi_gto_apoapsis_m(config)
        return (f"semi-GTO {apo / 1000:.0f} x {perigee / 1000:.0f} km at {a['inclinationDeg']} deg",
                ["--orbit", f"{apo:.0f}", "--meco-periapsis", f"{perigee:.0f}", "--transfer",
                 "--reference-altitude", f"{max(80000.0, perigee):.0f}",
                 "--insertion-altitude", f"{perigee:.0f}", "--inclination", str(a["inclinationDeg"])])
    flags = ["--orbit", f"{config.target_altitude_m:.0f}",
             "--meco-periapsis", f"{config.insertion_periapsis_m:.0f}",
             "--inclination", str(a["inclinationDeg"])]
    text = (f"{a['targetAltitudeKm']:.0f} x {config.insertion_periapsis_m / 1000:.0f} km "
            f"at {a['inclinationDeg']} deg")
    if kind in ("plane", "rendezvous"):
        flags += ["--plane-target", str(pr["planeTarget"])]
        if pr["launchWindow"]:
            flags += ["--launch-window", "--window-lead", str(pr["windowLeadSeconds"])]
        text = (f"{a['targetAltitudeKm']:.0f} x {config.insertion_periapsis_m / 1000:.0f} km in the plane of "
                f"{pr['planeTarget']}" + (" (launch window)" if pr["launchWindow"] else ""))
    return text, flags


# --- ascent ------------------------------------------------------------------------

class AscentProgram(Program):
    name = "ascent"
    phases = (Phase.PRELAUNCH, Phase.LAUNCH, Phase.ASCENT)

    def _argv(self, ctx: Context, preflight_only: bool) -> list[str]:
        a = ctx.config["ascent"]
        if a["guidanceMode"] == "intelligent":
            argv = [ctx.python, "-u", "-m", "ksl.ascent.intelligent.flight",
                    "--config", str(ctx.log.directory / "config.json"),
                    "--telemetry", str(ctx.log.directory / "ascent.csv")]
            return argv if preflight_only else argv + ["--launch"]
        argv = [ctx.python, "-u", "-m", "ksl.ascent.flight", *endpoint_args(ctx.config),
                *ascent_target_args(ctx.config)[1], "--ssme-limit", str(a["ssmeThrustLimit"]),
                "--telemetry", str(ctx.log.directory / "ascent.csv"), *map(str, a["extraArgs"])]
        if not preflight_only:
            argv.insert(4, "--launch")
        return argv

    def describe(self, ctx: Context, start: Phase, stop: Phase) -> list[str]:
        a = ctx.config["ascent"]
        if a["guidanceMode"] == "intelligent":
            periapsis = ctx.config["intelligentAscent"]["targetPeriapsisKm"] or a["targetAltitudeKm"]
            return ["ascent: intelligent vehicle discovery" +
                    (" (read-only)" if stop == Phase.PRELAUNCH else
                     f" and predictive ascent to {a['targetAltitudeKm']:g} x {periapsis:g} km / {a['inclinationDeg']:g} deg"),
                    "  $ " + " ".join(self._argv(ctx, stop == Phase.PRELAUNCH))]
        if stop == Phase.PRELAUNCH:
            return ["ascent: read-only preflight of the STS on the pad (no control writes)",
                    "  $ " + " ".join(self._argv(ctx, True))]
        steps = []
        if a["revertToLaunch"]:
            steps.append("ascent: revert the current flight to launch")
        steps += [f"ascent: wait for the flight scene, "
                  f"set Clydesdale limiters to {a['srbThrustLimit']}",
                  f"ascent: fly to {ascent_target_args(ctx.config)[0]} "
                  "(SSME lead, SRB/ET separation, corridor-tracked gravity turn, PEG MECO)",
                  "  $ " + " ".join(self._argv(ctx, False))]
        return steps

    def _pad_prep(self, ctx: Context) -> None:
        a = ctx.config["ascent"]
        if a["revertToLaunch"]:
            ctx.log.log("Reverting flight to launch")
            with connect(ctx.config, "ksl-revert") as conn:
                conn.space_center.revert_to_launch()
        wait_for_flight_scene(ctx.config)
        ctx.log.update(connection="connected")
        if ctx.stop_event.is_set():
            raise InterruptedError("stopped on the pad before ignition")
        if a["guidanceMode"] == "intelligent":
            return
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
        failure_message = ""

        def on_event(event: dict[str, Any]) -> None:
            nonlocal failure_message
            name = str(event.pop("event", ""))
            if name == "guidance-failure":
                failure_message = str(event.get("error", "guidance failed")).splitlines()[0]
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
        env = {**krpc_env(ctx.config), **controls_env(ctx)}
        code, tail = run_child(ctx, self.name, self._argv(ctx, preflight_only), env, on_event)
        if ctx.stop_event.is_set():
            return Outcome(False, "ascent stopped by operator; controls released", aborted=True)
        if code != 0:
            return Outcome(False, f"ascent program exited with code {code}: " + (failure_message or (tail[-1] if tail else "")))
        if preflight_only:
            return Outcome(True, "preflight complete", holding=True)
        if not seen & {"insertion", "et-separation"}:
            return Outcome(False, "ascent program ended without orbital insertion")
        return Outcome(True, "orbital insertion complete")


# --- orbit ---------------------------------------------------------------------------

def established_orbit_retarget(config: MissionConfig) -> str | None:
    """Refuse to silently move an orbit this run did not insert.

    A run started at Orbit from a saved checkpoint does not inherit the
    original run's --set overrides; on 2026-10-15 a 90 km polar orbit was about
    to be trimmed to the profile's 100 km default.  Circular profiles only.
    """
    if config["profile"]["kind"] != "circular":
        return None
    from .krpc_link import probe
    try:
        info = probe(config)
    except Exception:  # noqa: BLE001 - the trim child reports connection faults itself
        return None
    ap, pe = info.get("apoapsis"), info.get("periapsis")
    depth = info.get("atmosphereDepth") or 0.0
    if info.get("situation") != "orbiting" or ap is None or pe is None or pe <= depth:
        return None
    target = config.target_altitude_m
    # The profile's own achieved-orbit altitude tolerance decides "far".
    if max(abs(ap - target), abs(pe - target)) <= config["intelligentAscent"]["altitudeToleranceM"]:
        return None
    return (f"orbit trim refused: the current orbit ({ap / 1000:.1f} x {pe / 1000:.1f} km) is far from the "
            f"configured {target / 1000:.0f} km target. A saved checkpoint does not restore the original run's "
            f"overrides; re-apply them (--set ascent.targetAltitudeKm=...) or use `ksl continue`.")


class OrbitTrimProgram(Program):
    """Orbit operations for the profile kind, run as one or more child stages."""
    name = "orbit-trim"
    phases = (Phase.ORBIT,)

    def stages(self, ctx: Context) -> list[tuple[str, str, list[str]]]:
        """(stage name, description, argv) in flight order."""
        c, pr, o = ctx.config, ctx.config["profile"], ctx.config["orbit"]
        operation = getattr(getattr(ctx.log, "status", None), "operation", "mission")
        if c["ascent"]["guidanceMode"] == "intelligent" and operation != "orbit":
            # Intelligent ascent owns and verifies its full circular insertion.
            return []
        inc = str(c["ascent"]["inclinationDeg"])
        common = [*endpoint_args(c), "--min-burn-dv", str(o["minBurnDeltaV"]),
                  "--rcs-max-dv", str(o["rcsMaxDeltaV"])]
        if pr["kind"] in ("gto", "geo"):
            argv = [ctx.python, "-u", "-m", "ksl.ascent.geo_transfer", "--perigee",
                    f"{float(pr['gtoPerigeeKm']) * 1000.0:.0f}", "--inclination", inc, *common,
                    "--circularize", "never" if pr["kind"] == "gto" else str(pr["geoCircularize"]),
                    "--reserve-dv", str(pr["reserveDeltaV"])]
            if pr["kind"] == "geo" and pr["targetLongitude"]:
                argv += ["--longitude", str(pr["geoLongitudeDeg"])]
            text = ("GTO: perigee phasing" if pr["kind"] == "geo" and pr["targetLongitude"] else "GTO") + \
                ", apogee raise to synchronous altitude" + \
                ("" if pr["kind"] == "gto" else
                 f", circularize at apogee ({pr['geoCircularize']})"
                 + (f" over {pr['geoLongitudeDeg']} deg" if pr["targetLongitude"] else ""))
            return [("geo-transfer", text, argv)]
        alt = f"{c.target_altitude_m:.0f}"
        trim = [ctx.python, "-u", "-m", "ksl.ascent.orbit_trim", "--apoapsis", alt, "--periapsis", alt,
                "--inclination", inc, *common, "--max-burns", str(o["maxBurns"]), *map(str, o["extraArgs"])]
        if pr["kind"] in ("plane", "rendezvous"):
            trim.append("--keep-inclination")  # the launch window already set the plane
            inc = f"the plane of {pr['planeTarget']}"
        steps = [("orbit-trim", f"OMS trim to {c['ascent']['targetAltitudeKm']:.0f} km circular at {inc} deg "
                  f"(exact one-burn plan, at most {o['maxBurns']} burns)", trim)]
        if pr["kind"] == "rendezvous":
            steps.append(("rendezvous", f"rendezvous with {pr['planeTarget']}: Lambert transfer to "
                          f"{pr['standoffM']:.0f} m behind, midcourse, velocity match, RCS approach to "
                          f"{pr['stopDistanceM']:.0f} m",
                          [ctx.python, "-u", "-m", "ksl.ascent.rendezvous", "--target", str(pr["planeTarget"]),
                           "--standoff", str(pr["standoffM"]), "--stop-distance", str(pr["stopDistanceM"]),
                           *common]))
        return steps

    def describe(self, ctx: Context, start: Phase, stop: Phase) -> list[str]:
        if not ctx.config["orbit"]["trim"]:
            return ["orbit: orbit operations disabled (orbit.trim=false; toggle live from the TUI)"]
        lines = []
        if ctx.config["orbit"]["holdForTrigger"]:
            lines.append("orbit: hold until the operator triggers orbit operations")
        for name, text, argv in self.stages(ctx):
            lines += [f"orbit: {text}", "  $ " + " ".join(argv)]
        return lines

    def run(self, ctx: Context, start: Phase, stop: Phase) -> Outcome:
        ctx.advance(Phase.ORBIT)
        if ctx.mode == "sim":
            raise ProgramUnavailable("no offline orbital plant exists for the OMS trim program")

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
                return
            detail = ", ".join(f"{k}={v:.2f}" if isinstance(v, float) else f"{k}={v}" for k, v in event.items())
            ctx.log.emit(Event("milestone", f"orbit: {name}" + (f" ({detail})" if detail else ""), data=event))

        env = {**krpc_env(ctx.config), **controls_env(ctx)}
        operation = getattr(getattr(ctx.log, "status", None), "operation", "mission")
        if ctx.config["orbit"]["trim"] and start == Phase.ORBIT and operation != "orbit":
            refusal = established_orbit_retarget(ctx.config)
            if refusal:
                return Outcome(False, refusal)
        if ctx.config["orbit"]["trim"]:
            for name, _, argv in self.stages(ctx):
                code, tail = run_child(ctx, name, argv, env, on_event)
                if ctx.stop_event.is_set():
                    return Outcome(False, f"{name} stopped by operator", aborted=True)
                if code != 0:
                    return Outcome(False, f"{name} exited with code {code}: " + (tail[-1] if tail else ""))
        ctx.checkpoint(Phase.ORBIT, None)
        return Outcome(True, "orbit operations complete", holding=stop == Phase.ORBIT)


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
                 f"landing: connect to kRPC {ctx.config['connection']['address']}:{ctx.config['connection']['rpcPort']}",
                 "landing: inspect and close Benjee STS payload bay before deorbit/reentry"]
        if start == Phase.DEORBIT:
            steps += ["landing: createPlan, record entry/terminal confidence; "
                      + ("explicit strict mode" if landing["requireStrictPlan"]
                         else "attempt best available plan under diagnostic warnings"),
                      "landing: engage -> coast, deorbit burn, MM304 entry, MM305 TAEM, final, flare, rollout"]
            if ctx.review_plan:
                steps.insert(-1, "landing: wait for explicit plan execution; recheck vessel, orbit and burn window")
        else:
            steps += ["landing: verified direct-control handoff (SAS off, zero throttle, 1x warp)",
                      f"landing: {BACKEND_ENGAGE[start]} from {start.title}"]
        return steps

    def run(self, ctx: Context, start: Phase, stop: Phase) -> Outcome:
        if ctx.mode != "live":
            raise ProgramUnavailable("LandingProgram flies live KSP; use SimulationProgram offline")
        landing = ctx.config["landing"]
        start = Phase.DEORBIT if start.index < Phase.DEORBIT.index else start
        env = {"KSP_LANDER_MODEL_PROFILE": str(landing["modelProfile"]),
               "KSP_LANDER_ROOT": str(DATA_ROOT), **krpc_env(ctx.config)}
        # The payload-bay preflight below closes the bay before any planning or
        # atmospheric flight, so aero history is keyed to that configuration.
        env["KSP_LANDER_PHYSICS_VARIANT"] = "payload-bay-closed"
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
        backend_configuration = ctx.config.backend_configuration()
        # The TUI auto-warp toggle [g] also governs the backend's coast warp to
        # the deorbit burn; it is followed live below.
        from .controls import ENV_VAR, Controls
        controls = Controls(controls_env(ctx)[ENV_VAR])
        auto_warp = controls.auto_warp
        backend_configuration["guidance"]["useTimeWarp"] = auto_warp
        ctx.log.write_json("backend-configuration.json", backend_configuration)
        client.send("updateConfiguration", configuration=backend_configuration)
        ctx.log.update(connection="connecting")
        client.send("connect", timeout=180.0)
        snap = client.latest_snapshot
        if not snap or snap.get("connectionStatus") not in ("connected", "failed"):
            snap = client.read_until(lambda s: s.get("connectionStatus") in ("connected", "failed"), 180.0)
        if snap.get("connectionStatus") != "connected":
            raise ConnectionFailure(str(snap.get("lastError") or "native kRPC connection failed"))
        ctx.log.log("Landing backend connected to KSP")
        # Inventory is evidence, not the guidance authority. Missing wings or
        # a temporarily unavailable second probe should not cancel a commanded
        # KSP flight that the native backend can still steer. Keep the warning.
        try:
            integrity = probe_orbiter_integrity(env)
        except (GuardError, TimeoutError, OSError) as exc:
            ctx.log.log(f"Orbiter preflight warning: {exc}; proceeding with best-effort guidance", "warning")
        else:
            ctx.log.log(f"Orbiter structural inventory verified: {len(integrity['partNames'])} parts")

        # The Benjee OV-100 payload bay may remain open after orbital ops.
        # Close it *before* planning/burning or resuming atmospheric guidance;
        # a blind Toggle command would reopen an already-closed bay. Flight
        # continues with a visible warning if the guard cannot verify closure.
        try:
            bay = ensure_payload_bay_closed(env=env)
        except (GuardError, TimeoutError, OSError) as exc:
            ctx.log.log(f"Payload-bay closure not verified: {exc}; proceeding with best-effort guidance", "warning")
        else:
            status = str(bay.get("status") or "unknown")
            if status in ("closed", "already-closed", "not-applicable"):
                ctx.log.log(f"Payload bay preflight: {status} ({bay.get('bayCount', 0)} bay part(s))")
            else:
                ctx.log.log(f"Payload bay preflight: {status} ({bay.get('reason') or 'animation still in progress'}); proceeding with best-effort guidance", "warning")

        if start == Phase.DEORBIT:
            ctx.advance(Phase.DEORBIT)
            if ctx.review_plan:
                if not self._review(ctx, client, float(backend_configuration["guidance"]["minimumPlanningLeadTime"])):
                    return Outcome(False, "deorbit review stopped; guidance was not engaged", aborted=True)
            else:
                self._create_plan(ctx, client)
                client.engage("engage", timeout=120.0)
        else:
            payload = prepare_reentry_direct_control(env=env)
            ctx.log.log(f"Direct-control handoff verified at {float(payload.get('meanAltitude') or 0) / 1000:.1f} km")
            ctx.advance(start)
            client.engage(BACKEND_ENGAGE[start], timeout=180.0)
        state.engaged = True
        ctx.log.update(operation_stage="flying")
        ctx.log.log(f"Guidance engaged ({'engage' if start == Phase.DEORBIT else BACKEND_ENGAGE[start]})")

        flight_start_ut = None
        previous_ut = None
        while True:
            if ctx.stop_event.is_set():
                client.send("abort", timeout=30.0)
                state.terminal = True
                return Outcome(False, "landing aborted by operator; backend safe-control path applied", aborted=True)
            if client.process.poll() is not None:
                raise BackendError(f"landing backend exited with code {client.process.returncode}")
            client.pump(0.5)
            telemetry = (client.latest_snapshot or {}).get("telemetry") or {}
            ut = telemetry.get("ut")
            if isinstance(ut, (int, float)) and math.isfinite(ut):
                if flight_start_ut is None or (previous_ut is not None and ut < previous_ut):
                    flight_start_ut = ut
                previous_ut = ut
                if ut-flight_start_ut > float(landing["flightTimeoutSeconds"]):
                    raise TimeoutError("landing exceeded landing.flightTimeoutSeconds")
            state.flush(client)
            if controls.auto_warp != auto_warp:
                auto_warp = controls.auto_warp
                backend_configuration["guidance"]["useTimeWarp"] = auto_warp
                client.send("updateConfiguration", configuration=backend_configuration)
                ctx.log.log(f"Coast time warp {'enabled' if auto_warp else 'disabled'} by operator")
            phase = (client.latest_snapshot or {}).get("phase")
            if phase == "Complete":
                state.terminal = True
                return Outcome(True, "rollout complete; vehicle stopped")
            if phase in BACKEND_FAILURE_PHASES:
                state.terminal = True
                detail = (client.latest_snapshot or {}).get("lastError") or (client.latest_snapshot or {}).get("statusMessage")
                return Outcome(False, f"backend entered {phase}: {detail}")

    def _create_plan(self, ctx: Context, client: BackendClient) -> dict[str, Any]:
        landing = ctx.config["landing"]
        ctx.log.log("Deorbit search runs with KSP unpaused (MM305 campaign)")
        previous = client.latest_snapshot
        client.send("createPlan", timeout=float(landing["planTimeoutSeconds"]))
        planned = client.latest_snapshot
        if planned is previous or not planned or not planned.get("plan"):
            planned = client.read_until(lambda s: bool(s.get("plan")) or s.get("phase") == "Fault", 30.0)
        if planned.get("phase") == "Fault" or planned.get("lastError"):
            raise BackendError(str(planned.get("lastError") or "planning failed"))
        plan = planned.get("plan") or {}
        if not plan:
            raise BackendError(str(planned.get("lastError") or "no deorbit plan was produced"))
        ctx.log.write_json("deorbit-plan.json", plan)
        ctx.log.emit(Event("milestone", "deorbit plan: burn UT {:.1f}, dv {:.2f} m/s, executable={}, strict={}".format(
            float(plan.get("burnUT") or 0), float(plan.get("deltaV") or 0),
            bool(plan.get("executionQualified")), bool(plan.get("targetCaptureAchieved"))), data=plan))
        warning = validate_plan_qualification(
            plan, bool(landing["requireStrictPlan"]) and not ctx.review_plan, allow_unqualified=True)
        if warning:
            ctx.log.log(warning, "warning")
            ctx.log.emit(Event("guidance-degraded", warning,
                               data={"executionQualified": plan.get("executionQualified"),
                                     "targetCaptureAchieved": plan.get("targetCaptureAchieved"),
                                     "terminalInletEnergyMargin": plan.get("terminalInletEnergyMargin")}))
        return plan

    def _review(self, ctx: Context, client: BackendClient, minimum_lead: float) -> bool:
        from .krpc_link import probe
        commands = Commands(ctx.log.directory)
        replanning_request = None
        while not ctx.stop_event.is_set():
            ctx.log.update(operation_stage="planning", plan_revision=None)
            reference = probe(ctx.config)
            plan = self._create_plan(ctx, client)
            if ctx.stop_event.is_set():
                return False
            revision = uuid.uuid4().hex
            ctx.log.write_json("plan-review.json", {"revision": revision, "plan": plan,
                                                    "reference": reference, "minimumLead": minimum_lead,
                                                    "createdAt": time.time()})
            ctx.log.update(operation_stage="review", plan_revision=revision,
                           status_message="Plan ready: review then Execute plan, or Replan. Guidance is not engaged.")
            ctx.log.emit(Event("plan-review", "Awaiting explicit deorbit execution", data={"revision": revision}))
            if replanning_request is not None:
                commands.update(replanning_request, "completed", "New plan ready for review")
                replanning_request = None
            replan = False
            while not ctx.stop_event.is_set() and not replan:
                client.pump(0.2)
                if (client.latest_snapshot or {}).get("connectionStatus") != "connected":
                    raise ConnectionFailure("Landing connection lost during plan review")
                ctx.log.write_status()
                for request in commands.pending():
                    action = request.get("action")
                    if action == "abort":
                        continue  # controller command monitor owns aborts
                    if action not in ("execute-plan", "replan"):
                        commands.update(request, "rejected", "Unknown plan command")
                        continue
                    if request.get("revision") != revision:
                        commands.update(request, "rejected", "The reviewed plan changed; review the current revision")
                        continue
                    commands.update(request, "accepted", "Checking planning state" if action == "execute-plan" else "Replanning")
                    if action == "replan":
                        replan = True
                        replanning_request = request
                        commands.update(request, "running", "Creating a new plan")
                        break
                    try:
                        current = probe(ctx.config)
                        reason = invalid_reason(plan, reference, current, minimum_lead)
                        if reason:
                            raise BackendError(reason)
                        validate_plan_qualification(plan, bool(ctx.config["landing"]["requireStrictPlan"]))
                        if ctx.stop_event.is_set():
                            raise BackendError("Abort requested; guidance will not engage")
                        client.engage("engage", timeout=120.0)
                    except (BackendError, ConnectionFailure, TimeoutError) as exc:
                        if (client.latest_snapshot or {}).get("automationEngaged"):
                            raise  # owning program's finally must release any partial engagement
                        commands.update(request, "rejected", str(exc))
                        ctx.log.log(str(exc), "warning")
                    else:
                        commands.update(request, "completed", "Reviewed plan engaged; continuous guidance through rollout")
                        return True
        return False


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
        self.pending_altitude_saves: list[tuple[str, float, float]] = []
        self.descent_thresholds = sorted((float(v) * 1000.0 for v in
            ctx.config["checkpoints"]["kspSaveOnDescendingAltitudesKm"]), reverse=True)
        self.last_descent_altitude: float | None = None
        self.descent_checkpoints: list[dict[str, Any]] = []

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
        telemetry = snap.get("telemetry") or {}
        altitude, sink = telemetry.get("meanAltitude"), telemetry.get("verticalSpeed")
        if self.engaged and isinstance(altitude, (int, float)) and math.isfinite(altitude):
            # A resumed descent can begin below an earlier checkpoint. That
            # checkpoint was never crossed by this controller; retire it without
            # inventing a save, so lower thresholds remain reachable.
            if (self.last_descent_altitude is None and isinstance(sink, (int, float))
                    and math.isfinite(sink) and sink < -5):
                while self.descent_thresholds and altitude <= self.descent_thresholds[0]:
                    target = self.descent_thresholds.pop(0)
                    ctx.log.log(f"Descending checkpoint at {target / 1000.0:g} km already passed at "
                                f"guidance start {altitude / 1000.0:.2f} km; not backfilled", "warning")
            # A crossing, not simply being below a target when reentry starts.
            if self.last_descent_altitude is not None and isinstance(sink, (int, float)) and sink < -5:
                while self.descent_thresholds and self.last_descent_altitude > self.descent_thresholds[0] >= altitude:
                    target = self.descent_thresholds.pop(0)
                    if target - altitude <= 2000.0:
                        label = f"descent-{target / 1000.0:g}km"
                        self.pending_altitude_saves.append((label, altitude, float(telemetry.get("ut") or 0)))
                    else:
                        ctx.log.log(f"Descending checkpoint at {target / 1000.0:g} km missed: first observed at {altitude / 1000.0:.2f} km", "warning")
            self.last_descent_altitude = float(altitude)
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
        while self.pending_altitude_saves:
            label, measured_altitude, ut = self.pending_altitude_saves.pop(0)
            name = f"ksl-{self.ctx.log.status.run_id}-{label}"
            try:
                client.send("saveCheckpoint", timeout=30.0, name=name)
            except (BackendError, TimeoutError) as exc:
                self.ctx.log.log(f"KSP altitude checkpoint {name} failed: {exc}", "warning")
                continue
            record = {"kspSave": name, "measuredAltitude": measured_altitude, "ut": ut,
                      "runId": self.ctx.log.status.run_id, "phase": self.ctx.log.status.phase}
            self.descent_checkpoints.append(record)
            self.ctx.log.write_json("descent-checkpoints.json", self.descent_checkpoints)
            self.ctx.log.emit(Event("checkpoint", f"Saved descending checkpoint {name} at {measured_altitude / 1000.0:.2f} km", data=record))


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
