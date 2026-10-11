"""Mission controller: sequences phase programs through the mission state machine.

The controller is the shared backend of the CLI and the TUI. It owns the run
directory, the phase machine, checkpoints and the abort path; programs own
the flight software for their phases.
"""

from __future__ import annotations

import threading
import time
from dataclasses import asdict
from pathlib import Path
from typing import Callable

from .backend import BackendError
from .config import ConfigError, MissionConfig
from .krpc_link import ConnectionFailure
from .kspguard import GuardError
from .phases import ENTRY_PHASES, MissionState, Phase, PhaseMachine, PhaseTransitionError, validate_range
from .programs import (AscentProgram, Context, LandingProgram, OrbitTrimProgram, Outcome, Program,
                       ProgramUnavailable, SimulationProgram)
from .runlog import Event, MissionStatus, RunLog, new_run_id
from .scenarios import Scenario

MODES = ("live", "sim", "dry-run")

# Exit codes shared by the CLI and scripts driving it.
EXIT_OK, EXIT_FAILED, EXIT_USAGE, EXIT_CONNECTION, EXIT_ABORTED = 0, 1, 2, 3, 130


def holdable(stop: Phase, mode: str) -> bool:
    """Phases a mission may deliberately stop at.

    Live: on the pad (read-only preflight), in orbit, or after rollout. Powered
    ascent and the unpowered glide cannot be paused mid-phase. The simulator
    can stop at the start of any landing phase.
    """
    if stop in (Phase.PRELAUNCH, Phase.ORBIT, Phase.COMPLETE):
        return True
    return mode in ("sim", "dry-run") and stop.index >= Phase.DEORBIT.index


class MissionController:
    def __init__(self, config: MissionConfig, mode: str = "live", start: Phase = Phase.PRELAUNCH,
                 stop: Phase = Phase.COMPLETE, scenario: Scenario | None = None,
                 runs_root: Path | None = None, label: str | None = None,
                 resumed_from: str | None = None) -> None:
        if mode not in MODES:
            raise ConfigError(f"unknown mode {mode!r}; expected one of {MODES}")
        if mode == "sim":
            if scenario is None:
                raise ConfigError("simulation mode requires a scenario")
            start = scenario.start
        validate_range(start, stop)
        if not holdable(stop, mode):
            raise PhaseTransitionError(
                f"cannot stop at {stop.value} in {mode} mode: powered ascent and the unpowered glide "
                "cannot be held; stop at prelaunch, orbit or complete")
        problems = config.validate()
        if problems:
            raise ConfigError("invalid mission profile:\n  " + "\n  ".join(problems))
        self.config = config
        self.mode = mode
        self.start = start
        self.stop = stop
        self.scenario = scenario
        self.machine = PhaseMachine(start)
        self.stop_event = threading.Event()
        run_id = new_run_id(label or (scenario.name if scenario else config.name))
        status = MissionStatus(run_id=run_id, profile=config.name, mode=mode, phase=start.value,
                               start_phase=start.value, stop_phase=stop.value,
                               phase_history=[start.value])
        self.log = RunLog.create(status, runs_root)
        self.log.write_json("config.json", {"profile": config.name, "mode": mode, "start": start.value,
                                            "stop": stop.value, "scenario": scenario.name if scenario else None,
                                            "resumedFrom": resumed_from, "configuration": config.data})

    # --- public API -----------------------------------------------------------
    @property
    def status(self) -> MissionStatus:
        return self.log.status

    @property
    def run_dir(self) -> Path:
        return self.log.directory

    def subscribe(self, callback: Callable[[Event], None]) -> None:
        self.log.subscribe(callback)

    def plan(self) -> list[tuple[Program, Phase]]:
        """Programs to run, each with the phase it starts from."""
        steps: list[tuple[Program, Phase]] = []
        if self.start == Phase.PRELAUNCH:
            steps.append((AscentProgram(), Phase.PRELAUNCH))
            if self.stop == Phase.PRELAUNCH:
                return steps
        if self.start.index <= Phase.ORBIT.index <= self.stop.index and self.mode != "sim":
            steps.append((OrbitTrimProgram(), Phase.ORBIT))
        if self.stop.index >= Phase.DEORBIT.index:
            landing_start = self.start if self.start.index >= Phase.DEORBIT.index else Phase.DEORBIT
            program = SimulationProgram() if self.mode == "sim" else LandingProgram()
            steps.append((program, landing_start))
        return steps

    def abort(self) -> None:
        """Request the safe stop path (thread-safe; returns immediately)."""
        if not self.stop_event.is_set():
            self.stop_event.set()
            self.log.log("Abort requested by operator", "warning")

    def run(self) -> MissionStatus:
        log = self.log
        log.update(state=MissionState.RUNNING.value, started_at=time.time())
        log.emit(Event("state", f"Mission {self.status.run_id}: {self.mode} {self.start.value} -> {self.stop.value}"
                       + (f" (scenario {self.scenario.name})" if self.scenario else "")))
        ctx = Context(self.config, log, self.mode, self.stop_event, self._advance, self._checkpoint, self.scenario)
        outcome = Outcome(True)
        try:
            for program, phase in self.plan():
                if self.stop_event.is_set():
                    return self._finish(MissionState.ABORTED, "stopped before starting the next program")
                log.update(program=program.name)
                if self.mode == "dry-run":
                    for line in program.describe(ctx, phase, self.stop):
                        log.log(line)
                    continue
                log.emit(Event("state", f"Starting {program.name} at {phase.title}"))
                outcome = program.run(ctx, phase, self.stop)
                log.emit(Event("state", f"{program.name}: {outcome.message}",
                               level="info" if outcome.ok else "error"))
                if not outcome.ok or outcome.holding:
                    break
            if self.mode == "dry-run":
                return self._finish(MissionState.HOLDING, "dry run: plan validated, nothing flown")
            if outcome.aborted:
                return self._finish(MissionState.ABORTED, outcome.message)
            if not outcome.ok:
                return self._finish(MissionState.FAILED, outcome.message)
            if self.machine.phase == Phase.COMPLETE:
                return self._finish(MissionState.COMPLETE, outcome.message or "mission complete")
            return self._finish(MissionState.HOLDING, outcome.message or f"holding at {self.machine.phase.title}")
        except ConnectionFailure as exc:
            log.update(connection="failed")
            return self._finish(MissionState.FAILED, f"connection failure: {exc}")
        except (ProgramUnavailable, BackendError, GuardError, PhaseTransitionError, ConfigError,
                TimeoutError, OSError) as exc:
            return self._finish(MissionState.FAILED, f"{type(exc).__name__}: {exc}")
        except KeyboardInterrupt:
            self.abort()
            return self._finish(MissionState.ABORTED, "interrupted")
        except Exception as exc:
            # Program owners release controls in finally before arriving here.
            # Persist unexpected failures so the TUI never shows a dead worker
            # as an actively flying mission.
            import traceback
            log.log(traceback.format_exc(), "error")
            return self._finish(MissionState.FAILED, f"{type(exc).__name__}: {exc}")
        finally:
            log.close()

    def start_background(self) -> threading.Thread:
        thread = threading.Thread(target=self.run, name="ksl-mission", daemon=True)
        thread.start()
        return thread

    def exit_code(self) -> int:
        state = MissionState(self.status.state)
        if state in (MissionState.COMPLETE, MissionState.HOLDING):
            return EXIT_OK
        if state == MissionState.ABORTED:
            return EXIT_ABORTED
        if self.status.connection == "failed":
            return EXIT_CONNECTION
        return EXIT_FAILED

    # --- callbacks used by programs ---------------------------------------------
    def _advance(self, phase: Phase) -> None:
        try:
            changed = self.machine.advance(phase)
        except PhaseTransitionError as exc:
            self.log.log(str(exc), "warning")   # stale report from a program; keep forward-only
            return
        if changed:
            self.log.update(phase=phase.value, phase_history=[p.value for p in self.machine.history])
            self.log.emit(Event("phase", f"Phase: {phase.title}", phase=phase.value))

    def _checkpoint(self, phase: Phase, ksp_save: str | None) -> None:
        if ksp_save is None and phase.value in {str(p).lower() for p in self.config["checkpoints"]["kspSaveOnPhases"]} \
                and self.mode == "live":
            from .krpc_link import save_game
            ksp_save = f"ksl-{self.status.run_id}-{phase.value}"
            try:
                save_game(self.config, ksp_save)
            except ConnectionFailure as exc:
                self.log.log(f"KSP save {ksp_save} failed: {exc}", "warning")
                ksp_save = None
        record = {"runId": self.status.run_id, "profile": self.config.name, "phase": phase.value,
                  "kspSave": ksp_save, "time": time.time(), "telemetry": self.status.telemetry}
        self.log.write_json("checkpoint.json", record)
        self.log.emit(Event("checkpoint", f"Checkpoint at {phase.title}" + (f" (KSP save {ksp_save})" if ksp_save else ""),
                            data=record))

    def _finish(self, state: MissionState, message: str) -> MissionStatus:
        level = "info" if state in (MissionState.COMPLETE, MissionState.HOLDING) else "error"
        fields = {"state": state.value, "status_message": message, "finished_at": time.time()}
        if level == "error":
            fields["error"] = message
        self.log.update(**fields)
        self.log.emit(Event("state", f"Mission {state.value}: {message}", level=level))
        return self.status


def continue_phase(checkpoint_phase: Phase) -> Phase:
    """Entry phase that resumes a mission from a phase checkpoint/status."""
    if checkpoint_phase.index <= Phase.ASCENT.index:
        raise PhaseTransitionError("an interrupted launch or ascent cannot be resumed in flight; "
                                   "revert to launch and start a new mission")
    if checkpoint_phase == Phase.ORBIT:
        return Phase.DEORBIT
    if checkpoint_phase in (Phase.TOUCHDOWN, Phase.ROLLOUT, Phase.COMPLETE):
        raise PhaseTransitionError(f"nothing to continue after {checkpoint_phase.title}")
    if checkpoint_phase not in ENTRY_PHASES:
        raise PhaseTransitionError(f"cannot resume at {checkpoint_phase.title}")
    return checkpoint_phase


def status_dict(status: MissionStatus) -> dict:
    return asdict(status)
