"""Shared flight operations: prerequisites, previews, controller construction."""
from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .config import ConfigError, MissionConfig, config_from_snapshot, load_config
from .controller import MissionController, continue_phase
from .phases import Phase, PhaseTransitionError
from .runlog import read_status
from .scenarios import Scenario


@dataclass(frozen=True)
class Operation:
    key: str
    title: str
    start: Phase
    stop: Phase
    description: str
    context: str


OPERATIONS = (
    Operation("preflight", "Run preflight", Phase.PRELAUNCH, Phase.PRELAUNCH,
              "Inspect the pad stack without commanding flight.", "pad"),
    Operation("launch", "Launch to orbit", Phase.PRELAUNCH, Phase.ORBIT,
              "Fly launch and ascent continuously; wait after orbital insertion.", "pad"),
    Operation("orbit", "Orbit operations / trim", Phase.ORBIT, Phase.ORBIT,
              "Run the profile's OMS trim, transfer or rendezvous; wait in orbit.", "orbit"),
    Operation("deorbit", "Plan deorbit", Phase.DEORBIT, Phase.COMPLETE,
              "Create a plan, review it, then explicitly execute through landing.", "orbit"),
    Operation("entry", "Engage MM304 entry", Phase.MM304, Phase.COMPLETE,
              "Take over entry and continue guidance through rollout.", "entry"),
    Operation("taem", "Engage MM305 TAEM", Phase.MM305, Phase.COMPLETE,
              "Take over energy management and continue through rollout.", "airborne"),
    Operation("final", "Engage final approach", Phase.FINAL_APPROACH, Phase.COMPLETE,
              "Take over an aligned final approach and continue through rollout.", "airborne"),
    Operation("continue", "Continue checkpoint", Phase.ORBIT, Phase.COMPLETE,
              "Restore the selected run's configuration and resume its checkpoint.", "checkpoint"),
    Operation("simulate", "Run simulation", Phase.MM305, Phase.COMPLETE,
              "Fly the selected offline ShuttleSim scenario.", "simulation"),
)
BY_KEY = {op.key: op for op in OPERATIONS}


def blocked_reason(operation: Operation, info: dict[str, Any] | None, mode: str,
                   busy: bool = False, source: Path | None = None, load_save: bool = False) -> str | None:
    if busy:
        return "A KSL operation owns this endpoint; watch it or release its controls first"
    if operation.key == "continue":
        if source is None:
            return "Select a recorded run in Runs first"
        try:
            _, _, checkpoint = resume(source)
            if load_save and not checkpoint.get("kspSave"):
                return "The selected checkpoint has no KSP save"
        except (OSError, ValueError, ConfigError, PhaseTransitionError) as exc:
            return str(exc)
    if mode == "sim":
        return None if operation.key == "simulate" else "Choose Run simulation in simulation mode"
    if operation.key == "simulate":
        return None if mode == "dry-run" else "Choose simulation mode"
    if mode == "dry-run":
        return None
    if operation.key == "continue" and load_save:
        return None  # controller verifies the save before checking the vessel
    if not info:
        return "Connect to KSP to inspect the active vessel"
    if info.get("scene") != "flight":
        return "Load a vessel in the flight scene"
    if time.time() - info.get("observedAt", 0) > 15:
        return "Vessel status is stale; refresh the connection"
    situation = info.get("situation")
    if operation.context == "pad":
        if not (situation in ("pre_launch", "prelaunch") or
                (situation == "landed" and info.get("launchClamps", 0) >= 2 and info.get("speed", 99) < 1)):
            return "Requires a stationary stack on launch clamps"
    elif operation.context == "orbit":
        atmosphere = info.get("atmosphereDepth")
        if atmosphere is None:
            return "Atmosphere boundary unavailable; refresh vessel status"
        if operation.key == "deorbit":
            if min(info.get("apoapsis", 0), info.get("periapsis", 0)) <= atmosphere:
                return "Deorbit planning requires both apsides above the atmosphere"
        elif situation not in ("orbiting", "sub_orbital") or info.get("altitude", 0) <= atmosphere:
            return "Requires a post-insertion orbiter above the atmosphere"
    elif operation.context in ("entry", "airborne"):
        if situation not in ("flying", "sub_orbital", "orbiting"):
            return "Requires an airborne orbiter; select the matching takeover phase"
    return None


def resume(source: Path, profile: str | None = None, overrides: list[str] | None = None):
    status = read_status(source)
    if status.state in ("ready", "running"):
        raise PhaseTransitionError("Stop the selected run before continuing it")
    if status.mode == "sim":
        raise PhaseTransitionError("Simulation checkpoints are not live saves; select a simulation scenario")
    path = source / "checkpoint.json"
    checkpoint = json.loads(path.read_text()) if path.is_file() else {}
    start = continue_phase(Phase.parse(checkpoint.get("phase") or status.phase))
    saved = json.loads((source / "config.json").read_text())
    if profile or not isinstance(saved.get("configuration"), dict):
        config = load_config(profile or saved.get("profile"), overrides)
    else:
        config = config_from_snapshot(saved.get("profile") or status.profile, saved["configuration"], overrides)
    return config, start, checkpoint


@dataclass
class OperationRequest:
    key: str
    config: MissionConfig
    mode: str = "dry-run"
    automatic: bool = False
    scenario: Scenario | None = None
    source: Path | None = None
    use_save: bool = False
    resume_profile: str | None = None
    resume_overrides: list[str] | None = None

    def resolve(self):
        op = BY_KEY[self.key]
        config, start, stop = self.config, op.start, op.stop
        save = None
        if self.key == "continue":
            if self.source is None:
                raise ConfigError("Select a checkpoint run first")
            config, start, checkpoint = resume(self.source, self.resume_profile, self.resume_overrides)
            save = checkpoint.get("kspSave") if self.use_save else None
            if self.use_save and not save:
                raise ConfigError("The selected checkpoint has no KSP save")
        if self.key == "simulate":
            if self.mode == "live" or self.scenario is None:
                raise ConfigError("Run simulation requires a scenario and sim/dry-run mode")
            start = self.scenario.start
        elif self.mode == "sim":
            raise ConfigError("Select Run simulation for sim mode")
        if self.automatic and self.key in ("launch", "orbit"):
            stop = Phase.COMPLETE
        if self.key == "orbit":
            config = config_from_snapshot(config.name, config.data,
                                          ["orbit.trim=true", "orbit.holdForTrigger=false"])
        return config, start, stop, save

    def preview(self) -> str:
        config, start, stop, save = self.resolve()
        problems = config.validate()
        if problems:
            raise ConfigError("; ".join(problems))
        op = BY_KEY[self.key]
        lines = [f"{op.title} · {self.mode} · {config.name}", op.description,
                 f"{start.title} → {stop.title}"]
        if self.key in ("preflight", "launch", "orbit"):
            lines.append(f"Target: {config['ascent']['targetAltitudeKm']:g} km / "
                         f"{config['ascent']['inclinationDeg']:g}° · {config['profile']['kind']}")
        if self.key == "simulate" and self.scenario:
            lines.append(f"Scenario: {self.scenario.name} — {self.scenario.description}")
        if self.key == "launch" and not self.automatic:
            lines.append("Orbit trim and deorbit wait for separate operator actions.")
        if self.automatic and self.key in ("launch", "orbit"):
            lines.append("Automatic: continue through orbit operations and landing without operator gates.")
        if self.key in ("entry", "taem", "final", "deorbit", "continue"):
            lines.append("Atmospheric guidance continues through rollout; no intermediate hold.")
        if save:
            lines.append(f"Load and verify KSP save: {save}")
        return "\n".join(lines)

    def build(self, runs_root: Path | None = None) -> MissionController:
        self.preview()
        config, start, stop, save = self.resolve()
        def check_vessel():
            from .krpc_link import probe
            reason = blocked_reason(BY_KEY[self.key], probe(config), "live", source=self.source)
            if reason:
                raise ConfigError(reason)
        return MissionController(config, self.mode, start, stop, scenario=self.scenario,
                                 runs_root=runs_root, label=self.key,
                                 resumed_from=self.source.name if self.source else None,
                                 operation=self.key, automatic=self.automatic if self.key in ("launch", "orbit") else False,
                                 load_save=save,
                                 precondition=check_vessel)
