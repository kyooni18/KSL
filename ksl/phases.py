"""Mission flight phases and the explicit phase state machine.

The mission runs forward through ``SEQUENCE``. Phases are owned by different
programs (Python ascent guidance, the OMS trim program, and the native C
landing backend); this module is the single place that maps their own phase
vocabularies onto mission phases.
"""

from __future__ import annotations

from enum import Enum


class Phase(str, Enum):
    PRELAUNCH = "prelaunch"
    LAUNCH = "launch"
    ASCENT = "ascent"
    ORBIT = "orbit"
    DEORBIT = "deorbit"
    MM304 = "mm304"
    MM305 = "mm305"
    FINAL_APPROACH = "final-approach"
    TOUCHDOWN = "touchdown"
    ROLLOUT = "rollout"
    COMPLETE = "complete"

    @property
    def title(self) -> str:
        return _TITLES[self]

    @property
    def index(self) -> int:
        return SEQUENCE.index(self)

    @classmethod
    def parse(cls, text: str) -> "Phase":
        key = text.strip().lower().replace("_", "-").replace(" ", "-")
        for phase in cls:
            if key in (phase.value, phase.name.lower().replace("_", "-")):
                return phase
        aliases = {"pad": cls.PRELAUNCH, "final": cls.FINAL_APPROACH, "taem": cls.MM305,
                   "entry": cls.MM304, "landed": cls.ROLLOUT, "done": cls.COMPLETE}
        if key in aliases:
            return aliases[key]
        raise ValueError(f"unknown phase {text!r}; expected one of: "
                         + ", ".join(p.value for p in cls))


_TITLES = {
    Phase.PRELAUNCH: "Prelaunch",
    Phase.LAUNCH: "Launch",
    Phase.ASCENT: "Ascent",
    Phase.ORBIT: "Orbit",
    Phase.DEORBIT: "Deorbit",
    Phase.MM304: "MM304 Entry",
    Phase.MM305: "MM305 TAEM",
    Phase.FINAL_APPROACH: "Final Approach",
    Phase.TOUCHDOWN: "Touchdown",
    Phase.ROLLOUT: "Rollout",
    Phase.COMPLETE: "Complete",
}

SEQUENCE: tuple[Phase, ...] = tuple(Phase)

# Phases a mission may be started from, and the vehicle state each one needs.
# Launch and ascent cannot be entered mid-flight: the ascent program owns the
# pad-to-orbit sequence (clamps, SRB/ET staging) and must start on the pad.
ENTRY_PHASES: dict[Phase, str] = {
    Phase.PRELAUNCH: "STS on the launch pad, clamps attached",
    Phase.ORBIT: "post-MECO / ET-separated orbiter in orbit (OMS trim, then landing)",
    Phase.DEORBIT: "orbiter in a stable orbit; plan and fly the deorbit burn",
    Phase.MM304: "post-deorbit-burn orbiter before or during entry",
    Phase.MM305: "orbiter in TAEM (energy management toward the HAC)",
    Phase.FINAL_APPROACH: "orbiter aligned on final approach to KSC runway 09",
}


class MissionState(str, Enum):
    """Lifecycle of a mission run, orthogonal to its flight phase."""
    READY = "ready"
    RUNNING = "running"
    HOLDING = "holding"      # stopped deliberately at a requested phase
    COMPLETE = "complete"
    ABORTED = "aborted"
    FAILED = "failed"

    @property
    def finished(self) -> bool:
        return self in (MissionState.HOLDING, MissionState.COMPLETE,
                        MissionState.ABORTED, MissionState.FAILED)


class PhaseTransitionError(RuntimeError):
    pass


class PhaseMachine:
    """Forward-only phase tracker. Skipping ahead is allowed (a program may
    report a later phase without every intermediate one); moving back is not."""

    def __init__(self, start: Phase) -> None:
        self.phase = start
        self.history: list[Phase] = [start]

    def advance(self, target: Phase) -> bool:
        """Move to ``target``; return True if the phase changed."""
        if target == self.phase:
            return False
        if target.index < self.phase.index:
            raise PhaseTransitionError(
                f"illegal backward transition {self.phase.value} -> {target.value}")
        self.phase = target
        self.history.append(target)
        return True


def validate_range(start: Phase, stop: Phase) -> None:
    if start not in ENTRY_PHASES:
        raise PhaseTransitionError(
            f"a mission cannot start at {start.value}; valid start phases: "
            + ", ".join(p.value for p in ENTRY_PHASES))
    if stop.index < start.index:
        raise PhaseTransitionError(f"stop phase {stop.value} precedes start phase {start.value}")


# --- Native landing backend (CLanding phase_string) -> mission phase ----------
# Phases absent from this map (Idle, Paused, Abort, Fault, Attitude Recovery)
# are statuses, not progress, and leave the mission phase unchanged.
BACKEND_PHASES: dict[str, Phase] = {
    "Planning": Phase.DEORBIT,
    "Calibration Glide": Phase.DEORBIT,
    "Coast to Burn": Phase.DEORBIT,
    "Burn Setup": Phase.DEORBIT,
    "Deorbit Burn": Phase.DEORBIT,
    "Entry Interface": Phase.MM304,
    "MM304 Entry": Phase.MM304,
    "TAEM": Phase.MM305,
    "Heading Alignment": Phase.MM305,
    "Final Approach": Phase.FINAL_APPROACH,
    "Flare": Phase.FINAL_APPROACH,
    "Touchdown": Phase.TOUCHDOWN,
    "Rollout": Phase.ROLLOUT,
    "Complete": Phase.COMPLETE,
}
BACKEND_FAILURE_PHASES = frozenset({"Abort", "Fault"})

# Backend engage method for each landing entry phase.
BACKEND_ENGAGE: dict[Phase, str] = {
    Phase.DEORBIT: "engage",            # createPlan first, then deorbit burn
    Phase.MM304: "engageReentry",
    Phase.MM305: "engageReentry",
    Phase.FINAL_APPROACH: "engageFinalTest",
}

# --- Ascent program structured events -> mission phase ------------------------
ASCENT_EVENTS: dict[str, Phase] = {
    "preflight": Phase.PRELAUNCH,
    "ignition": Phase.LAUNCH,
    "liftoff": Phase.LAUNCH,
    "gravity-turn": Phase.ASCENT,
    "booster-separation": Phase.ASCENT,
    "meco": Phase.ORBIT,
    "et-separation": Phase.ORBIT,
    "insertion": Phase.ORBIT,
}
