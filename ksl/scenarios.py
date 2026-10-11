"""Named offline ShuttleSim scenarios for ``ksl simulate``.

Each entry pins the inputs ``ShuttleSim/scripts/run_guidance.py`` needs for a
reproducible closed-loop run of the native landing backend against the
ShuttleSim plant. ``baseline`` records the outcome measured on the
integration base commit so regressions are visible; it is not a claim that
the scenario passes acceptance.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from .paths import SHUTTLESIM, repo_path
from .phases import Phase

L5 = "Docs/Experiments/STS-Landing-20261010"


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    start: Phase
    scenario: str                       # ShuttleSim .ini (repo-relative)
    configuration: str                  # landing backend configuration JSON
    model_profile: str                  # KSP_LANDER_MODEL_PROFILE
    attitude: str | None = None         # plant attitude model override
    direct_control: bool = False        # fly the native FCS against the moment plant
    baseline: str = ""
    extra_args: tuple[str, ...] = field(default_factory=tuple)

    @property
    def engage(self) -> str:
        from .phases import BACKEND_ENGAGE
        return BACKEND_ENGAGE[self.start]

    def paths(self) -> list[Path]:
        return [repo_path(p) for p in (self.scenario, self.configuration, self.attitude) if p]


SCENARIOS: dict[str, Scenario] = {s.name: s for s in (
    Scenario(
        name="l5-mm305",
        description="STS from the archived L5 live MM305 checkpoint through Final, flare, touchdown and rollout (native FCS).",
        start=Phase.MM305,
        scenario=f"{L5}/L5-72km/mm305-continuous.ini",
        configuration=f"{L5}/L5-72km/configuration.json",
        model_profile="sts-live-20261010",
        attitude=f"{L5}/Codex-Flare-Offline/inputs/att-live.ini",
        direct_control=True,
        baseline="Complete on runway; touchdown 60.7 m/s, sink 4.94 m/s (fails sink gate), stop 989 m",
    ),
    Scenario(
        name="entry-70km-sts",
        description="STS at the 70 km entry interface (MM304) through TAEM to landing.",
        start=Phase.MM304,
        scenario="ShuttleSim/scenarios/ksp70km-entry-interface.ini",
        configuration="Configuration/sts.json",
        model_profile="sts-live-20261010",
        baseline="see Docs/STS-Campaign-2026-10-08.md: entry sensitivity probes abort before TAEM",
    ),
    Scenario(
        name="postburn-86km",
        description="STS-N after the deorbit burn from an 86 km orbit (MM304 onward, ideal attitude servo).",
        start=Phase.MM304,
        scenario="ShuttleSim/scenarios/ksp86km-postburn.ini",
        configuration="Configuration/default.json",
        model_profile="identified",
        baseline="touches down 90.6 km downrange / 18.7 km cross-track, gear up (not a landing)",
    ),
    Scenario(
        name="deorbit-86km",
        description="STS-N in an 86 km orbit before the burn: deorbit planning, burn, entry and landing.",
        start=Phase.DEORBIT,
        scenario="ShuttleSim/scenarios/ksp86km-preburn.ini",
        configuration="Configuration/default.json",
        model_profile="identified",
    ),
    Scenario(
        name="final-aligned",
        description="STS-N aligned on final at 185 m/s, FPA -10 deg (Final approach, flare, rollout).",
        start=Phase.FINAL_APPROACH,
        scenario="ShuttleSim/scenarios/final-aligned-185mps-fpa10-aoa3.ini",
        configuration="Configuration/default.json",
        model_profile="identified",
    ),
)}


def resolve_scenario(name_or_path: str, start: Phase | None = None,
                     configuration: str | None = None, model_profile: str | None = None) -> Scenario:
    """Look up a named scenario, or wrap a raw ShuttleSim .ini path."""
    if name_or_path in SCENARIOS:
        return SCENARIOS[name_or_path]
    path = Path(name_or_path).expanduser()
    if not path.is_file():
        candidate = SHUTTLESIM / "scenarios" / name_or_path
        path = candidate if candidate.is_file() else candidate.with_suffix(".ini")
    if not path.is_file():
        raise KeyError(f"unknown scenario {name_or_path!r}; named scenarios: {', '.join(SCENARIOS)}")
    return Scenario(name=path.stem, description=f"ShuttleSim scenario {path.name}",
                    start=start or Phase.MM304, scenario=str(path.resolve()),
                    configuration=configuration or "Configuration/default.json",
                    model_profile=model_profile or "identified")
