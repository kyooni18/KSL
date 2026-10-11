"""Load recorded flights for replay, inspection and export.

Sources:
  * a KSL run directory (events.jsonl + telemetry.jsonl)
  * an ascent telemetry CSV written by ksl.ascent.flight (``ascent.csv``)
  * a ShuttleSim run directory or its guidance-snapshots/guidance-events JSONL(.gz)
"""

from __future__ import annotations

import csv
import gzip
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Iterator

from .phases import ASCENT_EVENTS, BACKEND_PHASES, Phase
from .runlog import Event, read_events, read_telemetry

# Ascent program internal states (flight.py `state`) -> mission phase.
# Below LAUNCH_CEILING_M the ASCENT state is reported as the Launch phase.
ASCENT_STATES = {"ASCENT": Phase.ASCENT, "ORBITER_ASCENT": Phase.ASCENT,
                 "COAST": Phase.ORBIT, "INSERTION": Phase.ORBIT}
LAUNCH_CEILING_M = 1000.0


@dataclass
class Recording:
    source: Path
    kind: str                                     # run | ascent-csv | simulation
    samples: list[dict[str, Any]] = field(default_factory=list)
    events: list[Event] = field(default_factory=list)

    @property
    def phases(self) -> list[str]:
        seen: list[str] = []
        for sample in self.samples:
            phase = sample.get("phase")
            if phase and (not seen or seen[-1] != phase):
                seen.append(phase)
        return seen

    def timeline(self) -> list[tuple[float, str]]:
        """(elapsed seconds, phase) at each phase change."""
        out: list[tuple[float, str]] = []
        clocked = self.clocked_samples()
        if not clocked:
            return out
        t0 = clocked[0][1]
        last = None
        if self.samples[0] is not clocked[0][0]:
            # The native backend emits an initial UT=0 frame with the entry
            # phase before its first live telemetry read. Keep that phase in
            # the timeline without treating its wall clock as simulation UT.
            last = self.samples[0].get("phase")
            if last:
                out.append((0.0, last))
        for sample, sample_time in clocked:
            phase = sample.get("phase")
            if phase and phase != last:
                out.append((sample_time - t0, phase))
                last = phase
        if last != Phase.COMPLETE.value and any(
            event.kind == "phase" and event.phase == Phase.COMPLETE.value
            for event in self.events
        ):
            out.append((clocked[-1][1] - t0, Phase.COMPLETE.value))
        return out

    def clocked_samples(self) -> list[tuple[dict[str, Any], float]]:
        """Use one clock, ignoring a backend's initial UT=0 placeholder."""
        first_ut = next((i for i, sample in enumerate(self.samples)
                         if _is_number(sample.get("ut")) and
                         (sample["ut"] > 0 or not _is_number(sample.get("time")))), None)
        if first_ut is not None:
            return [(sample, float(sample["ut"])) for sample in self.samples[first_ut:]
                    if _is_number(sample.get("ut")) and
                    (sample["ut"] > 0 or not _is_number(sample.get("time")))]
        return [(sample, float(sample["time"])) for sample in self.samples
                if _is_number(sample.get("time"))]

    def summary(self) -> dict[str, Any]:
        def peak(key: str) -> float | None:
            values = [float(s[key]) for s in self.samples if _is_number(s.get(key))]
            return max(values) if values else None
        last = self.samples[-1] if self.samples else {}
        clocked = self.clocked_samples()
        return {"source": str(self.source), "kind": self.kind, "samples": len(self.samples),
                "events": len(self.events),
                "duration_s": clocked[-1][1] - clocked[0][1] if clocked else 0.0,
                "phases": self.phases, "max_altitude_m": peak("altitude"), "max_speed_mps": peak("speed"),
                "max_dynamic_pressure_pa": peak("dynamicPressure"), "final": last}


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _open_text(path: Path):
    return gzip.open(path, "rt", encoding="utf-8") if path.suffix == ".gz" else path.open(encoding="utf-8")


def _jsonl(path: Path) -> Iterator[dict[str, Any]]:
    with _open_text(path) as handle:
        for line in handle:
            line = line.strip()
            if line:
                try:
                    yield json.loads(line)
                except json.JSONDecodeError:
                    continue


def _number(text: str) -> Any:
    try:
        value = float(text)
    except (TypeError, ValueError):
        return text
    return value if math.isfinite(value) else None


def load_ascent_csv(path: Path) -> Recording:
    rec = Recording(path, "ascent-csv")
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            state = (row.get("state") or "").upper()
            sample = {"ut": _number(row.get("ut", "")), "state": state,
                      "altitude": _number(row.get("altitude_m", "")),
                      "apoapsis": _number(row.get("apoapsis_m", "")),
                      "periapsis": _number(row.get("periapsis_m", "")),
                      "mass": _number(row.get("mass_kg", "")),
                      "verticalSpeed": _number(row.get("vertical_speed_mps", "")),
                      "horizontalSpeed": _number(row.get("horizontal_speed_mps", "")),
                      "dynamicPressure": _number(row.get("dynamic_pressure_pa", "")),
                      "gForce": _number(row.get("actual_g", "")),
                      "commandedG": _number(row.get("commanded_g", "")),
                      "pitch": _number(row.get("actual_pitch_deg", "")),
                      "heading": _number(row.get("actual_heading_deg", "")),
                      "roll": _number(row.get("actual_roll_deg", "")),
                      "inclination": _number(row.get("orbit_inclination_deg", ""))}
            phase = ASCENT_STATES.get(state, Phase.ASCENT)
            if phase == Phase.ASCENT and _is_number(sample["altitude"]) and sample["altitude"] < LAUNCH_CEILING_M:
                phase = Phase.LAUNCH
            sample["phase"] = phase.value
            vs, hs = sample["verticalSpeed"], sample["horizontalSpeed"]
            if _is_number(vs) and _is_number(hs):
                sample["speed"] = math.hypot(vs, hs)
            rec.samples.append({k: v for k, v in sample.items() if v not in ("", None)})
    return rec


def _snapshot_sample(snap: dict[str, Any]) -> dict[str, Any]:
    from .backend import snapshot_summary
    sample = snapshot_summary(snap)
    phase = BACKEND_PHASES.get(str(snap.get("phase")))
    if phase is not None:
        sample["phase"] = phase.value
    return sample


def load_simulation(path: Path) -> Recording:
    if path.is_dir():
        for name in ("guidance-snapshots.jsonl.gz", "guidance-snapshots.jsonl",
                     "guidance-events.jsonl.gz", "guidance-events.jsonl"):
            if (path / name).is_file():
                path = path / name
                break
        else:
            raise FileNotFoundError(f"no guidance snapshots in {path}")
    rec = Recording(path, "simulation")
    for obj in _jsonl(path):
        snap = obj.get("snapshot") if obj.get("type") == "snapshot" else obj if "telemetry" in obj else None
        if isinstance(snap, dict):
            rec.samples.append(_snapshot_sample(snap))
    return rec


def load_run(path: Path) -> Recording:
    rec = Recording(path, "run", samples=read_telemetry(path), events=read_events(path))
    return rec


def load(path: Path) -> Recording:
    path = Path(path).expanduser()
    if path.is_dir():
        if (path / "status.json").is_file():
            return load_run(path)
        return load_simulation(path)
    if path.suffix == ".csv":
        return load_ascent_csv(path)
    if ".jsonl" in path.name:
        return load_simulation(path)
    raise ValueError(f"unrecognized recording {path}")


def export(recording: Recording, fmt: str, out: Path | None) -> str:
    """Write samples as CSV or JSONL; return the text when ``out`` is None."""
    rows = recording.samples
    if fmt == "jsonl":
        text = "".join(json.dumps(r, separators=(",", ":"), default=str) + "\n" for r in rows)
    elif fmt == "csv":
        import io
        keys: list[str] = []
        for row in rows:
            keys += [k for k in row if k not in keys and not isinstance(row[k], (dict, list))]
        buffer = io.StringIO()
        writer = csv.DictWriter(buffer, fieldnames=keys, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
        text = buffer.getvalue()
    else:
        raise ValueError(f"unknown export format {fmt!r} (csv or jsonl)")
    if out is not None:
        out.write_text(text)
    return text


def ascent_milestones(lines: Iterable[str]) -> list[dict[str, Any]]:
    """Parse @@ksl milestone lines from an ascent program log."""
    out = []
    for line in lines:
        if line.startswith("@@ksl "):
            try:
                event = json.loads(line[6:])
            except json.JSONDecodeError:
                continue
            if event.get("event") in ASCENT_EVENTS:
                out.append(event)
    return out
