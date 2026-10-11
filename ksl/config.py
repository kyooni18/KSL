"""Mission profiles: one JSON document configures a whole mission.

Resolution order (later wins):
    built-in DEFAULTS  <-  Configuration/missions/<profile>.json  <-  --set overrides

The profile references, rather than copies, the native landing backend's own
configuration (``landing.backendConfiguration``). Its kRPC endpoint is
overridden from the profile's single ``connection`` block so the ascent
programs and the C backend always talk to the same server.
"""

from __future__ import annotations

import copy
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .paths import PROFILES_DIR, repo_path

DEFAULT_PROFILE = "sts-kerbin"

DEFAULTS: dict[str, Any] = {
    "description": "",
    "connection": {
        "address": "127.0.0.1",
        "rpcPort": 50000,
        "streamPort": 50001,
        "timeoutSeconds": 10.0,
    },
    "ascent": {
        # Circular target orbit; ascent inserts into altitude x (altitude - insertionOffsetKm).
        "targetAltitudeKm": 90.0,
        "inclinationDeg": 34.0,
        "insertionOffsetKm": 30.0,
        "minimumInsertionPeriapsisKm": 55.0,
        # Pad preparation (values flown by the former shuttle.mission entry point).
        "srbThrustLimit": 0.9,
        "ssmeThrustLimit": 0.38,
        "padSettleSeconds": 25.0,
        "revertToLaunch": False,
        # Extra raw flags for ksl.ascent.flight (e.g. ["--max-q", "22000"]).
        "extraArgs": [],
    },
    "orbit": {
        "trim": True,             # run OMS trim to the circular target after insertion
        "minBurnDeltaV": 0.1,     # m/s; planned corrections at or below this are not flown
        "extraArgs": [],
    },
    "landing": {
        "backendConfiguration": "Configuration/sts-live.json",
        "modelProfile": "sts-live-20261010",
        "autoWarp": True,
        "requireStrictPlan": False,
        "planTimeoutSeconds": 600.0,
        "flightTimeoutSeconds": 7200.0,
    },
    "simulation": {
        "scenario": "l5-mm305",
        "telemetryHz": 10.0,
        "maxSimTimeSeconds": 2400.0,
    },
    "checkpoints": {
        # KSP saves written at these phase entries during live missions.
        "kspSaveOnPhases": ["orbit"],
    },
}


class ConfigError(ValueError):
    pass


def deep_merge(base: dict[str, Any], update: dict[str, Any]) -> dict[str, Any]:
    out = copy.deepcopy(base)
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def parse_override(text: str) -> tuple[list[str], Any]:
    """Parse ``section.key=value``; value is JSON when it parses, else a string."""
    if "=" not in text:
        raise ConfigError(f"override {text!r} must look like section.key=value")
    key, raw = text.split("=", 1)
    path = [p for p in key.strip().split(".") if p]
    if not path:
        raise ConfigError(f"override {text!r} has an empty key")
    try:
        value = json.loads(raw)
    except json.JSONDecodeError:
        value = raw
    return path, value


def apply_override(data: dict[str, Any], path: list[str], value: Any) -> None:
    node = data
    for part in path[:-1]:
        if not isinstance(node.get(part), dict):
            raise ConfigError(f"unknown configuration section {'.'.join(path[:-1])!r}")
        node = node[part]
    if path[-1] not in node:
        raise ConfigError(f"unknown configuration key {'.'.join(path)!r}")
    node[path[-1]] = value


def available_profiles(directory: Path = PROFILES_DIR) -> list[str]:
    return sorted(p.stem for p in directory.glob("*.json")) if directory.is_dir() else []


def profile_path(name: str, directory: Path = PROFILES_DIR) -> Path:
    candidate = Path(name).expanduser()
    if candidate.suffix == ".json" and candidate.is_file():
        return candidate
    path = directory / f"{name}.json"
    if not path.is_file():
        known = ", ".join(available_profiles(directory)) or "none"
        raise ConfigError(f"unknown mission profile {name!r} (known: {known})")
    return path


@dataclass
class MissionConfig:
    name: str
    data: dict[str, Any]
    source: Path | None = None

    def __getitem__(self, section: str) -> dict[str, Any]:
        return self.data[section]

    # --- derived values used by several programs ----------------------------
    @property
    def target_altitude_m(self) -> float:
        return float(self.data["ascent"]["targetAltitudeKm"]) * 1000.0

    @property
    def insertion_periapsis_m(self) -> float:
        a = self.data["ascent"]
        return 1000.0 * max(float(a["minimumInsertionPeriapsisKm"]),
                            float(a["targetAltitudeKm"]) - float(a["insertionOffsetKm"]))

    def backend_configuration(self) -> dict[str, Any]:
        """Native landing configuration with the profile's kRPC endpoint applied."""
        path = repo_path(self.data["landing"]["backendConfiguration"])
        try:
            landing = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            raise ConfigError(f"cannot read landing configuration {path}: {exc}") from exc
        if not isinstance(landing, dict) or any(
                key in landing and not isinstance(landing[key], dict) for key in ("connection", "guidance")):
            raise ConfigError(f"{path}: landing configuration, connection and guidance must be objects")
        conn = self.data["connection"]
        landing.setdefault("connection", {}).update({
            "rpcHost": conn["address"],
            "rpcPort": int(conn["rpcPort"]),
        })
        landing.setdefault("guidance", {})["useTimeWarp"] = bool(self.data["landing"]["autoWarp"])
        return landing

    def validate(self) -> list[str]:
        """Return a list of problems; empty when the profile is usable."""
        problems = _schema_problems(self.data, DEFAULTS)
        if problems:
            return problems
        a, c = self.data["ascent"], self.data["connection"]
        if a["targetAltitudeKm"] < 80.0:
            problems.append("ascent.targetAltitudeKm must be at least 80 (Kerbin atmosphere ends at 70 km)")
        if not -180.0 <= a["inclinationDeg"] <= 180.0:
            problems.append("ascent.inclinationDeg must be within [-180, 180]")
        for key in ("srbThrustLimit", "ssmeThrustLimit"):
            if not 0.0 < a[key] <= 1.0:
                problems.append(f"ascent.{key} must be within (0, 1]")
        for key in ("rpcPort", "streamPort"):
            if not isinstance(c[key], int) or not 0 < c[key] < 65536:
                problems.append(f"connection.{key} must be an integer within 1-65535")
        if c["rpcPort"] == c["streamPort"]:
            problems.append("connection RPC and stream ports must differ")
        for section, key in (("connection", "timeoutSeconds"), ("landing", "planTimeoutSeconds"),
                             ("landing", "flightTimeoutSeconds"), ("simulation", "telemetryHz"),
                             ("simulation", "maxSimTimeSeconds")):
            if self.data[section][key] <= 0:
                problems.append(f"{section}.{key} must be positive")
        for section, key in (("ascent", "padSettleSeconds"), ("ascent", "insertionOffsetKm"),
                             ("ascent", "minimumInsertionPeriapsisKm"), ("orbit", "minBurnDeltaV")):
            if self.data[section][key] < 0:
                problems.append(f"{section}.{key} must be nonnegative")
        if self.insertion_periapsis_m > self.target_altitude_m:
            problems.append("ascent insertion periapsis must not exceed target altitude")
        for section, key in (("connection", "address"), ("landing", "backendConfiguration"),
                             ("landing", "modelProfile"), ("simulation", "scenario")):
            if not self.data[section][key].strip():
                problems.append(f"{section}.{key} must not be empty")
        landing_path = repo_path(self.data["landing"]["backendConfiguration"])
        if not landing_path.is_file():
            problems.append(f"landing.backendConfiguration not found: {landing_path}")
        else:
            try:
                self.backend_configuration()
            except ConfigError as exc:
                problems.append(str(exc))
        from .phases import Phase
        for phase in self.data["checkpoints"]["kspSaveOnPhases"]:
            try:
                Phase.parse(str(phase))
            except ValueError as exc:
                problems.append(f"checkpoints.kspSaveOnPhases: {exc}")
        return problems


def _schema_problems(data: Any, schema: Any, prefix: str = "") -> list[str]:
    """Validate structure before conversions or flight-side effects."""
    if isinstance(schema, dict):
        if not isinstance(data, dict):
            return [f"{prefix.rstrip('.') or 'profile'} must be an object"]
        problems = []
        for key in data.keys() - schema.keys() - ({"name"} if not prefix else set()):
            problems.append(f"unknown configuration key {prefix + key}")
        for key, default in schema.items():
            if key not in data:
                problems.append(f"missing configuration key {prefix + key}")
            else:
                problems.extend(_schema_problems(data[key], default, prefix + key + "."))
        return problems
    name = prefix.rstrip(".")
    if isinstance(schema, bool):
        valid, expected = isinstance(data, bool), "a boolean"
    elif isinstance(schema, (int, float)):
        valid = (isinstance(data, (int, float)) and not isinstance(data, bool)
                 and math.isfinite(data))
        expected = "a finite number"
    elif isinstance(schema, list):
        valid = isinstance(data, list) and all(isinstance(item, str) for item in data)
        expected = "a list of strings"
    else:
        valid, expected = isinstance(data, str), "a string"
    return [] if valid else [f"{name} must be {expected}"]


def config_from_snapshot(name: str, data: dict[str, Any], overrides: list[str] | None = None) -> MissionConfig:
    """Restore the actual run configuration, independent of later profile edits."""
    resolved = deep_merge(DEFAULTS, data)
    for text in overrides or []:
        path, value = parse_override(text)
        apply_override(resolved, path, value)
    return MissionConfig(name=name, data=resolved)


def load_config(profile: str | None = None, overrides: list[str] | None = None,
                directory: Path = PROFILES_DIR) -> MissionConfig:
    name = profile or DEFAULT_PROFILE
    source: Path | None = None
    data = copy.deepcopy(DEFAULTS)
    if name != "defaults":
        source = profile_path(name, directory)
        try:
            raw = json.loads(source.read_text())
        except json.JSONDecodeError as exc:
            raise ConfigError(f"{source}: invalid JSON: {exc}") from exc
        if not isinstance(raw, dict):
            raise ConfigError(f"{source}: profile must be a JSON object")
        unknown = set(raw) - set(DEFAULTS) - {"name"}
        if unknown:
            raise ConfigError(f"{source}: unknown sections {sorted(unknown)}")
        data = deep_merge(data, raw)
        name = str(raw.get("name") or source.stem)
    for text in overrides or []:
        path, value = parse_override(text)
        apply_override(data, path, value)
    return MissionConfig(name=name, data=data, source=source)
