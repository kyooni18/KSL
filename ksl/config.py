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
PROFILE_KINDS = ("circular", "plane", "gto", "geo", "rendezvous")

DEFAULTS: dict[str, Any] = {
    "description": "",
    "connection": {
        "address": "127.0.0.1",
        "rpcPort": 50000,
        "streamPort": 50001,
        "timeoutSeconds": 10.0,
    },
    "ascent": {
        "guidanceMode": "legacy",
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
    "intelligentAscent": {
        "targetPeriapsisKm": 0.0,  # zero uses the ascent target altitude (circular)
        "vehicleConfig": "",
        "maxDynamicPressurePa": 25000.0,
        "maxG": 3.0,
        "altitudeToleranceM": 1000.0,
        "inclinationToleranceDeg": 0.2,
        "timeoutSeconds": 2400.0,
    },
    "orbit": {
        "trim": True,             # run OMS trim to the circular target after insertion
        "minBurnDeltaV": 0.1,     # m/s; planned corrections at or below this are not flown
        "maxBurns": 2,
        "rcsMaxDeltaV": 2.0,      # m/s; burns this small fly as RCS forward translation            # exact one-burn plan; a 2nd burn only finishes the residual
        "holdForTrigger": False,  # wait in orbit until the operator triggers orbit operations
        "extraArgs": [],
    },
    "profile": {
        # circular   ascent + OMS trim to targetAltitudeKm / inclinationDeg
        # plane      as circular, but into the orbital plane of planeTarget (launch window)
        # gto        ET semi-GTO, OMS raises apogee to synchronous altitude, stays in GTO
        # geo        as gto, then circularizes at apogee (if OMS dv allows) over geoLongitudeDeg
        # rendezvous plane launch to the target's plane, parking orbit, then rendezvous
        "kind": "circular",
        "planeTarget": "",            # moon or vessel name (plane / rendezvous)
        "launchWindow": True,         # hold on the pad for the plane launch window
        "windowLeadSeconds": 0.0,
        "gtoPerigeeKm": 85.0,
        "semiGtoApoapsisKm": 0.0,     # ET target apoapsis; 0 = design it from the longitude
        "geoLongitudeDeg": 0.0,
        "targetLongitude": True,      # phase the GTO so the apogee sits over geoLongitudeDeg
        "geoCircularize": "auto",     # auto (if OMS dv allows) | always | never
        "reserveDeltaV": 0.0,         # m/s of OMS dv kept when deciding to circularize
        "insertionLongitudeDeg": -60.0,  # estimated MECO/perigee longitude for semi-GTO design
        "standoffM": 1000.0,
        "stopDistanceM": 50.0,
    },
    "operations": {
        "autoWarp": True,         # initial runtime control; toggled live from the TUI
    },
    "landing": {
        "backendConfiguration": "Configuration/sts-live.json",
        "modelProfile": "sts-live-20261015",
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
        # One-shot native KSP saves while descending through these altitudes.
        "kspSaveOnDescendingAltitudesKm": [],
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
        pr = self.data["profile"]
        if a["guidanceMode"] not in ("legacy", "intelligent"):
            problems.append("ascent.guidanceMode must be legacy or intelligent")
        for key in ("maxDynamicPressurePa", "maxG", "altitudeToleranceM", "inclinationToleranceDeg", "timeoutSeconds"):
            if self.data["intelligentAscent"][key] <= 0:
                problems.append(f"intelligentAscent.{key} must be positive")
        if a["guidanceMode"] == "intelligent" and pr["kind"] != "circular":
            problems.append("intelligent ascent currently supports circular orbit profiles")
        if pr["kind"] not in PROFILE_KINDS:
            problems.append(f"profile.kind must be one of {', '.join(PROFILE_KINDS)}")
        if pr["kind"] in ("plane", "rendezvous") and not pr["planeTarget"].strip():
            problems.append(f"profile.planeTarget is required for a {pr['kind']} profile")
        if pr["kind"] in ("gto", "geo") and pr["gtoPerigeeKm"] < 75.0:
            problems.append("profile.gtoPerigeeKm must be at least 75 (above the atmosphere)")
        if not -180.0 <= pr["geoLongitudeDeg"] <= 180.0:
            problems.append("profile.geoLongitudeDeg must be within [-180, 180]")
        if pr["geoCircularize"] not in ("auto", "always", "never"):
            problems.append("profile.geoCircularize must be auto, always or never")
        if self.data["orbit"]["rcsMaxDeltaV"] < 0:
            problems.append("orbit.rcsMaxDeltaV must be nonnegative")
        if not isinstance(self.data["orbit"]["maxBurns"], int) or not 1 <= self.data["orbit"]["maxBurns"] <= 6:
            problems.append("orbit.maxBurns must be an integer 1-6")
        periapsis = self.data["intelligentAscent"]["targetPeriapsisKm"]
        if periapsis < 0 or periapsis > a["targetAltitudeKm"]:
            problems.append("intelligentAscent.targetPeriapsisKm must be between zero and target altitude")
        if a["guidanceMode"] == "intelligent" and a["targetAltitudeKm"] <= 0:
            problems.append("ascent.targetAltitudeKm must be positive")
        if a["guidanceMode"] == "legacy" and a["targetAltitudeKm"] < 80.0:
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
        if a["guidanceMode"] == "legacy" and self.insertion_periapsis_m > self.target_altitude_m:
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
        altitudes = self.data["checkpoints"]["kspSaveOnDescendingAltitudesKm"]
        for value in altitudes:
            try:
                altitude = float(value)
                if not math.isfinite(altitude) or not 1.0 <= altitude <= 150.0:
                    raise ValueError(value)
            except ValueError:
                problems.append(f"checkpoints.kspSaveOnDescendingAltitudesKm: invalid altitude {value!r}")
        if len(set(altitudes)) != len(altitudes):
            problems.append("checkpoints.kspSaveOnDescendingAltitudesKm: duplicate altitude")
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
