#!/usr/bin/env python3
"""Headless operator for the native C shuttle-landing backend.

The backend remains the sole owner of flight control. This wrapper only speaks
its NDJSON operator protocol so live testing does not require the PyQt UI.
"""

from __future__ import annotations

import argparse
from functools import lru_cache
import fcntl
import json
import math
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time
from typing import Any, Callable, NamedTuple

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))
from campaign_provenance import (
    load_pinned_campaign_artifacts,
    pin_campaign_artifacts,
    verify_running_executable,
    write_run_manifest,
)


TERMINAL_PHASES = {"Complete", "Abort", "Fault"}


def live_unpowered_policy_error(
    live: bool, connect_only: bool, environ: dict[str, str] | os._Environ[str] | None = None
) -> str | None:
    """Return an error when a control-capable live run lacks the unpowered policy."""
    if not live or connect_only:
        return None
    env = os.environ if environ is None else environ
    if env.get("KSP_LANDER_UNPOWERED_ONLY") != "1":
        return (
            "control-capable --live runs require KSP_LANDER_UNPOWERED_ONLY=1; "
            "the native controller only hard-zeroes atmospheric throttle when this policy is explicit"
        )
    return None


def signed_angle_degrees(value: float) -> float:
    wrapped = float(value) % 360.0
    return wrapped - 360.0 if wrapped > 180.0 else wrapped


class DescentCheckpoint(NamedTuple):
    altitude: float
    name: str


def descent_checkpoint_plan(raw: list[list[str]] | None) -> tuple[DescentCheckpoint, ...]:
    """Validate and normalize repeatable descent checkpoint specifications."""
    checkpoints: list[DescentCheckpoint] = []
    seen_names: set[str] = set()
    seen_altitudes: set[float] = set()
    for altitude_text, raw_name in raw or []:
        try:
            altitude = float(altitude_text)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid checkpoint altitude: {altitude_text!r}") from exc
        name = str(raw_name).strip()
        if not math.isfinite(altitude) or altitude <= 0.0:
            raise ValueError(f"checkpoint altitude must be a positive finite value: {altitude_text!r}")
        if not name:
            raise ValueError("checkpoint name must not be empty")
        if name in seen_names:
            raise ValueError(f"duplicate checkpoint name: {name}")
        if altitude in seen_altitudes:
            raise ValueError(f"duplicate checkpoint altitude: {altitude:g}")
        seen_names.add(name)
        seen_altitudes.add(altitude)
        checkpoints.append(DescentCheckpoint(altitude, name))
    checkpoints.sort(key=lambda checkpoint: checkpoint.altitude, reverse=True)
    return tuple(checkpoints)


class DescentCheckpointTracker:
    """Emit each checkpoint only after the live descent has actually crossed it."""

    def __init__(self, checkpoints: tuple[DescentCheckpoint, ...]) -> None:
        self.checkpoints = checkpoints
        self.armed: set[str] = set()
        self.saved: set[str] = set()

    def update(self, altitude: float, vertical_speed: float) -> tuple[DescentCheckpoint, ...]:
        if not math.isfinite(altitude):
            return ()
        for checkpoint in self.checkpoints:
            if checkpoint.name not in self.saved and altitude > checkpoint.altitude:
                self.armed.add(checkpoint.name)
        if not math.isfinite(vertical_speed) or vertical_speed >= 0.0:
            return ()
        ready = [
            checkpoint
            for checkpoint in self.checkpoints
            if checkpoint.name in self.armed
            and checkpoint.name not in self.saved
            and altitude <= checkpoint.altitude
        ]
        if len(ready) > 1:
            crossed = ", ".join(f"{checkpoint.name}@{checkpoint.altitude:g}m" for checkpoint in ready)
            raise RuntimeError(
                "descent checkpoint telemetry skipped multiple armed thresholds; "
                f"refusing to mislabel saves at altitude {altitude:.0f}m: {crossed}"
            )
        if not ready:
            return ()
        checkpoint = ready[0]
        self.saved.add(checkpoint.name)
        return (checkpoint,)


def landing_completion_evidence(
    snapshot: dict[str, Any], configuration: dict[str, Any]
) -> tuple[bool, str]:
    """Independently verify runway touchdown and completed rollout evidence.

    A backend phase label is not sufficient acceptance evidence. Mirror the C
    rollout stop envelope and require either KSP's landed state or the same
    near-contact telemetry used by the controller's debounced ground latch.
    """
    if str(snapshot.get("phase", "")) != "Complete":
        return False, f"phase={snapshot.get('phase') or 'unknown'}"

    telemetry = snapshot.get("telemetry") or {}
    site = configuration.get("site") or {}

    def number(value: Any) -> float:
        return BackendProcess._number(value, float("nan"))

    surface_speed = number(telemetry.get("surfaceSpeed"))
    if not math.isfinite(surface_speed) or surface_speed >= 1.5:
        return False, f"surface-speed={surface_speed:.2f}m/s"

    runway_length = number(site.get("runwayLength"))
    runway_width = number(site.get("runwayWidth"))
    if not math.isfinite(runway_length) or runway_length <= 0.0:
        return False, "runway-length-missing"
    if not math.isfinite(runway_width) or runway_width <= 0.0:
        return False, "runway-width-missing"

    along = number(telemetry.get("runwayAlongTrack"))
    cross = number(telemetry.get("runwayCrossTrack"))
    cross_limit = max(runway_width * 0.75, 45.0)
    if not math.isfinite(along) or not math.isfinite(cross):
        return False, "runway-position-missing"
    if along < -40.0 or along >= runway_length + 80.0 or abs(cross) >= cross_limit:
        return False, (
            f"outside-runway-envelope along={along:.1f}m cross={cross:.1f}m "
            f"limits=[-40,{runway_length + 80.0:.0f})/+/-{cross_limit:.1f}m"
        )

    situation = str(telemetry.get("vesselSituation", "")).strip().lower()
    gear = telemetry.get("gear") is True
    if situation == "landed":
        if not gear:
            return False, "landing-gear-unconfirmed"
        return True, (
            f"landed surface-speed={surface_speed:.2f}m/s "
            f"along={along:.1f}m cross={cross:.1f}m"
        )

    radar_altitude = number(telemetry.get("radarAltitude"))
    vertical_speed = number(telemetry.get("verticalSpeed"))
    near_contact = (
        gear
        and math.isfinite(radar_altitude)
        and radar_altitude < 0.6
        and math.isfinite(vertical_speed)
        and vertical_speed > -3.0
        and abs(vertical_speed) < 3.0
    )
    if not near_contact:
        return False, (
            f"ground-contact-unverified situation={situation or 'unknown'} "
            f"gear={int(gear)} radar={radar_altitude:.2f}m vertical={vertical_speed:.2f}m/s"
        )
    return True, (
        f"debounced-contact surface-speed={surface_speed:.2f}m/s "
        f"along={along:.1f}m cross={cross:.1f}m"
    )


def taem_hac_checkpoint_ready(
    snapshot: dict[str, Any], configuration: dict[str, Any]
) -> tuple[bool, str]:
    """Check the complete MM304->TAEM checkpoint contract.

    This deliberately does not treat an altitude crossing or a preview path as
    a checkpoint. The saved state must be inside the fixed alignment station,
    on the perpendicular runway-offset heading, descending above 15 km, and
    carrying a committed, non-degraded 6-20 km HAC plan.
    """
    telemetry = snapshot.get("telemetry") or {}
    guidance = snapshot.get("guidanceState") or {}
    site = configuration.get("site") or {}
    guidance_cfg = configuration.get("guidance") or {}

    def number(value: Any, default: float = float("nan")) -> float:
        return BackendProcess._number(value, default)

    phase = str(snapshot.get("phase", ""))
    if phase not in {"TAEM", "Heading Alignment"}:
        return False, f"phase={phase or 'unknown'}"
    altitude = number(telemetry.get("meanAltitude"))
    site_altitude = number(site.get("altitude"), 0.0)
    if not math.isfinite(altitude) or not math.isfinite(site_altitude):
        return False, "altitude-unavailable"
    if altitude < max(15000.0, site_altitude + 15000.0):
        return False, f"altitude={altitude:.0f}m"
    vertical_speed = number(telemetry.get("verticalSpeed"))
    if not math.isfinite(vertical_speed):
        return False, "vertical-speed-unavailable"
    if vertical_speed >= 0.0:
        return False, "not-descending"

    final_distance = number(guidance_cfg.get("finalApproachDistance"), 8000.0)
    along = number(telemetry.get("runwayAlongTrack"))
    cross = number(telemetry.get("runwayCrossTrack"))
    position_error = math.hypot(along + final_distance, cross)
    if not math.isfinite(position_error) or position_error > 1000.0:
        return False, f"station-error={position_error:.0f}m"

    runway_heading = number(site.get("runwayHeading"), 90.0)
    course = number(telemetry.get("groundTrackHeading"), number(telemetry.get("heading")))
    relative = abs(signed_angle_degrees(course - runway_heading))
    if not 60.0 <= relative <= 120.0:
        return False, f"runway-offset={relative:.1f}deg"

    if guidance.get("terminalPathCommitted") is not True:
        return False, "HAC-not-committed"
    if guidance.get("terminalCandidateValid") is not True:
        return False, "HAC-candidate-invalid"
    candidate_kind = number(guidance.get("candidateKind"))
    if not math.isfinite(candidate_kind) or candidate_kind != 2.0:
        return False, "candidate-not-HAC"
    radius = number(guidance.get("terminalCandidateRadius"))
    if not 6000.0 <= radius <= 20000.0:
        return False, f"HAC-radius={radius:.0f}m"
    degraded_fields = (
        "candidateGeometryDegraded",
        "candidateEnergyDegraded",
        "candidateShellDegraded",
        "candidatePathDegraded",
        "candidateControlDegraded",
        "candidateRateDegraded",
        "candidateEndDegraded",
    )
    for field in degraded_fields:
        if field not in guidance:
            return False, f"HAC-degradation-state-missing:{field}"
        if guidance.get(field) is not False:
            return False, "HAC-degraded"
    return True, "contract-passed"


class EntryDebtGateObservation(NamedTuple):
    checkpoint_range_m: float
    target_altitude_m: float
    range_m: float
    altitude_m: float
    vertical_speed_mps: float
    debt_m: float


class EntryDebtGateDecision(NamedTuple):
    reject: bool
    reason: str
    observation: EntryDebtGateObservation


class EntryDebtCampaignGuard:
    """Operational rejection gates for the pinned DirectFix75km campaign.

    These thresholds are campaign acceptance criteria, not MM304 guidance
    constants. Checkpoints fire only on the first decreasing-range crossing and
    interpolate the exact crossing state when snapshots skip across a boundary.
    """

    def __init__(self) -> None:
        self.previous_range_m: float | None = None
        self.previous_altitude_m: float | None = None
        self.previous_vertical_speed_mps: float | None = None
        self.debt_400_m: float | None = None
        self.crossed: set[float] = set()

    _targets = (
        (400_000.0, 34_500.0),
        (300_000.0, 29_000.0),
        (200_000.0, 23_500.0),
        (100_000.0, 18_000.0),
    )

    @staticmethod
    def _finite_number(value: Any) -> float | None:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) else None

    def update(self, telemetry: dict[str, Any]) -> list[EntryDebtGateDecision]:
        range_m = self._finite_number(telemetry.get("rangeToSite"))
        altitude_m = self._finite_number(telemetry.get("meanAltitude"))
        vertical_speed_mps = self._finite_number(telemetry.get("verticalSpeed"))
        if range_m is None or altitude_m is None or vertical_speed_mps is None:
            return []

        previous_range = self.previous_range_m
        previous_altitude = self.previous_altitude_m
        previous_vertical_speed = self.previous_vertical_speed_mps
        self.previous_range_m = range_m
        self.previous_altitude_m = altitude_m
        self.previous_vertical_speed_mps = vertical_speed_mps
        if previous_range is None or previous_altitude is None or previous_vertical_speed is None:
            return []

        decisions: list[EntryDebtGateDecision] = []
        for checkpoint_range_m, target_altitude_m in self._targets:
            if checkpoint_range_m in self.crossed:
                continue
            if not (previous_range > checkpoint_range_m >= range_m):
                continue

            self.crossed.add(checkpoint_range_m)
            span = previous_range - range_m
            fraction = 1.0 if span <= 0.0 else (previous_range - checkpoint_range_m) / span
            fraction = max(0.0, min(1.0, fraction))
            crossing_altitude_m = previous_altitude + (altitude_m - previous_altitude) * fraction
            crossing_vertical_speed_mps = previous_vertical_speed + (
                vertical_speed_mps - previous_vertical_speed
            ) * fraction
            debt_m = crossing_altitude_m - target_altitude_m
            observation = EntryDebtGateObservation(
                checkpoint_range_m=checkpoint_range_m,
                target_altitude_m=target_altitude_m,
                range_m=checkpoint_range_m,
                altitude_m=crossing_altitude_m,
                vertical_speed_mps=crossing_vertical_speed_mps,
                debt_m=debt_m,
            )

            if checkpoint_range_m == 400_000.0:
                self.debt_400_m = debt_m
                decisions.append(EntryDebtGateDecision(False, "400 km debt recorded", observation))
                continue
            if checkpoint_range_m == 300_000.0:
                worsening = self.debt_400_m is not None and debt_m >= self.debt_400_m
                reject = debt_m > 10_000.0 and worsening
                reason = (
                    "300 km altitude debt remains above 10 km and has not decreased from 400 km"
                    if reject
                    else "300 km debt gate remains recoverable"
                )
                decisions.append(EntryDebtGateDecision(reject, reason, observation))
                continue
            if checkpoint_range_m == 200_000.0:
                reject = debt_m > 7_000.0
                reason = "200 km altitude debt remains above 7 km" if reject else "200 km debt gate passed"
                decisions.append(EntryDebtGateDecision(reject, reason, observation))
                continue

            reject = crossing_altitude_m > 21_000.0 or crossing_vertical_speed_mps >= 0.0
            reason = (
                "100 km gate requires altitude <= 21 km and descending"
                if reject
                else "100 km altitude/descent gate passed"
            )
            decisions.append(EntryDebtGateDecision(reject, reason, observation))

        return decisions


def _guard_python_candidates(root: Path, configured: str, current_python: str) -> list[Path]:
    """Return deterministic guard-interpreter candidates in preference order."""
    candidates: list[Path] = []

    def add(path: Path) -> None:
        path = path.expanduser()
        if path not in candidates:
            candidates.append(path)

    if configured:
        add(Path(configured))

    # Prefer a project-local optional bridge environment when present. Keep the
    # historical sibling KSPFlightComputer environment as a migration fallback;
    # it is derived from the workspace layout rather than a user-specific path.
    project_env = root / "Runtime" / "PythonBridge" / ".venv"
    legacy_env = root.parent.parent / "Python" / "KSPFlightComputer" / "Shuttle" / "venv"
    for environment in (project_env, legacy_env):
        add(environment / "bin" / "python")
        add(environment / "Scripts" / "python.exe")
    add(Path(current_python))
    return candidates


@lru_cache(maxsize=32)
def _guard_python_probe(candidate: str) -> tuple[bool, str]:
    """Check once whether *candidate* is executable and can import Python kRPC."""
    python = Path(candidate)
    if not python.exists():
        return False, "missing"
    if not os.access(python, os.X_OK):
        return False, "not executable"
    try:
        completed = subprocess.run(
            [str(python), "-c", "import krpc"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            timeout=3.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, f"probe failed: {exc}"
    if completed.returncode != 0:
        detail = next((line.strip() for line in completed.stderr.splitlines() if line.strip()), "")
        return False, f"cannot import krpc{': ' + detail if detail else ''}"
    return True, "ok"


def guard_python(root: Path) -> Path:
    """Resolve the optional independent live-test guard's Python interpreter.

    `KSP_LANDER_GUARD_PYTHON` remains the first preference, but a stale or
    dependency-incomplete override no longer makes save restoration impossible.
    Every candidate must actually import `krpc`; merely existing is insufficient.
    The dependency probe is cached because `probe_ksp()` runs frequently.
    """
    configured = os.environ.get("KSP_LANDER_GUARD_PYTHON", "")
    rejected: list[str] = []
    for python in _guard_python_candidates(root, configured, sys.executable):
        usable, reason = _guard_python_probe(str(python))
        if usable:
            return python
        rejected.append(f"{python} ({reason})")
    raise RuntimeError(
        "No Python interpreter with the optional kRPC client is available for the live-test guard. "
        "Set KSP_LANDER_GUARD_PYTHON to a Python that can `import krpc`. Tried: "
        + "; ".join(rejected)
    )


def guard_script(root: Path) -> Path:
    script = root / "Tools" / "ksp_test_guard.py"
    if not script.exists():
        raise RuntimeError(f"KSP test guard is missing: {script}")
    return script


def probe_ksp(root: Path, timeout: float = 8.0) -> dict[str, Any] | None:
    try:
        completed = subprocess.run(
            [str(guard_python(root)), str(guard_script(root)), "probe"],
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if completed.returncode != 0:
        return None
    lines = [line for line in completed.stdout.splitlines() if line.strip()]
    if not lines:
        return None
    try:
        payload = json.loads(lines[-1])
    except json.JSONDecodeError:
        return None
    return payload if payload.get("ready") else None


def wait_for_orbital_state_settle(
    root: Path,
    initial: dict[str, Any],
    timeout: float = 12.0,
    minimum_ut_advance: float = 1.5,
    stable_samples: int = 4,
) -> dict[str, Any]:
    """Wait for the restored vessel to finish unpacking before freezing planning UT.

    KSP/kRPC can briefly expose the save's serialized vessel mass before the loaded
    craft has completed its first physics updates. Freezing immediately after the
    UT rewind can preserve that transient value indefinitely and certify a deorbit
    plan against the wrong mass. Require real 1x UT progress plus a short stable
    mass/thrust window before pausing for deterministic planning.
    """
    if bool(initial.get("paused")):
        return initial
    start_ut = BackendProcess._number(initial.get("ut"))
    vessel_name = str(initial.get("vessel") or "")
    samples: list[tuple[float, float, float, dict[str, Any]]] = []
    deadline = time.monotonic() + timeout
    last_state = initial
    while time.monotonic() < deadline:
        observed = probe_ksp(root, timeout=1.2)
        if observed is not None:
            last_state = observed
            same_vessel = not vessel_name or str(observed.get("vessel") or "") == vessel_name
            unwarped = observed.get("railsWarpFactor") == 0 and observed.get("physicsWarpFactor") == 0
            unpaused = not bool(observed.get("paused"))
            ut = BackendProcess._number(observed.get("ut"))
            mass = BackendProcess._number(observed.get("mass"))
            dry_mass = BackendProcess._number(observed.get("dryMass"))
            thrust = BackendProcess._number(observed.get("availableThrust"))
            # During quickload KSP can expose a stable-looking partial vessel. Two
            # transient forms are unsafe: wet mass below dry mass, and dryMass=0 while
            # the Part/Vessel graph is still unpacking. The latter reproduced on
            # DirectFix75km as a stable 20.000 t state even though the real STS-N is
            # 40.278 t / 30.053 t dry. Require a real dry-mass observation before the
            # stable window can certify startup physics.
            dry_mass_known = dry_mass > 1.0
            mass_unpacked = dry_mass_known and mass + max(1.0, dry_mass * 1e-3) >= dry_mass
            if same_vessel and unwarped and unpaused and mass > 1.0 and mass_unpacked and thrust >= 0.0:
                # Do not count apparently stable samples from the initial unpack
                # interval. DirectFix75km exposes a coherent 20.000/17.728 t partial
                # vessel for roughly the first 0.9 s before jumping to the real
                # 40.278/30.053 t craft. Starting the stability window only after the
                # warm-up makes this generic: certification requires stability after
                # physics has had time to materialize, not merely stability since load.
                if ut - start_ut < minimum_ut_advance:
                    samples.clear()
                else:
                    samples.append((ut, mass, thrust, observed))
                    samples = samples[-stable_samples:]
                    if len(samples) == stable_samples:
                        masses = [sample[1] for sample in samples]
                        thrusts = [sample[2] for sample in samples]
                        mass_span = max(masses) - min(masses)
                        thrust_span = max(thrusts) - min(thrusts)
                        mass_tol = max(1.0, abs(mass) * 1e-3)
                        thrust_tol = max(100.0, abs(thrust) * 5e-3)
                        if mass_span <= mass_tol and thrust_span <= thrust_tol:
                            print(
                                f"test preflight physics settled: UT +{ut - start_ut:.2f}s, "
                                f"mass {mass / 1000:.3f} t, thrust {thrust / 1000:.1f} kN",
                                flush=True,
                            )
                            return observed
        time.sleep(0.08)
    raise RuntimeError(
        "Restored orbital vessel did not reach a stable unpacked mass/thrust state "
        f"before planning freeze; last state={last_state}"
    )



def settle_restored_reentry_state(root: Path, initial: dict[str, Any]) -> dict[str, Any]:
    """Let a freshly loaded reentry checkpoint unpack before startup freeze.

    Reentry-only mode used to skip the orbital settle window and immediately
    start the physics hold. KSP can still expose a transient serialized vessel
    mass at that point, so freezing there makes live qualification run against
    a materially different state than the saved flight state.
    """
    if bool(initial.get("paused")):
        return initial
    # Atmospheric saves can retain throttle (live STS-N-transonic-stable had
    # 45%). Neutralize before measuring mass stability; otherwise fuel burn
    # makes the unpacking check fail and alters the experiment initial state.
    prepare_reentry_direct_control(root)
    # A loaded atmospheric craft can need several expensive guard probes before four
    # post-warmup samples are available.  Give that evidence window enough wall time;
    # do not weaken the actual mass/thrust stability criterion.
    return wait_for_orbital_state_settle(root, initial, timeout=45.0)
def restore_quicksave(root: Path, save_name: str = "quicksave", settle_timeout: float = 120.0) -> dict[str, Any]:
    before = probe_ksp(root, timeout=3.0)
    before_ut = BackendProcess._number(before.get("ut")) if before else None
    print(f"test preflight: loading KSP save {save_name}", flush=True)
    loader = subprocess.Popen(
        [
            str(guard_python(root)),
            str(guard_script(root)),
            "load-save",
            "--name",
            save_name,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    # A backward UT jump is the fastest proof that KSP switched to the requested
    # save, but atmospheric checkpoints are often *later* than the scene currently
    # loaded after cleanup. In that case a rewind-only test rejects a save that did
    # load successfully. Keep the fast rewind path, and also accept the guard's
    # authoritative load-save-settled event once its helper returns.
    deadline = time.monotonic() + settle_timeout
    state: dict[str, Any] | None = None
    loader_detail = ""
    while time.monotonic() < deadline:
        observed = probe_ksp(root, timeout=1.2)
        if observed is not None and observed.get("railsWarpFactor") == 0 and observed.get("physicsWarpFactor") == 0:
            observed_ut = BackendProcess._number(observed.get("ut"))
            rewound = before_ut is None or observed_ut < before_ut - 0.25
            if rewound:
                state = observed
                break
        if loader.poll() is not None:
            if not loader_detail and loader.stdout is not None:
                try:
                    loader_detail = loader.stdout.read().strip()
                except Exception:
                    loader_detail = ""
            settled = False
            for line in loader_detail.splitlines():
                try:
                    payload = json.loads(line)
                except Exception:
                    continue
                if payload.get("event") == "load-save-settled" and payload.get("name") == save_name:
                    settled = True
                    break
            if settled:
                observed = probe_ksp(root, timeout=2.0)
                if observed is not None and observed.get("railsWarpFactor") == 0 and observed.get("physicsWarpFactor") == 0:
                    state = observed
                    break
            else:
                status = loader.returncode
                detail = loader_detail or "load-save helper produced no JSON status"
                raise RuntimeError(
                    f"KSP save {save_name} load helper exited with status {status} before settle: {detail[:300]}"
                )
        time.sleep(0.10)
    if loader.poll() is None:
        loader.send_signal(signal.SIGINT)
        try:
            loader.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            loader.kill()
            loader.wait(timeout=1.0)
    if state is None:
        if not loader_detail and loader.stdout is not None:
            try:
                loader_detail = loader.stdout.read().strip()
            except Exception:
                loader_detail = ""
        detail = loader_detail or "load-save helper produced no JSON status (likely blocked during kRPC connection)"
        raise RuntimeError(f"KSP save {save_name} did not settle before timeout: {detail[:300]}")
    after_ut = BackendProcess._number(state.get("ut"))
    print(
        f"test preflight restored: {state.get('vessel', '?')} UT {after_ut:.1f}, "
        f"alt {BackendProcess._number(state.get('meanAltitude')) / 1000:.1f} km",
        flush=True,
    )
    return state


def prepare_reentry_direct_control(root: Path) -> dict[str, Any]:
    completed = subprocess.run(
        [str(guard_python(root)), str(guard_script(root)), "prepare-reentry"],
        capture_output=True,
        text=True,
        timeout=15.0,
        check=False,
    )
    lines = [line for line in completed.stdout.splitlines() if line.strip()]
    if completed.returncode != 0 or not lines:
        detail = (completed.stderr or completed.stdout or "no response").strip()
        raise RuntimeError(f"Could not prepare direct reentry handoff: {detail[:300]}")
    payload = json.loads(lines[-1])
    if payload.get("event") != "reentry-direct-control-ready":
        raise RuntimeError(f"Reentry handoff confirmation is invalid: {payload}")
    if payload.get("sas") is not False:
        raise RuntimeError(f"Reentry handoff did not disable SAS: {payload}")
    throttle = BackendProcess._number(payload.get("throttle"), float("nan"))
    if not math.isfinite(throttle) or abs(throttle) > 1e-4:
        raise RuntimeError(f"Reentry handoff did not confirm zero throttle: {payload}")
    actual_thrust = BackendProcess._number(payload.get("actualThrust"), float("nan"))
    if not math.isfinite(actual_thrust) or abs(actual_thrust) > 1.0:
        raise RuntimeError(f"Reentry handoff did not confirm zero actual thrust: {payload}")
    rails_warp = BackendProcess._number(payload.get("railsWarpFactor"), float("nan"))
    physics_warp = BackendProcess._number(payload.get("physicsWarpFactor"), float("nan"))
    if not math.isfinite(rails_warp) or rails_warp != 0.0:
        raise RuntimeError(f"Reentry handoff did not confirm 1x rails warp: {payload}")
    if not math.isfinite(physics_warp) or physics_warp != 0.0:
        raise RuntimeError(f"Reentry handoff did not confirm 1x physics warp: {payload}")
    print(
        f"reentry handoff: SAS off, throttle/thrust zero, 1x, direct controls neutral at "
        f"{BackendProcess._number(payload.get('meanAltitude')) / 1000:.1f} km",
        flush=True,
    )
    return payload


def set_ksp_paused(root: Path, paused: bool) -> None:
    mode = "pause" if paused else "resume"
    completed = subprocess.run(
        [str(guard_python(root)), str(guard_script(root)), mode],
        capture_output=True,
        text=True,
        timeout=15.0,
        check=False,
    )
    lines = [line for line in completed.stdout.splitlines() if line.strip()]
    if completed.returncode != 0 or not lines:
        detail = (completed.stderr or completed.stdout or "no response").strip()
        raise RuntimeError(f"Could not {'pause' if paused else 'resume'} KSP: {detail[:300]}")
    payload = json.loads(lines[-1])
    if payload.get("event") != "pause-state" or bool(payload.get("paused")) != paused:
        raise RuntimeError(f"KSP pause state did not change as requested: {payload}")


def start_crash_guard(root: Path, recovery_save: str = "quicksave") -> subprocess.Popen[str]:
    runtime = root / "Runtime" / "Headless"
    runtime.mkdir(parents=True, exist_ok=True)
    guard_log = runtime / "crash_guard.log"
    with guard_log.open("w", encoding="utf-8") as handle:
        process = subprocess.Popen(
            [
                str(guard_python(root)),
                str(guard_script(root)),
                "monitor",
                "--interval",
                "0.15",
                "--name",
                recovery_save,
                "--parent-pid",
                str(os.getpid()),
            ],
            stdout=handle,
            stderr=subprocess.STDOUT,
            text=True,
            # The safety guard must outlive an external signal delivered to the
            # runner's process group. It watches this process explicitly and
            # freezes KSP if the runner disappears before orderly cleanup.
            start_new_session=True,
        )
    return process


def lifecycle_watch_required(*, no_quickload: bool, live: bool, connect_only: bool) -> bool:
    """Protect any early save mutation or live run, but not a pure read-only probe."""
    return (not no_quickload) or (live and not connect_only)


def cleanup_handoff_state(completed: bool) -> str:
    """Only successful completion may authorize a startup-holder release."""
    return "released" if completed else "failed"


def start_parent_watch(root: Path) -> subprocess.Popen[str]:
    """Arm a passive sentinel before the first KSP state mutation.

    It never connects to KSP while this runner exists. If the runner disappears
    before ordinary cleanup can run, it independently freezes physics.
    """
    runtime = root / "Runtime" / "Headless"
    runtime.mkdir(parents=True, exist_ok=True)
    watch_log = runtime / "parent_watch.log"
    with watch_log.open("w", encoding="utf-8") as handle:
        process = subprocess.Popen(
            [
                str(guard_python(root)),
                str(guard_script(root)),
                "parent-watch",
                "--interval",
                "0.05",
                "--parent-pid",
                str(os.getpid()),
            ],
            stdout=handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
    return process


def secure_failed_test_state(root: Path, save_name: str, restore_save: bool) -> dict[str, Any] | None:
    """Leave every failed/aborted live test in a frozen, non-advancing state."""
    restored: dict[str, Any] | None = None
    try:
        if restore_save:
            restored = restore_quicksave(root, save_name)
        return restored
    finally:
        # Even a failed restore must never leave the current atmospheric scene
        # advancing without a controller.
        set_ksp_paused(root, True)


def start_startup_hold(root: Path) -> subprocess.Popen[str]:
    runtime = root / "Runtime" / "Headless"
    runtime.mkdir(parents=True, exist_ok=True)
    hold_log = runtime / "startup_hold.log"
    with hold_log.open("w", encoding="utf-8") as handle:
        process = subprocess.Popen(
            [
                str(guard_python(root)),
                str(guard_script(root)),
                "startup-hold",
                "--interval",
                "0.05",
                "--parent-pid",
                str(os.getpid()),
            ],
            stdout=handle,
            stderr=subprocess.STDOUT,
            text=True,
            # Like the crash guard, the startup freeze must survive a signal sent
            # to the runner's process group so it can fail closed on parent loss.
            start_new_session=True,
        )
    return process


def start_preentry_physics_warp(
    root: Path,
    multiplier: int,
    release_altitude: float,
) -> subprocess.Popen[str]:
    runtime = root / "Runtime" / "Headless"
    runtime.mkdir(parents=True, exist_ok=True)
    warp_log = runtime / "preentry_warp.log"
    with warp_log.open("w", encoding="utf-8") as handle:
        process = subprocess.Popen(
            [
                str(guard_python(root)),
                str(guard_script(root)),
                "preentry-warp",
                "--multiplier",
                str(multiplier),
                "--release-altitude",
                str(release_altitude),
                "--interval",
                "0.05",
                "--parent-pid",
                str(os.getpid()),
            ],
            stdout=handle,
            stderr=subprocess.STDOUT,
            text=True,
            # Warp ownership must not survive a runner process-group teardown.
            # Isolate this child, then let its parent-PID watch zero warp/pause.
            start_new_session=True,
        )
    return process


def start_backend_watch(root: Path, backend_pid: int, backend_executable: Path) -> subprocess.Popen[str]:
    """Reap the private native backend process group if this runner disappears."""
    runtime = root / "Runtime" / "Headless"
    runtime.mkdir(parents=True, exist_ok=True)
    watch_log = runtime / "backend_watch.log"
    with watch_log.open("w", encoding="utf-8") as handle:
        process = subprocess.Popen(
            [
                str(guard_python(root)),
                str(guard_script(root)),
                "backend-watch",
                "--interval",
                "0.05",
                "--parent-pid",
                str(os.getpid()),
                "--backend-pid",
                str(backend_pid),
                "--backend-executable",
                str(backend_executable.resolve()),
            ],
            stdout=handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
    return process


def terminate_private_backend(process: subprocess.Popen[bytes]) -> None:
    """Fail closed if the backend reaper itself could not be armed."""
    if process.poll() is not None:
        return
    try:
        if os.getpgid(process.pid) != process.pid:
            raise RuntimeError(f"Backend PID {process.pid} is not its own process-group leader")
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        if process.poll() is None and os.getpgid(process.pid) == process.pid:
            os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=2.0)


def stop_process(process: subprocess.Popen[str] | None) -> None:
    if process is None or process.poll() is not None:
        return
    # ksp_test_guard installs a KeyboardInterrupt cleanup path for SIGINT. Use
    # that cooperative path instead of subprocess.terminate()/SIGTERM.
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2.0)


class HUDCompanion:
    """Best-effort exact-snapshot HUD publisher for headless/live tests.

    A persistent observer provides fallback display. While this child is alive it
    registers ownership so the observer suspends its own overlay and avoids duplicate
    drawings. If the child dies, the next snapshot restarts it automatically.
    """

    def __init__(self, root: Path, configuration: dict[str, Any]) -> None:
        self.root = root
        self.configuration = configuration
        self.process: subprocess.Popen[str] | None = None
        self.last_emit = 0.0
        self.next_restart = 0.0
        self.owner_file = root / "Runtime" / "Headless" / "ingame_hud_exact.pid"
        self.enabled = os.environ.get("KSP_LANDER_AR_HUD", "1").strip().lower() not in {
            "0",
            "false",
            "off",
            "no",
        }
        self.python: Path | None = None
        self.env: dict[str, str] | None = None
        if not self.enabled:
            return
        try:
            self.python = guard_python(root)
            self.env = os.environ.copy()
            self.env["PYTHONPATH"] = str(root) + (
                os.pathsep + self.env["PYTHONPATH"] if self.env.get("PYTHONPATH") else ""
            )
            if self._spawn():
                print("in-game AR HUD: exact headless stream attached", flush=True)
        except Exception as exc:
            self._clear_owner()
            print(f"warning: could not start in-game AR HUD: {exc}", file=sys.stderr, flush=True)

    def _clear_owner(self, expected_pid: int | None = None) -> None:
        try:
            if expected_pid is not None:
                current = int(self.owner_file.read_text(encoding="utf-8").strip())
                if current != expected_pid:
                    return
            self.owner_file.unlink()
        except (OSError, ValueError):
            pass

    @staticmethod
    def _write_to(process: subprocess.Popen[str], message: dict[str, Any]) -> bool:
        if process.poll() is not None or process.stdin is None:
            return False
        try:
            process.stdin.write(json.dumps(message, separators=(",", ":"), allow_nan=False) + "\n")
            process.stdin.flush()
            return True
        except (BrokenPipeError, OSError, ValueError):
            return False

    def _spawn(self) -> bool:
        if not self.enabled or self.python is None or self.env is None:
            return False
        current = self.process
        if current is not None and current.poll() is None and current.stdin is not None:
            return True
        if current is not None:
            self._clear_owner(current.pid)
        self.process = None
        if time.monotonic() < self.next_restart:
            return False
        try:
            process = subprocess.Popen(
                [str(self.python), str(self.root / "Tools" / "ingame_hud_stream.py")],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=None,
                text=True,
                bufsize=1,
                cwd=self.root,
                env=self.env,
            )
            if not self._write_to(process, {"type": "configuration", "configuration": self.configuration}):
                process.terminate()
                process.wait(timeout=1.0)
                self.next_restart = time.monotonic() + 1.0
                return False
            self.process = process
            self.owner_file.parent.mkdir(parents=True, exist_ok=True)
            self.owner_file.write_text(str(process.pid), encoding="utf-8")
            self.next_restart = 0.0
            return True
        except Exception as exc:
            self.process = None
            self.next_restart = time.monotonic() + 1.0
            print(f"warning: in-game AR HUD restart failed: {exc}", file=sys.stderr, flush=True)
            return False

    def _send(self, message: dict[str, Any]) -> None:
        if not self._spawn():
            return
        process = self.process
        if process is None:
            return
        if self._write_to(process, message):
            return
        self._clear_owner(process.pid)
        try:
            if process.poll() is None:
                process.terminate()
        except OSError:
            pass
        self.process = None
        self.next_restart = time.monotonic() + 0.25

    def snapshot(self, snapshot: dict[str, Any]) -> None:
        now = time.monotonic()
        if now - self.last_emit < 0.12:
            return
        self.last_emit = now
        self._send({"type": "snapshot", "snapshot": snapshot})

    def suspend(self) -> None:
        self._send({"type": "suspend"})

    def close(self) -> None:
        process = self.process
        self.process = None
        if process is None:
            self._clear_owner()
            return
        self._clear_owner(process.pid)
        if process.poll() is None:
            try:
                if process.stdin is not None:
                    process.stdin.write('{"type":"shutdown"}\n')
                    process.stdin.flush()
                    process.stdin.close()
            except (BrokenPipeError, OSError):
                pass
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1.0)


class BackendProcess:
    def __init__(self, executable: Path, snapshot_sink: Callable[[dict[str, Any]], None] | None = None) -> None:
        env = os.environ.copy()
        self.process = subprocess.Popen(
            [str(executable)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=None,
            text=False,
            bufsize=0,
            env=env,
            start_new_session=True,
        )
        if self.process.stdin is None or self.process.stdout is None:
            raise RuntimeError("Could not open backend stdio pipes")
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.process.stdout, selectors.EVENT_READ)
        self.read_buffer = bytearray()
        self.next_id = 1
        self.latest_snapshot: dict[str, Any] | None = None
        self.snapshot_sink = snapshot_sink
        self.last_printed_phase = ""
        self.last_printed_status = ""
        self.last_printed_warning = ""
        self.last_printed_at = 0.0
    def close(self) -> None:
        try:
            self.selector.close()
        except Exception:
            pass
        for stream in (self.process.stdin, self.process.stdout):
            if stream is None:
                continue
            try:
                stream.close()
            except (BrokenPipeError, OSError):
                pass

    def _read_message(self, timeout: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        while b"\n" not in self.read_buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Timed out waiting for the landing backend")
            events = self.selector.select(remaining)
            if not events:
                raise TimeoutError("Timed out waiting for the landing backend")
            chunk = os.read(self.process.stdout.fileno(), 65536)
            if not chunk:
                code = self.process.poll()
                raise RuntimeError(f"Landing backend exited unexpectedly with code {code}")
            self.read_buffer.extend(chunk)
        raw_line, _, remainder = self.read_buffer.partition(b"\n")
        self.read_buffer = bytearray(remainder)
        line = raw_line.decode("utf-8", errors="replace")
        try:
            return json.loads(line)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Landing backend emitted invalid JSON: {line[:240]!r}") from exc

    @staticmethod
    def _number(value: Any, default: float = 0.0) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def _handle_snapshot(self, snapshot: dict[str, Any], force: bool = False) -> None:
        self.latest_snapshot = snapshot
        if self.snapshot_sink is not None:
            try:
                self.snapshot_sink(snapshot)
            except Exception as exc:
                print(f"warning: headless HUD snapshot sink failed: {exc}", file=sys.stderr, flush=True)
                self.snapshot_sink = None
        phase = str(snapshot.get("phase", "?"))
        status = str(snapshot.get("statusMessage", ""))
        warning = str(snapshot.get("warningMessage") or "")
        now = time.monotonic()
        should_print = (
            force
            or phase != self.last_printed_phase
            or warning != self.last_printed_warning
            or now - self.last_printed_at >= 5.0
        )
        if not should_print:
            return
        telemetry = snapshot.get("telemetry") or {}
        ut = self._number(telemetry.get("ut"))
        altitude = self._number(telemetry.get("meanAltitude"))
        speed = self._number(telemetry.get("trueAirSpeed"))
        distance = self._number(telemetry.get("rangeToSite"))
        print(
            f"[{phase}] UT {ut:.1f} | alt {altitude / 1000:.1f} km | "
            f"range {distance / 1000:.1f} km | TAS {speed:.1f} m/s | {status}",
            flush=True,
        )
        if warning:
            print(f"  warning: {warning}", flush=True)
        self.last_printed_phase = phase
        self.last_printed_status = status
        self.last_printed_warning = warning
        self.last_printed_at = now

    def read_until(
        self,
        predicate: Callable[[dict[str, Any]], bool],
        timeout: float,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Timed out waiting for the requested backend state")
            message = self._read_message(remaining)
            if message.get("type") == "snapshot":
                snapshot = message.get("snapshot") or {}
                self._handle_snapshot(snapshot)
                if predicate(snapshot):
                    return snapshot

    def send(self, method: str, timeout: float = 30.0, **fields: Any) -> dict[str, Any]:
        request_id = str(self.next_id)
        self.next_id += 1
        request = {"id": request_id, "method": method, **fields}
        self.process.stdin.write((json.dumps(request, separators=(",", ":")) + "\n").encode("utf-8"))
        self.process.stdin.flush()
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"Backend method {method} timed out")
            message = self._read_message(remaining)
            if message.get("type") == "snapshot":
                self._handle_snapshot(message.get("snapshot") or {})
                continue
            if message.get("type") != "response" or str(message.get("id")) != request_id:
                continue
            if not message.get("ok"):
                raise RuntimeError(str(message.get("error") or f"Backend method {method} failed"))
            return message.get("result") or {}


def load_configuration(path: Path, auto_warp: bool) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        configuration = json.load(handle)
    configuration.setdefault("guidance", {})["useTimeWarp"] = auto_warp
    return configuration


def main() -> int:
    def _interrupt(_signum: int, _frame: Any) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _interrupt)
    signal.signal(signal.SIGINT, _interrupt)

    root = Path(__file__).resolve().parents[1]
    runtime = root / "Runtime" / "Headless"
    runtime.mkdir(parents=True, exist_ok=True)
    lock_path = runtime / "live_test.lock"
    # A mission supervisor may retain the same open-file-description lock
    # between comparable flights. Never release it during build/analysis, or
    # another controller can reload KSP halfway through the experiment.
    inherited_lock = os.environ.get("KSP_INHERITED_LIVE_LOCK_FD")
    if inherited_lock is not None:
        inherited_fd = int(inherited_lock)
        held = os.fstat(inherited_fd)
        expected = lock_path.stat()
        if (held.st_dev, held.st_ino) != (expected.st_dev, expected.st_ino):
            raise RuntimeError("Inherited mission lock does not match this workspace")
        lock_handle = os.fdopen(os.dup(inherited_fd), "a+")
    else:
        lock_handle = lock_path.open("a+")
    try:
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        raise RuntimeError("Another headless live flight test is already running for this project") from exc

    parser = argparse.ArgumentParser(description="Run the KSP shuttle lander without PyQt.")
    parser.add_argument("--backend", type=Path, default=root / "CLanding" / "build" / "landing_backend")
    parser.add_argument("--config", type=Path, default=root / "Configuration" / "default.json")
    parser.add_argument("--live", action="store_true", help="Engage guidance after an executable plan is found.")
    parser.add_argument("--no-pin-campaign-artifacts", action="store_true", help="For live runs only, disable the default immutable backend/config snapshot and process-inode verification.")
    parser.add_argument("--campaign-artifact-root", type=Path, default=None, help="Optional directory for immutable live-campaign backend/config snapshots.")
    parser.add_argument(
        "--campaign-identity",
        default=os.environ.get("KSP_LANDER_CAMPAIGN_IDENTITY", ""),
        help="Reuse an exact previously pinned KSP simulation campaign identity.",
    )
    parser.add_argument("--reentry-only", action="store_true", help="Skip deorbit planning/burn and continue from a saved post-deorbit state.")
    parser.add_argument(
        "--hac-only",
        action="store_true",
        help="Skip deorbit, entry and S-turn guidance and start directly with HAC/terminal guidance from the live atmospheric state.",
    )
    parser.add_argument("--hac-start-altitude", type=float, default=15000.0)
    parser.add_argument("--hac-start-speed", type=float, default=360.0)
    parser.add_argument("--hac-altitude-tolerance", type=float, default=4000.0)
    parser.add_argument("--hac-speed-tolerance", type=float, default=100.0)
    parser.add_argument("--connect-only", action="store_true", help="Connect and report telemetry without planning.")
    parser.add_argument("--no-auto-warp", action="store_true", help="Disable automatic coast-to-burn time warp.")
    parser.add_argument("--no-quickload", action="store_true", help="Do not restore KSP quicksave before this test.")
    parser.add_argument("--initial-save", default="quicksave", help="Named KSP save to restore before the test and after failures; defaults to generic quicksave.")
    parser.add_argument("--checkpoint-physics-profile", type=Path, help="Verified full-mass checkpoint profile; freeze as soon as this known physical state is coherent.")
    parser.add_argument(
        "--enforce-75km-entry-gates",
        action="store_true",
        help="Enforce Skyline DirectFix75km campaign rejection gates at 400/300/200/100 km and safely restore on failure.",
    )
    parser.add_argument("--no-crash-guard", action="store_true", help="Disable automatic quickload on imminent impact.")
    parser.add_argument(
        "--preentry-physics-warp",
        type=int,
        choices=(1, 2, 3, 4),
        default=4,
        help="Maximum pre-entry physics-warp multiplier; capped to 2x during early MM304, hard-forced to 1x at/below 40 km, and 1x at TAEM (default: 4x).",
    )
    parser.add_argument(
        "--preentry-warp-release-altitude",
        type=float,
        default=72000.0,
        help="Altitude below which physics warp is capped to 2x and guarded by MM304 attitude/rate stability; a non-configurable 40 km hard cutoff forces 1x (default: 72000 m).",
    )
    parser.add_argument(
        "--checkpoint-altitude",
        type=float,
        default=0.0,
        help="Save one named KSP checkpoint on first descent through this mean altitude (disabled by default).",
    )
    parser.add_argument("--checkpoint-name", default="STS-N-reentry-checkpoint")
    parser.add_argument(
        "--checkpoint-at",
        action="append",
        nargs=2,
        metavar=("ALTITUDE_M", "NAME"),
        default=[],
        help=(
            "Save a named checkpoint on first live descent through ALTITUDE_M; "
            "repeat for a continuous checkpoint chain (for example 72/50/40/30 km)."
        ),
    )
    parser.add_argument(
        "--taem-checkpoint-name",
        default="",
        help="Save a checkpoint only after the complete MM304->TAEM 6-20 km HAC contract is true.",
    )
    parser.add_argument(
        "--require-strict",
        action="store_true",
        help="Require a strict robust deorbit plan and a strict achieved post-burn state before continuing entry.",
    )
    parser.add_argument(
        "--strict-checkpoint-name",
        default="",
        help="Save this checkpoint after the completed deorbit burn is verified inside the strict corridor.",
    )
    parser.add_argument(
        "--max-wall-seconds",
        type=float,
        default=0.0,
        help="optional wall-clock limit; <= 0 disables the limit",
    )
    parser.add_argument(
        "--ignore-sigint",
        action="store_true",
        help="Ignore SIGINT during unattended live validation; SIGTERM still performs a safe abort.",
    )
    parser.add_argument(
        "--ignore-sigterm",
        action="store_true",
        help="Ignore SIGTERM during an explicitly unattended validation run. Normal runs retain SIGTERM safe-abort behavior.",
    )
    args = parser.parse_args()
    try:
        descent_checkpoints = descent_checkpoint_plan(args.checkpoint_at)
    except ValueError as exc:
        parser.error(str(exc))
    config_was_explicit = any(
        arg == "--config" or arg.startswith("--config=") for arg in sys.argv[1:]
    )
    unpowered_policy_error = live_unpowered_policy_error(args.live, args.connect_only)
    if unpowered_policy_error is not None:
        raise SystemExit(unpowered_policy_error)

    if args.reentry_only and args.hac_only:
        raise SystemExit("--reentry-only and --hac-only are mutually exclusive")
    if args.enforce_75km_entry_gates:
        if not args.live or not args.reentry_only or args.hac_only or args.connect_only:
            raise SystemExit("--enforce-75km-entry-gates requires a normal --live --reentry-only campaign")
        if args.no_quickload:
            raise SystemExit("--enforce-75km-entry-gates requires failure quickload/restore to remain enabled")
        if args.initial_save != "DirectFix75km":
            raise SystemExit("--enforce-75km-entry-gates requires --initial-save DirectFix75km")

    if args.ignore_sigint:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    else:
        signal.signal(signal.SIGINT, _interrupt)
    if args.ignore_sigterm:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
    else:
        signal.signal(signal.SIGTERM, _interrupt)

    backend_path = args.backend.resolve()
    config_path = args.config.resolve()
    auto_warp = not args.no_auto_warp and not args.reentry_only and not args.hac_only
    if args.campaign_identity and (not args.live or args.connect_only or args.no_pin_campaign_artifacts):
        raise SystemExit("--campaign-identity requires a normal pinned --live KSP simulation run")
    if (
        args.live
        and not args.connect_only
        and not args.no_pin_campaign_artifacts
        and not args.campaign_identity
        and not config_was_explicit
    ):
        raise SystemExit(
            "Creating a new pinned live KSP campaign requires an explicit --config path; "
            "do not silently pin Configuration/default.json when a live bridge profile is intended."
        )
    if not args.campaign_identity:
        if not backend_path.exists():
            raise SystemExit(f"Landing backend does not exist: {backend_path}")
        if not config_path.exists():
            raise SystemExit(f"Configuration does not exist: {config_path}")

    campaign_pin = None
    if args.live and not args.connect_only and not args.no_pin_campaign_artifacts:
        if args.campaign_identity:
            campaign_pin = load_pinned_campaign_artifacts(root, args.campaign_identity, args.campaign_artifact_root)
            backend_path = campaign_pin.backend
            config_path = campaign_pin.config
            with config_path.open("r", encoding="utf-8") as handle:
                configuration = json.load(handle)
            pinned_auto_warp = bool((configuration.get("guidance") or {}).get("useTimeWarp"))
            if pinned_auto_warp != auto_warp:
                raise RuntimeError(
                    "Pinned KSP campaign configuration is incompatible with this restart mode: "
                    f"useTimeWarp={pinned_auto_warp}, requested={auto_warp}"
                )
            provenance_action = "reused"
        else:
            configuration = load_configuration(config_path, auto_warp=auto_warp)
            campaign_pin = pin_campaign_artifacts(
                root,
                backend_path,
                config_path,
                args.campaign_artifact_root,
                effective_configuration=configuration,
            )
            backend_path = campaign_pin.backend
            config_path = campaign_pin.config
            provenance_action = "pinned"
        os.environ["KSP_LANDER_CAMPAIGN_IDENTITY"] = campaign_pin.identity
        print(
            f"campaign provenance {provenance_action}: identity={campaign_pin.identity} "
            f"backendSHA={campaign_pin.backend_sha256} configSHA={campaign_pin.config_sha256} "
            f"sourceSHA={campaign_pin.source_tree_sha256} inode={campaign_pin.backend_inode} "
            f"manifest={campaign_pin.manifest}",
            flush=True,
        )
    else:
        configuration = load_configuration(config_path, auto_warp=auto_warp)

    # The broad cleanup try/finally begins only after restore/backend startup.
    # Protect that earlier mutation window against abrupt caller/tool teardown.
    lifecycle_watch: subprocess.Popen[str] | None = None
    if lifecycle_watch_required(no_quickload=args.no_quickload, live=args.live, connect_only=args.connect_only):
        lifecycle_watch = start_parent_watch(root)

    orbital_preplan_hold = False
    if not args.no_quickload:
        restored = restore_quicksave(root, args.initial_save)

        if args.reentry_only or args.hac_only:
            # Both atmospheric modes must use the unpacked live vessel mass,
            # not the transient serialized mass immediately after loading.
            if args.checkpoint_physics_profile:
                prepared = subprocess.run(
                    [str(guard_python(root)), str(root / "Tools/settle_known_checkpoint.py"),
                     "--profile", str(args.checkpoint_physics_profile.resolve()),
                     "--save", args.initial_save],
                    capture_output=True, text=True, timeout=30.0, check=True)
                restored = json.loads(prepared.stdout.strip().splitlines()[-1])
                print("Known checkpoint physics frozen: " + json.dumps(restored), flush=True)
                os.environ["KSP_LANDER_STARTUP_ADOPT_PAUSE"] = "1"
            else:
                restored = settle_restored_reentry_state(root, restored)
        if args.hac_only:
            print(
                f"HAC test quicksave restored at {BackendProcess._number(restored.get('meanAltitude')) / 1000:.1f} km",
                flush=True,
            )
        elif not args.reentry_only and not bool(restored.get("paused")):
            restored = wait_for_orbital_state_settle(root, restored)
            # Freeze only after the restored vessel has completed its first real
            # physics updates. This keeps planning deterministic without locking
            # in the transient serialized mass reported immediately after quickload.
            set_ksp_paused(root, True)
            orbital_preplan_hold = True
            print("test preflight: orbital physics frozen after live-state settle", flush=True)

    skip_reentry_handoff = (
        args.reentry_only
        and os.environ.get("KSP_LANDER_SKIP_REENTRY_HANDOFF") == "1"
    )

    handoff_path = None
    startup_hold: subprocess.Popen[str] | None = None

    def release_startup_freeze_after_engagement() -> None:
        nonlocal startup_hold
        if startup_hold is None or handoff_path is None:
            return
        # Connection alone is not control readiness: the Python bridge may
        # perform neutral/preparation writes before the C guidance machine is
        # armed.  Publish a distinct marker only after engageReentry/HAC has
        # succeeded, then wait for the physics holder to unpause and exit.
        with open(handoff_path, "r+", encoding="utf-8") as handoff:
            fcntl.flock(handoff, fcntl.LOCK_EX)
            handoff.seek(0)
            handoff.write("engaged")
            handoff.truncate()
            handoff.flush()
        try:
            result = startup_hold.wait(timeout=8.0)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("Startup checkpoint holder did not release after guidance engagement") from exc
        if result != 0:
            raise RuntimeError(f"Startup checkpoint holder exited with status {result} after guidance engagement")
        startup_hold = None

    if (args.reentry_only or args.hac_only) and not args.connect_only and os.environ.get("KSP_LANDER_DISABLE_STARTUP_HOLD") != "1":
        import tempfile
        handoff_fd, handoff_path = tempfile.mkstemp(prefix="ksp-control-handoff-")
        os.close(handoff_fd)
        os.environ["KSP_LANDER_CONTROL_HANDOFF"] = handoff_path
        startup_hold = start_startup_hold(root)
        # The startup holder freezes KSP physics rather than commanding a
        # synthetic attitude.  Wait for its explicit acknowledgement before
        # opening the backend bridge; a fixed sleep can lose the race on a busy
        # machine and recreate an uncontrolled/reversing startup interval.
        pause_deadline = time.monotonic() + 30.0
        while time.monotonic() < pause_deadline:
            if startup_hold.poll() is not None:
                raise RuntimeError("Startup checkpoint holder exited before KSP was paused")
            try:
                with open(handoff_path, "r") as handoff:
                    fcntl.flock(handoff, fcntl.LOCK_SH)
                    handoff_state = handoff.read().strip()
                if handoff_state == "startup-paused":
                    break
            except OSError:
                pass
            time.sleep(0.02)
        else:
            stop_process(startup_hold)
            startup_hold = None
            raise RuntimeError("Timed out waiting for startup checkpoint physics freeze")
    backend = BackendProcess(backend_path)
    backend_watch: subprocess.Popen[str] | None = None
    try:
        backend_watch = start_backend_watch(root, backend.process.pid, backend_path)
    except Exception:
        # The native controller is intentionally a private session leader. If its
        # independent owner-loss reaper cannot be armed, never proceed with a
        # control-capable orphan process in the background.
        terminate_private_backend(backend.process)
        backend.close()
        raise
    if campaign_pin is not None:
        verified_inode = verify_running_executable(backend.process.pid, backend_path)
        run_manifest = write_run_manifest(
            runtime,
            campaign_pin,
            sys.argv,
            backend.process.pid,
            verified_inode,
        )
        print(
            f"campaign provenance verified: backendPID={backend.process.pid} inode={verified_inode} "
            f"runManifest={run_manifest}",
            flush=True,
        )
    hud = HUDCompanion(root, configuration)
    backend.snapshot_sink = hud.snapshot
    engaged = False
    completed = False
    crash_guard: subprocess.Popen[str] | None = None
    preentry_warp: subprocess.Popen[str] | None = None
    try:
        ready = backend._read_message(10.0)
        if ready.get("type") != "ready":
            raise RuntimeError(f"Expected backend ready message, got {ready.get('type')!r}")

        backend.send("updateConfiguration", configuration=configuration)
        backend.send("connect", timeout=180.0)
        connected = backend.latest_snapshot
        if not connected or connected.get("connectionStatus") not in {"connected", "failed"}:
            connected = backend.read_until(
                lambda snapshot: snapshot.get("connectionStatus") in {"connected", "failed"},
                180.0,
            )
        if connected.get("connectionStatus") != "connected":
            raise RuntimeError(str(connected.get("lastError") or "kRPC connection failed"))
        backend._handle_snapshot(connected, force=True)

        if args.connect_only:
            # A successful observation/probe is not a failed flight. Preserve the
            # state it intentionally connected to instead of invoking failure restore.
            completed = True
            return 0

        # The holder remains the closed-loop owner through backend/bridge
        # initialization and engagement. Only a successful axis write transfers
        # ownership; preparation here would neutralize the holder's controls.
        if startup_hold is None and (args.reentry_only or args.hac_only) and not skip_reentry_handoff:
            prepare_reentry_direct_control(root)
        elif skip_reentry_handoff:
            print(
                "test-only late-terminal restart: skipping post-connect kRPC handoff",
                flush=True,
            )

        if args.hac_only:
            if not args.live:
                raise RuntimeError("--hac-only is a live terminal-guidance mode; pass --live")
            live_start = backend.latest_snapshot or connected
            start_telemetry = live_start.get("telemetry") or {}
            if BackendProcess._number(start_telemetry.get("meanAltitude")) <= 100.0:
                live_start = backend.read_until(
                    lambda snapshot: BackendProcess._number(
                        (snapshot.get("telemetry") or {}).get("meanAltitude")
                    ) > 100.0,
                    15.0,
                )
                start_telemetry = live_start.get("telemetry") or {}
            start_altitude = BackendProcess._number(start_telemetry.get("meanAltitude"), float("nan"))
            start_speed = BackendProcess._number(start_telemetry.get("trueAirSpeed"), float("nan"))
            if not math.isfinite(start_altitude) or not math.isfinite(start_speed):
                raise RuntimeError("HAC-only test could not read the live altitude/airspeed preflight state")
            if abs(start_altitude - args.hac_start_altitude) > args.hac_altitude_tolerance:
                raise RuntimeError(
                    f"HAC-only test expects about {args.hac_start_altitude / 1000:.1f} km; "
                    f"current altitude is {start_altitude / 1000:.1f} km"
                )
            if abs(start_speed - args.hac_start_speed) > args.hac_speed_tolerance:
                raise RuntimeError(
                    f"HAC-only test expects about {args.hac_start_speed:.0f} m/s; "
                    f"current airspeed is {start_speed:.0f} m/s"
                )
            if not args.no_crash_guard:
                crash_guard = start_crash_guard(root, args.initial_save)
            backend.send("engageHACTest", timeout=180.0)
            armed = backend.latest_snapshot
            if not armed or not armed.get("automationEngaged"):
                armed = backend.read_until(
                    lambda snapshot: bool(snapshot.get("automationEngaged"))
                    or snapshot.get("phase") in {"Abort", "Fault"}
                    or bool(snapshot.get("warningMessage")),
                    15.0,
                )
            if not armed.get("automationEngaged"):
                raise RuntimeError(
                    str(
                        armed.get("lastError")
                        or armed.get("warningMessage")
                        or "HAC-only landing test was not armed"
                    )
                )
            telemetry = armed.get("telemetry") or {}
            guidance_state = armed.get("guidanceState") or {}
            print(
                "HAC-only landing test: "
                f"alt {BackendProcess._number(telemetry.get('meanAltitude')) / 1000:.1f} km, "
                f"airspeed {BackendProcess._number(telemetry.get('trueAirSpeed')):.1f} m/s, "
                f"HAC side {BackendProcess._number(guidance_state.get('hacSide')):+.0f}; "
                "deorbit/entry/S-turn skipped",
                flush=True,
            )
            engaged = True
            release_startup_freeze_after_engagement()
        elif args.reentry_only:
            if not args.live:
                raise RuntimeError("--reentry-only is a live continuation mode; pass --live")
            if not args.no_crash_guard:
                crash_guard = start_crash_guard(root, args.initial_save)
            backend.send("engageReentry", timeout=180.0)
            armed = backend.latest_snapshot
            if not armed or not armed.get("automationEngaged"):
                armed = backend.read_until(
                    lambda snapshot: bool(snapshot.get("automationEngaged"))
                    or snapshot.get("phase") in {"Abort", "Fault"}
                    or bool(snapshot.get("warningMessage")),
                    15.0,
                )
            if not armed.get("automationEngaged"):
                raise RuntimeError(str(armed.get("lastError") or armed.get("warningMessage") or "Reentry-only continuation was not armed"))
            plan = armed.get("plan") or {}
            telemetry = armed.get("telemetry") or {}
            print(
                "reentry continuation: "
                f"UT {BackendProcess._number(telemetry.get('ut')):.1f}, "
                f"alt {BackendProcess._number(telemetry.get('meanAltitude')) / 1000:.1f} km, "
                f"Pe {BackendProcess._number(plan.get('achievedPostBurnPeriapsisAltitude')) / 1000:.1f} km, "
                "deorbit planner/burn skipped",
                flush=True,
            )
            engaged = True
            release_startup_freeze_after_engagement()
            if args.preentry_physics_warp > 1:
                preentry_warp = start_preentry_physics_warp(
                    root,
                    args.preentry_physics_warp,
                    args.preentry_warp_release_altitude,
                )
                print(
                    f"pre-entry coast: up to {args.preentry_physics_warp}x above "
                    f"{args.preentry_warp_release_altitude / 1000:.1f} km; max 2x stable MM304 only above 40 km; hard 1x at/below 40 km and at TAEM",
                    flush=True,
                )
        else:
            preplan_state = probe_ksp(root, timeout=3.0)
            was_paused = bool(preplan_state.get("paused")) if preplan_state else False
            if not was_paused:
                set_ksp_paused(root, True)
                print("orbital planning hold: KSP physics paused while deorbit search runs", flush=True)
            try:
                backend.send("createPlan", timeout=600.0)
            finally:
                if orbital_preplan_hold:
                    set_ksp_paused(root, False)
                    orbital_preplan_hold = False
                    print("orbital planning hold: KSP physics resumed after deorbit plan", flush=True)
                elif not was_paused:
                    set_ksp_paused(root, False)
                    print("orbital planning hold: KSP physics resumed after deorbit plan", flush=True)
            planned = backend.latest_snapshot
            if not planned or not planned.get("plan"):
                planned = backend.read_until(
                    lambda snapshot: bool(snapshot.get("plan")) or snapshot.get("phase") == "Fault",
                    30.0,
                )
            plan = planned.get("plan") or {}
            if not plan:
                raise RuntimeError(str(planned.get("lastError") or "No deorbit plan was produced"))
            print(
                "plan: "
                f"burn UT {BackendProcess._number(plan.get('burnUT')):.1f}, "
                f"delta-v {BackendProcess._number(plan.get('deltaV')):.2f} m/s, "
                f"strict={bool(plan.get('targetCaptureAchieved'))}, "
                f"executable={bool(plan.get('executionQualified'))}",
                flush=True,
            )
            if not plan.get("executionQualified"):
                raise RuntimeError("Planner produced a preview-only plan; live engagement is blocked")
            if args.require_strict and not plan.get("targetCaptureAchieved"):
                raise RuntimeError("Planner did not produce a strict robust deorbit plan; live engagement is blocked")
            if not args.live:
                print("plan validated; pass --live to engage the controller", flush=True)
                return 0

            if not args.no_crash_guard:
                crash_guard = start_crash_guard(root, args.initial_save)
            backend.send("engage", timeout=120.0)
            engaged = True
            # Engagement can immediately trigger a blocking coast-to-burn warp
            # inside the C control thread. The successful command response is
            # sufficient acknowledgement here; waiting for a fresh snapshot would
            # incorrectly treat that intentional quiet period as an engagement
            # failure.

        deadline = (
            time.monotonic() + args.max_wall_seconds
            if args.max_wall_seconds > 0
            else float("inf")
        )
        checkpoint_saved = False
        descent_checkpoint_tracker = DescentCheckpointTracker(descent_checkpoints)
        taem_checkpoint_saved = False
        taem_checkpoint_block_reported = False
        strict_checkpoint_saved = False
        burn_seen = False
        entry_debt_guard = EntryDebtCampaignGuard() if args.enforce_75km_entry_gates else None
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Headless live test exceeded the wall-clock safety limit")
            try:
                message = backend._read_message(min(remaining, 5.0))
            except TimeoutError:
                continue
            if message.get("type") != "snapshot":
                continue
            snapshot = message.get("snapshot") or {}
            backend._handle_snapshot(snapshot)
            telemetry = snapshot.get("telemetry") or {}
            guidance_state = snapshot.get("guidanceState") or {}
            plan = snapshot.get("plan") or {}
            burn_seen = burn_seen or bool(guidance_state.get("burnStarted"))
            altitude = BackendProcess._number(telemetry.get("meanAltitude"), float("inf"))
            vertical_speed = BackendProcess._number(telemetry.get("verticalSpeed"))
            for checkpoint in descent_checkpoint_tracker.update(altitude, vertical_speed):
                backend.send("saveCheckpoint", timeout=30.0, name=checkpoint.name)
                print(
                    f"checkpoint saved: {checkpoint.name} at {altitude / 1000:.1f} km "
                    f"(planned {checkpoint.altitude / 1000:.1f} km)",
                    flush=True,
                )
            if args.taem_checkpoint_name and not taem_checkpoint_saved:
                taem_ready, taem_reason = taem_hac_checkpoint_ready(snapshot, configuration)
                if taem_ready:
                    backend.send("saveCheckpoint", timeout=30.0, name=args.taem_checkpoint_name)
                    taem_checkpoint_saved = True
                    print(
                        f"TAEM HAC contract checkpoint saved: {args.taem_checkpoint_name}; "
                        f"alt {altitude / 1000:.1f} km",
                        flush=True,
                    )
                elif (
                    not taem_checkpoint_block_reported
                    and str(snapshot.get("phase", "")) in {"TAEM", "Heading Alignment"}
                ):
                    print(f"TAEM checkpoint withheld: {taem_reason}", flush=True)
                    taem_checkpoint_block_reported = True
            if entry_debt_guard is not None:
                for decision in entry_debt_guard.update(telemetry):
                    observation = decision.observation
                    print(
                        "75km entry gate: "
                        f"{observation.checkpoint_range_m / 1000:.0f} km | "
                        f"alt {observation.altitude_m / 1000:.2f} km | "
                        f"target {observation.target_altitude_m / 1000:.2f} km | "
                        f"debt {observation.debt_m / 1000:+.2f} km | "
                        f"vs {observation.vertical_speed_mps:+.1f} m/s | {decision.reason}",
                        flush=True,
                    )
                    if decision.reject:
                        if engaged:
                            try:
                                backend.send("abort", timeout=30.0)
                            except Exception:
                                pass
                            engaged = False
                        raise RuntimeError(f"75 km campaign rejected: {decision.reason}")
            if (
                not args.reentry_only
                and burn_seen
                and bool(guidance_state.get("burnCompleted"))
                and bool(plan.get("achievedStateVerified"))
                and preentry_warp is None
                and args.preentry_physics_warp > 1
                and altitude > args.preentry_warp_release_altitude
            ):
                preentry_warp = start_preentry_physics_warp(
                    root,
                    args.preentry_physics_warp,
                    args.preentry_warp_release_altitude,
                )
                print(
                    f"post-burn coast: up to {args.preentry_physics_warp}x above "
                    f"{args.preentry_warp_release_altitude / 1000:.1f} km; max 2x stable MM304 only above 40 km; hard 1x at/below 40 km and at TAEM",
                    flush=True,
                )
            if (
                not args.reentry_only
                and burn_seen
                and bool(guidance_state.get("burnCompleted"))
                and bool(plan.get("achievedStateVerified"))
                and not strict_checkpoint_saved
            ):
                achieved_strict = bool(plan.get("targetCaptureAchieved")) and bool(
                    plan.get("achievedStateCaptureQualified")
                )
                if args.require_strict and not achieved_strict:
                    print(
                        "post-burn strict verification failed: "
                        f"Pe {BackendProcess._number(plan.get('achievedPostBurnPeriapsisAltitude')) / 1000:.1f} km, "
                        f"TAEM {BackendProcess._number(plan.get('predictedTAEMDistance')) / 1000:.1f} km",
                        flush=True,
                    )
                    if engaged:
                        try:
                            backend.send("abort", timeout=30.0)
                        except Exception:
                            pass
                        engaged = False
                    raise RuntimeError("Achieved post-burn state is outside the strict corridor")
                if achieved_strict:
                    if args.strict_checkpoint_name:
                        pitch = BackendProcess._number(telemetry.get("pitch"))
                        roll = signed_angle_degrees(BackendProcess._number(telemetry.get("roll")))
                        heading = BackendProcess._number(telemetry.get("heading"))
                        course = BackendProcess._number(telemetry.get("groundTrackHeading"), heading)
                        heading_error = signed_angle_degrees(course - heading)
                        pitch_rate = abs(BackendProcess._number(telemetry.get("pitchRate")))
                        roll_rate = abs(BackendProcess._number(telemetry.get("rollRate")))
                        heading_rate = abs(BackendProcess._number(telemetry.get("headingRate")))
                        checkpoint_attitude_ready = (
                            abs(pitch - 5.0) <= 4.0
                            and abs(roll) <= 6.0
                            and abs(heading_error) <= 8.0
                            and pitch_rate <= 3.0
                            and roll_rate <= 3.0
                            and heading_rate <= 3.0
                        )
                        if checkpoint_attitude_ready:
                            backend.send("saveCheckpoint", timeout=30.0, name=args.strict_checkpoint_name)
                            print(
                                f"strict post-burn checkpoint saved: {args.strict_checkpoint_name}; "
                                f"Pe {BackendProcess._number(plan.get('achievedPostBurnPeriapsisAltitude')) / 1000:.1f} km; "
                                f"attitude {pitch:+.1f}/{roll:+.1f}/{heading_error:+.1f} deg",
                                flush=True,
                            )
                            strict_checkpoint_saved = True
                    else:
                        print(
                            "post-burn strict verification passed; continuing entry",
                            flush=True,
                        )
                        strict_checkpoint_saved = True
            if (
                not checkpoint_saved
                and args.checkpoint_altitude > 0
                and altitude <= args.checkpoint_altitude
                and vertical_speed < 0
            ):
                backend.send("saveCheckpoint", timeout=30.0, name=args.checkpoint_name)
                checkpoint_saved = True
                print(
                    f"checkpoint saved: {args.checkpoint_name} at {altitude / 1000:.1f} km",
                    flush=True,
                )
            phase = str(snapshot.get("phase", ""))
            if preentry_warp is not None and phase in {
                "TAEM", "Heading Alignment", "Final Approach", "Flare",
                "Touchdown", "Rollout", "Complete",
            }:
                stop_process(preentry_warp)
                preentry_warp = None
                print(f"entry physics warp released at {phase}; terminal flight locked to 1x", flush=True)
            if phase in TERMINAL_PHASES:
                backend._handle_snapshot(snapshot, force=True)
                if phase == "Complete":
                    landing_ok, landing_reason = landing_completion_evidence(snapshot, configuration)
                    if not landing_ok:
                        engaged = False
                        raise RuntimeError(
                            "Complete phase lacks independently verified touchdown/rollout evidence: "
                            + landing_reason
                        )
                    engaged = False
                    completed = True
                    print(
                        f"actual touchdown/rollout verified: {landing_reason}",
                        flush=True,
                    )
                    return 0
                engaged = False
                error = snapshot.get("lastError") or snapshot.get("warningMessage") or snapshot.get("statusMessage")
                raise RuntimeError(f"headless live flight ended in {phase}: {error}")
    except KeyboardInterrupt:
        print("interrupt received; releasing KSP controls", file=sys.stderr, flush=True)
        if engaged:
            try:
                backend.send("abort", timeout=30.0)
            except Exception:
                pass
        return 130
    finally:
        if handoff_path is not None:
            # A pre-engagement failure must NOT authorize the startup holder to
            # resume physics. `failed` is deliberately not a release token, so
            # stop_process(SIGINT) makes the holder retain its freeze through
            # backend shutdown and failure restore. Successful completion may
            # publish `released`.
            with open(handoff_path, "r+") as handoff:
                fcntl.flock(handoff, fcntl.LOCK_EX)
                handoff.seek(0)
                handoff.write(cleanup_handoff_state(completed))
                handoff.truncate()
                handoff.flush()
        stop_process(startup_hold)
        if engaged:
            try:
                backend.send("abort", timeout=30.0)
            except Exception:
                pass
        try:
            backend.send("shutdown", timeout=30.0)
        except Exception:
            if backend.process.poll() is None:
                # Closing stdin lets the native backend leave its command loop
                # without injecting SIGTERM into the native C control stack.
                try:
                    if backend.process.stdin is not None:
                        backend.process.stdin.close()
                except (BrokenPipeError, OSError):
                    pass
        try:
            backend.process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            # Keep the native backend in a private process group so a forced
            # test-run teardown cannot leave its serial/SQLite resources held.
            try:
                os.killpg(backend.process.pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass
            try:
                backend.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(backend.process.pid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
                backend.process.wait(timeout=2.0)
        # A normally exited group should be empty, but clean any descendant
        # that survived a partially completed native shutdown before returning.
        try:
            os.killpg(backend.process.pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass
        backend.close()
        stop_process(backend_watch)
        hud.close()
        if handoff_path is not None:
            os.environ.pop("KSP_LANDER_CONTROL_HANDOFF", None)
            Path(handoff_path).unlink(missing_ok=True)
        stop_process(preentry_warp)
        if orbital_preplan_hold:
            # Reaching outer cleanup with this flag still set means normal plan
            # handoff never resumed physics. Preserve that freeze on failure;
            # successful planning clears the flag in its inner finally block.
            orbital_preplan_hold = False
        # Preserve a successful landed/rollout state for inspection. Every failed
        # or aborted run is fail-closed: restore the campaign baseline when allowed,
        # then freeze physics before releasing the independent crash guard. Keeping
        # the guard alive across the load closes the window where an interrupted
        # runner could leave restored atmospheric physics advancing unattended.
        if not completed:
            try:
                secure_failed_test_state(root, args.initial_save, restore_save=not args.no_quickload)
                if args.no_quickload:
                    print("test cleanup froze failed KSP state", flush=True)
                else:
                    print("test cleanup restored and froze KSP quicksave", flush=True)
            except Exception as exc:
                print(f"warning: final KSP fail-safe cleanup failed: {exc}", file=sys.stderr, flush=True)
        stop_process(crash_guard)
        stop_process(lifecycle_watch)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"headless flight failed: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(1)
