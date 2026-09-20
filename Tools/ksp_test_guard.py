#!/usr/bin/env python3
"""Independent KSP safety guard for live landing tests.

This script intentionally uses its own kRPC connection so it can recover the
simulation even if the landing backend or its Python bridge has already faulted.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import signal
import sys
import subprocess
import threading
import time
from pathlib import Path

import krpc


def emit(payload: dict) -> None:
    print(json.dumps(payload, separators=(",", ":")), flush=True)


def validated_save_name(value: str) -> str:
    name = value.strip()
    if not name or len(name) > 96 or any(ch in name for ch in "/\\:\n\r\t"):
        raise ValueError("Save name must be a simple non-empty file name.")
    return name


def saved_game_ut(name: str) -> float | None:
    """Read the target save's UT when the local KSP save folder is visible."""

    roots: list[Path] = []
    configured = os.environ.get("KSP_SAVE_DIR")
    if configured:
        roots.append(Path(configured).expanduser())
    roots.append(
        Path.home()
        / "Library/Application Support/Steam/steamapps/common/Kerbal Space Program/saves"
    )
    for root in roots:
        if not root.exists():
            continue
        candidates = [root / f"{name}.sfs"] if (root / f"{name}.sfs").exists() else list(root.glob(f"*/{name}.sfs"))
        # Multiple KSP campaigns can each contain a generic quicksave.sfs. When
        # no campaign directory was explicitly configured, prefer the most
        # recently written matching save instead of filesystem/glob order. The
        # latter can silently bind preflight verification to an unrelated game.
        if not configured:
            candidates.sort(key=lambda path: path.stat().st_mtime if path.exists() else -1.0, reverse=True)
        for candidate in candidates:
            try:
                with candidate.open("r", encoding="utf-8", errors="ignore") as handle:
                    for line in handle:
                        stripped = line.strip()
                        if stripped.startswith("UT = "):
                            return float(stripped.split("=", 1)[1].strip())
            except (OSError, ValueError):
                continue
    return None


DEFAULT_KRPC_CONNECT_TIMEOUT = 8.0
ENTRY_PHYSICS_WARP_CUTOFF_ALTITUDE = 40000.0


def _connect_timeout_seconds() -> float:
    raw = os.environ.get("KSP_LANDER_KRPC_CONNECT_TIMEOUT", str(DEFAULT_KRPC_CONNECT_TIMEOUT))
    try:
        timeout = float(raw)
    except ValueError as exc:
        raise ValueError("KSP_LANDER_KRPC_CONNECT_TIMEOUT must be a positive finite number") from exc
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("KSP_LANDER_KRPC_CONNECT_TIMEOUT must be a positive finite number")
    return timeout


def connect(name: str, timeout: float | None = None):
    """Bound the full kRPC handshake, including its otherwise-unbounded reply wait.

    kRPC 0.5.4 repeatedly polls the socket while waiting for the connection reply and
    has no aggregate timeout. Run that handshake in a daemon thread so this guard
    process can fail closed instead of hanging before its first status event. Each
    guard mode is a separate process, so a timed-out daemon thread is discarded when
    the process reports the error and exits.
    """
    timeout_seconds = _connect_timeout_seconds() if timeout is None else float(timeout)
    if not math.isfinite(timeout_seconds) or timeout_seconds <= 0.0:
        raise ValueError("kRPC connect timeout must be a positive finite number")

    outcome: queue.Queue = queue.Queue(maxsize=1)

    def connect_worker() -> None:
        try:
            client = krpc.connect(name=name, address="127.0.0.1", rpc_port=50000, stream_port=50001)
            outcome.put((True, client))
        except Exception as exc:
            outcome.put((False, exc))

    worker = threading.Thread(target=connect_worker, name="krpc-connect", daemon=True)
    worker.start()
    try:
        succeeded, value = outcome.get(timeout=timeout_seconds)
    except queue.Empty as exc:
        raise TimeoutError(
            f"kRPC connection handshake timed out after {timeout_seconds:.1f}s for {name}"
        ) from exc
    if succeeded:
        return value
    if isinstance(value, BaseException):
        raise value
    raise RuntimeError(f"kRPC connection failed for {name}")


def probe() -> int:
    conn = connect("KSP Lander Test Probe")
    try:
        sc = conn.space_center
        vessel = sc.active_vessel
        if vessel is None:
            emit({"ready": False, "reason": "no-active-vessel"})
            return 2
        flight = vessel.flight()
        emit(
            {
                "ready": True,
                "ut": float(sc.ut),
                "vessel": str(vessel.name),
                "situation": str(vessel.situation),
                "meanAltitude": float(flight.mean_altitude),
                "mass": float(vessel.mass),
                "dryMass": float(vessel.dry_mass),
                "availableThrust": float(vessel.available_thrust),
                "railsWarpFactor": int(sc.rails_warp_factor),
                "physicsWarpFactor": int(sc.physics_warp_factor),
                "paused": bool(conn.krpc.paused),
            }
        )
        return 0
    finally:
        conn.close()


def quickload() -> int:
    conn = connect("KSP Lander Test Quickload")
    sc = conn.space_center
    vessel = sc.active_vessel
    emit(
        {
            "event": "quickload-requested",
            "ut": float(sc.ut),
            "vessel": str(vessel.name) if vessel is not None else "",
        }
    )
    # KSP 1 / kRPC 0.5.4 commonly never returns the RPC response because the
    # scene reload invalidates the connection that issued it. The parent test
    # runner therefore treats this as a fire-and-reconnect operation.
    sc.quickload()
    emit({"event": "quickload-returned"})
    return 0


def load_save(name: str) -> int:
    name = validated_save_name(name)
    target_ut = saved_game_ut(name)
    conn = connect("KSP Lander Test Load Save")
    sc = conn.space_center
    vessel = sc.active_vessel
    before_ut = float(sc.ut)
    emit(
        {
            "event": "load-save-requested",
            "name": name,
            "ut": before_ut,
            "vessel": str(vessel.name) if vessel is not None else "",
        }
    )
    # Loading invalidates the requesting RPC connection and KSP 1/kRPC 0.5.4
    # often never returns the corresponding RPC response.  Do not let that
    # dead response block the *verification* path: issue the load on a daemon
    # thread, then immediately reconnect below and treat the first verified UT
    # rewind as authoritative success.  The process exits as soon as settle is
    # proven, so the obsolete requesting connection/thread cannot accumulate.
    def request_load() -> None:
        try:
            sc.load(name)
        except Exception:
            pass

    threading.Thread(target=request_load, name="ksp-load-request", daemon=True).start()

    # For an atmospheric checkpoint, waiting for several "stable" telemetry
    # samples is actively harmful: KSP resumes physics as soon as the scene is
    # live, so a 200+ m/s sink rate can consume kilometres while this helper
    # merely proves the already-loaded vessel is still there.  The authoritative
    # scene-change signal for a saved checkpoint is the UT rewind itself.  Poll
    # quickly and return on the first post-load sample whose UT is older than
    # the request state.  A stale pre-load scene cannot satisfy that condition.
    time.sleep(0.20)
    # This mod-heavy KSP instance has repeatedly needed 35-55 s to rebuild the
    # Flight scene before kRPC accepts a verifying client. Keep this below the
    # headless caller's 120 s outer settle bound, but do not turn a slow valid
    # atmospheric restore into a false scene-did-not-settle failure.
    deadline = time.monotonic() + 75.0
    stable: list[dict] = []
    while time.monotonic() < deadline:
        try:
            check = connect("KSP Lander Test Load Save Settle")
            try:
                check_sc = check.space_center
                scene = check.krpc.current_game_scene
                scene_name = getattr(scene, "name", str(scene)).lower()
                in_flight_scene = scene_name == "flight"
                active = check_sc.active_vessel
                if active is None or not in_flight_scene:
                    stable.clear()
                else:
                    flight = active.flight()
                    state = {
                        "ut": float(check_sc.ut),
                        "vessel": str(active.name),
                        "meanAltitude": float(flight.mean_altitude),
                        "railsWarpFactor": int(check_sc.rails_warp_factor),
                        "physicsWarpFactor": int(check_sc.physics_warp_factor),
                        "gameScene": scene_name,
                    }
                    rewound = state["ut"] < before_ut - 0.25
                    loaded_target = (
                        target_ut is not None
                        and target_ut - 0.5 <= state["ut"] <= target_ut + 8.0
                    )
                    if (rewound or loaded_target) and state["railsWarpFactor"] == 0 and state["physicsWarpFactor"] == 0:
                        try:
                            conn.close()
                        except Exception:
                            pass
                        emit({"event": "load-save-settled", "name": name, **state})
                        return 0
                    if state["railsWarpFactor"] == 0 and state["physicsWarpFactor"] == 0:
                        stable.append(state)
                        if len(stable) >= 2 and abs(stable[-1]["ut"] - stable[0]["ut"]) < 0.05:
                            try:
                                conn.close()
                            except Exception:
                                pass
                            emit({"event": "load-save-settled", "name": name, **state})
                            return 0
                    else:
                        stable.clear()
            finally:
                check.close()
        except Exception:
            stable.clear()
        time.sleep(0.10)
    try:
        conn.close()
    except Exception:
        pass
    emit({"event": "load-save-failed", "name": name, "reason": "scene-did-not-settle"})
    return 2


def prepare_reentry() -> int:
    conn = connect("KSP Lander Reentry Direct-Control Handoff")
    try:
        sc = conn.space_center
        vessel = sc.active_vessel
        if vessel is None:
            emit({"event": "reentry-handoff-failed", "reason": "no-active-vessel"})
            return 2
        sc.rails_warp_factor = 0
        sc.physics_warp_factor = 0
        try:
            vessel.auto_pilot.disengage()
        except Exception:
            pass
        control = vessel.control
        control.sas = False
        control.throttle = 0.0
        control.pitch = 0.0
        control.roll = 0.0
        control.yaw = 0.0
        # kRPC control writes are consumed by a later physics update. Closing
        # the client immediately can discard a pending zero-throttle write,
        # leaving the save's inherited throttle burning fuel during settling.
        handoff_ut = float(sc.ut)
        if not bool(conn.krpc.paused):
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                control.throttle = 0.0
                if float(sc.ut) - handoff_ut >= 0.06 and abs(float(control.throttle)) <= 1e-4:
                    break
                time.sleep(0.02)
            else:
                emit({"event": "reentry-handoff-failed", "reason": "neutral-controls-not-applied"})
                return 2
        actual_thrust = float(vessel.thrust)
        if not math.isfinite(actual_thrust) or abs(actual_thrust) > 1.0:
            emit({
                "event": "reentry-handoff-failed",
                "reason": "nonzero-actual-thrust",
                "actualThrust": actual_thrust,
            })
            return 2
        flight = vessel.flight()
        emit(
            {
                "event": "reentry-direct-control-ready",
                "ut": float(sc.ut),
                "vessel": str(vessel.name),
                "meanAltitude": float(flight.mean_altitude),
                "sas": bool(control.sas),
                "throttle": float(control.throttle),
                "actualThrust": actual_thrust,
                "availableThrust": float(vessel.available_thrust),
                "appliedAfterUT": float(sc.ut) - handoff_ut,
                "railsWarpFactor": int(sc.rails_warp_factor),
                "physicsWarpFactor": int(sc.physics_warp_factor),
            }
        )
        return 0
    finally:
        conn.close()


def set_pause(paused: bool) -> int:
    conn = connect("KSP Lander Test Pause Control")
    try:
        conn.krpc.paused = paused
        emit({"event": "pause-state", "paused": bool(conn.krpc.paused)})
        return 0
    finally:
        conn.close()


def startup_hold(interval: float, parent_pid: int | None = None) -> int:
    """Freeze physics while the full backend cold-starts, then hand off.

    Replaying an atmospheric checkpoint must preserve the saved rigid-body
    state exactly.  The former helper fed surface-frame Euler pitch/roll back
    through the TAEM direct controller while the bridge was still connecting.
    Across a KSP scene reload those Euler values can transiently select the
    opposite attitude branch, so the supposed "hold" could rotate the shuttle
    before the real controller ever owned it.

    Instead, pause KSP physics before touching flight state. While physics is
    frozen, explicitly disengage AutoPilot and force SAS off so a saved SAS
    mode can never produce the first post-load attitude motion. Do not write
    pitch/roll/yaw axes here. The backend's first successful direct-axis write
    publishes ``backend`` in the shared handoff file; only then do we unpause.
    """

    handoff_path = os.environ.get("KSP_LANDER_CONTROL_HANDOFF")
    if not handoff_path:
        emit({"event": "startup-hold-failed", "reason": "missing-control-handoff"})
        return 2

    conn = connect("KSP Lander Startup Physics Freeze")
    sc = conn.space_center
    # Freeze simulation before *any* scene/flight inspection.  On a busy KSP
    # scene, querying the active vessel, zeroing warp and constructing Flight
    # can consume multiple seconds of simulation time.  Doing those RPCs
    # before the pause was the remaining startup drift after the old attitude
    # controller was removed.
    was_paused = bool(conn.krpc.paused)
    if not was_paused:
        conn.krpc.paused = True
    paused_by_us = (not was_paused) or (
        was_paused and os.environ.get("KSP_LANDER_STARTUP_ADOPT_PAUSE") == "1"
    )
    vessel = sc.active_vessel
    if vessel is None:
        # Startup failure is fail-closed. Retain the pause so a scene with no
        # active vessel cannot resume physics before the owning runner reports
        # the error and its parent watcher takes over.
        emit({"event": "startup-hold-failed", "reason": "no-active-vessel", "paused": bool(conn.krpc.paused)})
        conn.close()
        return 2
    sc.rails_warp_factor = 0
    sc.physics_warp_factor = 0
    # Reentry control is direct-only. A quicksave may persist SAS as enabled;
    # clear it while physics is frozen so there is no SAS-owned frame between
    # scene load and the backend's first direct-axis command.
    try:
        vessel.auto_pilot.disengage()
    except Exception:
        pass
    try:
        vessel.control.sas = False
    except Exception:
        pass
    flight = vessel.flight()
    initial_ut = float(sc.ut)
    initial_altitude = float(flight.mean_altitude)
    initial_pitch = float(flight.pitch)
    initial_roll = float(flight.roll)
    stopped = False
    release_authorized = False

    def interrupt(_signum, _frame):
        nonlocal stopped
        stopped = True

    previous_sigterm = signal.signal(signal.SIGTERM, interrupt)
    previous_sigint = signal.signal(signal.SIGINT, interrupt)
    emit(
        {
            "event": "startup-hold-armed",
            "mode": "paused-physics",
            "ut": initial_ut,
            "meanAltitude": initial_altitude,
            "targetPitch": initial_pitch,
            "targetRoll": initial_roll,
        }
    )
    try:
        # Publish readiness only after KSP has actually acknowledged the
        # paused state.  The headless runner waits for this marker before it
        # starts the much heavier backend/stream connection, eliminating the
        # last race in which physics could advance during startup.
        import fcntl
        with open(handoff_path, "r+", encoding="utf-8") as handoff:
            fcntl.flock(handoff, fcntl.LOCK_EX)
            owner = handoff.read().strip()
            if owner not in ("backend", "released"):
                handoff.seek(0)
                handoff.write("startup-paused")
                handoff.truncate()
                handoff.flush()
        next_pause_reassert = 0.0
        while not stopped:
            if not process_alive(parent_pid):
                emit(
                    {
                        "event": "startup-hold-parent-exit",
                        "parentPid": parent_pid,
                        "paused": bool(conn.krpc.paused),
                        "ut": float(sc.ut),
                    }
                )
                break
            with open(handoff_path, "r", encoding="utf-8") as handoff:
                fcntl.flock(handoff, fcntl.LOCK_SH)
                owner = handoff.read().strip()
            if owner in ("engaged", "released"):
                release_authorized = True
                break
            # Opening a new kRPC client can clear KSP's pause state even when
            # that client never explicitly requests a resume.  A single pause
            # setter therefore cannot protect an atmospheric checkpoint across
            # backend cold-start.  Reassert the pause at a modest cadence until
            # guidance is actually armed.  This is intentionally not a busy
            # 20 Hz setter: 5 Hz is enough to bound startup drift to a fraction
            # of a second without loading the kRPC server during connection.
            now = time.monotonic()
            if now >= next_pause_reassert:
                try:
                    conn.krpc.paused = True
                except Exception:
                    pass
                next_pause_reassert = now + 0.20
            time.sleep(max(0.02, interval))
        if stopped and process_alive(parent_pid):
            # Orderly runner cleanup writes released before signaling the holder.
            # Re-read once so a cooperative SIGINT cannot hide that authorization.
            try:
                with open(handoff_path, "r", encoding="utf-8") as handoff:
                    fcntl.flock(handoff, fcntl.LOCK_SH)
                    release_authorized = handoff.read().strip() in ("engaged", "released")
            except OSError:
                release_authorized = False
        if release_authorized and paused_by_us:
            conn.krpc.paused = False
            paused_by_us = False
        emit(
            {
                "event": "startup-hold-released" if release_authorized else "startup-hold-retained",
                "mode": "paused-physics",
                "paused": bool(conn.krpc.paused),
                "ut": float(sc.ut),
                "meanAltitude": float(flight.mean_altitude),
                "pitch": float(flight.pitch),
                "roll": float(flight.roll),
                "deltaUT": float(sc.ut) - initial_ut,
                "deltaAltitude": float(flight.mean_altitude) - initial_altitude,
            }
        )
        return 0
    finally:
        # Never release a startup freeze merely because this helper was signaled.
        # Only the explicit engaged/released handoff can authorize resuming KSP.
        if release_authorized and paused_by_us:
            try:
                conn.krpc.paused = False
            except Exception:
                pass
        signal.signal(signal.SIGTERM, previous_sigterm)
        signal.signal(signal.SIGINT, previous_sigint)
        conn.close()


def stop_warp_for_parent_loss(conn, sc, parent_pid: int | None) -> bool:
    if process_alive(parent_pid):
        return False
    sc.rails_warp_factor = 0
    sc.physics_warp_factor = 0
    conn.krpc.paused = True
    emit(
        {
            "event": "preentry-warp-parent-exit",
            "parentPid": parent_pid,
            "paused": bool(conn.krpc.paused),
            "railsWarpFactor": int(sc.rails_warp_factor),
            "physicsWarpFactor": int(sc.physics_warp_factor),
            "ut": float(sc.ut),
        }
    )
    return True


def preentry_warp(multiplier: int, release_altitude: float, interval: float, parent_pid: int | None = None) -> int:
    """Own KSP physics warp from the post-burn coast through MM304.

    KSP exposes physics_warp_factor as an index: 0=1x, 1=2x, 2=3x,
    3=4x. Above release_altitude the requested pre-entry multiplier may be
    used while the low-incidence coast attitude is settled. Between
    release_altitude and 40 km the helper caps itself at 2x and uses an
    aerodynamic-entry stability envelope. At or below 40 km physics warp is
    forbidden: the helper forces 1x and exits, so it cannot re-arm later.
    """

    factor = max(0, min(3, multiplier - 1))
    entry_factor = min(factor, 1)  # Never exceed 2x once aerodynamic entry starts.
    conn = connect("KSP Lander Entry Physics Warp")
    sc = conn.space_center
    if stop_warp_for_parent_loss(conn, sc, parent_pid):
        conn.close()
        return 0
    armed = False
    stable_since: float | None = None
    last_ut: float | None = None
    last_pitch = last_roll = last_heading = last_aoa = 0.0
    active_factor = factor
    entry_mode = False

    def signed_delta(value: float, previous: float) -> float:
        return ((value - previous + 180.0) % 360.0) - 180.0

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    previous_sigterm = signal.signal(signal.SIGTERM, interrupt)
    previous_sigint = signal.signal(signal.SIGINT, interrupt)
    try:
        vessel = sc.active_vessel
        if vessel is None:
            emit({"event": "entry-warp-skipped", "reason": "no-active-vessel"})
            return 2
        flight = vessel.flight()
        altitude = float(flight.mean_altitude)
        if factor == 0:
            sc.rails_warp_factor = 0
            sc.physics_warp_factor = 0
            emit({"event": "entry-warp-skipped", "reason": "1x-requested", "altitude": altitude})
            return 0

        if altitude <= ENTRY_PHYSICS_WARP_CUTOFF_ALTITUDE:
            sc.rails_warp_factor = 0
            sc.physics_warp_factor = 0
            emit({
                "event": "entry-warp-disabled",
                "reason": "below-40km-hard-cutoff",
                "altitude": altitude,
                "cutoffAltitude": ENTRY_PHYSICS_WARP_CUTOFF_ALTITUDE,
                "physicsWarpFactor": int(sc.physics_warp_factor),
            })
            return 0
        entry_mode = altitude <= release_altitude
        active_factor = entry_factor if entry_mode else factor
        sc.rails_warp_factor = 0
        sc.physics_warp_factor = 0
        emit(
            {
                "event": "entry-warp-waiting-for-stability",
                "requestedMultiplier": factor + 1,
                "activeMultiplier": active_factor + 1,
                "altitude": altitude,
                "entryCapAltitude": release_altitude,
                "entryMode": entry_mode,
            }
        )

        while True:
            if stop_warp_for_parent_loss(conn, sc, parent_pid):
                return 0
            active = sc.active_vessel
            if active is None:
                time.sleep(interval)
                continue
            if active != vessel:
                vessel = active
                flight = vessel.flight()
                last_ut = None
                stable_since = None
                armed = False
                sc.physics_warp_factor = 0

            altitude = float(flight.mean_altitude)
            if altitude <= ENTRY_PHYSICS_WARP_CUTOFF_ALTITUDE:
                sc.rails_warp_factor = 0
                sc.physics_warp_factor = 0
                emit({
                    "event": "entry-warp-disabled",
                    "reason": "below-40km-hard-cutoff",
                    "altitude": altitude,
                    "cutoffAltitude": ENTRY_PHYSICS_WARP_CUTOFF_ALTITUDE,
                    "physicsWarpFactor": int(sc.physics_warp_factor),
                })
                return 0
            now_entry_mode = altitude <= release_altitude
            if now_entry_mode and not entry_mode:
                # Crossing into the aerodynamic-entry warp regime is always made
                # at 1x first. Re-arm 2x only after the new attitude/rate envelope
                # has remained settled for the dwell below.
                sc.physics_warp_factor = 0
                armed = False
                stable_since = None
                entry_mode = True
                active_factor = entry_factor
                emit(
                    {
                        "event": "entry-warp-capped",
                        "altitude": altitude,
                        "entryCapAltitude": release_altitude,
                        "maximumMultiplier": entry_factor + 1,
                        "physicsWarpFactor": int(sc.physics_warp_factor),
                    }
                )
            elif not now_entry_mode:
                entry_mode = False

            pitch = float(flight.pitch)
            roll = ((float(flight.roll) + 180.0) % 360.0) - 180.0
            heading = float(flight.heading)
            aoa = float(flight.angle_of_attack)
            sideslip = float(flight.sideslip_angle)
            ut = float(sc.ut)
            finite = all(math.isfinite(value) for value in (pitch, roll, heading, aoa, sideslip, ut))

            pitch_rate = roll_rate = heading_rate = aoa_rate = 0.0
            rates_ready = False
            if finite and last_ut is not None:
                dt_ut = ut - last_ut
                if 0.01 <= dt_ut <= 3.0:
                    pitch_rate = abs(signed_delta(pitch, last_pitch) / dt_ut)
                    roll_rate = abs(signed_delta(roll, last_roll) / dt_ut)
                    heading_rate = abs(signed_delta(heading, last_heading) / dt_ut)
                    aoa_rate = abs(signed_delta(aoa, last_aoa) / dt_ut)
                    rates_ready = True
            if finite:
                last_ut = ut
                last_pitch, last_roll, last_heading, last_aoa = pitch, roll, heading, aoa

            if entry_mode:
                # MM304 legitimately flies high AoA and up to 45 deg bank. What
                # matters for physics warp is that the controlled attitude is not
                # diverging. These rate bounds are deliberately above the normal
                # 6-8 deg/s command limits, but below a loss-of-control transient.
                stable = (
                    finite
                    and rates_ready
                    and abs(aoa) <= 34.0
                    and abs(pitch) <= 42.0
                    and abs(roll) <= 52.0
                    and abs(sideslip) <= 18.0
                    and pitch_rate <= 7.0
                    and roll_rate <= 9.0
                    and heading_rate <= 8.0
                    and aoa_rate <= 6.0
                )
                unsafe = (
                    (not finite)
                    or abs(aoa) > 39.0
                    or abs(pitch) > 52.0
                    or abs(roll) > 62.0
                    or abs(sideslip) > 26.0
                    or (rates_ready and (pitch_rate > 12.0 or roll_rate > 15.0 or heading_rate > 12.0 or aoa_rate > 10.0))
                )
            else:
                stable = (
                    finite
                    and abs(aoa) <= 8.0
                    and abs(pitch) <= 20.0
                    and abs(roll) <= 12.0
                    and abs(sideslip) <= 12.0
                )
                unsafe = (
                    (not finite)
                    or abs(aoa) > 15.0
                    or abs(pitch) > 35.0
                    or abs(roll) > 22.0
                    or abs(sideslip) > 24.0
                )

            now = time.monotonic()
            if armed and unsafe:
                sc.physics_warp_factor = 0
                armed = False
                stable_since = None
                previous_factor = active_factor
                if entry_mode:
                    # Fail closed once aerodynamic-entry warp has observed an unsafe
                    # attitude/rate deviation. Re-arming 2x after the MM304 vehicle
                    # briefly settles can reintroduce the same sampled-control upset
                    # a few hundred metres lower, exactly before the 50 km checkpoint.
                    # Force 1x from here; the headless runner can continue normally.
                    active_factor = 0
                else:
                    active_factor = max(0, active_factor - 1)
                emit(
                    {
                        "event": "entry-warp-suspended",
                        "reason": "attitude-or-rate-deviation",
                        "entryMode": entry_mode,
                        "altitude": altitude,
                        "pitch": pitch,
                        "roll": roll,
                        "angleOfAttack": aoa,
                        "sideslip": sideslip,
                        "pitchRate": pitch_rate,
                        "rollRate": roll_rate,
                        "headingRate": heading_rate,
                        "aoaRate": aoa_rate,
                        "previousMultiplier": previous_factor + 1,
                        "eligibleMultiplier": active_factor + 1,
                        "physicsWarpFactor": int(sc.physics_warp_factor),
                    }
                )
                if not entry_mode and active_factor == 0:
                    emit({"event": "entry-warp-disabled", "reason": "preentry-backoff-reached-1x", "altitude": altitude})
                    return 0
            elif armed and int(sc.physics_warp_factor) != active_factor:
                # A checkpoint/save helper or another safety owner may temporarily
                # force physics warp back to 1x without changing this helper's
                # internal armed flag. Treat that as a disarm and require the same
                # stability dwell before restoring the eligible multiplier.
                observed_factor = int(sc.physics_warp_factor)
                armed = False
                stable_since = now if stable else None
                emit(
                    {
                        "event": "entry-warp-external-override",
                        "entryMode": entry_mode,
                        "altitude": altitude,
                        "observedMultiplier": observed_factor + 1,
                        "eligibleMultiplier": active_factor + 1,
                    }
                )
            elif not armed:
                if stable and active_factor > 0:
                    if stable_since is None:
                        stable_since = now
                    elif now - stable_since >= (1.0 if entry_mode else 1.5):
                        sc.physics_warp_factor = active_factor
                        armed = True
                        emit(
                            {
                                "event": "entry-warp-armed",
                                "multiplier": active_factor + 1,
                                "physicsWarpFactor": int(sc.physics_warp_factor),
                                "entryMode": entry_mode,
                                "altitude": altitude,
                                "pitch": pitch,
                                "roll": roll,
                                "angleOfAttack": aoa,
                                "sideslip": sideslip,
                                "pitchRate": pitch_rate,
                                "rollRate": roll_rate,
                                "headingRate": heading_rate,
                                "aoaRate": aoa_rate,
                                "entryCapAltitude": release_altitude,
                            }
                        )
                else:
                    stable_since = None
            time.sleep(interval)
    finally:
        try:
            sc.rails_warp_factor = 0
            sc.physics_warp_factor = 0
            emit({"event": "entry-warp-cleanup", "physicsWarpFactor": int(sc.physics_warp_factor)})
        except Exception:
            pass
        try:
            conn.close()
        finally:
            signal.signal(signal.SIGTERM, previous_sigterm)
            signal.signal(signal.SIGINT, previous_sigint)

def surface_motion_flight(vessel):
    """Use the rotating planet frame for descent relative to the terrain.

    Vessel.flight() defaults to a frame moving with the vessel. Its velocity,
    including vertical_speed, is therefore zero during an actual descent.
    Keep attitude observers in their local surface frame; only motion and
    impact evaluation use this planet-fixed frame.
    """
    return vessel.flight(vessel.orbit.body.reference_frame)


def impact_reason(vessel, flight) -> str | None:
    situation = str(vessel.situation).split(".")[-1].lower()
    if situation == "splashed":
        return "unexpected-splashed"

    radar = float(flight.surface_altitude)
    vertical = float(flight.vertical_speed)
    g_force = float(flight.g_force)
    if not all(math.isfinite(value) for value in (radar, vertical, g_force)):
        return None
    # A normal runway touchdown changes the vessel situation to ``landed``
    # while it is still rolling. Do not erase a successful landing merely
    # because KSP made that state transition; the pre-impact descent and g
    # checks below remain responsible for catching hard contacts.
    if situation == "landed" and vertical >= -8.0 and g_force <= 6.0:
        return None
    if radar < -5.0:
        return "below-terrain"
    if g_force > 12.0 and radar < 1500.0:
        return f"impact-g-{g_force:.1f}"
    if vertical >= -8.0:
        return None
    time_to_ground = radar / max(-vertical, 0.1)
    if radar < 1200.0 and vertical < -60.0:
        return f"terminal-descent-{radar:.0f}m-{vertical:.0f}mps"
    if radar < 500.0 and vertical < -25.0 and time_to_ground < 8.0:
        return f"impact-imminent-{time_to_ground:.1f}s"
    if radar < 150.0 and vertical < -12.0 and time_to_ground < 5.0:
        return f"impact-imminent-{time_to_ground:.1f}s"
    return None


def process_alive(pid: int | None) -> bool:
    if pid is None or pid <= 0:
        return True
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True

    # kill(pid, 0) also succeeds for an unreaped zombie. Parent-loss sentinels
    # must treat that state as dead or they can wait forever after their owner
    # exits. If ps itself fails unexpectedly, conservatively assume the process
    # is alive rather than killing a potentially unrelated PID.
    try:
        completed = subprocess.run(
            ["ps", "-p", str(pid), "-o", "state="],
            capture_output=True,
            text=True,
            timeout=1.0,
            check=False,
        )
    except Exception:
        return True
    if completed.returncode != 0:
        return False
    state = completed.stdout.strip()
    if not state:
        return False
    return not state.startswith("Z")


def backend_process_matches(pid: int, expected_executable: str) -> bool:
    """Confirm a PID still names the exact backend executable before group kill."""
    if not process_alive(pid):
        return False
    try:
        completed = subprocess.run(
            ["ps", "-p", str(pid), "-o", "comm="],
            capture_output=True,
            text=True,
            timeout=1.0,
            check=False,
        )
    except Exception:
        return False
    if completed.returncode != 0:
        return False
    observed = completed.stdout.strip()
    if not observed:
        return False
    try:
        return Path(observed).resolve() == Path(expected_executable).resolve()
    except OSError:
        return observed == expected_executable


def backend_watch(interval: float, parent_pid: int | None, backend_pid: int | None, backend_executable: str) -> int:
    """Kill only the verified private backend group after its runner disappears."""
    if parent_pid is None or parent_pid <= 0 or backend_pid is None or backend_pid <= 0 or not backend_executable:
        emit({"event": "backend-watch-failed", "reason": "missing-identity"})
        return 2
    emit({"event": "backend-watch-armed", "parentPid": parent_pid, "backendPid": backend_pid})
    try:
        while process_alive(parent_pid):
            if not process_alive(backend_pid):
                emit({"event": "backend-watch-backend-exit", "backendPid": backend_pid})
                return 0
            time.sleep(max(0.02, interval))
    except KeyboardInterrupt:
        emit({"event": "backend-watch-stopped", "backendPid": backend_pid})
        return 0

    if not process_alive(backend_pid):
        emit({"event": "backend-watch-backend-exit", "backendPid": backend_pid})
        return 0
    try:
        pgid = os.getpgid(backend_pid)
    except ProcessLookupError:
        emit({"event": "backend-watch-backend-exit", "backendPid": backend_pid})
        return 0
    if pgid != backend_pid or not backend_process_matches(backend_pid, backend_executable):
        emit(
            {
                "event": "backend-watch-refused",
                "backendPid": backend_pid,
                "observedPgid": pgid,
                "expectedExecutable": backend_executable,
            }
        )
        return 2

    emit({"event": "backend-watch-parent-exit", "parentPid": parent_pid, "backendPid": backend_pid})
    try:
        os.killpg(backend_pid, signal.SIGTERM)
    except ProcessLookupError:
        return 0
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and process_alive(backend_pid):
        time.sleep(max(0.02, interval))
    if process_alive(backend_pid):
        # Re-verify both private-group identity and executable immediately before
        # escalation so PID reuse cannot redirect SIGKILL at an unrelated process.
        try:
            pgid = os.getpgid(backend_pid)
        except ProcessLookupError:
            return 0
        if pgid != backend_pid or not backend_process_matches(backend_pid, backend_executable):
            emit({"event": "backend-watch-escalation-refused", "backendPid": backend_pid, "observedPgid": pgid})
            return 2
        os.killpg(backend_pid, signal.SIGKILL)
    emit({"event": "backend-watch-reaped", "backendPid": backend_pid})
    return 0


def parent_watch(interval: float, parent_pid: int | None) -> int:
    """Freeze KSP only if the owning headless runner disappears."""
    if parent_pid is None or parent_pid <= 0:
        emit({"event": "parent-watch-failed", "reason": "missing-parent-pid"})
        return 2
    emit({"event": "parent-watch-armed", "parentPid": parent_pid})
    try:
        while process_alive(parent_pid):
            time.sleep(max(0.02, interval))
    except KeyboardInterrupt:
        emit({"event": "parent-watch-stopped", "parentPid": parent_pid})
        return 0

    # Do not keep a kRPC connection while the parent is alive: scene loads
    # invalidate it, and opening clients can perturb pause state. Contact KSP
    # only after owner loss, retrying across any in-progress scene transition.
    emit({"event": "parent-watch-parent-exit", "parentPid": parent_pid})
    deadline = time.monotonic() + 30.0
    last_error = ""
    while time.monotonic() < deadline:
        conn = None
        try:
            conn = connect("KSP Lander Parent Loss Freeze", timeout=2.0)
            conn.krpc.paused = True
            paused = bool(conn.krpc.paused)
            emit(
                {
                    "event": "parent-watch-frozen",
                    "parentPid": parent_pid,
                    "paused": paused,
                    "ut": float(conn.space_center.ut),
                }
            )
            return 0 if paused else 2
        except Exception as exc:
            last_error = f"{type(exc).__name__}: {exc}"
        finally:
            if conn is not None:
                try:
                    conn.close()
                except Exception:
                    pass
        time.sleep(max(0.05, interval))
    emit({"event": "parent-watch-failed", "parentPid": parent_pid, "reason": last_error[:240]})
    return 2


def freeze_recovered_scene(interval: float = 0.10, timeout: float = 30.0) -> bool:
    """Reconnect after a scene load and leave the recovered checkpoint frozen."""
    deadline = time.monotonic() + max(0.1, timeout)
    last_error = ""
    while time.monotonic() < deadline:
        recovery_conn = None
        try:
            recovery_conn = connect("KSP Lander Crash Recovery Freeze", timeout=2.0)
            recovery_sc = recovery_conn.space_center
            recovery_sc.rails_warp_factor = 0
            recovery_sc.physics_warp_factor = 0
            recovery_conn.krpc.paused = True
            paused = bool(recovery_conn.krpc.paused)
            emit(
                {
                    "event": "guard-recovery-frozen",
                    "paused": paused,
                    "ut": float(recovery_sc.ut),
                }
            )
            return paused
        except Exception as exc:
            last_error = f"{type(exc).__name__}: {exc}"
        finally:
            if recovery_conn is not None:
                try:
                    recovery_conn.close()
                except Exception:
                    pass
        time.sleep(max(0.05, interval))
    emit({"event": "guard-recovery-freeze-failed", "reason": last_error[:240]})
    return False


def request_recovery_and_freeze(sc, recovery_save: str, interval: float) -> bool:
    """Request the crash-policy restore and freeze it even if load invalidates RPC."""
    try:
        if recovery_save == "quicksave":
            sc.quickload()
        else:
            sc.load(validated_save_name(recovery_save))
    except Exception as exc:
        # Scene-load RPCs can invalidate their own transport after KSP accepts
        # the request. Treat that as an expected transition and still reconnect
        # to prove the recovered scene is frozen before this guard exits.
        emit({"event": "guard-recovery-load-transition", "error": f"{type(exc).__name__}: {exc}"[:240]})
    return freeze_recovered_scene(interval=max(0.05, interval), timeout=30.0)


def freeze_for_parent_loss(conn, sc, parent_pid: int | None) -> bool:
    if process_alive(parent_pid):
        return False
    conn.krpc.paused = True
    emit(
        {
            "event": "guard-parent-exit",
            "parentPid": parent_pid,
            "paused": bool(conn.krpc.paused),
            "ut": float(sc.ut),
        }
    )
    return True


def monitor(interval: float, recovery_save: str = "quicksave", parent_pid: int | None = None) -> int:
    conn = connect("KSP Lander Crash Guard")
    try:
        sc = conn.space_center
        if freeze_for_parent_loss(conn, sc, parent_pid):
            return 0
        vessel = sc.active_vessel
        if vessel is None:
            emit({"event": "guard-failed", "reason": "no-active-vessel"})
            return 2
        flight = surface_motion_flight(vessel)
        emit({"event": "guard-armed", "vessel": str(vessel.name), "ut": float(sc.ut)})
        while True:
            try:
                if freeze_for_parent_loss(conn, sc, parent_pid):
                    return 0
                active = sc.active_vessel
                if active is None:
                    time.sleep(interval)
                    continue
                if active != vessel:
                    vessel = active
                    flight = surface_motion_flight(vessel)
                reason = impact_reason(vessel, flight)
                if reason:
                    emit(
                        {
                            "event": "guard-recovery",
                            "reason": reason,
                            "ut": float(sc.ut),
                            "vessel": str(vessel.name),
                            "radarAltitude": float(flight.surface_altitude),
                            "verticalSpeed": float(flight.vertical_speed),
                        }
                    )
                    try:
                        vessel.control.throttle = 0.0
                    except Exception:
                        pass
                    try:
                        vessel.auto_pilot.disengage()
                    except Exception:
                        pass
                    return 0 if request_recovery_and_freeze(sc, recovery_save, interval) else 2
            except Exception as exc:
                # A scene transition can temporarily invalidate proxies. The
                # guard stays alive and reconnects rather than assuming KSP is
                # gone while a recovery load is taking place.
                emit({"event": "guard-reconnect", "error": str(exc)[:240]})
                try:
                    conn.close()
                except Exception:
                    pass
                time.sleep(max(interval, 0.25))
                conn = connect("KSP Lander Crash Guard")
                sc = conn.space_center
                vessel = sc.active_vessel
                if vessel is not None:
                    flight = surface_motion_flight(vessel)
            time.sleep(interval)
    finally:
        try:
            conn.close()
        except Exception:
            pass


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("probe", "quickload", "load-save", "prepare-reentry", "pause", "resume", "monitor", "preentry-warp", "startup-hold", "parent-watch", "backend-watch"))
    parser.add_argument("--name", default="")
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--multiplier", type=int, choices=(1, 2, 3, 4), default=4)
    parser.add_argument("--release-altitude", type=float, default=72000.0)
    parser.add_argument("--parent-pid", type=int, default=0)
    parser.add_argument("--backend-pid", type=int, default=0)
    parser.add_argument("--backend-executable", default="")
    args = parser.parse_args()
    if args.mode == "probe":
        return probe()
    if args.mode == "quickload":
        return quickload()
    if args.mode == "load-save":
        return load_save(args.name)
    if args.mode == "prepare-reentry":
        return prepare_reentry()
    if args.mode == "pause":
        return set_pause(True)
    if args.mode == "resume":
        return set_pause(False)
    if args.mode == "preentry-warp":
        return preentry_warp(args.multiplier, args.release_altitude, max(0.02, args.interval), args.parent_pid or None)
    if args.mode == "startup-hold":
        return startup_hold(max(0.02, args.interval), args.parent_pid or None)
    if args.mode == "parent-watch":
        return parent_watch(max(0.02, args.interval), args.parent_pid or None)
    if args.mode == "backend-watch":
        return backend_watch(max(0.02, args.interval), args.parent_pid or None, args.backend_pid or None, args.backend_executable)
    return monitor(max(0.05, args.interval), args.name or "quicksave", args.parent_pid or None)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        emit({"event": "guard-error", "error": f"{type(exc).__name__}: {exc}"})
        raise SystemExit(1)
