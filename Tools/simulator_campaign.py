#!/usr/bin/env python3
"""Run Guidance closed-loop against standalone ShuttleSim at maximum speed.

Guidance remains the sole controller. This process only orchestrates the two
executables, mirrors exact Guidance snapshots for Telemetry Web, and records
replay artifacts.
"""
from __future__ import annotations

import argparse
import gzip
import json
import math
import os
from pathlib import Path
import signal
import socket
import shutil
import subprocess
import sys
import time
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "Tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from clanding_layout import build_artifact as clanding_build_artifact, source_root as clanding_source_root
from headless_flight import BackendProcess, load_configuration

TERMINAL_PHASES = {"Complete", "Abort", "Fault"}


def _downsample_trajectory(points: Any, limit: int = 120) -> list[Any]:
    if not isinstance(points, list) or len(points) <= limit:
        return points if isinstance(points, list) else []
    scale = (len(points) - 1) / float(limit - 1)
    indices = sorted({min(len(points) - 1, int(round(i * scale))) for i in range(limit)})
    return [points[i] for i in indices]


def _lean_snapshot(snapshot: dict[str, Any]) -> dict[str, Any]:
    lean = dict(snapshot)
    # Strip actualTrajectory: completely redundant with simulator-telemetry and discarded by replay
    lean["actualTrajectory"] = []
    # Downsample trajectory arrays to max 120 points
    for key in ("predictedTrajectory", "plannedTrajectory", "projectedTAEMTrajectory",
                "referenceTrajectory", "orbitalTrajectory"):
        if key in lean and isinstance(lean[key], list):
            lean[key] = _downsample_trajectory(lean[key], 120)
    # Plan trajectory is huge (1000 items, 113KB) and redundant with plannedTrajectory
    plan = lean.get("plan")
    if isinstance(plan, dict):
        plan_copy = dict(plan)
        if "trajectory" in plan_copy:
            plan_copy["trajectory"] = _downsample_trajectory(plan_copy["trajectory"], 60)
        lean["plan"] = plan_copy
    return lean


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, separators=(",", ":"), allow_nan=False), encoding="utf-8")
    os.replace(temp, path)


class SnapshotRecorder:
    def __init__(self, run_dir: Path, mirror: Path, manifest: dict[str, Any]) -> None:
        self.run_dir = run_dir
        self.mirror = mirror
        self.path = run_dir / "guidance-snapshots.jsonl"
        self.handle = self.path.open("w", encoding="utf-8", buffering=1)
        self.manifest = manifest
        self.count = 0
        self.last_sim_ut: float | None = None
        self.last_recorded_sim_ut: float | None = None
        self.last_phase = ""

    def close(self) -> None:
        self.handle.close()

    def __call__(self, snapshot: dict[str, Any]) -> None:
        now = time.time()
        telemetry = snapshot.get("telemetry") or {}
        ut = telemetry.get("ut")
        try:
            sim_ut = float(ut)
        except (TypeError, ValueError):
            sim_ut = None
        if self.last_sim_ut is None and sim_ut is not None:
            self.last_sim_ut = sim_ut
        phase = str(snapshot.get("phase") or "")

        should_record = True
        if (self.last_recorded_sim_ut is not None and sim_ut is not None and
                abs(sim_ut - self.last_recorded_sim_ut) < 0.5 and
                phase == self.last_phase):
            should_record = False

        if should_record:
            lean = _lean_snapshot(snapshot)
            wrapped = {
                "wallTime": now,
                "runId": self.manifest["runId"],
                "snapshotIndex": self.count,
                "snapshot": lean,
            }
            self.handle.write(json.dumps(wrapped, separators=(",", ":"), allow_nan=False) + "\n")
            self.count += 1
            self.last_recorded_sim_ut = sim_ut
            self.last_phase = phase

        mirror = dict(snapshot)
        mirror["generatedAt"] = now
        mirror["simulation"] = {
            "active": True,
            "mode": "closed-loop",
            "runId": self.manifest["runId"],
            "scenario": self.manifest["scenario"],
            "rateMode": "max",
            "lockstep": True,
            "replayAvailable": True,
            "snapshotIndex": self.count - 1,
            "guidanceSnapshots": str(self.path),
            "simulatorTelemetry": self.manifest["simulatorTelemetry"],
        }
        mirror["server"] = {
            "generatedAt": now,
            "source": "ShuttleSim closed-loop",
            "simulation": True,
            "runId": self.manifest["runId"],
        }
        atomic_json(self.mirror, mirror)


def terminate(proc: subprocess.Popen[Any] | None, timeout: float = 2.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        try:
            proc.terminate()
        except ProcessLookupError:
            return
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            proc.kill()
        proc.wait(timeout=timeout)




def read_last_json_object(path: Path) -> dict[str, Any] | None:
    try:
        with path.open("rb") as handle:
            handle.seek(0, os.SEEK_END)
            end = handle.tell()
            if end <= 0:
                return None
            pos = end - 1
            while pos >= 0:
                handle.seek(pos)
                if handle.read(1) not in b"\r\n":
                    break
                pos -= 1
            if pos < 0:
                return None
            line_end = pos + 1
            while pos >= 0:
                handle.seek(pos)
                if handle.read(1) == b"\n":
                    pos += 1
                    break
                pos -= 1
            if pos < 0:
                pos = 0
            handle.seek(pos)
            raw = handle.read(line_end - pos)
        value = json.loads(raw.decode("utf-8"))
        return value if isinstance(value, dict) else None
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return None


def simulator_elapsed_seconds(path: Path) -> float | None:
    packet = read_last_json_object(path)
    if not packet:
        return None
    try:
        elapsed = float(packet.get("sim_time"))
    except (TypeError, ValueError):
        return None
    return elapsed if math.isfinite(elapsed) and elapsed >= 1.0 else None


def simulator_horizon_packet(path: Path, max_sim_time: float,
                             physics_dt: float = 0.02) -> dict[str, Any] | None:
    packet = read_last_json_object(path)
    if not packet:
        return None
    try:
        sim_time = float(packet.get("sim_time"))
    except (TypeError, ValueError):
        return None
    tolerance = max(1e-6, physics_dt * 1.5)
    return packet if sim_time + tolerance >= max_sim_time else None


def simulator_packet_final(packet: dict[str, Any]) -> dict[str, Any]:
    position = packet.get("position") or {}
    velocity = packet.get("velocity") or {}
    runway = packet.get("runway") or {}
    ground = packet.get("ground") or {}
    return {
        "ut": packet.get("ut"),
        "altitude": position.get("altitude_m"),
        "surfaceSpeed": velocity.get("surface_mps"),
        "verticalSpeed": velocity.get("vertical_mps"),
        "runwayAlongTrack": runway.get("along_m"),
        "runwayCrossTrack": runway.get("cross_m"),
        "situation": "landed" if ground.get("on_ground") else "flying",
    }
def build(root: Path) -> None:
    subprocess.run(["cmake", "-S", str(root / "ShuttleSim"), "-B", str(root / "ShuttleSim/build")],
                   check=True, stdout=subprocess.DEVNULL)
    subprocess.run(["cmake", "--build", str(root / "ShuttleSim/build"), "-j", "4"],
                   check=True, stdout=subprocess.DEVNULL)
    subprocess.run(["make", "-C", str(clanding_source_root(root)), "-j4"],
                   check=True, stdout=subprocess.DEVNULL)


def main() -> int:
    ap = argparse.ArgumentParser(description="Run Guidance closed-loop against ShuttleSim")
    ap.add_argument("--scenario", type=Path, default=ROOT / "ShuttleSim/scenarios/ksp86km-preburn.ini")
    ap.add_argument("--config", type=Path, default=ROOT / "Configuration/default.json")
    ap.add_argument("--backend", type=Path, default=clanding_build_artifact(ROOT))
    ap.add_argument("--simulator", type=Path, default=ROOT / "ShuttleSim/build/shuttlesim")
    ap.add_argument("--run-id", default=None)
    ap.add_argument("--max-sim-time", type=float, default=2600.0)
    ap.add_argument("--wall-timeout", type=float, default=120.0)
    ap.add_argument("--publish-hz", type=float, default=30.0)
    ap.add_argument("--preroll-seconds", type=float, default=None,
                    help=("Seconds to advance ShuttleSim before Guidance connects. "
                          "Defaults to 30 s for reentry/full runs and 0 s for "
                          "HAC/MM305/final-test fixtures."))
    ap.add_argument("--no-build", action="store_true")
    ap.add_argument("--engage", choices=("reentry", "full", "hac", "hac-upstream", "final-test"), default="reentry")
    args = ap.parse_args()

    if args.preroll_seconds is None:
        preroll_seconds = (0.0 if args.engage in ("hac", "hac-upstream", "final-test")
                           else 30.0)
    else:
        preroll_seconds = args.preroll_seconds
    if not math.isfinite(preroll_seconds) or preroll_seconds < 0.0:
        ap.error("--preroll-seconds must be a finite non-negative value")

    if not args.no_build:
        build(ROOT)

    config = load_configuration(args.config, auto_warp=False)
    guidance = config.setdefault("guidance", {})
    guidance_rate = max(2.0, min(30.0, float(guidance.get("guidanceRate", 10.0))))
    base_run_id = args.run_id or time.strftime("sim-%Y%m%dT%H%M%SZ", time.gmtime())
    run_id = base_run_id
    run_dir = ROOT / "ShuttleSim/runs" / run_id
    suffix = 2
    while run_dir.exists():
        run_id = f"{base_run_id}-{suffix}"
        run_dir = ROOT / "ShuttleSim/runs" / run_id
        suffix += 1
    run_dir.mkdir(parents=True, exist_ok=False)
    sim_record = run_dir / "simulator-telemetry.jsonl"
    sim_stderr = (run_dir / "simulator.log").open("w", encoding="utf-8")
    backend_stderr = (run_dir / "guidance.log").open("w", encoding="utf-8")
    mirror = ROOT / "Runtime/WebTelemetry/controller-snapshot.json"
    run_meta = ROOT / "Runtime/WebTelemetry/simulator-run.json"

    manifest: dict[str, Any] = {
        "schema": 1,
        "runId": run_id,
        "state": "starting",
        "mode": "closed-loop",
        "scenario": str(args.scenario),
        "config": str(args.config),
        "startedAt": time.time(),
        "rateMode": "max",
        "lockstep": True,
        "physicsDt": 0.02,
        "guidanceRateHz": guidance_rate,
        "prerollSeconds": preroll_seconds,
        "simulatorTelemetry": str(sim_record),
        "guidanceSnapshots": str(run_dir / "guidance-snapshots.jsonl"),
        "runDirectory": str(run_dir),
    }
    atomic_json(run_meta, manifest)
    atomic_json(run_dir / "manifest.json", manifest)

    sim_cmd = [
        str(args.simulator),
        "--scenario", str(args.scenario),
        "--atmosphere", str(ROOT / "ShuttleSim/data/fitted/kerbin_atmosphere_ksp.csv"),
        "--aero", str(ROOT / "ShuttleSim/data/fitted/stsn_aero_ksp_robust.csv"),
        "--aero-book", str(ROOT / "ShuttleSim/data/fitted/stsn_force_book.csv"),
        "--attitude", str(ROOT / "ShuttleSim/data/fitted/stsn_attitude_ksp.ini"),
        "--dt", "0.02",
        "--rate", "max",
        "--telemetry-hz", f"{guidance_rate:g}",
        "--max-sim-time", f"{args.max_sim_time:g}",
        "--command-port", "8795",
        "--telemetry-port", "8796",
        "--web-telemetry-port", "8797",
        "--record", str(sim_record),
        "--lockstep",
        "--quiet",
    ]

    sim: subprocess.Popen[Any] | None = None
    backend: BackendProcess | None = None
    recorder: SnapshotRecorder | None = None
    started = time.monotonic()
    terminal: dict[str, Any] | None = None
    exit_code = 1

    old_env = os.environ.copy()
    preroll_socket: socket.socket | None = None
    try:
        # During orbital pre-roll no Guidance process is connected yet. Observe the
        # simulator's Web telemetry sink directly and release exactly one lockstep
        # frame at a time. This exercises the real simulator deorbit burn without
        # weakening Guidance's post-burn continuation gates.
        os.environ["SHUTTLESIM_LOCKSTEP_RESEND"] = "1"
        preroll_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        preroll_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        preroll_socket.bind(("127.0.0.1", 8796))
        preroll_socket.settimeout(10.0)

        sim = subprocess.Popen(
            sim_cmd,
            cwd=ROOT,
            stdout=subprocess.DEVNULL,
            stderr=sim_stderr,
            start_new_session=True,
        )
        time.sleep(0.05)
        if sim.poll() is not None:
            raise RuntimeError(f"ShuttleSim exited during startup with code {sim.returncode}")

        preroll_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            last_packet: dict[str, Any] | None = None
            while True:
                data, _ = preroll_socket.recvfrom(65535)
                packet = json.loads(data)
                if packet.get("source") != "sim":
                    continue
                last_packet = packet
                sim_time = float(packet.get("sim_time", 0.0))
                if sim_time + 1e-9 >= preroll_seconds:
                    break
                preroll_cmd.sendto(b'{"type":"step","step":true}', ("127.0.0.1", 8795))
            manifest["prerollFinal"] = {
                "simTime": last_packet.get("sim_time") if last_packet else None,
                "ut": last_packet.get("ut") if last_packet else None,
                "altitude": (last_packet.get("position") or {}).get("altitude_m") if last_packet else None,
                "verticalSpeed": (last_packet.get("velocity") or {}).get("vertical_mps") if last_packet else None,
                "airspeed": (last_packet.get("velocity") or {}).get("air_mps") if last_packet else None,
            }
        finally:
            preroll_cmd.close()
            preroll_socket.close()
            preroll_socket = None

        os.environ["KSP_LANDER_SIMULATOR"] = "1"
        os.environ["KSP_LANDER_UNPOWERED_ONLY"] = "1"
        # Simulator campaigns intentionally exercise Guidance even when the live
        # restart predictor does not certify this synthetic long-horizon state.
        # This environment variable is never set by live-KSP launchers.
        os.environ["KSP_LANDER_FORCE_REENTRY_TEST"] = "1"
        if args.engage == "final-test":
            os.environ["KSP_LANDER_FINAL_APPROACH_TEST"] = "1"
        else:
            os.environ.pop("KSP_LANDER_FINAL_APPROACH_TEST", None)
        # Only explicitly named MM305 fixtures may enter the fixed, dynamic
        # runway-anchored HAC path.  The generic simulator flag alone must not
        # alter unrelated ShuttleSim scenarios or native/offline tests.
        scenario_name = args.scenario.name
        mm305_fixed_fixture = scenario_name in {
            "mm304-hac-interface.ini",
            "mm304-hac-interface-20km.ini",
        } or scenario_name.startswith("mm305-hac-")
        if mm305_fixed_fixture:
            os.environ["KSP_LANDER_MM305_FIXED_HAC"] = "1"
        else:
            os.environ.pop("KSP_LANDER_MM305_FIXED_HAC", None)
        os.environ["KSP_LANDER_ROOT"] = str(ROOT)
        os.environ["KSP_LANDER_SIM_PUBLISH_HZ"] = f"{args.publish_hz:g}"
        if args.engage == "hac-upstream":
            os.environ["KSP_LANDER_HAC_VARIANT_B"] = "1"

        recorder = SnapshotRecorder(run_dir, mirror, manifest)
        # BackendProcess inherits stderr; redirecting it requires launching a small
        # wrapper only for logs, so keep stderr in the campaign console for now.
        backend = BackendProcess(args.backend, snapshot_sink=recorder, stderr_target=backend_stderr)

        backend.send("updateConfiguration", configuration=config, timeout=30.0)
        backend.send("connect", timeout=30.0)

        # HAC fixtures must engage at the exact initial lockstep barrier. The
        # resend path delivers that state after the backend binds its socket;
        # an unconditional wake raced engagement against the first physics step.
        if args.engage not in ("hac", "hac-upstream", "final-test"):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as wake:
                wake.sendto(b'{"type":"step","step":true}', ("127.0.0.1", 8795))

        connected = backend.latest_snapshot
        if not connected or connected.get("connectionStatus") != "connected":
            connected = backend.read_until(
                lambda s: s.get("connectionStatus") in {"connected", "failed"}, 30.0
            )
        if connected.get("connectionStatus") != "connected":
            raise RuntimeError(str(connected.get("lastError") or "simulator Guidance connect failed"))

        if args.engage in ("hac", "hac-upstream", "final-test"):
            expected_ut = float(manifest["prerollFinal"]["ut"])
            def fixture_ready(snapshot: dict[str, Any]) -> bool:
                tel = snapshot.get("telemetry") or {}
                return (isinstance(tel.get("ut"), (int, float)) and
                        abs(float(tel["ut"]) - expected_ut) <= 1e-6 and
                        float(tel.get("dynamicPressure") or 0.0) > 0.0)
            if not fixture_ready(connected):
                connected = backend.read_until(fixture_ready, 15.0)
            if not fixture_ready(connected):
                raise RuntimeError("HAC fixture did not reach its exact initialized lockstep state")
            manifest["initialTelemetry"] = connected["telemetry"]

        manifest["state"] = "connected"
        atomic_json(run_meta, manifest)
        atomic_json(run_dir / "manifest.json", manifest)

        if args.engage == "full":
            backend.send("createPlan", timeout=180.0)
            backend.send("engage", timeout=30.0)
        elif args.engage in ("hac", "hac-upstream"):
            backend.send("engageHACTest", timeout=30.0)
        elif args.engage == "final-test":
            previous_sequence = int((backend.latest_snapshot or {}).get("logSequence") or 0)
            backend.send("engageFinalTest", timeout=30.0)
            try:
                backend.read_until(
                    lambda snapshot: int(snapshot.get("logSequence") or 0) > previous_sequence,
                    3.0,
                )
            except TimeoutError:
                pass
        else:
            # Reentry-only qualification runs the production shadow executive
            # synchronously before the request is acknowledged. Keep the
            # campaign harness from misclassifying a slow, valid qualification
            # as a backend startup failure.
            backend.send("engageReentry", timeout=180.0)

        # Engagement requests can be rejected after a synchronous production
        # shadow qualification.  The backend remains connected and therefore
        # cannot satisfy the simulator's terminal-phase predicate by itself;
        # preserve the rejection as an explicit campaign outcome instead of
        # waiting for an unrelated wall timeout.
        armed = backend.latest_snapshot or {}
        if not armed.get("automationEngaged"):
            # The synchronous request has returned. An unengaged snapshot is
            # a rejection even when the warning was cleared by an idle frame;
            # it must not strand a lockstep fixture until the wall timeout.
            warning = str(armed.get("warningMessage") or armed.get("lastError") or
                          armed.get("statusMessage") or "Engagement was not accepted")
            sim_elapsed = simulator_elapsed_seconds(sim_record)
            manifest["state"] = "rejected" if sim_elapsed is not None else "invalid"
            manifest["finishedAt"] = time.time()
            manifest["wallSeconds"] = time.monotonic() - started
            manifest["simElapsedSeconds"] = sim_elapsed
            if sim_elapsed is None:
                manifest["invalidReason"] = "no-simulated-time"
            manifest["engagementRejected"] = True
            manifest["engagementWarning"] = warning
            manifest["engagementPhase"] = armed.get("phase")
            atomic_json(run_meta, manifest)
            atomic_json(run_dir / "manifest.json", manifest)
            print(json.dumps(manifest, indent=2, allow_nan=False))
            exit_code = 2
            return exit_code

        manifest["state"] = "running"
        manifest["engagedAt"] = time.time()
        atomic_json(run_meta, manifest)
        atomic_json(run_dir / "manifest.json", manifest)

        deadline = time.monotonic() + args.wall_timeout
        horizon_packet: dict[str, Any] | None = None
        while time.monotonic() < deadline:
            sim_code = sim.poll()
            if sim_code is not None:
                if sim_code != 0:
                    raise RuntimeError(f"ShuttleSim exited with code {sim_code}")
                horizon_packet = simulator_horizon_packet(sim_record, args.max_sim_time)
                if horizon_packet is not None:
                    terminal = backend.latest_snapshot or {}
                    break
                latest = backend.latest_snapshot or {}
                if str(latest.get("phase") or "") in TERMINAL_PHASES:
                    terminal = latest
                    break
                raise RuntimeError(
                    "ShuttleSim exited cleanly before a terminal phase or the requested simulation horizon"
                )
            remaining = max(0.05, min(0.25, deadline - time.monotonic()))
            try:
                snap = backend.read_until(
                    lambda s: str(s.get("phase") or "") in TERMINAL_PHASES,
                    remaining,
                )
                if str(snap.get("phase") or "") == "Fault":
                    horizon_packet = simulator_horizon_packet(sim_record, args.max_sim_time)
                    if horizon_packet is not None:
                        terminal = snap
                        break
                terminal = snap
                break
            except TimeoutError:
                latest = backend.latest_snapshot or {}
                if str(latest.get("phase") or "") in TERMINAL_PHASES:
                    horizon_packet = simulator_horizon_packet(sim_record, args.max_sim_time)
                    if str(latest.get("phase") or "") == "Fault" and horizon_packet is not None:
                        terminal = latest
                        break
                    terminal = latest
                    break
                continue

        if terminal is None:
            raise TimeoutError(f"closed-loop simulation exceeded {args.wall_timeout:g}s wall timeout")

        telemetry = terminal.get("telemetry") or {}
        phase = "Cutoff" if horizon_packet is not None else str(terminal.get("phase") or "")
        sim_elapsed = simulator_elapsed_seconds(sim_record)
        manifest["state"] = "finished" if sim_elapsed is not None else "invalid"
        manifest["finishedAt"] = time.time()
        manifest["wallSeconds"] = time.monotonic() - started
        manifest["simElapsedSeconds"] = sim_elapsed
        if sim_elapsed is None:
            manifest["invalidReason"] = "no-simulated-time"
        manifest["terminalPhase"] = phase
        guidance_state = terminal.get("guidanceState") or {}
        if isinstance(guidance_state, dict):
            manifest["variantBPathCommitted"] = bool(
                guidance_state.get("terminalPathCommitted")
            )
        if horizon_packet is not None:
            manifest["maxSimTimeReached"] = True
            manifest["final"] = simulator_packet_final(horizon_packet)
        else:
            manifest["final"] = {
                "ut": telemetry.get("ut"),
                "altitude": telemetry.get("meanAltitude"),
                "surfaceSpeed": telemetry.get("surfaceSpeed"),
                "verticalSpeed": telemetry.get("verticalSpeed"),
                "runwayAlongTrack": telemetry.get("runwayAlongTrack"),
                "runwayCrossTrack": telemetry.get("runwayCrossTrack"),
                "situation": telemetry.get("vesselSituation"),
            }
        atomic_json(run_meta, manifest)
        atomic_json(run_dir / "manifest.json", manifest)
        print(json.dumps(manifest, indent=2, allow_nan=False))
        if horizon_packet is not None:
            exit_code = 0 if args.engage == "hac" or (
                args.engage == "hac-upstream" and
                manifest.get("variantBPathCommitted") is True
            ) else 2
        else:
            exit_code = 0 if phase == "Complete" else 2
        if sim_elapsed is None:
            exit_code = 2

    except Exception as exc:
        sim_elapsed = simulator_elapsed_seconds(sim_record)
        manifest["state"] = "failed" if sim_elapsed is not None else "invalid"
        manifest["finishedAt"] = time.time()
        manifest["wallSeconds"] = time.monotonic() - started
        manifest["simElapsedSeconds"] = sim_elapsed
        if sim_elapsed is None:
            manifest["invalidReason"] = "no-simulated-time"
        manifest["error"] = str(exc)
        atomic_json(run_meta, manifest)
        atomic_json(run_dir / "manifest.json", manifest)
        import traceback
        traceback.print_exc()
        print(f"simulator campaign failed: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        if backend is not None:
            try:
                backend.send("shutdown", timeout=2.0)
            except Exception:
                pass
            terminate(backend.process)
            backend.close()
        terminate(sim)
        if preroll_socket is not None:
            preroll_socket.close()
        if recorder is not None:
            recorder.close()
        sim_stderr.close()
        backend_stderr.close()
        os.environ.clear()
        os.environ.update(old_env)
        for log_file in (run_dir / "simulator.log", run_dir / "guidance.log"):
            if log_file.is_file() and log_file.stat().st_size == 0:
                log_file.unlink(missing_ok=True)
        if manifest.get("invalidReason") == "no-simulated-time" or manifest.get("simElapsedSeconds") is None:
            try:
                current = json.loads(run_meta.read_text(encoding="utf-8")) if run_meta.is_file() else {}
                if current.get("runId") == run_id:
                    run_meta.unlink(missing_ok=True)
            except (OSError, json.JSONDecodeError):
                pass
            if args.engage != "final-test":
                shutil.rmtree(run_dir, ignore_errors=True)
        else:
            for artifact_name in ("guidance-snapshots.jsonl", "simulator-telemetry.jsonl"):
                p = run_dir / artifact_name
                if p.is_file():
                    gz_p = p.with_name(p.name + ".gz")
                    try:
                        with p.open("rb") as f_in, gzip.open(gz_p, "wb", compresslevel=6) as f_out:
                            shutil.copyfileobj(f_in, f_out)
                        if gz_p.is_file() and gz_p.stat().st_size > 0:
                            p.unlink(missing_ok=True)
                    except Exception:
                        pass
            if (run_dir / "manifest.json").is_file():
                try:
                    m = json.loads((run_dir / "manifest.json").read_text(encoding="utf-8"))
                    if (run_dir / "guidance-snapshots.jsonl.gz").is_file():
                        m["guidanceSnapshots"] = "guidance-snapshots.jsonl.gz"
                    if (run_dir / "simulator-telemetry.jsonl.gz").is_file():
                        m["simulatorTelemetry"] = "simulator-telemetry.jsonl.gz"
                    atomic_json(run_dir / "manifest.json", m)
                    if run_meta.is_file():
                        cur = json.loads(run_meta.read_text(encoding="utf-8"))
                        if cur.get("runId") == run_id:
                            if (run_dir / "guidance-snapshots.jsonl.gz").is_file():
                                cur["guidanceSnapshots"] = str(run_dir / "guidance-snapshots.jsonl.gz")
                            if (run_dir / "simulator-telemetry.jsonl.gz").is_file():
                                cur["simulatorTelemetry"] = str(run_dir / "simulator-telemetry.jsonl.gz")
                            atomic_json(run_meta, cur)
                except Exception:
                    pass
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
