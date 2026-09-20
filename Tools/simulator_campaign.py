#!/usr/bin/env python3
"""Run Guidance closed-loop against standalone ShuttleSim at maximum speed.

Guidance remains the sole controller. This process only orchestrates the two
executables, mirrors exact Guidance snapshots for Telemetry Web, and records
replay artifacts.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "Tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from headless_flight import BackendProcess, load_configuration

TERMINAL_PHASES = {"Complete", "Abort", "Fault"}


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
        wrapped = {
            "wallTime": now,
            "runId": self.manifest["runId"],
            "snapshotIndex": self.count,
            "snapshot": snapshot,
        }
        self.handle.write(json.dumps(wrapped, separators=(",", ":"), allow_nan=False) + "\n")
        self.count += 1
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


def build(root: Path) -> None:
    subprocess.run(["cmake", "-S", str(root / "ShuttleSim"), "-B", str(root / "ShuttleSim/build")],
                   check=True, stdout=subprocess.DEVNULL)
    subprocess.run(["cmake", "--build", str(root / "ShuttleSim/build"), "-j", "4"],
                   check=True, stdout=subprocess.DEVNULL)
    subprocess.run(["make", "-C", str(root / "CLanding"), "-j4"],
                   check=True, stdout=subprocess.DEVNULL)


def main() -> int:
    ap = argparse.ArgumentParser(description="Run Guidance closed-loop against ShuttleSim")
    ap.add_argument("--scenario", type=Path, default=ROOT / "ShuttleSim/scenarios/ksp86km-preburn.ini")
    ap.add_argument("--config", type=Path, default=ROOT / "Configuration/default.json")
    ap.add_argument("--backend", type=Path, default=ROOT / "CLanding/build/landing_backend")
    ap.add_argument("--simulator", type=Path, default=ROOT / "ShuttleSim/build/shuttlesim")
    ap.add_argument("--run-id", default=None)
    ap.add_argument("--max-sim-time", type=float, default=2600.0)
    ap.add_argument("--wall-timeout", type=float, default=120.0)
    ap.add_argument("--publish-hz", type=float, default=30.0)
    ap.add_argument("--preroll-seconds", type=float, default=30.0,
                    help="Advance ShuttleSim through the prescribed deorbit burn before Guidance connects.")
    ap.add_argument("--no-build", action="store_true")
    ap.add_argument("--engage", choices=("reentry", "full"), default="reentry")
    args = ap.parse_args()

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
        "prerollSeconds": args.preroll_seconds,
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
        preroll_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        preroll_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        preroll_socket.bind(("127.0.0.1", 8796))
        preroll_socket.settimeout(2.0)

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
                if sim_time + 1e-9 >= args.preroll_seconds:
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
        os.environ["KSP_LANDER_ROOT"] = str(ROOT)
        os.environ["KSP_LANDER_SIM_PUBLISH_HZ"] = f"{args.publish_hz:g}"

        recorder = SnapshotRecorder(run_dir, mirror, manifest)
        # BackendProcess inherits stderr; redirecting it requires launching a small
        # wrapper only for logs, so keep stderr in the campaign console for now.
        backend = BackendProcess(args.backend, snapshot_sink=recorder)

        backend.send("updateConfiguration", configuration=config, timeout=30.0)
        backend.send("connect", timeout=30.0)

        # The simulator is waiting at the final pre-roll lockstep barrier. Release
        # one frame now that Guidance owns UDP 8796 so its control thread receives
        # the exact post-burn state rather than timing out on an empty socket.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as wake:
            wake.sendto(b'{"type":"step","step":true}', ("127.0.0.1", 8795))

        connected = backend.latest_snapshot
        if not connected or connected.get("connectionStatus") != "connected":
            connected = backend.read_until(
                lambda s: s.get("connectionStatus") in {"connected", "failed"}, 30.0
            )
        if connected.get("connectionStatus") != "connected":
            raise RuntimeError(str(connected.get("lastError") or "simulator Guidance connect failed"))

        manifest["state"] = "connected"
        atomic_json(run_meta, manifest)
        atomic_json(run_dir / "manifest.json", manifest)

        if args.engage == "full":
            backend.send("createPlan", timeout=180.0)
            backend.send("engage", timeout=30.0)
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
            warning = str(armed.get("warningMessage") or armed.get("lastError") or "")
            if warning:
                manifest["state"] = "rejected"
                manifest["finishedAt"] = time.time()
                manifest["wallSeconds"] = time.monotonic() - started
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
        while time.monotonic() < deadline:
            if sim.poll() is not None and sim.returncode not in (None, 0):
                raise RuntimeError(f"ShuttleSim exited with code {sim.returncode}")
            remaining = max(0.05, min(2.0, deadline - time.monotonic()))
            try:
                snap = backend.read_until(
                    lambda s: str(s.get("phase") or "") in TERMINAL_PHASES,
                    remaining,
                )
                terminal = snap
                break
            except TimeoutError:
                latest = backend.latest_snapshot or {}
                if str(latest.get("phase") or "") in TERMINAL_PHASES:
                    terminal = latest
                    break
                continue

        if terminal is None:
            raise TimeoutError(f"closed-loop simulation exceeded {args.wall_timeout:g}s wall timeout")

        telemetry = terminal.get("telemetry") or {}
        phase = str(terminal.get("phase") or "")
        manifest["state"] = "finished"
        manifest["finishedAt"] = time.time()
        manifest["wallSeconds"] = time.monotonic() - started
        manifest["terminalPhase"] = phase
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
        exit_code = 0 if phase == "Complete" else 2

    except Exception as exc:
        manifest["state"] = "failed"
        manifest["finishedAt"] = time.time()
        manifest["wallSeconds"] = time.monotonic() - started
        manifest["error"] = str(exc)
        atomic_json(run_meta, manifest)
        atomic_json(run_dir / "manifest.json", manifest)
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
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
