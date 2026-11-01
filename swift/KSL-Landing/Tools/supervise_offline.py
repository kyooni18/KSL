#!/usr/bin/env python3
"""Run one offline experiment with durable command, heartbeat and exit artifacts.

Usage: supervise_offline.py --output NEW_DIRECTORY --timeout SECONDS -- COMMAND...
A wall-time limit records an incomplete experiment; it is not a trajectory verdict.
Only the child process group created by this supervisor can receive its signals.
"""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w") as output:
        json.dump(value, output, indent=2)
        output.flush()
        os.fsync(output.fileno())
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout", type=float, required=True)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command or args.timeout <= 0:
        parser.error("a command and positive wall-time budget are required")
    args.output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    atomic_json(args.output / "command.json", {"argv": command, "cwd": str(Path.cwd()),
        "timeout_seconds": args.timeout, "started_utc": datetime.datetime.now(datetime.timezone.utc).isoformat()})
    requested_signal = None

    def interrupted(signum, frame):
        nonlocal requested_signal
        requested_signal = signum

    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupted)
    termination = None
    terminated_at = None
    process = None
    try:
        with (args.output / "stdout.log").open("wb") as out, (args.output / "stderr.log").open("wb") as err:
            process = subprocess.Popen(command, stdout=out, stderr=err, start_new_session=True)
            atomic_json(args.output / "process.json", {"supervisor_pid": os.getpid(), "child_pid": process.pid})
            while process.poll() is None:
                elapsed = time.monotonic() - started
                if termination is None and (requested_signal is not None or elapsed >= args.timeout):
                    termination = "interrupted" if requested_signal is not None else "wall-time-budget-exhausted"
                    terminated_at = time.monotonic()
                    try:
                        os.killpg(process.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                elif terminated_at is not None and time.monotonic() - terminated_at >= 5:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                atomic_json(args.output / "status.json", {"state": termination or "running",
                    "elapsed_seconds": elapsed, "child_pid": process.pid,
                    "heartbeat_utc": datetime.datetime.now(datetime.timezone.utc).isoformat()})
                time.sleep(min(1, max(0.01, args.timeout - elapsed)) if termination is None else 0.1)
            code = process.wait()
        result = {"state": termination or ("completed" if code == 0 else "process-failed"),
            "return_code": code, "signal": -code if code < 0 else None,
            "requested_signal": requested_signal, "elapsed_seconds": time.monotonic() - started,
            "trajectory_verdict": "not-inferred-from-process-exit"}
        atomic_json(args.output / "exit.json", result)
        atomic_json(args.output / "status.json", result)
        print(json.dumps(result))
        return 124 if termination == "wall-time-budget-exhausted" else (0 if code == 0 else 1)
    except BaseException as error:
        # Artifact/IO failures must not orphan the experiment we own.
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
        failure = {"state": "supervisor-failed", "error": str(error),
            "elapsed_seconds": time.monotonic() - started,
            "child_return_code": None if process is None else process.returncode,
            "trajectory_verdict": "not-inferred-from-process-exit"}
        try:
            atomic_json(args.output / "exit.json", failure)
            atomic_json(args.output / "status.json", failure)
        except Exception as artifact_error:
            print(json.dumps({**failure, "artifact_error": str(artifact_error)}), file=sys.stderr)
        raise


if __name__ == "__main__":
    sys.exit(main())
