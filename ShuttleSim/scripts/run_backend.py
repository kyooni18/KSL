"""Backend protocol streaming, latest-state handoff, and owned process shutdown."""
from __future__ import annotations

import json
import pathlib
import subprocess
import threading
import time

from backend_mailbox import BackendMailbox
from run_artifacts import write_json

MIRROR = pathlib.Path(__file__).resolve().parents[2] / "Runtime" / "WebTelemetry" / "controller-snapshot.json"

def _downsample_trajectory(points, limit=120):
    if not isinstance(points, list) or len(points) <= limit:
        return points if isinstance(points, list) else []
    scale = (len(points) - 1) / float(limit - 1)
    indices = sorted({min(len(points) - 1, int(round(i * scale))) for i in range(limit)})
    return [points[i] for i in indices]

def _lean_snapshot(snapshot):
    lean = dict(snapshot)
    lean["actualTrajectory"] = []
    for key in ("predictedTrajectory", "plannedTrajectory", "projectedTAEMTrajectory",
                "referenceTrajectory", "orbitalTrajectory"):
        if key in lean and isinstance(lean[key], list):
            lean[key] = _downsample_trajectory(lean[key], 120)
    plan = lean.get("plan")
    if isinstance(plan, dict):
        plan_copy = dict(plan)
        if "trajectory" in plan_copy:
            plan_copy["trajectory"] = _downsample_trajectory(plan_copy["trajectory"], 60)
        lean["plan"] = plan_copy
    return lean

class Backend:
    def __init__(self, proc: subprocess.Popen[str], log_path: pathlib.Path, mirror: bool = True,
                 replay_path: pathlib.Path | None = None, compact_log: bool = False,
                 lean_snapshots: bool = True):
        self.proc=proc; self.mailbox=BackendMailbox(); self.latest=None; self.mirror=mirror
        self.replay_lock=threading.Lock()
        self.compact_log=compact_log
        self.lean_snapshots=lean_snapshots
        self.log=log_path.open("w",encoding="utf-8")
        self.replay_log=replay_path.open("w",encoding="utf-8") if replay_path else None
        self.last_replay_ut=None
        self.thread=threading.Thread(target=self._read_guarded,daemon=True); self.thread.start()
    def _read_guarded(self):
        try:
            self._read()
        except Exception as error:
            self.mailbox.finish(error)
    def _read(self):
        assert self.proc.stdout
        last_recorded_ut=None
        last_phase=None
        last_status=None
        for line in self.proc.stdout:
            try: obj=json.loads(line)
            except json.JSONDecodeError:
                continue
            typ=obj.get("type")
            emit=True
            if typ=="snapshot" and isinstance(obj.get("snapshot"),dict):
                snap=obj["snapshot"]; self.latest=snap
                tel=snap.get("telemetry") if isinstance(snap.get("telemetry"),dict) else {}
                ut=tel.get("ut")
                phase=snap.get("phase")
                changed_ut=ut is not None and ut!=last_recorded_ut
                changed_phase=phase!=last_phase
                status=tuple(snap.get(key) for key in (
                    "phase","statusMessage","warningMessage","lastError","automationEngaged"))
                status_changed=status!=last_status
                emit=changed_ut or changed_phase or status_changed
                if emit:
                    last_recorded_ut=ut
                    last_phase=phase
                    last_status=status
                    with self.replay_lock:
                        if self.replay_log:
                            record_replay = (changed_phase or status_changed or self.last_replay_ut is None or ut is None
                                             or abs(ut-self.last_replay_ut)>=0.5)
                            if record_replay:
                                rec_snap = _lean_snapshot(snap) if self.lean_snapshots else snap
                                self.replay_log.write(json.dumps(rec_snap,separators=(",",":"))+"\n")
                                self.replay_log.flush()
                                self.last_replay_ut = ut
                    if self.mirror:
                        mirror=dict(snap); mirror["generatedAt"]=time.time()
                        mirror.setdefault("server",{})
                        mirror["server"]={**mirror["server"],"source":"ShuttleSim Guidance lockstep","mode":"simulator"}
                        write_json(MIRROR,mirror)
            if emit:
                log_obj=obj
                if self.compact_log and typ=="snapshot" and isinstance(obj.get("snapshot"),dict):
                    snap=obj["snapshot"]
                    tel=snap.get("telemetry") if isinstance(snap.get("telemetry"),dict) else {}
                    cmd=snap.get("command") if isinstance(snap.get("command"),dict) else {}
                    log_obj={"type":"snapshot","snapshot":{
                        "phase":snap.get("phase"),"statusMessage":snap.get("statusMessage"),
                        "warningMessage":snap.get("warningMessage"),
                        "telemetry":{k:tel.get(k) for k in (
                            "ut","meanAltitude","trueAirSpeed","verticalSpeed","flightPathAngle",
                            "runwayAlongTrack","runwayCrossTrack","rangeToSite","angleOfAttack","roll",
                            "heading","vesselSituation","dynamicPressure","mach")},
                        "command":{k:cmd.get(k) for k in (
                            "targetAoA","targetRoll","targetHeading","targetThrottle",
                            "gear","airbrakes","brakes","autopilotEngaged")},
                        "guidanceState":{k:(snap.get("guidanceState") or {}).get(k) for k in (
                            "hacRadius","terminalPathCommitted","hacCaptured","hacCompleted",
                            "hacTransitionActive","hacArcRemaining","hacCircuitCount",
                            "terminalPathSelected","finalCaptured")},
                    }}
                self.log.write(json.dumps(log_obj,separators=(",",":"))+"\n"); self.log.flush()
            self.mailbox.publish(obj)
        self.mailbox.finish()
    def send(self, method:str, ident:str, **extra):
        assert self.proc.stdin
        self.mailbox.expect(ident)
        msg={"id":ident,"method":method,**extra}
        try:
            self.proc.stdin.write(json.dumps(msg,separators=(",",":"))+"\n")
            self.proc.stdin.flush()
        except (OSError,ValueError):
            self.mailbox.cancel(ident)
            raise
    def wait(self,pred,timeout=20):
        message=self.mailbox.wait(pred,timeout)
        if message.get("type")=="response" and message.get("ok") is not True:
            raise RuntimeError(f"backend request failed: {message}")
        return message
    def freeze_replay(self):
        with self.replay_lock:
            if self.replay_log:
                self.replay_log.close()
                self.replay_log=None
    def close(self):
        # The backend reader writes every received record to self.log.  Join
        # it before closing the file so process shutdown cannot race a write.
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise RuntimeError("backend reader is still active; refusing to close its log")
        self.log.close()
        self.freeze_replay()

def stop_process(proc, timeout=3.0):
    if proc is None:
        return
    if proc.poll() is None:
        proc.terminate()
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=2)
