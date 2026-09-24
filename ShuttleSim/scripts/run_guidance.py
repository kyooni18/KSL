#!/usr/bin/env python3
from __future__ import annotations
import argparse, gzip, io, json, math, os, pathlib, queue, re, shutil, subprocess, sys, threading, time

ROOT = pathlib.Path(__file__).resolve().parents[2]
SIM = ROOT / "ShuttleSim"
_clanding_raw = pathlib.Path(os.environ.get("KSP_CLANDING_ROOT", "CLanding")).expanduser()
CLANDING = _clanding_raw if _clanding_raw.is_absolute() else (ROOT / _clanding_raw)
CLANDING = CLANDING.resolve()
MIRROR = ROOT / "Runtime" / "WebTelemetry" / "controller-snapshot.json"
WEB_RUN = ROOT / "Runtime" / "WebTelemetry" / "simulator-run.json"

def publish_web_run(doc):
    """Mirror run status for Tools/telemetry_web.py (same schema as simulator_campaign.py)."""
    try:
        WEB_RUN.parent.mkdir(parents=True, exist_ok=True)
        tmp = WEB_RUN.with_suffix(".tmp")
        tmp.write_text(json.dumps(dict(doc, schema=1), indent=2, sort_keys=True) + "\n", encoding="utf-8")
        os.replace(tmp, WEB_RUN)
    except OSError:
        pass


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

def fixed_hac_geometry_evidence(text: str) -> dict:
    """Validate committed analytic geometry; this is not a flight-tracking test."""
    frozen = None
    count = 0
    for line in text.splitlines():
        if not line.startswith("MM305 fixed-HAC geometry:"):
            continue
        try:
            points = {key: (float(e), float(n)) for key, e, n in
                      re.findall(r"(runwayStation|center|upper|exit|entry)=\(([-+0-9.eE]+),([-+0-9.eE]+)\)", line)}
            lengths = re.search(r"lead=([-+0-9.eE]+) arc=([-+0-9.eE]+)\.$", line)
            if len(points) != 5 or lengths is None:
                return {"valid": False, "reason": "incomplete-geometry-record"}
            lead, arc = map(float, lengths.groups())
        except (ValueError, OverflowError):
            return {"valid": False, "reason": "malformed-geometry-number"}
        radius = math.dist(points["center"], points["exit"])
        values = tuple(x for key in ("runwayStation", "center", "upper", "exit", "entry") for x in points[key]) + (lead, arc)
        if not all(math.isfinite(x) for x in values) or radius < 3000.0 - .002 or lead < 0:
            return {"valid": False, "reason": "nonfinite-or-out-of-domain"}
        # Log coordinates are rounded to millimetres. These tolerances cover
        # their serialization error, not a relaxed planner geometry contract.
        if (math.dist(points["exit"], points["runwayStation"]) > .01 or
            abs(math.dist(points["entry"], points["center"])-radius) > .01 or
            math.dist(points["upper"], (points["center"][0], points["center"][1]+radius)) > .01 or
            abs(arc-1.5*math.pi*radius) > .02):
            return {"valid": False, "reason": "not-anchored-analytic-270"}
        if frozen is not None and any(abs(a-b) > .02 for a, b in zip(frozen, values)):
            return {"valid": False, "reason": "committed-geometry-changed"}
        frozen = values
        count += 1
    if frozen is None:
        return {"valid": False, "reason": "no-committed-geometry-record"}
    return {"valid": True, "radius": radius, "records": count,
            "center": points["center"], "entry": points["entry"],
            "exit": points["exit"], "leadLength": lead, "arcLength": arc}

class Backend:
    def __init__(self, proc: subprocess.Popen[str], log_path: pathlib.Path, mirror: bool = True,
                 replay_path: pathlib.Path | None = None, compact_log: bool = False,
                 lean_snapshots: bool = True):
        self.proc=proc; self.q: queue.Queue[dict]=queue.Queue(); self.latest=None; self.mirror=mirror
        self.compact_log=compact_log
        self.lean_snapshots=lean_snapshots
        self.log=log_path.open("w",encoding="utf-8")
        self.replay_log=replay_path.open("w",encoding="utf-8") if replay_path else None
        self.last_replay_ut=None
        self.thread=threading.Thread(target=self._read,daemon=True); self.thread.start()
    def _read(self):
        assert self.proc.stdout
        last_recorded_ut=None
        last_phase=None
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
                emit=changed_ut or changed_phase
                if emit:
                    last_recorded_ut=ut
                    last_phase=phase
                    if self.replay_log:
                        record_replay = True
                        if (not changed_phase and self.last_replay_ut is not None and ut is not None
                                and abs(ut - self.last_replay_ut) < 0.5):
                            record_replay = False
                        if record_replay:
                            rec_snap = _lean_snapshot(snap) if self.lean_snapshots else snap
                            self.replay_log.write(json.dumps(rec_snap,separators=(",",":"))+"\n")
                            self.replay_log.flush()
                            self.last_replay_ut = ut
                    if self.mirror:
                        mirror=dict(snap); mirror["generatedAt"]=time.time()
                        mirror.setdefault("server",{})
                        mirror["server"]={**mirror["server"],"source":"ShuttleSim Guidance lockstep","mode":"simulator"}
                        MIRROR.parent.mkdir(parents=True,exist_ok=True)
                        tmp=MIRROR.with_suffix(".tmp")
                        tmp.write_text(json.dumps(mirror,separators=(",",":")),encoding="utf-8")
                        tmp.replace(MIRROR)
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
                self.q.put(obj)
            elif typ in ("ready","response"):
                self.q.put(obj)
    def send(self, method:str, ident:str, **extra):
        assert self.proc.stdin
        msg={"id":ident,"method":method,**extra}
        self.proc.stdin.write(json.dumps(msg,separators=(",",":"))+"\n"); self.proc.stdin.flush()
    def wait(self,pred,timeout=20):
        deadline=time.time()+timeout
        while time.time()<deadline:
            try:o=self.q.get(timeout=.1)
            except queue.Empty:continue
            if pred(o):return o
        raise TimeoutError("backend response timeout")
    def freeze_replay(self):
        try:
            if self.replay_log:
                self.replay_log.flush()
                self.replay_log.close()
                self.replay_log=None
        except Exception:
            self.replay_log=None
    def close(self):
        # The backend reader writes every received record to self.log.  Join
        # it before closing the file so process shutdown cannot race a write.
        if self.thread.is_alive():
            self.thread.join(timeout=2)
        try:self.log.close()
        except Exception:pass
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


# Tracks the in-flight run so an exception or signal can still finalize it
# (manifest state, simulator-run.json, archive into runs/) instead of leaving
# the telemetry server showing a phantom "running" run forever.
_ACTIVE={}


def _recorded_sim_time(sim_log):
    best=0.0
    try:
        with open(sim_log,encoding="utf-8") as f:
            for line in f:
                try:t=json.loads(line).get("sim_time")
                except ValueError:continue
                if isinstance(t,(int,float)) and math.isfinite(t):best=max(best,float(t))
    except OSError:
        pass
    return best


def _finalize_interrupted(reason):
    run=_ACTIVE
    if not run or run.get("done"):
        return
    run["done"]=True
    for proc in (run.get("sim_proc"),run.get("be_proc")):
        try:stop_process(proc)
        except Exception:pass
    manifest=run.get("manifest"); manifest_path=run.get("manifest_path"); run_dir=run.get("run_dir")
    if manifest is None or manifest_path is None or run_dir is None:
        return
    _close_out_run(manifest,manifest_path,run_dir,run.get("final_run_dir"),reason)


def _pid_alive(pid):
    try:os.kill(int(pid),0)
    except (OSError,TypeError,ValueError):return False
    return True


def recover_orphaned_runs(pending_root,runs_root):
    """Close out staged runs whose runner died (e.g. SIGKILL) so they reach history."""
    if not pending_root.is_dir():
        return
    for run_dir in sorted(pending_root.iterdir()):
        manifest_path=run_dir/"manifest.json"
        try:manifest=json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError,ValueError):continue
        if manifest.get("state")!="running" or _pid_alive(manifest.get("runnerPid")):
            continue
        # Orphaned simulator children keep the lockstep ports busy; stop them.
        subprocess.run(["pkill","-f","--",f"--record {run_dir}/"],check=False)
        _close_out_run(manifest,manifest_path,run_dir,runs_root/run_dir.name,"runner process died")
    current=None
    try:current=json.loads(WEB_RUN.read_text(encoding="utf-8"))
    except (OSError,ValueError):pass
    if current and current.get("state")=="running" and not _pid_alive(current.get("runnerPid")):
        final=runs_root/str(current.get("runId"))
        publish_web_run(dict(current,state="failed",runDirectory=str(final) if final.is_dir() else current.get("runDirectory")))


def _close_out_run(manifest,manifest_path,run_dir,final_dir,reason):
    finished_at=time.time()
    elapsed=_recorded_sim_time(run_dir/"simulator-telemetry.jsonl")
    if manifest.get("finishedAt") is not None:
        finished_at=manifest["finishedAt"]
    manifest.update({
        "state":"failed","finishedAt":finished_at,
        "wallSeconds":finished_at-(manifest.get("startedAt") or finished_at),
        "simElapsedSeconds":elapsed if elapsed>=1.0 else None,
        "success":False,
        "final":dict(manifest.get("final") or {},abortReason=f"runner interrupted: {reason}"),
    })
    try:
        manifest_path.write_text(json.dumps(manifest,indent=2,sort_keys=True)+"\n",encoding="utf-8")
        if final_dir is not None and not final_dir.exists():
            run_dir.replace(final_dir); run_dir=final_dir
    except OSError:
        pass
    publish_web_run(dict(manifest,runDirectory=str(run_dir) if run_dir.is_dir() else None))


def _mm305_run_label(label: str, scenario: str, engage: str) -> tuple[str, str | None]:
    """Give direct MM305 replays a stable prefix and catalog mission tag."""
    scenario_name = pathlib.Path(scenario).stem.lower()
    is_mm305 = (engage.lower() == "engagehactest" or
                (scenario_name.startswith("mm305-") and engage.lower() != "engagefinaltest"))
    if not is_mm305:
        return label, None
    normalized = label if label.lower() == "mm305" or label.lower().startswith("mm305-") else f"mm305-{label}"
    return normalized, "MM305"


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--scenario",default=str(SIM/"scenarios/ksp86km-postburn.ini"))
    ap.add_argument("--telemetry-hz",type=float,default=10.0)
    ap.add_argument("--max-sim-time",type=float,default=2400.0)
    ap.add_argument("--plan-timeout",type=float,default=600.0,
                    help="maximum wall-clock seconds to allow production deorbit createPlan")
    ap.add_argument("--label",default="guidance")
    ap.add_argument("--command-port",type=int,default=18895)
    ap.add_argument("--telemetry-port",type=int,default=18896)
    ap.add_argument("--web-telemetry-port",type=int,default=8797)
    ap.add_argument("--web",action="store_true",help="deprecated no-op; the persistent Telemetry Web service is used")
    ap.add_argument("--configuration",default=str(ROOT/"Configuration/default.json"))
    ap.add_argument("--dt",type=float,default=0.02)
    ap.add_argument("--atmosphere",default=str(SIM/"data/fitted/kerbin_atmosphere_ksp.csv"))
    ap.add_argument("--aero",default=str(SIM/"data/fitted/stsn_aero_ksp_robust.csv"))
    ap.add_argument("--aero-book",default=str(SIM/"data/fitted/stsn_force_book.csv"),
                    help="direct-force data book, or 'none' to disable")
    ap.add_argument("--attitude",default=str(SIM/"data/fitted/stsn_attitude_ksp.ini"))
    ap.add_argument("--engage",default="engageReentry",
                    choices=["engage","engageReentry","engageHACTest","engageFinalTest"],
                    help="backend engage method; engage runs createPlan + production deorbit/entry/landing guidance")
    ap.add_argument("--skip-build",action="store_true")
    ap.add_argument("--no-mirror",action="store_true")
    ap.add_argument("--quiet-progress",action="store_true")
    ap.add_argument("--compact-guidance-log",action="store_true",
                    help="omit heavy trajectory arrays from scratch Guidance logs")
    ap.add_argument("--archive-replay",action=argparse.BooleanOptionalAction,default=True,
                    help="write a Telemetry Web replay manifest + exact Guidance snapshots (default on, so every advanced run appears in sim run history)")
    args=ap.parse_args()
    args.scenario=str(pathlib.Path(args.scenario).resolve())

    stamp=time.strftime("%Y%m%dT%H%M%SZ",time.gmtime())
    run_label, mission = _mm305_run_label(args.label, args.scenario, args.engage)
    run_id=f"{run_label}-{stamp}"
    runs_root=SIM/"runs"; runs_root.mkdir(parents=True,exist_ok=True)
    recover_orphaned_runs(SIM/".pending-runs",runs_root)
    final_run_dir=None
    if args.archive_replay:
        # Do not publish an attempt into the replay catalog until physics has
        # actually advanced. Planning/admission failures remain outside runs/.
        pending_root=SIM/".pending-runs"; pending_root.mkdir(parents=True,exist_ok=True)
        run_dir=pending_root/run_id
        final_run_dir=runs_root/run_id
        if run_dir.exists():
            shutil.rmtree(run_dir)
        run_dir.mkdir(parents=True,exist_ok=True)
        sim_log=run_dir/"simulator-telemetry.jsonl"
        guidance_log=run_dir/"guidance-events.jsonl"
        guidance_replay=run_dir/"guidance-snapshots.jsonl"
        backend_err=run_dir/"backend.stderr.log"
        sim_err=run_dir/"simulator.stderr.log"
        manifest_path=run_dir/"manifest.json"
    else:
        # Stage flat logs outside runs/ too; they are published only once
        # physics has advanced (sim_time > 0).
        pending_root=SIM/".pending-runs"; pending_root.mkdir(parents=True,exist_ok=True)
        run_dir=pending_root/run_id
        if run_dir.exists():
            shutil.rmtree(run_dir)
        run_dir.mkdir(parents=True,exist_ok=True)
        sim_log=run_dir/f"{run_id}-sim.jsonl"
        guidance_log=run_dir/f"{run_id}-guidance.jsonl"
        guidance_replay=None
        backend_err=run_dir/f"{run_id}-backend.stderr.log"
        sim_err=run_dir/f"{run_id}-sim.stderr.log"
        manifest_path=None

    started_at=time.time()
    if manifest_path:
        manifest={
            "runId":run_id,"state":"running","mode":"closed-loop",
            **({"mission":mission,"tags":[mission]} if mission else {}),
            "scenario":str(pathlib.Path(args.scenario)),
            "config":str(pathlib.Path(args.configuration)),
            "startedAt":started_at,"finishedAt":None,"wallSeconds":None,"runnerPid":os.getpid(),
            "terminalPhase":None,"final":{},
            "guidanceSnapshots":"guidance-snapshots.jsonl",
            "simulatorTelemetry":"simulator-telemetry.jsonl",
            "guidanceEvents":"guidance-events.jsonl",
            "backendStderr":"backend.stderr.log",
            "simulatorStderr":"simulator.stderr.log",
            "rateMode":"max","physicsDt":args.dt,"telemetryHz":args.telemetry_hz,
        }
        manifest_path.write_text(json.dumps(manifest,indent=2,sort_keys=True)+"\n",encoding="utf-8"); publish_web_run(dict(manifest,runDirectory=str(run_dir)))
        _ACTIVE.update(manifest=manifest,manifest_path=manifest_path,run_dir=run_dir,final_run_dir=final_run_dir)

    if not args.skip_build:
        subprocess.run(["make","-C",str(CLANDING),"-j","4"],check=True,stdout=subprocess.DEVNULL)
        subprocess.run(["cmake","--build",str(SIM/"build"),"-j","4"],check=True,stdout=subprocess.DEVNULL)

    env=os.environ.copy()
    env.update({
        "KSP_LANDER_ROOT":str(ROOT),
        "KSP_LANDER_SIMULATOR":"1",
        "KSP_LANDER_UNPOWERED_ONLY":"1",
        "KSP_LANDER_HAC_DIAGNOSTICS":"1",
        "KSP_LANDER_SIM_COMMAND_PORT":str(args.command_port),
        "KSP_LANDER_SIM_TELEMETRY_PORT":str(args.telemetry_port),
    })
    # Checkpoint-only reentry runs intentionally bypass orbital planning and hold
    # the current attempt out of the learned physics store. A production full
    # mission must use the normal orbital/deorbit state machine instead.
    if args.engage != "engage":
        env["KSP_LANDER_FORCE_REENTRY_TEST"]="1"
    else:
        env.pop("KSP_LANDER_FORCE_REENTRY_TEST",None)
    be_err=backend_err.open("w",encoding="utf-8")
    be_proc=subprocess.Popen([str(CLANDING/"build/landing_backend")],cwd=ROOT,env=env,
        stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=be_err,text=True,bufsize=1)
    _ACTIVE["be_proc"]=be_proc
    backend=Backend(be_proc,guidance_log,mirror=not args.no_mirror,replay_path=guidance_replay,
                    compact_log=args.compact_guidance_log)
    backend.wait(lambda o:o.get("type")=="ready",10)

    config=json.loads(pathlib.Path(args.configuration).read_text())
    backend.send("connect","connect",configuration=config)
    backend.wait(lambda o:o.get("type")=="response" and o.get("id")=="connect",10)

    web_proc=None
    # Never spawn a separate Telemetry Web server: the persistent service
    # (Tools/telemetry_web_service.sh) receives simulator UDP on 8797 and
    # lists this run from ShuttleSim/runs.

    sim_stderr=sim_err.open("w",encoding="utf-8")
    sim_cmd=[
        str(SIM/"build/shuttlesim"),
        "--scenario",args.scenario,
        "--atmosphere",args.atmosphere,
        "--aero",args.aero,
        "--attitude",args.attitude,
        "--dt",str(args.dt),"--rate","max","--telemetry-hz",str(args.telemetry_hz),
        "--max-sim-time",str(args.max_sim_time),
        "--command-port",str(args.command_port),"--telemetry-port",str(args.telemetry_port),"--web-telemetry-port",str(args.web_telemetry_port),
        "--record",str(sim_log),"--lockstep","--quiet"
    ]
    if args.aero_book.lower()!="none":
        sim_cmd[7:7]=["--aero-book",args.aero_book]
    sim_proc=subprocess.Popen(sim_cmd,cwd=ROOT,stdout=subprocess.DEVNULL,stderr=sim_stderr,text=True)
    _ACTIVE["sim_proc"]=sim_proc

    # Wait for the first real simulator telemetry snapshot.
    first=backend.wait(lambda o:o.get("type")=="snapshot" and
        isinstance(o.get("snapshot",{}).get("telemetry"),dict) and
        (o["snapshot"]["telemetry"].get("ut") or 0)>0,15)
    t=first["snapshot"]["telemetry"]
    print(f"connected: alt={t.get('meanAltitude',0):.1f} speed={t.get('trueAirSpeed',0):.1f}",flush=True)
    engage_rejected=False
    if args.engage=="engage":
        # Exercise the production path from the orbital state: generate and
        # qualify a deorbit plan first, then engage that exact plan. Do not seed
        # a post-burn continuation or manufacture an external impulse.
        backend.send("createPlan","createPlan")
        try:
            backend.wait(lambda o:o.get("type")=="response" and o.get("id")=="createPlan",args.plan_timeout)
        except TimeoutError:
            # Planning happens while lockstep physics is parked on the initial frame.
            # Never leak an orphaned backend/simulator if a long production witness
            # exceeds the runner's wall-clock budget.
            backend.freeze_replay()
            stop_process(sim_proc)
            stop_process(be_proc)
            stop_process(web_proc)
            if backend.thread.is_alive():
                backend.thread.join(timeout=2)
            for f in (be_err,sim_stderr):
                try:f.close()
                except Exception:pass
            backend.close()
            if manifest_path:
                finished_at=time.time()
                manifest.update({
                    "state":"failed","finishedAt":finished_at,
                    "wallSeconds":finished_at-started_at,"terminalPhase":"Planning",
                    "final":{"abortReason":f"createPlan exceeded {args.plan_timeout:.0f}s wall-clock budget"},
                    "success":False,
                })
                manifest_path.write_text(json.dumps(manifest,indent=2,sort_keys=True)+"\n",encoding="utf-8"); publish_web_run(dict(manifest,runDirectory=str(run_dir)))
            raise
        planned=backend.latest if isinstance(backend.latest,dict) else {}
        if str(planned.get("phase") or "") in ("Fault","Abort"):
            raise RuntimeError(f"deorbit planning failed: {planned.get('statusMessage') or planned.get('lastError') or planned.get('warningMessage')}")
        plan=planned.get("plan") if isinstance(planned.get("plan"),dict) else {}
        engage_rejected=plan.get("executionQualified") is False
        if engage_rejected:
            print("plan rejected:",json.dumps({k:v for k,v in plan.items() if not isinstance(v,(list,dict))}),flush=True)
            # A preview-only plan cannot advance lockstep because production
            # engage correctly refuses it. End the run at the planning result
            # instead of leaving ShuttleSim parked forever on its first frame.
            sim_proc.terminate()
    if not engage_rejected:
        backend.send(args.engage,"engage")
        backend.wait(lambda o:o.get("type")=="response" and o.get("id")=="engage",30)
    if args.engage in ("engageFinalTest", "engageHACTest") and not engage_rejected:
        # Admission is published before the response; rejected lockstep runs
        # cannot advance and must be closed out from the latest snapshot.
        admitted=backend.latest if isinstance(backend.latest,dict) else {}
        engage_rejected=(str(admitted.get("phase") or "") in ("Idle","Fault") or
                         admitted.get("automationEngaged") is not True)
        if engage_rejected:
            print("engagement rejected:",admitted.get("statusMessage") or
                  admitted.get("warningMessage") or admitted.get("lastError") or
                  admitted.get("phase"),flush=True)
            sim_proc.terminate()

    last_print=0.0
    while sim_proc.poll() is None and be_proc.poll() is None:
        snap=backend.latest
        if snap:
            tel=snap.get("telemetry") or {}; now=time.monotonic()
            # An Abort releases control.  In lockstep the simulator then waits
            # forever for the next guidance command, so stop this run at the
            # terminal snapshot instead of leaving both processes resident.
            if snap.get("phase")=="Abort":
                sim_proc.terminate()
                break
            if not args.quiet_progress and now-last_print>.5:
                print("phase=%s alt=%.0f speed=%.1f range=%.0f along=%.0f cross=%.0f aoa=%.1f bank=%.1f" % (
                    snap.get("phase","?"),tel.get("meanAltitude") or 0,tel.get("trueAirSpeed") or 0,
                    tel.get("rangeToSite") or 0,tel.get("runwayAlongTrack") or 0,tel.get("runwayCrossTrack") or 0,
                    tel.get("angleOfAttack") or 0,tel.get("roll") or 0),flush=True)
                last_print=now
        time.sleep(.01)

    try: sim_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        sim_proc.terminate()
        try: sim_proc.wait(timeout=2)
        except subprocess.TimeoutExpired: sim_proc.kill()

    final_snap=dict(backend.latest) if isinstance(backend.latest,dict) else {}
    final_tel=final_snap.get("telemetry") if isinstance(final_snap.get("telemetry"),dict) else {}
    terminal_phase=str(final_snap.get("phase") or "")
    final={
        "ut":final_tel.get("ut"),
        "meanAltitude":final_tel.get("meanAltitude"),
        "trueAirSpeed":final_tel.get("trueAirSpeed"),
        "verticalSpeed":final_tel.get("verticalSpeed"),
        "latitude":final_tel.get("latitude"),
        "longitude":final_tel.get("longitude"),
        "runwayAlongTrack":final_tel.get("runwayAlongTrack"),
        "runwayCrossTrack":final_tel.get("runwayCrossTrack"),
        "situation":final_tel.get("vesselSituation"),
        "abortReason":final_snap.get("statusMessage") if terminal_phase=="Abort" else None,
    }

    # Preserve the actual terminal controller state in replay; the disconnect
    # handshake intentionally produces a synthetic Idle snapshot that must not
    # become the replay endpoint.
    backend.freeze_replay()
    backend.send("disconnect","disconnect")
    time.sleep(.1)
    be_proc.terminate()
    if web_proc: web_proc.terminate()
    for proc in (be_proc, web_proc):
        if proc is not None:
            try: proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2)
    if backend.thread.is_alive():
        backend.thread.join(timeout=2)
    for f in (be_err,sim_stderr): f.close()
    backend.close()

    simulator_summary={}
    for line in sim_err.read_text(encoding="utf-8").splitlines():
        if line.startswith("SHUTTLESIM_SUMMARY "):
            simulator_summary=json.loads(line.partition(" ")[2])
    first_touchdown=None
    final_sim=None
    max_recorded_sim_time=0.0
    rollout_seen=False
    rollout_runway_valid=True
    rollout_brakes_seen=False
    rollout_airbrakes_seen=False
    rollout_max_lateral_accel=0.0
    rollout_max_abs_cross=0.0
    rollout_max_heading_error=0.0
    with (sim_log.open(encoding="utf-8") if sim_log.is_file() else io.StringIO()) as records:
        for line in records:
            sample=json.loads(line)
            sample_time=sample.get("sim_time")
            if isinstance(sample_time,(int,float)) and math.isfinite(sample_time):
                max_recorded_sim_time=max(max_recorded_sim_time,float(sample_time))
            ground=sample.get("ground") or {}
            if ground.get("touchdown_seen") and first_touchdown is None:
                first_touchdown=sample
            if ground.get("on_ground"):
                runway=sample.get("runway") or {}
                velocity=sample.get("velocity") or {}
                attitude=sample.get("attitude") or {}
                along=runway.get("along_m")
                cross=runway.get("cross_m")
                if not isinstance(along,(int,float)) or not isinstance(cross,(int,float)) or not (0.0<=along<=2500.0 and abs(cross)<=35.0):
                    rollout_runway_valid=False
                if isinstance(cross,(int,float)):
                    rollout_max_abs_cross=max(rollout_max_abs_cross,abs(cross))
                lat_accel=ground.get("steering_lateral_accel_mps2")
                if isinstance(lat_accel,(int,float)) and math.isfinite(lat_accel):
                    rollout_max_lateral_accel=max(rollout_max_lateral_accel,abs(lat_accel))
                rollout_brakes_seen=rollout_brakes_seen or ground.get("brakes") is True
                rollout_airbrakes_seen=rollout_airbrakes_seen or ground.get("airbrakes") is True
                speed=velocity.get("surface_mps",float("inf"))
                if isinstance(speed,(int,float)) and speed>5.0:
                    heading=attitude.get("heading_deg")
                    runway_heading=runway.get("heading_deg")
                    if isinstance(heading,(int,float)) and isinstance(runway_heading,(int,float)):
                        err=abs((heading-runway_heading+180.0)%360.0-180.0)
                        rollout_max_heading_error=max(rollout_max_heading_error,err)
                if isinstance(speed,(int,float)) and speed<0.5:
                    rollout_seen=True
            final_sim=sample
    touchdown_ground=(first_touchdown or {}).get("ground") or {}
    touchdown_runway=(first_touchdown or {}).get("runway") or {}
    touchdown_gear=touchdown_ground.get("gear_down") is True
    runway_contact=(simulator_summary.get("touchdown") is True and
                    simulator_summary.get("on_runway") is True and
                    touchdown_ground.get("on_runway_touchdown") is True and
                    0 <= simulator_summary.get("touchdown_along_m",float("-inf")) <= 2500 and
                    abs(simulator_summary.get("touchdown_cross_m",float("inf"))) <= 35)
    controls_clean=True
    observed_phases=set()
    fixed_hac_geometry=fixed_hac_geometry_evidence(backend_err.read_text(encoding="utf-8"))
    fixed_hac_seen=False
    spline_seen=False
    with guidance_log.open(encoding="utf-8") as records:
        for line in records:
            event=json.loads(line)
            if event.get("type")!="snapshot":
                continue
            snap=event.get("snapshot") or {}
            observed_phases.add(str(snap.get("phase") or ""))
            state=snap.get("guidanceState") or {}
            status=str(snap.get("statusMessage") or "")
            # Dynamic radius is permitted only when the committed log proves
            # one immutable runway-anchored analytic 270-degree circle.
            radius=state.get("hacRadius")
            fixed_hac_seen=fixed_hac_seen or (fixed_hac_geometry.get("valid") is True and
                state.get("terminalPathCommitted") is True and "fixed-HAC" in status and
                isinstance(radius,(int,float)) and math.isfinite(radius) and
                abs(radius-fixed_hac_geometry["radius"]) <= .01)
            spline_seen=spline_seen or "spline" in status.lower()
            command=snap.get("command") or {}
            rollout_airbrake_ok=(command.get("airbrakes") is False or
                (args.engage=="engageFinalTest" and str(snap.get("phase") or "") in ("Touchdown","Rollout","Complete")))
            controls_clean=controls_clean and command.get("targetThrottle")==0 and rollout_airbrake_ok
    final_ground=(final_sim or {}).get("ground") or {}
    rollout_valid=rollout_seen and final_ground.get("on_ground") is True
    touchdown_speed=simulator_summary.get("touchdown_speed_mps")
    strict_touchdown=args.engage in ("engageFinalTest","engageHACTest")
    touchdown_speed_ok=(not strict_touchdown or
        (isinstance(touchdown_speed,(int,float)) and 60.0<=touchdown_speed<=70.0))
    touchdown_sink=simulator_summary.get("touchdown_sink_mps")
    touchdown_sink_ok=(not strict_touchdown or
        (isinstance(touchdown_sink,(int,float)) and 0.0<=touchdown_sink<=3.0))
    rollout_controls_ok=(args.engage!="engageFinalTest" or
        (rollout_brakes_seen and rollout_airbrakes_seen))
    rollout_heading_ok=(args.engage!="engageFinalTest" or rollout_max_heading_error<=8.0)
    rollout_stability_proxy_ok=(args.engage!="engageFinalTest" or rollout_max_lateral_accel<=1.25)
    hac_requirement_ok=(args.engage=="engageFinalTest" or fixed_hac_seen)
    success=bool(not engage_rejected and runway_contact and touchdown_gear and rollout_valid and
                 rollout_runway_valid and touchdown_speed_ok and touchdown_sink_ok and
                 rollout_controls_ok and rollout_heading_ok and rollout_stability_proxy_ok and
                 controls_clean and sim_proc.returncode==0 and hac_requirement_ok and
                 not spline_seen and any("Final" in phase for phase in observed_phases) and
                 "Flare" in observed_phases)

    final["simulatorSummary"]=simulator_summary
    final["touchdownGearDown"]=touchdown_gear
    final["rolloutValid"]=rollout_valid
    final["rolloutRunwayValid"]=rollout_runway_valid
    final["rolloutBrakesSeen"]=rollout_brakes_seen
    final["rolloutAirbrakesSeen"]=rollout_airbrakes_seen
    final["rolloutMaxAbsCrossTrack"]=rollout_max_abs_cross
    final["rolloutMaxHeadingErrorDeg"]=rollout_max_heading_error
    final["rolloutMaxLateralAccelMps2"]=rollout_max_lateral_accel
    final["rolloutStabilityProxyOk"]=rollout_stability_proxy_ok
    final["controlsClean"]=controls_clean
    final["touchdownSpeedOk"]=touchdown_speed_ok
    final["touchdownSinkOk"]=touchdown_sink_ok
    final["engageRejected"]=engage_rejected
    final["fixedHacSeen"]=fixed_hac_seen
    final["fixedHacGeometry"]=fixed_hac_geometry
    final["splineSeen"]=spline_seen
    final["observedPhases"]=sorted(observed_phases)
    final["touchdownRunwayTelemetry"]=touchdown_runway

    sim_elapsed_seconds=max_recorded_sim_time
    if manifest_path:
        finished_at=time.time()
        manifest.update({
            "state":"complete" if success else ("rejected" if engage_rejected else ("aborted" if terminal_phase.lower()=="abort" else "finished")),
            "finishedAt":finished_at,
            "wallSeconds":finished_at-started_at,
            "simElapsedSeconds":sim_elapsed_seconds,
            "terminalPhase":terminal_phase or ("Rejected" if engage_rejected else None),
            "final":final,
            "success":success,
        })
        manifest_path.write_text(json.dumps(manifest,indent=2,sort_keys=True)+"\n",encoding="utf-8")
    _ACTIVE["done"]=True

    discarded_zero_second_run=False
    _dbg=os.environ.get("KSP_LANDER_KEEP_BACKEND_STDERR")
    if _dbg and backend_err.exists(): shutil.copyfile(backend_err,_dbg)
    if args.archive_replay:
        assert final_run_dir is not None
        if final_run_dir.exists():
            raise RuntimeError(f"refusing to overwrite existing run archive: {final_run_dir}")
        run_dir.replace(final_run_dir)
        run_dir=final_run_dir
        sim_log=run_dir/"simulator-telemetry.jsonl"
        guidance_log=run_dir/"guidance-events.jsonl"
        guidance_replay=run_dir/"guidance-snapshots.jsonl"
        backend_err=run_dir/"backend.stderr.log"
        sim_err=run_dir/"simulator.stderr.log"
        manifest_path=run_dir/"manifest.json"
        if manifest_path:
            publish_web_run(dict(manifest,runDirectory=str(run_dir)))
    else:
        staged=run_dir
        moved=[]
        for p in (sim_log, guidance_log, backend_err, sim_err):
            if p and p.is_file():
                dest=runs_root/p.name
                p.replace(dest)
                moved.append(dest)
            else:
                moved.append(runs_root/p.name if p else None)
        sim_log, guidance_log, backend_err, sim_err = moved
        run_dir=runs_root
        shutil.rmtree(staged,ignore_errors=True)

    if sim_err and sim_err.is_file() and sim_err.stat().st_size == 0:
        sim_err.unlink(missing_ok=True)
    for p in (sim_log, guidance_log, guidance_replay):
        if p and p.is_file():
            gz_p = p.with_name(p.name + ".gz")
            try:
                with p.open("rb") as f_in, gzip.open(gz_p, "wb", compresslevel=6) as f_out:
                    shutil.copyfileobj(f_in, f_out)
                if gz_p.is_file() and gz_p.stat().st_size > 0:
                    p.unlink(missing_ok=True)
            except Exception:
                pass
    if manifest_path and manifest_path.is_file():
        try:
            m = json.loads(manifest_path.read_text(encoding="utf-8"))
            if (run_dir / "guidance-snapshots.jsonl.gz").is_file():
                m["guidanceSnapshots"] = "guidance-snapshots.jsonl.gz"
            if (run_dir / "simulator-telemetry.jsonl.gz").is_file():
                m["simulatorTelemetry"] = "simulator-telemetry.jsonl.gz"
            manifest_path.write_text(json.dumps(m, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        except Exception:
            pass

    output={
        "success":success,"simulatorSummary":simulator_summary,
        "abortReason":final["abortReason"],
        "touchdownGearDown":touchdown_gear,"rolloutValid":rollout_valid,
        "rolloutRunwayValid":rollout_runway_valid,"rolloutBrakesSeen":rollout_brakes_seen,
        "rolloutAirbrakesSeen":rollout_airbrakes_seen,"rolloutMaxHeadingErrorDeg":rollout_max_heading_error,
        "rolloutMaxLateralAccelMps2":rollout_max_lateral_accel,"rolloutStabilityProxyOk":rollout_stability_proxy_ok,
        "touchdownSpeedOk":touchdown_speed_ok,"touchdownSinkOk":touchdown_sink_ok,
        "engageRejected":engage_rejected,"controlsClean":controls_clean,
        "fixedHacSeen":fixed_hac_seen,"splineSeen":spline_seen,
        "observedPhases":sorted(observed_phases),
        "sim":None if discarded_zero_second_run else str(sim_log),
        "guidance":None if discarded_zero_second_run else str(guidance_log),
        "backendStderr":None if discarded_zero_second_run else str(backend_err),
        "simStderr":None if discarded_zero_second_run else str(sim_err),
        "runId":None if discarded_zero_second_run else (run_id if args.archive_replay else None),
        "manifest":None if discarded_zero_second_run else (str(manifest_path) if manifest_path else None),
        "discardedZeroSecondRun":discarded_zero_second_run,
        "recordedSimTime":max_recorded_sim_time,
    }
    print(json.dumps(output))
    return 0 if success else 2

def _on_signal(signum,frame):
    raise KeyboardInterrupt(f"signal {signum}")


if __name__=="__main__":
    import signal
    for _sig in (signal.SIGTERM,signal.SIGHUP):
        signal.signal(_sig,_on_signal)
    try:
        raise SystemExit(main())
    except BaseException as exc:
        if not isinstance(exc,SystemExit):
            _finalize_interrupted(f"{type(exc).__name__}: {exc}"[:300])
        raise
