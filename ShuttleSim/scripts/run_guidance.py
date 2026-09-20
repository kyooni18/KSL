#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, os, pathlib, queue, subprocess, sys, threading, time

ROOT = pathlib.Path(__file__).resolve().parents[2]
SIM = ROOT / "ShuttleSim"
CLANDING = ROOT / "CLanding"
MIRROR = ROOT / "Runtime" / "WebTelemetry" / "controller-snapshot.json"

class Backend:
    def __init__(self, proc: subprocess.Popen[str], log_path: pathlib.Path, mirror: bool = True,
                 replay_path: pathlib.Path | None = None, compact_log: bool = False):
        self.proc=proc; self.q: queue.Queue[dict]=queue.Queue(); self.latest=None; self.mirror=mirror
        self.compact_log=compact_log
        self.log=log_path.open("w",encoding="utf-8")
        self.replay_log=replay_path.open("w",encoding="utf-8") if replay_path else None
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
                        self.replay_log.write(json.dumps(snap,separators=(",",":"))+"\n")
                        self.replay_log.flush()
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
                            "vesselSituation","dynamicPressure","mach")},
                        "command":{k:cmd.get(k) for k in ("targetAoA","targetRoll","targetHeading","gear")},
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
        try:self.log.close()
        except Exception:pass
        self.freeze_replay()

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--scenario",default=str(SIM/"scenarios/ksp86km-postburn.ini"))
    ap.add_argument("--telemetry-hz",type=float,default=10.0)
    ap.add_argument("--max-sim-time",type=float,default=2400.0)
    ap.add_argument("--label",default="guidance")
    ap.add_argument("--command-port",type=int,default=18895)
    ap.add_argument("--telemetry-port",type=int,default=18896)
    ap.add_argument("--web-telemetry-port",type=int,default=18897)
    ap.add_argument("--web",action="store_true",help="start Telemetry Web in simulator mode")
    ap.add_argument("--configuration",default=str(ROOT/"Configuration/default.json"))
    ap.add_argument("--dt",type=float,default=0.02)
    ap.add_argument("--atmosphere",default=str(SIM/"data/fitted/kerbin_atmosphere_ksp.csv"))
    ap.add_argument("--aero",default=str(SIM/"data/fitted/stsn_aero_ksp_robust.csv"))
    ap.add_argument("--aero-book",default=str(SIM/"data/fitted/stsn_force_book.csv"),
                    help="direct-force data book, or 'none' to disable")
    ap.add_argument("--attitude",default=str(SIM/"data/fitted/stsn_attitude_ksp.ini"))
    ap.add_argument("--skip-build",action="store_true")
    ap.add_argument("--no-mirror",action="store_true")
    ap.add_argument("--quiet-progress",action="store_true")
    ap.add_argument("--compact-guidance-log",action="store_true",
                    help="omit heavy trajectory arrays from scratch Guidance logs")
    ap.add_argument("--archive-replay",action="store_true",
                    help="write a Telemetry Web replay manifest + exact Guidance snapshots")
    args=ap.parse_args()

    stamp=time.strftime("%Y%m%dT%H%M%SZ",time.gmtime())
    run_id=f"{args.label}-{stamp}"
    runs_root=SIM/"runs"; runs_root.mkdir(parents=True,exist_ok=True)
    if args.archive_replay:
        run_dir=runs_root/run_id
        run_dir.mkdir(parents=True,exist_ok=True)
        sim_log=run_dir/"simulator-telemetry.jsonl"
        guidance_log=run_dir/"guidance-events.jsonl"
        guidance_replay=run_dir/"guidance-snapshots.jsonl"
        backend_err=run_dir/"backend.stderr.log"
        sim_err=run_dir/"simulator.stderr.log"
        manifest_path=run_dir/"manifest.json"
    else:
        run_dir=runs_root
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
            "scenario":str(pathlib.Path(args.scenario)),
            "config":str(pathlib.Path(args.configuration)),
            "startedAt":started_at,"finishedAt":None,"wallSeconds":None,
            "terminalPhase":None,"final":{},
            "guidanceSnapshots":"guidance-snapshots.jsonl",
            "simulatorTelemetry":"simulator-telemetry.jsonl",
            "guidanceEvents":"guidance-events.jsonl",
            "backendStderr":"backend.stderr.log",
            "simulatorStderr":"simulator.stderr.log",
            "rateMode":"max","physicsDt":args.dt,"telemetryHz":args.telemetry_hz,
        }
        manifest_path.write_text(json.dumps(manifest,indent=2,sort_keys=True)+"\n",encoding="utf-8")

    if not args.skip_build:
        subprocess.run(["make","-C",str(CLANDING),"-j","4"],check=True,stdout=subprocess.DEVNULL)
        subprocess.run(["cmake","--build",str(SIM/"build"),"-j","4"],check=True,stdout=subprocess.DEVNULL)

    env=os.environ.copy()
    env.update({
        "KSP_LANDER_ROOT":str(ROOT),
        "KSP_LANDER_SIMULATOR":"1",
        "KSP_LANDER_FORCE_REENTRY_TEST":"1",
        "KSP_LANDER_UNPOWERED_ONLY":"1",
        "KSP_LANDER_SIM_COMMAND_PORT":str(args.command_port),
        "KSP_LANDER_SIM_TELEMETRY_PORT":str(args.telemetry_port),
    })
    be_err=backend_err.open("w",encoding="utf-8")
    be_proc=subprocess.Popen([str(CLANDING/"build/landing_backend")],cwd=ROOT,env=env,
        stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=be_err,text=True,bufsize=1)
    backend=Backend(be_proc,guidance_log,mirror=not args.no_mirror,replay_path=guidance_replay,
                    compact_log=args.compact_guidance_log)
    backend.wait(lambda o:o.get("type")=="ready",10)

    config=json.loads(pathlib.Path(args.configuration).read_text())
    backend.send("connect","connect",configuration=config)
    backend.wait(lambda o:o.get("type")=="response" and o.get("id")=="connect",10)

    web_proc=None
    if args.web:
        web_proc=subprocess.Popen([sys.executable,str(ROOT/"Tools/telemetry_web.py"),"--mode","simulator"],
            cwd=ROOT,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)

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

    # Wait for the first real simulator telemetry snapshot.
    first=backend.wait(lambda o:o.get("type")=="snapshot" and
        isinstance(o.get("snapshot",{}).get("telemetry"),dict) and
        (o["snapshot"]["telemetry"].get("ut") or 0)>0,15)
    t=first["snapshot"]["telemetry"]
    print(f"connected: alt={t.get('meanAltitude',0):.1f} speed={t.get('trueAirSpeed',0):.1f}",flush=True)

    backend.send("engageReentry","engage")
    backend.wait(lambda o:o.get("type")=="response" and o.get("id")=="engage",30)

    last_print=0.0
    success=False
    while sim_proc.poll() is None and be_proc.poll() is None:
        snap=backend.latest
        if snap:
            tel=snap.get("telemetry") or {}; now=time.monotonic()
            if not args.quiet_progress and now-last_print>.5:
                print("phase=%s alt=%.0f speed=%.1f range=%.0f along=%.0f cross=%.0f aoa=%.1f bank=%.1f" % (
                    snap.get("phase","?"),tel.get("meanAltitude") or 0,tel.get("trueAirSpeed") or 0,
                    tel.get("rangeToSite") or 0,tel.get("runwayAlongTrack") or 0,tel.get("runwayCrossTrack") or 0,
                    tel.get("angleOfAttack") or 0,tel.get("roll") or 0),flush=True)
                last_print=now
            if str(snap.get("phase","")).lower()=="abort":
                break
            sit=str(tel.get("vesselSituation","")).lower()
            if sit=="landed":
                success=abs(float(tel.get("runwayCrossTrack") or 1e9))<=100 and abs(float(tel.get("runwayAlongTrack") or 1e9))<=1800
                break
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
    }

    # Preserve the actual terminal controller state in replay; the disconnect
    # handshake intentionally produces a synthetic Idle snapshot that must not
    # become the replay endpoint.
    backend.freeze_replay()
    backend.send("disconnect","disconnect")
    time.sleep(.1)
    be_proc.terminate()
    if web_proc: web_proc.terminate()
    for f in (be_err,sim_stderr): f.close()
    backend.close()

    if manifest_path:
        finished_at=time.time()
        manifest.update({
            "state":"complete" if success else ("aborted" if terminal_phase.lower()=="abort" else "finished"),
            "finishedAt":finished_at,
            "wallSeconds":finished_at-started_at,
            "terminalPhase":terminal_phase or None,
            "final":final,
            "success":success,
        })
        manifest_path.write_text(json.dumps(manifest,indent=2,sort_keys=True)+"\n",encoding="utf-8")

    print(json.dumps({
        "success":success,"sim":str(sim_log),"guidance":str(guidance_log),
        "backendStderr":str(backend_err),"simStderr":str(sim_err),
        "runId":run_id if manifest_path else None,
        "manifest":str(manifest_path) if manifest_path else None,
    }))
    return 0 if success else 2

if __name__=="__main__":
    raise SystemExit(main())
