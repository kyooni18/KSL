#!/usr/bin/env python3
from __future__ import annotations
import argparse, io, json, math, os, pathlib, shutil, socket, subprocess, time
from run_backend import Backend, stop_process
from run_geometry import fixed_hac_geometry_evidence
from run_artifacts import allocate_run, compress_recording, write_json
from model_paths import model_file

ROOT = pathlib.Path(__file__).resolve().parents[2]
SIM = ROOT / "ShuttleSim"
_clanding_raw = pathlib.Path(os.environ.get("KSP_CLANDING_ROOT", "CLanding")).expanduser()
CLANDING = _clanding_raw if _clanding_raw.is_absolute() else (ROOT / _clanding_raw)
CLANDING = CLANDING.resolve()
WEB_RUN = ROOT / "Runtime" / "WebTelemetry" / "simulator-run.json"

def publish_web_run(doc):
    """Mirror run status for Tools/telemetry_web.py (same schema as simulator_campaign.py)."""
    if not _ACTIVE.get("mirror",True):
        return
    try:
        write_json(WEB_RUN,dict(doc,schema=1))
    except OSError:
        pass


# Tracks the in-flight run so an exception or signal can still finalize it
# (manifest state, simulator-run.json, archive into runs/) instead of leaving
# the telemetry server showing a phantom "running" run forever.
_ACTIVE={}


def _reserve_udp_endpoint(requested_port: int):
    """Reserve one loopback UDP endpoint; port 0 asks the OS for a free port."""
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1",requested_port))
    except Exception:
        sock.close()
        raise
    return sock,int(sock.getsockname()[1])


def _reserve_sim_endpoints(command_port: int, telemetry_port: int):
    """Atomically hold distinct command/telemetry endpoints until their owners bind."""
    if command_port and telemetry_port and command_port==telemetry_port:
        raise ValueError("command and telemetry ports must be distinct")
    command_sock,command_actual=_reserve_udp_endpoint(command_port)
    try:
        telemetry_sock,telemetry_actual=_reserve_udp_endpoint(telemetry_port)
    except Exception:
        command_sock.close()
        raise
    if command_actual==telemetry_actual:
        command_sock.close(); telemetry_sock.close()
        raise RuntimeError("OS assigned the same UDP endpoint twice")
    return {"command":command_sock,"telemetry":telemetry_sock},command_actual,telemetry_actual


def _release_endpoint(reservations,key):
    sock=reservations.pop(key,None) if reservations else None
    if sock is not None:
        try:sock.close()
        except OSError:pass


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
    for key in tuple((run.get("port_reservations") or {}).keys()):
        _release_endpoint(run.get("port_reservations"),key)
    for proc in (run.get("sim_proc"),run.get("be_proc")):
        try:stop_process(proc)
        except Exception:pass
    manifest=run.get("manifest"); manifest_path=run.get("manifest_path"); run_dir=run.get("run_dir")
    if manifest is None or manifest_path is None or run_dir is None:
        return
    _close_out_run(manifest,manifest_path,run_dir,run.get("final_run_dir"),reason)


def _pid_alive(pid):
    if not isinstance(pid,int) or isinstance(pid,bool) or pid<=0:
        return False
    try:
        os.kill(pid,0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    except (OverflowError,ValueError):
        return False
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
        # Old manifests cannot prove ownership of their children. Leave those
        # attempts intact rather than killing processes by a command-line regex.
        children=[manifest.get("simulatorPid"),manifest.get("backendPid")]
        if any(pid is None or _pid_alive(pid) for pid in children):
            continue
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
        write_json(manifest_path,manifest)
        if final_dir is not None and not final_dir.exists():
            run_dir.replace(final_dir); run_dir=final_dir
    except OSError:
        pass
    publish_web_run(dict(manifest,runDirectory=str(run_dir) if run_dir.is_dir() else None))


def _phase_run_label(label: str, scenario: str, engage: str) -> tuple[str, str | None]:
    """Classify runs by the guidance mode that actually owns the vehicle."""
    engage_name=engage.lower()
    scenario_name=pathlib.Path(scenario).stem.lower()

    if engage_name=="engagereentry":
        normalized=label if label.lower()=="mm304" or label.lower().startswith("mm304-") else f"mm304-{label}"
        return normalized,"MM304"
    if engage_name=="engagehactest":
        normalized=label if label.lower()=="mm305" or label.lower().startswith("mm305-") else f"mm305-{label}"
        return normalized,"MM305"
    if engage_name=="engagefinaltest":
        normalized=label if label.lower()=="final" or label.lower().startswith("final-") else f"final-{label}"
        return normalized,"Final"

    # Production engage owns the whole mission. Scenario names are only a useful
    # fallback tag for older direct invocations; they never override an explicit
    # phase-specific engage method above.
    if scenario_name.startswith("mm304-"):
        normalized=label if label.lower()=="mm304" or label.lower().startswith("mm304-") else f"mm304-{label}"
        return normalized,"MM304"
    if scenario_name.startswith("mm305-"):
        normalized=label if label.lower()=="mm305" or label.lower().startswith("mm305-") else f"mm305-{label}"
        return normalized,"MM305"
    return label,None


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--scenario",default=str(SIM/"scenarios/ksp86km-postburn.ini"))
    ap.add_argument("--telemetry-hz",type=float,default=10.0)
    ap.add_argument("--max-sim-time",type=float,default=2400.0)
    ap.add_argument("--plan-timeout",type=float,default=600.0,
                    help="maximum wall-clock seconds to allow production deorbit createPlan")
    ap.add_argument("--label",default="guidance")
    ap.add_argument("--command-port",type=int,default=0,help="simulator command UDP port; 0 selects a fresh OS-assigned endpoint")
    ap.add_argument("--telemetry-port",type=int,default=0,help="simulator telemetry UDP port; 0 selects a fresh OS-assigned endpoint")
    ap.add_argument("--web-telemetry-port",type=int,default=8797)
    ap.add_argument("--web",action="store_true",help="deprecated no-op; the persistent Telemetry Web service is used")
    ap.add_argument("--configuration",default=str(ROOT/"Configuration/default.json"))
    ap.add_argument("--dt",type=float,default=0.02)
    ap.add_argument("--atmosphere",default=str(model_file("atmosphere")))
    ap.add_argument("--aero",default=str(model_file("aero")))
    ap.add_argument("--aero-book",default=str(model_file("aero_book")),
                    help="direct-force data book, or 'none' to disable")
    ap.add_argument("--attitude",default=str(model_file("attitude")))
    # Guidance-side model files.  By default guidance is given the plant's own
    # files (perfect knowledge); pass different files to test model error.
    ap.add_argument("--terminal-atmosphere",default=None,help="guidance atmosphere model (default: --atmosphere)")
    ap.add_argument("--terminal-aero",default=None,help="MM305 aero table (default: --aero)")
    ap.add_argument("--terminal-aero-book",default=None,help="guidance force book / certified prior (default: --aero-book)")
    ap.add_argument("--terminal-attitude",default=None,help="MM305 attitude model (default: --attitude)")
    ap.add_argument("--engage",default="engageReentry",
                    choices=["engage","engageReentry","engageHACTest","engageFinalTest"],
                    help="backend engage method; engage runs createPlan + production deorbit/entry/landing guidance")
    ap.add_argument("--backend-build-dir",type=pathlib.Path,default=CLANDING/"build")
    ap.add_argument("--sim-build-dir",type=pathlib.Path,default=SIM/"build")
    ap.add_argument("--skip-build",action="store_true")
    ap.add_argument("--no-mirror",action="store_true")
    ap.add_argument("--quiet-progress",action="store_true")
    ap.add_argument("--compact-guidance-log",action="store_true",
                    help="omit heavy trajectory arrays from scratch Guidance logs")
    ap.add_argument("--archive-replay",action=argparse.BooleanOptionalAction,default=True,
                    help="write a Telemetry Web replay manifest + exact Guidance snapshots (default on, so every advanced run appears in sim run history)")
    args=ap.parse_args()
    args.scenario=str(pathlib.Path(args.scenario).resolve())
    args.backend_build_dir=args.backend_build_dir.expanduser().resolve()
    args.sim_build_dir=args.sim_build_dir.expanduser().resolve()
    if any(not 0<=p<=65535 for p in (args.command_port,args.telemetry_port)):
        ap.error("command and telemetry ports must be 0 (automatic) or valid UDP ports")
    if args.command_port and args.telemetry_port and args.command_port==args.telemetry_port:
        ap.error("explicit command and telemetry ports must be distinct")
    if any(not math.isfinite(v) or v<=0 for v in (args.dt,args.telemetry_hz,args.max_sim_time,args.plan_timeout)):
        ap.error("time steps, rates, and timeouts must be positive finite values")
    if not args.label or pathlib.Path(args.label).name!=args.label or args.label in (".",".."):
        ap.error("label must be a single nonempty filename component")
    for name in ("scenario","configuration","atmosphere","aero","aero_book","attitude",
                 "terminal_atmosphere","terminal_aero","terminal_aero_book","terminal_attitude"):
        value=getattr(args,name)
        if value is None:
            continue
        if name in ("aero_book","terminal_aero_book") and value.lower()=="none":
            setattr(args,name,"none")
            continue
        path=pathlib.Path(value).expanduser().resolve()
        if not path.is_file():
            ap.error(f"{name} input does not exist: {path}")
        setattr(args,name,str(path))
    if args.no_mirror:
        args.web_telemetry_port=0
    _ACTIVE["mirror"]=not args.no_mirror

    try:
        port_reservations,args.command_port,args.telemetry_port=_reserve_sim_endpoints(
            args.command_port,args.telemetry_port)
    except (OSError,ValueError,RuntimeError) as exc:
        ap.error(f"cannot reserve simulator UDP endpoints: {exc}")
    _ACTIVE["port_reservations"]=port_reservations

    stamp=time.strftime("%Y%m%dT%H%M%SZ",time.gmtime())
    run_label, mission = _phase_run_label(args.label, args.scenario, args.engage)
    run_dir=allocate_run(SIM/".pending-runs",run_label,stamp)
    run_id=run_dir.name
    runs_root=SIM/"runs"; runs_root.mkdir(parents=True,exist_ok=True)
    if not args.no_mirror:
        recover_orphaned_runs(SIM/".pending-runs",runs_root)
    final_run_dir=None
    if args.archive_replay:
        # Do not publish an attempt into the replay catalog until physics has
        # actually advanced. Planning/admission failures remain outside runs/.
        final_run_dir=runs_root/run_id
        sim_log=run_dir/"simulator-telemetry.jsonl"
        guidance_log=run_dir/"guidance-events.jsonl"
        guidance_replay=run_dir/"guidance-snapshots.jsonl"
        backend_err=run_dir/"backend.stderr.log"
        sim_err=run_dir/"simulator.stderr.log"
        manifest_path=run_dir/"manifest.json"
    else:
        # Stage flat logs outside runs/ too; they are published only once
        # physics has advanced (sim_time > 0).
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
            "commandPort":args.command_port,"telemetryPort":args.telemetry_port,
        }
        write_json(manifest_path,manifest); publish_web_run(dict(manifest,runDirectory=str(run_dir)))
        _ACTIVE.update(manifest=manifest,manifest_path=manifest_path,run_dir=run_dir,final_run_dir=final_run_dir)

    if not args.skip_build:
        subprocess.run(["make","-C",str(CLANDING),f"BUILD={args.backend_build_dir}","-j","2"],check=True,stdout=subprocess.DEVNULL)
        subprocess.run(["cmake","-S",str(SIM),"-B",str(args.sim_build_dir)],check=True,stdout=subprocess.DEVNULL)
        subprocess.run(["cmake","--build",str(args.sim_build_dir),"-j","2"],check=True,stdout=subprocess.DEVNULL)

    env=os.environ.copy()
    env.update({
        "KSP_LANDER_ROOT":str(ROOT),
        "KSP_LANDER_SIMULATOR":"1",
        "KSP_LANDER_UNPOWERED_ONLY":"1",
        "KSP_LANDER_HAC_DIAGNOSTICS":"1",
        "KSP_LANDER_SIM_COMMAND_PORT":str(args.command_port),
        "KSP_LANDER_SIM_TELEMETRY_PORT":str(args.telemetry_port),
        "KSP_LANDER_TERMINAL_ATMOSPHERE":args.terminal_atmosphere or args.atmosphere,
        "KSP_LANDER_TERMINAL_AERO":args.terminal_aero or args.aero,
        "KSP_LANDER_TERMINAL_AERO_BOOK":args.terminal_aero_book or args.aero_book,
        "KSP_LANDER_TERMINAL_ATTITUDE":args.terminal_attitude or args.attitude,
    })
    # A phase-specific CLI action must select its matching backend mode, not
    # depend on an inherited operator shell variable.
    env.pop("KSP_LANDER_FINAL_TEST_LIVE",None)
    env.pop("KSP_LANDER_FINAL_APPROACH_TEST",None)
    if args.engage=="engageFinalTest":
        env["KSP_LANDER_FINAL_APPROACH_TEST"]="1"
    # Checkpoint-only reentry runs intentionally bypass orbital planning and hold
    # the current attempt out of the learned physics store. A production full
    # mission must use the normal orbital/deorbit state machine instead.
    if args.engage != "engage":
        env["KSP_LANDER_FORCE_REENTRY_TEST"]="1"
    else:
        env.pop("KSP_LANDER_FORCE_REENTRY_TEST",None)
    be_err=backend_err.open("w",encoding="utf-8")
    be_proc=subprocess.Popen([str(args.backend_build_dir/"landing_backend")],cwd=ROOT,env=env,
        stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=be_err,text=True,bufsize=1)
    _ACTIVE["be_proc"]=be_proc
    if manifest_path:
        manifest["backendPid"]=be_proc.pid
        manifest["physicsSources"]={name:getattr(args,name) for name in ("atmosphere","aero","aero_book","attitude")}
        manifest["guidanceModelSources"]={name:getattr(args,"terminal_"+name) or getattr(args,name) for name in ("atmosphere","aero","aero_book","attitude")}
        write_json(manifest_path,manifest)
    backend=Backend(be_proc,guidance_log,mirror=not args.no_mirror,replay_path=guidance_replay,
                    compact_log=args.compact_guidance_log)
    backend.wait(lambda o:o.get("type")=="ready",10)

    config=json.loads(pathlib.Path(args.configuration).read_text())
    # The backend owns the telemetry bind. Release only that reservation at the
    # exact handoff point; the command endpoint remains reserved until ShuttleSim
    # is ready to become its owner.
    _release_endpoint(port_reservations,"telemetry")
    backend.send("connect","connect",configuration=config)
    backend.wait(lambda o:o.get("type")=="response" and o.get("id")=="connect",10)

    web_proc=None
    # Never spawn a separate Telemetry Web server: the persistent service
    # (Tools/telemetry_web_service.sh) receives simulator UDP on 8797 and
    # lists this run from ShuttleSim/runs.

    sim_stderr=sim_err.open("w",encoding="utf-8")
    sim_cmd=[
        str(args.sim_build_dir/"shuttlesim"),
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
    _release_endpoint(port_reservations,"command")
    sim_proc=subprocess.Popen(sim_cmd,cwd=ROOT,stdout=subprocess.DEVNULL,stderr=sim_stderr,text=True)
    _ACTIVE["sim_proc"]=sim_proc
    if manifest_path:
        manifest["simulatorPid"]=sim_proc.pid
        write_json(manifest_path,manifest)

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
                write_json(manifest_path,manifest); publish_web_run(dict(manifest,runDirectory=str(run_dir)))
            raise
        planned=backend.latest if isinstance(backend.latest,dict) else {}
        if str(planned.get("phase") or "") not in ("Fault","Abort"):
            # The createPlan response can arrive before the throttled snapshot
            # that carries the finished plan; judging qualification from the
            # still-"Planning" snapshot rejected qualified plans.
            try:
                settled=backend.wait(lambda o:o.get("type")=="snapshot" and
                    isinstance(o.get("snapshot"),dict) and
                    str(o["snapshot"].get("phase") or "")!="Planning" and
                    isinstance(o["snapshot"].get("plan"),dict) and
                    "executionQualified" in o["snapshot"]["plan"],15)
                planned=settled["snapshot"]
            except TimeoutError:
                pass
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
    if not engage_rejected:
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
    last_ut=None
    last_progress=time.monotonic()
    while sim_proc.poll() is None and be_proc.poll() is None:
        backend.mailbox.check()
        snap=backend.latest
        if snap:
            tel=snap.get("telemetry") or {}; now=time.monotonic()
            ut=tel.get("ut")
            if isinstance(ut,(int,float)) and math.isfinite(ut) and ut!=last_ut:
                last_ut=ut
                last_progress=now
            elif now-last_progress>=args.plan_timeout:
                raise TimeoutError("lockstep telemetry stopped advancing within the configured timeout")
            # An Abort releases control.  In lockstep the simulator then waits
            # forever for the next guidance command, so stop this run at the
            # terminal snapshot instead of leaving both processes resident.
            if snap.get("phase") in ("Abort","Fault"):
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
    fixed_hac_geometry=fixed_hac_geometry_evidence(
        backend_err.read_text(encoding="utf-8",errors="replace"),
        config["guidance"]["finalApproachDistance"])
    runway_length=config["site"]["runwayLength"]
    runway_half_width=config["site"]["runwayWidth"]/2.0
    runway_end=fixed_hac_geometry.get("runwayEnd",0)
    selected_heading=(config["site"]["runwayHeading"]+180.0*runway_end)%360.0
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
                if not isinstance(along,(int,float)) or not isinstance(cross,(int,float)) or not (0.0<=along<=runway_length and abs(cross)<=runway_half_width):
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
                    runway_heading=selected_heading
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
                    0 <= simulator_summary.get("touchdown_along_m",float("-inf")) <= runway_length and
                    abs(simulator_summary.get("touchdown_cross_m",float("inf"))) <= runway_half_width)
    controls_clean=True
    observed_phases=set()
    fixed_hac_seen=False
    hac_completed_seen=False
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
            # Commitment geometry and observed completion are separate evidence.
            radius=state.get("hacRadius")
            fixed_hac_seen=fixed_hac_seen or (fixed_hac_geometry.get("valid") is True and
                state.get("terminalPathCommitted") is True and
                isinstance(radius,(int,float)) and math.isfinite(radius) and
                abs(radius-fixed_hac_geometry["radius"]) <= .01)
            hac_completed_seen=hac_completed_seen or state.get("hacCompleted") is True
            spline_seen=spline_seen or "spline" in status.lower()
            command=snap.get("command") or {}
            controls_clean=controls_clean and command.get("targetThrottle")==0 and command.get("airbrakes") is False
    final_ground=(final_sim or {}).get("ground") or {}
    rollout_valid=rollout_seen and final_ground.get("on_ground") is True
    touchdown_speed=simulator_summary.get("touchdown_speed_mps")
    touchdown_speed_ok=isinstance(touchdown_speed,(int,float)) and 60.0<=touchdown_speed<=70.0
    touchdown_sink=simulator_summary.get("touchdown_sink_mps")
    touchdown_sink_ok=isinstance(touchdown_sink,(int,float)) and 0.0<=touchdown_sink<=3.0
    rollout_controls_ok=rollout_brakes_seen and not rollout_airbrakes_seen
    rollout_heading_ok=rollout_max_heading_error<=8.0
    rollout_stability_proxy_ok=rollout_max_lateral_accel<=1.25
    hac_requirement_ok=(args.engage=="engageFinalTest" or (fixed_hac_seen and hac_completed_seen))
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
    final["hacCompletedSeen"]=hac_completed_seen
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
        write_json(manifest_path,manifest)
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

    sim_log=compress_recording(sim_log)
    guidance_log=compress_recording(guidance_log)
    guidance_replay=compress_recording(guidance_replay)
    if manifest_path:
        manifest["simulatorTelemetry"]=sim_log.name
        manifest["guidanceEvents"]=guidance_log.name
        manifest["guidanceSnapshots"]=guidance_replay.name if guidance_replay else None
        write_json(manifest_path,manifest)
        publish_web_run(dict(manifest,runDirectory=str(run_dir)))

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
