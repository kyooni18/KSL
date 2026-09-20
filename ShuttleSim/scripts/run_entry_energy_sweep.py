#!/usr/bin/env python3
from __future__ import annotations
import concurrent.futures, csv, importlib.util, json, math, pathlib, re, subprocess, time

ROOT=pathlib.Path(__file__).resolve().parents[2]
SIM=ROOT/"ShuttleSim"
BASE=SIM/"campaigns/20260918-one-hour-full/inputs/configs/G057.json"
SCENARIO=SIM/"scenarios/ksp86km-postburn.ini"
ATM=SIM/"data/fitted/kerbin_atmosphere_ksp.csv"
AERO=SIM/"data/fitted/stsn_aero_ksp_robust.csv"
BOOK=SIM/"data/fitted/stsn_force_book.csv"
ATT=SIM/"data/fitted/stsn_attitude_ksp.ini"
RUNNER=SIM/"scripts/run_guidance.py"

spec=importlib.util.spec_from_file_location("campaign",SIM/"scripts/run_campaign.py")
campaign_mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(campaign_mod)

STAMP=time.strftime("%Y%m%dT%H%M%SZ",time.gmtime())
OUT=SIM/"campaigns"/f"{STAMP}-focused-entry-energy"
(OUT/"configs").mkdir(parents=True,exist_ok=True)

BASECFG=json.loads(BASE.read_text())

cases=[]
idx=0
for aoa in (26.0,30.0,34.0):
    for bank in (35.0,45.0,55.0):
        cfg=json.loads(json.dumps(BASECFG))
        cfg["vehicle"]["entryAngleOfAttack"]=aoa
        cfg["vehicle"]["maximumAngleOfAttack"]=max(38.0,aoa)
        cfg["vehicle"]["maximumBankAngle"]=bank
        cid=f"E{idx:02d}"; idx+=1
        cp=OUT/"configs"/f"{cid}.json"
        cp.write_text(json.dumps(cfg,indent=2,sort_keys=True)+"\n")
        cases.append((cid,aoa,bank,cp))

handoff_re=re.compile(r"MM304 SAFETY HANDOFF: UT ([0-9.]+) h ([0-9.]+) V ([0-9.]+) FPA ([+-]?[0-9.]+) along ([+-]?[0-9.]+) cross ([+-]?[0-9.]+)")

def run(case):
    cid,aoa,bank,cp=case
    base_port=25200+int(cid[1:])*3
    label=f"focused-{cid}-a{aoa:.0f}-b{bank:.0f}"
    cmd=[
        "python3",str(RUNNER),
        "--scenario",str(SCENARIO),"--configuration",str(cp),
        "--dt","0.02","--atmosphere",str(ATM),"--aero",str(AERO),
        "--aero-book",str(BOOK),"--attitude",str(ATT),
        "--telemetry-hz","10","--max-sim-time","2200",
        "--label",label,"--command-port",str(base_port),
        "--telemetry-port",str(base_port+1),"--web-telemetry-port","0",
        "--skip-build","--no-mirror","--quiet-progress","--compact-guidance-log",
    ]
    t0=time.monotonic()
    p=subprocess.run(cmd,cwd=ROOT,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    wall=time.monotonic()-t0
    result={}
    for line in reversed(p.stdout.splitlines()):
        try:
            x=json.loads(line)
        except Exception:
            continue
        if isinstance(x,dict) and "sim" in x:
            result=x;break
    sim=pathlib.Path(result.get("sim",""))
    guide=pathlib.Path(result.get("guidance",""))
    be=pathlib.Path(result.get("backendStderr",""))
    se=pathlib.Path(result.get("simStderr",""))
    metrics=campaign_mod.parse_sim_metrics(sim) if sim.is_file() else {}
    g=campaign_mod.parse_guidance(guide) if guide.is_file() else {}
    handoff={}
    if be.is_file():
        txt=be.read_text(errors="replace")
        m=handoff_re.search(txt)
        if m:
            handoff={
                "handoff_ut":float(m.group(1)),"handoff_altitude_m":float(m.group(2)),
                "handoff_speed_mps":float(m.group(3)),"handoff_fpa_deg":float(m.group(4)),
                "handoff_along_m":float(m.group(5)),"handoff_cross_m":float(m.group(6)),
            }
    out={"id":cid,"entry_aoa_deg":aoa,"max_bank_deg":bank,"returncode":p.returncode,
         "wall_s":wall,**metrics,**g,**handoff}
    # The sweep is disposable; preserve only compact results.
    for path in (sim,guide,be,se):
        if path.is_file():
            try:path.unlink()
            except OSError:pass
    return out

rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as ex:
    futures=[ex.submit(run,c) for c in cases]
    for fut in concurrent.futures.as_completed(futures):
        r=fut.result()
        rows.append(r)
        print(json.dumps(r,sort_keys=True),flush=True)

rows.sort(key=lambda r:(0 if r.get("on_runway") else 1,0 if r.get("touchdown") else 1,float(r.get("terminal_metric_m",1e12))))
(OUT/"results.json").write_text(json.dumps(rows,indent=2,sort_keys=True)+"\n")
fields=sorted({k for r in rows for k in r if not isinstance(r[k],(dict,list))})
with (OUT/"results.csv").open("w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows({k:v for k,v in r.items() if k in fields} for r in rows)
print("OUT",OUT)
print("BEST",json.dumps(rows[0],sort_keys=True))
