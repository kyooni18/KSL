#!/usr/bin/env python3
"""Build a compact direct KSP force/q data book.

KSP lift/drag are refreshed by the C-Nano transport's 0.25 s medium telemetry
batch, while q, Mach and AoA are read by the fast batch every telemetry call.
The historical production book intentionally uses all normalized rows because
held-out trajectory replay shows that the repeated cached forces act as useful
temporal smoothing.  --fresh-force-only is provided as a calibration diagnostic,
not as the production default: a changed force value only proves that a medium
RPC completed, not that it was sampled at exactly the same instant as the fast
q/AoA batch.

Cells are indexed by log2(q), Mach and AoA. Each source flight contributes at
most one median observation per cell so repeated samples from one flight cannot
silently dominate cross-flight evidence.
"""
import argparse,csv,math,statistics
from collections import defaultdict

def num(r,k,d=float("nan")):
    try:return float(r.get(k,""))
    except:return d

def flag(r,k):
    return str(r.get(k,"")).strip().lower() in ("1","true","yes","on")

def med(xs):return statistics.median(xs)

def main():
    ap=argparse.ArgumentParser();ap.add_argument("input");ap.add_argument("output")
    ap.add_argument("--min-q",type=float,default=1.0)
    ap.add_argument("--max-beta",type=float,default=6.0)
    ap.add_argument("--q-octave-bin",type=float,default=0.25)
    ap.add_argument("--mach-bin",type=float,default=0.25)
    ap.add_argument("--aoa-bin",type=float,default=2.5)
    ap.add_argument("--fresh-force-only",action="store_true",
        help="diagnostic: discard rows that repeat cached lift/drag values")
    a=ap.parse_args()
    if min(a.q_octave_bin,a.mach_bin,a.aoa_bin)<=0:raise SystemExit("bin widths must be positive")
    perflight=defaultdict(list)
    last_force={}
    total_rows=0
    fresh_rows=0
    with open(a.input,newline="") as f:
        for r in csv.DictReader(f):
            total_rows+=1
            source=r.get("source_log","unknown")
            q=num(r,"dynamic_pressure_pa");m=num(r,"mach");alpha=num(r,"aoa_deg")
            lift=num(r,"lift_force_n");drag=num(r,"drag_force_n");beta=num(r,"sideslip_deg",0)
            if not all(math.isfinite(x) for x in (q,m,alpha,lift,drag,beta)):
                continue

            previous=last_force.get(source)
            fresh=(previous is None or
                   not (math.isclose(lift,previous[0],rel_tol=1e-10,abs_tol=1e-6) and
                        math.isclose(drag,previous[1],rel_tol=1e-10,abs_tol=1e-6)))
            last_force[source]=(lift,drag)
            if a.fresh_force_only and not fresh:
                continue
            if fresh:
                fresh_rows+=1

            if q<a.min_q:
                continue
            if abs(beta)>a.max_beta or num(r,"current_thrust_n",0)>500 or flag(r,"airbrakes") or flag(r,"gear_down"):
                continue
            lpq=lift/q;dpq=drag/q
            if lpq<0 or dpq<0 or lpq>1000 or dpq>1000:
                continue
            qb=round(math.log(q,2)/a.q_octave_bin)*a.q_octave_bin
            mb=round(m/a.mach_bin)*a.mach_bin
            ab=round(alpha/a.aoa_bin)*a.aoa_bin
            perflight[(source,qb,mb,ab)].append((q,m,alpha,lpq,dpq))
    rows=[]
    flights=set()
    for (flight,_qb,_mb,_ab),vals in perflight.items():
        cols=list(zip(*vals));q,m,alpha,lpq,dpq=(med(c) for c in cols)
        rows.append((q,m,alpha,lpq,dpq,1));flights.add(flight)
    rows.sort(key=lambda r:(r[1],r[2],r[0]))
    with open(a.output,"w",newline="") as f:
        w=csv.writer(f);w.writerow(["q_pa","mach","aoa_deg","lift_per_q_m2","drag_per_q_m2","support"])
        for r in rows:w.writerow([f"{x:.9g}" if isinstance(x,float) else x for x in r])
    mode="fresh force updates only" if a.fresh_force_only else "all normalized rows"
    print(f"wrote {len(rows)} independent per-flight cells from {len(flights)} flights; "
          f"input_rows={total_rows} fresh_force_rows={fresh_rows} mode={mode}")

if __name__=="__main__":main()
