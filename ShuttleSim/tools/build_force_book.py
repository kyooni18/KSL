#!/usr/bin/env python3
"""Build a compact cross-flight KSP direct-force data book.

Cells are indexed by log2(q), Mach and AoA. Each flight contributes at most one
median observation per cell; the final cell is the cross-flight median so a
single dense log cannot dominate the model.
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
    a=ap.parse_args()
    perflight=defaultdict(list)
    with open(a.input,newline="") as f:
        for r in csv.DictReader(f):
            q=num(r,"dynamic_pressure_pa");m=num(r,"mach");alpha=num(r,"aoa_deg")
            lift=num(r,"lift_force_n");drag=num(r,"drag_force_n");beta=num(r,"sideslip_deg",0)
            if not all(math.isfinite(x) for x in (q,m,alpha,lift,drag,beta)) or q<a.min_q:continue
            if abs(beta)>a.max_beta or num(r,"current_thrust_n",0)>500 or flag(r,"airbrakes") or flag(r,"gear_down"):continue
            lpq=lift/q;dpq=drag/q
            if lpq<0 or dpq<0 or lpq>1000 or dpq>1000:continue
            # half-octave q, 0.5 Mach, 5-degree AoA neighborhoods
            qb=round(math.log(q,2)*2)/2
            mb=round(m*2)/2
            ab=round(alpha/5)*5
            perflight[(r.get("source_log","unknown"),qb,mb,ab)].append((q,m,alpha,lpq,dpq))
    cross=defaultdict(list)
    for (flight,qb,mb,ab),vals in perflight.items():
        cols=list(zip(*vals))
        cross[(qb,mb,ab)].append(tuple(med(c) for c in cols))
    rows=[]
    for key,flights in cross.items():
        cols=list(zip(*flights))
        q,m,alpha,lpq,dpq=(med(c) for c in cols)
        rows.append((q,m,alpha,lpq,dpq,len(flights)))
    rows.sort(key=lambda r:(r[1],r[2],r[0]))
    with open(a.output,"w",newline="") as f:
        w=csv.writer(f);w.writerow(["q_pa","mach","aoa_deg","lift_per_q_m2","drag_per_q_m2","support"])
        for r in rows:w.writerow([f"{x:.9g}" if isinstance(x,float) else x for x in r])
    print(f"wrote {len(rows)} cells from {len(perflight)} per-flight cells; max support={max((r[-1] for r in rows),default=0)}")
if __name__=="__main__":main()
