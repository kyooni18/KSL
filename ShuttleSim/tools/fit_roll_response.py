#!/usr/bin/env python3
"""Fit ShuttleSim roll response from normalized KSP training telemetry.

The fit replays target_roll_deg through the same second-order, rate/acceleration-
limited, q-scaled plant used by ShuttleSim. Flights without meaningful bank
maneuvers are excluded so launch/orbit frame transients cannot bias the fit.
"""
import argparse,csv,collections,math

def wrap(x):
    return (x+180.0)%360.0-180.0

def load(path,min_q):
    groups=collections.defaultdict(dict)
    with open(path,newline="") as f:
        for r in csv.DictReader(f):
            try:
                t=float(r["time_s"]); q=float(r["dynamic_pressure_pa"])
                x=float(r["bank_deg"]); u=float(r["target_roll_deg"])
                v=float(r["roll_rate_deg_s"])
            except Exception:
                continue
            if not all(math.isfinite(z) for z in (t,q,x,u,v)):
                continue
            groups[r["source_log"]][round(t,6)]=(t,q,x,u,v)
    segments=[]
    for name,d in groups.items():
        rows=sorted(d.values())
        active=[r for r in rows if r[1]>=max(100.0,min_q)]
        if len(active)<50:
            continue
        targets=[r[3] for r in active]
        if max(targets)-min(targets)<30.0 or max(r[1] for r in active)<500.0:
            continue
        seg=[]
        for r in rows:
            bad=r[1]<min_q or abs(r[3])>90.0 or (seg and r[0]-seg[-1][0]>0.30)
            if bad:
                if len(seg)>20: segments.append((name,seg[::2]))
                seg=[]
                if r[1]<min_q or abs(r[3])>90.0: continue
            seg.append(r)
        if len(seg)>20: segments.append((name,seg[::2]))
    return segments

def evaluate(segments,p,details=False):
    wn,zeta,vmax,amax,qfull=p
    errs=[]; fast=[]
    for _,rows in segments:
        x=rows[0][2]; v=rows[0][4]
        prev_actual=x; prev_t=rows[0][0]
        for i,r in enumerate(rows[1:],1):
            t,q,actual,target,_=r
            dt=t-rows[i-1][0]
            if not (0.0<dt<=0.30):
                x=actual; v=0.0; prev_actual=actual; prev_t=t
                continue
            n=max(1,math.ceil(dt/0.02)); h=dt/n
            for _ in range(n):
                authority=max(0.0,min(1.0,q/max(qfull,1e-9)))
                root=math.sqrt(authority)
                e=wrap(target-x)
                ewn=wn*root; evmax=vmax*root; eamax=amax*authority
                if authority<=1e-12:
                    v=0.0
                else:
                    acc=ewn*ewn*e-2.0*zeta*ewn*v
                    acc=max(-eamax,min(eamax,acc))
                    v=max(-evmax,min(evmax,v+acc*h))
                    x=wrap(x+v*h)
            err=abs(wrap(x-actual)); errs.append(err)
            actual_rate=wrap(actual-prev_actual)/(t-prev_t) if t>prev_t else 0.0
            if abs(actual_rate)>=2.0: fast.append(err)
            prev_actual=actual; prev_t=t
    if not errs:
        return (math.inf,)*6 if details else math.inf
    se=sorted(errs); mae=sum(errs)/len(errs)
    fmae=sum(fast)/len(fast) if fast else mae
    p95=se[int(0.95*(len(se)-1))]
    objective=mae+0.35*fmae+0.05*p95
    if details:
        return objective,mae,fmae,p95,max(errs),len(errs)
    return objective

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("--min-q",type=float,default=10.0)
    args=ap.parse_args()
    segments=load(args.input,args.min_q)
    if not segments:
        raise SystemExit("no suitable maneuver segments")
    # Keep the directly observed KSP envelope fixed; fit only the closed-loop
    # shape and the q level at which full aerodynamic authority is reached.
    vmax=12.450661
    amax=23.324585
    candidates=[]
    for wn in (1.2,1.6,2.0,2.4,3.0):
        for zeta in (0.60,0.85,1.10,1.30):
            for qfull in (5.0,10.0,25.0,50.0):
                p=(wn,zeta,vmax,amax,qfull)
                candidates.append((evaluate(segments,p),p))
    candidates.sort()
    print(f"segments={len(segments)} samples={sum(len(s) for _,s in segments)}")
    for _,p in candidates[:10]:
        obj,mae,fmae,p95,mx,n=evaluate(segments,p,True)
        print("wn=%.3f zeta=%.3f vmax=%.6f amax=%.6f qfull=%.1f objective=%.5f mae=%.5f maneuver_mae=%.5f p95=%.5f max=%.5f n=%d" %
              (*p,obj,mae,fmae,p95,mx,n))
    current=(1.4,0.85,12.450661,23.324585,10.0)
    print("current",evaluate(segments,current,True))

if __name__=="__main__":
    main()
