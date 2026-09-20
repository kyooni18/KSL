#!/usr/bin/env python3
"""Estimate rate/acceleration limits from normalized commanded-vs-actual telemetry.
Required: time_s,aoa_deg,bank_deg. Command columns are optional for future model fitting.
Outputs a ShuttleSim attitude INI while retaining conservative seed damping/frequencies.
"""
import argparse,csv,math,statistics

def percentile(xs,p):
    xs=sorted(x for x in xs if math.isfinite(x))
    if not xs:return 0.0
    k=(len(xs)-1)*p;i=int(k);f=k-i
    return xs[i]*(1-f)+xs[min(i+1,len(xs)-1)]*f

def angle_delta(a,b):
    d=b-a
    while d>180:d-=360
    while d<-180:d+=360
    return d

def main():
    ap=argparse.ArgumentParser();ap.add_argument('input');ap.add_argument('output');a=ap.parse_args()
    rows=[]
    with open(a.input,newline='') as f:
        for r in csv.DictReader(f):rows.append((float(r['time_s']),float(r['aoa_deg']),float(r['bank_deg'])))
    pr=[];rr=[]
    for x,y in zip(rows,rows[1:]):
        dt=y[0]-x[0]
        if 1e-4<dt<1.0:pr.append((y[1]-x[1])/dt);rr.append(angle_delta(x[2],y[2])/dt)
    pa=[(b-a)/(rows[i+2][0]-rows[i+1][0]) for i,(a,b) in enumerate(zip(pr,pr[1:])) if i+2<len(rows) and rows[i+2][0]>rows[i+1][0]]
    ra=[(b-a)/(rows[i+2][0]-rows[i+1][0]) for i,(a,b) in enumerate(zip(rr,rr[1:])) if i+2<len(rows) and rows[i+2][0]>rows[i+1][0]]
    vals={'max_pitch_rate_deg_s':percentile(map(abs,pr),.995),'max_roll_rate_deg_s':percentile(map(abs,rr),.995),'max_pitch_accel_deg_s2':percentile(map(abs,pa),.995),'max_roll_accel_deg_s2':percentile(map(abs,ra),.995)}
    with open(a.output,'w') as f:
        f.write('# Auto-estimated limits. Natural frequency/damping remain seeds until step-response fitting.\n')
        f.write('pitch_wn=1.4\npitch_zeta=0.9\nroll_wn=1.8\nroll_zeta=0.85\n')
        for k,v in vals.items():f.write(f'{k}={v:.6f}\n')
    print(vals)
if __name__=='__main__':main()
