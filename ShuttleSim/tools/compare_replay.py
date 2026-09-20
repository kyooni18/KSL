#!/usr/bin/env python3
"""Compare two ShuttleSim/KSP-normalized JSONL telemetry streams at altitude checkpoints."""
import argparse,json,math

def load(path):
    out=[]
    with open(path) as f:
        for line in f:
            try:r=json.loads(line)
            except json.JSONDecodeError:continue
            p=r.get('position',{});v=r.get('velocity',{});rw=r.get('runway',{})
            if 'altitude_m' in p:out.append((p['altitude_m'],v.get('air_mps',float('nan')),rw.get('along_m',float('nan')),rw.get('cross_m',float('nan')),r.get('sim_time',r.get('ut',0))))
    return out

def nearest(rows,h):return min(rows,key=lambda r:abs(r[0]-h))
def main():
    ap=argparse.ArgumentParser();ap.add_argument('reference');ap.add_argument('candidate');ap.add_argument('--checkpoints',default='70000,50000,40000,30000,20000,10000');a=ap.parse_args()
    r=load(a.reference);c=load(a.candidate)
    if not r or not c:raise SystemExit('missing telemetry rows')
    print('alt_m,speed_err_mps,along_err_m,cross_err_m,time_err_s')
    for h in map(float,a.checkpoints.split(',')):
        x=nearest(r,h);y=nearest(c,h)
        print(f'{h:.0f},{y[1]-x[1]:.3f},{y[2]-x[2]:.3f},{y[3]-x[3]:.3f},{y[4]-x[4]:.3f}')
if __name__=='__main__':main()
