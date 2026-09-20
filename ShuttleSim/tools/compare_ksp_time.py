#!/usr/bin/env python3
import argparse,csv,json,math

def fnum(r,k,default=float("nan")):
    try:return float(r[k])
    except:return default

def load_ksp(path,start_ut):
    out=[]
    with open(path,newline='') as f:
        for r in csv.DictReader(f):
            ut=fnum(r,'ut')
            if not math.isfinite(ut) or ut<start_ut: continue
            out.append({
                't':ut-start_ut,'alt':fnum(r,'altitude_m'),'speed':fnum(r,'true_air_speed_mps'),
                'vs':fnum(r,'vertical_speed_mps'),'q':fnum(r,'dynamic_pressure_pa'),
                'mach':fnum(r,'mach'),'lift':fnum(r,'lift_force_n'),'drag':fnum(r,'drag_force_n'),
                'aoa':fnum(r,'aoa_deg'),'bank':fnum(r,'bank_deg'),
                'lat':fnum(r,'latitude_deg'),'lon':fnum(r,'longitude_deg')
            })
    return out

def load_sim(path):
    out=[]
    with open(path) as f:
        for line in f:
            try:r=json.loads(line)
            except:continue
            p=r.get('position',{});v=r.get('velocity',{});a=r.get('aero',{});att=r.get('attitude',{})
            if 'altitude_m' not in p:continue
            out.append({
                't':float(r.get('sim_time',0)),'alt':float(p['altitude_m']),
                'speed':float(v.get('air_mps',float('nan'))),'vs':float(v.get('vertical_mps',float('nan'))),
                'q':float(a.get('q_pa',float('nan'))),'mach':float(a.get('mach',float('nan'))),
                'lift':float(a.get('lift_n',float('nan'))),'drag':float(a.get('drag_n',float('nan'))),
                'aoa':float(att.get('aoa_deg',float('nan'))),'bank':float(att.get('bank_deg',float('nan'))),
                'lat':float(p.get('lat_deg',float('nan'))),'lon':float(p.get('lon_deg',float('nan')))
            })
    return out

def nearest(rows,key,val):return min(rows,key=lambda r:abs(r[key]-val))
def dlon(a,b):
    d=b-a
    while d>180:d-=360
    while d<-180:d+=360
    return d

def main():
    ap=argparse.ArgumentParser();ap.add_argument('ksp_csv');ap.add_argument('sim_jsonl')
    ap.add_argument('--start-ut',type=float,required=True)
    ap.add_argument('--altitudes',default='70000,65000,60000,55000,50000,45000,40000,30000,20000,10000')
    a=ap.parse_args(); k=load_ksp(a.ksp_csv,a.start_ut); s=load_sim(a.sim_jsonl)
    print('ksp_alt,t,sim_alt,alt_err,speed_err,vs_err,q_ratio,lift_ratio,drag_ratio,aoa_err,bank_err,lat_err,lon_err')
    for h in map(float,a.altitudes.split(',')):
        kr=nearest(k,'alt',h); sr=nearest(s,'t',kr['t'])
        ratio=lambda x,y: (x/y if y and math.isfinite(y) and math.isfinite(x) else float('nan'))
        print(f"{kr['alt']:.1f},{kr['t']:.2f},{sr['alt']:.1f},{sr['alt']-kr['alt']:.1f},"
              f"{sr['speed']-kr['speed']:.2f},{sr['vs']-kr['vs']:.2f},{ratio(sr['q'],kr['q']):.3f},"
              f"{ratio(sr['lift'],kr['lift']):.3f},{ratio(sr['drag'],kr['drag']):.3f},"
              f"{sr['aoa']-kr['aoa']:.2f},{sr['bank']-kr['bank']:.2f},"
              f"{sr['lat']-kr['lat']:.4f},{dlon(kr['lon'],sr['lon']):.4f}")
if __name__=='__main__':main()
