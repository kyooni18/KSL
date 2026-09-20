#!/usr/bin/env python3
import argparse,csv,json,math

def load_ksp(path,start_ut):
    rows=[]
    with open(path,newline='') as f:
        for r in csv.DictReader(f):
            try:
                ut=float(r['ut']); alt=float(r['altitude_m']); tas=float(r['true_air_speed_mps'])
                lat=float(r['latitude_deg']); lon=float(r['longitude_deg'])
                vs=float(r['vertical_speed_mps']); aoa=float(r['aoa_deg']); bank=float(r['bank_deg'])
            except (ValueError,KeyError):
                continue
            if ut < start_ut: continue
            rows.append({'t':ut-start_ut,'alt':alt,'speed':tas,'lat':lat,'lon':lon,'vs':vs,'aoa':aoa,'bank':bank})
    return rows

def load_sim(path):
    rows=[]
    with open(path) as f:
        for line in f:
            try:r=json.loads(line)
            except json.JSONDecodeError:continue
            p=r.get('position',{});v=r.get('velocity',{});a=r.get('attitude',{});sim=r.get('simulation',{})
            if 'altitude_m' not in p:continue
            rows.append({'t':sim.get('time_s',r.get('sim_time',0.0)),'alt':p['altitude_m'],'speed':v.get('air_mps',float('nan')),'lat':p.get('lat_deg',float('nan')),'lon':p.get('lon_deg',float('nan')),'vs':v.get('vertical_mps',float('nan')),'aoa':a.get('aoa_deg',float('nan')),'bank':a.get('bank_deg',float('nan'))})
    return rows

def nearest_alt(rows,h):
    return min(rows,key=lambda r:abs(r['alt']-h))

def dlon(a,b):
    d=b-a
    while d>180:d-=360
    while d<-180:d+=360
    return d

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('ksp_csv');ap.add_argument('sim_jsonl');ap.add_argument('--start-ut',type=float,required=True)
    ap.add_argument('--checkpoints',default='70000,60000,50000,40000,30000,20000,10000,5000,2000')
    a=ap.parse_args()
    k=load_ksp(a.ksp_csv,a.start_ut);s=load_sim(a.sim_jsonl)
    print('alt_m,ksp_t,sim_t,dt_s,speed_err,vs_err,lat_err_deg,lon_err_deg,aoa_err,bank_err')
    for h in map(float,a.checkpoints.split(',')):
        kr=nearest_alt(k,h);sr=nearest_alt(s,h)
        print(f"{h:.0f},{kr['t']:.2f},{sr['t']:.2f},{sr['t']-kr['t']:.2f},{sr['speed']-kr['speed']:.2f},{sr['vs']-kr['vs']:.2f},{sr['lat']-kr['lat']:.5f},{dlon(kr['lon'],sr['lon']):.5f},{sr['aoa']-kr['aoa']:.2f},{sr['bank']-kr['bank']:.2f}")
if __name__=='__main__':main()
