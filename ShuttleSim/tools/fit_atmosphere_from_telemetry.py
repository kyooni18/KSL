#!/usr/bin/env python3
"""Bin normalized KSP atmospheric telemetry into a ShuttleSim atmosphere CSV.
Required columns: altitude_m,density_kg_m3,pressure_pa,temperature_k,speed_of_sound_mps
"""
import argparse,csv,math,statistics

def main():
    ap=argparse.ArgumentParser();ap.add_argument('input');ap.add_argument('output');ap.add_argument('--bin-m',type=float,default=250.0);a=ap.parse_args()
    bins={}
    with open(a.input,newline='') as f:
        for r in csv.DictReader(f):
            h=float(r['altitude_m']);k=round(h/a.bin_m)*a.bin_m
            vals=tuple(float(r[x]) for x in ('density_kg_m3','pressure_pa','temperature_k','speed_of_sound_mps'))
            if all(math.isfinite(x) for x in vals):bins.setdefault(k,[]).append(vals)
    with open(a.output,'w',newline='') as f:
        w=csv.writer(f);w.writerow(['altitude_m','density_kg_m3','pressure_pa','temperature_k','speed_of_sound_mps'])
        for h in sorted(bins):
            cols=list(zip(*bins[h]));w.writerow([h]+[statistics.median(x) for x in cols])
    print(f'wrote {len(bins)} altitude bins to {a.output}')
if __name__=='__main__':main()
