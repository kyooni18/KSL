#!/usr/bin/env python3
"""Offline trajectory check against an archived STS flight; never connects to KSP.

Actual-attitude replay isolates translational physics. Command replay additionally
exercises the borrowed attitude servo. This is an in-flight reconstruction, not
held-out aerodynamic validation or proof of landing capability.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
from normalize_ksp_vehicle_log import decode_log
from compare_ksp_time import load_ksp, load_sim, nearest, dlon

ROOT = Path(__file__).resolve().parents[2]


def identity(path):
    return dict(path=str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path),
                sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--log', type=Path, default=ROOT/'FlightLogs/2026-10-07T21-34-08Z-STS-vehicle.jsonl')
    ap.add_argument('--scenario', type=Path, default=ROOT/'ShuttleSim/scenarios/sts70km-run9-provisional.ini')
    ap.add_argument('--aero', type=Path, default=ROOT/'ShuttleSim/reference-model/sts_aero_provisional.csv')
    ap.add_argument('--attitude', type=Path, default=ROOT/'ShuttleSim/reference-model/stsn_attitude_identified.ini')
    ap.add_argument('--report', type=Path, required=True)
    args = ap.parse_args()
    with args.log.open() as f:
        header=json.loads(next(f))
    if header.get('vessel') != 'STS':
        ap.error('archived source must have exact vessel identity STS')
    scenario=dict(line.strip().split('=', 1) for line in args.scenario.read_text().splitlines()
                  if '=' in line and not line.startswith('#'))
    start_ut=float(scenario['ut0'])
    rows=[r for r in decode_log(args.log, 0) if r['ut'] >= start_ut]
    if not rows or abs(rows[0]['ut']-start_ut) > .1:
        ap.error('scenario must start at an archived sample within 0.1 s')
    duration=rows[-1]['ut']-start_ut
    atmosphere=ROOT/'ShuttleSim/reference-model/kerbin_atmosphere_identified.csv'
    attitude=args.attitude
    binary=ROOT/'ShuttleSim/build/shuttlesim'
    report=dict(evidence='offline archived-flight reconstruction; not held-out validation',
                source=identity(args.log), scenario=identity(args.scenario), aero=identity(args.aero),
                atmosphere=identity(atmosphere), borrowed_attitude=identity(attitude),
                simulator=identity(binary), start_ut=start_ut, duration_s=duration,
                attitude_source_status='borrowed STS-N baseline' if attitude.resolve()==
                    (ROOT/'ShuttleSim/reference-model/stsn_attitude_identified.ini').resolve()
                    else 'operator-supplied offline attitude experiment; not certification',
                runs=[])
    with tempfile.TemporaryDirectory(prefix='sts-archive-replay-') as scratch:
        scratch=Path(scratch)
        normalized=scratch/'live.csv'
        with normalized.open('w') as f:
            w=csv.DictWriter(f, fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
        live=load_ksp(normalized, start_ut)
        for mode in ('actual', 'command'):
            replay=scratch/f'{mode}.csv'
            with replay.open('w') as f:
                w=csv.writer(f);w.writerow(['time_s','aoa_deg','bank_deg','gear_down'])
                for r in rows:
                    w.writerow([r['ut']-start_ut,
                                r['aoa_deg'] if mode=='actual' else r['target_aoa_deg'],
                                r['bank_deg'] if mode=='actual' else r['target_roll_deg'],
                                r['gear_down']])
            output=scratch/f'{mode}.jsonl'
            command=[str(binary), '--scenario', str(args.scenario), '--atmosphere', str(atmosphere),
                     '--aero', str(args.aero), '--attitude', str(attitude), '--attitude-replay', str(replay),
                     '--replay-mode', mode, '--max-sim-time', str(duration), '--rate', 'max',
                     '--command-port', '0', '--telemetry-port', '0', '--web-telemetry-port', '0',
                     '--record', str(output), '--quiet']
            subprocess.run(command, check=True, capture_output=True, text=True, timeout=60)
            sim=load_sim(output)
            checkpoints=[]
            for h in (70000, 65000, 60000, 55000, 50000, 45000, 40000, 30000, 20000, 11000):
                lr=nearest(live, 'alt', h); sr=nearest(sim, 't', lr['t'])
                checkpoints.append(dict(live_altitude_m=lr['alt'], time_s=lr['t'],
                                        altitude_error_m=sr['alt']-lr['alt'],
                                        speed_error_mps=sr['speed']-lr['speed'],
                                        longitude_error_deg=dlon(lr['lon'],sr['lon']),
                                        latitude_error_deg=sr['lat']-lr['lat']))
            report['runs'].append(dict(mode=mode, checkpoints=checkpoints,
                                       live_final=live[-1], simulated_final=sim[-1]))
    args.report.write_text(json.dumps(report, indent=2)+'\n')
    print(f'Archived STS actual/command replays completed; report: {args.report}')


if __name__ == '__main__':
    main()
