#!/usr/bin/env python3
"""Offline prescribed-command entry diagnostic; never connects to KSP."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--report',type=Path,required=True)
    ap.add_argument('--reversal-sweep',action='store_true',
                    help='28 deg AoA, alternating +/-40 deg bank every 60/120/180 s')
    ap.add_argument('--incidence-switch-sweep',action='store_true',
                    help='18 to 28 deg at 150/300/450 s, +/-40 bank every 60 s')
    args=ap.parse_args()
    if args.reversal_sweep and args.incidence_switch_sweep:
        ap.error('choose one sweep family')
    binary=ROOT/'ShuttleSim/build/shuttlesim'
    scenario=ROOT/'ShuttleSim/scenarios/sts70km-run9-provisional.ini'
    aero=ROOT/'ShuttleSim/reference-model/experiments/sts_api_terminal_polar.csv'
    atmosphere=ROOT/'ShuttleSim/reference-model/kerbin_atmosphere_identified.csv'
    attitude=ROOT/'ShuttleSim/reference-model/stsn_attitude_identified.ini'
    cfg=json.loads((ROOT/'Configuration/sts.json').read_text())
    report=dict(classification='offline prescribed-command STS entry diagnostic; no controller, admission or landing qualification',status='running',sourceHashes={str(p.relative_to(ROOT)):sha(p) for p in (binary,scenario,aero,atmosphere,attitude,ROOT/'Configuration/sts.json')},runs=[],limits=dict(qPa=cfg['vehicle']['maximumDynamicPressure'],normalLoadG=cfg['vehicle']['maximumGLoad']),limitations=['Borrowed STS-N servo; provisional STS aero and Mach mapping.','Prescribed commands ignore guidance and lateral runway capture; this is not a proposed flight law.','Report to first 22 km descending crossing only; no downstream feasibility or rollout proof.','Published 10 Hz telemetry extrema may miss intersample peaks.'])
    with tempfile.TemporaryDirectory(prefix='sts-entry-sweep-') as directory:
        cases = [(28,40,60,switch) for switch in (150,300,450)] if args.incidence_switch_sweep else (
            [(28,40,period,0) for period in (60,120,180)] if args.reversal_sweep else [
            (alpha,bank,0,0) for alpha in (18,24,28) for bank in (0,40)])
        for alpha,bank,period,switch in cases:
            output=Path(directory)/f'a{alpha}-b{bank}.jsonl'
            cmd=[str(binary),'--scenario',str(scenario),'--aero',str(aero),'--atmosphere',str(atmosphere),'--attitude',str(attitude),'--fixed-aoa',str(alpha),'--fixed-bank',str(bank),'--rate','max','--max-sim-time','900','--command-port','0','--telemetry-port','0','--web-telemetry-port','0','--record',str(output),'--quiet']
            replay = None
            if period:
                replay=Path(directory)/f'bank-reversal-{period}.csv'
                with replay.open('w') as stream:
                    writer=csv.writer(stream)
                    writer.writerow(['time_s','aoa_deg','bank_deg','gear_down'])
                    def command(t):
                        return [t,18 if switch and t<switch else alpha,
                                bank*(-1)**int(t//period),0]
                    writer.writerow(command(0))
                    events=set(range(period,901,period))
                    if switch: events.add(switch)
                    for t in sorted(events):
                        writer.writerow(command(t-.01))
                        writer.writerow(command(t))
                    if 900 not in events: writer.writerow(command(900))
                cmd=cmd[:cmd.index('--fixed-aoa')]+cmd[cmd.index('--rate'):]
                cmd+=['--attitude-replay',str(replay),'--replay-mode','command']
            result=subprocess.run(cmd,capture_output=True,text=True,timeout=60)
            if result.returncode: raise RuntimeError(result.stderr)
            rows=[];crossing=None
            with output.open() as f:
                for line in f:
                    row=json.loads(line)
                    if row.get('type')!='telemetry':continue
                    rows.append(row)
                    if row['position']['altitude_m']<=22000:
                        crossing=row;break
            if not rows:raise RuntimeError('empty simulation')
            maxq=max(r['aero']['q_pa'] for r in rows)
            maxg=max(abs(r['aero']['lift_n'])/r['vehicle']['mass_kg']/9.80665 for r in rows)
            record=dict(alphaCommandDeg=alpha,bankCommandDeg=bank,recordSHA256=sha(output),reached22km=crossing is not None,maximumQPa=maxq,maximumNormalLoadG=maxg,withinPublishedQAndG=maxq<=report['limits']['qPa'] and maxg<=report['limits']['normalLoadG'],lastOrCrossingState=rows[-1])
            record.update(reversalPeriodS=period,initialAlphaDeg=18 if switch else alpha,
                          incidenceSwitchTimeS=switch,replaySHA256=sha(replay) if replay else None)
            report['runs'].append(record)
            args.report.write_text(json.dumps(report,indent=2)+'\n')
            print(alpha,bank,'period',period,'crossed',crossing is not None,'along',rows[-1]['runway']['along_m'],'V',rows[-1]['velocity']['air_mps'],flush=True)
    report['status']='complete'
    args.report.write_text(json.dumps(report,indent=2)+'\n')

if __name__=='__main__':main()
