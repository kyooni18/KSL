#!/usr/bin/env python3
"""Offline A/B experiment; candidate INIs are not production model profiles."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BASE = ROOT / 'Docs/Experiments/STS-Landing-20261010/Codex-Flare-Offline/inputs/att-live.ini'

def main():
    integral = json.loads((HERE.parent/'Pitch-Integral-Crosscheck/result.json').read_text())
    candidates = {'existing': None}
    for flight in ('L4','L7a'):
        candidates[flight.lower()] = next(f for f in integral['crossTraining']
            if f['trainingFlight']==flight and f['coordinate']=='body' and not f['constantControlTorque'])
    for name, fit in candidates.items():
        text = BASE.read_text()+'\n# Experimental body coordinates; unqualified coefficients.\ndirect_body_coordinates=1\n'
        if fit:
            text += '\n'.join([
                f"direct_pitch_accel_per_kpa_deg={fit['controlPerKPa']:.12g}",
                f"direct_pitch_accel_max_deg={fit['controlPerKPa']*fit['capKPa']:.12g}",
                f"direct_pitch_stiffness_per_kpa={fit['stiffnessPerKPa']:.12g}",
                f"direct_trim_aoa_deg={fit['trimDeg']:.12g}",
                f"direct_pitch_damping_ratio={-fit['sqrtQDamping']/(2*math.sqrt(fit['stiffnessPerKPa'])):.12g}",
            ])+'\n'
        (HERE/f'att-body-{name}.ini').write_text(text)
    results = []
    for model in candidates:
        for flight in ('l5','l7'):
            label=f'body-ratefix-{model}-{flight}'
            scenario = (ROOT/'Docs/Experiments/STS-Landing-20261010/L5-72km/mm305-continuous.ini'
                        if flight=='l5' else ROOT/'Docs/Experiments/STS-Landing-20261011/L7-85km-10deg/mm305-taem.ini')
            config = (ROOT/'Docs/Experiments/STS-Landing-20261010/L5-72km/configuration.json'
                      if flight=='l5' else ROOT/'Configuration/sts-live.json')
            args=[sys.executable,str(ROOT/'ShuttleSim/scripts/run_guidance.py'),
                  '--scenario',str(scenario),'--configuration',str(config),
                  '--attitude',str(HERE/f'att-body-{model}.ini'),'--direct-control',
                  '--engage','engageReentry','--max-sim-time','1200','--skip-build',
                  '--no-mirror','--quiet-progress','--label',label]
            env=dict(os.environ,KSP_LANDER_MODEL_PROFILE='sts-live-20261010',KSP_LANDER_ROOT=str(ROOT))
            print('Starting',label,flush=True)
            with (HERE/f'{label}.log').open('w') as out:
                proc=subprocess.run(args,cwd=ROOT,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=600)
            results.append({'label':label,'exitCode':proc.returncode,'args':args})
            (HERE/'runs.json').write_text(json.dumps(results,indent=2)+'\n')
            print('Finished',label,'exit',proc.returncode,flush=True)
if __name__=='__main__':main()
