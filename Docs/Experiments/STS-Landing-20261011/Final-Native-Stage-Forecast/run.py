#!/usr/bin/env python3
"""Passive native-stage forecast evaluated every Final tick on unchanged native L5/L7 tails."""
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sequence = (ROOT / 'CLanding/guidance/final/sequence.inc').read_text()
trace = r'''
    if(height>12.0){
        double nsf_sink0=0,nsf_sink1=0;
        double nsf0=experiment_native_stage_forecast(g,t,v,&flare_plan,height,V,sink_now,lift_div,NAN,0.0,g->terminal_pull_latch,&nsf_sink0);
        double nsf1=experiment_native_stage_forecast(g,t,v,&flare_plan,height,V,sink_now,lift_div,NAN,.5,g->terminal_pull_latch,&nsf_sink1);
        fprintf(stderr,"NSF ut=%.6f h=%.6f x=%.6f V=%.6f sink=%.6f latch=%d plan=%d existing=%.6f native0=%.6f sink0=%.6f native05=%.6f sink05=%.6f\n",
            t->ut,height,along,V,sink_now,g->terminal_pull_latch,g->terminal_preflare_plan_valid,
            g->terminal_predicted_touchdown_speed,nsf0,nsf_sink0,nsf1,nsf_sink1);
    }
'''
forecast = (HERE / 'forecast.inc').read_text()
marker = '#include "adaptive_flare.inc"'
assert sequence.count(marker) == 1
needle = '    const char*lat_diag=getenv("KSP_LANDER_FINAL_LATERAL_DIAGNOSTICS");'
assert sequence.count(needle) == 1
modified = sequence.replace(marker, forecast + '\n' + marker).replace(needle, trace + '\n' + needle)
source = (ROOT / 'CLanding/guidance/guidance_final.c').read_text()
report = {
    'sourceCommit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
    'policy': 'Original native controller/FCS/plant prefix and tail, with passive Final trace. Native-stage forecast printed passively each tick; no command substitution, restart, or live plant qualification. No controller state reset or command substitution.',
    'hashes': {'sequenceOriginal': hashlib.sha256(sequence.encode()).hexdigest(),
               'sequenceInstrumented': hashlib.sha256(modified.encode()).hexdigest()},
    'runs': [],
}
subprocess.run(['make', '-C', 'CLanding'], cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
dry = subprocess.check_output(['make', '-n', '-C', 'CLanding', 'mm305-feasibility-test'], cwd=ROOT, text=True).replace('\\\n', ' ')
fixture = next(shlex.split(s) for s in dry.splitlines() if s.startswith('clang ') and 'MM305FeasibilityPropertyTests.c' in s)
flags = fixture[1:fixture.index('../Validation/MM305FeasibilityPropertyTests.c')] + ['-Iguidance/final']
dry = subprocess.check_output(['make', '-n', '-B', '-C', 'CLanding', 'all'], cwd=ROOT, text=True).replace('\\\n', ' ')
link = next(shlex.split(s) for s in dry.splitlines() if s.startswith('clang ') and s.endswith('-o build/landing_backend'))
prior = json.loads((HERE.parent / 'Final-Warmed-Tail/result.json').read_text())
with tempfile.TemporaryDirectory(prefix='ksl-native-stage-') as temp:
    temp = Path(temp)
    (temp / 'final').mkdir()
    (temp / 'final/sequence.inc').write_text(modified)
    (temp / 'final.c').write_text(source)
    obj = temp / 'final.o'
    binary = temp / 'landing_backend'
    subprocess.run(['clang', *flags, '-c', str(temp / 'final.c'), '-o', str(obj)], cwd=ROOT / 'CLanding', check=True)
    link = [str(obj) if x == 'build/guidance_final.o' else x for x in link]
    link[link.index('-o') + 1] = str(binary)
    subprocess.run(link, cwd=ROOT / 'CLanding', check=True)
    report['binarySHA256'] = hashlib.sha256(binary.read_bytes()).hexdigest()
    for name, ut, scenario, configuration in [
        ('l5', 2462.65, 'Docs/Experiments/STS-Landing-20261010/L5-72km/mm305-continuous.ini', 'Docs/Experiments/STS-Landing-20261010/L5-72km/configuration.json'),
        ('l7', 7021.45, 'Docs/Experiments/STS-Landing-20261011/L7-85km-10deg/mm305-taem.ini', 'Configuration/sts-live.json'),
    ]:
        env = dict(os.environ, KSL_DATA_ROOT=str(ROOT), KSP_LANDER_MODEL_PROFILE='sts-live-20261010', KSP_LANDER_FINAL_TRACE='1', KSL_WARMED_UT=str(ut))
        args = [str(ROOT / '.venv/bin/python'), 'ShuttleSim/scripts/run_guidance.py', '--scenario', scenario, '--configuration', configuration,
                '--attitude', 'Docs/Experiments/STS-Landing-20261010/Codex-Flare-Offline/inputs/att-live.ini', '--engage', 'engageReentry',
                '--direct-control', '--skip-build', '--backend-build-dir', str(temp), '--no-mirror', '--quiet-progress',
                '--label', f'native-stage-{name}', '--max-sim-time', '2400', '--web-telemetry-port', '0']
        print('running', name, flush=True)
        out = subprocess.run(args, cwd=ROOT, env=env, text=True, capture_output=True)
        (HERE / f'{name}-runner.log').write_text(out.stdout + out.stderr)
        result = json.loads(out.stdout.strip().splitlines()[-1])
        previous = next(r for r in prior['runs'] if r['name'] == name)['runResult']
        assert result['simulatorSummary'] == previous['simulatorSummary'], 'passive instrumentation changed flight'
        assert result['success'] == previous['success']
        report['runs'].append({'name': name, 'targetUT': ut, 'runResult': result, 'priorResult': previous,
                               'identicalFlightSummary': True, 'manifest': json.loads(Path(result['manifest']).read_text())})
        (HERE / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        print(name, 'unchanged summary', flush=True)
