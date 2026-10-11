#!/usr/bin/env python3
"""Passive learned-force and controller trace on unchanged continuous native tails."""
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
    /* Force-only snapshot: terminal_aero_q reads scalar areas and fixed arrays.
       No pointer from this shallow copy is dereferenced or modified here. */
    static GuidanceMachine experiment_force_seed;
    static const GuidanceMachine*experiment_owner;
    const char*experiment_target=getenv("KSL_WARMED_UT");
    double experiment_ut=experiment_target?strtod(experiment_target,NULL):INFINITY;
    if(!experiment_owner&&fabs(t->ut-experiment_ut)<.015){
        experiment_force_seed=*g;experiment_owner=g;
    }
    if(experiment_owner==g){
        double liveL=0,liveD=0,seedL=0,seedD=0,commandL=0,commandD=0;
        terminal_aero_q(g,t,v,t->angle_of_attack,&liveL,&liveD);
        terminal_aero_q(&experiment_force_seed,t,v,t->angle_of_attack,&seedL,&seedD);
        terminal_aero_q(&experiment_force_seed,t,v,r.command.target_aoa,&commandL,&commandD);
        fprintf(stderr,"WARMED_TAIL g=%p ut=%.6f h=%.6f x=%.6f V=%.6f sink=%.6f aoa=%.6f aoaRate=%.6f cmdAoa=%.6f rawAoa=%.6f actualBank=%.6f cmdBank=%.6f latch=%d plan=%d trigger=%.6f usableTrigger=%.6f response=%.6f arrest=%.6f liftRatio=%.6f seedLiftRatio=%.6f refFpa=%.6f glideFpa=%.6f verticalAccel=%.6f arcAccel=%.6f aim=%.6f q=%.6f mass=%.6f lift=%.6f drag=%.6f liveLift=%.6f liveDrag=%.6f seedLift=%.6f seedDrag=%.6f seedCommandDrag=%.6f long=%d\n",
            (void*)g,t->ut,height,along,V,sink_now,t->angle_of_attack,t->angle_of_attack_rate,
            r.command.target_aoa,aoa,t->roll,r.command.target_roll,g->terminal_pull_latch,
            g->terminal_preflare_plan_valid,g->preflare_trigger_altitude,planned_preflare_trigger,
            response,arrest_accel,g->terminal_lift_ratio,experiment_force_seed.terminal_lift_ratio,
            reference_fpa*RAD2DEG,glide_fpa*RAD2DEG,vertical_accel,arc_accel,
            g->terminal_aim_along,t->dynamic_pressure,t->mass,t->lift_force,t->drag_force,
            liveL*t->dynamic_pressure,liveD*t->dynamic_pressure,
            seedL*t->dynamic_pressure,seedD*t->dynamic_pressure,
            commandD*t->dynamic_pressure,long_landing);
    }
'''
needle = '    const char*lat_diag=getenv("KSP_LANDER_FINAL_LATERAL_DIAGNOSTICS");'
assert sequence.count(needle) == 1
modified = sequence.replace(needle, trace + '\n' + needle)
source = (ROOT / 'CLanding/guidance/guidance_final.c').read_text()
report = {
    'sourceCommit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
    'policy': 'Original native controller/FCS/plant prefix and tail, with passive Final trace. Frozen force-only learned arrays evaluated on actual future states; not a forecast, restart, or live plant qualification. No controller state reset or command substitution.',
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
prior = json.loads((HERE.parent / 'Frozen-Flare-Response/result.json').read_text())
with tempfile.TemporaryDirectory(prefix='ksl-warmed-tail-') as temp:
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
                '--label', f'warmed-tail-{name}', '--max-sim-time', '2400', '--web-telemetry-port', '0']
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
