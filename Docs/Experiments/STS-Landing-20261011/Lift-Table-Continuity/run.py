#!/usr/bin/env python3
"""Scratch-backend variants anchoring the analytic lift/drag fallback to the nearest learned node (not production)."""
import json, os, shlex, subprocess, tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sequence = (ROOT / 'CLanding/guidance/final/sequence.inc').read_text()
source = (ROOT / 'CLanding/guidance/guidance_final.c').read_text()
old = """    double lf=0.0,df=0.0;
    aerodynamic_force_factors_mach(t->mach,aoa,v,&lf,&df);
    *L=(g->terminal_lift_area_ema>0.0?g->terminal_lift_area_ema:100.0)*lf;
    *D=(g->terminal_drag_area_ema>0.0?g->terminal_drag_area_ema:50.0)*df;
}"""
assert sequence.count(old) == 1
new = """    double lf=0.0,df=0.0;
    aerodynamic_force_factors_mach(t->mach,aoa,v,&lf,&df);
    double la=g->terminal_lift_area_ema>0.0?g->terminal_lift_area_ema:100.0;
    double da=g->terminal_drag_area_ema>0.0?g->terminal_drag_area_ema:50.0;
    *L=la*lf;*D=da*df;
    /* Continuity: anchor the analytic fallback to the nearest learned node. */
    int near=-1;double best=1e9;
    for(int k=0;k<=20;++k)
        if(g->terminal_aero_seen[k]&&fabs(t->mach-g->terminal_aero_mach[k])<=.06&&fabs(k-aoa)<best){best=fabs(k-aoa);near=k;}
    if(near>=0&&best<=FADE_LIMIT){
        double lfn=0.0,dfn=0.0;
        aerodynamic_force_factors_mach(t->mach,(double)near,v,&lfn,&dfn);
        if(lfn>1e-6&&dfn>1e-6){
            double w=exp(-best/FADE_SCALE);
            double rl=clampd(g->terminal_lift_q[near]/(la*lfn),.5,2.0);
            double rd=clampd(g->terminal_drag_q[near]/(da*dfn),.5,2.0);
            *L*=1.0+(rl-1.0)*w;*D*=1.0+(rd-1.0)*w;
        }
    }
}"""
subprocess.run(['make', '-C', 'CLanding'], cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
dry = subprocess.check_output(['make', '-n', '-C', 'CLanding', 'mm305-feasibility-test'], cwd=ROOT, text=True).replace('\\\n', ' ')
fixture = next(shlex.split(s) for s in dry.splitlines() if s.startswith('clang ') and 'MM305FeasibilityPropertyTests.c' in s)
flags = fixture[1:fixture.index('../Validation/MM305FeasibilityPropertyTests.c')] + ['-Iguidance/final']
dry = subprocess.check_output(['make', '-n', '-B', '-C', 'CLanding', 'all'], cwd=ROOT, text=True).replace('\\\n', ' ')
link0 = next(shlex.split(s) for s in dry.splitlines() if s.startswith('clang ') and s.endswith('-o build/landing_backend'))
cases = [('l5', 'Docs/Experiments/STS-Landing-20261010/L5-72km/mm305-continuous.ini', 'Docs/Experiments/STS-Landing-20261010/L5-72km/configuration.json'),
         ('l7', 'Docs/Experiments/STS-Landing-20261011/L7-85km-10deg/mm305-taem.ini', 'Configuration/sts-live.json')]
# floor on the arc push-over (m/s^2, negative = allowed push); baseline is the native -0.3 g clamp.
M = 'Docs/Experiments/STS-Landing-20261011/Measured-State-Replay/'
CFG5 = 'Docs/Experiments/STS-Landing-20261010/L5-72km/configuration.json'
ATT = 'Docs/Experiments/STS-Landing-20261011/Body-Coordinate-Rate-Fix/att-body-l4.ini'
if os.environ.get('LIFTCONT_MATRIX'):
    cases = [(f'{n}-matrix', M + f'{n}-mm305.ini', 'Configuration/sts-live.json' if n == 'l7' else CFG5) for n in ('l5', 'l6', 'l7', 'high')]
    variants = {'baseline': None, 'fade1': (1.0, 3.0), 'fade2': (2.0, 6.0), 'fade4': (4.0, 20.0)}
else:
    variants = {'baseline': None, 'fade1': (1.0, 3.0), 'fade2': (2.0, 6.0), 'fade4': (4.0, 20.0)}
results = {}
with tempfile.TemporaryDirectory(prefix='ksl-liftcont-') as temp:
    temp = Path(temp)
    for vname, floor in variants.items():
        text = sequence if floor is None else sequence.replace(old, new.replace('FADE_SCALE', str(floor[0])).replace('FADE_LIMIT', str(floor[1])))
        d = temp / vname
        (d / 'final').mkdir(parents=True)
        (d / 'final/sequence.inc').write_text(text)
        (d / 'final.c').write_text(source)
        subprocess.run(['clang', *flags, '-c', str(d / 'final.c'), '-o', str(d / 'final.o')], cwd=ROOT / 'CLanding', check=True)
        link = [str(d / 'final.o') if x == 'build/guidance_final.o' else x for x in link0]
        link[link.index('-o') + 1] = str(d / 'landing_backend')
        subprocess.run(link, cwd=ROOT / 'CLanding', check=True)
        for name, scenario, configuration in cases:
            env = dict(os.environ, KSL_DATA_ROOT=str(ROOT), KSP_LANDER_MODEL_PROFILE='sts-live-20261010')
            args = [str(ROOT / '.venv/bin/python'), 'ShuttleSim/scripts/run_guidance.py', '--scenario', scenario, '--configuration', configuration,
                    '--attitude', ATT if os.environ.get('LIFTCONT_MATRIX') else 'Docs/Experiments/STS-Landing-20261010/Codex-Flare-Offline/inputs/att-live.ini', '--engage', 'engageReentry',
                    '--direct-control', '--skip-build', '--backend-build-dir', str(d), '--no-mirror', '--quiet-progress',
                    '--label', f'liftcont-{vname}-{name}', '--max-sim-time', '2400', '--web-telemetry-port', '0']
            out = subprocess.run(args, cwd=ROOT, env=env, text=True, capture_output=True)
            r = json.loads(out.stdout.strip().splitlines()[-1])
            s = r['simulatorSummary']
            results[f'{vname}/{name}'] = {k: s.get(k) for k in ('touchdown_speed_mps', 'touchdown_sink_mps', 'touchdown_along_m', 'stop_along_m', 'bounce_count', 'crashed', 'on_runway', 'touchdown_gear')} | {'success': r['success']}
            print(vname, name, results[f'{vname}/{name}'], flush=True)
(HERE / ('matrix-result.json' if os.environ.get('LIFTCONT_MATRIX') else 'result.json')).write_text(json.dumps(results, indent=2) + '\n')
