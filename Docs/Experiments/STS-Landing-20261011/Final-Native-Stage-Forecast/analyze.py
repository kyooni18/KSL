#!/usr/bin/env python3
"""Assert unchanged native histories, then score forecasts against the flown touchdown speed."""
import gzip, json, math, re
from pathlib import Path

HERE = Path(__file__).resolve().parent
report = json.loads((HERE / 'result.json').read_text())


def packets(path):
    with gzip.open(path, 'rt') as f:
        return [json.loads(s) for s in f if s.strip()]


def trace(path, prefix):
    return [{k: v if k == 'g' else float(v) for k, v in re.findall(r'(\w+)=([^\s]+)', line)}
            for line in Path(path).read_text().splitlines() if line.startswith(prefix)]


out = {}
for run in report['runs']:
    result, prior = run['runResult'], run['priorResult']
    a = [{k: v for k, v in p.items() if k != 'sim_rate'} for p in packets(result['sim'])]
    b = [{k: v for k, v in p.items() if k != 'sim_rate'} for p in packets(prior['sim'])]
    assert a == b, 'native history changed'
    cur = [{k: v for k, v in r.items() if k != 'g'} for r in trace(result['backendStderr'], 'FINAL_TRACE')]
    old = [{k: v for k, v in r.items() if k != 'g'} for r in trace(prior['backendStderr'], 'FINAL_TRACE')]
    assert json.dumps(cur, sort_keys=True) == json.dumps(old, sort_keys=True), 'Final trace changed'
    actual = result['simulatorSummary']['touchdown_speed_mps']
    rows = trace(result['backendStderr'], 'NSF')
    assert rows
    pre = [r for r in rows if not r['latch']]
    post = [r for r in rows if r['latch']]
    def err(rs, key):
        v = [r[key] - actual for r in rs if math.isfinite(r[key])]
        return {'count': len(v), 'of': len(rs), 'first': v[0] if v else None, 'last': v[-1] if v else None,
                'meanAbs': sum(map(abs, v)) / len(v) if v else None}
    out[run['name']] = {'actualTouchdownSpeed': actual, 'packets': len(a), 'finalTraceRecords': len(cur),
        'prelatch': {k: err(pre, k) for k in ('existing', 'native0', 'native05')},
        'postlatch': {k: err(post, k) for k in ('existing', 'native0', 'native05')}}
    with (HERE / f"{run['name']}-forecast.csv").open('w') as f:
        f.write('ut,h,V,sink,latch,plan,existing,native_lag0,native_lag05\n')
        for r in rows:
            f.write(f"{r['ut']},{r['h']},{r['V']},{r['sink']},{int(r['latch'])},{int(r['plan'])},{r['existing']},{r['native0']},{r['native05']}\n")
(HERE / 'analysis.json').write_text(json.dumps(out, indent=2) + '\n')
print(json.dumps(out, indent=1))
