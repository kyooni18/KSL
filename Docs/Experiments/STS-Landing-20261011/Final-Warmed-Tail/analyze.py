#!/usr/bin/env python3
"""Assert unchanged native histories, then compare force models on their warmed tails."""
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
report = json.loads((HERE / 'result.json').read_text())


def packets(path):
    with gzip.open(path, 'rt') as f:
        return [json.loads(s) for s in f if s.strip()]


def trace(path, prefix):
    return [{k: v if k == 'g' else float(v) for k, v in re.findall(r'(\w+)=([^\s]+)', line)}
            for line in Path(path).read_text().splitlines() if line.startswith(prefix)]


def work(rows, field):
    rates = [r[field] * r['V'] / r['mass'] for r in rows]
    return sum(.5 * (ra + rb) * (b['ut'] - a['ut'])
               for a, b, ra, rb in zip(rows, rows[1:], rates, rates[1:]))


for run in report['runs']:
    result, prior = run['runResult'], run['priorResult']
    physical, previous = packets(result['sim']), packets(prior['sim'])
    # sim_rate is wall-clock pacing; every published physical/control field is compared.
    normalized = [{k: v for k, v in p.items() if k != 'sim_rate'} for p in physical]
    prior_normalized = [{k: v for k, v in p.items() if k != 'sim_rate'} for p in previous]
    assert normalized == prior_normalized, 'native actuator/state/contact history changed'
    current_final = [{k: v for k, v in r.items() if k != 'g'} for r in trace(result['backendStderr'], 'FINAL_TRACE')]
    prior_final = [{k: v for k, v in r.items() if k != 'g'} for r in trace(prior['backendStderr'], 'FINAL_TRACE')]
    # Post-latch forecasts intentionally contain NaN; compare its serialized token.
    assert json.dumps(current_final, sort_keys=True) == json.dumps(prior_final, sort_keys=True), 'Final commands/plan history changed'
    rows = trace(result['backendStderr'], 'WARMED_TAIL')
    assert rows and len({r['g'] for r in rows}) == 1
    assert all(b['ut'] > a['ut'] for a, b in zip(rows, rows[1:]))
    contact = next(p for p in physical if p['ground']['touchdown_seen'])
    rows = [r for r in rows if r['ut'] < contact['ut']]
    assert abs(rows[0]['ut'] - run['targetUT']) < .015
    native = [p for p in physical if rows[0]['ut'] <= p['ut'] <= contact['ut']]
    # Guidance consumes the published state, so these should match at the same UT.
    by_ut = {p['ut']: p for p in physical}
    for r in rows:
        p = by_ut[r['ut']]
        assert abs(r['V'] - p['velocity']['air_mps']) < 1e-6
        assert abs(r['aoa'] - p['attitude']['aoa_deg']) < 1e-6
        assert abs(r['drag'] - p['aero']['drag_n']) < 1e-6
    ranges = {key: {'minimum': min(r[key] for r in rows), 'maximum': max(r[key] for r in rows)}
              for key in ['arrest', 'response', 'liftRatio', 'cmdAoa', 'aoa', 'verticalAccel', 'actualBank']}
    first_latch = next(r for r in rows if r['latch'])
    first_sgs = next(r for r in rows if r['h'] <= 12)
    first_arc = next(r for r in rows if r['latch'] and math.isfinite(r['arcAccel']))
    checkpoints = [rows[0], first_latch, first_sgs, rows[-1]]
    previous_time = -math.inf
    for r in rows:
        if r['ut'] - previous_time >= 2:
            checkpoints.append(r)
            previous_time = r['ut']
    checkpoints = sorted({r['ut']: r for r in checkpoints}.values(), key=lambda r: r['ut'])
    with (HERE / f"{run['name']}-tail.csv").open('w') as f:
        w = csv.DictWriter(f, fieldnames=[k for k in rows[0] if k != 'g'])
        w.writeheader()
        w.writerows({k: v for k, v in r.items() if k != 'g'} for r in checkpoints)
    rates = [p['aero']['drag_n'] * p['velocity']['air_mps'] / p['vehicle']['mass_kg'] for p in native]
    native_work = sum(.5 * (ra + rb) * (b['ut'] - a['ut'])
                      for a, b, ra, rb in zip(native, native[1:], rates, rates[1:]))
    measured_work, learned_work, frozen_work = [work(rows, f) for f in ['drag', 'liveDrag', 'seedDrag']]
    run['analysis'] = {
        'parity': {'allPhysicalTelemetryAndActuatorFieldsExceptWallPacingExact': True,
                   'physicalSamples': len(physical), 'allFinalTraceFieldsExceptPointerExact': True,
                   'finalTraceSamples': len(current_final)},
        'tailTraceSamples': len(rows), 'seed': {k: v for k, v in rows[0].items() if k != 'g'},
        'contactUT': contact['ut'], 'firstLatch': {k: v for k, v in first_latch.items() if k != 'g'},
        'firstShallowGlideHeight': {k: v for k, v in first_sgs.items() if k != 'g'},
        'firstFiniteArcDemand': {k: v for k, v in first_arc.items() if k != 'g'},
        'latchedAbove12mArcInactiveSamples': sum(bool(r['latch'] and r['h'] > 12 and not math.isfinite(r['arcAccel'])) for r in rows),
        'nativeTimeToContact': contact['ut'] - rows[0]['ut'],
        'nativeTimeBelow12m': contact['ut'] - first_sgs['ut'],
        'sampledParameterRanges': ranges,
        'nativeDragWorkThroughContactJPerKg': native_work,
        'commonTraceIntervalSeconds': rows[-1]['ut'] - rows[0]['ut'],
        'commonTraceMeasuredDragWorkJPerKg': measured_work,
        'commonTraceCurrentLearnedDragWorkJPerKg': learned_work,
        'commonTraceSeedLearnedDragWorkJPerKg': frozen_work,
        'currentModelWorkBiasPercent': 100 * (learned_work / measured_work - 1),
        'frozenModelWorkBiasPercent': 100 * (frozen_work / measured_work - 1),
        'modelWorkPolicy': 'Trapezoidal D*V/m on actual future native states/incidences at guidance cadence; frozen force arrays only. No counterfactual trajectory or causal speed attribution.',
    }
    run['evidenceHashes'] = {key: hashlib.sha256(Path(result[key]).read_bytes()).hexdigest()
                            for key in ['sim', 'backendStderr', 'manifest']}
    print(run['name'], json.dumps(run['analysis']), flush=True)


def clean(value):
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, list):
        return [clean(v) for v in value]
    return None if isinstance(value, float) and not math.isfinite(value) else value


(HERE / 'result.json').write_text(json.dumps(clean(report), indent=2, allow_nan=False) + '\n')
