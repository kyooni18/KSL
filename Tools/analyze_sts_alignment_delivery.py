#!/usr/bin/env python3
"""Summarize unchanged Final admission criteria in offline TAEM replay logs."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('log', type=Path)
    ap.add_argument('--configuration', type=Path, default=Path('Configuration/sts.json'))
    ap.add_argument('--report', type=Path, required=True)
    args = ap.parse_args()
    text = args.log.read_text()
    cfg = json.loads(args.configuration.read_text())
    station = -cfg['guidance']['finalApproachDistance']
    limits = dict(cross=cfg['site']['runwayWidth']/2, hdgErr=2, bank=5,
                  bankRate=3, dh=150, dV=10, dFpa=3)
    strict = {'cross', 'hdgErr', 'bank', 'bankRate'}
    samples = []
    for line in text.splitlines():
        if not line.startswith('TAEM alignment station sample:'):
            continue
        values = {k: float(v) for k, v in re.findall(r'([A-Za-z]+)=([+\-\d.]+)', line)}
        if abs(values['along']-station) > 250:
            continue
        failures = [k for k, limit in limits.items() if abs(values[k]) > limit or
                    (k in strict and abs(values[k]) == limit)]
        values['failedCriteria'] = failures
        values['maximumConstraintRatio'] = max(abs(values[k])/limit for k, limit in limits.items())
        samples.append(values)
    report = dict(classification='offline replay Final admission diagnosis; no gate changes',
        logSHA256=hashlib.sha256(args.log.read_bytes()).hexdigest(),
        configurationSHA256=hashlib.sha256(args.configuration.read_bytes()).hexdigest(),
        alignmentSourceSHA256=hashlib.sha256(Path('CLanding/include/taem_alignment.h').read_bytes()).hexdigest(),
        stationM=station, stationWindowM=250, limits=limits,
        stationWindowSamples=len(samples), allCriteriaReadyCount=sum(not s['failedCriteria'] for s in samples),
        criterionFailureCounts=dict(Counter(k for s in samples for k in s['failedCriteria'])),
        closestSamples=sorted(samples,key=lambda s:s['maximumConstraintRatio'])[:8],
        contractLines=[l for l in text.splitlines() if l.startswith('MM305 downstream-feasibility contract:')],
        limitations=['Logged values are rounded; boundary classification uses displayed precision.',
                     'Repeated candidate station samples are not independent flights.',
                     'Profile screening tolerance differs from actual Final readiness; neither is modified.'])
    args.report.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('stationWindowSamples','allCriteriaReadyCount','criterionFailureCounts')}))


if __name__ == '__main__':
    main()
