#!/usr/bin/env python3
"""Coherent HAC fixture generation and independent certificate reconstruction."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

RADIUS = 600000.0
OMEGA = 2.0 * math.pi / 21549.425
LAT = -0.0486111111
LON = -74.7283333333
UT = 66564.2698140202
ROOT = Path(__file__).resolve().parents[1]


def fixture(path: Path, altitude: float, speed: float, aoa: float, fpa: float) -> None:
    if not all(map(math.isfinite, (altitude, speed, aoa, fpa))) or altitude <= 70 or speed <= 0 or abs(fpa) >= 75:
        raise ValueError('Invalid fixture state')
    lat = math.radians(LAT)
    theta = math.radians(LON) + math.pi / 2 + OMEGA * UT
    up = (math.cos(lat)*math.cos(theta), math.cos(lat)*math.sin(theta), math.sin(lat))
    north = (-math.sin(lat)*math.cos(theta), -math.sin(lat)*math.sin(theta), math.cos(lat))
    position = tuple((RADIUS+altitude)*u for u in up)
    gamma = math.radians(fpa)
    surface = tuple(speed*(-math.cos(gamma)*n+math.sin(gamma)*u) for n, u in zip(north, up))
    rotation = (-OMEGA*position[1], OMEGA*position[0], 0.0)
    velocity = tuple(s+r for s, r in zip(surface, rotation))
    values = dict(name=path.stem, ut0=UT)
    values.update({f'position_{a}_m': x for a, x in zip('xyz', position)})
    values.update({f'velocity_{a}_mps': x for a, x in zip('xyz', velocity)})
    values.update(mass_kg=40252.91796875, initial_aoa_deg=aoa, initial_bank_deg=0,
                  deorbit_delta_v_mps=0, deorbit_duration_s=0, deorbit_delay_s=0,
                  runway_latitude_deg=LAT, runway_longitude_deg=LON,
                  runway_elevation_m=70, runway_heading_deg=90, runway_length_m=2500, runway_width_m=70)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('# Coherent ENU state, heading=180 deg; inertial velocity includes rotation.\n'
                    + f'# altitude={altitude} speed={speed} aoa={aoa} fpa={fpa}\n'
                    + ''.join(f'{k}={v:.17g}\n' if isinstance(v, (int, float)) else f'{k}={v}\n' for k, v in values.items()))


def records(lines: list[str], prefix: str) -> list[dict[str, float]]:
    result = []
    for line in lines:
        if line.startswith(prefix+' '):
            row = {}
            for word in line.split()[1:]:
                key, sep, value = word.partition('=')
                if sep:
                    try:
                        row[key] = float(value)
                    except ValueError:
                        pass
            result.append(row)
    return result


def energy(lat: float, alt: float, speed: float, radius: float, mu: float, omega: float) -> float:
    r = radius + alt
    return speed*speed/2 - mu/r - (omega*r*math.cos(math.radians(lat)))**2/2


def clean(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def summarize_dynamic_candidates(run: Path, lines: list[str], manifest: dict[str, Any], initial: dict[str, Any]) -> dict[str, Any] | None:
    """Audit the fixed-MM305 dynamic selector when legacy ledger records are absent.

    This path intentionally does not claim independent leg-state reconstruction:
    the dynamic selector publishes the certificate components directly on each
    HAC_DYNAMIC_CANDIDATE record.  We verify all three published accounting
    identities and report the mode explicitly so callers cannot confuse this
    with the legacy HAC_EVAL/HAC_LEDGER/HAC_LEG_STATES audit path.
    """
    candidates = records(lines, 'HAC_DYNAMIC_CANDIDATE')
    if not candidates:
        return None
    nominal_residuals = []
    certified_residuals = []
    required_residuals = []
    finite = []
    for row in candidates:
        available = row.get('energyAvailable', math.nan)
        modeled = row.get('modeledWork', math.nan)
        uncertainty = row.get('uncertainty', math.nan)
        nominal = row.get('nominalMargin', math.nan)
        certified = row.get('energyMargin', math.nan)
        required = row.get('energyRequired', math.nan)
        if all(math.isfinite(v) for v in (available, modeled, nominal)):
            nominal_residuals.append((available-modeled)-nominal)
        if all(math.isfinite(v) for v in (nominal, uncertainty, certified)):
            certified_residuals.append((nominal-uncertainty)-certified)
        if all(math.isfinite(v) for v in (modeled, uncertainty, required)):
            required_residuals.append((modeled+uncertainty)-required)
        if math.isfinite(certified):
            finite.append(row)
    best = max(finite, key=lambda r: r['energyMargin']) if finite else None
    initial.pop('sim_rate', None)
    initial.pop('scenario', None)
    summary = dict(
        run=run.name,
        certificateMode='dynamic-candidate-identity',
        evaluations=len(candidates),
        dynamicCandidates=len(candidates),
        expectedAudits=0,
        actualAudits=0,
        uniqueAudits=0,
        finiteCertificates=len(finite),
        legAudits=0,
        state=manifest.get('state'),
        engagementRejected=manifest.get('engagementRejected', False),
        initializationRejections=[x for x in lines if x.startswith('HAC_INIT_REJECT ')],
        positiveCandidates=sum(int(r.get('pass', 0) > 0) for r in candidates),
        maxNominalIdentityResidual=max((abs(x) for x in nominal_residuals), default=None),
        maxCertifiedIdentityResidual=max((abs(x) for x in certified_residuals), default=None),
        maxRequiredIdentityResidual=max((abs(x) for x in required_residuals), default=None),
        best=best,
        firstEvaluation=candidates[0] if candidates else None,
        initialPosition=initial.get('runway'),
        initialVelocity=initial.get('velocity'),
        initialHeading=(initial.get('attitude') or {}).get('heading_deg'),
        initialAoA=(initial.get('attitude') or {}).get('aoa_deg'),
        initialFingerprint=hashlib.sha256(json.dumps(initial, sort_keys=True).encode()).hexdigest(),
        guidanceFingerprint=hashlib.sha256((run/'guidance.log').read_bytes()).hexdigest(),
    )
    report = dict(summary=summary, dynamicCandidates=candidates)
    (run/'certificate-audit.json').write_text(json.dumps(clean(report), indent=2, allow_nan=False)+'\n')
    return clean(summary)


def summarize(run: Path) -> dict[str, Any]:
    lines = (run/'guidance.log').read_text().splitlines()
    evaluations = records(lines, 'HAC_EVAL')
    ledgers = records(lines, 'HAC_LEDGER')
    audits = records(lines, 'HAC_ENERGY_AUDIT')
    legs = records(lines, 'HAC_LEG_STATES')
    leg_by_key = {(r['evalId'],r['candidateId']):r for r in legs}
    audit_by_key = {(r['id'],r['candidateId']):r for r in audits}
    manifest = json.loads((run/'manifest.json').read_text())
    with (run/'simulator-telemetry.jsonl').open() as f:
        initial = json.loads(next(f))
    initial.pop('sim_rate', None)
    initial.pop('scenario', None)  # provenance label, not a physical state
    dynamic_summary = summarize_dynamic_candidates(run, lines, manifest, initial.copy()) if not evaluations else None
    if dynamic_summary is not None:
        return dynamic_summary
    world = initial['world']
    omega = world['rotation_rate_rad_s']
    maximum_residual = 0.0
    reference_residual = 0.0
    work_residuals = []
    uncertainty_residuals = []
    for row in ledgers:
        independently_available = energy(row['liveLat'], row['liveAlt'], row['liveV'],
            row['planetR'], row['mu'], omega) - energy(row['gateLat'], row['gateAlt'],
            row['gateV'], row['planetR'], row['mu'], omega)
        reconstructed = independently_available - sum(row[k] for k in
            ('leadWork','closureCorrection','arcWork','finalWork','effectiveUncertainty'))
        residual = reconstructed-row['certifiedMargin']
        row['independentAvailable'] = independently_available
        row['identityResidual'] = residual
        if math.isfinite(residual):
            maximum_residual = max(maximum_residual, abs(residual))
        reference_residual = max(reference_residual, abs(independently_available-row['liveAvailable']))
        key = (row['evalId'],row['candidateId'])
        leg = leg_by_key.get(key)
        a = audit_by_key.get(key)
        if leg and math.isfinite(row['certifiedMargin']):
            e_live = energy(row['liveLat'],row['liveAlt'],row['liveV'],row['planetR'],row['mu'],omega)
            e_p0 = energy(leg['p0Lat'],leg['p0Alt'],leg['p0V'],row['planetR'],row['mu'],omega)
            e_arc = energy(leg['p0Lat'],leg['arcEndAlt'],leg['arcEndV'],row['planetR'],row['mu'],omega)
            e_end = energy(leg['p0Lat'],leg['finalEndAlt'],leg['finalEndV'],row['planetR'],row['mu'],omega)
            residuals = [e_live-e_p0-row['leadWork'], e_p0-e_arc-row['arcWork'], e_arc-e_end-row['finalWork']]
            row['independentLegWorkResiduals'] = residuals
            work_residuals.extend(abs(x) for x in residuals)
        if a and math.isfinite(row['certifiedMargin']):
            dv = abs(a['speedResidual'])
            uncertainty = (row['mu']/row['planetR']**2*abs(a['altResidual']) +
                abs(a['p0Speed'])*dv+dv*dv/2 + a['modeledWork']/a['totalPath']*abs(a['rangeResidual']) +
                a['modeledWork']*a['physicsRel'])
            uncertainty_residuals.append(abs(uncertainty-row['effectiveUncertainty']))
    finite = [r for r in ledgers if math.isfinite(r['certifiedMargin'])]
    best = max(finite, key=lambda r: r['certifiedMargin']) if finite else None
    audit_keys = {(int(r['id']), int(r['candidateId'])) for r in audits}
    expected_rows = int(sum(r['authorityValid'] for r in evaluations))
    summary = dict(run=run.name, evaluations=len(evaluations), expectedAudits=expected_rows,
                   actualAudits=len(audits), uniqueAudits=len(audit_keys), finiteCertificates=len(finite),
                   state=manifest.get('state'), engagementRejected=manifest.get('engagementRejected',False),
                   initializationRejections=[x for x in lines if x.startswith('HAC_INIT_REJECT ')],
                   legAudits=len(legs), maxIndependentWorkResidual=max(work_residuals,default=None),
                   maxIndependentUncertaintyResidual=max(uncertainty_residuals,default=None),
                   maxIdentityResidual=maximum_residual if finite else None,
                   maxReferenceResidual=reference_residual if ledgers else None,
                   maxLeadPhysicalClosure=max((abs(r['leadEnergyClosure']) for r in audits if math.isfinite(r.get('leadEnergyClosure', math.nan))), default=None),
                   maxArcFinalPhysicalClosure=max((abs(r['closure']) for r in audits if math.isfinite(r.get('closure', math.nan))), default=None),
                   positiveCandidates=sum(r['energyValid'] for r in evaluations),
                   best=best, bestAudit=audit_by_key.get((best['evalId'],best['candidateId'])) if best else None,
                   firstEvaluation=evaluations[0] if evaluations else None,
                   initialPosition=initial['runway'], initialVelocity=initial['velocity'],
                   initialHeading=initial['attitude']['heading_deg'], initialAoA=initial['attitude']['aoa_deg'],
                   initialFingerprint=hashlib.sha256(json.dumps(initial, sort_keys=True).encode()).hexdigest(),
                   guidanceFingerprint=hashlib.sha256((run/'guidance.log').read_bytes()).hexdigest())
    report = dict(summary=summary, evaluations=evaluations, audits=audits, ledgers=ledgers)
    (run/'certificate-audit.json').write_text(json.dumps(clean(report), indent=2, allow_nan=False)+'\n')
    return clean(summary)


def campaign(label: str, axis: str, values: list[float], *, altitude: float=4500,
             speed: float=150, aoa: float=10, fpa: float=0) -> None:
    out = ROOT/'Runtime/HACCertifiedInvestigation-20260922'/label
    out.mkdir(parents=True, exist_ok=False)
    tracked = ['CLanding/guidance/guidance.c','CLanding/build/landing_backend',
               'ShuttleSim/src/main.c','ShuttleSim/build/shuttlesim',
               'Tools/simulator_campaign.py','Tools/hac_certified_audit.py','Configuration/default.json']
    def hashes() -> dict[str,str]:
        return {p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in tracked}
    frozen = hashes()
    (out/'provenance.json').write_text(json.dumps(frozen,indent=2)+'\n')
    env = os.environ.copy()
    for key in ('KSP_LANDER_HAC_TEST_GLIDE_SLOPE_DEG', 'KSP_LANDER_HAC_JOIN_RATE_RATIO_LIMIT'):
        env.pop(key, None)
    env.update(KSP_LANDER_HAC_DIAGNOSTICS='1', KSP_LANDER_HAC_ENERGY_AUDIT='1',
               KSP_LANDER_HAC_DEBUG_FORCE_CAPTURE='0', KSP_LANDER_HAC_PROVISIONAL_NOMINAL='0',
               KSP_LANDER_HAC_FUTURE_PREVIEW='0')
    results = []
    for index, value in enumerate(values):
        if hashes() != frozen:
            raise RuntimeError('Frozen source/binary/config changed during campaign')
        params = dict(altitude=altitude, speed=speed, aoa=aoa, fpa=fpa)
        params[axis] = value
        run_id = f'{label}-{index:02d}'
        scenario = out/f'{run_id}.ini'
        fixture(scenario, **params)
        with (out/f'{run_id}.stdout').open('w') as log:
            result = subprocess.run([sys.executable, str(ROOT/'Tools/simulator_campaign.py'),
                '--no-build', '--engage', 'hac-upstream', '--scenario', str(scenario),
                '--preroll-seconds', '0', '--max-sim-time', '3.3', '--wall-timeout', '25',
                '--run-id', run_id], env=env, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, timeout=50)
        if hashes() != frozen:
            raise RuntimeError(f'{run_id}: frozen source/binary/config changed during run')
        if result.returncode not in (0, 2):
            raise RuntimeError(f'{run_id}: unexpected exit {result.returncode}; see stdout')
        summary = summarize(ROOT/'ShuttleSim/runs'/run_id)
        summary['parameters'] = params
        results.append(summary)
        (out/'summary.json').write_text(json.dumps(results, indent=2, allow_nan=False)+'\n')
        best = summary['best'] or {}
        print(f'{axis}={value:g} evals={summary["evaluations"]} audit={summary["actualAudits"]}/{summary["expectedAudits"]} '
              f'positive={summary["positiveCandidates"]} best={best.get("certifiedMargin")} '
              f'residual={summary["maxIdentityResidual"]}', flush=True)
        if summary['actualAudits'] != summary['expectedAudits'] or summary['uniqueAudits'] != summary['expectedAudits']:
            raise AssertionError(f'{run_id}: incomplete or duplicated candidate audit')
        if summary['legAudits'] != summary['expectedAudits']:
            raise AssertionError(f'{run_id}: missing independent leg states')
        if not summary['engagementRejected'] and summary['evaluations'] < 3:
            raise AssertionError(f'{run_id}: fewer than three planner evaluations')
        for key in ('maxIndependentWorkResidual','maxIndependentUncertaintyResidual'):
            if summary[key] is not None and summary[key] >= 1.0:
                raise AssertionError(f'{run_id}: {key} does not close below 1 J/kg')
        if summary['maxIdentityResidual'] is not None and summary['maxIdentityResidual'] >= 1.0:
            raise AssertionError(f'{run_id}: energy identity does not close')
        if abs(summary['initialHeading']-180) > .0001 or abs(summary['initialPosition']['along_m']) > .001 or abs(summary['initialPosition']['cross_m']) > .001:
            raise AssertionError(f'{run_id}: incoherent initial position/heading')


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest='command', required=True)
    gen = sub.add_parser('fixture')
    gen.add_argument('path', type=Path)
    for name, default in [('altitude',4500),('speed',150),('aoa',10),('fpa',0)]:
        gen.add_argument('--'+name, type=float, default=default)
    report = sub.add_parser('summarize')
    report.add_argument('runs', type=Path, nargs='+')
    report.add_argument('--output', type=Path)
    sweep = sub.add_parser('sweep')
    sweep.add_argument('--label', required=True)
    sweep.add_argument('--axis', choices=('altitude','speed','aoa','fpa'), required=True)
    sweep.add_argument('values', type=float, nargs='+')
    for name, default in [('altitude',4500),('speed',150),('aoa',10),('fpa',0)]:
        sweep.add_argument('--'+name, type=float, default=default)
    args = ap.parse_args()
    if args.command == 'fixture':
        fixture(args.path, args.altitude, args.speed, args.aoa, args.fpa)
    elif args.command == 'sweep':
        campaign(args.label, args.axis, args.values, altitude=args.altitude,
                 speed=args.speed, aoa=args.aoa, fpa=args.fpa)
    else:
        results = [summarize(run) for run in args.runs]
        text = json.dumps(results, indent=2, allow_nan=False)+'\n'
        if args.output:
            args.output.write_text(text)
        print(text)

if __name__ == '__main__':
    main()
