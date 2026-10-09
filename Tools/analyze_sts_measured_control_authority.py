#!/usr/bin/env python3
"""Offline diagnostic of archived STS aerodynamic control and rate matrices.

Uses hypothetical KSP wrench responses, not measured actuator effectiveness.
Linear cancellation estimates are deliberately checked against compound-input
wrenches; near-singular results must not be promoted into control gains.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'Docs/Experiments/STS-Entry-20261009'


def solve(matrix, rhs):
    n = len(rhs)
    a = [list(map(float, matrix[i]))+[float(rhs[i])] for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(a[row][col]))
        a[col], a[pivot] = a[pivot], a[col]
        scale = a[col][col]
        if abs(scale) < 1e-12:
            raise ValueError('singular matrix')
        a[col] = [x/scale for x in a[col]]
        for row in range(n):
            if row != col:
                factor=a[row][col]
                a[row]=[a[row][j]-factor*a[col][j] for j in range(n+1)]
    return [a[i][-1] for i in range(n)]


def multiply(a, b):
    return [sum(x*y for x,y in zip(row,b)) for row in a]


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def condition_inf(a):
    n=len(a)
    inverse_columns=[solve(a, [float(i==j) for i in range(n)]) for j in range(n)]
    return max(sum(abs(x) for x in row) for row in a)*max(
        sum(abs(inverse_columns[j][i]) for j in range(n)) for i in range(n))


def identity(path):
    return dict(path=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--controls',type=Path,default=BASE/'live-measured-run9-surface-wrench-20261009.json')
    ap.add_argument('--rates',type=Path,default=BASE/'live-measured-run9-aero-rate-matrix-20261009.json')
    ap.add_argument('--output',type=Path,default=BASE/'measured-control-authority-analysis.json')
    args=ap.parse_args()
    controls=json.loads(args.controls.read_text()); rates=json.loads(args.rates.read_text())
    if controls['status']!='complete' or rates['status']!='complete':
        ap.error('both source probes must be complete')
    rate_by_record={a['anchorRecordSequence']:a for a in rates['anchorMatrices']}
    report=dict(classification='offline analysis of hypothetical aerodynamic API responses; unqualified',
                sources=[identity(args.controls),identity(args.rates)],
                conventions='raw local +X pitch, +Y roll, +Z yaw torques; normalized vessel pitch/roll/yaw inputs',
                limitations=['Five anchors from one historically bypassed run; no gate or landing evidence.',
                             'Rate probe used neutral surfaces; rates with deflected surfaces are not identified.',
                             'Control matrix is an isolated-input finite difference; compound-input response may be nonlinear.',
                             'Surface angles and historical RCS state are unavailable.',
                             'Static zero-torque cancellation is not a dynamic sideslip/attitude controller.'], anchors=[])
    for anchor in controls['anchorControlMatrices']:
        record=anchor['anchorRecordSequence']; rate=rate_by_record[record]
        if abs(anchor['ut']-rate['ut'])>1e-6:
            raise ValueError('anchor epoch mismatch')
        b=anchor['controlEffectivenessMatrixNmPerControl_bodyTorqueXYZ_by_pitchRollYaw']
        neutral=anchor['neutralSurfaceTorqueNm_bodyXYZ']
        command=anchor['recordedControlReadbackPitchRollYaw']
        observed=anchor['replayedRecordedControlTorqueNm_bodyXYZ']
        predicted_delta=multiply(b,command)
        observed_delta=[observed[i]-neutral[i] for i in range(3)]
        residual=[predicted_delta[i]-observed_delta[i] for i in range(3)]
        trim=solve(b,[-x for x in neutral])
        trim_residual=[x+y for x,y in zip(multiply(b,trim),neutral)]
        # Hold pitch input: solve the yaw-torque row for a unit roll input.
        yaw_per_roll=-b[2][1]/b[2][2]
        roll_effect=b[1][1]+b[1][2]*yaw_per_roll
        # Hold pitch torque too: eliminate pitch and yaw inputs jointly.
        pitch_per_roll,yaw_with_pitch=solve([[b[0][0],b[0][2]],[b[2][0],b[2][2]]],[-b[0][1],-b[2][1]])
        pitch_yaw_neutral_effect=b[1][0]*pitch_per_roll+b[1][1]+b[1][2]*yaw_with_pitch
        derivatives=[]
        for r in rate['localRateDerivativeMatrices']:
            d=r['matrixNmPerRadPerSecond_bodyTorqueXYZ_by_bodyRateXYZ']
            derivatives.append(dict(offset_deg_s=r['centralDifferenceOffsetDegPerSecond'],
                                    yaw_rate_to_roll_torque_per_one_deg_s=d[1][2]*math.pi/180,
                                    roll_input_to_yaw_torque_per_quarter_input=b[2][1]*.25,
                                    neutral_surface_rate_matrix_condition_inf=condition_inf(d)))
        report['anchors'].append(dict(record_sequence=record,ut=anchor['ut'],mach=anchor['mach'],
            alpha_deg=anchor['angleOfAttackDeg'],beta_deg=anchor['sideslipDeg'],
            source_control_matrix_nm_per_input=b,
            source_neutral_torque_nm=neutral,
            source_compound_torque_nm=observed,
            control_matrix_condition_inf=condition_inf(b),
            linear_zero_aerodynamic_torque_trim=trim,
            trim_within_unit_input_box=all(abs(x)<=1 for x in trim),
            trim_linear_residual_nm=trim_residual,
            yaw_input_per_roll_input_for_zero_yaw_torque=yaw_per_roll,
            zero_yaw_roll_effectiveness_fraction=roll_effect/b[1][1],
            zero_pitch_and_yaw_torque_roll_effectiveness_fraction=pitch_yaw_neutral_effect/b[1][1],
            zero_pitch_and_yaw_control_direction=[pitch_per_roll,1.0,yaw_with_pitch],
            unit_box_max_roll_input_in_that_direction=1/max(abs(x) for x in [pitch_per_roll,1,yaw_with_pitch]),
            compound_input_linear_residual_nm=residual,
            compound_input_relative_delta_residual=norm(residual)/max(1e-12,norm(observed_delta)),
            recorded_command=command, rate_comparison=derivatives))
    report['interpretation']=(
        'The isolated surface matrix predicts substantial yaw torque from roll input. '
        'Cancelling it consumes yaw input and reduces the static roll effectiveness. '
        'The final anchor is near a cancellation singularity, but compound-input residuals '
        'are too large to establish the sign of the remaining roll authority. '
        'Do not invert these matrices into flight gains or change the low-q bank ramp. '
        'A dynamic actuator model needs joint-input and surface/rate evidence first.')
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    for a in report['anchors']:
        print(f"record={a['record_sequence']} condition={a['control_matrix_condition_inf']:.1f} "
              f"yaw/roll={a['yaw_input_per_roll_input_for_zero_yaw_torque']:.3f} "
              f"remainingRoll={a['zero_yaw_roll_effectiveness_fraction']:.3f} "
              f"compoundResidual={a['compound_input_relative_delta_residual']:.1%}")


if __name__=='__main__':
    main()
