#!/usr/bin/env python3
"""Plant-only bank probe with MM304 q authority and roll-motion limits."""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import socket
import subprocess
from probe_sts_low_alpha_uncertainty import snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    transitions = parser.add_mutually_exclusive_group()
    transitions.add_argument('--incidence-transitions', action='store_true',
                        help='Probe 18 to 28 deg incidence transitions at 50/45/40 km')
    transitions.add_argument('--reverse-incidence-transitions', action='store_true',
                             help='Probe 28 to 18 deg incidence transitions at 50/45/40 km')
    parser.add_argument('--counterfactual-full-bank', action='store_true',
                        help='Diagnostic only: omit positive-q pressure ramp, preserving structural and roll motion limits')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    inputs = [root / 'ShuttleSim/build/shuttlesim',
              root / 'ShuttleSim/scenarios/sts72km-run9-entry-interface.ini',
              root / 'ShuttleSim/reference-model/kerbin_atmosphere_identified.csv',
              root / 'ShuttleSim/reference-model/sts_aero_provisional.csv',
              root / 'ShuttleSim/reference-model/stsn_attitude_identified.ini',
              Path(__file__).resolve(),
              Path(__file__).resolve().with_name('probe_sts_low_alpha_uncertainty.py')]
    results = []
    transition_target = 18 if args.reverse_incidence_transitions else 28
    cases = ([(28 if args.reverse_incidence_transitions else 18, h)
              for h in [50000, 45000, 40000]]
             if args.incidence_transitions or args.reverse_incidence_transitions
             else [(18, None), (28, None)])
    for alpha, transition in cases:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as telemetry, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as commands:
            telemetry.bind(('127.0.0.1', 0))
            commands.bind(('127.0.0.1', 0))
            command_port = commands.getsockname()[1]
            commands.close()
            telemetry.settimeout(5)
            label = f'aoa{alpha}-ramped-bank70' if transition is None else f'aoa{alpha}-to{transition_target}-at{transition}-ramped-bank70'
            if args.counterfactual_full_bank:
                label += '-counterfactual-full-bank'
            raw = out / f'{label}.jsonl'
            cmd = [str(inputs[0]), '--scenario', str(inputs[1]),
                   '--atmosphere', str(inputs[2]), '--aero', str(inputs[3]),
                   '--attitude', str(inputs[4]), '--rate', 'max', '--lockstep',
                   '--command-port', str(command_port), '--telemetry-port',
                   str(telemetry.getsockname()[1]), '--web-telemetry-port', '0',
                   '--fixed-aoa', str(alpha), '--fixed-bank', '0',
                   '--max-sim-time', '800', '--record', str(raw), '--quiet']
            result = dict(aoa=alpha, incidenceTransitionAltitude=transition,
                          incidenceTransitionTarget=transition_target if transition is not None else None,
                          bankMaximum=70, counterfactualFullBank=args.counterfactual_full_bank, command=cmd, peakQ=0, peakG=0)
            process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            # The initial lockstep frame is at t=0 and needs an acknowledgment.
            previous = -.02
            bank = rate = 0
            try:
                while process.poll() is None:
                    try:
                        data, _ = telemetry.recvfrom(65535)
                    except socket.timeout:
                        if process.poll() is not None:
                            break
                        raise RuntimeError('Lockstep frame observation timed out; terminating probe')
                    row = json.loads(data)
                    now = row['sim_time']
                    if now <= previous:
                        continue
                    state = snapshot(row)
                    result['peakQ'] = max(result['peakQ'], state['q'])
                    result['peakG'] = max(result['peakG'], state['g'])
                    for key, condition in [('at25km', state['h'] <= 25000),
                                           ('at860mps', state['v'] <= 860)]:
                        if condition and key not in result:
                            result[key] = state
                    result['final'] = state
                    authority = (70 if state['q'] > 0 else 0) if args.counterfactual_full_bank else 70 * min(1, max(0, state['q'] / 360)) ** 2
                    if state['v'] < 85 or state['q'] > 45000 or state['g'] > 3.5:
                        authority = 0
                    dt = now - previous
                    error = authority - bank
                    desired_rate = math.copysign(min(4.5, math.sqrt(3.6 * abs(error))), error)
                    rate += max(-1.8 * dt, min(1.8 * dt, desired_rate - rate))
                    next_bank = bank + rate * dt
                    if (authority - bank) * (authority - next_bank) <= 0:
                        next_bank = authority
                        rate = 0
                    bank = max(0, min(authority, next_bank))
                    # Commands use the plant protocol; no admission/guidance API.
                    incidence = transition_target if transition is not None and state['h'] <= transition else alpha
                    payload = dict(aoa_deg=incidence, bank_deg=bank, throttle=0,
                                   airbrakes=False, step=True)
                    telemetry.sendto(json.dumps(payload, separators=(',', ':')).encode(),
                                     ('127.0.0.1', command_port))
                    previous = now
                process.wait(timeout=5)
                if process.returncode:
                    raise RuntimeError(process.stderr.read().decode())
            finally:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=5)
                process.stderr.close()
            result['rawSha256'] = hashlib.sha256(raw.read_bytes()).hexdigest()
            with raw.open('rb') as source, gzip.open(str(raw) + '.gz', 'wb') as archive:
                archive.write(source.read())
            raw.unlink()
            result['retainedLog'] = str(raw) + '.gz'
            results.append(result)
            print(alpha, json.dumps(result), flush=True)
    report = dict(counterfactualFullBank=args.counterfactual_full_bank, scope='Fixed positive-bank plant probe with q ramp, nominal roll '
                        'rate/acceleration and structural limits. Not guidance, '
                        'nominal 40-degree planning, coordination validation, '
                        'qualification, or landing. Original provisional STS aero; '
                        'borrowed STS-N attitude dynamics. With counterfactual-full-bank, '
                        'the positive-q pressure ramp is deliberately omitted only '
                        'for diagnosis, never for live guidance or qualification.',
                  inputs=[dict(path=str(p), sha256=hashlib.sha256(p.read_bytes()).hexdigest())
                          for p in inputs],
                  limits=dict(qAuthorityPa=360, maximumQPa=45000, maximumG=3.5,
                              minimumSpeedMps=85, maximumRollRateDegS=4.5,
                              maximumRollAccelerationDegS2=1.8,
                              controlPeriodS=.1,
                              captureRule='Reset bank reference rate at target crossing, '
                                          'as in entry_lateral.update_bank_command'),
                  results=results)
    (out / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
