"""One-command STS mission: launch to a circular orbit of a given altitude and inclination.

    python -m shuttle.mission --altitude 90 --inclination 34

Steps: (optionally revert to launch) -> pad prep (Clydesdale limiters, settle) ->
ascent to an <altitude> x <altitude-30> km insertion orbit with ET separation ->
OMS circularization at apoapsis to <altitude> x <altitude> km at <inclination>.
"""

import argparse
import datetime
import os
import subprocess
import sys
import time

import krpc

SRB_LIMIT = 0.9     # Clydesdale thrust limiter (revert reloads the VAB value)
SSME_LIMIT = 0.38   # SSME limiter at liftoff (pitch-moment feasible)
PAD_SETTLE_S = 25.0


def wait_for_flight_scene(timeout=180.0):
    deadline = time.time()+timeout
    while time.time() < deadline:
        try:
            conn = krpc.connect(name='mission-wait')
            if str(conn.krpc.current_game_scene) == 'GameScene.flight':
                conn.space_center.active_vessel.name
                return conn
            conn.close()
        except Exception:
            pass
        time.sleep(2.0)
    raise RuntimeError('KSP did not reach the flight scene')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--altitude', type=float, required=True,
                        help='Circular orbit altitude, km (>= 80)')
    parser.add_argument('--inclination', type=float, required=True,
                        help='Orbit inclination, deg')
    parser.add_argument('--revert', action='store_true',
                        help='Revert the current flight to launch first')
    args = parser.parse_args()
    if args.altitude < 80.0:
        parser.error('altitude must be at least 80 km (atmosphere top is 70 km)')
    alt = args.altitude*1000.0
    insertion_pe = max(55000.0, alt-30000.0)

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    logdir = os.path.join(root, 'runs')
    os.makedirs(logdir, exist_ok=True)
    tag = f'{stamp}_{args.altitude:.0f}km_{args.inclination:g}deg'

    if args.revert:
        krpc.connect(name='mission-revert').space_center.revert_to_launch()
        time.sleep(3.0)
    conn = wait_for_flight_scene()
    print(f'Pad: settling {PAD_SETTLE_S:.0f} s before launch')
    time.sleep(PAD_SETTLE_S)
    vessel = conn.space_center.active_vessel
    for engine in vessel.parts.engines:
        if 'Clydesdale' in engine.part.title:
            engine.thrust_limit = SRB_LIMIT
    conn.close()

    py = sys.executable
    ascent = [py, '-u', '-m', 'shuttle.flight', '--launch', '--ssme-limit', str(SSME_LIMIT),
              '--orbit', str(alt), '--meco-periapsis', str(insertion_pe),
              '--inclination', str(args.inclination),
              '--telemetry', os.path.join(logdir, f'{tag}_ascent.csv')]
    circ = [py, '-u', '-m', 'shuttle.orbit_trim', '--apoapsis', str(alt),
            '--periapsis', str(alt), '--inclination', str(args.inclination)]
    print(f'Ascent to {alt/1000:.0f} x {insertion_pe/1000:.0f} km at {args.inclination:g} deg')
    with open(os.path.join(logdir, f'{tag}_ascent.log'), 'w') as log:
        if subprocess.call(ascent, cwd=root, stdout=log, stderr=subprocess.STDOUT) != 0:
            sys.exit(f'Ascent failed; see runs/{tag}_ascent.log')
    print(f'Circularizing to {alt/1000:.0f} x {alt/1000:.0f} km at {args.inclination:g} deg')
    with open(os.path.join(logdir, f'{tag}_circularize.log'), 'w') as log:
        code = subprocess.call(circ, cwd=root, stdout=log, stderr=subprocess.STDOUT)
    with open(os.path.join(logdir, f'{tag}_circularize.log')) as log:
        lines = [line.strip() for line in log if line.startswith(('final', 'after burn'))]
    print(lines[-1] if lines else 'no circularization result')
    sys.exit(code)


if __name__ == '__main__':
    main()
