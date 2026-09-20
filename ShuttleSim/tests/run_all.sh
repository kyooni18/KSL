#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cmake -S "$ROOT" -B "$ROOT/build" >/dev/null
cmake --build "$ROOT/build" -j2 >/dev/null

OUT="$ROOT/tests/.smoke.jsonl"
ERR="$ROOT/tests/.smoke.err"
"$ROOT/build/shuttlesim" \
  --scenario "$ROOT/scenarios/orbit86km.ini" \
  --atmosphere "$ROOT/data/kerbin_atmosphere_seed.csv" \
  --aero "$ROOT/data/stsn_aero_seed.csv" \
  --attitude "$ROOT/data/stsn_attitude_seed.ini" \
  --fixed-aoa 35 --fixed-bank 0 --rate max --telemetry-hz 0.1 \
  --command-port 0 --telemetry-port 0 --web-telemetry-port 0 \
  --quiet --max-sim-time 2400 >"$OUT" 2>"$ERR"
grep -q '"touchdown":true' "$ERR"

"$ROOT/build/shuttlesim" \
  --scenario "$ROOT/scenarios/orbit86km.ini" \
  --attitude-replay "$ROOT/tests/replay_sample.csv" --replay-mode actual \
  --rate max --telemetry-hz 1 --max-sim-time 90 \
  --command-port 0 --telemetry-port 0 --web-telemetry-port 0 \
  --quiet >/dev/null 2>"$ROOT/tests/.replay.err"

grep -q '"touchdown":false' "$ROOT/tests/.replay.err"
python3 "$ROOT/tests/orbit_test.py"
python3 "$ROOT/tests/protocol_test.py"
rm -f "$OUT" "$ERR" "$ROOT/tests/.replay.err"
echo 'run_all: PASS'
