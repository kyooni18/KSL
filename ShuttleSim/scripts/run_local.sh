#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
SIM="$ROOT/ShuttleSim"
BUILD="$SIM/build"
RATE=${SIM_RATE:-max}
SCENARIO=${SIM_SCENARIO:-"$SIM/scenarios/ksp86km-preburn.ini"}
STAMP=$(date -u +%Y%m%dT%H%M%SZ)
RECORD=${SIM_RECORD:-"$SIM/runs/live-$STAMP.jsonl"}

cmake -S "$SIM" -B "$BUILD" >/dev/null
cmake --build "$BUILD" -j "${SIM_JOBS:-4}" >/dev/null

echo "ShuttleSim starting paused"
echo "  command:       udp://127.0.0.1:8795"
echo "  guidance:      udp://127.0.0.1:8796"
echo "  telemetry web: udp://127.0.0.1:8797"
echo "  scenario:      $SCENARIO"
echo "  atmosphere:    stock Kerbin spatial model"
echo "  record:        $RECORD"
echo "  rate:          $RATE"
echo "Resume with: $SIM/scripts/simctl.py resume"

exec "$BUILD/shuttlesim" \
  --scenario "$SCENARIO" \
  --aero "$SIM/data/fitted/stsn_aero_ksp_robust.csv" \
  --aero-book "$SIM/data/fitted/stsn_force_book.csv" \
  --attitude "$SIM/data/fitted/stsn_attitude_ksp.ini" \
  --dt "${SIM_DT:-0.02}" \
  --rate "$RATE" \
  --telemetry-hz "${SIM_TELEMETRY_HZ:-20}" \
  --max-sim-time "${SIM_MAX_TIME:-2400}" \
  --command-port 8795 \
  --telemetry-port 8796 \
  --web-telemetry-port 8797 \
  --record "$RECORD" \
  --start-paused \
  --quiet
