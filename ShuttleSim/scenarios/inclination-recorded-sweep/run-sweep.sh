#!/bin/sh
set -u
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../../.." && pwd)
cd "$ROOT" || exit 1
for i in 045 060 075 085 090 095 105 120 135 150 165 180; do
  echo "=== inclination ${i} ==="
  python3 ShuttleSim/scripts/run_guidance.py \
    --scenario "ShuttleSim/scenarios/inclination-recorded-sweep/ksp86km-postburn-i${i}.ini" \
    --engage engageReentry \
    --label "incl-recorded-i${i}" \
    --no-mirror --quiet-progress --compact-guidance-log --skip-build
done
