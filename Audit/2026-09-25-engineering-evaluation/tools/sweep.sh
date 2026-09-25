#!/bin/bash
# sweep.sh TAG PLANT ENGAGE SCENARIO... [-- extra run_guidance args]
#   PLANT: ref (tracked reference model) | A | C (audit plants in ../plants)
#   ENGAGE: engageHACTest | engageFinalTest | engageReentry | engage
# Builds the dev backend, snapshots it (so later rebuilds cannot contaminate
# the sweep), runs the scenarios 4 at a time, and prints a touchdown summary.
# Results: $OUT/res/TAG-<scenario>.json (default OUT=/tmp/ksl-sweeps).
set -u
HERE=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$HERE/../../.." && pwd)
OUT=${OUT:-/tmp/ksl-sweeps}; mkdir -p "$OUT/res" "$OUT/bin-$1"
tag=$1; plant=$2; engage=$3; shift 3
scen=(); extra=()
while [ $# -gt 0 ]; do [ "$1" = "--" ] && { shift; extra=("$@"); break; }; scen+=("$1"); shift; done
make -C "$ROOT/CLanding" BUILD=build-dev -j4 >/dev/null 2>&1 || { echo BUILD FAILED; exit 1; }
cp "$ROOT/CLanding/build-dev/landing_backend" "$OUT/bin-$tag/"
P="$HERE/../plants"
case $plant in
  ref) pargs=();;
  A) pargs=(--aero "$P/stsn_aero_seed.csv" --aero-book "$P/force_book_A.csv");;
  C) pargs=(--aero "$P/stsn_aero_kspC.csv" --aero-book "$P/force_book_C.csv");;
esac
run_one(){ f=$1; b=$(basename "$f" .ini)
  (cd "$ROOT" && timeout 1500 python3 ShuttleSim/scripts/run_guidance.py --scenario "$f" "${pargs[@]}" \
     --skip-build --no-mirror --quiet-progress --label "$tag-$b" --engage "$engage" \
     --backend-build-dir "$OUT/bin-$tag" "${extra[@]}" 2>&1 | tail -1) > "$OUT/res/$tag-$b.json"; }
for f in "${scen[@]}"; do
  while [ "$(jobs -rp | wc -l)" -ge 4 ]; do wait -n; done
  run_one "$f" &
done
wait
for f in "${scen[@]}"; do python3 "$HERE/touchdown_summary.py" "$OUT/res/$tag-$(basename "$f" .ini).json"; done
