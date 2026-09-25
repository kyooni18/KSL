#!/bin/bash
# sweep.sh <LABEL_PREFIX> <PLANT(ref|A|B|C)> <ENGAGE> <SCENARIOS...> [-- EXTRA_ARGS...]
set -e

REPO_DIR="$(cd "$(dirname "$0")/../../.." && pwd)"
AUDIT_PLANTS="$REPO_DIR/Audit/2026-09-25-engineering-evaluation/plants"
REF_MODEL="$REPO_DIR/ShuttleSim/reference-model"

PRE="$1"
PLANT="$2"
ENGAGE="$3"
shift 3

# Parse scenarios vs extra args (after --)
SCENARIOS=()
EXTRA_ARGS=()
PAST_SEPARATOR=false
for arg in "$@"; do
    if [ "$arg" = "--" ]; then
        PAST_SEPARATOR=true
        continue
    fi
    if [ "$PAST_SEPARATOR" = true ]; then
        EXTRA_ARGS+=("$arg")
    else
        SCENARIOS+=("$arg")
    fi
done

SWEEP_DIR="/tmp/ksl-sweeps"
RES_DIR="$SWEEP_DIR/res"
BIN_DIR="$SWEEP_DIR/bin_$$"
mkdir -p "$RES_DIR" "$BIN_DIR"

# Snapshot the backend binary
SRC_BACKEND="$REPO_DIR/CLanding/build/landing_backend"
if [ ! -f "$SRC_BACKEND" ]; then
    echo "Building landing_backend..." >&2
    make -C "$REPO_DIR/CLanding"
fi
cp "$SRC_BACKEND" "$BIN_DIR/landing_backend"

cleanup() {
    rm -rf "$BIN_DIR"
}
trap cleanup EXIT

# Resolve plant files
case "$PLANT" in
    ref|B|b)
        AERO="$REF_MODEL/stsn_aero_reference.csv"
        BOOK="$REF_MODEL/stsn_force_book_reference.csv"
        ATM="$REF_MODEL/kerbin_atmosphere_reference.csv"
        ATT="$REF_MODEL/stsn_attitude_reference.ini"
        ;;
    A|a)
        AERO="$AUDIT_PLANTS/stsn_aero_seed.csv"
        BOOK="$AUDIT_PLANTS/force_book_A.csv"
        ATM="$AUDIT_PLANTS/kerbin_atmosphere_seed.csv"
        ATT="$AUDIT_PLANTS/stsn_attitude_seed.ini"
        ;;
    C|c)
        AERO="$AUDIT_PLANTS/stsn_aero_kspC.csv"
        BOOK="$AUDIT_PLANTS/force_book_C.csv"
        ATM="$AUDIT_PLANTS/kerbin_atmosphere_seed.csv"
        ATT="$AUDIT_PLANTS/stsn_attitude_seed.ini"
        ;;
    *)
        echo "Unknown plant: $PLANT (expected ref, A, B, or C)" >&2
        exit 1
        ;;
esac

echo "======================================================================"
echo "Starting sweep: prefix=$PRE plant=$PLANT engage=$ENGAGE"
echo "Binary snapshot: $BIN_DIR/landing_backend"
echo "Extra args: ${EXTRA_ARGS[*]}"
echo "======================================================================"

for scen in "${SCENARIOS[@]}"; do
    scen_path="$(cd "$(dirname "$scen")" && pwd)/$(basename "$scen")"
    scen_name="$(basename "$scen" .ini)"
    out_json="$RES_DIR/${PRE}-${scen_name}.json"
    label="${PRE}-${scen_name}"

    echo -n "Running $scen_name ... "
    
    out=$(python3 "$REPO_DIR/ShuttleSim/scripts/run_guidance.py" \
        --scenario "$scen_path" \
        --backend-build-dir "$BIN_DIR" \
        --atmosphere "$ATM" \
        --aero "$AERO" \
        --aero-book "$BOOK" \
        --attitude "$ATT" \
        --engage "$ENGAGE" \
        --label "$label" \
        --skip-build \
        --no-mirror \
        --quiet-progress \
        "${EXTRA_ARGS[@]}" 2>&1 | tail -1)

    echo "$out" > "$out_json"

    # Format quick summary from JSON
    python3 -c '
import json, sys
try:
    with open(sys.argv[1]) as f:
        d = json.load(f)
    sim = d.get("simulatorSummary", {})
    success = d.get("success", False)
    abort = d.get("abortReason")
    if abort:
        print(f"ABORT ({abort})")
    elif sim.get("touchdown"):
        rw = "on-runway" if sim.get("on_runway") else "OFF-RUNWAY"
        along = sim.get("touchdown_along_m", 0.0)
        cross = sim.get("touchdown_cross_m", 0.0)
        sink = sim.get("touchdown_sink_mps", 0.0)
        spd = sim.get("touchdown_speed_mps", 0.0)
        res = "PASS" if success else "FAIL"
        print(f"{rw}: sink={sink:.1f} m/s, spd={spd:.1f} m/s, along={along:.0f} m, cross={cross:.0f} m [{res}]")
    else:
        st = sim.get("sim_time_s", 0)
        print(f"No touchdown (sim_time={st:.1f}s)")
except Exception as e:
    print(f"Parse error: {e}")
' "$out_json"
done

echo "======================================================================"
echo "Sweep complete. Results saved in $RES_DIR"
