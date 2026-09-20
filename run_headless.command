#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

PINNED_RESTART=0
if [ -n "${KSP_LANDER_CAMPAIGN_IDENTITY:-}" ]; then
  PINNED_RESTART=1
fi
for arg in "$@"; do
  case "$arg" in
    --campaign-identity|--campaign-identity=*) PINNED_RESTART=1; break ;;
  esac
done
if [ "$PINNED_RESTART" -eq 0 ]; then
  make -C CLanding
fi
export KSP_LANDER_ROOT="$ROOT"
exec python3 "$ROOT/Tools/headless_flight.py" --backend "$ROOT/CLanding/build/landing_backend" "$@"
