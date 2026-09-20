#!/bin/sh
set -eu
ROOT="${1:-/Users/kyooni18/Code/C/KSPShuttleLander}"
KEEP_FLIGHTLOGS="${KEEP_FLIGHTLOGS:-8}"
cd "$ROOT" || exit 1
printf 'cleanup_started=%s\n' "$(date '+%Y-%m-%d %H:%M:%S %z')"
printf 'disk_before='; df -h . | awk 'NR==2 {print $4 " free, " $5 " used"}'
if [ -d FlightLogs ]; then
  tmp=$(mktemp)
  find FlightLogs -mindepth 1 -maxdepth 1 -print0 | \
    xargs -0 ls -dt 2>/dev/null > "$tmp" || true
  total=$(wc -l < "$tmp" | tr -d ' ')
  printf 'flightlogs_total=%s keep=%s\n' "$total" "$KEEP_FLIGHTLOGS"
  if [ "$total" -gt "$KEEP_FLIGHTLOGS" ]; then
    tail -n +$((KEEP_FLIGHTLOGS + 1)) "$tmp" | while IFS= read -r path; do
      [ -n "$path" ] && rm -rf "$path"
    done
  fi
  rm -f "$tmp"
fi
find /tmp -maxdepth 1 -name 'shuttlesim-*' -prune -exec rm -rf {} + 2>/dev/null || true
find Runtime/SlowRuns -maxdepth 1 -name 'rl-fasttrend-*' -prune -exec rm -rf {} + 2>/dev/null || true
printf 'disk_after='; df -h . | awk 'NR==2 {print $4 " free, " $5 " used"}'
if [ -d FlightLogs ]; then
  printf 'kept_flightlogs:\n'
  find FlightLogs -mindepth 1 -maxdepth 1 -print0 | xargs -0 ls -dt 2>/dev/null | sed -n '1,20p'
fi
