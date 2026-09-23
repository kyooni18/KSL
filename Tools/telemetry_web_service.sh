#!/bin/zsh
set -euo pipefail

ROOT=${0:A:h:h}
RUNTIME="$ROOT/Runtime/TelemetryWeb"
PID_FILE="$RUNTIME/server.pid"
LOG_FILE="$RUNTIME/server.log"
LABEL=${KSP_LANDER_WEB_LAUNCH_LABEL:-com.kyooni18.ksp.telemetry-web}
HOST=${KSP_LANDER_WEB_HOST:-127.0.0.1}
PORT=${KSP_LANDER_WEB_PORT:-8789}
MODE=${KSP_LANDER_WEB_MODE:-live}
SIM_TELEMETRY_PORT=${KSP_LANDER_SIM_WEB_TELEMETRY_PORT:-8797}
ACTION=${1:-start}
mkdir -p "$RUNTIME"

find_python() {
  local candidates=()
  [[ -n ${KSP_LANDER_WEB_PYTHON:-} ]] && candidates+=("$KSP_LANDER_WEB_PYTHON")
  candidates+=(
    "$ROOT/Runtime/PythonBridge/.venv/bin/python"
    "$HOME/Code/Python/KSPFlightComputer/Shuttle/venv/bin/python"
  )
  local py
  for py in $candidates; do
    if [[ -x "$py" ]]; then
      if [[ "$MODE" == "simulator" ]] || "$py" -c 'import krpc' >/dev/null 2>&1; then
        print -r -- "$py"
        return 0
      fi
    fi
  done
  if command -v python3 >/dev/null 2>&1; then
    if [[ "$MODE" == "simulator" ]] || python3 -c 'import krpc' >/dev/null 2>&1; then
      command -v python3
      return 0
    fi
  fi
  return 1
}

current_pid() {
  [[ -f "$PID_FILE" ]] || return 1
  local pid
  pid=$(<"$PID_FILE")
  [[ "$pid" == <-> ]] || return 1
  kill -0 "$pid" 2>/dev/null || return 1
  print -r -- "$pid"
}

launchd_pid() {
  local pid
  pid=$(launchctl list 2>/dev/null | awk -v label="$LABEL" '$3 == label {print $1; exit}')
  [[ "$pid" == <-> ]] || return 1
  [[ "$pid" != "-" ]] || return 1
  kill -0 "$pid" 2>/dev/null || return 1
  print -r -- "$pid"
}

stop_server() {
  launchctl remove "$LABEL" 2>/dev/null || true
  local pid=""
  if ! pid=$(current_pid); then
    local listener listener_cmd
    listener=$(lsof -tiTCP:"$PORT" -sTCP:LISTEN 2>/dev/null | head -1 || true)
    if [[ -n "$listener" ]]; then
      listener_cmd=$(ps -p "$listener" -o command= 2>/dev/null || true)
      [[ "$listener_cmd" == *telemetry_web.py* ]] && pid="$listener"
    fi
  fi
  if [[ -n "$pid" ]]; then
    local cmd
    cmd=$(ps -p "$pid" -o command= 2>/dev/null || true)
    if [[ "$cmd" == *telemetry_web.py* ]]; then
      kill "$pid" 2>/dev/null || true
      for _ in {1..20}; do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.1
      done
    fi
  fi
  rm -f "$PID_FILE"
}

case "$ACTION" in
  stop)
    stop_server
    print "telemetry web stopped"
    exit 0
    ;;
  restart)
    stop_server
    ;;
  start)
    if pid=$(current_pid); then
      current_cmd=$(ps -p "$pid" -o command= 2>/dev/null || true)
      expected_script="$ROOT/Tools/telemetry_web.py"
      if [[ "$current_cmd" == *"$expected_script"* ]]; then
        print "telemetry web already running: pid=$pid http://$HOST:$PORT"
        exit 0
      fi
      print -u2 "telemetry web process uses a stale repo root; restarting from $ROOT"
      stop_server
    fi
    ;;
  *)
    print -u2 "usage: $0 [start|restart|stop]"
    exit 2
    ;;
esac

listener=$(lsof -tiTCP:"$PORT" -sTCP:LISTEN 2>/dev/null | head -1 || true)
if [[ -n "$listener" ]]; then
  cmd=$(ps -p "$listener" -o command= 2>/dev/null || true)
  print -u2 "port $PORT is already in use by pid=$listener: $cmd"
  exit 3
fi

PY=$(find_python) || {
  print -u2 "no Python interpreter with krpc is available"
  exit 4
}

RL_ARGS=()
if [[ -n ${KSP_LANDER_RL_OUTPUT:-} ]]; then
  RL_ARGS+=(--rl-output "$KSP_LANDER_RL_OUTPUT")
fi
MODE_ARGS=(--mode "$MODE")
if [[ "$MODE" == "simulator" ]]; then
  MODE_ARGS+=(--sim-telemetry-port "$SIM_TELEMETRY_PORT")
fi

launchctl submit -l "$LABEL" -p "$PY" -o "$LOG_FILE" -e "$LOG_FILE" -- \
  "$PY" "$ROOT/Tools/telemetry_web.py" --host "$HOST" --port "$PORT" \
  "${MODE_ARGS[@]}" "${RL_ARGS[@]}"
pid=""
for _ in {1..20}; do
  pid=$(launchd_pid || true)
  [[ -n "$pid" ]] && break
  sleep 0.1
done
if [[ -z "$pid" ]]; then
  print -u2 "telemetry web failed to start"
  tail -40 "$LOG_FILE" >&2 || true
  exit 5
fi
print -r -- "$pid" > "$PID_FILE"
print "telemetry web running: pid=$pid http://$HOST:$PORT"
