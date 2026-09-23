#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
RUNTIME="$ROOT/Runtime/PyQtUI"
VENV="$RUNTIME/.venv"
CLANDING_ROOT="${KSP_CLANDING_ROOT:-CLanding}"
case "$CLANDING_ROOT" in
  /*) CLANDING_DIR="$CLANDING_ROOT" ;;
  *) CLANDING_DIR="$ROOT/$CLANDING_ROOT" ;;
esac
export KSP_CLANDING_ROOT="$CLANDING_DIR"

mkdir -p "$RUNTIME"

if [ ! -x "$VENV/bin/python3" ]; then
  python3 -m venv "$VENV"
fi

if ! "$VENV/bin/python3" -c 'import PyQt6, krpc' >/dev/null 2>&1; then
  "$VENV/bin/python3" -m pip install --disable-pip-version-check -r "$ROOT/PyQtApp/requirements.txt"
fi

cd "$ROOT"
make -C "$CLANDING_DIR"
export KSP_LANDER_ROOT="$ROOT"
exec "$VENV/bin/python3" -m PyQtApp.main --backend "$CLANDING_DIR/build/landing_backend"
