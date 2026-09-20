#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
RUNTIME="$ROOT/Runtime/PyQtUI"
VENV="$RUNTIME/.venv"

mkdir -p "$RUNTIME"

if [ ! -x "$VENV/bin/python3" ]; then
  python3 -m venv "$VENV"
fi

if ! "$VENV/bin/python3" -c 'import PyQt6, krpc' >/dev/null 2>&1; then
  "$VENV/bin/python3" -m pip install --disable-pip-version-check -r "$ROOT/PyQtApp/requirements.txt"
fi

cd "$ROOT"
make -C CLanding
export KSP_LANDER_ROOT="$ROOT"
exec "$VENV/bin/python3" -m PyQtApp.main --backend "$ROOT/CLanding/build/landing_backend"
