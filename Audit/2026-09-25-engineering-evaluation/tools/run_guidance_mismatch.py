#!/usr/bin/env python3
"""Run ShuttleSim/scripts/run_guidance.py with a MM305 terminal model that differs
from the simulator plant.

run_guidance.py passes the same --aero/--aero-book files to both the simulator
(plant) and the backend (KSP_LANDER_TERMINAL_AERO/_AERO_BOOK -> MM305 model and
guidance "certified prior").  This wrapper only rewrites the *backend* process
environment when AUDIT_TERMINAL_AERO / AUDIT_TERMINAL_AERO_BOOK are set, so the
plant stays as given on the command line.  No production code is modified.
"""
import os
import pathlib
import runpy
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "ShuttleSim" / "scripts" / "run_guidance.py"

_real_popen = subprocess.Popen


def _popen(*args, **kwargs):
    env = kwargs.get("env")
    if env and "KSP_LANDER_TERMINAL_AERO" in env:
        env = dict(env)
        for src, dst in (("AUDIT_TERMINAL_AERO", "KSP_LANDER_TERMINAL_AERO"),
                         ("AUDIT_TERMINAL_AERO_BOOK", "KSP_LANDER_TERMINAL_AERO_BOOK")):
            if os.environ.get(src):
                env[dst] = os.environ[src]
        kwargs["env"] = env
    return _real_popen(*args, **kwargs)


subprocess.Popen = _popen
sys.path.insert(0, str(SCRIPT.parent))
sys.argv = [str(SCRIPT)] + sys.argv[1:]
runpy.run_path(str(SCRIPT), run_name="__main__")
