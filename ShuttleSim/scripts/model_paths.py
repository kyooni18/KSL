"""ShuttleSim model data file selection shared by the simulator tooling.

The KSP-fitted model lives under the git-ignored ShuttleSim/data/fitted/.  A
tracked, explicitly labelled reference model under ShuttleSim/reference-model/
keeps a clean checkout runnable.  The fitted file wins whenever it exists
(mirrors shuttle_sim_model_path() in CLanding/telemetry/sim_telemetry.c).
"""
from __future__ import annotations

import pathlib

SIM = pathlib.Path(__file__).resolve().parents[1]

_FILES = {
    "atmosphere": ("data/fitted/kerbin_atmosphere_ksp.csv",
                   "reference-model/kerbin_atmosphere_reference.csv"),
    "aero": ("data/fitted/stsn_aero_ksp_robust.csv",
             "reference-model/stsn_aero_reference.csv"),
    "aero_book": ("data/fitted/stsn_force_book.csv",
                  "reference-model/stsn_force_book_reference.csv"),
    "attitude": ("data/fitted/stsn_attitude_ksp.ini",
                 "reference-model/stsn_attitude_reference.ini"),
}


def model_file(kind: str) -> pathlib.Path:
    fitted, reference = _FILES[kind]
    path = SIM / fitted
    return path if path.is_file() else SIM / reference
