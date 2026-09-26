"""ShuttleSim model profile selection shared by the simulator tooling.

Mirrors shuttle_sim_model_path() in CLanding/telemetry/sim_telemetry.c: a named
profile (KSP_LANDER_MODEL_PROFILE, default "identified") is the single source of
truth for the atmosphere/aero/force-book/attitude files.  The choice never
depends on which files exist on disk.  A profile without a force book returns
the literal path "none".
"""
from __future__ import annotations

import hashlib
import os
import pathlib

SIM = pathlib.Path(__file__).resolve().parents[1]

PROFILES = {
    "identified": {
        "atmosphere": "reference-model/kerbin_atmosphere_identified.csv",
        "aero": "reference-model/stsn_aero_identified.csv",
        "aero_book": None,
        "attitude": "reference-model/stsn_attitude_identified.ini",
        "certified_prior": "reference-model/stsn_certified_prior_identified.csv",
    },
    "fitted-legacy": {
        "atmosphere": "data/fitted/kerbin_atmosphere_ksp.csv",
        "aero": "data/fitted/stsn_aero_ksp_robust.csv",
        "aero_book": "data/fitted/stsn_force_book.csv",
        "attitude": "data/fitted/stsn_attitude_ksp.ini",
        "certified_prior": "data/fitted/stsn_force_book.csv",
    },
    "reference-b": {
        "atmosphere": "reference-model/kerbin_atmosphere_reference.csv",
        "aero": "reference-model/stsn_aero_reference.csv",
        "aero_book": "reference-model/stsn_force_book_reference.csv",
        "attitude": "reference-model/stsn_attitude_reference.ini",
        "certified_prior": "reference-model/stsn_force_book_reference.csv",
    },
}


def active_profile() -> str:
    name = os.environ.get("KSP_LANDER_MODEL_PROFILE") or "identified"
    if name not in PROFILES:
        raise SystemExit(f"unknown KSP_LANDER_MODEL_PROFILE {name!r}; known: {sorted(PROFILES)}")
    return name


def model_file(kind: str, profile: str | None = None) -> pathlib.Path | str:
    rel = PROFILES[profile or active_profile()][kind]
    return "none" if rel is None else SIM / rel


def file_digest(path) -> str | None:
    if path is None or str(path).lower() == "none" or not pathlib.Path(path).is_file():
        return None
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()
