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
    "sts-provisional": {
        "atmosphere": "reference-model/kerbin_atmosphere_identified.csv",
        "aero": "reference-model/sts_aero_provisional.csv",
        "aero_book": None,
        # Borrowed response dynamics, not STS attitude validation.
        "attitude": "reference-model/stsn_attitude_identified.ini",
        "certified_prior": None,
    },
    # Recalibrated to the latest live STS vehicle state (2026-10-09 flights); still
    # provisional: no certified cells, Mach<3 latest-state data absent, attitude borrowed.
    "sts-live-20261009": {
        "atmosphere": "reference-model/kerbin_atmosphere_identified.csv",
        "aero": "reference-model/sts_aero_live20261009.csv",
        "aero_book": None,
        "attitude": "reference-model/stsn_attitude_identified.ini",
        "certified_prior": None,
    },
    # sts-live-20261009 with Mach 0.2-1.5 rows refitted to live 2026-10-09 terminal
    # flights (Tools/fit_sts_terminal_polar_live.py); still provisional/uncertified.
    "sts-live-20261010": {
        "atmosphere": "reference-model/kerbin_atmosphere_identified.csv",
        "aero": "reference-model/sts_aero_live20261010.csv",
        "aero_book": None,
        "attitude": "reference-model/stsn_attitude_identified.ini",
        "certified_prior": None,
    },
    # sts-live-20261010 with Mach 1.6-7.1 rows scaled to live 2026-10-09 entry
    # drag/lift ratios; still provisional/uncertified.
    "sts-live-20261011": {
        "atmosphere": "reference-model/kerbin_atmosphere_identified.csv",
        "aero": "reference-model/sts_aero_live20261011.csv",
        "aero_book": None,
        "attitude": "reference-model/stsn_attitude_identified.ini",
        "certified_prior": None,
    },
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

# Profile identity is not a statement of flight certification. In particular,
# sts-provisional has no certified cells and borrows STS-N attitude dynamics.
PROFILE_VEHICLES = {
    "identified": "STS-N",
    "sts-provisional": "STS",
    "sts-live-20261009": "STS",
    "sts-live-20261010": "STS",
    "sts-live-20261011": "STS",
    "fitted-legacy": "STS-N",
    "reference-b": "STS-N",
}


def validate_vehicle_profile(configuration: dict, profile: str | None = None) -> None:
    name = active_profile() if profile is None else profile
    expected = PROFILE_VEHICLES.get(name)
    vehicle = configuration.get("vehicle") if isinstance(configuration, dict) else None
    actual = vehicle.get("modelId") if isinstance(vehicle, dict) else None
    if expected is None:
        raise ValueError(f"Unknown aerodynamic profile {name!r}")
    if actual != expected:
        raise ValueError(
            f"Aerodynamic profile {name!r} describes {expected}, not {actual!r}; "
            "select a vehicle-matched profile explicitly. Cross-vehicle model reuse is blocked."
        )


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
