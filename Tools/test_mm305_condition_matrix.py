#!/usr/bin/env python3
"""Regression checks for the MM305 reproducibility compiler."""
from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory

from generate_mm305_condition_matrix import (
    compile_case,
    inertial_velocity,
    lla_from_position,
    load_ini,
    norm,
    position_from_lla,
)


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "ShuttleSim/scenarios/mm305-hac-staging-ideal-seed-10km-180mps.ini"


def read(path: Path) -> dict[str, str]:
    return load_ini(path)


def main() -> int:
    base = read(BASE)
    base_p = tuple(float(base[k]) for k in ("position_x_m", "position_y_m", "position_z_m"))
    base_v = tuple(float(base[k]) for k in ("velocity_x_mps", "velocity_y_mps", "velocity_z_mps"))
    with TemporaryDirectory() as temp:
        out = Path(temp) / "mm305-hac-repro-h180-a10000-v180.ini"
        compile_case(base, out, 180.0, 10000.0, 180.0, -8.0)
        got = read(out)
        got_p = tuple(float(got[k]) for k in ("position_x_m", "position_y_m", "position_z_m"))
        got_v = tuple(float(got[k]) for k in ("velocity_x_mps", "velocity_y_mps", "velocity_z_mps"))
        assert norm(tuple(a - b for a, b in zip(got_p, base_p))) < 1e-3
        assert norm(tuple(a - b for a, b in zip(got_v, base_v))) < 1e-6
        assert out.name.startswith("mm305-hac-")
        assert got["name"].startswith("mm305-hac-")

        offset_out = Path(temp) / "mm305-hac-repro-h180-a10000-v180-e2k-n-2k.ini"
        compile_case(base, offset_out, 180.0, 10000.0, 180.0, -8.0,
                     east_offset_m=2000.0, north_offset_m=-2000.0)
        offset = read(offset_out)
        offset_p = tuple(float(offset[k]) for k in ("position_x_m", "position_y_m", "position_z_m"))
        assert abs(norm(offset_p) - 610000.0) < 1e-6
        assert norm(tuple(a - b for a, b in zip(offset_p, got_p))) > 1999.0
        assert "east_offset=2000" in offset_out.read_text()
        assert "north_offset=-2000" in offset_out.read_text()

        ut0 = float(got["ut0"])
        p = tuple(float(got[k]) for k in ("position_x_m", "position_y_m", "position_z_m"))
        v = tuple(float(got[k]) for k in ("velocity_x_mps", "velocity_y_mps", "velocity_z_mps"))
        lat, lon = lla_from_position(p, ut0)
        p9 = position_from_lla(lat, lon, 9000.0, ut0)
        v9 = inertial_velocity(p9, ut0, 175.0, 170.0, -8.0)
        assert abs(norm(p9) - 609000.0) < 1e-6
        assert norm(v9) > 0.0
    print("mm305 condition-matrix generator tests: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
