#!/usr/bin/env python3
"""Physically consistent Kerbin orbit scenario generator for the STS deorbit campaign.

Keplerian elements are converted to a Kerbin-centred *inertial* Cartesian state in
exactly the frame ShuttleSim uses (world.c): body-fixed -> inertial is a +z rotation
by  pi/2 + omega_K * UT.  Constants below mirror world_seed_kerbin().

Plane placement is not a rotated velocity vector.  For a requested inclination, the
right ascension of the ascending node is solved so that the orbital plane passes
the KSC runway-09 threshold (at the rotating-body position it has at UT_pass) with a
requested signed cross-range offset, and the argument of latitude is chosen so the
vehicle is over that point at UT_pass.  UT_pass = ut0 + pass_delay_s, which is the
orbital-phase / deorbit-timing knob.  `branch` selects whether the vehicle crosses
the site heading north (ascending) or south (descending) -- both exist for every
non-equatorial inclination.

Everything here is geometry only: no guidance, no aerodynamics, no reachability claim.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import pathlib

MU = 3.5316e12
RADIUS = 600000.0
ROT_RATE = 2.0 * math.pi / 21549.425
ROT_PHASE0 = math.pi / 2.0
SITE_LAT_DEG = -0.0486111111
SITE_LON_DEG = -74.7283333333
SITE_ALT_M = 70.0
SITE_HEADING_DEG = 90.0

# STS (modelId "STS") pre-burn orbital mass/engine, from archived live logs:
# 26147 kg measured; 2 x AJ10.v2 at 30 kN available each.  Isp ~285 s is the
# mean of four archived OMS-only burns (281.6-291.6 s, mass-drop vs F*dt).
STS_MASS_KG = 26146.85
STS_OMS_THRUST_N = 60000.0
STS_OMS_ISP_S = 285.0


def rotz(v, a):
    c, s = math.cos(a), math.sin(a)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])


def dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def norm(a): return math.sqrt(dot(a, a))
def scale(a, k): return (a[0] * k, a[1] * k, a[2] * k)
def add(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def unit(a): return scale(a, 1.0 / norm(a))


def site_inertial(ut, alt_m=0.0):
    """Runway threshold inertial position (spherical Kerbin, as ShuttleSim)."""
    lat, lon = math.radians(SITE_LAT_DEG), math.radians(SITE_LON_DEG)
    r = RADIUS + alt_m
    fixed = (r * math.cos(lat) * math.cos(lon), r * math.cos(lat) * math.sin(lon), r * math.sin(lat))
    return rotz(fixed, ROT_PHASE0 + ROT_RATE * ut)


def kepler_solve(M, e):
    M = math.fmod(M, 2 * math.pi)
    E = M if e < 0.8 else math.pi
    for _ in range(60):
        d = (E - e * math.sin(E) - M) / (1 - e * math.cos(E))
        E -= d
        if abs(d) < 1e-14:
            break
    return E


def true_from_mean(M, e):
    E = kepler_solve(M, e)
    return 2 * math.atan2(math.sqrt(1 + e) * math.sin(E / 2), math.sqrt(1 - e) * math.cos(E / 2))


def mean_from_true(nu, e):
    E = 2 * math.atan2(math.sqrt(1 - e) * math.sin(nu / 2), math.sqrt(1 + e) * math.cos(nu / 2))
    return E - e * math.sin(E)


def elements_to_state(a, e, inc, raan, argp, nu):
    p = a * (1 - e * e)
    r = p / (1 + e * math.cos(nu))
    rp = (r * math.cos(nu), r * math.sin(nu), 0.0)
    vp = (-math.sqrt(MU / p) * math.sin(nu), math.sqrt(MU / p) * (e + math.cos(nu)), 0.0)

    def rot(v):
        v = rotz(v, argp)
        c, s = math.cos(inc), math.sin(inc)
        v = (v[0], c * v[1] - s * v[2], s * v[1] + c * v[2])
        return rotz(v, raan)
    return rot(rp), rot(vp)


def state_to_elements(pos, vel):
    r, v = norm(pos), norm(vel)
    h = cross(pos, vel)
    hn = norm(h)
    energy = v * v / 2 - MU / r
    a = -MU / (2 * energy)
    evec = sub(scale(cross(vel, h), 1 / MU), scale(pos, 1 / r))
    e = norm(evec)
    inc = math.acos(max(-1.0, min(1.0, h[2] / hn)))
    node = (-h[1], h[0], 0.0)
    raan = math.atan2(node[1], node[0]) % (2 * math.pi) if norm(node) > 1e-9 else 0.0
    nu = math.atan2(dot(cross(evec, pos), unit(h)), dot(evec, pos)) % (2 * math.pi) if e > 1e-9 else 0.0
    return {"a_m": a, "e": e, "i_deg": math.degrees(inc), "raan_deg": math.degrees(raan),
            "energy_j_kg": energy, "h_m2_s": hn, "rp_alt_m": a * (1 - e) - RADIUS,
            "ra_alt_m": a * (1 + e) - RADIUS, "period_s": 2 * math.pi * math.sqrt(a ** 3 / MU)}


def plane_normal(inc, raan):
    return (math.sin(inc) * math.sin(raan), -math.sin(inc) * math.cos(raan), math.cos(inc))


def solve_raan(inc, site_hat, crossrange_deg, branch):
    """RAAN such that n . site_hat = sin(crossrange).  Returns (raan, achieved_crossrange_rad, clipped)."""
    lat_s = math.asin(site_hat[2])
    theta = math.atan2(site_hat[1], site_hat[0])
    if math.sin(inc) < 1e-9:
        return 0.0, math.asin(site_hat[2]) * (1 if math.cos(inc) > 0 else -1), True
    x = (math.sin(math.radians(crossrange_deg)) - math.cos(inc) * math.sin(lat_s)) / (math.sin(inc) * math.cos(lat_s))
    clipped = abs(x) > 1.0
    x = max(-1.0, min(1.0, x))
    d = math.asin(x)
    # branch "asc": site is crossed while moving north.
    delta = d if branch == "asc" else math.pi - d
    raan = (theta + delta) % (2 * math.pi)
    n = plane_normal(inc, raan)
    return raan, math.asin(max(-1.0, min(1.0, dot(n, site_hat)))), clipped


def generate(inc_deg, rp_alt_km, ra_alt_km, argp_deg=0.0, pass_delay_s=1200.0, crossrange_deg=0.0,
             branch="asc", ut0=10000.0, mass_kg=STS_MASS_KG):
    """Return (scenario dict, geometry/validation dict)."""
    rp_alt_km, ra_alt_km = min(rp_alt_km, ra_alt_km), max(rp_alt_km, ra_alt_km)
    rp = RADIUS + rp_alt_km * 1000.0
    ra = RADIUS + ra_alt_km * 1000.0
    a = (rp + ra) / 2
    e = (ra - rp) / (ra + rp)
    inc = math.radians(inc_deg)
    n_mean = math.sqrt(MU / a ** 3)
    ut_pass = ut0 + pass_delay_s
    site = site_inertial(ut_pass)
    site_hat = unit(site)
    raan, achieved_cross, clipped = solve_raan(inc, site_hat, crossrange_deg, branch)
    if math.sin(inc) < 1e-9:
        raan = 0.0
    n = plane_normal(inc, raan)
    # In-plane angle of the site's projection measured from the ascending node.
    node = (math.cos(raan), math.sin(raan), 0.0)
    ip = unit(sub(site_hat, scale(n, dot(n, site_hat))))
    u_site = math.atan2(dot(cross(node, ip), n), dot(node, ip))
    argp = math.radians(argp_deg)
    nu_pass = (u_site - argp) % (2 * math.pi)
    M_pass = mean_from_true(nu_pass, e)
    M0 = M_pass - n_mean * pass_delay_s
    nu0 = true_from_mean(M0, e)
    pos, vel = elements_to_state(a, e, inc, raan, argp, nu0)
    el = state_to_elements(pos, vel)
    geo = {
        "requested": {"inc_deg": inc_deg, "rp_alt_km": rp_alt_km, "ra_alt_km": ra_alt_km, "argp_deg": argp_deg,
                      "pass_delay_s": pass_delay_s, "crossrange_deg": crossrange_deg, "branch": branch,
                      "ut0": ut0, "mass_kg": mass_kg},
        "raan_deg": math.degrees(raan), "nu0_deg": math.degrees(nu0) % 360.0,
        "achieved_pass_crossrange_deg": math.degrees(achieved_cross),
        "achieved_pass_crossrange_km": math.radians(math.degrees(achieved_cross)) * RADIUS / 1000.0,
        "crossrange_clipped": clipped,
        "ut_pass": ut_pass,
        "elements_from_state": el,
    }
    geo["validation"] = validate(geo, pos, vel, a, e, inc_deg, rp_alt_km, ra_alt_km)
    scenario = {"ut0": ut0, "position": pos, "velocity": vel, "mass_kg": mass_kg}
    return scenario, geo


def validate(geo, pos, vel, a, e, inc_deg, rp_km, ra_km):
    el = geo["elements_from_state"]
    h_expect = math.sqrt(MU * a * (1 - e * e))
    checks = {
        "inclination_err_deg": el["i_deg"] - inc_deg,
        "eccentricity_err": el["e"] - e,
        "rp_alt_err_m": el["rp_alt_m"] - rp_km * 1000.0,
        "ra_alt_err_m": el["ra_alt_m"] - ra_km * 1000.0,
        "energy_err_j_kg": el["energy_j_kg"] - (-MU / (2 * a)),
        "angular_momentum_rel_err": el["h_m2_s"] / h_expect - 1.0,
    }
    ok = (abs(checks["inclination_err_deg"]) < 1e-6 and abs(checks["eccentricity_err"]) < 1e-9 and
          abs(checks["rp_alt_err_m"]) < 1e-3 and abs(checks["ra_alt_err_m"]) < 1e-3 and
          abs(checks["energy_err_j_kg"]) < 1e-4 and abs(checks["angular_momentum_rel_err"]) < 1e-10 and
          rp_km * 1000.0 > 0)
    return {"ok": ok, **checks}


def write_ini(path, name, scenario, extra=None):
    pos, vel = scenario["position"], scenario["velocity"]
    lines = [
        f"name={name}", f"ut0={scenario['ut0']!r}",
        f"position_x_m={pos[0]!r}", f"position_y_m={pos[1]!r}", f"position_z_m={pos[2]!r}",
        f"velocity_x_mps={vel[0]!r}", f"velocity_y_mps={vel[1]!r}", f"velocity_z_mps={vel[2]!r}",
        f"mass_kg={scenario['mass_kg']!r}", "initial_aoa_deg=0", "initial_bank_deg=0",
        f"orbital_engine_available_thrust_n={STS_OMS_THRUST_N}",
        f"orbital_engine_isp_s={STS_OMS_ISP_S}",
        f"runway_latitude_deg={SITE_LAT_DEG}", f"runway_longitude_deg={SITE_LON_DEG}",
        f"runway_elevation_m={SITE_ALT_M}", f"runway_heading_deg={SITE_HEADING_DEG}",
        "runway_length_m=2500", "runway_width_m=70",
    ]
    pathlib.Path(path).write_text("# Generated by tools/orbit_scenarios.py (geometry only)\n" + "\n".join(lines) + "\n")


def scenario_id(params: dict) -> str:
    blob = json.dumps(params, sort_keys=True).encode()
    return hashlib.sha256(blob).hexdigest()[:10]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--inc", type=float, required=True)
    ap.add_argument("--rp", type=float, default=86.0)
    ap.add_argument("--ra", type=float, default=86.0)
    ap.add_argument("--argp", type=float, default=0.0)
    ap.add_argument("--delay", type=float, default=1200.0)
    ap.add_argument("--cross", type=float, default=0.0)
    ap.add_argument("--branch", choices=["asc", "desc"], default="asc")
    ap.add_argument("--ut0", type=float, default=10000.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    sc, geo = generate(a.inc, a.rp, a.ra, a.argp, a.delay, a.cross, a.branch, a.ut0)
    write_ini(a.out, pathlib.Path(a.out).stem, sc)
    print(json.dumps(geo, indent=1))


if __name__ == "__main__":
    main()
