#!/usr/bin/env python3
"""Identify the STS-N rigid-body pitch/roll/yaw plant from live KSP telemetry.

Uses the applied stick (appliedControl.reportedPitch/Roll/Yaw) and body rates
(attitude.bodyPitchRate/bodyRollRate/bodyYawRate, deg/s; bodyPitchRate matches
d(pitch)/dt with slope 1.00 in level flight) logged at ~10 Hz by live KSP
flights.  Angular acceleration is the forward difference of consecutive body
rates within one log.

Models (Q = dynamic pressure in kPa, angles deg, rates deg/s):
  pitch:  qdot = min(Q*Md, Mmax)*dp - Q*Ma*(alpha - alpha_trim) + Mq*q
  roll:   pdot = min(Q*Ld, Lmax)*dr + Lp*p
  yaw:    rdot = min(Q*Nd, Nmax)*dy - Q*Nb*beta + Nr*r

The saturating control power reproduces the observed fall of per-kPa control
power with speed (KSP surface authority / Mach).  Parameters are fitted on
training flights and scored on held-out flights (crc32(log)%4==0), and the
fitted values are written as ShuttleSim direct-plant ini keys.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import zlib
from multiprocessing import Pool

from identify_stsn_aero import _g, _merge, _f, _wls


def _log_samples(path):
    out = []
    with open(path, encoding="utf-8") as h:
        try:
            first = json.loads(h.readline())
        except json.JSONDecodeError:
            return out
        if "shuttlesim" in str(_g(first, "runtime", "transport")):
            return out
        state = {}
        last_ut = None
        for line in h:
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue
            t = r.get("recordType")
            fields = r.get("fields")
            if not isinstance(fields, dict):
                continue
            if t == "vehicleKeyframe":
                state = json.loads(json.dumps(fields))
            elif t == "vehicleDelta":
                _merge(state, fields)
            else:
                continue
            ut = _f(r.get("ut"))
            if ut is None or ut == last_ut:
                continue
            last_ut = ut
            ac = state.get("appliedControl") or {}
            out.append(dict(
                log=os.path.basename(path), ut=ut,
                sit=_g(state, "state", "vesselSituation"),
                ralt=_f(_g(state, "position", "radarAltitude")),
                q=_f(_g(state, "aero", "dynamicPressure")),
                mach=_f(_g(state, "aero", "mach")),
                aoa=_f(_g(state, "attitude", "angleOfAttack")),
                beta=_f(_g(state, "attitude", "sideslip")),
                roll=_f(_g(state, "attitude", "roll")),
                wp=_f(_g(state, "attitude", "bodyPitchRate")),
                wr=_f(_g(state, "attitude", "bodyRollRate")),
                wy=_f(_g(state, "attitude", "bodyYawRate")),
                dp=_f(ac.get("reportedPitch")), dr=_f(ac.get("reportedRoll")),
                dy=_f(ac.get("reportedYaw")),
                thrust=_f(_g(state, "vehicle", "currentThrust"))))
    return out


def pairs(seq, args):
    """Consecutive in-log samples -> (state at k, angular accel k->k+1)."""
    out = []
    for a, b in zip(seq, seq[1:]):
        if a["log"] != b["log"]:
            continue
        dt = b["ut"] - a["ut"]
        if not (0.05 <= dt <= 0.15):
            continue
        if a["sit"] != "flying" or (a["ralt"] or 0) < args.min_radar_alt:
            continue
        if None in (a["q"], a["mach"], a["aoa"], a["beta"], a["wp"], a["wr"], a["wy"],
                    b["wp"], b["wr"], b["wy"], a["dp"], a["dr"], a["dy"]):
            continue
        if a["q"] < args.min_q or a["mach"] > args.max_mach or (a["thrust"] or 0) > 1:
            continue
        s = dict(a)
        s["Q"] = a["q"] / 1000.0
        s["qd"] = (b["wp"] - a["wp"]) / dt
        s["pd"] = (b["wr"] - a["wr"]) / dt
        s["rd"] = (b["wy"] - a["wy"]) / dt
        if max(abs(s["qd"]), abs(s["pd"]), abs(s["rd"])) > args.max_accel:
            continue
        # Departed/tumbling flight is not the controllable plant.
        if max(abs(a["wp"]), abs(a["wr"]), abs(a["wy"])) > args.max_body_rate or abs(a["beta"]) > args.max_sideslip:
            continue
        out.append(s)
    return out


def heldout(log):
    return zlib.crc32(log.encode()) % 4 == 0


def r2(rows, target, pred):
    if not rows:
        return None
    m = sum(target(r) for r in rows) / len(rows)
    ss = sum((target(r) - m) ** 2 for r in rows)
    se = sum((target(r) - pred(r)) ** 2 for r in rows)
    return 1.0 - se / ss if ss > 0 else None


def fit_axis(train, test, name, target, rate, ctrl, stiff_state, cap_grid):
    """Grid over the control-power cap; linear least squares for the rest."""
    best = None
    for cap in cap_grid:
        def basis(r, cap=cap):
            x = [min(r["Q"], cap) * r[ctrl], r[rate]]
            if stiff_state:
                x += [r["Q"] * r[stiff_state], r["Q"]]
            return x
        sol = _wls(train, basis, lambda r: r[target], lambda r: 1.0)
        if sol is None:
            continue

        def pred(r, sol=sol, basis=basis):
            return sum(c * x for c, x in zip(sol, basis(r)))
        score = r2(train, lambda r: r[target], pred)
        if best is None or score > best[0]:
            best = (score, cap, sol, pred)
    score, cap, sol, pred = best
    res = dict(axis=name, q_cap_kpa=cap, control_per_kpa=sol[0], control_max=sol[0] * cap,
               rate_damping=sol[1], train_r2=score,
               heldout_r2=r2(test, lambda r: r[target], pred),
               train_n=len(train), heldout_n=len(test))
    if stiff_state:
        # target = ... + Q*(k*x + c)  ->  k = -stiffness, trim = -c/k
        res["stiffness_per_kpa"] = -sol[2]
        res["trim"] = -sol[3] / sol[2] if sol[2] else None
    # Held-out residual by q band, to show where the fit is weakest.
    bands = []
    for lo, hi in ((0.5, 2), (2, 4), (4, 7), (7, 12), (12, 40)):
        rows = [r for r in test if lo <= r["Q"] < hi]
        if len(rows) >= 100:
            bands.append(dict(q_kpa=[lo, hi], n=len(rows), r2=r2(rows, lambda r: r[target], pred)))
    res["heldout_by_q"] = bands
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--live-logs", required=True)
    ap.add_argument("--base-ini", required=True, help="closed-loop response ini to extend")
    ap.add_argument("--ini", required=True)
    ap.add_argument("--report", required=True)
    ap.add_argument("--min-q", type=float, default=500.0)
    ap.add_argument("--max-mach", type=float, default=0.9)
    ap.add_argument("--min-radar-alt", type=float, default=15.0)
    ap.add_argument("--max-accel", type=float, default=80.0)
    ap.add_argument("--max-body-rate", type=float, default=30.0)
    ap.add_argument("--max-sideslip", type=float, default=4.0)
    ap.add_argument("--jobs", type=int, default=8)
    args = ap.parse_args()

    logs = sorted(glob.glob(os.path.join(args.live_logs, "*vehicle.jsonl")))
    samples = []
    with Pool(args.jobs) as pool:
        for seq in pool.imap(_log_samples, logs, chunksize=4):
            samples.extend(pairs(seq, args))
    train = [s for s in samples if not heldout(s["log"])]
    test = [s for s in samples if heldout(s["log"])]
    # Roll only while near wings-level-to-moderate bank so gravity/turn
    # coupling in the body rates stays small; pitch additionally below 45 deg.
    caps = [x * 0.5 for x in range(2, 41)]
    pitch = fit_axis([s for s in train if abs(s["roll"] or 0) < 45],
                     [s for s in test if abs(s["roll"] or 0) < 45],
                     "pitch", "qd", "wp", "dp", "aoa", caps)
    roll = fit_axis(train, test, "roll", "pd", "wr", "dr", None, caps)
    yaw = fit_axis(train, test, "yaw", "rd", "wy", "dy", "beta", caps)

    d2r = math.pi / 180.0
    lines = [l for l in open(args.base_ini).read().splitlines() if l.strip()]
    lines[0] = lines[0] + (" Direct-control rigid-body coefficients below: "
                           "ShuttleSim/tools/identify_stsn_attitude.py (held-out R2 in the report).")
    # The direct plant uses q in kPa, angular accelerations in deg/s^2 keys,
    # stiffness in 1/s^2 per kPa and damping as a ratio of the stiffness mode.
    wn2 = pitch["stiffness_per_kpa"] * 4.0  # representative landing q ~4 kPa
    zeta = max(0.0, -pitch["rate_damping"]) / (2.0 * math.sqrt(wn2)) if wn2 > 0 else 0.0
    lines += [
        "direct_pitch_accel_per_kpa_deg=%.4f" % pitch["control_per_kpa"],
        "direct_pitch_accel_max_deg=%.4f" % pitch["control_max"],
        "direct_pitch_stiffness_per_kpa=%.5f" % pitch["stiffness_per_kpa"],
        "direct_trim_aoa_deg=%.3f" % pitch["trim"],
        "direct_pitch_damping_ratio=%.4f" % zeta,
        "direct_roll_accel_per_kpa_deg=%.4f" % roll["control_per_kpa"],
        "direct_roll_accel_max_deg=%.4f" % roll["control_max"],
        "direct_roll_damping_s_inv=%.4f" % max(0.0, -roll["rate_damping"]),
        "direct_yaw_accel_per_kpa_deg=%.4f" % yaw["control_per_kpa"],
        "direct_yaw_accel_max_deg=%.4f" % yaw["control_max"],
        "direct_yaw_stiffness_per_kpa=%.5f" % yaw["stiffness_per_kpa"],
    ]
    with open(args.ini, "w") as f:
        f.write("\n".join(lines) + "\n")
    report = dict(tool="ShuttleSim/tools/identify_stsn_attitude.py", live_logs=args.live_logs,
                  samples=len(samples), train=len(train), heldout=len(test),
                  settings=vars(args), pitch=pitch, roll=roll, yaw=yaw,
                  pitch_damping_ratio_at_4kpa=zeta, units="deg, deg/s, deg/s^2, kPa")
    with open(args.report, "w") as f:
        json.dump(report, f, indent=1)
    for ax in (pitch, roll, yaw):
        print("%-5s cap %.1f kPa  ctrl %.2f deg/s2/kPa (max %.1f)  damp %.3f  stiff %s trim %s  R2 train %.2f held-out %.2f  %s" % (
            ax["axis"], ax["q_cap_kpa"], ax["control_per_kpa"], ax["control_max"], ax["rate_damping"],
            "%.3f" % ax["stiffness_per_kpa"] if "stiffness_per_kpa" in ax else "-",
            "%.2f" % ax["trim"] if ax.get("trim") is not None else "-",
            ax["train_r2"], ax["heldout_r2"] or float("nan"),
            " ".join("[%g-%g kPa n%d R2 %.2f]" % (b["q_kpa"][0], b["q_kpa"][1], b["n"], b["r2"]) for b in ax["heldout_by_q"])))


if __name__ == "__main__":
    main()
