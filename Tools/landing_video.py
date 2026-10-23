#!/usr/bin/env python3
"""Produce a KSL landing video: crop a macOS KSP screen recording and burn in a telemetry HUD.

The HUD is replayed from the run's recorded logs and synced to the recording by wall clock:
  * runs/<id>/telemetry.jsonl   phase, Mach, range, runway-frame position, guidance status
  * runs/<id>/events.jsonl      phase changes and guidance decisions
  * FlightLogs/*-vehicle.jsonl  true surface-frame pitch/heading/roll and inertial state (10 Hz)

HDR10 (PQ/BT.2020) recordings stay 10-bit PQ end to end; the HUD is converted from sRGB into PQ
at the recording's graphics white. SDR recordings are composited as plain BT.709.

Outputs <name>-HUD.mp4 and <name>-clean.mp4 (cropped, no HUD) next to the recording by default.

  python3 Tools/landing_video.py "~/Desktop/Screen Recording ....mov"
  python3 Tools/landing_video.py REC.mov --preview 800 1290 1555     # SDR JPG stills only
  python3 Tools/landing_video.py REC.mov --run runs/<id> --start-epoch 1792643698

Needs ffmpeg/ffprobe (VideoToolbox HEVC encoder) and `pip install -e .[video]` (Pillow, numpy).
"""

from __future__ import annotations

import argparse
import bisect
import calendar
import json
import math
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
SITE_LAT, SITE_LON = -0.0486111111, -74.7283333333  # KSC runway 09 (Configuration site)


# ---------------------------------------------------------------------------------------------
# Recording metadata, sync and crop

def ffprobe(path: Path) -> dict:
    out = subprocess.run(["ffprobe", "-v", "error", "-print_format", "json", "-show_format", "-show_streams",
                          str(path)], check=True, capture_output=True, text=True)
    info = json.loads(out.stdout)
    video = next(s for s in info["streams"] if s["codec_type"] == "video")
    cll = next((d for d in video.get("side_data_list", []) if d.get("side_data_type") == "Content light level metadata"), {})
    return {"width": video["width"], "height": video["height"], "duration": float(info["format"]["duration"]),
            "transfer": video.get("color_transfer", "bt709"), "max_cll": float(cll.get("max_content") or 0),
            "has_audio": any(s["codec_type"] == "audio" for s in info["streams"])}


def recording_start_epoch(path: Path) -> float:
    """Wall-clock start from the macOS screen-recording file name, else the file's birth time."""
    name = path.name
    m = re.search(r"(\d{4})-(\d{2})-(\d{2}) (오전|오후) (\d{1,2})\.(\d{2})\.(\d{2})", name)
    if m:
        y, mo, d, ampm, h, mi, s = m.groups()
        h = int(h) % 12 + (12 if ampm == "오후" else 0)
    else:
        m = re.search(r"(\d{4})-(\d{2})-(\d{2}) at (\d{1,2})\.(\d{2})\.(\d{2})\s*(AM|PM)", name, re.I)
        if not m:
            return os.stat(path).st_birthtime
        y, mo, d, h, mi, s, ampm = m.groups()
        h = int(h) % 12 + (12 if ampm.upper() == "PM" else 0)
    return time.mktime((int(y), int(mo), int(d), h, int(mi), int(s), 0, 0, -1))


def gray_frame(path: Path, t: float, w: int, h: int) -> np.ndarray:
    raw = subprocess.run(["ffmpeg", "-v", "error", "-ss", f"{t:.2f}", "-i", str(path), "-frames:v", "1",
                          "-f", "rawvideo", "-pix_fmt", "gray", "pipe:1"], check=True, capture_output=True).stdout
    return np.frombuffer(raw, np.uint8).reshape(h, w)


def detect_crop(path: Path, meta: dict, inset: int = 10) -> tuple[int, int, int, int]:
    """Window content box (w, h, x, y): non-black window bounds minus the title bar and corners."""
    w, h, dur = meta["width"], meta["height"], meta["duration"]
    frames = [gray_frame(path, dur * f, w, h) for f in (0.2, 0.4, 0.6, 0.8)]
    lit = np.max([f > 12 for f in frames], axis=0)
    rows, cols = np.nonzero(lit.any(axis=1))[0], np.nonzero(lit.any(axis=0))[0]
    top, bottom, left, right = rows[0], rows[-1], cols[0], cols[-1]
    # The title bar ends at a dark separator line; content starts right below it.
    col = np.median([f[:, (left + right) // 2] for f in frames], axis=0)
    band = [y for y in range(top + 20, min(top + 140, bottom)) if col[y] < 8]
    content_top = (band[-1] + 1) if band else top + 57
    x0, y0, x1, y1 = left + inset, content_top + inset, right - inset, bottom - inset
    cw, ch = (x1 - x0) // 2 * 2, (y1 - y0) // 2 * 2
    return cw, ch, x0, y0


# ---------------------------------------------------------------------------------------------
# Flight data

def read_jsonl(path: Path):
    with open(path) as f:
        for line in f:
            if line.strip():
                yield json.loads(line)


def jsonl_time_range(path: Path) -> tuple[float, float]:
    with open(path, "rb") as f:
        first = json.loads(f.readline())
        f.seek(max(0, os.path.getsize(path) - 200_000))
        last = json.loads(f.read().splitlines()[-1])
    return first["time"], last["time"]


def find_run(start: float, end: float) -> Path:
    best, overlap = None, 0.0
    for tel in (ROOT / "runs").glob("*/telemetry.jsonl"):
        try:
            a, b = jsonl_time_range(tel)
        except (ValueError, KeyError, IndexError):
            continue
        o = min(b, end) - max(a, start)
        if o > overlap:
            best, overlap = tel.parent, o
    if best is None:
        sys.exit("no run in runs/ overlaps the recording; pass --run")
    return best


def find_vehicle_log(run_start: float) -> Path | None:
    best, gap = None, 120.0
    for p in (ROOT / "FlightLogs").glob("*-vehicle.jsonl"):
        m = re.match(r"(\d{4}-\d{2}-\d{2})T(\d{2})-(\d{2})-(\d{2})Z", p.name)
        if not m:
            continue
        t = calendar.timegm(time.strptime(f"{m[1]} {m[2]}:{m[3]}:{m[4]}", "%Y-%m-%d %H:%M:%S"))
        if abs(t - run_start) < gap:
            best, gap = p, abs(t - run_start)
    return best


@dataclass
class Attitude:
    """Vehicle-log attitude and surface-relative velocity direction, keyed by wall time."""
    t: np.ndarray
    pitch: np.ndarray
    heading: np.ndarray
    roll: np.ndarray
    vel_heading: np.ndarray
    vel_fpa: np.ndarray


def load_attitude(path: Path, telemetry: list[dict]) -> Attitude | None:
    planet, rows, pos, vel = None, [], None, None
    for r in read_jsonl(path):
        if r.get("recordType") == "sessionStart":
            planet = r["planet"]
            continue
        fields = r.get("fields", {})
        pos = fields.get("position", {}).get("inertial", pos)
        vel = fields.get("motion", {}).get("velocity", vel)
        a = fields.get("attitude")
        if a and "ut" in r and pos and vel:
            rows.append((r["ut"], a["pitch"], a["heading"], a["roll"], *pos, *vel))
    if not rows or not planet:
        return None
    d = np.array(rows)
    n = np.array(planet["northAxis"], float)
    n /= np.linalg.norm(n)
    r, v = d[:, 4:7], d[:, 7:10]
    v_surf = v - planet["rotationalSpeed"] * np.cross(n, r)  # inertial -> surface-relative
    up = r / np.linalg.norm(r, axis=1, keepdims=True)
    east = np.cross(n, up)
    east /= np.linalg.norm(east, axis=1, keepdims=True)
    north = np.cross(up, east)
    ve, vn, vu = (np.sum(v_surf * e, axis=1) for e in (east, north, up))
    vel_heading = np.degrees(np.arctan2(ve, vn)) % 360
    vel_fpa = np.degrees(np.arctan2(vu, np.hypot(ve, vn)))
    # vehicle log is keyed by UT; telemetry pairs UT with wall time
    tu = np.array([s["ut"] for s in telemetry]); tt = np.array([s["time"] for s in telemetry])
    t = np.interp(d[:, 0], tu, tt)
    return Attitude(t, d[:, 1], d[:, 2], d[:, 3], vel_heading, vel_fpa)


def collect_decisions(events: list[dict]) -> list[tuple[float, str, bool]]:
    """Phase changes and guidance verdicts; repeats collapsed. (time, text, is_warning)."""
    out: list[tuple[float, str, bool]] = []
    for e in events:
        m, warn = e["message"], False
        if e["kind"] == "backend-phase":
            m = m.removeprefix("backend phase: ")
            if m in ("Idle", "Planning"):
                continue
        elif e["kind"] in ("guidance-degraded", "milestone"):
            warn = e["kind"] == "guidance-degraded" or "executable=False" in m
        elif e["kind"] == "log" and any(k in m for k in ("cannot reach", "not certified", "inhibited", "abort")):
            m, warn = m.split(" (")[0].split(". ")[0], True
        else:
            continue
        if out and out[-1][1][:30] == m[:30]:
            continue
        out.append((e["time"], m if len(m) <= 78 else m[:77] + "…", warn))
    return out


# ---------------------------------------------------------------------------------------------
# HUD

def font(names: list[str], size: int):
    for n in names:
        try:
            return ImageFont.truetype(n, size)
        except OSError:
            continue
    return ImageFont.load_default(size)


MONO = ["/System/Library/Fonts/SFNSMono.ttf", "/System/Library/Fonts/Menlo.ttc", "DejaVuSansMono.ttf"]
SANS = ["/System/Library/Fonts/SFNS.ttf", "/System/Library/Fonts/Helvetica.ttc", "DejaVuSans.ttf"]
PANEL = (0, 0, 0, 130)
WHITE, DIM, ACC, WARN, GREEN = (255, 255, 255, 255), (200, 205, 210, 255), (120, 210, 255, 255), (255, 195, 90, 255), (130, 235, 150, 255)


def wrap180(a):
    return (a + 180) % 360 - 180


class Hud:
    def __init__(self, W: int, H: int, telemetry: list[dict], decisions, attitude: Attitude | None):
        self.W, self.H = W, H
        self.S = [s for s in telemetry if s.get("situation") != "Unknown"]
        self.ts = [s["time"] for s in self.S]
        self.dec, self.att = decisions, attitude
        k = H / 1200
        self.k = k
        self.f_s, self.f_m, self.f_l = (font(MONO, round(n * k)) for n in (15, 19, 26))
        self.sans = font(SANS, round(16 * k))
        lon_u, prev, off = [], None, 0.0
        for s in self.S:
            lo = s["longitude"]
            if prev is not None:
                off += 360 if lo - prev < -180 else -360 if lo - prev > 180 else 0
            lon_u.append(lo + off)
            prev = lo
        self.lon_u = lon_u
        self.site_lon = SITE_LON + off

    def sample(self, t: float) -> dict:
        S = self.S
        i = max(0, min(bisect.bisect_right(self.ts, t) - 1, len(S) - 2))
        a, b = S[i], S[i + 1]
        f = max(0.0, min(1.0, (t - a["time"]) / max(1e-6, b["time"] - a["time"])))
        d = {k: a[k] + (b[k] - a[k]) * f for k in ("altitude", "speed", "mach", "rangeToSite", "runwayAlongTrack",
                                                    "runwayCrossTrack", "latitude", "angleOfAttack", "flightPathAngle")}
        d["lon_u"] = self.lon_u[i] + (self.lon_u[i + 1] - self.lon_u[i]) * f
        d["i"], d["raw"] = i, (a if f < 0.5 else b)
        if self.att is not None:
            A = self.att
            interp = lambda arr: float(np.interp(t, A.t, arr))
            ang = lambda arr: float(np.degrees(np.arctan2(np.interp(t, A.t, np.sin(np.radians(arr))),
                                                          np.interp(t, A.t, np.cos(np.radians(arr))))) % 360)
            d.update(pitch=interp(A.pitch), heading=ang(A.heading), roll=wrap180(ang(A.roll)),
                     vel_heading=ang(A.vel_heading), vel_fpa=interp(A.vel_fpa))
        else:  # telemetry only: valid while the nose is near the velocity vector
            d.update(pitch=d["flightPathAngle"] + d["angleOfAttack"], heading=a["heading"], roll=a["roll"],
                     vel_heading=a["heading"], vel_fpa=d["flightPathAngle"])
        return d

    def navball(self, img: Image.Image, cx: int, cy: int, R: int, d: dict):
        ppd = R / 45.0
        n = R * 4
        c = n // 2
        ball = Image.new("RGBA", (n, n))
        b = ImageDraw.Draw(ball)
        hy = c + d["pitch"] * ppd  # horizon; elevation e sits at hy - e*ppd
        b.rectangle((0, 0, n, hy), fill=(45, 115, 185, 255))
        b.rectangle((0, hy, n, n), fill=(135, 90, 50, 255))
        b.line((0, hy, n, hy), fill=WHITE, width=2)
        for p in range(-60, 70, 20):
            if p:
                b.line((c - R * .25, hy - p * ppd, c + R * .25, hy - p * ppd), fill=(255, 255, 255, 170), width=2)
        # velocity marker in the same horizon frame: prograde, or retrograde when it is behind the nose
        dh, e, retro = wrap180(d["vel_heading"] - d["heading"]), d["vel_fpa"], False
        if abs(dh) > 90:
            dh, e, retro = wrap180(dh + 180), -e, True
        mx, my = c + dh * ppd, hy - e * ppd
        rr = 8 * self.k
        b.ellipse((mx - rr, my - rr, mx + rr, my + rr), outline=GREEN if not retro else WARN, width=3)
        if retro:
            b.line((mx - rr * .7, my - rr * .7, mx + rr * .7, my + rr * .7), fill=WARN, width=2)
            b.line((mx - rr * .7, my + rr * .7, mx + rr * .7, my - rr * .7), fill=WARN, width=2)
        ball = ball.rotate(d["roll"], resample=Image.BICUBIC, center=(c, c)).crop((c - R, c - R, c + R, c + R))
        mask = Image.new("L", (2 * R, 2 * R))
        ImageDraw.Draw(mask).ellipse((0, 0, 2 * R - 1, 2 * R - 1), fill=255)
        img.paste(ball, (cx - R, cy - R), mask)
        dr = ImageDraw.Draw(img)
        dr.ellipse((cx - R, cy - R, cx + R, cy + R), outline=(255, 255, 255, 200), width=2)
        w = R * .42
        dr.line((cx - w, cy, cx - w / 3, cy, cx, cy + w / 5, cx + w / 3, cy, cx + w, cy), fill=WARN, width=4)
        dr.text((cx, cy - R - 16 * self.k), f'{d["heading"]:03.0f}°', font=self.f_m, fill=WHITE, anchor="mm")

    def frame(self, t: float) -> Image.Image:
        W, H, k = self.W, self.H, self.k
        img = Image.new("RGBA", (W, H), (0, 0, 0, 0))
        if t < self.S[0]["time"]:
            return img
        d = self.sample(t)
        raw = d["raw"]
        dr = ImageDraw.Draw(img)
        m = round(24 * k)
        # phase + mission clock, stacked so long phase names never collide with the clock
        met = t - self.S[0]["time"]
        phase = (raw.get("backendPhase") or raw["phase"]).upper()
        pw = max(dr.textlength(phase, font=self.f_l), 120 * k) + 36 * k
        dr.rounded_rectangle((m, m, m + pw, m + 80 * k), 12, fill=PANEL)
        dr.text((m + 18 * k, m + 28 * k), phase, font=self.f_l, fill=ACC, anchor="lm")
        dr.text((m + 18 * k, m + 60 * k), f"T+{int(met) // 60:02d}:{int(met) % 60:02d}", font=self.f_m, fill=DIM, anchor="lm")
        # navball + four numbers
        R = round(105 * k)
        cx, cy = m + round(30 * k) + R, H - m - round(30 * k) - R
        dr.rounded_rectangle((m, cy - R - 44 * k, cx + R + 200 * k, H - m), 12, fill=PANEL)
        self.navball(img, cx, cy, R, d)
        alt = d["altitude"]
        rows = (("ALT", f"{alt / 1000:.1f} km" if alt >= 1000 else f"{alt:.0f} m"), ("SPD", f'{d["speed"]:.0f} m/s'),
                ("MACH", f'{d["mach"]:.2f}'), ("RNG", f'{d["rangeToSite"] / 1000:.1f} km'))
        for j, (label, value) in enumerate(rows):
            y = cy - R + 8 * k + j * 52 * k
            dr.text((cx + R + 30 * k, y), label, font=self.sans, fill=DIM)
            dr.text((cx + R + 30 * k, y + 18 * k), value, font=self.f_l, fill=WHITE)
        self.minimap(img, d)
        # latest decisions, newest at the bottom
        past = [e for e in self.dec if e[0] <= t][-3:]
        for j, (et, msg, warn) in enumerate(past):
            newest = j == len(past) - 1
            alpha = 255 if newest else 150
            col = (WARN if warn else WHITE)[:3] + (alpha,)
            y = H - m - (len(past) - j) * 34 * k
            x = W - m - dr.textlength(msg, font=self.f_s) - 90 * k
            fresh = newest and t - et < 10
            dr.rounded_rectangle((x, y, W - m, y + 28 * k), 8, fill=(0, 0, 0, 170 if fresh else 110),
                                 outline=col[:3] + (200,) if fresh else None)
            et_met = et - self.S[0]["time"]
            dr.text((x + 12 * k, y + 14 * k), f"{int(et_met) // 60:02d}:{int(et_met) % 60:02d}", font=self.f_s,
                    fill=DIM[:3] + (alpha,), anchor="lm")
            dr.text((x + 72 * k, y + 14 * k), msg, font=self.f_s, fill=col, anchor="lm")
        return img

    def minimap(self, img: Image.Image, d: dict):
        """North-up: global ground track, then runway frame (crosstrack + is right of RW09 = south)."""
        k, S = self.k, self.S
        mw, mh = round(340 * k), round(200 * k)
        X, Y = self.W - round(24 * k) - mw, round(24 * k)
        layer = Image.new("RGBA", (mw, mh), PANEL)
        md = ImageDraw.Draw(layer)
        if d["rangeToSite"] > 45000:
            lo0, lo1 = self.lon_u[0] - 3, self.site_lon + 6
            sc = (mw - 30 * k) / (lo1 - lo0)
            P = lambda la, lo: (15 * k + (lo - lo0) * sc, mh / 2 - la * 40 * sc)  # latitude exaggerated x40
            pts = [P(s["latitude"], self.lon_u[j]) for j, s in enumerate(S)]
            md.line(pts, fill=(255, 255, 255, 60), width=2)
            md.line(pts[:d["i"] + 1] + [P(d["latitude"], d["lon_u"])], fill=ACC, width=3)
            sx, sy = P(SITE_LAT, self.site_lon)
            md.rectangle((sx - 5, sy - 5, sx + 5, sy + 5), outline=WARN, width=2)
            here, label = P(d["latitude"], d["lon_u"]), "GROUND TRACK · N↑ lat×40"
        else:
            span = 75000 if d["rangeToSite"] > 15000 else 18000 if d["rangeToSite"] > 4000 else 6000
            sc = (mw - 30 * k) / span
            ox, oy = mw * 0.8, mh / 2
            P = lambda along, cross: (ox + along * sc, oy + cross * sc)
            md.line([(ox - 1250 * sc, oy), (ox + 1250 * sc, oy)], fill=WARN, width=4)
            tr = [P(s["runwayAlongTrack"], s["runwayCrossTrack"]) for s in S[:d["i"] + 1] if s["rangeToSite"] < span * 1.5]
            if len(tr) > 1:
                md.line(tr, fill=ACC, width=3)
            here, label = P(d["runwayAlongTrack"], d["runwayCrossTrack"]), f"RUNWAY 09 · N↑ · {span / 1000:.0f} km"
        hx, hy = max(8, min(mw - 8, here[0])), max(8, min(mh - 8, here[1]))
        md.ellipse((hx - 6, hy - 6, hx + 6, hy + 6), fill=WHITE)
        md.text((12 * k, 10 * k), label, font=self.sans, fill=DIM)
        rounded = Image.new("L", (mw, mh))
        ImageDraw.Draw(rounded).rounded_rectangle((0, 0, mw - 1, mh - 1), 12, fill=255)
        layer.putalpha(Image.fromarray(np.minimum(np.asarray(layer.getchannel("A")), np.asarray(rounded))))
        img.alpha_composite(layer, (X, Y))


# ---------------------------------------------------------------------------------------------
# sRGB HUD -> PQ BT.2020

M709_2020 = np.array([[0.6274, 0.3293, 0.0433], [0.0691, 0.9195, 0.0114], [0.0164, 0.0880, 0.8956]], np.float32)
_s = np.arange(256) / 255.0
SRGB_LINEAR = np.where(_s <= 0.04045, _s / 12.92, ((_s + 0.055) / 1.055) ** 2.4).astype(np.float32)


def pq_encode(nits_fraction: np.ndarray) -> np.ndarray:
    m1, m2, c1, c2, c3 = 0.1593017578125, 78.84375, 0.8359375, 18.8515625, 18.6875
    lp = np.power(np.clip(nits_fraction, 0, 1), m1)
    return np.power((c1 + c2 * lp) / (1 + c3 * lp), m2)


def to_pq_rgba64(img: Image.Image, white_nits: float) -> bytes:
    a = np.asarray(img)
    out = np.zeros(a.shape, np.uint16)
    sel = a[..., 3] > 0
    px = a[sel]
    # a HUD frame has few distinct colors: convert each once
    key = px[:, 0].astype(np.uint32) << 16 | px[:, 1].astype(np.uint32) << 8 | px[:, 2]
    uniq, inv = np.unique(key, return_inverse=True)
    rgb = np.stack([(uniq >> 16) & 255, (uniq >> 8) & 255, uniq & 255], 1)
    conv = (pq_encode((SRGB_LINEAR[rgb] @ M709_2020.T) * (white_nits / 10000.0)) * 65535 + 0.5).astype(np.uint16)
    o = np.empty((len(px), 4), np.uint16)
    o[:, :3] = conv[inv.ravel()]
    o[:, 3] = px[:, 3].astype(np.uint16) * 257
    out[sel] = o
    return out.tobytes()


# ---------------------------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording", type=Path)
    ap.add_argument("--run", type=Path, help="run directory (default: the run overlapping the recording)")
    ap.add_argument("--vehicle-log", type=Path, help="FlightLogs/*-vehicle.jsonl (default: matched to the run)")
    ap.add_argument("--start-epoch", type=float, help="recording start, Unix seconds (default: from the file name)")
    ap.add_argument("--offset", type=float, default=0.0, help="extra seconds added to the telemetry sync")
    ap.add_argument("--crop", help="W:H:X:Y window content box, or 'none' for an already-cropped input (default: auto-detect)")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--fps", type=int, default=60)
    ap.add_argument("--hud-fps", type=int, default=30)
    ap.add_argument("--white-nits", type=float, help="HDR HUD white (default: recording MaxCLL, else 203)")
    ap.add_argument("--bitrate", default="30M")
    ap.add_argument("--out-dir", type=Path)
    ap.add_argument("--name", default="landing")
    ap.add_argument("--no-clean", action="store_true", help="skip the cropped copy without HUD")
    ap.add_argument("--preview", type=float, nargs="+", metavar="SEC", help="write SDR JPG stills at these times and exit")
    args = ap.parse_args()

    rec = args.recording.expanduser().resolve()
    meta = ffprobe(rec)
    hdr = meta["transfer"] == "smpte2084"
    if meta["transfer"] == "arib-std-b67":
        sys.exit("HLG recordings are not supported; record in HDR10 (PQ) or SDR")
    start = args.start_epoch if args.start_epoch is not None else recording_start_epoch(rec)
    start += args.offset
    run = args.run.resolve() if args.run else find_run(start, start + meta["duration"])
    telemetry = list(read_jsonl(run / "telemetry.jsonl"))
    events = list(read_jsonl(run / "events.jsonl"))
    vlog = args.vehicle_log or find_vehicle_log(telemetry[0]["time"])
    attitude = load_attitude(vlog, telemetry) if vlog else None
    if args.crop == "none":
        cw, ch, cx, cy = meta["width"], meta["height"], 0, 0
    elif args.crop:
        cw, ch, cx, cy = map(int, args.crop.split(":"))
    else:
        cw, ch, cx, cy = detect_crop(rec, meta)
    W = args.width
    H = round(W * ch / cw / 2) * 2
    print(f"recording {rec.name}: {meta['width']}x{meta['height']} {meta['duration']:.0f}s "
          f"{'HDR10 PQ' if hdr else 'SDR'}, starts {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(start))}", file=sys.stderr)
    print(f"run {run.name}; vehicle log {vlog.name if vlog else 'none (attitude from telemetry)'}; "
          f"crop {cw}:{ch}:{cx}:{cy} -> {W}x{H}", file=sys.stderr)

    hud = Hud(W, H, telemetry, collect_decisions(events), attitude)
    out_dir = (args.out_dir or rec.parent).expanduser()
    base = ["ffmpeg", "-v", "error", "-y"]
    crop = f"crop={cw}:{ch}:{cx}:{cy},scale={W}:{H}:flags=lanczos"

    if args.preview:
        for t in args.preview:
            png = out_dir / f"{args.name}-preview-{t:.0f}.png"
            hud.frame(start + t).save(png)
            jpg = png.with_suffix(".jpg")
            subprocess.run(base + ["-ss", str(t), "-i", str(rec), "-i", str(png), "-filter_complex",
                                   f"[0:v]{crop}[b];[b][1:v]overlay", "-frames:v", "1", str(jpg)], check=True)
            png.unlink()
            print(jpg)
        return

    if hdr:
        hud_fmt, white = "rgba64le", args.white_nits or (meta["max_cll"] if meta["max_cll"] > 0 else 203.0)
        graph = (f"[0:v]fps={args.fps},{crop}:in_color_matrix=bt2020:out_color_matrix=bt2020,format=yuv444p10le,split[v][c];"
                 "[1:v]scale=out_color_matrix=bt2020:out_range=tv,format=yuva444p10le[h];"
                 "[v][h]overlay=format=yuv444p10:eof_action=pass,format=yuv420p10le[o];[c]format=yuv420p10le[clean]")
        enc = ["-c:v", "hevc_videotoolbox", "-profile:v", "main10", "-b:v", args.bitrate, "-tag:v", "hvc1",
               "-color_primaries", "bt2020", "-color_trc", "smpte2084", "-colorspace", "bt2020nc", "-color_range", "tv"]
        encode = lambda img: to_pq_rgba64(img, white)
    else:
        hud_fmt = "rgba"
        graph = (f"[0:v]fps={args.fps},{crop},format=yuv420p,split[v][c];[v][1:v]overlay=eof_action=pass,format=yuv420p[o];"
                 "[c]null[clean]")
        enc = ["-c:v", "hevc_videotoolbox", "-b:v", args.bitrate, "-tag:v", "hvc1",
               "-color_primaries", "bt709", "-color_trc", "bt709", "-colorspace", "bt709"]
        encode = lambda img: img.tobytes()
    audio = ["-map", "0:a", "-c:a", "aac", "-b:a", "192k"] if meta["has_audio"] else []
    outputs = ["-map", "[o]", *audio, *enc, str(out_dir / f"{args.name}-HUD.mp4")]
    if args.no_clean:
        graph = graph.replace(",split[v][c]", "[v]").rsplit(";", 1)[0]
    else:
        outputs += ["-map", "[clean]", *audio, *enc, str(out_dir / f"{args.name}-clean.mp4")]
    cmd = base + ["-stats", "-i", str(rec), "-f", "rawvideo", "-pix_fmt", hud_fmt, "-s", f"{W}x{H}",
                  "-r", str(args.hud_fps), "-i", "pipe:0", "-filter_complex", graph, *outputs]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    try:
        for f in range(int(meta["duration"] * args.hud_fps) + 1):
            proc.stdin.write(encode(hud.frame(start + f / args.hud_fps)))
    except BrokenPipeError:
        pass
    proc.stdin.close()
    sys.exit(proc.wait())


if __name__ == "__main__":
    main()
