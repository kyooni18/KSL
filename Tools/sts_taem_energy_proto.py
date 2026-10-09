#!/usr/bin/env python3
"""Prototype STS TAEM energy-management law on the point-mass bench.

Frame: x east along Runway 09 (0 at the configured site point), y north,
h above the runway.  Targets the Final station (x=-D, y=0, h=D*tan(OGS),
V=V_station, course 090).  Not flight code; used to size and debug the law
before it is ported to CLanding.

Vertical: dynamic-pressure schedule (supersonic: 1-g equilibrium at the
nominal incidence; subsonic: q_sub blending to the station q).  Flight-path
angle is commanded from the q error, incidence from the lift that the
flight-path change needs.
Lateral: heading-alignment circle tangent to the Final line at the station;
energy (specific energy height E) against the path length the nominal
schedule needs; S-turns while E is above it, shortest geometry/slower q
while below.
"""
import argparse, math
from sts_terminal_bench import Plant, State, Aero, G0, AREA

D_STATION = 8000.0
OGS = math.radians(20.0)
V_STATION = 160.0
H_STATION = D_STATION * math.tan(OGS)
E_STATION = H_STATION + V_STATION ** 2 / (2 * G0)
AOA_MAX, AOA_MIN, AOA_NOM_SUP, AOA_SUP = 15.0, 1.0, 11.0, 13.0
BANK_MAX, BANK_S = 50.0, 50.0
Y_S_MAX = 7000.0
BANK_SUP = 45.0
R_LOOP, K_PLAN, R_MIN, R_MAX = 5000.0, 0.5, 2200.0, 6000.0
K_LOOP = 0.25
MODE_FREEZE = 8000.0
Q_SUB = 7000.0
Q_LO, Q_HI, AOA_SUP_MIN, BLEND, AIM = 3500.0, 11000.0, 8.0, 8000.0, 9000.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Law:
    def __init__(self, plant, mass):
        self.P, self.m = plant, mass
        self.s_side = 0
        self.hac_side = None
        self.on_hac = False
        self.final_line = False
        self.radius = None
        self.sched = self._schedule()

    # --- nominal schedule: E -> (h, V, q, L/D) -------------------------------
    def q_nominal(self, mach, h):
        rho, a = self.P.atmo(h + 70.0)
        q_sta = 0.5 * self.P.atmo(H_STATION + 70)[0] * V_STATION ** 2
        if mach >= 1.2:
            cl, _ = self.P.aero.coef(mach, AOA_NOM_SUP)
            return self.m * G0 / (cl * AREA)
        if mach <= 0.9:
            return Q_SUB
        cl, _ = self.P.aero.coef(1.2, AOA_NOM_SUP)
        qs = self.m * G0 / (cl * AREA)
        return Q_SUB + (qs - Q_SUB) * (mach - 0.9) / 0.3

    def _point(self, e):
        """Solve h + V^2/2g = e with q(h,V) = q_nominal."""
        lo, hi = 0.0, e
        for _ in range(40):
            h = 0.5 * (lo + hi)
            v = math.sqrt(max(1.0, 2 * G0 * (e - h)))
            rho, a = self.P.atmo(h + 70.0)
            q = 0.5 * rho * v * v
            if q > self.q_nominal(v / a, h):
                lo = h            # too fast/low: raise h
            else:
                hi = h
        h = 0.5 * (lo + hi)
        v = math.sqrt(max(1.0, 2 * G0 * (e - h)))
        rho, a = self.P.atmo(h + 70.0)
        q = 0.5 * rho * v * v
        cl_req = self.m * G0 / (q * AREA)
        lo2, hi2 = -2.0, AOA_MAX
        for _ in range(25):
            mid = 0.5 * (lo2 + hi2)
            if self.P.aero.coef(v / a, mid)[0] < cl_req:
                lo2 = mid
            else:
                hi2 = mid
        cl, cd = self.P.aero.coef(v / a, 0.5 * (lo2 + hi2))
        return h, v, q, max(0.3, cl / cd)

    def _schedule(self):
        pts, e = [], E_STATION
        while e < 90000.0:
            pts.append((e,) + self._point(e))
            e += 500.0
        # cumulative range required from the station
        rng, acc = [0.0], 0.0
        for (e0, *_, ld0), (e1, *_, ld1) in zip(pts, pts[1:]):
            acc += 0.5 * (ld0 + ld1) * (e1 - e0)
            rng.append(acc)
        return pts, rng

    def lookup(self, e):
        pts, rng = self.sched
        i = max(0, min(len(pts) - 2, int((e - E_STATION) // 500.0)))
        f = max(0.0, min(1.0, (e - pts[i][0]) / 500.0))
        lerp = lambda a, b: a + f * (b - a)
        return dict(h=lerp(pts[i][1], pts[i + 1][1]), v=lerp(pts[i][2], pts[i + 1][2]),
                    q=lerp(pts[i][3], pts[i + 1][3]), ld=lerp(pts[i][4], pts[i + 1][4]),
                    s=max(0.0, lerp(rng[i], rng[i + 1]) if e > E_STATION else 0.0))

    # --- geometry ------------------------------------------------------------
    def hac_path(self, x, y, side, radius):
        """Path via the HAC on `side` (+1 centre north, CCW; -1 south, CW)."""
        cx, cy = -D_STATION, side * radius
        dx, dy = x - cx, y - cy
        d = math.hypot(dx, dy)
        if d <= radius * 1.001:
            return None
        ang_t = math.atan2(dy, dx) + side * math.acos(radius / d)
        tx, ty = cx + radius * math.cos(ang_t), cy + radius * math.sin(ang_t)
        ang_s = math.atan2(-cy, 0.0)
        turn = ((ang_s - ang_t) if side > 0 else (ang_t - ang_s)) % (2 * math.pi)
        return math.hypot(tx - x, ty - y) + radius * turn, (tx, ty), turn

    def vertical(self, s, q, mach, lift, drag, bank_deg, k, along):
        """Incidence command for drag-modulation level k (0 max range .. 1 min range)."""
        if mach > 1.05:
            if s.g < math.radians(-32.0):
                return AOA_MAX
            return AOA_MAX - (AOA_MAX - AOA_SUP_MIN) * k
        q_ref = Q_LO + (Q_HI - Q_LO) * k
        if along < BLEND:
            q_sta = 0.5 * self.P.atmo(s.h + 70)[0] * V_STATION ** 2
            w = max(0.0, along) / BLEND
            q_ref = w * q_ref + (1 - w) * q_sta
        a1 = self.P.lift_aoa(s, s.m * G0, AOA_MAX)
        _, _, l1, d1 = self.P.forces(s, a1)
        ld = max(0.3, l1 / d1) if d1 > 0 else 1.0
        g_glide = -math.atan(1.0 / ld)
        g_cmd = g_glide - 0.6 * max(-0.6, min(0.6, (q_ref - q) / q_ref))
        if 0.0 < along < AIM:
            # aim at the station point; energy (already matched) sets the speed
            g_aim = -math.atan2(s.h - H_STATION, max(500.0, along))
            w = min(1.0, (AIM - along) / 3000.0)
            g_cmd = (1 - w) * g_cmd + w * g_aim
        g_cmd = max(math.radians(-35), min(math.radians(6), g_cmd))
        gdot = max(-0.08, min(0.08, (g_cmd - s.g) / 4.0))
        phi = math.radians(abs(bank_deg))
        lift_req = s.m * (G0 * math.cos(s.g) + s.v * gdot) / max(0.3, math.cos(phi))
        return max(AOA_MIN, min(AOA_MAX, self.P.lift_aoa(s, lift_req, AOA_MAX)))

    def predict_range(self, s, k, along, segs=None):
        """Path distance a flight at level k needs to reach E_STATION, banking on planned arcs."""
        p = State(0.0, 0.0, s.h, s.v, math.degrees(s.g), 0.0, s.m, aoa=s.aoa.v, bank=0.0)
        segs = list(segs or [])
        dt, dist, i, used = 1.0, 0.0, 0, 0.0
        while p.t < 1500:
            e = p.h + p.v * p.v / (2 * G0)
            if e <= E_STATION or p.h < 0:
                break
            while i < len(segs) and used >= segs[i][0]:
                used -= segs[i][0]
                i += 1
            rad = segs[i][1] if i < len(segs) else None
            gnd = p.v * math.cos(p.g)
            bank = math.degrees(math.atan(gnd * gnd / (G0 * rad))) if rad else 0.0
            bank = min(BANK_MAX, bank)
            q, mach, lift, drag = self.P.forces(p, p.aoa.v)
            aoa = self.vertical(p, q, mach, lift, drag, bank, k, along - dist)
            p.bank.v = bank
            p.psi = 0.0
            x0 = p.x
            self.P.step(p, aoa, bank, dt)
            p.bank.v, p.bank.r = bank, 0.0
            d = p.x - x0
            dist += d
            used += d
        return dist

    def solve_k(self, s, s_go):
        lo, hi = 0.0, 1.0
        segs = self.segments(s)
        r_lo, r_hi = self.predict_range(s, 0.0, s_go, segs), self.predict_range(s, 1.0, s_go, segs)
        if r_hi >= s_go:
            return 1.0, r_hi
        if r_lo <= s_go:
            return 0.0, r_lo
        for _ in range(7):
            mid = 0.5 * (lo + hi)
            if self.predict_range(s, mid, s_go, segs) > s_go:
                lo = mid
            else:
                hi = mid
        k = 0.5 * (lo + hi)
        return k, s_go

    # --- pattern geometry ------------------------------------------------------
    # STRAIGHT: centreline to the station.
    # LOOP: centreline east to x_turn, half circle (radius R, side) onto a downwind
    # leg at y=2R heading west, HAC half circle back onto the centreline at the station.
    def loop_length(self, x, x_turn, r):
        return (x_turn - x) + (x_turn + D_STATION) + 2.0 * math.pi * r

    def segments(self, s):
        """Remaining planned path as [(length, turn radius or None)]."""
        r, side = self.radius, self.side
        if self.mode == "straight" or self.leg == "final":
            return [(math.hypot(max(0.0, -D_STATION - s.x), s.y), None)]
        if self.leg == "out":
            rt = r + 0.5 * abs(s.y)
            return [(max(0.0, self.x_turn - s.x), None), (math.pi * rt, rt),
                    (self.x_turn + D_STATION, None), (math.pi * r, r)]
        if self.leg in ("turn", "hac"):
            cx, cy = self.turn_c if self.leg == "turn" else (-D_STATION, side * r)
            ang = math.atan2(s.y - cy, s.x - cx)
            start = -side * math.pi / 2 if self.leg == "turn" else side * math.pi / 2
            done = (side * (ang - start)) % (2 * math.pi)
            if done > 1.9 * math.pi:
                done = 0.0
            rr = self.r_turn if self.leg == "turn" else r
            arc = (rr * max(0.0, math.pi - done), rr)
            if self.leg == "turn":
                return [arc, (max(0.0, cx + D_STATION), None), (math.pi * r, r)]
            return [arc]
        return [(max(0.0, s.x + D_STATION), None), (math.pi * r, r)]

    def remaining(self, s):
        return sum(l for l, _ in self.segments(s))

    def step(self, s):
        e = s.h + s.v * s.v / (2 * G0)
        q, mach, lift, drag = self.P.forces(s, s.aoa.v)
        gnd = s.v * math.cos(s.g)
        if not hasattr(self, "mode"):
            self.mode, self.leg, self.side, self.x_turn = "straight", "out", 1, -D_STATION
            self.k, self._s_req = 0.5, 0.0
            self.radius = R_LOOP
        if self.leg == "out":
            # pick the pattern while still outbound
            if s.t >= getattr(self, "_geo_t", -1e9) + 2.0 and s.x < -D_STATION - MODE_FREEZE:
                self._geo_t = s.t
                s_dir = math.hypot(max(0.0, -D_STATION - s.x), s.y)
                # pattern choice with hysteresis on the drag level each pattern needs
                mode0, r0, xt0 = self.mode, self.radius, self.x_turn
                self.mode, self.radius, self.x_turn = "loop", R_MIN, -D_STATION
                k_loop, _ = self.solve_k(s, self.remaining(s))
                self.mode = "straight"
                k_str, _ = self.solve_k(s, self.remaining(s))
                self.mode, self.radius, self.x_turn = mode0, r0, xt0
                if self.mode == "straight" and k_str >= 1.0 and k_loop >= K_LOOP:
                    self.mode = "loop"
                elif self.mode == "loop" and k_loop <= 0.0 and k_str < 0.9:
                    self.mode = "straight"
                if self.mode == "loop":
                    # size the pattern so the plan flies at K_PLAN
                    best = None
                    for rr in (R_MIN, 3000.0, 3500.0, 4000.0, 4500.0, 5000.0, 5500.0, R_MAX):
                        self.radius, self.x_turn = rr, -D_STATION
                        segs = self.segments(s)
                        need = self.predict_range(s, K_PLAN, 1e9, segs)
                        ext = need - sum(l for l, _ in segs)
                        if best is None or abs(ext) < abs(best[1]):
                            best = (rr, ext)
                        if ext <= 0.0:
                            break
                    rr, ext = best
                    self.radius = rr
                    self.x_turn = -D_STATION + max(0.0, ext) / 2.0
            elif self.mode == "loop" and s.t >= getattr(self, "_geo_t", -1e9) + 2.0:
                self._geo_t = s.t
                segs = self.segments(s)
                ext = self.predict_range(s, K_PLAN, 1e9, segs) - sum(l for l, _ in segs)
                self.x_turn = max(-D_STATION, s.x, self.x_turn + 0.5 * ext)
            if self.mode == "loop" and s.x >= self.x_turn:
                self.leg = "turn"
                self.side = 1 if s.y <= 0 else -1
                self.r_turn = self.radius + 0.5 * abs(s.y)
                self.turn_c = (s.x, s.y + self.side * self.r_turn)
        if self.mode == "loop" and self.leg in ("turn", "down") and \
                s.t >= getattr(self, "_geo_t", -1e9) + 2.0:
            # re-size the HAC while it can still be changed (spiral-free radius update)
            self._geo_t = s.t
            best = None
            for rr in (R_MIN, 2600.0, 3000.0, 3500.0, 4000.0, 4500.0, 5000.0, 5500.0, R_MAX):
                self.radius = rr
                segs = self.segments(s)
                ext = self.predict_range(s, K_PLAN, 1e9, segs) - sum(l for l, _ in segs)
                if best is None or abs(ext) < abs(best[1]):
                    best = (rr, ext)
            self.radius = best[0]
        s_go = self.remaining(s)
        if s.t >= getattr(self, "_pred_t", -1e9) + 2.0:
            self._pred_t = s.t
            self.k, self._s_req = self.solve_k(s, s_go)
        s_req = self._s_req
        r, side = self.radius, self.side
        # ---- lateral path following ----
        bank_ff = 0.0
        if self.leg in ("out", "final"):
            look = max(3000.0, min(12000.0, 0.4 * max(0.0, -D_STATION - s.x)))
            if mach > 1.1:
                look = 20000.0
            course_cmd = math.atan2(-s.y, look)
            if self.mode == "straight" and -D_STATION - s.x < 6000.0:
                self.leg = "final"
        elif self.leg == "down":
            course_cmd = math.pi + math.atan2((s.y - side * 2 * r), 3000.0) * (1 if side > 0 else 1)
            if s.x <= -D_STATION:
                self.leg = "hac"
        else:
            cx, cy = self.turn_c if self.leg == "turn" else (-D_STATION, side * r)
            ang = math.atan2(s.y - cy, s.x - cx)
            rr = self.r_turn if self.leg == "turn" else r
            r_err = math.hypot(s.x - cx, s.y - cy) - rr
            course_cmd = ang + side * (math.pi / 2 + max(-0.6, min(0.6, r_err / 1500.0)))
            bank_ff = side * math.degrees(math.atan(gnd * gnd / (G0 * rr)))
            heading_done = wrap(s.psi - (math.pi if self.leg == "turn" else 0.0))
            if self.leg == "turn" and abs(heading_done) < 0.15 and side * (s.y - cy) > 0:
                self.leg = "down"
            if self.leg == "hac" and abs(heading_done) < 0.15 and side * (s.y - cy) < 0:
                self.leg = "final"
        cerr = wrap(course_cmd - s.psi)
        omega = max(-0.12, min(0.12, cerr / 6.0))
        bank_left = bank_ff + math.degrees(math.atan(gnd * omega / G0))
        lim = BANK_SUP if mach > 1.1 else BANK_MAX
        bank_cmd = -max(-lim, min(lim, bank_left))
        # ---- vertical ----
        aoa = self.vertical(s, q, mach, lift, drag, bank_cmd, self.k, s_go)
        q_ref = Q_LO + (Q_HI - Q_LO) * self.k
        self.final_line = self.leg == "final"
        return aoa, bank_cmd, dict(e=e, s_req=s_req, s_go=s_go, excess=s_req - s_go, side=side,
                                   on_hac=self.leg, s_turn=0, q_ref=q_ref,
                                   turn=0.0, radius=r, fl=self.final_line)


def run(x, y, h, v, gamma, course_deg, mass=25722.0, cl=1.0, cd=1.0, verbose=False, table=None):
    P = Plant(Aero(table, cl_scale=cl, cd_scale=cd) if table else Aero(cl_scale=cl, cd_scale=cd))
    psi = math.radians(90.0 - course_deg)
    s = State(x, y, h, v, gamma, math.degrees(psi), mass, aoa=12.0)
    law = Law(P, mass)
    dt, nt = 0.1, 0.0
    worst = dict(qmax=0.0, nzmax=0.0)
    while s.t < 1200:
        aoa, bank, info = law.step(s)
        o = P.step(s, aoa, bank, dt)
        worst["qmax"] = max(worst["qmax"], o["q"])
        worst["nzmax"] = max(worst["nzmax"], o["nz"])
        if verbose and s.t >= nt:
            nt += 10
            print(f"t={s.t:4.0f} x={s.x/1e3:7.2f} y={s.y/1e3:6.2f} h={s.h:6.0f} V={s.v:6.1f} M={o['mach']:.2f} q={o['q']:5.0f}/{info['q_ref']:5.0f} "
                  f"g={math.degrees(s.g):6.1f} crs={(90-math.degrees(s.psi))%360:5.1f} aoa={o['aoa']:5.1f} bank={o['bank']:6.1f} "
                  f"E={info['e']:6.0f} sreq={info['s_req']/1e3:6.1f} sgo={info['s_go']/1e3:6.1f} side={info['side']:+d} "
                  f"k={law.k:.2f} xt={law.x_turn/1e3:.1f}")
        if info["fl"] and s.x >= -D_STATION:
            return dict(ok=True, t=round(s.t, 1), h=round(s.h), v=round(s.v, 1), y=round(s.y),
                        course=round((90 - math.degrees(s.psi)) % 360, 1), gamma=round(math.degrees(s.g), 1),
                        dh=round(s.h - H_STATION), dv=round(s.v - V_STATION, 1),
                        qmax=round(worst["qmax"]), nzmax=round(worst["nzmax"], 2))
        if s.h < 0:
            return dict(ok=False, crashed=True, x=round(s.x), y=round(s.y), **worst)
    return dict(ok=False, timeout=True, x=s.x, y=s.y, h=s.h, v=s.v, **worst)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, default=-59000)
    ap.add_argument("--y", type=float, default=2000)
    ap.add_argument("--h", type=float, default=26900)
    ap.add_argument("--speed", type=float, default=878)
    ap.add_argument("--gamma", type=float, default=-7.4)
    ap.add_argument("--course", type=float, default=112)
    ap.add_argument("--cl", type=float, default=1.0)
    ap.add_argument("--cd", type=float, default=1.0)
    ap.add_argument("--table", default=None)
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()
    print(run(a.x, a.y, a.h, a.speed, a.gamma, a.course, cl=a.cl, cd=a.cd,
              verbose=a.verbose, table=a.table))
