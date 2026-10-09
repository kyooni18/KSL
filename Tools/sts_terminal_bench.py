#!/usr/bin/env python3
"""Fast point-mass STS terminal-area bench (offline design tool, not flight code).

Flat-runway frame: x east along Runway 09 (0 at the configured site point),
y north, h above runway.  Air-relative 3-DOF point mass with the provisional
STS aero table, Kerbin atmosphere table, curvature relief V^2/(R+h), and
rate/acceleration-limited AoA/bank servos.  Used to size terminal geometry and
to prototype guidance laws before porting them to CLanding.
"""
import bisect, math, os, csv

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REF = os.path.join(ROOT, "ShuttleSim", "reference-model")
G0, RADIUS, AREA = 9.81, 600000.0, 250.0


def _rows(path):
    with open(path) as f:
        for line in f:
            if not line.strip() or line[0] == "#" or line[0].isalpha():
                continue
            yield [float(v) for v in line.strip().split(",")]


class Atmosphere:
    def __init__(self, path=os.path.join(REF, "kerbin_atmosphere_identified.csv")):
        rows = list(_rows(path))
        self.h = [r[0] for r in rows]
        self.rho = [r[1] for r in rows]
        self.a = [r[4] for r in rows]

    def __call__(self, h):
        h = max(self.h[0], min(self.h[-1], h))
        i = max(0, min(len(self.h) - 2, bisect.bisect_right(self.h, h) - 1))
        f = (h - self.h[i]) / (self.h[i + 1] - self.h[i])
        return (self.rho[i] + f * (self.rho[i + 1] - self.rho[i]),
                self.a[i] + f * (self.a[i + 1] - self.a[i]))


class Aero:
    def __init__(self, path=os.path.join(REF, "sts_aero_live20261009.csv"),
                 cl_scale=1.0, cd_scale=1.0):
        tab = {}
        for m, a, cl, cd in (r[:4] for r in _rows(path)):
            tab.setdefault(m, []).append((a, cl, cd))
        self.mach = sorted(tab)
        self.tab = {m: sorted(v) for m, v in tab.items()}
        self.cl_scale, self.cd_scale = cl_scale, cd_scale

    @staticmethod
    def _alpha(rows, a):
        a = max(rows[0][0], min(rows[-1][0], a))
        for i in range(len(rows) - 1):
            if rows[i][0] <= a <= rows[i + 1][0]:
                f = (a - rows[i][0]) / (rows[i + 1][0] - rows[i][0])
                return (rows[i][1] + f * (rows[i + 1][1] - rows[i][1]),
                        rows[i][2] + f * (rows[i + 1][2] - rows[i][2]))
        return rows[-1][1], rows[-1][2]

    def coef(self, mach, alpha_deg):
        ms = self.mach
        i = max(0, min(len(ms) - 2, bisect.bisect_right(ms, mach) - 1))
        f = max(0.0, min(1.0, (mach - ms[i]) / (ms[i + 1] - ms[i])))
        c0, c1 = self._alpha(self.tab[ms[i]], alpha_deg), self._alpha(self.tab[ms[i + 1]], alpha_deg)
        return ((c0[0] + f * (c1[0] - c0[0])) * self.cl_scale,
                (c0[1] + f * (c1[1] - c0[1])) * self.cd_scale)


class Servo:
    """Second-order rate/acceleration limited attitude response."""
    def __init__(self, value, rate_max, accel_max, wn):
        self.v, self.r = value, 0.0
        self.rate_max, self.accel_max, self.wn = rate_max, accel_max, wn

    def step(self, cmd, dt):
        want_rate = max(-self.rate_max, min(self.rate_max, self.wn * (cmd - self.v)))
        # brake early enough to stop at the command
        stop = math.sqrt(2.0 * self.accel_max * abs(cmd - self.v))
        want_rate = max(-stop, min(stop, want_rate))
        acc = max(-self.accel_max, min(self.accel_max, (want_rate - self.r) / dt))
        self.r += acc * dt
        self.v += self.r * dt
        return self.v


class State:
    def __init__(self, x, y, h, v, gamma_deg, psi_deg, mass, aoa=10.0, bank=0.0):
        self.x, self.y, self.h, self.v = x, y, h, v
        self.g, self.psi = math.radians(gamma_deg), math.radians(psi_deg)
        self.m, self.t = mass, 0.0
        self.aoa = Servo(aoa, 5.8, 14.0, 1.5)
        self.bank = Servo(bank, 12.0, 23.0, 1.4)
        self.ground = False


class Plant:
    def __init__(self, aero=None, atmo=None):
        self.aero, self.atmo = aero or Aero(), atmo or Atmosphere()

    def forces(self, s, aoa):
        rho, a = self.atmo(s.h + 70.0)
        q = 0.5 * rho * s.v * s.v
        cl, cd = self.aero.coef(s.v / a, aoa)
        return q, s.v / a, cl * q * AREA, cd * q * AREA

    def step(self, s, aoa_cmd, bank_cmd, dt):
        aoa = s.aoa.step(aoa_cmd, dt)
        bank = s.bank.step(bank_cmd, dt)
        q, mach, lift, drag = self.forces(s, aoa)
        m, v, g = s.m, max(s.v, 1.0), s.g
        grav = G0 * (RADIUS / (RADIUS + s.h)) ** 2
        phi = math.radians(bank)
        dv = -drag / m - grav * math.sin(g)
        dg = lift * math.cos(phi) / (m * v) - (grav / v - v / (RADIUS + s.h)) * math.cos(g)
        dpsi = -lift * math.sin(phi) / (m * v * max(0.05, math.cos(g)))  # +bank = right turn
        s.v += dv * dt
        s.g += dg * dt
        s.psi += dpsi * dt
        s.x += v * math.cos(g) * math.cos(s.psi) * dt
        s.y += v * math.cos(g) * math.sin(s.psi) * dt
        s.h += v * math.sin(g) * dt
        s.t += dt
        return dict(q=q, mach=mach, lift=lift, drag=drag, aoa=aoa, bank=bank,
                    nz=lift / (m * grav))

    def lift_aoa(self, s, lift_needed, aoa_max=28.0):
        """AoA producing the requested lift at the current state (bisection)."""
        lo, hi = -5.0, aoa_max
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            if self.forces(s, mid)[2] < lift_needed:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)
