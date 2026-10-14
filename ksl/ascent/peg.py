"""Planar Powered Explicit Guidance (linear-tangent steering, closed loop).

Every cycle re-solves a steering law sin(pitch) = A + B*t + C(t) that brings the
vehicle to a *target state* (radius, radial speed, horizontal speed) at burnout,
with the burn time predicted from the rocket equation. Unlike velocity-to-be-gained
steering it constrains where insertion happens, so the climb is planned instead of
overshot and then cancelled by pitching nose-down.

All speeds are inertial. Pure math; no kRPC.
"""
import math

G0 = 9.80665


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


class PoweredExplicitGuidance:
    def __init__(self, mu, freeze_tgo=8.0):
        self.mu = mu
        self.freeze_tgo = freeze_tgo
        self.A = self.B = None
        self.tgo = None
        self.converged = False
        self._last_t = None

    def reset(self):
        self.A = self.B = self.tgo = self._last_t = None
        self.converged = False

    def _integrals(self, T, accel, ve, constant):
        if constant:
            return accel*T, accel*T*T/2, accel*T*T/2, accel*T**3/6
        tau = ve/accel
        T = min(T, .999*tau)
        b0 = -ve*math.log(1-T/tau)
        b1 = b0*tau-ve*T
        c0 = b0*T-b1
        c1 = c0*tau-ve*T*T/2
        return b0, b1, c0, c1

    def update(self, t, r, vr, vh, accel, ve, r_t, vr_t, vh_t, constant_accel=False):
        """Return (pitch_deg, time_to_go). ``constant_accel`` models a G-limited burn."""
        accel = max(accel, .05)
        ve = max(ve, 50.)
        tau = ve/accel
        if self._last_t is not None and self.A is not None:
            dt = max(0., t-self._last_t)
            self.A += self.B*dt
            self.tgo = max(0., self.tgo-dt)
        self._last_t = t
        if self.A is None:
            dv = math.hypot(vh_t-vh, vr_t-vr)
            self.tgo = dv/accel if constant_accel else tau*(1-math.exp(-min(dv, 5*ve)/ve))
        mu = self.mu
        T = max(self.tgo, .1)
        C = (mu/(r*r)-vh*vh/r)/accel
        dh = r_t*vh_t-r*vh
        need = dh/((r+r_t)/2)  # horizontal speed to gain, from angular momentum
        peak = 0.
        T_cap = 5000. if constant_accel else .98*tau
        for _ in range(25):
            if T > self.freeze_tgo or self.A is None:
                b0, b1, c0, c1 = self._integrals(T, accel, ve, constant_accel)
                det = b0*c1-b1*c0
                if abs(det) < 1e-9:
                    break
                k1 = vr_t-vr
                k2 = r_t-r-vr*T
                self.A = (c1*k1-b1*k2)/det
                self.B = (b0*k2-c0*k1)/det
            # Horizontal velocity the planned steering delivers in T (Simpson rule).
            a_t = accel if constant_accel else accel/(1-min(T, .999*tau)/tau)
            C_t = (mu/(r_t*r_t)-vh_t*vh_t/r_t)/a_t
            gain = 0.
            peak = 0.
            n = 16
            for i in range(n+1):
                x = T*i/n
                a_x = accel if constant_accel else accel/(1-x/tau)
                fr = self.A+self.B*x+C+(C_t-C)*x/T
                peak = max(peak, abs(fr))
                fr = _clamp(fr, -1., 1.)
                w = 1 if i in (0, n) else 4 if i % 2 else 2
                gain += w*a_x*math.sqrt(1-fr*fr)
            gain *= T/(3*n)
            if need <= 0.:
                T = 0.
                break
            ratio = _clamp(need/max(gain, 1e-6), .7, 1.4)
            T_new = min(T_cap, T*ratio)
            done = abs(T_new-T) < .02
            T = T_new
            if done:
                break
        if self.A is None:
            return None, T
        # Feasible: the planned steering never asks for more than ~72 deg off horizontal.
        self.peak = peak if T > 0. else 0.
        self.converged = math.isfinite(T) and T < T_cap and self.peak < .95
        self.tgo = T
        fr = _clamp(self.A+C, -1., 1.)
        return math.degrees(math.asin(fr)), T


def orbit_state_at(mu, apoapsis_r, periapsis_r, insertion_r):
    """Target (radial, horizontal) speed on the ascending branch at ``insertion_r``."""
    insertion_r = _clamp(insertion_r, periapsis_r, apoapsis_r)
    vh = math.sqrt(2*mu*apoapsis_r*periapsis_r/(apoapsis_r+periapsis_r))/insertion_r
    vr = math.sqrt(max(0., 2*mu*(1/insertion_r-1/(apoapsis_r+periapsis_r))-vh*vh))
    return vr, vh
