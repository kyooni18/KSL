"""Planar Powered Explicit Guidance (linear-tangent steering, closed loop).

Every cycle re-solves a steering law sin(pitch) = A + B*t + C(t) that brings the
vehicle to a *target state* (radius, radial speed, horizontal speed) at burnout,
with the burn time predicted from the rocket equation. Unlike velocity-to-be-gained
steering it constrains where insertion happens, so the climb is planned instead of
overshot and then cancelled by pitching nose-down.

All speeds are inertial. Pure math; no kRPC.
"""
import math
from .geometry import smooth_value
from .guidance import separation_pitch_transition
from .plan_timeline import sample_trajectory, track_trajectory

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
        self.peak = 0.
        self.hold = None
        self._last_t = None

    def reset(self):
        self.A = self.B = self.tgo = self._last_t = self.hold = None
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
        pitch = math.degrees(math.asin(fr))
        # Final seconds: hold the attitude reached at freeze. Extrapolating A+B*t
        # while a throttle taper stretches the burn walks the nose away and trades
        # apoapsis for periapsis at cutoff.
        if T <= self.freeze_tgo:
            if self.hold is None:
                self.hold = pitch
            return self.hold, T
        self.hold = None
        return pitch, T


def orbit_state_at(mu, apoapsis_r, periapsis_r, insertion_r):
    """Target (radial, horizontal) speed on the ascending branch at ``insertion_r``."""
    insertion_r = _clamp(insertion_r, periapsis_r, apoapsis_r)
    vh = math.sqrt(2*mu*apoapsis_r*periapsis_r/(apoapsis_r+periapsis_r))/insertion_r
    vr = math.sqrt(max(0., 2*mu*(1/insertion_r-1/(apoapsis_r+periapsis_r))-vh*vh))
    return vr, vh


def choose_insertion(mu, t, r, vr, vh, accel, ve, apoapsis_r, periapsis_r, radii,
                     constant_accel=False):
    """Burnout radius on one target ellipse that PEG reaches in the least burn time.

    Returns (radius, solved guide) or (None, None). Only solutions whose steering
    stays feasible count, so a lofted vehicle inserts higher on the ascending
    branch instead of diving to a fixed altitude.
    """
    best = None
    for r_t in radii:
        guide = PoweredExplicitGuidance(mu)
        vr_t, vh_t = orbit_state_at(mu, apoapsis_r, periapsis_r, r_t)
        guide.update(t, r, vr, vh, accel, ve, r_t, vr_t, vh_t, constant_accel)
        if guide.converged and guide.tgo > 1. and (best is None or guide.tgo < best[1].tgo):
            best = r_t, guide
    return best or (None, None)


def apsides(mu, r, vr, vh):
    """(apoapsis radius, periapsis radius) of a planar state; apoapsis inf if unbound."""
    energy = .5*(vr*vr+vh*vh)-mu/r
    h = r*vh
    if energy >= 0.:
        return math.inf, h*h/mu/2.
    a = -mu/(2*energy)
    e = math.sqrt(max(0., 1-h*h/(mu*a)))
    return a*(1+e), a*(1-e)


class InsertionGuidance:
    """Attitude-aware closed-loop insertion to one target ellipse.

    PEG alone assumes the thrust can point anywhere instantly. The shuttle's
    commanded pitch has a gradual response, is held briefly at separation,
    and the SSME limiter ramps up afterwards. Plans that ignore this
    leave the stack climbing at the old pitch for tens of seconds: apoapsis runs
    past target and is then cancelled at a saturated nose-down limit. The burnout
    point is therefore chosen by flying each candidate through the same closed
    loop (PEG + command response + hold + throttle schedule) and scoring the cutoff orbit.
    """

    def __init__(self, mu, apoapsis_r, periapsis_r, min_insertion_r, slew=2.0,
                 hold_slew=0.5, lag=2.0, pitch_bounds=(-30.0, 45.0), replan_period=10.0,
                 apoapsis_tolerance=3000.0, periapsis_tolerance=300.0,
                 pitch_response_seconds=None, follow_trajectory=False):
        self.mu, self.ra, self.rp = mu, apoapsis_r, periapsis_r
        self.min_insertion_r = min_insertion_r
        self.slew, self.hold_slew = slew, hold_slew
        self.pitch_response_seconds = pitch_response_seconds
        self.lag = lag  # attitude response time constant behind the command, s
        self.lo, self.hi = pitch_bounds
        self.replan_period = replan_period
        self.ap_tol, self.pe_tol = apoapsis_tolerance, periapsis_tolerance
        self.guide = PoweredExplicitGuidance(mu)
        self.r_t = None
        self.planned_at = None
        self.prediction = None
        self.follow_trajectory = follow_trajectory
        self.planned_sample = None

    def done(self, r, vr, vh):
        ra, rp = apsides(self.mu, r, vr, vh)
        return ra >= self.ra-self.ap_tol and rp >= self.rp-self.pe_tol

    @staticmethod
    def _throttle_cap(t, hold_left, ramp, cap_now):
        """Limiter schedule: held at ``cap_now`` through the hold, then ramped to 1."""
        if t < hold_left:
            return cap_now
        if ramp <= 0.:
            return 1.
        return min(1., cap_now+(1.-cap_now)*(t-hold_left)/ramp)

    def steer(self, guide, t, r, vr, vh, accel, ve, r_t, constant_accel, pitch):
        """PEG to the planned burnout; velocity-to-be-gained in the final seconds.

        Near burnout the position constraint is ill-conditioned and any attitude
        frozen there is wrong as soon as the burn runs long or short. The last
        seconds instead steer along the velocity still missing from the target
        orbit at the current radius, which closes apoapsis and periapsis together
        wherever the burn actually ends.
        """
        vr_t, vh_t = orbit_state_at(self.mu, self.ra, self.rp, r_t)
        cmd, tgo = guide.update(t, r, vr, vh, accel, ve, r_t, vr_t, vh_t, constant_accel)
        # Already at or past the planned burnout altitude: insert here rather than
        # diving back down to it.
        if cmd is None or tgo < guide.freeze_tgo or r >= r_t-500.:
            vr_here, vh_here = orbit_state_at(self.mu, self.ra, self.rp, r)
            dvr, dvh = vr_here-vr, vh_here-vh
            go = max(1., math.hypot(dvr, dvh)/max(accel, .1))
            g_eff = self.mu/(r*r)-vh*vh/r
            cmd = math.degrees(math.atan2(dvr/go+g_eff, max(dvh/go, 1e-3)))
        return cmd

    def fly(self, r_t, r, vr, vh, mass, thrust, ve, g_limit, pitch, hold_left=0.,
            ramp=15., cap_now=1., dt=1., t_max=500., flown=None, attitude_hold=0.,
            transition_anchor=None, since_separation=0., record=False, dry_mass=0.):
        """Closed-loop point-mass prediction to cutoff; returns a summary dict."""
        mu = self.mu
        guide = PoweredExplicitGuidance(mu)
        m0, t, peak_ra, saturated = mass, 0., 0., 0.
        flown = pitch if flown is None else flown
        trajectory = []
        while t < t_max:
            cap = self._throttle_cap(t, hold_left, ramp, cap_now)
            a_avail = thrust*cap/mass
            a = min(a_avail, g_limit)
            cmd = self.steer(guide, t, r, vr, vh, a, ve, r_t, a < .99*a_avail, pitch)
            if cmd < self.lo or cmd > self.hi:
                saturated += dt
            cmd = _clamp(cmd, self.lo, self.hi)
            if transition_anchor is not None:
                cmd = separation_pitch_transition(transition_anchor, cmd, since_separation+t)
            if record:
                trajectory.append(dict(time=t, radius=r, radial_speed=vr, horizontal_speed=vh,
                                       mass=mass, raw_pitch=pitch if t < attitude_hold else cmd,
                                       commanded_pitch=pitch, flown_pitch=flown,
                                       thrust=a*mass, mass_flow=a*mass/ve,
                                       throttle=a*mass/thrust))
            if t < attitude_hold:
                pass  # The separation controller holds the measured attitude.
            elif self.pitch_response_seconds is None:
                rate = self.hold_slew if t < hold_left else self.slew
                pitch += _clamp(cmd-pitch, -rate*dt, rate*dt)
            else:
                pitch = smooth_value(pitch, cmd, self.pitch_response_seconds, dt)
            if t >= attitude_hold:
                flown += (pitch-flown)*min(1., dt/max(self.lag, 1e-3))
            p = math.radians(flown)
            # Midpoint step of the planar inertial equations.
            def deriv(r_, vr_, vh_):
                return (vr_, a*math.sin(p)-mu/(r_*r_)+vh_*vh_/r_, a*math.cos(p)-vr_*vh_/r_)
            k = deriv(r, vr, vh)
            k = deriv(r+k[0]*dt/2, vr+k[1]*dt/2, vh+k[2]*dt/2)
            r, vr, vh = r+k[0]*dt, vr+k[1]*dt, vh+k[2]*dt
            mass -= a*mass/ve*dt
            t += dt
            ra, rp = apsides(mu, r, vr, vh)
            peak_ra = max(peak_ra, min(ra, 10*self.ra))
            if self.done(r, vr, vh) or mass < max(dry_mass, .2*m0):
                break
        ra, rp = apsides(mu, r, vr, vh)
        dv = ve*math.log(m0/max(mass, 1.))
        # Cutoff orbit error, any apoapsis overshoot on the way, and propellant
        # (50 m/s weighs like 1 km).
        score = (abs(min(ra, 10*self.ra)-self.ra)+abs(rp-self.rp) +
                 2*max(0., peak_ra-self.ra-self.ap_tol) + 20*dv + 500*saturated)
        result = dict(r_t=r_t, score=score, apoapsis_r=ra, periapsis_r=rp,
                      peak_apoapsis_r=peak_ra, time=t, dv=dv, saturated=saturated,
                      radius=r, radial_speed=vr, horizontal_speed=vh, mass=mass,
                      propellant_used=m0-mass, fuel_feasible=mass >= dry_mass,
                      target_feasible=(self.ra-self.ap_tol <= ra <= self.ra+self.ap_tol and
                                       rp >= self.rp-self.pe_tol and
                                       peak_ra <= self.ra+self.ap_tol))
        if record:
            trajectory.append(dict(trajectory[-1], time=t, radius=r, radial_speed=vr,
                                   horizontal_speed=vh, mass=mass,
                                   commanded_pitch=pitch, flown_pitch=flown))
            result['trajectory'] = trajectory
        return result

    def plan(self, r, vr, vh, mass, thrust, ve, g_limit, pitch, hold_left=0., ramp=15.,
             cap_now=1., candidates=8, flown_pitch=None, attitude_hold=0.,
             transition_anchor=None, since_separation=0., dry_mass=0.):
        low = max(self.min_insertion_r, r+2000.)
        high = self.ra-self.ap_tol
        radii = ([low+(high-low)*i/(candidates-1) for i in range(candidates)] if low < high
                 else [max(r, self.rp)])
        best = None
        for r_t in radii:
            result = self.fly(r_t, r, vr, vh, mass, thrust, ve, g_limit, pitch,
                              hold_left, ramp, cap_now, dt=2., flown=flown_pitch,
                              attitude_hold=attitude_hold, transition_anchor=transition_anchor,
                              since_separation=since_separation, record=True, dry_mass=dry_mass)
            if not result['fuel_feasible']:
                result['score'] = float('inf')
            if best is None or result['score'] < best['score']:
                best = result
        return best

    def command(self, t, r, vr, vh, mass, thrust, ve, g_limit, accel, constant_accel,
                pitch, hold_left=0., ramp=15., cap_now=1., flown_pitch=None,
                attitude_hold=0., transition_anchor=None, since_separation=0., dry_mass=0.):
        """Plan using commanded pitch and, when available, measured thrust pitch."""
        end = self.guide.tgo is not None and self.guide.tgo < 2*self.guide.freeze_tgo
        if (self.r_t is None or t < self.planned_at or
                (self.follow_trajectory and t-self.planned_at >= self.prediction['time']) or
                (not end and t-self.planned_at >= self.replan_period)):
            best = self.plan(r, vr, vh, mass, thrust, ve, g_limit, pitch,
                             hold_left, ramp, cap_now, flown_pitch=flown_pitch,
                             attitude_hold=attitude_hold, transition_anchor=transition_anchor,
                             since_separation=since_separation, dry_mass=dry_mass)
            keep = None
            if self.r_t is not None:
                keep = self.fly(self.r_t, r, vr, vh, mass, thrust, ve, g_limit, pitch,
                                hold_left, ramp, cap_now, dt=2., flown=flown_pitch,
                                attitude_hold=attitude_hold, transition_anchor=transition_anchor,
                                since_separation=since_separation, record=True, dry_mass=dry_mass)
                if not keep['fuel_feasible']:
                    keep['score'] = float('inf')
            # Switching burnout points steps the command; only do it for a clear gain,
            # or when the current plan no longer reaches the target apoapsis at all.
            keep_missing = (keep is not None and
                            abs(keep['apoapsis_r']-self.ra) > 3*self.ap_tol and
                            abs(best['apoapsis_r']-self.ra) < abs(keep['apoapsis_r']-self.ra))
            if keep is None or keep_missing or best['score'] < .7*keep['score']-1000.:
                if best['r_t'] != self.r_t:
                    self.guide = PoweredExplicitGuidance(self.mu)
                self.r_t, self.prediction = best['r_t'], best
            else:
                self.prediction = keep
            self.planned_at = t
        if self.follow_trajectory:
            if not math.isfinite(self.prediction['score']):
                raise ValueError('No fuel-feasible insertion trajectory')
            elapsed = t-self.planned_at
            self.planned_sample = sample_trajectory(self.prediction['trajectory'], elapsed)
            return _clamp(track_trajectory(self.planned_sample, r, vr, vh,
                                           self.prediction['time']-elapsed), self.lo, self.hi)
        cmd = self.steer(self.guide, t, r, vr, vh, accel, ve, self.r_t, constant_accel, pitch)
        return _clamp(cmd, self.lo, self.hi)
