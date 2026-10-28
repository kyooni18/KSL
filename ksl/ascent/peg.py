"""Planar Powered Explicit Guidance (linear-tangent steering, closed loop).

Every cycle re-solves a steering law sin(pitch) = A + B*t + C(t) that brings the
vehicle to a *target state* (radius, radial speed, horizontal speed) at burnout,
with the burn time predicted from the rocket equation. Unlike velocity-to-be-gained
steering it constrains where insertion happens, so the climb is planned instead of
overshot and then cancelled by pitching nose-down.

All speeds are inertial. Pure math; no kRPC.
"""
import math
from . import native
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

    def update(self, t, r, vr, vh, accel, ve, r_t, vr_t, vh_t, constant_accel=False):
        """Return the Rust PEG solution and time to go for the measured state."""
        return native.peg_update(self, t, r, vr, vh, accel, ve, r_t, vr_t, vh_t,
                                 constant_accel)



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
        return native.insertion_fly(self, r_t, r, vr, vh, mass, thrust, ve, g_limit,
                                    pitch, hold_left, ramp, cap_now, dt, t_max, flown,
                                    attitude_hold, transition_anchor, since_separation,
                                    record, dry_mass)

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
