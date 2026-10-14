import math

import pytest

from ksl.ascent.peg import InsertionGuidance, apsides

MU, R, G0 = 3.5316e12, 600000., 9.80665


def fly(h, vr, vh, mass, pitch, hold=34., ramp=15., thrust=2811e3, isp=315.,
        slew=2., lag=2., upper_g=1.4, dt=.2):
    """Point-mass truth plant: slew-limited command, lagged attitude, roll-hold limiter."""
    ve = isp*G0
    guide = InsertionGuidance(MU, R+90000., R+60000., R+72000., slew=slew, lag=lag)
    r, t, cmd, flown, peak = R+h, 0., pitch, pitch, 0.
    while t < 400.:
        cap = guide._throttle_cap(t, hold, ramp, .3)
        a_avail = thrust*cap/mass
        a = min(a_avail, upper_g*G0)
        ramp_left = ramp if t < hold else max(0., ramp-(t-hold))
        raw = guide.command(t, r, vr, vh, mass, thrust, ve, upper_g*G0, a, a < .99*a_avail,
                            cmd, max(0., hold-t), ramp_left, cap)
        rate = .5 if t < hold else slew
        cmd += max(-rate*dt, min(rate*dt, raw-cmd))
        flown += (cmd-flown)*dt/lag
        p = math.radians(flown)
        vr += (a*math.sin(p)-MU/r**2+vh*vh/r)*dt
        vh += (a*math.cos(p)-vr*vh/r)*dt
        r += vr*dt
        mass -= a*mass/ve*dt
        t += dt
        ra, rp = apsides(MU, r, vr, vh)
        peak = max(peak, ra)
        if guide.done(r, vr, vh):
            break
    return ra-R, rp-R, peak-R


# Separation states of live runs peg-legacy-v2 and the lofted peg-legacy-v3,
# whose old guidance ran apoapsis to 114 km before pulling it back to 96 km.
@pytest.mark.parametrize('state', [
    (23116.8, 665.9, 939.4, 183593., 37.95),
    (24012.0, 779.3, 759.6, 184632., 46.18)])
def test_planned_insertion_reaches_target_without_apoapsis_overshoot(state):
    ap, pe, peak = fly(*state)
    assert 87000. <= ap <= 93000.
    assert pe >= 59700.
    assert peak <= 93000.
