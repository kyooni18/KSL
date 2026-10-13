import math

import pytest

from ksl.ascent import geo, launch_window, orbit_math as om

MU, R = 3.5316e12, 600000.0
OMEGA = om.TAU/21549.425


def plane_normal(inc_deg, lan_deg):
    i, o = math.radians(inc_deg), math.radians(lan_deg)
    return (math.sin(i)*math.sin(o), -math.sin(i)*math.cos(o), math.cos(i))


def test_equatorial_pad_reaches_minmus_plane_at_its_node_heading_84_deg():
    site = (R, 0.0, 0.0)
    wins = launch_window.windows(site, OMEGA, plane_normal(6.0, 78.0), horizon=21549.425)
    assert len(wins) == 1 and wins[0]['northbound']
    assert wins[0]['t'] == pytest.approx(math.radians(78.0)/OMEGA, abs=1.0)
    assert wins[0]['azimuth'] == pytest.approx(84.0, abs=0.01)


def test_plane_below_pad_latitude_has_no_window():
    lat = math.radians(10.0)
    site = (R*math.cos(lat), 0.0, R*math.sin(lat))
    assert launch_window.crossings(site, OMEGA, plane_normal(5.0, 0.0)) == []


def test_phasing_places_gto_apogee_on_requested_longitude():
    body = geo.KERBIN
    rate = geo.rotation_rate_deg(body)
    pe, geo_alt = 85000.0, geo.synchronous_altitude(body)
    semi = geo.period_from_apsides(MU, R, 900000.0, pe)
    gto = geo.period_from_apsides(MU, R, geo_alt, pe)
    for target in (-120.0, 0.0, 75.0):
        options, _ = geo.phasing_plan(-60.0, semi, gto, target, rate, MU, R, pe)
        best = options[0]
        assert geo.apogee_longitude(-60.0, best['revs'], best['period'], gto, rate) == \
            pytest.approx(target, abs=1e-6)
        assert abs(best['dv']) < 150.0


def test_final_approach_thrusts_toward_a_distant_still_target_and_brakes_an_overshoot():
    from ksl.ascent.rendezvous import translation_command
    right, forward, up = translation_command((0.0, 500.0, 0.0), (0.0, 0.0, 0.0), stop=50.0)
    assert forward > 0 and right == 0 and up == 0
    # target rushing toward us faster than scheduled near the stop range: brake
    _, forward, _ = translation_command((0.0, 60.0, 0.0), (0.0, -2.0, 0.0), stop=50.0)
    assert forward < 0
