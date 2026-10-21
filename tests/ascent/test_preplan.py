"""Vehicle dependence, full-trajectory targeting, and honest feasibility gates."""
from dataclasses import replace
import json
import math
from pathlib import Path

import pytest

from ksl.ascent.preplan import MassPoint, VehicleModel, evaluate, optimize

FIXTURE = json.loads((Path(__file__).parents[1]/'fakes/ascent-planning-model.json').read_text())


def model():
    return VehicleModel.from_document(FIXTURE['model'])


def test_depletion_and_staging_move_com_and_change_control_inertia():
    m = model()
    launch_mass, launch_com, launch_inertia = m.mass_state(1., 1., True)
    depleted_mass, depleted_com, depleted_inertia = m.mass_state(.5, 0., True)
    staged_mass, staged_com, staged_inertia = m.mass_state(.5, 0., False)
    assert launch_mass == pytest.approx(520000.)
    assert depleted_mass == pytest.approx(190000.)
    assert staged_mass == pytest.approx(140000.)
    assert launch_com != depleted_com != staged_com
    assert staged_inertia != depleted_inertia != launch_inertia


def test_prelaunch_search_reaches_both_apsides_without_lofting():
    m = model()
    result = optimize(m, 90000., 60000.)
    assert result.feasible, result.violations
    assert abs(result.samples[-1].apoapsis-90000.) < 1000.
    assert abs(result.samples[-1].periapsis-60000.) < 1000.
    assert max(s.apoapsis for s in result.samples) < 93000.
    assert result.propellant < 350000.
    assert any(s.separated for s in result.samples)
    assert result.samples[-1].mass > 50000.
    # The old fixed family can consume fuel without satisfying the same orbit.
    upright = evaluate(m, 90000., 60000., [340., 85., 65., 45., 25., 10., 0.], retain=False)
    assert result.score < upright.score


def test_missing_control_authority_cannot_qualify_a_ballistic_prediction():
    m = model()
    m = replace(m, points=(*m.points, MassPoint(10000., (0., 8., 5.))),
                core=tuple(replace(engine, gimbal=0.) for engine in m.core),
                boosters=tuple(replace(engine, gimbal=0.) for engine in m.boosters))
    result = evaluate(m, 90000., 60000., FIXTURE['reference_parameters'], dt=.5)
    assert not result.feasible
    assert result.violations['trim_fraction'] > 0.


def test_saved_plan_is_reproducible_and_feedback_is_bounded(tmp_path):
    m = model()
    result = evaluate(m, 90000., 60000., FIXTURE['reference_parameters'], dt=.25)
    ref = result.reference(100.)
    assert result.tracking_pitch(100., ref['altitude'], ref['radial'], ref['horizontal']) == ref['pitch']
    # Excess climb requests a pitch-down correction without rewriting the plan.
    assert result.tracking_pitch(100., ref['altitude']+100000., ref['radial'], ref['horizontal']) == ref['pitch']-8.
    path = tmp_path/'trajectory.json'
    result.save(path, m)
    document = json.loads(path.read_text())
    restored = VehicleModel.from_document(document['model'])
    assert math.isinf(restored.separation_max_q)
    assert restored.mass_state(.5, .25, True) == m.mass_state(.5, .25, True)
    assert document['trajectory']['feasible'] == result.feasible


def test_corrupt_renderer_bounds_do_not_become_astronomical_drag():
    from types import SimpleNamespace
    from ksl.ascent.preplan_vehicle import estimate_drag_area
    parts = [SimpleNamespace(position=lambda frame: (-5., -10., -3.),
                             bounding_box=lambda frame: ((-6., -11., -4.), (-4., -9., -2.))),
             SimpleNamespace(position=lambda frame: (5., 10., 3.),
                             bounding_box=lambda frame: ((-1e38,)*3, (1e38,)*3))]
    vessel = SimpleNamespace(bounding_box=lambda frame: ((-1e38,)*3, (1e38,)*3))
    area, source = estimate_drag_area(vessel, 'body', parts)
    assert area == pytest.approx(.3*12.*8.)
    assert 'invalid vessel bounds discarded' in source
    # Valid measured extents still take precedence over the fallback estimate.
    vessel.bounding_box = lambda frame: ((-7., -12., -5.), (7., 12., 5.))
    assert estimate_drag_area(vessel, 'body', parts)[0] == pytest.approx(.3*14.*10.)


def test_impossible_drag_input_is_rejected_before_search():
    with pytest.raises(ValueError, match='environment or vehicle limits'):
        replace(model(), drag_area=6.817162944202548e35)


def test_booster_floating_point_residue_does_not_abort_at_zero_dt():
    m = model()
    m = replace(m, points=tuple(replace(p, mass=1e-9) if p.pool == 'solid' else p
                               for p in m.points))
    result = evaluate(m, 90000., 60000., FIXTURE['reference_parameters'], dt=.25)
    assert result.samples[0].separated
    assert result.samples[-1].time >= .25
