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
