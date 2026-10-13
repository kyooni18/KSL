"""Offline evidence reconstruction tests; no simulator or connection required."""
import importlib.util
import json
import math
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location("polar_audit", Path(__file__).parents[1]/"Tools/polar_reentry_audit.py")
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


def test_recursive_delta_does_not_mutate_prior_sample(tmp_path):
    path = tmp_path/"vehicle.jsonl"
    records = [dict(recordType="sessionStart", sessionId="one"),
               dict(recordType="vehicleKeyframe", sessionId="one", recordSequence=1, ut=10., wallMonotonicSeconds=1., fields={"a":{"b":1,"keep":4},"items":[1]}),
               dict(recordType="vehicleDelta", sessionId="one", recordSequence=2, ut=11., wallMonotonicSeconds=2., fields={"a":{"b":2},"items":[3]})]
    path.write_text("\n".join(map(json.dumps,records)))
    _, rows = audit.load_vehicle(path)
    assert rows[0]["fields"] == {"a":{"b":1,"keep":4},"items":[1]}
    assert rows[1]["fields"] == {"a":{"b":2,"keep":4},"items":[3]}


def test_delta_without_keyframe_rejected(tmp_path):
    path = tmp_path/"bad.jsonl"
    path.write_text(json.dumps(dict(recordType="sessionStart",sessionId="one"))+"\n"+json.dumps(dict(recordType="vehicleDelta",sessionId="one",recordSequence=1,fields={})))
    with pytest.raises(ValueError,match="delta before keyframe"):
        audit.load_vehicle(path)


def test_wrong_session_rejected(tmp_path):
    path = tmp_path/"bad.jsonl"
    path.write_text(json.dumps(dict(recordType="sessionStart",sessionId="one"))+"\n"+json.dumps(dict(recordType="vehicleKeyframe",sessionId="two",recordSequence=1)))
    with pytest.raises(ValueError,match="session"):
        audit.load_vehicle(path)


def test_clock_step_and_later_physical_catchup_are_distinct():
    v = [100.,0.,0.]
    # UT +3.6 while the position advances .1 physical seconds.
    dt1 = audit.kinematic_dt([0,0,0],v,[10,0,0],v)
    assert dt1 == pytest.approx(.1)
    # A following +.1 UT with +3.6 physical seconds cancels the discrepancy.
    dt2 = audit.kinematic_dt([10,0,0],v,[370,0,0],v)
    assert (3.6-dt1)+(.1-dt2) == pytest.approx(0)
    # Normal next physics tick does not cancel a genuine persistent clock step.
    dt3 = audit.kinematic_dt([10,0,0],v,[20,0,0],v)
    assert (3.6-dt1)+(.1-dt3) == pytest.approx(3.5)


def test_irregular_polynomial_derivative():
    times=[0.,.2,1.,1.4,2.2,3.]
    p=[[t**4,2*t**2,7*t] for t in times]
    for i,t in enumerate(times):
        assert audit.derivative(times,p,i)==pytest.approx([4*t**3,4*t,7.],abs=1e-10)


def test_first_descending_crossing_retains_clock_bracket():
    rows=[dict(altitude=100.,ut=0.,longitude=179.,clock_error_step=0.),
          dict(altitude=90.,ut=4.,longitude=-179.,clock_error_step=3.5),
          dict(altitude=100.,ut=8.,longitude=-170.,clock_error_step=0.),
          dict(altitude=90.,ut=9.,longitude=-169.,clock_error_step=0.)]
    r=audit.crossing(rows,95.)
    assert r["ut"]==2. and r["longitude"]==180.
    assert r["bracket_clock_error"]==3.5


def test_frame_round_trip_at_large_ut_and_rotated_epoch():
    p=dict(radius=600000.,northAxis=[0,0,1],primeMeridianAtEpoch=[-.8,.6,0],rotationalSpeed=.00029157,epochUT=146972703.5)
    site=dict(latitude=-.0486,longitude=-74.7283)
    for ut in [146972703.5,146991335.8,146992703.5]:
        position=audit.inertial(p,48.,179.99,50000.,ut)
        g=audit.geometry(p,site,position,[100,200,300],ut)
        assert g["latitude"]==pytest.approx(48.)
        assert g["longitude"]==pytest.approx(179.99)
        assert g["altitude"]==pytest.approx(50000.)


def test_periapsis_circular_orbit():
    p=dict(radius=600000.,gravitationalParameter=3.5316e12)
    r=700000.
    assert audit.periapsis([r,0,0],[0,math.sqrt(p["gravitationalParameter"]/r),0],p)==pytest.approx(100000.,abs=.02)


def test_incomplete_pre_taem_recording_does_not_claim_landing():
    rows = [dict(ut=1., phase="Deorbit Burn"), dict(ut=2., phase="MM304 Entry")]
    result = audit.classify_observed_outcome(rows, dict(state="running"))
    assert result["statusState"] == "running"
    assert result["lastObservedPhase"] == "MM304 Entry"
    assert result["mm305Observed"] is False
    assert result["finalObserved"] is False
    assert result["touchdownPhaseObserved"] is False
    assert "incomplete" in result["description"]


def test_abort_and_full_completion_have_distinct_certificates():
    aborted = audit.classify_observed_outcome([dict(ut=4., phase="TAEM")], dict(state="aborted"))
    assert "aborted" in aborted["description"]
    assert aborted["mm305Observed"] and not aborted["finalObserved"]
    complete = audit.classify_observed_outcome(
        [dict(ut=float(i), phase=name) for i, name in enumerate(
            ("TAEM", "Final Approach", "Touchdown", "Rollout", "Complete"))],
        dict(state="completed"))
    assert complete["completePhaseObserved"]
    assert complete["touchdownPhaseObserved"] and complete["rolloutPhaseObserved"]
    assert "completed" in complete["description"]
    unverified = audit.classify_observed_outcome([dict(ut=1., phase="TAEM")], dict(state="completed"))
    assert "unverified" in unverified["description"]
