import importlib.util
import json
from pathlib import Path


def test_live_aero_requires_live_source_evidence(tmp_path):
    spec = importlib.util.spec_from_file_location(
        "fit_sts_aero_cells", Path(__file__).parents[1] / "Tools/fit_sts_aero_cells.py"
    )
    fitter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fitter)
    path = tmp_path / "STS-vehicle.jsonl"
    header = dict(recordType="sessionStart", vessel="STS", sessionId="flight-session",
                  runtime={"transport": "tcp:127.0.0.1:50000"})
    control = dict(recordType="vehicleKeyframe", fields={
        "appliedControl": {"reportedAvailable": True}})
    def write(*records):
        path.write_text("\n".join(json.dumps(r) for r in records))
    write(header, control)
    assert fitter.live_session(path) == ("flight-session", "live")
    # Matching vessel/schema/control fields cannot override explicit sim provenance.
    write(dict(header, campaignIdentity="sim:synthetic-flight"), control)
    assert fitter.live_session(path) == (None, "simulator")
    write(header)
    assert fitter.live_session(path)[0] is None
    write(control)
    assert fitter.live_session(path)[0] is None
