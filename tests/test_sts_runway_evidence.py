import importlib.util
from pathlib import Path


def test_runway_evidence_accepts_timestamped_and_archived_commits():
    spec = importlib.util.spec_from_file_location(
        "run_geometry", Path(__file__).parents[1] / "ShuttleSim/scripts/run_geometry.py"
    )
    geometry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(geometry)
    archived = "MM305 energy-law runway end: RW09 end=0 score09=1 score27=2\n"
    current = "MM305 energy-law runway end: UT 26005.0 RW27 end=1 score09=2 score27=1\n"
    assert geometry.energy_law_runway_end(archived) == 0
    assert geometry.energy_law_runway_end(current) == 1
    assert geometry.energy_law_runway_end(archived + current) == 1
    assert geometry.energy_law_runway_end("no commitment") is None
