"""Important opt-in, topology safety and achieved-orbit contracts."""
import copy
from pathlib import Path
from types import SimpleNamespace as NS
import math
import pytest
from ksl.config import DEFAULTS, config_from_snapshot, load_config
from ksl.programs import AscentProgram, OrbitTrimProgram
from ksl.ascent.intelligent.vehicle import Vehicle
from ksl.ascent.intelligent.planner import State, Target, Planner, achieved, coast_apoapsis
from ksl.ascent.intelligent.simulation import simulate


class Part:
    def __init__(self, name, stage=0, engine=False, command=False):
        self.name, self.stage, self.decouple_stage = name, stage, stage
        self.mass, self.children, self.command, self.decoupler = 100., [], command or None, None
        self.resources = NS(names=[])
        self.engine = NS(part=self, max_thrust=1000., vacuum_specific_impulse=300.,
                         throttle_locked=False, thrusters=[], active=True, has_fuel=False,
                         thrust=0.) if engine else None
    def position(self, frame):
        return 1., 2., 3.
    def direction(self, frame):
        return 0., 1., 0.


def model():
    root = Part('unknown-cockpit', command=True)
    sep = Part('unknown-separator')
    sep.decoupler = NS()
    motor = Part('unknown-motor', engine=True)
    root.children = [sep]
    sep.children = [motor]
    return Vehicle(NS(parts=NS(root=root), reference_frame=None, name='unfamiliar'))


def context(config):
    return NS(config=config, python='python', log=NS(directory=Path('/tmp/run')))


def test_default_and_explicit_routes_are_isolated():
    legacy = load_config()
    before = AscentProgram()._argv(context(legacy), False)
    assert before[3] == 'ksl.ascent.flight'
    intelligent = config_from_snapshot('test', legacy.data, ['ascent.guidanceMode=intelligent'])
    after = AscentProgram()._argv(context(intelligent), False)
    assert after[3] == 'ksl.ascent.intelligent.flight'
    assert '--ssme-limit' not in after and '--launch' in after
    assert OrbitTrimProgram().stages(context(intelligent)) == []
    assert AscentProgram()._argv(context(legacy), False) == before
    assert DEFAULTS['ascent']['guidanceMode'] == 'legacy'
    assert not intelligent.validate()
    invalid = config_from_snapshot('test', legacy.data, ['ascent.guidanceMode=typo'])
    assert any('guidanceMode' in x for x in invalid.validate())


def test_discovery_and_separation_require_positive_evidence():
    vehicle = model()
    assert len(vehicle.usable_engines()) == 1
    assert vehicle.exhausted_branch('0/0')
    vehicle.engines['0/0/0'].has_fuel = True
    assert not vehicle.exhausted_branch('0/0')
    vehicle.engines['0/0/0'].has_fuel = False
    vehicle.engines['0/0/0'].active = False
    assert not vehicle.exhausted_branch('0/0')
    assert not vehicle.exhausted_branch('0')
    doc = vehicle.document()
    doc['roles']['0/0/0'] = 'ignore'
    vehicle.apply(doc)
    assert not vehicle.usable_engines()
    assert not vehicle.exhausted_branch('0/0')
    doc['fingerprint'] = 'other vehicle'
    with pytest.raises(ValueError, match='does not match'):
        vehicle.apply(doc)


def test_invalid_telemetry_and_measured_acceptance():
    target = Target(600000., 3.5316e12, 70000., 90000., 90000., 34.)
    assert achieved(90500., 89500., 34.1, target)
    assert not achieved(90500., 88000., 34.1, target)
    assert not achieved(math.nan, 90000., 34., target)
    with pytest.raises(ValueError, match='Invalid flight state'):
        Planner(target).command(State(1000., math.nan, 0., 1000., 10000., 300.))
    state = State(30000., 600., 800., 20000., 0., 300., 20000., 4.)
    vacuum = State(30000., 600., 800., 20000., 0., 300.)
    assert coast_apoapsis(state, target) < coast_apoapsis(vacuum, target)


@pytest.mark.parametrize('mass,thrust,dry,target', [
    (30000.,600000.,5000.,80000.), (80000.,1200000.,12000.,120000.),
    (10000.,250000.,1500.,200000.)])
def test_independent_plant_reaches_measured_orbit(mass, thrust, dry, target):
    result = simulate(mass, thrust, 340., dry, target)
    assert result['success'], result
    assert abs(result['apoapsis']-target) < 1000
    assert abs(result['periapsis']-target) < 1000
    assert result['mass'] > dry
