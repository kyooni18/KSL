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


def test_rotation_and_fixed_thrust_are_not_throttleable_airspeed():
    from ksl.ascent.intelligent.planner import step
    target = Target(600000., 3.5316e12, 70000., 90000., 90000.)
    pad = State(800., 0., 175., 30000., 600000., 300., surface_horizontal=0.)
    assert Planner(target).command(pad).pitch == 90.
    solid = State(10000., 200., 100., 30000., 600000., 300., fixed_thrust=400000.)
    # Zero liquid throttle must still predict SRB acceleration and depletion.
    next_state = step(solid, target, 90., 0., 1.)
    assert next_state.mass < solid.mass
    assert next_state.radial > solid.radial


def test_autopilot_property_compatibility():
    from ksl.ascent.intelligent.flight import engage
    autopilot = NS(engaged=False)
    engage(autopilot, True)
    assert autopilot.engaged
    engage(autopilot, False)
    assert not autopilot.engaged


@pytest.mark.parametrize('thrust', [(0.,1.,0.), (1.,0.,0.), (0.,-1.,0.), (.3,.7,.2)])
def test_thrust_alignment_handles_large_cants(thrust):
    from ksl.ascent.intelligent.flight import rotate_thrust_to_target
    from ksl.ascent.orbit_math import unit
    target = unit((.7,.2,.3))
    result = rotate_thrust_to_target(unit(thrust), thrust, target)
    assert result == pytest.approx(target)


def test_staged_mass_drop_and_elliptical_insertion():
    result = simulate(mass=30000., altitude=200000., periapsis=90000.,
                      stages=[(600000.,300.,12000.,3000.), (200000.,350.,3000.,0.)])
    assert result['success'], result
    assert abs(result['apoapsis']-200000.) < 1000
    assert abs(result['periapsis']-90000.) < 1000


def test_burn_alignment_uses_force_not_roll_and_disables_competing_sas():
    from ksl.ascent.intelligent.flight import aim
    engine = NS(active=True, thrust=0., available_thrust=1000.,
                thrusters=[NS(thrust_direction=lambda frame: (0.,1.,0.))])
    ap = NS(engaged=False, error=180.)
    control = NS(sas=True)
    vessel = NS(parts=NS(engines=[engine]), reference_frame='body', auto_pilot=ap, control=control)
    conn = NS(space_center=NS(transform_direction=lambda vector, src, dst: vector))
    error = aim(conn, vessel, 'inertial', (0.,1.,0.))
    assert error == pytest.approx(0.)
    assert ap.error == 180.  # roll must not prohibit an aligned burn
    assert ap.engaged and not control.sas
    assert ap.target_direction == (0.,1.,0.)


def test_role_correction_cannot_silently_authorize_controlled_branch_release():
    vehicle = model()
    motor = next(c for c in vehicle.components if c.key == '0/0/0')
    motor.command = True
    assert not vehicle.exhausted_branch('0/0')
    doc = vehicle.document()
    doc['approvedSeparators'] = ['0/0']
    vehicle.apply(doc)
    assert vehicle.exhausted_branch('0/0')
