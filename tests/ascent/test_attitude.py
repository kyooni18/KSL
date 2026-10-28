"""Topology changes must refresh dynamics before the next control step."""
import threading
import math
from types import SimpleNamespace
from unittest.mock import Mock, patch
from krpc.schema import KRPC_pb2
import pytest
from ksl.ascent.attitude import AttitudeController, CustomAutopilot, rcs_up_translation_moment
from ksl.ascent.guidance import attitude_settled
from ksl.ascent.vehicle import Shuttle, separate_sts_boosters_batch
from ksl.ascent.geometry import direction_error_deg, slew_direction, smooth_value


def test_separation_invalidation_refreshes_dynamics_without_waiting_for_timer():
    controller = AttitudeController.__new__(AttitudeController)
    controller._dynamics_dirty = threading.Event()
    controller._last_slow = 100.
    controller._integral = [10., 20., 30.]
    controller._refresh_slow = Mock()
    controller._update_dynamics(100.01)
    controller._refresh_slow.assert_not_called()
    controller.invalidate_dynamics()
    controller._update_dynamics(100.02)
    controller._refresh_slow.assert_called_once()
    assert controller._integral == [0., 0., 0.]
    assert not controller._dynamics_dirty.is_set()

    # An event arriving during the RPC reads must survive for the next step.
    controller._refresh_slow.side_effect = controller.invalidate_dynamics
    controller.invalidate_dynamics()
    controller._update_dynamics(100.03)
    assert controller._dynamics_dirty.is_set()


def test_separation_hold_keeps_measured_nose_and_roll_for_five_simulation_seconds():
    controller = AttitudeController.__new__(AttitudeController)
    controller.lock = threading.Lock()
    controller._rotation = lambda: (0., 0., 0., 1.)
    now = [100.]
    controller._ut = lambda: now[0]
    controller.forward = (0., 1., 0.)
    controller.roof = (0., 0., -1.)
    controller._hold_until = None
    controller._held_attitude = None
    controller.hold_attitude(105.)
    controller.set_target((1., 0., 0.), (0., 1., 0.))
    for t in (100., 102., 104.999):
        now[0] = t
        assert controller._target_for_step() == ((0., 1., 0.), (0., 0., -1.))
    now[0] = 105.
    assert controller._target_for_step() == ((1., 0., 0.), (0., 1., 0.))
    assert controller._hold_until is None


def test_preseparation_capture_is_preserved_until_after_actual_staging():
    controller = AttitudeController.__new__(AttitudeController)
    controller.lock = threading.Lock()
    controller._rotation = lambda: (0., 0., 0., 1.)
    now = [96.]
    controller._ut = lambda: now[0]
    controller.forward = (1., 0., 0.)
    controller.roof = None
    controller._hold_until = None
    controller._held_attitude = None
    assert not controller.extend_attitude_hold(105.)
    controller.hold_attitude(float('inf'))
    controller._rotation = lambda: (0., 0., 1., 0.)
    now[0] = 100.
    assert controller.extend_attitude_hold(105.)
    assert controller._target_for_step() == ((0., 1., 0.), (0., 0., -1.))
    now[0] = 105.
    assert controller._target_for_step() == (controller.forward, None)


def test_separation_hold_does_not_freeze_existing_roll_error():
    controller = AttitudeController.__new__(AttitudeController)
    controller.lock = threading.Lock()
    controller._rotation = lambda: (0., 0., 0., 1.)
    controller._ut = lambda: 96.
    controller.forward = (0., 1., 0.)
    controller.roof = (.5, 0., -math.sqrt(.75))
    controller._hold_until = None
    controller._held_attitude = None
    controller.hold_attitude(105., preserve_target_roof=True)
    assert controller._target_for_step() == ((0., 1., 0.), controller.roof)


def test_meco_target_hold_and_settle_check_include_all_axis_rates():
    controller = AttitudeController.__new__(AttitudeController)
    controller.lock = threading.Lock()
    controller._ut = lambda: 100.
    controller.forward = (0., 1., 0.)
    controller.roof = (0., 0., -1.)
    controller._hold_until = None
    controller._held_attitude = None
    assert controller.hold_commanded_attitude(105.)
    controller.set_target((1., 0., 0.), (0., 1., 0.))
    assert controller._target_for_step() == ((0., 1., 0.), (0., 0., -1.))
    assert attitude_settled(2., 1., (0., 0., math.radians(.4)))
    assert not attitude_settled(2., 1., (0., 0., math.radians(.6)))
    assert not attitude_settled(2., 4., (0., 0., 0.))
    controller.release_attitude_hold()
    assert controller._target_for_step() == ((1., 0., 0.), (0., 1., 0.))


def test_rcs_roofward_moment_accounts_for_each_block_location():
    def block(y, thrust):
        return SimpleNamespace(enabled=True, up_enabled=True,
                               available_force=((0., 0., 0.), (0., 0., -thrust)),
                               part=SimpleNamespace(position=lambda _: (0., y, 0.)),
                               thrusters=[SimpleNamespace(
                                   thrust_direction=lambda _: (0., 0., -1.),
                                   thrust_position=lambda _: (0., y, 0.))])
    vessel = SimpleNamespace(parts=SimpleNamespace(rcs=[block(10., 100.), block(-5., 100.)]))
    assert rcs_up_translation_moment(vessel, object()) == (-500., 0., 0.)
    angled = block(10., 80.)
    angled.thrusters[0].thrust_direction = lambda _: (.6, 0., -.8)
    vessel.parts.rcs = [angled]
    assert rcs_up_translation_moment(vessel, object()) == pytest.approx((-800., 0., -600.))
    angled.thrusters[0].thrust_direction = lambda _: (0., 1., 0.)
    assert rcs_up_translation_moment(vessel, object()) == pytest.approx((-800., 0., 0.))
    vessel.parts.rcs = []
    assert rcs_up_translation_moment(vessel, object()) is None


def test_settled_booster_roll_uses_conservative_authority_and_input_limit():
    controller = AttitudeController.__new__(AttitudeController)
    controller._target_for_step = lambda: ((0., 1., 0.), (.1, 0., -math.sqrt(.99)))
    controller._rotation = lambda: (0., 0., 0., 1.)
    controller._angvel = lambda: (0., 0., 0.)
    controller.control = SimpleNamespace(pitch=0., roll=0., yaw=0.)
    controller.roll_gate = (8., 5.)
    controller._torque = (100., 100., 100.)
    controller._moi = (10., 10., 10.)
    controller.brake_fraction = (.6, .25, .6)
    controller.max_rate = (1., 1., 1.)
    controller.tau = (2., 2., 2.)
    controller.rate_tau = .6
    controller._integral = [0., 0., 0.]
    controller.disturbance = (0., 0., 0.)
    controller.translation_moment = (0., 0., 0.)
    controller.int_limit = .35
    controller.int_time = 6.
    controller.dt = .04
    controller.deadband = 0.
    controller._telemetry = None
    controller.roll_authority_fraction = 1.
    controller.roll_input_limit = 1.
    controller._step()
    baseline = abs(controller.control.roll)
    controller._integral = [0., 0., 0.]
    controller.roll_authority_fraction = .25
    controller.roll_input_limit = .5
    controller._step()
    assert abs(controller.control.roll) == pytest.approx(4*baseline)
    controller._target_for_step = lambda: ((0., 1., 0.), (1., 0., 0.))
    controller._step()
    assert abs(controller.control.roll) <= .5


def test_capture_callback_runs_before_detachment():
    shuttle = Shuttle.__new__(Shuttle)
    shuttle.et_attached = False
    shuttle.separated = False
    shuttle.vehicle_kind = 'other'
    shuttle.solid_boosters = []
    shuttle._liquid_booster_depleted = lambda: True
    shuttle._solid_boosters_depleted = lambda: False
    events = []
    def separate():
        events.append('separate')
        shuttle.separated = True
    shuttle._separate_boosters = separate
    shuttle.groups = [[SimpleNamespace(thrust_limit=.5)]]
    shuttle.allocation = lambda **kwargs: SimpleNamespace(throttles=(.5,))
    shuttle.allocation_for_throttles = lambda values: values
    shuttle.update(before_booster_separation=lambda: events.append('capture'))
    assert events == ['capture', 'separate']


def test_thrust_correction_jump_is_limited_by_simulation_time():
    previous = (0., 1., 0.)
    target = (math.sin(math.radians(27.)), math.cos(math.radians(27.)), 0.)
    first = slew_direction(previous, target, 2., .1)
    assert abs(direction_error_deg(previous, first) - .2) < 1e-8
    assert direction_error_deg(slew_direction(first, target, 2., 20.), target) < 1e-6
    assert direction_error_deg(previous, slew_direction(previous, target, 2., 0.)) < 1e-6
    opposite = slew_direction(previous, (0., -1., 0.), 2., 1.)
    assert abs(direction_error_deg(previous, opposite) - 2.) < 1e-8


def test_ascent_pitch_response_is_smooth_without_a_fixed_rate_cap():
    command = smooth_value(89., 29., 5., 1.)
    assert 29. < command < 89.
    assert 89.-command > 0.4
    assert smooth_value(89., 29., 5., 0.) == 89.
    ap = CustomAutopilot.__new__(CustomAutopilot)
    object.__setattr__(ap, 'ctrl', SimpleNamespace(unlimited_pitch_rate=True))
    ap.max_angular_velocity = (1., .2, 1.)
    assert math.isinf(ap.ctrl.max_rate[0])
    assert ap.ctrl.max_rate[1] == .2
    assert ap.ctrl.max_rate[2] == pytest.approx(math.radians(5.))


def test_srb_cutoff_ignition_and_detachment_share_one_request():
    class Socket:
        def __init__(self):
            self.requests = []
            self.fail = False
        def send_message(self, request):
            self.requests.append(request)
        def receive_message(self, response_type):
            response = response_type()
            for _ in self.requests[-1].calls:
                response.results.add()
            if self.fail:
                response.results[-1].error.description = 'decoupler failed'
            return response

    class Client:
        def __init__(self):
            self._rpc_connection = Socket()
            self._rpc_connection_lock = threading.Lock()
            self._types = SimpleNamespace(class_type=lambda *args: object(),
                                          float_type=object(), bool_type=object(),
                                          double_type=object())
            self.space_center = object()
        def _build_call(self, service, procedure, args, *rest):
            return KRPC_pb2.ProcedureCall(service=service, procedure=procedure)
        def get_call(self, method, *args):
            procedure = 'get_UT' if method is getattr else 'Decoupler_Decouple'
            return KRPC_pb2.ProcedureCall(service='SpaceCenter', procedure=procedure)

    conn = Client()
    core = SimpleNamespace(thrust_limit=.9)
    motors = [object(), object()]
    decouplers = [SimpleNamespace(decouple=Mock()), SimpleNamespace(decouple=Mock())]
    with patch('ksl.ascent.vehicle.Decoder.decode', return_value=123.0):
        assert separate_sts_boosters_batch(conn, [[core]], [object(), object()],
                                           motors, decouplers) == (123., 123.)
    assert len(conn._rpc_connection.requests) == 1
    procedures = [c.procedure for c in conn._rpc_connection.requests[0].calls]
    assert procedures == (['Engine_set_ThrustLimit'] +
                          ['Engine_set_ThrustLimit', 'Engine_set_Active']*2 +
                          ['Engine_set_ThrustLimit']*2 + ['get_UT'] +
                          ['Engine_set_Active']*2 + ['Decoupler_Decouple']*2 + ['get_UT'])
    conn._rpc_connection.fail = True
    with pytest.raises(RuntimeError, match='staging may be partial'):
        separate_sts_boosters_batch(conn, [[core]], [], motors, decouplers)


def test_batch_failure_finishes_only_attached_branch_without_aborting():
    detached, attached, motor_part = object(), object(), object()
    class Decoupler:
        def __init__(self, part):
            self.part = part
            self.decoupled = False
            self.calls = 0
        def decouple(self):
            self.calls += 1
            self.decoupled = True
    first, second = Decoupler(detached), Decoupler(attached)
    motor = SimpleNamespace(part=motor_part, active=False, has_fuel=True, thrust_limit=0.)
    shuttle = Shuttle.__new__(Shuttle)
    shuttle.vehicle_kind = 'orbiter-shuttle'
    shuttle.vessel = SimpleNamespace(parts=SimpleNamespace(all=[attached, motor_part]))
    shuttle.conn = object()
    shuttle.sc = SimpleNamespace(ut=100.)
    shuttle.core_groups = [[object()]]
    shuttle.liquid_boosters = []
    shuttle.solid_boosters = [object(), object()]
    shuttle.sepmotors = [motor]
    shuttle.booster_branch_parts = [[detached], [attached, motor_part]]
    shuttle.booster_decouplers = [first, second]
    shuttle.cruise_control_moment_budget = 1.
    with patch('ksl.ascent.vehicle.separate_sts_boosters_batch',
               side_effect=RuntimeError('server failure')) as batch:
        assert shuttle._separate_boosters()
        batch.assert_called_once()
    assert first.calls == 0 and second.calls == 1
    assert motor.active and motor.thrust_limit == 1.
    assert shuttle.separated and shuttle.booster_separation_ut == 100.
