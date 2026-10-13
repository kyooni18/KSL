"""Pure offline safety regressions for KSP's Benjee OV-100 bay event."""
from dataclasses import dataclass, field
from types import SimpleNamespace

from Tools.ksp_test_guard import ensure_sts_payload_bay_closed


@dataclass
class FakeToggle:
    gui_name: str
    name: str = 'Toggle'
    active: bool = True
    triggers: int = 0
    transition: bool = True

    def trigger(self):
        self.triggers += 1
        if self.transition:
            self.gui_name = 'Open'


@dataclass
class FakeModule:
    event: FakeToggle
    name: str = 'ModuleAnimateGeneric'

    @property
    def event_list(self):
        return [self.event]


@dataclass
class FakePart:
    toggle: FakeToggle
    name: str = 'benjee10.shuttle.midFuselage'
    modules: list = field(default_factory=list)

    def __post_init__(self):
        if not self.modules:
            self.modules = [FakeModule(self.toggle)]


def vessel(*parts):
    return SimpleNamespace(parts=SimpleNamespace(all=list(parts)))


def test_open_bay_is_closed_by_active_close_event():
    event = FakeToggle('Close')
    result = ensure_sts_payload_bay_closed(vessel(FakePart(event)))
    assert result == {'status': 'closed', 'bayCount': 1, 'triggered': 1, 'pending': 0}
    assert event.triggers == 1 and event.gui_name == 'Open'


def test_closed_bay_is_not_opened_by_blind_toggle():
    event = FakeToggle('Open')
    result = ensure_sts_payload_bay_closed(vessel(FakePart(event)))
    assert result['status'] == 'already-closed' and result['triggered'] == 0
    assert event.triggers == 0


def test_multiple_bays_only_trigger_open_one():
    open_event, closed_event = FakeToggle('Close'), FakeToggle('Open')
    result = ensure_sts_payload_bay_closed(vessel(FakePart(open_event), FakePart(closed_event)))
    assert result == {'status': 'closed', 'bayCount': 2, 'triggered': 1, 'pending': 0}
    assert open_event.triggers == 1 and closed_event.triggers == 0


def test_unsupported_vessel_skipped_without_touching_animation():
    other = FakePart(FakeToggle('Close'), name='other.parts.cargoBay')
    assert ensure_sts_payload_bay_closed(vessel(other)) == {
        'status': 'not-applicable', 'bayCount': 0, 'triggered': 0}
    assert other.toggle.triggers == 0


def test_unknown_event_does_not_issue_any_toggle():
    event = FakeToggle('Deploy')
    result = ensure_sts_payload_bay_closed(vessel(FakePart(event)))
    assert result['status'] == 'unknown' and result['reason'] == 'bay-close-event-unavailable'
    assert event.triggers == 0


def test_disabled_close_event_is_not_triggered():
    event = FakeToggle('Close', active=False)
    result = ensure_sts_payload_bay_closed(vessel(FakePart(event)))
    assert result['status'] == 'unknown' and event.triggers == 0


def test_paused_close_is_not_falsely_reported_complete():
    event = FakeToggle('Close', transition=False)
    result = ensure_sts_payload_bay_closed(vessel(FakePart(event)), paused=True)
    assert result == {'status': 'closing-pending', 'bayCount': 1, 'triggered': 1, 'pending': 1}
    assert event.triggers == 1


def test_unresponsive_animation_reports_pending_not_closed():
    event = FakeToggle('Close', transition=False)
    result = ensure_sts_payload_bay_closed(vessel(FakePart(event)), timeout=0)
    assert result['status'] == 'closing-pending' and result['pending'] == 1
    assert event.triggers == 1


def test_ambiguous_animation_module_refuses_toggle():
    event = FakeToggle('Close')
    part = FakePart(event, modules=[FakeModule(event), FakeModule(event)])
    result = ensure_sts_payload_bay_closed(vessel(part))
    assert result['status'] == 'unknown' and result['reason'] == 'bay-animation-module-ambiguous'
    assert event.triggers == 0
