"""Deterministic phase-independent descent checkpoint triggers (no KSP)."""
from __future__ import annotations

from types import SimpleNamespace
from unittest import TestCase

from ksl.config import load_config
from ksl.programs import _LandingState


class FakeLog:
    def __init__(self):
        self.status = SimpleNamespace(run_id="unit-test", phase="mm304", connection="connected")
        self.events = []
        self.messages = []
        self.saved_files = {}

    def emit(self, event):
        self.events.append(event)

    def log(self, message, level="info"):
        self.messages.append((level, message))

    def write_json(self, path, value):
        self.saved_files[path] = list(value)

    def update(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self.status, key, value)

    def telemetry(self, *_args):
        pass


class FakeBackend:
    def __init__(self):
        self.calls = []

    def send(self, method, **kwargs):
        self.calls.append((method, kwargs))


class DescentCheckpointTests(TestCase):
    def test_descending_crossing_saves_once_at_72_and_45_km(self):
        log = FakeLog()
        config = load_config("sts-polar")
        ctx = SimpleNamespace(config=config, log=log, advance=lambda phase: None)
        state = _LandingState(ctx)
        state.engaged = True
        backend = FakeBackend()

        def observe(altitude, vertical=-90, ut=10):
            state.on_snapshot({"phase": "MM304 Entry", "connectionStatus": "connected",
                               "telemetry": {"meanAltitude": altitude, "verticalSpeed": vertical, "ut": ut}})
            state.flush(backend)

        observe(73000)
        observe(72100)
        self.assertEqual(len(backend.calls), 0)
        observe(71950, ut=11)
        observe(71800, ut=12)
        observe(46000, ut=30)
        observe(44930, ut=31)
        observe(44800, ut=32)
        self.assertEqual(len(backend.calls), 2)
        self.assertEqual([args["name"].rsplit("-", 2)[-2:] for _method, args in backend.calls],
                         [["descent", "72km"], ["descent", "45km"]])
        self.assertEqual([round(r["measuredAltitude"]) for r in log.saved_files["descent-checkpoints.json"]],
                         [71950, 44930])
        self.assertEqual(len([e for e in log.events if e.kind == "checkpoint"]), 2)

    def test_skipped_altitude_not_mislabeled(self):
        log = FakeLog()
        state = _LandingState(SimpleNamespace(config=load_config("sts-polar"), log=log,
                                               advance=lambda phase: None))
        state.engaged = True
        backend = FakeBackend()
        for h in (80000, 69500, 42000):
            state.on_snapshot({"phase": "MM304 Entry", "telemetry": {"meanAltitude": h,
                                                                       "verticalSpeed": -200, "ut": 10}})
        state.flush(backend)
        self.assertEqual(backend.calls, [])
        self.assertTrue(any("missed" in msg for _level, msg in log.messages))

    def test_resume_below_first_threshold_keeps_lower_checkpoint_armed(self):
        log = FakeLog()
        state = _LandingState(SimpleNamespace(config=load_config("sts-polar"), log=log,
                                               advance=lambda phase: None))
        state.engaged = True
        backend = FakeBackend()
        for altitude in (70600, 46000, 44920, 44800):
            state.on_snapshot({"phase": "MM304 Entry",
                               "telemetry": {"meanAltitude": altitude, "verticalSpeed": -80,
                                             "ut": altitude / 1000}})
            state.flush(backend)
        self.assertEqual(len(backend.calls), 1)
        self.assertTrue(backend.calls[0][1]["name"].endswith("-descent-45km"))
        self.assertEqual(round(log.saved_files["descent-checkpoints.json"][0]["measuredAltitude"]), 44920)
        self.assertTrue(any("72 km already passed" in msg and "not backfilled" in msg
                            for _level, msg in log.messages))
        self.assertFalse(any("72km" in e.message for e in log.events if e.kind == "checkpoint"))

    def test_resume_below_all_thresholds_does_not_fabricate_saves(self):
        log = FakeLog()
        state = _LandingState(SimpleNamespace(config=load_config("sts-polar"), log=log,
                                               advance=lambda phase: None))
        state.engaged = True
        backend = FakeBackend()
        for altitude in (44000, 42500):
            state.on_snapshot({"phase": "MM304 Entry",
                               "telemetry": {"meanAltitude": altitude, "verticalSpeed": -50,
                                             "ut": altitude / 1000}})
            state.flush(backend)
        self.assertEqual(backend.calls, [])
        self.assertEqual(sum("not backfilled" in msg for _level, msg in log.messages), 2)

    def test_invalid_altitude_configuration_rejected(self):
        config = load_config("sts-polar")
        config.data["checkpoints"]["kspSaveOnDescendingAltitudesKm"] = ["NaN"]
        self.assertTrue(any("invalid altitude" in s for s in config.validate()))
