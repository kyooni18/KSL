#!/usr/bin/env python3
"""Pure tests for headless live-test guard interpreter resolution."""
from __future__ import annotations

import importlib.util
import io
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("headless_flight_under_test", ROOT / "Tools" / "headless_flight.py")
assert SPEC and SPEC.loader
HEADLESS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HEADLESS)


class GuardPythonTests(unittest.TestCase):
    def test_candidates_prefer_configured_then_project_then_legacy_then_current(self) -> None:
        root = Path("/tmp/Code/C/KSPShuttleLander")
        candidates = HEADLESS._guard_python_candidates(root, "/configured/python", "/current/python")
        self.assertEqual(candidates[0], Path("/configured/python"))
        self.assertEqual(candidates[1], root / "Runtime/PythonBridge/.venv/bin/python")
        self.assertIn(Path("/tmp/Code/Python/KSPFlightComputer/Shuttle/venv/bin/python"), candidates)
        self.assertEqual(candidates[-1], Path("/current/python"))

    def test_missing_configured_interpreter_falls_back_to_valid_legacy_environment(self) -> None:
        root = Path("/tmp/Code/C/KSPShuttleLander")
        legacy = Path("/tmp/Code/Python/KSPFlightComputer/Shuttle/venv/bin/python")

        def probe(candidate: str) -> tuple[bool, str]:
            return (True, "ok") if Path(candidate) == legacy else (False, "missing")

        with mock.patch.dict(os.environ, {"KSP_LANDER_GUARD_PYTHON": "/stale/python"}, clear=False), \
             mock.patch.object(HEADLESS, "_guard_python_probe", side_effect=probe), \
             mock.patch.object(HEADLESS.sys, "executable", "/current/python"):
            self.assertEqual(HEADLESS.guard_python(root), legacy)

    def test_existing_python_without_krpc_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            fake_python = Path(tmp) / "python"
            fake_python.write_text("#!/bin/sh\nexit 1\n", encoding="utf-8")
            fake_python.chmod(0o755)
            HEADLESS._guard_python_probe.cache_clear()
            usable, reason = HEADLESS._guard_python_probe(str(fake_python))
            self.assertFalse(usable)
            self.assertIn("cannot import krpc", reason)

    def test_error_lists_rejected_candidates_and_recovery_instruction(self) -> None:
        root = Path("/tmp/Code/C/KSPShuttleLander")
        with mock.patch.dict(os.environ, {"KSP_LANDER_GUARD_PYTHON": "/broken/python"}, clear=False), \
             mock.patch.object(HEADLESS, "_guard_python_probe", return_value=(False, "missing")), \
             mock.patch.object(HEADLESS.sys, "executable", "/current/python"):
            with self.assertRaises(RuntimeError) as context:
                HEADLESS.guard_python(root)
        message = str(context.exception)
        self.assertIn("KSP_LANDER_GUARD_PYTHON", message)
        self.assertIn("/broken/python (missing)", message)
        self.assertIn("/current/python (missing)", message)




def load_guard_module(fake_connect):
    fake_krpc = types.ModuleType("krpc")
    fake_krpc.connect = fake_connect
    spec = importlib.util.spec_from_file_location("ksp_test_guard_under_test", ROOT / "Tools" / "ksp_test_guard.py")
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, {"krpc": fake_krpc}):
        spec.loader.exec_module(module)
    return module


class CrashGuardFailSafeTests(unittest.TestCase):
    def test_lifecycle_watch_covers_save_mutations_and_live_runs_only(self) -> None:
        self.assertTrue(HEADLESS.lifecycle_watch_required(no_quickload=False, live=False, connect_only=True))
        self.assertTrue(HEADLESS.lifecycle_watch_required(no_quickload=True, live=True, connect_only=False))
        self.assertFalse(HEADLESS.lifecycle_watch_required(no_quickload=True, live=False, connect_only=True))

    def test_failed_cleanup_never_authorizes_startup_holder_release(self) -> None:
        self.assertEqual(HEADLESS.cleanup_handoff_state(False), "failed")
        self.assertEqual(HEADLESS.cleanup_handoff_state(True), "released")

    def test_crash_guard_is_process_group_independent_and_tracks_parent(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            process = mock.Mock()
            with mock.patch.object(HEADLESS, "guard_python", return_value=Path("/python")), \
                 mock.patch.object(HEADLESS, "guard_script", return_value=Path("/guard")), \
                 mock.patch.object(HEADLESS.os, "getpid", return_value=4242), \
                 mock.patch.object(HEADLESS.subprocess, "Popen", return_value=process) as popen:
                result = HEADLESS.start_crash_guard(root, "DirectFix75km")
            self.assertIs(result, process)
            args, kwargs = popen.call_args
            self.assertIn("--parent-pid", args[0])
            self.assertEqual(args[0][args[0].index("--parent-pid") + 1], "4242")
            self.assertTrue(kwargs["start_new_session"])

    def test_preentry_warp_is_process_group_independent_and_tracks_parent(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            process = mock.Mock()
            with mock.patch.object(HEADLESS, "guard_python", return_value=Path("/python")), \
                 mock.patch.object(HEADLESS, "guard_script", return_value=Path("/guard")), \
                 mock.patch.object(HEADLESS.os, "getpid", return_value=4242), \
                 mock.patch.object(HEADLESS.subprocess, "Popen", return_value=process) as popen:
                result = HEADLESS.start_preentry_physics_warp(root, 4, 72_000.0)
            self.assertIs(result, process)
            args, kwargs = popen.call_args
            self.assertIn("--parent-pid", args[0])
            self.assertEqual(args[0][args[0].index("--parent-pid") + 1], "4242")
            self.assertTrue(kwargs["start_new_session"])

    def test_preentry_warp_parent_loss_zeroes_warp_and_pauses(self) -> None:
        class FakeConnection:
            def __init__(self) -> None:
                self.krpc = types.SimpleNamespace(paused=False)
                self.space_center = types.SimpleNamespace(
                    ut=500.0,
                    rails_warp_factor=2,
                    physics_warp_factor=3,
                )
                self.closed = False

            def close(self) -> None:
                self.closed = True

        connection = FakeConnection()
        guard = load_guard_module(lambda **_kwargs: connection)
        events: list[dict] = []
        with mock.patch.object(guard, "process_alive", return_value=False), \
             mock.patch.object(guard, "emit", side_effect=events.append):
            status = guard.preentry_warp(4, 72_000.0, 0.05, parent_pid=4242)
        self.assertEqual(status, 0)
        self.assertTrue(connection.krpc.paused)
        self.assertEqual(connection.space_center.rails_warp_factor, 0)
        self.assertEqual(connection.space_center.physics_warp_factor, 0)
        self.assertTrue(connection.closed)
        self.assertEqual(events[-1]["event"], "preentry-warp-parent-exit")

    def test_preentry_warp_hard_disables_at_40km(self) -> None:
        class FakeVessel:
            def flight(self):
                return types.SimpleNamespace(mean_altitude=39_999.0)

        class FakeConnection:
            def __init__(self) -> None:
                self.krpc = types.SimpleNamespace(paused=False)
                self.space_center = types.SimpleNamespace(
                    active_vessel=FakeVessel(),
                    ut=500.0,
                    rails_warp_factor=2,
                    physics_warp_factor=1,
                )
                self.closed = False

            def close(self) -> None:
                self.closed = True

        connection = FakeConnection()
        guard = load_guard_module(lambda **_kwargs: connection)
        events: list[dict] = []
        with mock.patch.object(guard, "emit", side_effect=events.append):
            status = guard.preentry_warp(4, 72_000.0, 0.05)
        self.assertEqual(status, 0)
        self.assertEqual(connection.space_center.rails_warp_factor, 0)
        self.assertEqual(connection.space_center.physics_warp_factor, 0)
        self.assertTrue(connection.closed)
        cutoff = [event for event in events if event.get("reason") == "below-40km-hard-cutoff"]
        self.assertEqual(len(cutoff), 1)
        self.assertEqual(cutoff[0]["cutoffAltitude"], 40_000.0)

    def test_process_alive_treats_unreaped_zombie_as_dead(self) -> None:
        guard = load_guard_module(lambda **_kwargs: None)
        completed = types.SimpleNamespace(returncode=0, stdout="Z+\n")
        with mock.patch.object(guard.os, "kill", return_value=None), \
             mock.patch.object(guard.subprocess, "run", return_value=completed):
            self.assertFalse(guard.process_alive(4242))

    def test_process_alive_accepts_non_zombie_state(self) -> None:
        guard = load_guard_module(lambda **_kwargs: None)
        completed = types.SimpleNamespace(returncode=0, stdout="S+\n")
        with mock.patch.object(guard.os, "kill", return_value=None), \
             mock.patch.object(guard.subprocess, "run", return_value=completed):
            self.assertTrue(guard.process_alive(4242))

    def test_backend_watch_is_independent_and_carries_exact_identity(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            backend = root / "landing_backend"
            process = mock.Mock()
            with mock.patch.object(HEADLESS, "guard_python", return_value=Path("/python")), \
                 mock.patch.object(HEADLESS, "guard_script", return_value=Path("/guard")), \
                 mock.patch.object(HEADLESS.os, "getpid", return_value=4242), \
                 mock.patch.object(HEADLESS.subprocess, "Popen", return_value=process) as popen:
                result = HEADLESS.start_backend_watch(root, 9001, backend)
            self.assertIs(result, process)
            args, kwargs = popen.call_args
            self.assertEqual(args[0][2], "backend-watch")
            self.assertEqual(args[0][args[0].index("--parent-pid") + 1], "4242")
            self.assertEqual(args[0][args[0].index("--backend-pid") + 1], "9001")
            self.assertEqual(Path(args[0][args[0].index("--backend-executable") + 1]), backend.resolve())
            self.assertTrue(kwargs["start_new_session"])

    def test_backend_watch_parent_loss_reaps_verified_private_group(self) -> None:
        guard = load_guard_module(lambda **_kwargs: None)
        events: list[dict] = []
        with mock.patch.object(guard, "process_alive", side_effect=[False, True, True, False, False]), \
             mock.patch.object(guard.os, "getpgid", return_value=9001), \
             mock.patch.object(guard, "backend_process_matches", return_value=True), \
             mock.patch.object(guard.os, "killpg") as killpg, \
             mock.patch.object(guard.time, "sleep", return_value=None), \
             mock.patch.object(guard, "emit", side_effect=events.append):
            status = guard.backend_watch(0.02, 4242, 9001, "/tmp/landing_backend")
        self.assertEqual(status, 0)
        killpg.assert_called_once_with(9001, guard.signal.SIGTERM)
        self.assertEqual(events[-1]["event"], "backend-watch-reaped")

    def test_backend_watch_refuses_wrong_process_group(self) -> None:
        guard = load_guard_module(lambda **_kwargs: None)
        events: list[dict] = []
        with mock.patch.object(guard, "process_alive", side_effect=[False, True]), \
             mock.patch.object(guard.os, "getpgid", return_value=7777), \
             mock.patch.object(guard.os, "killpg") as killpg, \
             mock.patch.object(guard, "emit", side_effect=events.append):
            status = guard.backend_watch(0.02, 4242, 9001, "/tmp/landing_backend")
        self.assertEqual(status, 2)
        killpg.assert_not_called()
        self.assertEqual(events[-1]["event"], "backend-watch-refused")

    def test_backend_watch_refuses_executable_mismatch(self) -> None:
        guard = load_guard_module(lambda **_kwargs: None)
        with mock.patch.object(guard, "process_alive", side_effect=[False, True]), \
             mock.patch.object(guard.os, "getpgid", return_value=9001), \
             mock.patch.object(guard, "backend_process_matches", return_value=False), \
             mock.patch.object(guard.os, "killpg") as killpg:
            status = guard.backend_watch(0.02, 4242, 9001, "/tmp/landing_backend")
        self.assertEqual(status, 2)
        killpg.assert_not_called()

    def test_startup_hold_is_process_group_independent_and_tracks_parent(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            process = mock.Mock()
            with mock.patch.object(HEADLESS, "guard_python", return_value=Path("/python")), \
                 mock.patch.object(HEADLESS, "guard_script", return_value=Path("/guard")), \
                 mock.patch.object(HEADLESS.os, "getpid", return_value=4242), \
                 mock.patch.object(HEADLESS.subprocess, "Popen", return_value=process) as popen:
                result = HEADLESS.start_startup_hold(root)
            self.assertIs(result, process)
            args, kwargs = popen.call_args
            self.assertIn("--parent-pid", args[0])
            self.assertEqual(args[0][args[0].index("--parent-pid") + 1], "4242")
            self.assertTrue(kwargs["start_new_session"])

    def test_parent_watch_is_process_group_independent_and_tracks_parent(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            process = mock.Mock()
            with mock.patch.object(HEADLESS, "guard_python", return_value=Path("/python")), \
                 mock.patch.object(HEADLESS, "guard_script", return_value=Path("/guard")), \
                 mock.patch.object(HEADLESS.os, "getpid", return_value=4242), \
                 mock.patch.object(HEADLESS.subprocess, "Popen", return_value=process) as popen:
                result = HEADLESS.start_parent_watch(root)
            self.assertIs(result, process)
            args, kwargs = popen.call_args
            self.assertEqual(args[0][2], "parent-watch")
            self.assertEqual(args[0][args[0].index("--parent-pid") + 1], "4242")
            self.assertTrue(kwargs["start_new_session"])

    def test_parent_watch_is_passive_until_parent_loss_then_freezes(self) -> None:
        class FakeConnection:
            def __init__(self) -> None:
                self.krpc = types.SimpleNamespace(paused=False)
                self.space_center = types.SimpleNamespace(ut=222.0)
                self.closed = False

            def close(self) -> None:
                self.closed = True

        connection = FakeConnection()
        guard = load_guard_module(lambda **_kwargs: connection)
        events: list[dict] = []
        with mock.patch.object(guard, "process_alive", side_effect=[True, False]), \
             mock.patch.object(guard.time, "sleep", return_value=None), \
             mock.patch.object(guard, "emit", side_effect=events.append):
            status = guard.parent_watch(0.02, parent_pid=4242)
        self.assertEqual(status, 0)
        self.assertTrue(connection.krpc.paused)
        self.assertTrue(connection.closed)
        self.assertEqual(
            [event["event"] for event in events],
            ["parent-watch-armed", "parent-watch-parent-exit", "parent-watch-frozen"],
        )

    def test_startup_holder_retains_pause_when_parent_disappears(self) -> None:
        class FakeVessel:
            def __init__(self) -> None:
                self.name = "STS-N"
                self.auto_pilot = types.SimpleNamespace(disengage=mock.Mock())
                self.control = types.SimpleNamespace(sas=True)

            def flight(self):
                return types.SimpleNamespace(mean_altitude=75_000.0, pitch=-1.0, roll=0.0)

        class FakeConnection:
            def __init__(self) -> None:
                self.krpc = types.SimpleNamespace(paused=False)
                self.space_center = types.SimpleNamespace(
                    ut=66901.0,
                    active_vessel=FakeVessel(),
                    rails_warp_factor=0,
                    physics_warp_factor=0,
                )
                self.closed = False

            def close(self) -> None:
                self.closed = True

        with tempfile.TemporaryDirectory() as tmp:
            handoff = Path(tmp) / "handoff"
            handoff.write_text("startup", encoding="utf-8")
            connection = FakeConnection()
            guard = load_guard_module(lambda **_kwargs: connection)
            events: list[dict] = []
            with mock.patch.dict(os.environ, {"KSP_LANDER_CONTROL_HANDOFF": str(handoff)}, clear=False), \
                 mock.patch.object(guard, "process_alive", return_value=False), \
                 mock.patch.object(guard, "emit", side_effect=events.append):
                status = guard.startup_hold(0.02, parent_pid=4242)
        self.assertEqual(status, 0)
        self.assertTrue(connection.krpc.paused)
        self.assertTrue(connection.closed)
        self.assertTrue(any(event.get("event") == "startup-hold-parent-exit" for event in events))
        self.assertFalse(any(event.get("event") == "startup-hold-released" and not connection.krpc.paused for event in events))

    def test_startup_holder_no_active_vessel_retains_pause(self) -> None:
        class FakeConnection:
            def __init__(self) -> None:
                self.krpc = types.SimpleNamespace(paused=False)
                self.space_center = types.SimpleNamespace(active_vessel=None)
                self.closed = False

            def close(self) -> None:
                self.closed = True

        with tempfile.TemporaryDirectory() as tmp:
            handoff = Path(tmp) / "handoff"
            handoff.write_text("startup", encoding="utf-8")
            connection = FakeConnection()
            guard = load_guard_module(lambda **_kwargs: connection)
            events: list[dict] = []
            with mock.patch.dict(os.environ, {"KSP_LANDER_CONTROL_HANDOFF": str(handoff)}, clear=False), \
                 mock.patch.object(guard, "emit", side_effect=events.append):
                status = guard.startup_hold(0.02, parent_pid=4242)
        self.assertEqual(status, 2)
        self.assertTrue(connection.krpc.paused)
        self.assertTrue(connection.closed)
        self.assertEqual(events[-1]["event"], "startup-hold-failed")
        self.assertTrue(events[-1]["paused"])


    def test_recovery_freeze_reconnects_zeroes_warp_and_pauses(self) -> None:
        class FakeConnection:
            def __init__(self) -> None:
                self.krpc = types.SimpleNamespace(paused=False)
                self.space_center = types.SimpleNamespace(ut=444.0, rails_warp_factor=3, physics_warp_factor=2)
                self.closed = False

            def close(self) -> None:
                self.closed = True

        connection = FakeConnection()
        guard = load_guard_module(lambda **_kwargs: connection)
        events: list[dict] = []
        with mock.patch.object(guard, "emit", side_effect=events.append):
            frozen = guard.freeze_recovered_scene(interval=0.01, timeout=1.0)
        self.assertTrue(frozen)
        self.assertTrue(connection.krpc.paused)
        self.assertEqual(connection.space_center.rails_warp_factor, 0)
        self.assertEqual(connection.space_center.physics_warp_factor, 0)
        self.assertTrue(connection.closed)
        self.assertEqual(events[-1]["event"], "guard-recovery-frozen")

    def test_recovery_load_transport_error_still_runs_freeze_reconnect(self) -> None:
        class FakeSpaceCenter:
            def load(self, _name: str) -> None:
                raise RuntimeError("scene transition")

        guard = load_guard_module(lambda **_kwargs: None)
        events: list[dict] = []
        with mock.patch.object(guard, "freeze_recovered_scene", return_value=True) as freeze, \
             mock.patch.object(guard, "emit", side_effect=events.append):
            frozen = guard.request_recovery_and_freeze(FakeSpaceCenter(), "DirectFix75km", 0.05)
        self.assertTrue(frozen)
        freeze.assert_called_once_with(interval=0.05, timeout=30.0)
        self.assertEqual(events[-1]["event"], "guard-recovery-load-transition")

    def test_parent_loss_freezes_ksp_before_vessel_access(self) -> None:
        class FakeConnection:
            def __init__(self) -> None:
                self.krpc = types.SimpleNamespace(paused=False)
                self.space_center = types.SimpleNamespace(ut=123.5)
                self.closed = False

            def close(self) -> None:
                self.closed = True

        connection = FakeConnection()
        guard = load_guard_module(lambda **_kwargs: connection)
        events: list[dict] = []
        with mock.patch.object(guard, "process_alive", return_value=False), \
             mock.patch.object(guard, "emit", side_effect=events.append):
            status = guard.monitor(0.05, "DirectFix75km", parent_pid=4242)
        self.assertEqual(status, 0)
        self.assertTrue(connection.krpc.paused)
        self.assertTrue(connection.closed)
        self.assertEqual(events[-1]["event"], "guard-parent-exit")
        self.assertEqual(events[-1]["parentPid"], 4242)
        self.assertTrue(events[-1]["paused"])

    def test_failed_state_is_frozen_after_restore(self) -> None:
        root = Path("/tmp/Code/C/KSPShuttleLander")
        calls: list[str] = []

        def restore(_root, save_name):
            calls.append(f"restore:{save_name}")
            return {"ut": 66901.3}

        def pause(_root, paused):
            calls.append(f"pause:{paused}")

        with mock.patch.object(HEADLESS, "restore_quicksave", side_effect=restore), \
             mock.patch.object(HEADLESS, "set_ksp_paused", side_effect=pause):
            state = HEADLESS.secure_failed_test_state(root, "DirectFix75km", restore_save=True)
        self.assertEqual(state, {"ut": 66901.3})
        self.assertEqual(calls, ["restore:DirectFix75km", "pause:True"])

    def test_no_quickload_failure_is_still_frozen(self) -> None:
        root = Path("/tmp/Code/C/KSPShuttleLander")
        with mock.patch.object(HEADLESS, "restore_quicksave") as restore, \
             mock.patch.object(HEADLESS, "set_ksp_paused") as pause:
            state = HEADLESS.secure_failed_test_state(root, "DirectFix75km", restore_save=False)
        self.assertIsNone(state)
        restore.assert_not_called()
        pause.assert_called_once_with(root, True)

    def test_failed_restore_still_freezes_current_scene(self) -> None:
        root = Path("/tmp/Code/C/KSPShuttleLander")
        with mock.patch.object(HEADLESS, "restore_quicksave", side_effect=RuntimeError("load failed")), \
             mock.patch.object(HEADLESS, "set_ksp_paused") as pause:
            with self.assertRaisesRegex(RuntimeError, "load failed"):
                HEADLESS.secure_failed_test_state(root, "DirectFix75km", restore_save=True)
        pause.assert_called_once_with(root, True)


class KspGuardConnectionTests(unittest.TestCase):
    def test_stalled_krpc_handshake_is_bounded(self) -> None:
        release = threading.Event()

        def blocked_connect(**_kwargs):
            release.wait(1.0)
            return object()

        guard = load_guard_module(blocked_connect)
        started = time.monotonic()
        try:
            with self.assertRaisesRegex(TimeoutError, "KSP Lander Test Probe"):
                guard.connect("KSP Lander Test Probe", timeout=0.02)
            self.assertLess(time.monotonic() - started, 0.5)
        finally:
            release.set()

    def test_successful_handshake_returns_client(self) -> None:
        client = object()
        guard = load_guard_module(lambda **_kwargs: client)
        self.assertIs(guard.connect("test", timeout=0.1), client)


class RestoreQuicksaveFailureTests(unittest.TestCase):
    @staticmethod
    def state(ut: float) -> dict[str, object]:
        return {"ut": ut, "railsWarpFactor": 0, "physicsWarpFactor": 0, "vessel": "STS-N", "meanAltitude": 75_000.0}

    def test_exited_loader_without_settled_event_fails_immediately(self) -> None:
        root = Path("/tmp/Code/C/KSPShuttleLander")
        loader = mock.Mock()
        loader.returncode = 1
        loader.poll.return_value = 1
        loader.stdout = io.StringIO(
            '{"event":"guard-error","error":"TimeoutError: kRPC connection handshake timed out"}\n'
        )
        current = self.state(100.0)
        with mock.patch.object(HEADLESS, "guard_python", return_value=Path("/python")), \
             mock.patch.object(HEADLESS, "guard_script", return_value=Path("/guard")), \
             mock.patch.object(HEADLESS.subprocess, "Popen", return_value=loader), \
             mock.patch.object(HEADLESS, "probe_ksp", side_effect=[current, current]):
            with self.assertRaisesRegex(RuntimeError, "load helper exited") as context:
                HEADLESS.restore_quicksave(root, settle_timeout=120.0)
        self.assertIn("guard-error", str(context.exception))
        loader.send_signal.assert_not_called()

    def test_exited_loader_with_settled_event_is_accepted(self) -> None:
        root = Path("/tmp/Code/C/KSPShuttleLander")
        loader = mock.Mock()
        loader.returncode = 0
        loader.poll.return_value = 0
        loader.stdout = io.StringIO(
            '{"event":"load-save-settled","name":"DirectFix75km","ut":50.0}\n'
        )
        before = self.state(100.0)
        after = self.state(50.0)
        with mock.patch.object(HEADLESS, "guard_python", return_value=Path("/python")), \
             mock.patch.object(HEADLESS, "guard_script", return_value=Path("/guard")), \
             mock.patch.object(HEADLESS.subprocess, "Popen", return_value=loader), \
             mock.patch.object(HEADLESS, "probe_ksp", side_effect=[before, before, after]):
            restored = HEADLESS.restore_quicksave(root, save_name="DirectFix75km", settle_timeout=120.0)
        self.assertEqual(restored["ut"], 50.0)


class EntryDebtCampaignGuardTests(unittest.TestCase):
    @staticmethod
    def sample(range_m: float, altitude_m: float, vertical_speed_mps: float = -60.0) -> dict[str, float]:
        return {
            "rangeToSite": range_m,
            "meanAltitude": altitude_m,
            "verticalSpeed": vertical_speed_mps,
        }

    def test_400_km_records_exact_crossing_without_rejecting(self) -> None:
        guard = HEADLESS.EntryDebtCampaignGuard()
        guard.update(self.sample(410_000.0, 49_000.0))
        decisions = guard.update(self.sample(399_900.0, 48_100.0))
        self.assertEqual(len(decisions), 1)
        self.assertFalse(decisions[0].reject)
        self.assertAlmostEqual(decisions[0].observation.range_m, 400_000.0)
        self.assertAlmostEqual(guard.debt_400_m or 0.0, 13_608.910891089108)

    def test_300_km_above_10_km_and_not_improving_rejects(self) -> None:
        guard = HEADLESS.EntryDebtCampaignGuard()
        guard.update(self.sample(410_000.0, 49_000.0))
        guard.update(self.sample(399_900.0, 48_100.0))
        decisions = guard.update(self.sample(299_950.0, 44_350.0))
        at_300 = [d for d in decisions if d.observation.checkpoint_range_m == 300_000.0]
        self.assertEqual(len(at_300), 1)
        self.assertTrue(at_300[0].reject)

    def test_300_km_above_10_km_but_improving_remains_recoverable(self) -> None:
        guard = HEADLESS.EntryDebtCampaignGuard()
        guard.update(self.sample(410_000.0, 49_000.0))
        guard.update(self.sample(399_900.0, 48_500.0))
        decisions = guard.update(self.sample(299_900.0, 40_000.0))
        at_300 = [d for d in decisions if d.observation.checkpoint_range_m == 300_000.0]
        self.assertEqual(len(at_300), 1)
        self.assertFalse(at_300[0].reject)

    def test_200_km_above_7_km_rejects(self) -> None:
        guard = HEADLESS.EntryDebtCampaignGuard()
        guard.update(self.sample(210_000.0, 31_000.0))
        self.assertTrue(guard.update(self.sample(199_900.0, 31_000.0))[0].reject)

    def test_100_km_requires_altitude_and_descent(self) -> None:
        high = HEADLESS.EntryDebtCampaignGuard()
        high.update(self.sample(110_000.0, 22_000.0))
        self.assertTrue(high.update(self.sample(99_900.0, 21_100.0, -50.0))[0].reject)

        climbing = HEADLESS.EntryDebtCampaignGuard()
        climbing.update(self.sample(110_000.0, 20_000.0, 0.05))
        self.assertTrue(climbing.update(self.sample(99_900.0, 20_000.0, 0.1))[0].reject)

        good = HEADLESS.EntryDebtCampaignGuard()
        good.update(self.sample(110_000.0, 20_000.0))
        self.assertFalse(good.update(self.sample(99_900.0, 20_000.0, -10.0))[0].reject)

    def test_checkpoint_fires_only_once(self) -> None:
        guard = HEADLESS.EntryDebtCampaignGuard()
        guard.update(self.sample(410_000.0, 49_000.0))
        first = guard.update(self.sample(399_900.0, 48_000.0))
        second = guard.update(self.sample(399_800.0, 47_900.0))
        self.assertEqual(len(first), 1)
        self.assertEqual(second, [])

    def test_skipped_checkpoints_interpolate_exact_crossing_state(self) -> None:
        guard = HEADLESS.EntryDebtCampaignGuard()
        guard.update(self.sample(410_000.0, 49_000.0, -50.0))
        decisions = guard.update(self.sample(290_000.0, 40_000.0, -80.0))
        self.assertEqual([d.observation.checkpoint_range_m for d in decisions], [400_000.0, 300_000.0])
        at_400, at_300 = decisions
        self.assertAlmostEqual(at_400.observation.altitude_m, 48_250.0)
        self.assertAlmostEqual(at_300.observation.altitude_m, 40_750.0)
        self.assertAlmostEqual(at_400.observation.vertical_speed_mps, -52.5)
        self.assertAlmostEqual(at_300.observation.vertical_speed_mps, -77.5)
        self.assertFalse(at_400.reject)
        self.assertFalse(at_300.reject)

    def test_bd75_live_trace_would_reject_at_300_km(self) -> None:
        guard = HEADLESS.EntryDebtCampaignGuard()
        # Runtime/Headless/bd75-live1-console.log brackets the exact gates:
        # 400.4 km / 48.1 km -> 394.4 km / 47.9 km, then
        # 304.6 km / 44.1 km -> 298.8 km / 43.8 km.
        guard.update(self.sample(400_400.0, 48_100.0, -60.0))
        at_400 = guard.update(self.sample(394_400.0, 47_900.0, -62.0))[0]
        self.assertFalse(at_400.reject)
        self.assertAlmostEqual(at_400.observation.altitude_m, 48_086.666666666664)
        self.assertAlmostEqual(at_400.observation.debt_m, 13_586.666666666664)

        guard.update(self.sample(304_600.0, 44_100.0, -80.0))
        at_300 = guard.update(self.sample(298_800.0, 43_800.0, -82.0))[0]
        self.assertTrue(at_300.reject)
        self.assertAlmostEqual(at_300.observation.altitude_m, 43_862.06896551724)
        self.assertGreater(at_300.observation.debt_m, at_400.observation.debt_m)

    def test_missing_or_nan_telemetry_is_ignored(self) -> None:
        guard = HEADLESS.EntryDebtCampaignGuard()
        self.assertEqual(guard.update({}), [])
        self.assertEqual(guard.update(self.sample(math.nan, 48_000.0)), [])
        self.assertEqual(guard.update(self.sample(400_000.0, math.nan)), [])


class ReentryHandoffAppliedTests(unittest.TestCase):
    def test_handoff_waits_for_physics_to_apply_inherited_throttle(self) -> None:
        guard = load_guard_module(lambda **kwargs: None)
        class Control:
            sas = True
            requested = 0.45
            applied = 0.45
            @property
            def throttle(self):
                return self.applied
            @throttle.setter
            def throttle(self, value):
                self.requested = value
        control = Control()
        vessel = types.SimpleNamespace(name="STS-N", control=control,
            auto_pilot=types.SimpleNamespace(disengage=lambda: None),
            thrust=0.0, available_thrust=820000.0,
            flight=lambda: types.SimpleNamespace(mean_altitude=7900.0))
        sc = types.SimpleNamespace(active_vessel=vessel, ut=100.0,
            rails_warp_factor=0, physics_warp_factor=0)
        conn = types.SimpleNamespace(space_center=sc, krpc=types.SimpleNamespace(paused=False), close=mock.Mock())
        def physics_tick(_):
            sc.ut += 0.02
            control.applied = control.requested
        with mock.patch.object(guard, "connect", return_value=conn), \
             mock.patch.object(guard.time, "sleep", side_effect=physics_tick), \
             mock.patch.object(guard, "emit") as emit:
            result = guard.prepare_reentry()
        self.assertEqual(result, 0)
        self.assertEqual(control.applied, 0.0)
        self.assertGreaterEqual(sc.ut - 100.0, 0.06)
        self.assertEqual(emit.call_args.args[0]["throttle"], 0.0)
        self.assertEqual(emit.call_args.args[0]["actualThrust"], 0.0)
        conn.close.assert_called_once()


class ReentryRestoreSettleTests(unittest.TestCase):
    def test_unpaused_reentry_restore_waits_for_stable_physics(self) -> None:
        root = Path("/tmp/Code/C/KSPShuttleLander")
        initial = {"paused": False, "mass": 20000.0}
        settled = {"paused": False, "mass": 40278.16}
        calls = []
        with mock.patch.object(HEADLESS, "prepare_reentry_direct_control", side_effect=lambda root: calls.append("neutralize")) as prepare, \
             mock.patch.object(HEADLESS, "wait_for_orbital_state_settle", side_effect=lambda root, state, **_kwargs: calls.append("settle") or settled) as wait:
            result = HEADLESS.settle_restored_reentry_state(root, initial)
        self.assertIs(result, settled)
        prepare.assert_called_once_with(root)
        wait.assert_called_once_with(root, initial, timeout=45.0)
        self.assertEqual(calls, ["neutralize", "settle"])

    def test_settle_rejects_stable_partial_vessel_mass_below_dry_mass(self) -> None:
        root = Path("/tmp/Code/C/KSPShuttleLander")
        initial = {"paused": False, "ut": 100.0, "vessel": "STS-N"}
        transient = {
            "ready": True, "paused": False, "ut": 100.9, "vessel": "STS-N",
            "mass": 20000.0, "dryMass": 30053.0, "availableThrust": 820000.0,
            "railsWarpFactor": 0, "physicsWarpFactor": 0,
        }
        settled_a = dict(transient, ut=101.0, mass=40278.16)
        settled_b = dict(transient, ut=101.1, mass=40278.18)
        with mock.patch.object(HEADLESS, "probe_ksp", side_effect=[transient, transient, settled_a, settled_b]), \
             mock.patch.object(HEADLESS.time, "sleep", return_value=None):
            result = HEADLESS.wait_for_orbital_state_settle(
                root, initial, timeout=1.0, minimum_ut_advance=0.8, stable_samples=2
            )
        self.assertEqual(result["mass"], settled_b["mass"])
        self.assertGreaterEqual(result["mass"], result["dryMass"])

    def test_settle_rejects_stable_partial_vessel_during_unpack_warmup(self) -> None:
        root = Path("/tmp/Code/C/KSPShuttleLander")
        initial = {"paused": False, "ut": 100.0, "vessel": "STS-N"}
        transient_a = {
            "ready": True, "paused": False, "ut": 100.9, "vessel": "STS-N",
            "mass": 20000.0, "dryMass": 17728.3, "availableThrust": 820000.0,
            "railsWarpFactor": 0, "physicsWarpFactor": 0,
        }
        transient_b = dict(transient_a, ut=101.2)
        transient_c = dict(transient_a, ut=101.4)
        settled_a = dict(transient_a, ut=101.6, mass=40278.16, dryMass=30053.0)
        settled_b = dict(transient_a, ut=101.8, mass=40278.18, dryMass=30053.0)
        with (
            mock.patch.object(HEADLESS, "probe_ksp", side_effect=[transient_a, transient_b, transient_c, settled_a, settled_b]),
            mock.patch.object(HEADLESS.time, "sleep", return_value=None),
        ):
            result = HEADLESS.wait_for_orbital_state_settle(
                root, initial, timeout=1.0, minimum_ut_advance=1.5, stable_samples=2
            )
        self.assertEqual(result["mass"], settled_b["mass"])
        self.assertEqual(result["dryMass"], settled_b["dryMass"])

    def test_already_paused_reentry_restore_is_not_advanced(self) -> None:
        root = Path("/tmp/Code/C/KSPShuttleLander")
        initial = {"paused": True, "mass": 40278.16}
        with mock.patch.object(HEADLESS, "wait_for_orbital_state_settle") as wait:
            result = HEADLESS.settle_restored_reentry_state(root, initial)
        self.assertIs(result, initial)
        wait.assert_not_called()
class CrashGuardMotionFrameTests(unittest.TestCase):
    def test_monitor_detects_descent_hidden_by_default_vessel_frame(self) -> None:
        guard = load_guard_module(lambda **kwargs: None)
        body_frame = object()
        vessel_frame_flight = types.SimpleNamespace(surface_altitude=100.0, vertical_speed=0.0, g_force=1.0)
        body_frame_flight = types.SimpleNamespace(surface_altitude=100.0, vertical_speed=-75.0, g_force=1.0)
        vessel = types.SimpleNamespace(
            name="STS-N", situation="flying", control=types.SimpleNamespace(throttle=0.0),
            auto_pilot=mock.Mock(), orbit=types.SimpleNamespace(body=types.SimpleNamespace(reference_frame=body_frame)),
            flight=mock.Mock(side_effect=lambda frame=None: body_frame_flight if frame is body_frame else vessel_frame_flight),
        )
        connection = types.SimpleNamespace(space_center=types.SimpleNamespace(active_vessel=vessel, ut=123.0), close=mock.Mock())
        self.assertIsNone(guard.impact_reason(vessel, vessel_frame_flight))
        with mock.patch.object(guard, "connect", return_value=connection), \
             mock.patch.object(guard, "freeze_for_parent_loss", return_value=False), \
             mock.patch.object(guard, "request_recovery_and_freeze", return_value=True) as recover, \
             mock.patch.object(guard, "emit"):
            self.assertEqual(guard.monitor(0.05, "DirectFix75km", parent_pid=4242), 0)
        vessel.flight.assert_called_once_with(body_frame)
        recover.assert_called_once_with(connection.space_center, "DirectFix75km", 0.05)

    def test_planet_frame_does_not_reject_controlled_landed_rollout(self) -> None:
        guard = load_guard_module(lambda **kwargs: None)
        vessel = types.SimpleNamespace(situation="landed")
        flight = types.SimpleNamespace(surface_altitude=3.0, vertical_speed=-1.0, g_force=1.5)
        self.assertIsNone(guard.impact_reason(vessel, flight))


class DescentCheckpointPlanTests(unittest.TestCase):
    def test_plan_normalizes_to_descending_altitude(self) -> None:
        plan = HEADLESS.descent_checkpoint_plan([
            ["30000", "STS-N-30km"],
            ["72000", "STS-N-72km"],
            ["50000", "STS-N-50km"],
            ["40000", "STS-N-40km"],
        ])
        self.assertEqual([checkpoint.altitude for checkpoint in plan], [72000.0, 50000.0, 40000.0, 30000.0])
        self.assertEqual([checkpoint.name for checkpoint in plan], [
            "STS-N-72km", "STS-N-50km", "STS-N-40km", "STS-N-30km"
        ])

    def test_plan_rejects_invalid_or_ambiguous_entries(self) -> None:
        for raw in (
            [["nan", "bad"]],
            [["0", "bad"]],
            [["50000", ""]],
            [["50000", "same"], ["40000", "same"]],
            [["50000", "a"], ["50000", "b"]],
        ):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                HEADLESS.descent_checkpoint_plan(raw)

    def test_tracker_saves_one_continuous_checkpoint_chain(self) -> None:
        plan = HEADLESS.descent_checkpoint_plan([
            ["72000", "STS-N-72km"],
            ["50000", "STS-N-50km"],
            ["40000", "STS-N-40km"],
            ["30000", "STS-N-30km"],
        ])
        tracker = HEADLESS.DescentCheckpointTracker(plan)
        self.assertEqual(tracker.update(86000.0, -10.0), ())
        self.assertEqual([c.name for c in tracker.update(71950.0, -100.0)], ["STS-N-72km"])
        self.assertEqual(tracker.update(60000.0, -100.0), ())
        self.assertEqual([c.name for c in tracker.update(49950.0, -100.0)], ["STS-N-50km"])
        self.assertEqual([c.name for c in tracker.update(39950.0, -100.0)], ["STS-N-40km"])
        self.assertEqual([c.name for c in tracker.update(29950.0, -100.0)], ["STS-N-30km"])
        self.assertEqual(tracker.update(25000.0, -100.0), ())

    def test_tracker_does_not_backfill_a_checkpoint_started_below_threshold(self) -> None:
        plan = HEADLESS.descent_checkpoint_plan([["72000", "STS-N-72km"]])
        tracker = HEADLESS.DescentCheckpointTracker(plan)
        self.assertEqual(tracker.update(65000.0, -100.0), ())
        self.assertEqual(tracker.update(64000.0, -100.0), ())

    def test_tracker_refuses_to_label_multiple_thresholds_at_one_state(self) -> None:
        plan = HEADLESS.descent_checkpoint_plan([
            ["72000", "STS-N-72km"],
            ["50000", "STS-N-50km"],
        ])
        tracker = HEADLESS.DescentCheckpointTracker(plan)
        self.assertEqual(tracker.update(80000.0, -100.0), ())
        with self.assertRaisesRegex(RuntimeError, "skipped multiple armed thresholds"):
            tracker.update(49000.0, -100.0)


class LiveUnpoweredPolicyTests(unittest.TestCase):
    def test_control_capable_live_run_requires_explicit_unpowered_policy(self) -> None:
        error = HEADLESS.live_unpowered_policy_error(True, False, {})
        self.assertIsNotNone(error)
        self.assertIn("KSP_LANDER_UNPOWERED_ONLY=1", error or "")

    def test_control_capable_live_run_accepts_explicit_unpowered_policy(self) -> None:
        self.assertIsNone(HEADLESS.live_unpowered_policy_error(
            True, False, {"KSP_LANDER_UNPOWERED_ONLY": "1"}
        ))

    def test_offline_and_connect_only_modes_do_not_require_control_policy(self) -> None:
        self.assertIsNone(HEADLESS.live_unpowered_policy_error(False, False, {}))
        self.assertIsNone(HEADLESS.live_unpowered_policy_error(True, True, {}))

    def test_near_miss_environment_values_fail_closed(self) -> None:
        for value in ("", "0", "true", "yes", " 1"):
            with self.subTest(value=value):
                self.assertIsNotNone(HEADLESS.live_unpowered_policy_error(
                    True, False, {"KSP_LANDER_UNPOWERED_ONLY": value}
                ))


class ReentryDirectControlEvidenceTests(unittest.TestCase):
    @staticmethod
    def completed(payload: dict[str, object], returncode: int = 0):
        return types.SimpleNamespace(
            returncode=returncode,
            stdout=json.dumps(payload) + "\n",
            stderr="",
        )

    @staticmethod
    def ready_payload(**overrides):
        payload = {
            "event": "reentry-direct-control-ready",
            "meanAltitude": 50000.0,
            "sas": False,
            "throttle": 0.0,
            "actualThrust": 0.0,
            "railsWarpFactor": 0,
            "physicsWarpFactor": 0,
        }
        payload.update(overrides)
        return payload

    def run_prepare(self, payload):
        with mock.patch.object(HEADLESS, "guard_python", return_value=Path("/python")), \
             mock.patch.object(HEADLESS, "guard_script", return_value=Path("/guard")), \
             mock.patch.object(HEADLESS.subprocess, "run", return_value=self.completed(payload)):
            return HEADLESS.prepare_reentry_direct_control(Path("/tmp/project"))

    def test_safe_handoff_requires_explicit_zero_throttle_and_warp(self) -> None:
        payload = self.ready_payload()
        self.assertEqual(self.run_prepare(payload), payload)

    def test_nonzero_throttle_is_rejected(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "zero throttle"):
            self.run_prepare(self.ready_payload(throttle=0.01))

    def test_missing_throttle_is_rejected(self) -> None:
        payload = self.ready_payload()
        payload.pop("throttle")
        with self.assertRaisesRegex(RuntimeError, "zero throttle"):
            self.run_prepare(payload)

    def test_actual_thrust_must_be_explicitly_zero(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "zero actual thrust"):
            self.run_prepare(self.ready_payload(actualThrust=25.0))
        payload = self.ready_payload()
        payload.pop("actualThrust")
        with self.assertRaisesRegex(RuntimeError, "zero actual thrust"):
            self.run_prepare(payload)

    def test_nonzero_or_missing_warp_is_rejected(self) -> None:
        cases = [
            ({"physicsWarpFactor": 1}, "physics warp"),
            ({"railsWarpFactor": 1}, "rails warp"),
        ]
        for overrides, expected in cases:
            with self.subTest(overrides=overrides), self.assertRaisesRegex(RuntimeError, expected):
                self.run_prepare(self.ready_payload(**overrides))
        payload = self.ready_payload()
        payload.pop("physicsWarpFactor")
        with self.assertRaisesRegex(RuntimeError, "physics warp"):
            self.run_prepare(payload)

    def test_sas_still_fails_closed(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "disable SAS"):
            self.run_prepare(self.ready_payload(sas=True))


class TaemHacCheckpointEvidenceTests(unittest.TestCase):
    CONFIG = {
        "site": {"altitude": 0.0, "runwayHeading": 90.0},
        "guidance": {"finalApproachDistance": 8000.0},
    }

    @staticmethod
    def snapshot(*, phase="TAEM", telemetry=None, guidance=None):
        telemetry_state = {
            "meanAltitude": 16000.0,
            "verticalSpeed": -100.0,
            "runwayAlongTrack": -8000.0,
            "runwayCrossTrack": 0.0,
            "groundTrackHeading": 0.0,
        }
        telemetry_state.update(telemetry or {})
        guidance_state = {
            "terminalPathCommitted": True,
            "terminalCandidateValid": True,
            "candidateKind": 2,
            "terminalCandidateRadius": 12000.0,
            "candidateGeometryDegraded": False,
            "candidateEnergyDegraded": False,
            "candidateShellDegraded": False,
            "candidatePathDegraded": False,
            "candidateControlDegraded": False,
            "candidateRateDegraded": False,
            "candidateEndDegraded": False,
        }
        guidance_state.update(guidance or {})
        return {"phase": phase, "telemetry": telemetry_state, "guidanceState": guidance_state}

    def test_nominal_perpendicular_hac_contract_passes(self) -> None:
        ok, reason = HEADLESS.taem_hac_checkpoint_ready(self.snapshot(), self.CONFIG)
        self.assertTrue(ok)
        self.assertEqual(reason, "contract-passed")

    def test_any_degraded_terminal_candidate_cannot_certify_checkpoint(self) -> None:
        degraded_fields = (
            "candidateGeometryDegraded",
            "candidateEnergyDegraded",
            "candidateShellDegraded",
            "candidatePathDegraded",
            "candidateControlDegraded",
            "candidateRateDegraded",
            "candidateEndDegraded",
        )
        for field in degraded_fields:
            with self.subTest(field=field):
                ok, reason = HEADLESS.taem_hac_checkpoint_ready(
                    self.snapshot(guidance={field: True}), self.CONFIG
                )
                self.assertFalse(ok)
                self.assertEqual(reason, "HAC-degraded")

    def test_malformed_contract_state_fails_closed(self) -> None:
        cases = (
            (self.snapshot(telemetry={"meanAltitude": float("nan")}), "altitude-unavailable"),
            (self.snapshot(telemetry={"verticalSpeed": float("nan")}), "vertical-speed-unavailable"),
            (self.snapshot(guidance={"terminalPathCommitted": "true"}), "HAC-not-committed"),
            (self.snapshot(guidance={"terminalCandidateValid": 1}), "HAC-candidate-invalid"),
            (self.snapshot(guidance={"candidateKind": 2.5}), "candidate-not-HAC"),
            (self.snapshot(guidance={"candidateKind": float("nan")}), "candidate-not-HAC"),
            (self.snapshot(guidance={"candidateShellDegraded": 0}), "HAC-degraded"),
        )
        for snapshot, expected in cases:
            with self.subTest(expected=expected):
                ok, reason = HEADLESS.taem_hac_checkpoint_ready(snapshot, self.CONFIG)
                self.assertFalse(ok)
                self.assertEqual(reason, expected)

    def test_missing_degradation_state_fails_closed(self) -> None:
        snapshot = self.snapshot()
        snapshot["guidanceState"].pop("candidateShellDegraded")
        ok, reason = HEADLESS.taem_hac_checkpoint_ready(snapshot, self.CONFIG)
        self.assertFalse(ok)
        self.assertEqual(
            reason, "HAC-degradation-state-missing:candidateShellDegraded"
        )

    def test_hac_kind_and_radius_are_fail_closed(self) -> None:
        for guidance, expected in (
            ({"candidateKind": 1}, "candidate-not-HAC"),
            ({"terminalCandidateRadius": 5999.0}, "HAC-radius=5999m"),
            ({"terminalCandidateRadius": 20001.0}, "HAC-radius=20001m"),
        ):
            with self.subTest(guidance=guidance):
                ok, reason = HEADLESS.taem_hac_checkpoint_ready(
                    self.snapshot(guidance=guidance), self.CONFIG
                )
                self.assertFalse(ok)
                self.assertEqual(reason, expected)

    def test_strict_station_heading_altitude_and_descent_remain_required(self) -> None:
        cases = (
            ({"runwayAlongTrack": -6900.0}, "station-error=1100m"),
            ({"groundTrackHeading": 45.0}, "runway-offset=45.0deg"),
            ({"meanAltitude": 14999.0}, "altitude=14999m"),
            ({"verticalSpeed": 0.0}, "not-descending"),
        )
        for telemetry, expected in cases:
            with self.subTest(telemetry=telemetry):
                ok, reason = HEADLESS.taem_hac_checkpoint_ready(
                    self.snapshot(telemetry=telemetry), self.CONFIG
                )
                self.assertFalse(ok)
                self.assertEqual(reason, expected)


class LandingCompletionEvidenceTests(unittest.TestCase):
    CONFIG = {
        "site": {
            "runwayLength": 2500.0,
            "runwayWidth": 70.0,
        }
    }

    @staticmethod
    def snapshot(**telemetry):
        base = {
            "surfaceSpeed": 0.8,
            "runwayAlongTrack": 1200.0,
            "runwayCrossTrack": 10.0,
            "vesselSituation": "landed",
            "radarAltitude": 0.2,
            "verticalSpeed": 0.0,
            "gear": True,
        }
        base.update(telemetry)
        return {"phase": "Complete", "telemetry": base}

    def test_landed_stopped_inside_runway_is_verified(self) -> None:
        ok, reason = HEADLESS.landing_completion_evidence(self.snapshot(), self.CONFIG)
        self.assertTrue(ok)
        self.assertIn("landed", reason)

    def test_landed_state_requires_explicit_gear_confirmation(self) -> None:
        for gear in (False, None, 0, 1, "true", "false"):
            with self.subTest(gear=gear):
                ok, reason = HEADLESS.landing_completion_evidence(
                    self.snapshot(gear=gear), self.CONFIG
                )
                self.assertFalse(ok)
                self.assertEqual(reason, "landing-gear-unconfirmed")

    def test_debounced_near_contact_can_verify_before_ksp_reports_landed(self) -> None:
        ok, reason = HEADLESS.landing_completion_evidence(
            self.snapshot(vesselSituation="flying", radarAltitude=0.3, verticalSpeed=-0.2),
            self.CONFIG,
        )
        self.assertTrue(ok)
        self.assertIn("debounced-contact", reason)

    def test_complete_phase_with_rollout_speed_is_rejected(self) -> None:
        ok, reason = HEADLESS.landing_completion_evidence(
            self.snapshot(surfaceSpeed=2.0), self.CONFIG
        )
        self.assertFalse(ok)
        self.assertIn("surface-speed", reason)

    def test_complete_phase_outside_runway_envelope_is_rejected(self) -> None:
        for telemetry in (
            {"runwayAlongTrack": -40.1},
            {"runwayAlongTrack": 2580.0},
            {"runwayCrossTrack": 52.5},
        ):
            with self.subTest(telemetry=telemetry):
                ok, reason = HEADLESS.landing_completion_evidence(
                    self.snapshot(**telemetry), self.CONFIG
                )
                self.assertFalse(ok)
                self.assertIn("outside-runway-envelope", reason)

    def test_flying_state_without_near_contact_is_rejected(self) -> None:
        ok, reason = HEADLESS.landing_completion_evidence(
            self.snapshot(vesselSituation="flying", radarAltitude=2.0), self.CONFIG
        )
        self.assertFalse(ok)
        self.assertIn("ground-contact-unverified", reason)

    def test_missing_runway_dimensions_fail_closed(self) -> None:
        ok, reason = HEADLESS.landing_completion_evidence(self.snapshot(), {"site": {}})
        self.assertFalse(ok)
        self.assertIn("runway-length-missing", reason)

    def test_non_complete_phase_is_not_landing_success(self) -> None:
        snapshot = self.snapshot()
        snapshot["phase"] = "Rollout"
        ok, reason = HEADLESS.landing_completion_evidence(snapshot, self.CONFIG)
        self.assertFalse(ok)
        self.assertIn("phase=Rollout", reason)


if __name__ == "__main__":
    unittest.main()
