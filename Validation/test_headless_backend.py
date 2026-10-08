#!/usr/bin/env python3
"""Offline regressions for live-flight admission and backend acknowledgements."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Tools"))
import headless_flight as flight


class BackendEngagementTests(unittest.TestCase):
    def backend(self):
        backend = flight.BackendProcess.__new__(flight.BackendProcess)
        backend.latest_snapshot = {"phase": "Idle", "automationEngaged": False}
        backend.read_until = Mock(side_effect=AssertionError("unexpected extra read"))
        return backend

    def test_consumes_rejection_published_before_response(self):
        backend = self.backend()
        reason = "Reentry rejected by closed-loop shadow: MM305=false"
        def reply(*args, **kwargs):
            backend.latest_snapshot = {"phase": "Idle", "automationEngaged": False,
                                       "warningMessage": reason}
            return {}
        backend.send = Mock(side_effect=reply)
        with self.assertRaisesRegex(RuntimeError, "Reentry rejected by closed-loop shadow"):
            backend.engage("engageReentry")
        backend.read_until.assert_not_called()

    def test_consumes_success_before_blocking_coast(self):
        backend = self.backend()
        armed = {"phase": "Coast", "automationEngaged": True,
                 "warningMessage": "Guarded recovery execution engaged"}
        def reply(*args, **kwargs):
            backend.latest_snapshot = armed
            return {}
        backend.send = Mock(side_effect=reply)
        self.assertIs(backend.engage("engage"), armed)
        backend.read_until.assert_not_called()

    def test_does_not_use_stale_armed_snapshot(self):
        backend = self.backend()
        backend.latest_snapshot = {"phase": "Coast", "automationEngaged": True}
        backend.send = Mock(return_value={})
        backend.read_until = Mock(return_value={"phase": "Fault", "automationEngaged": False,
                                               "lastError": "control transport failed"})
        with self.assertRaisesRegex(RuntimeError, "control transport failed"):
            backend.engage("engage")
        backend.read_until.assert_called_once()

    def test_waits_when_no_decision_has_been_published(self):
        backend = self.backend()
        backend.send = Mock(return_value={})
        armed = {"phase": "Entry", "automationEngaged": True}
        backend.read_until = Mock(return_value=armed)
        self.assertIs(backend.engage("engageReentry"), armed)

    def test_fault_wins_over_stale_engaged_flag(self):
        backend = self.backend()
        def reply(*args, **kwargs):
            backend.latest_snapshot = {"phase": "Fault", "automationEngaged": True,
                                       "lastError": "link lost"}
            return {}
        backend.send = Mock(side_effect=reply)
        with self.assertRaisesRegex(RuntimeError, "link lost"):
            backend.engage("engage")


class PlanAdmissionTests(unittest.TestCase):
    def test_unqualified_plan_cannot_be_called_validated(self):
        with self.assertRaisesRegex(RuntimeError, "entry-stage"):
            flight.validate_plan_qualification({"executionQualified": False}, False)

    def test_stage_qualified_plan_is_distinct_from_strict_capture(self):
        plan = {"executionQualified": True, "targetCaptureAchieved": False}
        flight.validate_plan_qualification(plan, False)
        with self.assertRaisesRegex(RuntimeError, "strict"):
            flight.validate_plan_qualification(plan, True)

    def test_strict_plan_passes(self):
        flight.validate_plan_qualification(
            {"executionQualified": True, "targetCaptureAchieved": True}, True)

    def test_missing_qualification_fails_closed(self):
        with self.assertRaises(RuntimeError):
            flight.validate_plan_qualification({}, False)


if __name__ == "__main__":
    unittest.main()
