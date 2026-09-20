#!/usr/bin/env python3
"""Offline regression tests for recorded terminal-rejection summaries."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "terminal_rejection_summary", ROOT / "Tools/summarize_terminal_rejections.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class TerminalRejectionSummaryTests(unittest.TestCase):
    def summarize(self, rows):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "planner.jsonl"
            path.write_text("\n" + "\n\n".join(json.dumps(row) for row in rows) + "\n",
                            encoding="utf-8")
            return MODULE.summarize(path)

    def test_reasons_are_partitioned_by_phase(self):
        reason = "Terminal contract invalid"
        rows = [{"phase": phase, "terminalGeometry": {
            "taemTerminalBlockReason": reason, "terminalCommitted": False}}
            for phase in ("idle", "entryEnergy", "taem", "taem", "abort")]
        rows[2].update(ut=123.0, tickSequence=17)
        result = self.summarize(rows)
        self.assertEqual(result["phases"],
                         {"idle": 1, "entryEnergy": 1, "taem": 2, "abort": 1})
        self.assertEqual(result["blockReasonsByPhase"], {
            "idle": {reason: 1}, "entryEnergy": {reason: 1},
            "taem": {reason: 2}, "abort": {reason: 1}})
        self.assertEqual(result["taemBlockReasons"], {reason: 2})
        self.assertEqual(result["taemCommittedSamples"], 0)
        self.assertEqual(result["firstTaemSample"]["ut"], 123.0)
        self.assertEqual(result["firstTaemSample"]["tickSequence"], 17)

    def test_missing_geometry_and_commitment_scope(self):
        result = self.summarize([
            {}, {"phase": "idle", "terminalGeometry": None},
            {"phase": "taem"},
            {"phase": "taem", "terminalGeometry": {"terminalCommitted": True}},
            {"phase": "abort", "terminalGeometry": {"terminalCommitted": True}},
        ])
        self.assertEqual(result["taemBlockReasons"], {"unreported": 2})
        self.assertEqual(result["blockReasonsByPhase"]["idle"], {"unreported": 1})
        self.assertEqual(result["taemCommittedSamples"], 1)
        self.assertEqual(result["firstTaemSample"]["terminalGeometry"], {})

    def test_empty_log(self):
        result = self.summarize([])
        self.assertEqual(result["phases"], {})
        self.assertEqual(result["blockReasonsByPhase"], {})
        self.assertEqual(result["taemBlockReasons"], {})
        self.assertEqual(result["taemCommittedSamples"], 0)
        self.assertIsNone(result["firstTaemSample"])
        self.assertIn("forecasts, not live telemetry", result["limitation"])
        self.assertIn("default or retained state", result["limitation"])


if __name__ == "__main__":
    unittest.main()
