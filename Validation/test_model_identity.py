#!/usr/bin/env python3
"""STS/STSN model identity regressions; no simulator or KSP is launched."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ShuttleSim" / "scripts"))
import model_paths


class ModelIdentityTests(unittest.TestCase):
    def test_every_named_profile_has_an_identity(self):
        self.assertEqual(set(model_paths.PROFILES), set(model_paths.PROFILE_VEHICLES))

    def test_default_is_still_stsn(self):
        with patch.dict("os.environ", {}, clear=True):
            self.assertEqual(model_paths.active_profile(), "identified")
            model_paths.validate_vehicle_profile({"vehicle": {"modelId": "STS-N"}})
            with self.assertRaisesRegex(ValueError, "cross-vehicle|Cross-vehicle"):
                model_paths.validate_vehicle_profile({"vehicle": {"modelId": "STS"}})

    def test_sts_requires_explicit_matching_profile(self):
        model_paths.validate_vehicle_profile({"vehicle": {"modelId": "STS"}}, "sts-provisional")
        with self.assertRaises(ValueError):
            model_paths.validate_vehicle_profile({"vehicle": {"modelId": "STS-N"}}, "sts-provisional")

    def test_missing_or_malformed_identity_fails_closed(self):
        for configuration in ({}, {"vehicle": None}, {"vehicle": []}, [], None,
                              {"vehicle": {"modelId": ""}}, {"vehicle": {"modelId": "sts"}}):
            with self.subTest(configuration=configuration), self.assertRaises(ValueError):
                model_paths.validate_vehicle_profile(configuration, "identified")

    def test_unknown_profile_fails_closed(self):
        with self.assertRaises(ValueError):
            model_paths.validate_vehicle_profile({"vehicle": {"modelId": "STS"}}, "unknown")

    def test_provisional_profile_does_not_gain_certification(self):
        self.assertEqual(model_paths.model_file("certified_prior", "sts-provisional"), "none")
        self.assertEqual(model_paths.model_file("aero_book", "sts-provisional"), "none")


class RunnerPreflightTests(unittest.TestCase):
    def test_wrong_profile_is_rejected_before_resources_are_allocated(self):
        import contextlib
        import io
        import run_guidance
        root = Path(__file__).resolve().parents[1]
        argv = ["run_guidance.py", "--configuration", str(root / "Configuration" / "sts.json"),
                "--scenario", str(root / "ShuttleSim" / "scenarios" / "sts70km-run9-provisional.ini")]
        with patch.dict("os.environ", {}, clear=True), patch.object(sys, "argv", argv), \
             patch.object(run_guidance, "_reserve_sim_endpoints") as reserve, \
             patch.object(run_guidance, "allocate_run") as allocate, \
             patch.object(run_guidance.subprocess, "Popen") as spawn, \
             contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                run_guidance.main()
            self.assertEqual(error.exception.code, 2)
            reserve.assert_not_called()
            allocate.assert_not_called()
            spawn.assert_not_called()


if __name__ == "__main__":
    unittest.main()
