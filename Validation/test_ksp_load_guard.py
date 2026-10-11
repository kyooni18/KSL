#!/usr/bin/env python3
"""Offline save-load verification: never accept the old paused scene."""
from __future__ import annotations

import itertools
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Tools"))
try:
    import krpc
except ModuleNotFoundError:
    sys.modules["krpc"] = types.ModuleType("krpc")
import ksp_test_guard as guard
from ksl.kspguard import GuardError, load_save_verified


def sample(ut):
    return {"ut": ut, "vessel": "STS", "meanAltitude": 70000.0,
            "railsWarpFactor": 0, "physicsWarpFactor": 0, "gameScene": "flight"}


class LoadEvidenceTests(unittest.TestCase):
    def settled(self, ut, **kwargs):
        return guard.load_state_is_settled(sample(ut), before_ut=1000.0,
                                           target_ut=kwargs.pop("target_ut", 2000.0),
                                           elapsed=5.0, **kwargs)

    def test_forward_load(self):
        self.assertTrue(self.settled(2000.0))

    def test_old_paused_scene(self):
        self.assertFalse(self.settled(1000.0))

    def test_old_scene_already_inside_target_window(self):
        self.assertFalse(self.settled(1000.0, target_ut=1000.0))

    def test_wrong_rewind_is_not_requested_save(self):
        self.assertFalse(self.settled(500.0))

    def test_same_clock_requires_load_ack_or_scene_transition(self):
        self.assertTrue(self.settled(1000.0, target_ut=1000.0, load_acknowledged=True))
        self.assertTrue(self.settled(1000.0, target_ut=1000.0, scene_transition_observed=True))

    def test_normal_clock_progress_is_not_forward_load_evidence(self):
        self.assertFalse(self.settled(1005.0, target_ut=1004.0))

    def test_nonfinite_clock_fails_closed(self):
        self.assertFalse(self.settled(float("nan"), load_acknowledged=True))

    def test_warp_or_wrong_scene_cannot_settle(self):
        for key, value in (("railsWarpFactor", 1), ("physicsWarpFactor", 1), ("gameScene", "loading")):
            state = sample(2000.0)
            state[key] = value
            self.assertFalse(guard.load_state_is_settled(state, before_ut=1000.0,
                                                        target_ut=2000.0, elapsed=5.0))

    def test_orbit_load_waits_for_postload_mass_refresh(self):
        state = sample(5249.4)
        state.update(situation="VesselSituation.orbiting", mass=22000.0)
        self.assertFalse(guard.orbital_load_physics_ready(state, 5248.5))
        state.update(ut=5253.5, mass=25521.25)
        self.assertTrue(guard.orbital_load_physics_ready(state, 5248.5))
        state["mass"] = float("nan")
        self.assertFalse(guard.orbital_load_physics_ready(state, 5248.5))

    def test_pad_load_waits_for_physics_settle(self):
        state = sample(2683.1)
        state.update(situation="VesselSituation.pre_launch", mass=35000.0,
                     meanAltitude=86.5)
        self.assertFalse(guard.orbital_load_physics_ready(state, 2682.9))
        state.update(ut=2688.0, mass=520694.8)
        self.assertTrue(guard.orbital_load_physics_ready(state, 2682.9))

    def test_cli_load_requires_the_requested_settled_flight(self):
        with patch("ksl.kspguard.run_guard", return_value={"event": "load-save-settled",
                                                       "name": "other", "gameScene": "flight"}) as invoke:
            with self.assertRaises(GuardError):
                load_save_verified("STS-test")
            invoke.assert_called_once_with("load-save", env=None, timeout=90.0,
                                           args=["--name", "STS-test"])


class LoadLoopTests(unittest.TestCase):
    def run_load(self, before, after, target, target_parts=None, live_parts=None):
        def connection(ut):
            client = Mock()
            client.space_center.ut = ut
            client.space_center.rails_warp_factor = 0
            client.space_center.physics_warp_factor = 0
            client.space_center.active_vessel.name = "STS"
            client.space_center.active_vessel.mass = 25000.0
            client.space_center.active_vessel.situation = "VesselSituation.flying"
            client.space_center.active_vessel.flight.return_value.mean_altitude = 70000.0
            if live_parts is not None:
                client.space_center.active_vessel.parts.all = [Mock()] * live_parts
            client.krpc.current_game_scene = types.SimpleNamespace(name="flight")
            return client
        initial, current = connection(before), connection(after)
        clients = itertools.chain([initial], itertools.repeat(current))
        with patch.object(guard, "connect", side_effect=lambda *a, **k: next(clients)), \
             patch.object(guard, "saved_game_ut", return_value=target), \
             patch.object(guard, "saved_game_part_count", return_value=target_parts), \
             patch.object(guard.threading, "Thread"), \
             patch.object(guard.time, "monotonic", side_effect=itertools.count()), \
             patch.object(guard.time, "sleep"), patch.object(guard, "emit"):
            return guard.load_save("STS-test")

    def test_failed_load_does_not_accept_stable_old_scene(self):
        self.assertEqual(self.run_load(1000.0, 1000.0, 2000.0), 2)

    def test_same_paused_clock_is_not_load_success(self):
        self.assertEqual(self.run_load(1000.0, 1000.0, 1000.0), 2)

    def test_wrong_checkpoint_rewind_is_rejected(self):
        self.assertEqual(self.run_load(10000.0, 1000.0, 9000.0), 2)

    def test_forward_target_load_is_accepted(self):
        self.assertEqual(self.run_load(1290.0, 9930.2, 9930.0), 0)

    def test_partial_vessel_load_is_rejected(self):
        self.assertEqual(self.run_load(1290.0, 9930.2, 9930.0,
                                       target_parts=41, live_parts=13), 2)
        self.assertEqual(self.run_load(1290.0, 9930.2, 9930.0,
                                       target_parts=41, live_parts=41), 0)


if __name__ == "__main__":
    unittest.main()
