"""Run with: python3 -m unittest discover -s ShuttleSim/rl -p test_environment.py"""
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

from environment import (DEORBIT_COVERAGE_MAX_ALTITUDE_M, Environment, FEATURES,
                         MAX_INITIAL_INCLINATION_DEG,
                         deorbit_condition_feasible,
                         deorbit_delta_v, deorbit_entry_screen,
                         deorbit_entry_true_anomaly,
                         orbital_elements, project, taem_course_signed_error_deg,
                         taem_interface_debt, terminal_path_contract,
                         touchdown_outcome)
from policy import Policy


class EnvironmentTests(unittest.TestCase):
    def make(self, **kwargs):
        env = Environment(simulator_only=True, **kwargs)
        self.addCleanup(env.close)
        return env

    def test_explicit_opt_in(self):
        with self.assertRaises(ValueError):
            Environment()

    def test_reset_step_and_determinism(self):
        env = self.make(horizon=5)
        def run(seed):
            obs, info = env.reset(seed)
            result = [(obs, info)]
            for _ in range(5):
                result.append(env.step([.2, -.1]))
            return result
        first = run(77)
        self.assertEqual(first, run(77))
        self.assertNotEqual(first, run(78))
        self.assertTrue(first[-1][3])
        self.assertFalse(first[-1][4]["success"])
        with self.assertRaises(RuntimeError):
            env.step([0, 0])
        env.close()
        env.close()

    def test_curriculum_records_initial_handoff_without_losing_the_event(self):
        terminal = self.make(curriculum="terminal-corridor", horizon=2)
        _, reset_info = terminal.reset(0)
        self.assertTrue(reset_info["initial_handoff"])
        self.assertFalse(reset_info["initial_terminal_path_ready"])
        self.assertFalse(reset_info["initial_terminal_candidate_ready"])
        self.assertTrue(terminal.handoff_seen)
        self.assertEqual(terminal.first_handoff_elapsed, 0)
        _, _, terminated, truncated, step_info = terminal.step([0, 0])
        self.assertFalse(terminated)
        self.assertFalse(truncated)
        self.assertFalse(step_info["handoff_event"])
        self.assertFalse(step_info["terminal_path_committed"])
        self.assertEqual(step_info["elapsed"], 1)

        interface = self.make(curriculum="interface-corridor", horizon=2)
        _, reset_info = interface.reset(0)
        self.assertTrue(reset_info["initial_handoff"])
        self.assertFalse(interface.handoff_seen)
        self.assertTrue(interface.initial_handoff_pending)
        _, _, terminated, truncated, step_info = interface.step([0, 0])
        self.assertTrue(terminated)
        self.assertFalse(truncated)
        self.assertTrue(step_info["handoff_event"])
        self.assertEqual(step_info["elapsed"], 0)
        self.assertEqual(step_info["outcome"], "handoff_ready")

    def test_terminal_path_contract_does_not_confuse_handoff_with_commit(self):
        negative = terminal_path_contract({"diagnostics": {
            "candidate_valid": True,
            "candidate_degraded": False,
            "candidate_geometry_degraded": False,
            "candidate_energy_degraded": False,
            "candidate_live_energy_valid": True,
            "candidate_live_energy_margin_j_kg": -1.0,
            "path_committed": False,
        }})
        self.assertFalse(negative["candidate_ready"])
        self.assertFalse(negative["path_ready"])
        self.assertIn("negative-live-energy", negative["blockers"])
        self.assertIn("path-not-committed", negative["blockers"])

        committed = terminal_path_contract({"diagnostics": {
            "candidate_valid": True,
            "candidate_degraded": False,
            "candidate_geometry_degraded": False,
            "candidate_energy_degraded": False,
            "candidate_live_energy_valid": True,
            "candidate_live_energy_margin_j_kg": 1.0,
            "path_committed": True,
        }})
        self.assertTrue(committed["candidate_ready"])
        self.assertTrue(committed["path_ready"])
        self.assertEqual(committed["blockers"], [])

    def test_interleaved_instances_independent(self):
        a, b = self.make(), self.make()
        a.reset(12)
        b.reset(12)
        for _ in range(10):
            self.assertEqual(a.step([0, 0]), b.step([0, 0]))

    def test_schema(self):
        env = self.make()
        obs, _ = env.reset(1)
        self.assertEqual(len(obs), len(FEATURES))
        self.assertEqual(len(set(FEATURES)), len(FEATURES))
        self.assertTrue(all(math.isfinite(x) and abs(x) <= 5 for x in obs))
        self.assertFalse(any("force" in name or "mass" in name for name in FEATURES))
        self.assertEqual(FEATURES[-2:], ("capture_valid", "capture_ready"))

    def test_taem_inlet_geometry_tracks_configured_runway_heading(self):
        def packet(runway_heading, vehicle_heading):
            return {"runway": {"heading_deg": runway_heading},
                    "attitude": {"heading_deg": vehicle_heading}}
        self.assertEqual(taem_course_signed_error_deg(packet(90, 0)), 0.0)
        self.assertEqual(taem_course_signed_error_deg(packet(90, 180)), 0.0)
        self.assertEqual(taem_course_signed_error_deg(packet(45, -45)), 0.0)
        self.assertEqual(taem_course_signed_error_deg(packet(45, 135)), 0.0)
        self.assertEqual(abs(taem_course_signed_error_deg(packet(45, -15))), 30.0)

    def test_taem_debt_comes_from_production_capture_contract(self):
        telemetry = {"runway": {"heading_deg": 90}, "attitude": {"heading_deg": 0}}
        expert = {
            "taem_capture_valid": True, "taem_capture_ready": False,
            "taem_capture_veto": 2 | 16, "taem_capture_along_m": 300,
            "taem_capture_cross_m": 400, "taem_capture_altitude_error_m": -50,
            "taem_capture_energy_margin": -20, "taem_capture_turn_margin": 10,
        }
        debt = taem_interface_debt(telemetry, expert)
        self.assertEqual(debt["position_error_m"], 500.0)
        self.assertEqual(debt["constraint_violations"], 2)
        self.assertEqual(debt["capture_veto_reasons"], ["spatial", "maneuver-energy"])
        self.assertNotIn("interface_score", debt)
        self.assertNotIn("speed_error_mps", debt)
        invalid = taem_interface_debt(telemetry, {"taem_capture_valid": False,
                                                "taem_capture_ready": False,
                                                "taem_capture_veto": 0})
        self.assertIsNone(invalid["constraint_violations"])
        self.assertEqual(invalid["capture_veto_reasons"], ["capture-invalid"])

    def test_projection_boundaries(self):
        e = dict(enabled=True, abort=False, aoa=24, bank=35,
                 aoa_min=20, aoa_max=28, bank_max=60,
                 authority_valid=True, authority_survivable=True,
                 authority_controllable=True, authority_response_time_s=4.0)
        cmd, residual, reason = project([100, -100], e, [0, 0])
        self.assertEqual(cmd, [26.0, 20.0])
        self.assertEqual(residual, [2.0, -15.0])
        self.assertEqual(reason, "projected")
        for _ in range(20):
            cmd, residual, _ = project([100, -100], e, residual)
        self.assertEqual(residual, [4.0, -35.0])
        for bad in ([float("nan"), 0], [0], None, ["bad", 0]):
            self.assertEqual(project(bad, e, residual)[0], [24, 35])
            self.assertEqual(project(bad, e, residual)[2], "invalid_policy")
        for field, value in (("abort", True), ("enabled", False)):
            self.assertEqual(project([1, 1], {**e, field: value}, residual)[0], [24, 35])
        cmd, _, _ = project([1, 1], {**e, "aoa": 28, "bank": 60}, [2, 5])
        self.assertEqual(cmd, [28, 60])
        cmd, _, _ = project([0, -1], {**e, "bank": .1}, [0, -5])
        self.assertEqual(cmd[1], 0.0)
        one_sided = project([0, 0], {**e, "aoa": 6, "aoa_min": 18}, [0, 0])
        self.assertEqual(one_sided[2], "projected")
        self.assertEqual(one_sided[0][0], 6)
        self.assertEqual(one_sided[1][0], 0)
        raised = project([1, 0], {**e, "aoa": 6, "aoa_min": 18}, [0, 0])
        self.assertGreater(raised[0][0], one_sided[0][0])
        self.assertLessEqual(raised[0][0], 28)
        slow = project([1, 1], {**e, "authority_response_time_s": 8.0}, [0, 0])[1]
        fast = project([1, 1], {**e, "authority_response_time_s": 2.0}, [0, 0])[1]
        self.assertLess(abs(slow[0]), abs(fast[0]))
        self.assertLess(abs(slow[1]), abs(fast[1]))
        gated = project([1, 1], {**e, "authority_response_time_s": 0.0}, [0, 0])
        self.assertEqual(gated[2], "deterministic_gate")
        self.assertEqual(gated[1], [0.0, 0.0])

    def test_abort_never_advances_or_succeeds(self):
        env = self.make()
        env.reset(0)
        env.expert["abort"] = True
        before = env.lib.offline_telemetry(env.handle)
        _, _, terminated, _, info = env.step([1, 1])
        self.assertTrue(terminated)
        self.assertFalse(info["success"])
        self.assertEqual(info["elapsed"], 0)
        self.assertEqual(before, env.lib.offline_telemetry(env.handle))

    def test_jsonl_invalid_fallback(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            env = self.make(horizon=1, log=path)
            env.reset(1, "invalid-test")
            env.step([float("nan"), 0])
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual([r["type"] for r in rows], ["reset", "step", "episode"])
            self.assertEqual(rows[-1]["fallbacks"], 1)
            self.assertEqual(rows[-1]["projection"], "invalid_policy")

    def test_no_randomization_mutation_after_reset(self):
        env = self.make()
        env.reset(1)
        env.step([0, 0])
        self.assertEqual(env.lib.offline_randomize(env.handle, 1, 1, 1, 1, 1, 0), 0)

    def test_randomized_initial_condition_metadata(self):
        env = self.make(horizon=1)
        _, info = env.reset(12)
        condition = info["initial_condition"]
        atmosphere_top = env.telemetry["world"]["atmosphere_top_m"]
        self.assertGreaterEqual(condition["apoapsis_altitude_m"], atmosphere_top)
        self.assertLessEqual(condition["apoapsis_altitude_m"], DEORBIT_COVERAGE_MAX_ALTITUDE_M)
        self.assertGreaterEqual(condition["periapsis_altitude_m"], atmosphere_top)
        self.assertLessEqual(condition["periapsis_altitude_m"], condition["apoapsis_altitude_m"])
        self.assertGreaterEqual(condition["target_post_deorbit_periapsis_altitude_m"], 0.0)
        self.assertLess(condition["target_post_deorbit_periapsis_altitude_m"], atmosphere_top)
        self.assertGreaterEqual(condition["inclination_deg"], 0.0)
        self.assertLessEqual(condition["inclination_deg"], MAX_INITIAL_INCLINATION_DEG)
        screen = condition["mm304_taem_screen"]
        self.assertEqual(screen["entry_altitude_m"], atmosphere_top)
        self.assertLess(screen["entry_fpa_deg"], 0.0)
        self.assertGreater(screen["entry_speed_mps"], 0.0)
        self.assertGreater(screen["time_to_entry_s"], 0.0)
        self.assertGreater(condition["post_deorbit_orbital_elements"]["apoapsis_altitude_m"],
                           condition["post_deorbit_orbital_elements"]["periapsis_altitude_m"])
        self.assertEqual(condition, env.reset(12)[1]["initial_condition"])

    def test_deorbit_screen_rejects_unrealistic_condition(self):
        env = self.make(horizon=1)
        _, info = env.reset(12)
        condition = info["initial_condition"]
        world = env.telemetry["world"]
        feasible, prediction = deorbit_condition_feasible(
            condition, 0.0, env.runway, world)
        self.assertTrue(feasible)
        self.assertIsNotNone(prediction)

        non_retrograde = dict(condition, deorbit_delta_v_mps=0.0)
        rejected, rejected_prediction = deorbit_condition_feasible(
            non_retrograde, 0.0, env.runway, world)
        self.assertFalse(rejected)
        self.assertIsNotNone(rejected_prediction)

        # A steep but physically valid descending entry is no longer rejected by
        # an experiment-only FPA corridor; downstream simulation owns reachability.
        steep_entry = dict(condition, apoapsis_altitude_m=400000.0,
                           target_post_deorbit_periapsis_altitude_m=50000.0)
        steep_entry["deorbit_delta_v_mps"] = deorbit_delta_v(
            steep_entry["apoapsis_altitude_m"], steep_entry["periapsis_altitude_m"],
            steep_entry["target_post_deorbit_periapsis_altitude_m"], world)
        accepted, steep_prediction = deorbit_condition_feasible(
            steep_entry, 0.0, env.runway, world)
        self.assertTrue(accepted)
        self.assertLess(steep_prediction["entry_fpa_deg"], 0.0)

    def test_upper_inclination_can_pass_when_orbit_is_usable(self):
        env = self.make(randomized=False)
        env.reset(0)
        world = env.telemetry["world"]
        runway = {
            "latitude_deg": -0.0486111111,
            "longitude_deg": -74.7283333333,
            "heading_deg": 90.0,
        }
        apoapsis, periapsis, post_periapsis = 130000.0, 70000.0, 60000.0
        inclination = MAX_INITIAL_INCLINATION_DEG
        argument = -math.degrees(deorbit_entry_true_anomaly(
            apoapsis, post_periapsis, world))
        anchor = deorbit_entry_screen(
            apoapsis, post_periapsis, inclination, 0.0, argument, 0.0, runway, world)
        raan = (runway["longitude_deg"] - 18.0 - anchor["entry_longitude_deg"] + 180.0) % 360.0 - 180.0
        condition = {
            "apoapsis_altitude_m": apoapsis,
            "periapsis_altitude_m": periapsis,
            "inclination_deg": inclination,
            "raan_deg": raan,
            "argument_of_periapsis_deg": argument,
            "target_post_deorbit_periapsis_altitude_m": post_periapsis,
            "deorbit_delta_v_mps": deorbit_delta_v(apoapsis, periapsis, post_periapsis, world),
        }
        feasible, prediction = deorbit_condition_feasible(condition, 0.0, runway, world)
        self.assertTrue(feasible)
        self.assertAlmostEqual(prediction["entry_fpa_deg"], -2.02, places=1)

    def test_orbital_elements_are_diagnostic_only(self):
        env = self.make(randomized=False)
        env.reset(0)
        elements = orbital_elements(env.telemetry)
        self.assertTrue(math.isfinite(elements["specific_energy_m2_s2"]))
        self.assertIsNotNone(elements["semimajor_axis_m"])

    def test_export_inference_and_reject_corruption(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "policy.json"
            p = Policy()
            p.weights[0] = .1
            p.save(path)
            obs = [0.] * len(FEATURES)
            self.assertEqual(p(obs), Policy.load(path)(obs))
            data = json.loads(path.read_text())
            data["schema"] = "invalid"
            path.write_text(json.dumps(data))
            with self.assertRaises(ValueError):
                Policy.load(path)

    def test_touchdown_requires_valid_contact_and_stopped_rollout(self):
        s = dict(touchdown=True, on_runway=True, touchdown_cross_m=0,
                 touchdown_along_m=400, touchdown_sink_mps=1, touchdown_speed_mps=70)
        t = dict(ground=dict(gear_down=True, on_ground=True, stopped=True),
                 runway=dict(cross_m=0, along_m=1000, length_m=2500, width_m=70),
                 velocity=dict(surface_mps=0))
        e = dict(phase="COMPLETE", touchdown_speed_mps=75,
                 touchdown_sink_rate_mps=-1.5)
        self.assertEqual(touchdown_outcome(s, t, e, False), (True, False))
        self.assertFalse(touchdown_outcome(s, t, e, True)[0])
        for field, value in (("touchdown", False), ("on_runway", False),
                             ("touchdown_sink_mps", 2), ("touchdown_speed_mps", 80),
                             ("touchdown_cross_m", 36), ("touchdown_along_m", -1)):
            self.assertFalse(touchdown_outcome({**s, field: value}, t, e, False)[0])
        for group, field, value in (("ground", "gear_down", False),
                                    ("ground", "on_ground", False),
                                    ("ground", "stopped", False),
                                    ("runway", "along_m", 2501)):
            bad = copy.deepcopy(t)
            bad[group][field] = value
            self.assertFalse(touchdown_outcome(s, bad, e, False)[0])
        self.assertFalse(touchdown_outcome(s, t, {**e, "phase": "TAEM"}, False)[0])

        edge = {**s, "touchdown_cross_m": 40}
        self.assertFalse(touchdown_outcome(edge, t, e, False)[0])
        wider = copy.deepcopy(t)
        wider["runway"]["width_m"] = 100
        self.assertTrue(touchdown_outcome(edge, wider, e, False)[0])
        self.assertFalse(touchdown_outcome(s, t, {**e, "touchdown_speed_mps": 60}, False)[0])


if __name__ == "__main__":
    unittest.main()
