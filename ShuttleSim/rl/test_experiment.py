"""Run with: python3 -m unittest discover -s ShuttleSim/rl -p test_experiment.py"""
import json
from pathlib import Path
import tempfile
import unittest

from experiment import aggregate, candidate_score, curriculum_cases, policy_has_authority, promotion_report, run_cases
from policy import Policy


def episode(*, outcome="timeout", success=False, touchdown=False, rollout=False, active=0, ret=-1.0):
    return {
        "outcome": outcome,
        "success": success,
        "rollout_valid": rollout,
        "return": ret,
        "metrics": {"touchdown": touchdown},
        "eligible_steps": 10,
        "active_steps": active,
        "final_altitude_m": 50000.0,
        "final_along_m": 100000.0,
        "final_cross_m": 5000.0,
        "final_air_speed_mps": 2000.0,
        "taem_interface_debt": {"position_error_m": 100000.0},
    }


class ExperimentGateTests(unittest.TestCase):
    def test_zero_timeout_policy_is_rejected(self):
        policy = Policy()
        rows = [episode(), episode()]
        report = promotion_report(policy, [{"active_steps": 0}], rows, rows)
        self.assertFalse(report["deployable"])
        self.assertEqual(report["status"], "blocked_no_safe_policy_authority")
        self.assertIn("all_zero_policy", report["blockers"])
        self.assertIn("validation_all_timeout", report["blockers"])

    def test_successful_active_policy_can_promote(self):
        policy = Policy([0.01] + [0.0] * (len(Policy().weights) - 1))
        validation = [episode(outcome="success", success=True, touchdown=True, rollout=True, active=4, ret=10.0)]
        baseline = [episode(outcome="timeout", active=0, ret=-10.0)]
        report = promotion_report(policy, [{"active_steps": 4}], validation, baseline)
        self.assertTrue(report["deployable"])
        self.assertEqual(report["status"], "validated_deployable")




    def test_candidate_score_prefers_lower_interface_debt(self):
        worse = episode(active=2, ret=-100.0)
        worse["stage"] = "terminal-corridor"
        worse["taem_interface_debt"] = {"position_error_m": 20000.0}
        better = episode(active=2, ret=-100.0)
        better["stage"] = "terminal-corridor"
        better["taem_interface_debt"] = {"position_error_m": 5000.0}
        self.assertGreater(candidate_score([better]), candidate_score([worse]))

    def test_candidate_score_does_not_reward_activity_by_itself(self):
        quiet = [episode(active=0, ret=-10.0)]
        active = [episode(active=5, ret=-10.0)]
        self.assertEqual(candidate_score(active), candidate_score(quiet))


    def test_handoff_outcome_is_reported_but_not_deployable(self):
        row = episode(active=1, ret=42.0)
        row["outcome"] = "handoff_ready"
        summary = aggregate([row])
        self.assertEqual(summary["handoffs"], 1)
        promotion = promotion_report(
            Policy([0.1] * len(Policy().weights)), [], [row], [episode(ret=0.0)])
        self.assertEqual(promotion["status"], "interface_handoff_unproven")
        self.assertFalse(promotion["deployable"])

    def test_terminal_reset_handoff_does_not_count_as_terminal_progress(self):
        row = episode(active=1, ret=1.0)
        row.update({"stage": "terminal-corridor", "curriculum_handoff": True,
                    "handoff_event_seen": False,
                    "terminal_path_candidate_seen": False,
                    "terminal_path_committed": False})
        summary = aggregate([row])
        self.assertEqual(summary["handoffs"], 0)
        self.assertEqual(summary["curriculum_handoff_episodes"], 1)
        self.assertEqual(summary["terminal_path_episodes"], 1)
        self.assertEqual(summary["terminal_path_blocked"], 1)
        report = promotion_report(
            Policy([0.1] * len(Policy().weights)), [], [row], [episode(ret=0.0)])
        self.assertEqual(report["status"], "terminal_path_unproven")
        self.assertIn("terminal_path_unproven", report["blockers"])
        self.assertFalse(report["deployable"])

    def test_mixed_curriculum_has_short_interface_cases(self):
        cases = curriculum_cases("mixed")
        names = [case[0] for case in cases]
        self.assertIn("entry-corridor", names)
        self.assertIn("interface-corridor", names)
        self.assertIn("terminal-corridor", names)
        self.assertEqual(names[-1], "full")

    def test_policy_loader_can_require_deployable_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "policy.json"
            Policy().save(path, {"status": "smoke_failed_unusable", "deployable": False})
            with self.assertRaises(ValueError):
                Policy.load(path, require_deployable=True)
            data = json.loads(path.read_text())
            data["metadata"] = {"status": "validated_deployable", "deployable": True}
            path.write_text(json.dumps(data))
            self.assertIsInstance(Policy.load(path, require_deployable=True), Policy)

    def test_aggregate_exposes_terminal_diagnostics(self):
        summary = aggregate([episode(active=2), episode(active=3)])
        self.assertEqual(summary["episodes"], 2)
        self.assertEqual(summary["timeouts"], 2)
        self.assertEqual(summary["active_steps"], 5)
        self.assertEqual(summary["mean_final_taem_position_error_m"], 100000.0)


if __name__ == "__main__":
    unittest.main()
