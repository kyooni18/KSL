#!/usr/bin/env python3
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Tools"))
import audit_decision_literals as audit


class DecisionLiteralAuditTests(unittest.TestCase):
    def audit_c(self, source: str):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            target = root / "CLanding" / "fixture.c"
            target.parent.mkdir(parents=True)
            target.write_text(source)
            return audit.audit(root)

    def one(self, source: str):
        findings = self.audit_c(source)
        self.assertEqual(len(findings), 1, findings)
        return findings[0]

    def test_explicit_mission_contract_is_justified(self):
        finding = self.one(
            "if (endpoint_error <= 1000.0) {} "
            "/* decision-literal: explicit-mission-contract | "
            "MM304 capture is explicitly limited to one kilometre */\n"
        )
        self.assertTrue(audit.finding_is_justified(finding))
        self.assertFalse(audit.finding_is_active(finding))
        self.assertFalse(audit.finding_is_strict_failure(finding))

    def test_derived_quantity_remains_action_required(self):
        finding = self.one(
            "if (stopping_distance > 400.0) {} "
            "/* decision-literal: derived-control-planning-quantity | "
            "derive this distance from measured response and speed */\n"
        )
        self.assertEqual(finding.category, "derived-control-planning-quantity")
        self.assertTrue(finding.annotation_valid)
        self.assertTrue(audit.finding_is_active(finding))
        self.assertTrue(audit.finding_is_strict_failure(finding))

    def test_arbitrary_threshold_remains_action_required(self):
        finding = self.one(
            "if (capture_error < 12.0) {} "
            "/* decision-literal: arbitrary-tuned-behavioral-threshold | "
            "historical tuned capture gate pending replacement */\n"
        )
        self.assertTrue(finding.annotation_valid)
        self.assertTrue(audit.finding_is_strict_failure(finding))

    def test_invalid_category_fails_strict(self):
        finding = self.one(
            "if (capture_error < 12.0) {} "
            "/* decision-literal: convenient-threshold | this is not taxonomy */\n"
        )
        self.assertFalse(finding.annotation_valid)
        self.assertTrue(audit.finding_is_strict_failure(finding))

    def test_legacy_suppression_is_migration_debt(self):
        finding = self.one(
            "if (bank < 89.0) {} "
            "/* decision-literal-ok: tangent domain protection */\n"
        )
        self.assertTrue(finding.annotation_legacy)
        self.assertEqual(audit.report_category(finding), "legacy-unclassified")
        self.assertTrue(audit.finding_is_strict_failure(finding))

    def test_generic_variable_rename_does_not_hide_branch(self):
        finding = self.one("if (x > 400.0) {}\n")
        self.assertLess(finding.score, 8)
        self.assertTrue(audit.finding_is_strict_failure(finding))

    def test_preceding_line_annotation_applies(self):
        finding = self.one(
            "/* decision-literal: mathematical-numerical-requirement | "
            "exclude the tangent singularity at ninety degrees */\n"
            "if (bank < 90.0) {}\n"
        )
        self.assertTrue(audit.finding_is_justified(finding))

    def test_weak_rationale_is_invalid(self):
        finding = self.one(
            "if (bank < 90.0) {} "
            "/* decision-literal: mathematical-numerical-requirement | math */\n"
        )
        self.assertFalse(finding.annotation_valid)
        self.assertTrue(audit.finding_is_strict_failure(finding))

    def test_python_annotation_uses_same_taxonomy(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            target = root / "ShuttleSim" / "rl" / "fixture.py"
            target.parent.mkdir(parents=True)
            target.write_text(
                "# decision-literal: protocol-domain-requirement | "
                "external retry field is bounded by the protocol\n"
                "if retry_limit > 4:\n    pass\n"
            )
            findings = audit.audit(root)
        self.assertEqual(len(findings), 1, findings)
        self.assertTrue(audit.finding_is_justified(findings[0]))

    def test_c_named_constant_feeding_branch_is_audited(self):
        findings = self.audit_c(
            "const double x = 400.0;\n"
            "if (capture_error > x) {}\n"
        )
        self.assertEqual(len(findings), 1, findings)
        self.assertEqual(findings[0].kind, "decision-source-assignment")
        self.assertEqual(findings[0].literals, ("400.0",))
        self.assertTrue(audit.finding_is_strict_failure(findings[0]))

    def test_python_named_constant_feeding_branch_is_audited(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            target = root / "ShuttleSim" / "rl" / "fixture.py"
            target.parent.mkdir(parents=True)
            target.write_text("x = 400.0\nif capture_error > x:\n    pass\n")
            findings = audit.audit(root)
        self.assertEqual(len(findings), 1, findings)
        self.assertEqual(findings[0].kind, "decision-source-assignment")
        self.assertTrue(audit.finding_is_strict_failure(findings[0]))

    def test_python_annotation_only_assignment_is_null_safe(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            target = root / "ShuttleSim" / "rl" / "fixture.py"
            target.parent.mkdir(parents=True)
            target.write_text("limit: float\nif capture_error > limit:\n    pass\n")
            findings = audit.audit(root)
        self.assertEqual(findings, [])

    def test_adjacent_same_line_annotations_do_not_conflict(self):
        findings = self.audit_c(
            "if (bank < 90.0) {} /* decision-literal: "
            "mathematical-numerical-requirement | exclude tangent singularity */\n"
            "if (pitch < 90.0) {} /* decision-literal: "
            "mathematical-numerical-requirement | exclude vertical singularity */\n"
        )
        self.assertEqual(len(findings), 2, findings)
        self.assertTrue(all(audit.finding_is_justified(f) for f in findings))

    def test_python_adjacent_annotations_do_not_conflict(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            target = root / "ShuttleSim" / "rl" / "fixture.py"
            target.parent.mkdir(parents=True)
            target.write_text(
                "if bank < 90.0:  # decision-literal: "
                "mathematical-numerical-requirement | exclude tangent singularity\n"
                "    pass\n"
                "if pitch < 90.0:  # decision-literal: "
                "mathematical-numerical-requirement | exclude vertical singularity\n"
                "    pass\n"
            )
            findings = audit.audit(root)
        self.assertEqual(len(findings), 2, findings)
        self.assertTrue(all(audit.finding_is_justified(f) for f in findings))

    def test_python_body_annotation_cannot_justify_header(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            target = root / "ShuttleSim" / "rl" / "fixture.py"
            target.parent.mkdir(parents=True)
            target.write_text(
                "if x > 4:\n"
                "    # decision-literal: protocol-domain-requirement | body-only note\n"
                "    pass\n"
            )
            findings = audit.audit(root)
        self.assertEqual(len(findings), 1, findings)
        self.assertEqual(audit.report_category(findings[0]), "unclassified")
        self.assertTrue(audit.finding_is_strict_failure(findings[0]))


if __name__ == "__main__":
    unittest.main()
