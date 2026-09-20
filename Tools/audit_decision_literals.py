#!/usr/bin/env python3
"""Find raw numeric literals that participate in runtime flight policy.

The audit deliberately ignores ordinary literals in logs, buffers, serialization,
array indexing and arithmetic that is not used to make a decision.  It inspects:

* C/C++ if/while conditions and policy-bearing assignments/calls;
* Python If/While/IfExp conditions and policy-bearing assignments/calls.

A finding may be classified only with:
    decision-literal: <category> | <rationale>

Categories and dispositions are listed by --categories. This is an audit trail, not permission to hide a tuning knob behind a name.
"""

from __future__ import annotations

import argparse
import ast
import json
import re
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path

ROOTS = (
    "CLanding",
    "ShuttleSim/src",
    "ShuttleSim/include",
    "ShuttleSim/rl",
    "Experimental",
)
EXTENSIONS = {".c", ".h", ".py"}
SKIP_PARTS = {
    "build", "__pycache__", "runs", "FlightLogs", "ThirdParty",
    "node_modules", ".git",
}

NUMBER = re.compile(
    r"(?<![A-Za-z_])[-+]?(?:\d+\.\d*|\.\d+|\d+)"
    r"(?:[eE][-+]?\d+)?(?![A-Za-z_])"
)
IDENTIFIER = re.compile(r"\b[A-Za-z_][A-Za-z0-9_]*\b")
TRIVIAL = {"0", "0.0", "0.0f", "1", "1.0", "1.0f", "-1", "-1.0"}
ANNOTATION_MARKER = re.compile(r"decision-literal\s*:", re.I)
ANNOTATION = re.compile(
    r"decision-literal\s*:\s*([a-z0-9-]+)\s*\|\s*(.+?)(?:\*/)?\s*$",
    re.I | re.M,
)
LEGACY_SUPPRESS = re.compile(
    r"decision-literal-ok\s*:\s*(.+?)(?:\*/)?\s*$",
    re.I | re.M,
)

CATEGORY_ORDER = (
    "explicit-mission-contract",
    "physical-law-constant",
    "mathematical-numerical-requirement",
    "protocol-domain-requirement",
    "configuration-vehicle-parameter",
    "discretization-convergence-parameter",
    "derived-control-planning-quantity",
    "arbitrary-tuned-behavioral-threshold",
)

CATEGORY_DESCRIPTIONS = {
    "explicit-mission-contract": "explicit mission requirement",
    "physical-law-constant": "physical law or physical constant",
    "mathematical-numerical-requirement": "mathematical or numerical domain requirement",
    "protocol-domain-requirement": "external protocol or normalized domain requirement",
    "configuration-vehicle-parameter": "certified or configured vehicle/site parameter",
    "discretization-convergence-parameter": "documented accuracy or convergence parameter",
    "derived-control-planning-quantity": "must be derived from state, model, or geometry",
    "arbitrary-tuned-behavioral-threshold": "arbitrary or tuned behavioral threshold",
}
JUSTIFIED_CATEGORIES = frozenset(CATEGORY_ORDER[:6])
ACTION_CATEGORIES = frozenset(CATEGORY_ORDER[6:])
MIN_RATIONALE_CHARS = 12

POLICY_WORD = re.compile(
    r"(abort|admiss|authority|bank|capture|ceiling|confidence|corridor|course|"
    r"cross|deadband|delay|duration|energy|error|flare|floor|gain|guard|"
    r"handoff|heading|hysteresis|limit|load|margin|minimum|maximum|overspeed|"
    r"pitch|pressure|qbar|radius|range|rate|recovery|reserve|roll|safe|"
    r"settle|sink|speed|stall|threshold|timeout|touchdown|transition|valid|"
    r"velocity|vertical|yaw|aoa|altitude)",
    re.I,
)
POLICY_CALL = {
    "clampd", "fc_clamp", "clamp", "fmin", "fmax", "min", "max",
}


@dataclass(frozen=True)
class LiteralAnnotation:
    category: str | None
    rationale: str | None
    valid: bool
    legacy: bool = False
    error: str | None = None


@dataclass(frozen=True)
class Finding:
    path: str
    line: int
    literals: tuple[str, ...]
    score: int
    kind: str
    text: str
    category: str | None = None
    rationale: str | None = None
    annotation_valid: bool | None = None
    annotation_legacy: bool = False
    annotation_error: str | None = None


def runtime_files(root: Path) -> list[Path]:
    out: list[Path] = []
    for rel in ROOTS:
        base = root / rel
        if not base.exists():
            continue
        for path in base.rglob("*"):
            if not path.is_file() or path.suffix not in EXTENSIONS:
                continue
            parts = path.relative_to(root).parts
            if any(part in SKIP_PARTS or part.startswith("build") for part in parts):
                continue
            if path.name.startswith("test_") or "tests" in parts or "Validation" in parts:
                continue
            out.append(path)
    return sorted(set(out))


def strip_c_noncode(source: str) -> str:
    """Replace comments and string/character contents with spaces, preserving offsets."""
    chars = list(source)
    i = 0
    n = len(chars)
    state = "code"
    quote = ""
    while i < n:
        c = source[i]
        nxt = source[i + 1] if i + 1 < n else ""
        if state == "code":
            if c == "/" and nxt == "/":
                chars[i] = chars[i + 1] = " "
                i += 2
                state = "line_comment"
                continue
            if c == "/" and nxt == "*":
                chars[i] = chars[i + 1] = " "
                i += 2
                state = "block_comment"
                continue
            if c in ('"', "'"):
                quote = c
                chars[i] = " "
                state = "string"
                i += 1
                continue
        elif state == "line_comment":
            if c == "\n":
                state = "code"
            else:
                chars[i] = " "
        elif state == "block_comment":
            if c == "*" and nxt == "/":
                chars[i] = chars[i + 1] = " "
                i += 2
                state = "code"
                continue
            if c != "\n":
                chars[i] = " "
        elif state == "string":
            if c == "\\" and i + 1 < n:
                if c != "\n":
                    chars[i] = " "
                if source[i + 1] != "\n":
                    chars[i + 1] = " "
                i += 2
                continue
            if c == quote:
                chars[i] = " "
                state = "code"
            elif c != "\n":
                chars[i] = " "
        i += 1
    return "".join(chars)


def matching_paren(text: str, open_pos: int) -> int | None:
    depth = 0
    for i in range(open_pos, len(text)):
        if text[i] == "(":
            depth += 1
        elif text[i] == ")":
            depth -= 1
            if depth == 0:
                return i
    return None


def statement_end(text: str, start: int) -> int:
    par = br = sq = 0
    for i in range(start, len(text)):
        c = text[i]
        if c == "(":
            par += 1
        elif c == ")":
            par = max(0, par - 1)
        elif c == "{":
            br += 1
        elif c == "}":
            if br == 0:
                return i
            br -= 1
        elif c == "[":
            sq += 1
        elif c == "]":
            sq = max(0, sq - 1)
        elif c == ";" and par == br == sq == 0:
            return i
    return min(len(text) - 1, start + 1000)


def line_of(source: str, pos: int) -> int:
    return source.count("\n", 0, pos) + 1


def source_lines(source: str, start: int, end: int) -> str:
    lo = source.rfind("\n", 0, start) + 1
    hi = source.find("\n", end)
    if hi < 0:
        hi = len(source)
    return " ".join(source[lo:hi].strip().split())[:500]


def literals_in(expr: str) -> tuple[str, ...]:
    # Unsuffixed numeric values only.  Enum/bit encodings such as 8u are protocol,
    # not threshold literals and are skipped by the trailing identifier guard.
    return tuple(v for v in NUMBER.findall(expr) if v not in TRIVIAL)

def identifiers_in(expr: str) -> set[str]:
    return set(IDENTIFIER.findall(expr))



def parse_annotation(text: str) -> LiteralAnnotation | None:
    modern = list(ANNOTATION_MARKER.finditer(text))
    legacy = list(LEGACY_SUPPRESS.finditer(text))
    if modern and legacy:
        return LiteralAnnotation(None, None, False, error="modern and legacy annotations both apply")
    if len(modern) > 1:
        return LiteralAnnotation(None, None, False, error="multiple decision-literal annotations apply")
    if modern:
        match = ANNOTATION.search(text)
        if not match:
            return LiteralAnnotation(None, None, False, error="malformed decision-literal annotation")
        category = match.group(1).lower()
        rationale = match.group(2).strip()
        if category not in CATEGORY_DESCRIPTIONS:
            return LiteralAnnotation(category, rationale, False, error=f"unsupported category: {category}")
        if len(rationale) < MIN_RATIONALE_CHARS or not re.search(r"[A-Za-z]", rationale):
            return LiteralAnnotation(category, rationale, False, error="rationale is too weak to justify a literal")
        return LiteralAnnotation(category, rationale, True)
    if legacy:
        rationale = legacy[0].group(1).strip() if len(legacy) == 1 else None
        error = "legacy decision-literal-ok must be migrated" if len(legacy) == 1 else "multiple legacy annotations apply"
        return LiteralAnnotation("legacy-unclassified", rationale, False, legacy=True, error=error)
    return None


def annotation_for(source: str, start: int, end: int) -> LiteralAnnotation | None:
    lo = source.rfind("\n", 0, start) + 1
    hi = source.find("\n", end)
    if hi < 0:
        hi = len(source)
    span = source[lo:hi]
    if ANNOTATION_MARKER.search(span) or LEGACY_SUPPRESS.search(span):
        return parse_annotation(span)
    prev_end = max(0, lo - 1)
    prev = source.rfind("\n", 0, prev_end) + 1
    return parse_annotation(source[prev:prev_end])


def build_finding(path: str, line: int, literals: tuple[str, ...], score: int,
                  kind: str, text: str,
                  annotation: LiteralAnnotation | None) -> Finding:
    if annotation is None:
        return Finding(path, line, literals, score, kind, text)
    return Finding(
        path, line, literals, score, kind, text,
        annotation.category, annotation.rationale, annotation.valid,
        annotation.legacy, annotation.error,
    )


def c_findings(path: Path, root: Path) -> list[Finding]:
    source = path.read_text(errors="ignore")
    code = strip_c_noncode(source)
    spans: list[tuple[int, int, str, int]] = []
    decision_names: set[str] = set()


    # Branch/state-transition conditions.
    for m in re.finditer(r"\b(if|while)\b", code):
        j = m.end()
        while j < len(code) and code[j].isspace():
            j += 1
        if j >= len(code) or code[j] != "(":
            continue
        end = matching_paren(code, j)
        if end is not None:
            expr = code[j + 1:end]
            decision_names.update(identifiers_in(expr))
            # A numeric literal in ordinary mathematics is not flight-policy
            # authority.  Only promote a branch when its predicate names a
            # policy-bearing quantity; everything else remains visible at low
            # severity for audit/debugging but cannot fail --strict.
            spans.append((j + 1, end, "condition",
                          10 if POLICY_WORD.search(expr) else 4))

    # Ternary predicates commonly encode compact state gates.
    for m in re.finditer(r"(?P<expr>[^;{}\n?]{1,260})\?", code):
        start, end = m.start("expr"), m.end("expr")
        expr = code[start:end]
        decision_names.update(identifiers_in(expr))
        spans.append((start, end, "ternary",
                      9 if POLICY_WORD.search(expr) else 3))

    # Policy-bearing assignments: capture_limit=..., timeout=..., bank_guard=...
    assign = re.compile(
        r"\b(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*(?:\[[^\]]+\]\s*)?"
        r"(?P<op>\+=|-=|\*=|/=|=(?!=))"
    )
    for m in assign.finditer(code):
        name = m.group("name")
        if not POLICY_WORD.search(name):
            continue
        end = statement_end(code, m.end())
        statement = code[m.start():end + 1]
        decision_names.update(identifiers_in(statement))
        spans.append((m.start(), end + 1, "policy-assignment", 8))

    # Policy helpers can hide tuning outside assignments.
    call_re = re.compile(r"\b(clampd|fc_clamp|fmin|fmax)\s*\(")
    for m in call_re.finditer(code):
        open_pos = code.find("(", m.start())
        end = matching_paren(code, open_pos)
        if end is None:
            continue
        context_lo = max(0, code.rfind(";", 0, m.start()) + 1)
        while context_lo < m.start() and code[context_lo].isspace():
            context_lo += 1
        context = code[context_lo:end + 1]
        if POLICY_WORD.search(context):
            decision_names.update(identifiers_in(context))
            spans.append((context_lo, end + 1, "policy-call", 8))

    # Numeric assignments feeding a decision remain auditable even when
    # their variable names contain no policy words.
    for m in assign.finditer(code):
        name = m.group("name")
        if POLICY_WORD.search(name) or name not in decision_names:
            continue
        end = statement_end(code, m.end())
        statement = code[m.start():end + 1]
        if literals_in(statement):
            spans.append((m.start(), end + 1, "decision-source-assignment", 8))

    findings: list[Finding] = []
    seen: set[tuple[int, tuple[str, ...], str]] = set()
    for start, end, kind, score in spans:
        expr = code[start:end]
        literals = literals_in(expr)
        if not literals:
            continue
        line = line_of(source, start)
        key = (line, literals, kind)
        if key in seen:
            continue
        seen.add(key)
        annotation = annotation_for(source, start, end)
        findings.append(build_finding(
            str(path.relative_to(root)),
            line,
            literals,
            score + (2 if score >= 8 and POLICY_WORD.search(expr) else 0),
            kind,
            source_lines(source, start, end),
            annotation,
        ))
    return findings


def py_node_literals(node: ast.AST) -> tuple[str, ...]:
    values: list[str] = []
    for child in ast.walk(node):
        if isinstance(child, ast.Constant) and isinstance(child.value, (int, float)):
            v = child.value
            if v in (0, 1, -1):
                continue
            values.append(repr(v))
    return tuple(values)


def py_identifiers(node: ast.AST | None) -> set[str]:
    if node is None:
        return set()
    names: set[str] = set()
    for child in ast.walk(node):
        if isinstance(child, ast.Name):
            names.add(child.id)
        elif isinstance(child, ast.Attribute):
            names.add(child.attr)
    return names


def py_assignment_names(node: ast.AST) -> list[str]:
    names: list[str] = []
    targets = node.targets if isinstance(node, ast.Assign) else [node.target]
    for target in targets:
        if isinstance(target, ast.Name):
            names.append(target.id)
        elif isinstance(target, ast.Attribute):
            names.append(target.attr)
    return names


def py_text(lines: list[str], node: ast.AST) -> str:
    a = max(1, getattr(node, "lineno", 1))
    b = max(a, getattr(node, "end_lineno", a))
    return " ".join(" ".join(lines[i - 1].strip().split()) for i in range(a, min(b, len(lines)) + 1))[:500]


def py_annotation(lines: list[str], node: ast.AST) -> LiteralAnnotation | None:
    a = max(1, getattr(node, "lineno", 1))
    b = max(a, getattr(node, "end_lineno", a))
    span = "\n".join(lines[a - 1:min(len(lines), b)])
    if ANNOTATION_MARKER.search(span) or LEGACY_SUPPRESS.search(span):
        return parse_annotation(span)
    if a <= 1:
        return None
    return parse_annotation(lines[a - 2])


def py_findings(path: Path, root: Path) -> list[Finding]:
    source = path.read_text(errors="ignore")
    lines = source.splitlines()
    try:
        tree = ast.parse(source)
    except SyntaxError:
        return []
    decision_names: set[str] = set()
    for candidate in ast.walk(tree):
        if isinstance(candidate, (ast.If, ast.While, ast.IfExp)):
            decision_names.update(py_identifiers(candidate.test))
        elif isinstance(candidate, (ast.Assign, ast.AnnAssign, ast.AugAssign)):
            names = py_assignment_names(candidate)
            if any(POLICY_WORD.search(name) for name in names):
                decision_names.update(py_identifiers(candidate.value))
        elif isinstance(candidate, ast.Call):
            func_name = ""
            if isinstance(candidate.func, ast.Name):
                func_name = candidate.func.id
            elif isinstance(candidate.func, ast.Attribute):
                func_name = candidate.func.attr
            if func_name in POLICY_CALL and POLICY_WORD.search(py_text(lines, candidate)):
                decision_names.update(py_identifiers(candidate))

    findings: list[Finding] = []
    for node in ast.walk(tree):
        target: ast.AST | None = None
        kind = ""
        score = 0
        if isinstance(node, (ast.If, ast.While, ast.IfExp)):
            target = node.test
            kind = "condition"
            predicate_text = py_text(lines, target)
            score = 10 if POLICY_WORD.search(predicate_text) else 4
        elif isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign)):
            names = py_assignment_names(node)
            policy_assignment = any(POLICY_WORD.search(name) for name in names)
            decision_source = any(name in decision_names for name in names)
            if policy_assignment or decision_source:
                target = node.value
                kind = ("policy-assignment" if policy_assignment else "decision-source-assignment")
                score = 8
        elif isinstance(node, ast.Call):
            name = ""
            if isinstance(node.func, ast.Name):
                name = node.func.id
            elif isinstance(node.func, ast.Attribute):
                name = node.func.attr
            if name in POLICY_CALL:
                text = py_text(lines, node)
                if POLICY_WORD.search(text):
                    target = node
                    kind = "policy-call"
                    score = 8
        if target is None:
            continue
        literals = py_node_literals(target)
        if not literals:
            continue
        text = py_text(lines, node)
        annotation = py_annotation(lines, target)
        findings.append(build_finding(
            str(path.relative_to(root)),
            getattr(node, "lineno", 1),
            literals,
            score + (2 if score >= 8 and POLICY_WORD.search(text) else 0),
            kind,
            text,
            annotation,
        ))
    return findings


def audit(root: Path) -> list[Finding]:
    findings: list[Finding] = []
    for path in runtime_files(root):
        findings.extend(py_findings(path, root) if path.suffix == ".py"
                        else c_findings(path, root))
    findings.sort(key=lambda f: (-f.score, f.path, f.line, f.kind))
    return findings


def finding_is_justified(finding: Finding) -> bool:
    return finding.annotation_valid is True and finding.category in JUSTIFIED_CATEGORIES


def finding_is_active(finding: Finding) -> bool:
    return not finding_is_justified(finding)


def report_category(finding: Finding) -> str:
    if finding.annotation_legacy:
        return "legacy-unclassified"
    if finding.annotation_valid is False:
        return "invalid-annotation"
    return finding.category or "unclassified"


def finding_is_strict_failure(finding: Finding) -> bool:
    if not finding_is_active(finding):
        return False
    if finding.annotation_valid is False:
        return True
    # All branch predicates remain strict decision sites even when variable
    # names are deliberately made generic, so renaming cannot hide a threshold.
    return finding.kind in {"condition", "ternary"} or finding.score >= 8


def finding_json(finding: Finding) -> dict[str, object]:
    item = asdict(finding)
    item["active"] = finding_is_active(finding)
    item["strictFailure"] = finding_is_strict_failure(finding)
    item["reportCategory"] = report_category(finding)
    return item


def category_counts(findings: list[Finding]) -> Counter[str]:
    return Counter(report_category(f) for f in findings)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=".")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--top", type=int, default=120)
    ap.add_argument("--strict", action="store_true")
    ap.add_argument("--categories", action="store_true",
                    help="print semantic categories and exit")
    args = ap.parse_args()

    if args.categories:
        for category in CATEGORY_ORDER:
            disposition = "justified" if category in JUSTIFIED_CATEGORIES else "action-required"
            print(f"{category}: {disposition}: {CATEGORY_DESCRIPTIONS[category]}")
        return 0

    findings = audit(Path(args.root).resolve())
    active = [f for f in findings if finding_is_active(f)]
    justified = [f for f in findings if finding_is_justified(f)]
    high = [f for f in active if f.score >= 10]
    strict_failures = [f for f in active if finding_is_strict_failure(f)]
    annotation_errors = [f for f in findings if f.annotation_valid is False and not f.annotation_legacy]
    legacy = [f for f in findings if f.annotation_legacy]
    categories = category_counts(findings)
    top = max(0, args.top)

    if args.json:
        print(json.dumps({
            "files": len({f.path for f in findings}),
            "findings": len(findings),
            "active": len(active),
            "justified": len(justified),
            "highRisk": len(high),
            "strictFailures": len(strict_failures),
            "invalidAnnotations": len(annotation_errors),
            "legacyAnnotations": len(legacy),
            "categoryCounts": dict(sorted(categories.items())),
            "categoryDefinitions": {
                category: {
                    "description": CATEGORY_DESCRIPTIONS[category],
                    "disposition": "justified" if category in JUSTIFIED_CATEGORIES else "action-required",
                }
                for category in CATEGORY_ORDER
            },
            "items": [finding_json(f) for f in findings[:top]],
        }, indent=2))
    else:
        counts = Counter(f.path for f in strict_failures)
        print(f"decision literal findings: {len(active)} active / {len(findings)} total ({len(justified)} justified)")
        print(f"strict failures: {len(strict_failures)}; high-risk active: {len(high)}")
        print(f"annotation debt: {len(annotation_errors)} invalid, {len(legacy)} legacy")
        print("category inventory:")
        for category in list(CATEGORY_ORDER) + ["unclassified", "legacy-unclassified", "invalid-annotation"]:
            if categories.get(category):
                print(f"{categories[category]:5d}  {category}")
        print("\nstrict-failure files:")
        for path_name, count in counts.most_common(20):
            print(f"{count:5d}  {path_name}")
        print("\nhighest-risk active:")
        for finding in active[:top]:
            detail = f" category={report_category(finding)}"
            if finding.annotation_error:
                detail += f" annotation-error={finding.annotation_error}"
            print(f"{finding.score:02d} {finding.path}:{finding.line} {finding.kind} [{','.join(finding.literals)}]{detail} {finding.text}")

    return 1 if args.strict and strict_failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
