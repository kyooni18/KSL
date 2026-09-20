#!/usr/bin/env python3
"""Simulator-only CEM training/evaluation. No network or live guidance adapter."""
import argparse
from collections import Counter
import json
from pathlib import Path
import random
import statistics

from environment import Environment
from policy import Policy, SIZE



def policy_has_authority(policy):
    return any(abs(weight) > 1e-12 for weight in policy.weights)


def _mean_present(rows, key):
    values = [r.get(key) for r in rows if isinstance(r.get(key), (int, float))]
    return statistics.mean(values) if values else None

TRAIN_SEEDS = (11, 12)
VALIDATION_SEEDS = (101, 102)
EVALUATION_SEEDS = (1001, 1002, 1003)
HELDOUT_SEEDS = (2001, 2002)
BASE = "ksp86km-postburn.ini"
HELDOUT = "ksp86km-heldout-20260915.ini"
CURRICULUM_TRAIN_SEEDS = (21, 22)
CURRICULUM_VALIDATION_SEEDS = (121, 122)
CURRICULUM_STAGES = {
    "full": (("full", None, TRAIN_SEEDS, None),),
    "entry": (("entry-corridor", "entry-corridor", CURRICULUM_TRAIN_SEEDS, 900),),
    "interface": (("interface-corridor", "interface-corridor", CURRICULUM_TRAIN_SEEDS, 600),
                  ("terminal-corridor", "terminal-corridor", CURRICULUM_TRAIN_SEEDS, 720)),
    "mixed": (("entry-corridor", "entry-corridor", CURRICULUM_TRAIN_SEEDS, 900),
              ("interface-corridor", "interface-corridor", CURRICULUM_TRAIN_SEEDS, 600),
              ("terminal-corridor", "terminal-corridor", CURRICULUM_TRAIN_SEEDS, 720),
              ("full", None, TRAIN_SEEDS[:1], None)),
}
CURRICULUM_VALIDATION = {
    "full": (("full", None, VALIDATION_SEEDS, None),),
    "entry": (("entry-corridor", "entry-corridor", CURRICULUM_VALIDATION_SEEDS, 900),),
    "interface": (("interface-corridor", "interface-corridor", CURRICULUM_VALIDATION_SEEDS, 600),
                  ("terminal-corridor", "terminal-corridor", CURRICULUM_VALIDATION_SEEDS, 720)),
    "mixed": (("entry-corridor", "entry-corridor", CURRICULUM_VALIDATION_SEEDS, 900),
              ("interface-corridor", "interface-corridor", CURRICULUM_VALIDATION_SEEDS, 600),
              ("terminal-corridor", "terminal-corridor", CURRICULUM_VALIDATION_SEEDS, 720),
              ("full", None, VALIDATION_SEEDS[:1], None)),
}


def _stage_horizon(global_horizon, stage_horizon):
    return global_horizon if stage_horizon is None else min(global_horizon, stage_horizon)


def rollout(policy, seed, *, scenario=BASE, horizon=3600, log=None,
            stress=None, randomized=True, version=None, curriculum=None, stage=None):
    env = Environment(simulator_only=True, scenario=scenario, horizon=horizon,
                      log=log, stress=stress, randomized=randomized,
                      curriculum=curriculum)
    try:
        obs, _ = env.reset(seed, version or getattr(policy, "version", "heuristic"))
        while True:
            obs, _, terminated, truncated, info = env.step(policy(obs))
            if terminated or truncated:
                info.update({"curriculum": curriculum or "full",
                             "stage": stage or curriculum or "full"})
                return info
    finally:
        env.close()


def aggregate(rows):
    terminal_rows = [r for r in rows
                     if (r.get("stage") or r.get("curriculum")) == "terminal-corridor"]
    return {"episodes": len(rows), "successes": sum(r["success"] for r in rows),
            "touchdowns": sum(r["metrics"]["touchdown"] for r in rows),
            "valid_rollouts": sum(r["rollout_valid"] for r in rows),
            # A terminal curriculum reset can begin inside the local MM304
            # set. Count only an observed handoff event here; the downstream
            # terminal path has its own explicit contract below.
            "handoffs": sum(bool(r.get("handoff_event_seen")) or
                             r.get("outcome") == "handoff_ready" for r in rows),
            "curriculum_handoff_episodes": sum(bool(r.get("curriculum_handoff")) for r in rows),
            "handoff_terminal_outcomes": sum(r.get("outcome") == "handoff_ready" for r in rows),
            "terminal_path_episodes": len(terminal_rows),
            "terminal_path_candidates": sum(bool(r.get("terminal_path_candidate_seen"))
                                            for r in terminal_rows),
            "terminal_path_commits": sum(bool(r.get("terminal_path_committed"))
                                          for r in terminal_rows),
            "terminal_path_blocked": sum(
                not bool(r.get("terminal_path_committed")) for r in terminal_rows),
            "mean_return": statistics.mean(r["return"] for r in rows) if rows else None,
            "outcomes": dict(Counter(r["outcome"] for r in rows)),
            "eligible_steps": sum(r["eligible_steps"] for r in rows),
            "active_steps": sum(r["active_steps"] for r in rows),
            "timeouts": sum(r["outcome"] == "timeout" for r in rows),
            "mean_final_altitude_m": _mean_present(rows, "final_altitude_m"),
            "mean_final_along_m": _mean_present(rows, "final_along_m"),
            "mean_final_cross_m": _mean_present(rows, "final_cross_m"),
            "mean_final_air_speed_mps": _mean_present(rows, "final_air_speed_mps"),
            "mean_final_taem_position_error_m": _mean_present([
                {"value": r.get("taem_interface_debt", {}).get("position_error_m")}
                for r in rows if isinstance(r.get("taem_interface_debt"), dict)
            ], "value"),
            "mean_final_course_error_deg": _mean_present(rows, "final_course_error_deg"),
            "unsafe_reasons": dict(Counter(
                reason
                for r in rows
                for reason in (r.get("unsafe_reasons") or [])
            )),
            "stages": dict(Counter(str(r.get("stage") or r.get("curriculum") or "unknown") for r in rows))}


def promotion_report(policy, history, validation, baseline_validation=None):
    validation_summary = aggregate(validation)
    baseline_summary = aggregate(baseline_validation or []) if baseline_validation is not None else None
    blockers = []
    if not policy_has_authority(policy):
        blockers.append("all_zero_policy")
    if validation_summary["episodes"] == 0:
        blockers.append("no_validation_episodes")
    if validation_summary["active_steps"] <= 0:
        blockers.append("no_active_validation_steps")
    if (validation_summary["timeouts"] == validation_summary["episodes"]
            and validation_summary.get("handoffs", 0) == 0):
        blockers.append("validation_all_timeout")
    if validation_summary["touchdowns"] <= 0:
        blockers.append("no_touchdown")
    if (validation_summary.get("terminal_path_episodes", 0) > 0 and
            validation_summary.get("terminal_path_commits", 0) <= 0):
        blockers.append("terminal_path_unproven")
    if validation_summary["successes"] <= 0 or validation_summary["valid_rollouts"] <= 0:
        blockers.append("no_valid_runway_rollout")
    improvement = None
    if baseline_summary and validation_summary["mean_return"] is not None and baseline_summary["mean_return"] is not None:
        improvement = validation_summary["mean_return"] - baseline_summary["mean_return"]
        if improvement <= 0.0:
            blockers.append("no_validation_return_improvement")
    deployable = not blockers
    if deployable:
        status = "validated_deployable"
    elif not policy_has_authority(policy) and not any(h.get("active_steps", 0) for h in history):
        status = "blocked_no_safe_policy_authority"
    elif "terminal_path_unproven" in blockers:
        status = "terminal_path_unproven"
    elif validation_summary.get("handoffs", 0) > 0:
        status = "interface_handoff_unproven"
    elif validation_summary["touchdowns"] > 0:
        status = "terminal_touchdown_unproven_rollout"
    else:
        status = "smoke_failed_unusable"
    return {
        "status": status,
        "deployable": deployable,
        "blockers": blockers,
        "validation": validation_summary,
        "baseline_validation": baseline_summary,
        "mean_return_improvement": improvement,
    }


def write_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def candidate_score(rows):
    """Lexicographic mission score; no weighted proxy objective."""
    summary = aggregate(rows)
    episodes = max(1, summary["episodes"])
    interface_rows = [
        row for row in rows
        if (row.get("stage") or row.get("curriculum")) in ("interface-corridor", "terminal-corridor")
    ] or rows
    interface_summary = aggregate(interface_rows)
    interface_episodes = max(1, interface_summary["episodes"])
    position_error = interface_summary.get("mean_final_taem_position_error_m")
    mean_return = summary["mean_return"]
    return (
        summary.get("successes", 0) / episodes,
        interface_summary.get("handoffs", 0) / interface_episodes,
        float("-inf") if position_error is None else -position_error,
        float("-inf") if mean_return is None else mean_return,
    )


def curriculum_cases(kind, *, validation=False):
    table = CURRICULUM_VALIDATION if validation else CURRICULUM_STAGES
    return table[kind]


def run_cases(policy, cases, *, global_horizon, log, version_prefix=""):
    rows = []
    for stage_name, curriculum, seeds, stage_horizon in cases:
        horizon = _stage_horizon(global_horizon, stage_horizon)
        for seed in seeds:
            version = f"{version_prefix}{stage_name}:{getattr(policy, 'version', 'heuristic')}"
            rows.append(rollout(policy, seed, horizon=horizon, log=log,
                                curriculum=curriculum, stage=stage_name,
                                version=version))
    return rows


def train(args):
    out = args.output
    out.mkdir(parents=True, exist_ok=False)  # Never silently mix/overwrite runs.
    rng = random.Random(args.seed)
    incumbent = Policy()  # Exact expert warm start: zero guidance residual.
    mean, sigma = incumbent.weights[:], [.08] * SIZE
    history = []
    for generation in range(args.generations):
        candidates = [incumbent] + [Policy([rng.gauss(m, s) for m, s in zip(mean, sigma)])
                                   for _ in range(args.population - 1)]
        scored = []
        cases = curriculum_cases(args.curriculum)
        for index, candidate in enumerate(candidates):
            rows = run_cases(candidate, cases, global_horizon=args.horizon,
                             log=out / "train.jsonl",
                             version_prefix=f"g{generation}:c{index}:")
            score = candidate_score(rows)
            scored.append((score, candidate))
            history.append({"generation": generation, "candidate": index,
                            "version": candidate.version,
                            "selection_score": score, **aggregate(rows)})
        scored.sort(key=lambda pair: pair[0], reverse=True)
        # Stable sort deliberately retains deterministic teacher when all candidates tie.
        incumbent = scored[0][1]
        elites = [p.weights for _, p in scored[:max(2, args.population // 3)]]
        mean = [statistics.mean(ws) for ws in zip(*elites)]
        sigma = [max(.01, statistics.pstdev(ws)) for ws in zip(*elites)]
        incumbent.save(out / f"checkpoint-{generation:03d}.json", {"generation": generation})
        write_json(out / f"optimizer-{generation:03d}.json", {
            "generation": generation, "mean": mean, "sigma": sigma,
            "rng_state": rng.getstate(), "seed": args.seed, "train_seeds": TRAIN_SEEDS})
    # Validation is diagnostic, never used to tune physics or select evaluation seeds.
    validation_cases = curriculum_cases(args.curriculum, validation=True)
    validation = run_cases(incumbent, validation_cases, global_horizon=args.horizon,
                           log=out / "validation.jsonl", version_prefix="validation:")
    baseline_policy = Policy()
    baseline_validation = run_cases(baseline_policy, validation_cases,
                                    global_horizon=args.horizon,
                                    log=out / "baseline-validation.jsonl",
                                    version_prefix="baseline:")
    promotion = promotion_report(incumbent, history, validation, baseline_validation)
    status = promotion["status"]
    metadata = {"algorithm": "seeded CEM", "seed": args.seed, "train_seeds": TRAIN_SEEDS,
                "validation_seeds": VALIDATION_SEEDS, "curriculum": args.curriculum,
                "curriculum_cases": curriculum_cases(args.curriculum),
                "validation_curriculum_cases": validation_cases, "status": status,
                "deployable": promotion["deployable"], "promotion": promotion,
                "generations": args.generations, "population": args.population}
    incumbent.save(out / "policy.json", metadata)
    incumbent.save(out / "policy-candidate.json", metadata)
    report = {**metadata, "training": history, "validation": aggregate(validation),
              "baseline_validation": aggregate(baseline_validation)}
    write_json(out / "training-report.json", report)
    print(json.dumps({"policy": str(out / "policy.json"), "status": status,
                      "deployable": promotion["deployable"],
                      "blockers": promotion["blockers"],
                      "validation": report["validation"]}))


def evaluate(args):
    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    try:
        learned = Policy.load(args.policy, require_deployable=not args.allow_unproven_policy)
    except ValueError as exc:
        raise SystemExit(f"refusing unproven policy artifact: {exc}; pass --allow-unproven-policy for diagnostics") from None
    # Fixed high-level target expressed as residual request; same safety projection.
    # This is a constrained heuristic, not an unsafe open-loop flight controller.
    def fixed(obs):
        return [(25 - obs[24] * 45) / 2, (30 - obs[25] * 80) / 5]
    policies = {"deterministic": Policy(), "fixed-target-projected": fixed, "learned": learned}
    rows, grouped = [], {}
    cases = [("unseen", BASE, EVALUATION_SEEDS, None, True),
             ("heldout-scenario", HELDOUT, HELDOUT_SEEDS, None, True),
             ("nominal", BASE, (0,), None, False)]
    cases += [(stress, BASE, (3001,), stress, False)
              for stress in ("density-low", "lag-slow", "mass-high")]
    for case, scenario, seeds, stress, randomized in cases:
        for name, policy in policies.items():
            selected = []
            for seed in seeds:
                row = rollout(policy, seed, scenario=scenario, horizon=args.horizon,
                              stress=stress, randomized=randomized,
                              log=out / f"{case}-{name}.jsonl",
                              version=f"{name}:{getattr(policy, 'version', 'v1')}")
                row.update({"case": case, "baseline": name})
                rows.append(row)
                selected.append(row)
            grouped[f"{case}/{name}"] = aggregate(selected)
    report = {"policy": str(args.policy), "groups": grouped, "episodes": rows,
              "total": aggregate(rows), "heldout_seeds": HELDOUT_SEEDS,
              "evaluation_seeds": EVALUATION_SEEDS}
    write_json(out / "evaluation-report.json", report)
    print(json.dumps({"report": str(out / "evaluation-report.json"), "total": report["total"]}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("train", "evaluate"))
    parser.add_argument("--simulator-only", action="store_true", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--policy", type=Path)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--generations", type=int, default=2)
    parser.add_argument("--population", type=int, default=4)
    parser.add_argument("--horizon", type=int, default=3600)
    parser.add_argument("--curriculum", choices=tuple(CURRICULUM_STAGES), default="mixed",
                        help="training distribution; use full for legacy 86 km-only rollouts")
    parser.add_argument("--allow-unproven-policy", action="store_true",
                        help="evaluate a rejected/smoke-only policy artifact for diagnostics")
    args = parser.parse_args()
    if args.generations < 1 or args.population < 2 or args.horizon < 1:
        parser.error("positive generations/horizon and population >= 2 required")
    if args.mode == "evaluate" and args.policy is None:
        parser.error("evaluate requires --policy")
    (train if args.mode == "train" else evaluate)(args)


if __name__ == "__main__":
    main()
