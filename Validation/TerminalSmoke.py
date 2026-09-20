#!/usr/bin/env python3
"""Deterministic diagnostic replay using the existing native expert/physics ABI.

No RL observation or residual projection participates. This measures the
production expert commands; terminal-corridor initialization does not imply a
valid MM304 handoff. Reports phase coverage and retains every telemetry sample.
This diagnostic cannot certify the established live 50 km checkpoint.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ShuttleSim/rl"))
from environment import Environment, touchdown_outcome


def run(seed, output, horizon):
    env = Environment(simulator_only=True, curriculum="terminal-corridor", horizon=horizon)
    phases = set()
    result = {"seed": seed, "success": False, "touchdown": False, "rollout_valid": False}
    try:
        env.reset(seed)
        with (output / f"terminal-{seed}.jsonl").open("x") as stream:
            for elapsed in range(horizon + 1):
                t, e = env.telemetry, env.expert
                phases.add(e["phase"])
                summary = json.loads(env.lib.offline_summary(env.handle))
                stream.write(json.dumps({"elapsed": elapsed, "telemetry": t,
                                         "expert": e, "metrics": summary}) + "\n")
                unsafe = e["abort"] or not e["authority_valid"] or not e["authority_survivable"]
                success, miss = touchdown_outcome(summary, t, e, unsafe)
                outcome = ("unsafe_or_abort" if unsafe else "success" if success else
                           "runway_miss" if miss else "timeout" if elapsed == horizon else None)
                if outcome:
                    result.update(outcome=outcome, success=success, rollout_valid=success,
                                  touchdown=summary["touchdown"], elapsed=elapsed,
                                  final_phase=e["phase"], final_altitude_m=t["position"]["altitude_m"],
                                  final_air_speed_mps=t["velocity"]["air_mps"],
                                  final_along_m=t["runway"]["along_m"],
                                  final_cross_m=t["runway"]["cross_m"])
                    break
                # offline_step count is the existing ABI's 50 x .02 s control interval.
                if not env.lib.offline_step(env.handle, e["aoa"], e["bank"], e["gear"], e["brakes"], 50):
                    raise RuntimeError("physics step rejected")
                env.elapsed = elapsed + 1
                env._sample()
    except Exception as exc:
        result.update(outcome="harness_error", error=f"{type(exc).__name__}: {exc}")
    finally:
        env.close()
    result["phases"] = sorted(phases)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3])
    parser.add_argument("--horizon", type=int, default=720)
    args = parser.parse_args()
    if args.horizon <= 0:
        parser.error("horizon must be positive")
    args.output.mkdir(parents=True, exist_ok=False)
    results = [run(seed, args.output, args.horizon) for seed in args.seeds]
    (args.output / "summary.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results, indent=2))
    return 0 if all(row["success"] for row in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
