from __future__ import annotations

import json
from pathlib import Path

import mlx.core as mx

from model import FlyBrainV2, rollout, sample_scenario

ROOT = Path(__file__).resolve().parent
WEIGHTS = ROOT / "brain_v2.safetensors"


def summarize(metrics):
    stable = (metrics["max_rate"] < 80.0) & (metrics["max_error"] < 120.0)
    return {
        "stable_fraction": float(mx.mean(stable.astype(mx.float32)).item()),
        "mean_rms_error_deg": float(mx.mean(metrics["rms_error"]).item()),
        "mean_final_error_deg": float(mx.mean(metrics["final_error"]).item()),
        "worst_final_error_deg": float(mx.max(metrics["final_error"]).item()),
        "worst_rate_deg_s": float(mx.max(metrics["max_rate"]).item()),
        "mean_activity_per_s": float(mx.mean(metrics["activity"]).item()),
        "saturation_fraction": float(mx.mean(metrics["saturation"]).item()),
    }


def run_case(model, seed: int, hard: bool, batch: int, horizon: int):
    mx.random.seed(seed)
    scenario = sample_scenario(batch, horizon, hard=hard)
    base_loss, base_metrics = rollout(None, scenario, collect_loss=True)
    brain_loss, brain_metrics = rollout(model, scenario, collect_loss=True)
    mx.eval(base_loss, brain_loss, base_metrics, brain_metrics)
    return (
        float(base_loss.item()),
        summarize(base_metrics),
        float(brain_loss.item()),
        summarize(brain_metrics),
    )


def main() -> None:
    if not WEIGHTS.exists():
        raise SystemExit("brain_v2.safetensors missing; run `make train` first")

    model = FlyBrainV2()
    model.load_weights(str(WEIGHTS))

    normal = run_case(model, 0xC001D00D, False, 2400, 180)
    hard = run_case(model, 0xD15EA5E, True, 2400, 200)

    result = {
        "parameters": model.parameter_count(),
        "normal": {"baseline_loss": normal[0], "baseline": normal[1], "brain_loss": normal[2], "brain": normal[3]},
        "hard": {"baseline_loss": hard[0], "baseline": hard[1], "brain_loss": hard[2], "brain": hard[3]},
    }
    print(json.dumps(result, indent=2))

    n_base, n, n_brain_loss, nb = normal
    h_base, h, h_brain_loss, hb = hard
    normal_ok = (
        nb["stable_fraction"] >= n["stable_fraction"]
        and n_brain_loss < n_base
        and nb["mean_final_error_deg"] < n["mean_final_error_deg"]
        and nb["worst_rate_deg_s"] <= n["worst_rate_deg_s"] * 1.08
    )
    hard_ok = (
        hb["stable_fraction"] >= h["stable_fraction"]
        and h_brain_loss < h_base
        and hb["mean_final_error_deg"] < h["mean_final_error_deg"]
        and hb["worst_rate_deg_s"] <= h["worst_rate_deg_s"] * 1.08
    )
    if not (normal_ok and hard_ok):
        raise SystemExit(f"FlyBrain V2 validation FAILED normal={normal_ok} hard={hard_ok}")
    print("FlyBrain V2 validation PASSED")


if __name__ == "__main__":
    main()
