from __future__ import annotations

import json
import time
from pathlib import Path

import mlx.core as mx
import mlx.nn as nn
import mlx.optimizers as optim

from model import FlyBrainV2, rollout, sample_scenario

ROOT = Path(__file__).resolve().parent
WEIGHTS = ROOT / "brain_v2.safetensors"
REPORT = ROOT / "training_report.json"

BATCH = 160
HORIZON = 104
STEPS = 90
LEARNING_RATE = 2.5e-4
SEED = 0x5A17B2


def main() -> None:
    mx.random.seed(SEED)
    model = FlyBrainV2()
    optimizer = optim.Adam(learning_rate=LEARNING_RATE)

    def loss_fn():
        scenario = sample_scenario(BATCH, HORIZON, hard=False)
        loss, _ = rollout(model, scenario, collect_loss=True)
        return loss

    loss_and_grad = nn.value_and_grad(model, loss_fn)
    started = time.perf_counter()
    best_loss = float("inf")
    best_path = ROOT / ".brain_v2_best.safetensors"

    print(
        f"FlyBrain V2: params={model.parameter_count():,} batch={BATCH} "
        f"horizon={HORIZON} steps={STEPS} device={mx.default_device()}"
    )

    for step in range(1, STEPS + 1):
        loss, grads = loss_and_grad()
        optimizer.update(model, grads)
        mx.eval(model.parameters(), optimizer.state, loss)
        value = float(loss.item())
        if value < best_loss:
            best_loss = value
            model.save_weights(str(best_path))
        if step == 1 or step % 10 == 0:
            print(f"step {step:03d}/{STEPS} loss={value:.5f} best={best_loss:.5f}")

    model.load_weights(str(best_path))
    model.save_weights(str(WEIGHTS))
    best_path.unlink(missing_ok=True)
    elapsed = time.perf_counter() - started

    # Fast post-train sanity set; the independent validator is intentionally larger.
    mx.random.seed(0x91A2D3)
    normal = sample_scenario(768, 140, hard=False)
    baseline_loss, baseline = rollout(None, normal, collect_loss=True)
    trained_loss, trained = rollout(model, normal, collect_loss=True)
    mx.eval(baseline_loss, trained_loss, baseline, trained)

    report = {
        "revision": "flybrain-v2-gru-residual-20260913-r1",
        "parameters": model.parameter_count(),
        "device": str(mx.default_device()),
        "training_seconds": elapsed,
        "steps": STEPS,
        "batch": BATCH,
        "horizon": HORIZON,
        "learning_rate": LEARNING_RATE,
        "best_training_loss": best_loss,
        "sanity_baseline_loss": float(baseline_loss.item()),
        "sanity_trained_loss": float(trained_loss.item()),
        "sanity_baseline_final_error": float(mx.mean(baseline["final_error"]).item()),
        "sanity_trained_final_error": float(mx.mean(trained["final_error"]).item()),
        "sanity_baseline_max_rate": float(mx.max(baseline["max_rate"]).item()),
        "sanity_trained_max_rate": float(mx.max(trained["max_rate"]).item()),
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
