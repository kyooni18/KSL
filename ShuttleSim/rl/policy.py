"""Portable, deterministic feature-sized residual policy; standard library only."""
import hashlib
import json
import math
from pathlib import Path

from environment import FEATURES, SCHEMA

SIZE = 2 * (len(FEATURES) + 1)


class Policy:
    def __init__(self, weights=None):
        self.weights = [0.0] * SIZE if weights is None else list(weights)
        if len(self.weights) != SIZE or not all(math.isfinite(w) for w in self.weights):
            raise ValueError("invalid policy weights")

    @property
    def version(self):
        return hashlib.sha256(json.dumps(self.weights).encode()).hexdigest()[:16]

    def __call__(self, observation):
        if len(observation) != len(FEATURES) or not all(map(math.isfinite, observation)):
            return [float("nan")] * 2  # Environment falls back to deterministic guidance.
        x = [1.0, *observation]
        return [math.tanh(sum(w * value for w, value in zip(
            self.weights[i * len(x):(i + 1) * len(x)], x))) for i in range(2)]

    def save(self, path, metadata=None):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({
            "format": "shuttlesim-residual-linear-tanh-v1", "schema": SCHEMA,
            "features": list(FEATURES), "weights": self.weights, "version": self.version,
            "simulator_only": True, "metadata": metadata or {},
        }, indent=2, allow_nan=False) + "\n")

    @classmethod
    def load(cls, path, *, require_deployable=False):
        data = json.loads(Path(path).read_text())
        if (data.get("format") != "shuttlesim-residual-linear-tanh-v1"
                or data.get("schema") != SCHEMA or data.get("simulator_only") is not True
                or data.get("features") != list(FEATURES)):
            raise ValueError("incompatible simulator-only policy artifact")
        policy = cls(data["weights"])
        if policy.version != data["version"]:
            raise ValueError("policy checksum mismatch")
        metadata = data.get("metadata") if isinstance(data.get("metadata"), dict) else {}
        if require_deployable and metadata.get("deployable") is not True:
            status = metadata.get("status", "unknown")
            raise ValueError(f"policy artifact is not deployable: {status}")
        return policy
