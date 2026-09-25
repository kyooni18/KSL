#!/usr/bin/env python3
"""Exercise the disconnected native protocol without leaving child processes behind."""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Tools"))
from clanding_layout import build_artifact as clanding_build_artifact


def main() -> int:
    backend = Path(sys.argv[1]) if len(sys.argv) > 1 else clanding_build_artifact(ROOT)
    expected = json.loads((ROOT / "Configuration/default.json").read_text())
    changed = json.loads(json.dumps(expected))
    changed["guidance"]["entryRollRate"] = 999
    requests = [
        {"id": "config", "method": "getConfiguration"},
        {"id": "normalize", "method": "updateConfiguration", "configuration": changed},
        {"id": "shutdown", "method": "shutdown"},
    ]
    result = subprocess.run(
        [str(backend)], input="".join(json.dumps(r) + "\n" for r in requests),
        text=True, capture_output=True, cwd=ROOT, timeout=15, check=True,
    )
    messages = [json.loads(line) for line in result.stdout.splitlines() if line.strip()]
    ready = next(m for m in messages if m.get("type") == "ready")
    assert ready["protocolVersion"] == 1
    assert ready["snapshot"]["connectionStatus"] == "disconnected"
    connection = ready["configuration"]["connection"]
    for key in ("serialPort", "baudRate", "timeoutMs", "rpcHost", "rpcPort"):
        assert connection[key] == expected["connection"][key], (key, connection)
    assert "streamPort" not in connection  # C-Nano supports synchronous RPC, not streams.
    assert set(ready["configuration"]) == set(expected)
    assert ready["configuration"]["site"]["name"] == expected["site"]["name"]
    assert isinstance(ready["configuration"]["site"]["allowReciprocalRunway"], bool)
    responses = {m["id"]: m for m in messages if m.get("type") == "response"}
    for request in requests:
        assert responses[request["id"]]["ok"] is True, responses[request["id"]]
    config = responses["config"]["result"]["configuration"]
    assert config == ready["configuration"]
    normalized = responses["normalize"]["result"]["configuration"]
    assert normalized["guidance"]["entryRollRate"] == changed["guidance"]["entryRollRate"]
    assert normalized["connection"]["rpcPort"] == expected["connection"]["rpcPort"]
    print("LandingBackend native TCP/simulator UI protocol tests passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
