#!/usr/bin/env python3
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Tools"))
from clanding_layout import build_artifact as clanding_build_artifact


def request(process: subprocess.Popen[str], request_id: str, method: str, **payload) -> None:
    assert process.stdin is not None
    process.stdin.write(json.dumps({"id": request_id, "method": method, **payload}) + "\n")
    process.stdin.flush()


def read_message(process: subprocess.Popen[str]) -> dict:
    assert process.stdout is not None
    line = process.stdout.readline()
    assert line, "LandingBackend closed stdout unexpectedly"
    return json.loads(line)


def read_response(process: subprocess.Popen[str], request_id: str) -> dict:
    while True:
        message = read_message(process)
        if message.get("type") == "response" and message.get("id") == request_id:
            return message


def main() -> int:
    backend = Path(sys.argv[1]) if len(sys.argv) > 1 else clanding_build_artifact(ROOT)
    process = subprocess.Popen(
        [str(backend)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        cwd=ROOT,
    )
    ready = read_message(process)
    assert ready["type"] == "ready"
    assert ready["protocolVersion"] == 1
    assert ready["configuration"]["connection"]["serialPort"] == "/dev/tty.krpc"
    assert ready["configuration"]["connection"]["baudRate"] == 921600
    assert ready["configuration"]["connection"]["timeoutMs"] == 2000
    assert "rpcPort" not in ready["configuration"]["connection"]
    assert "streamPort" not in ready["configuration"]["connection"]
    assert ready["snapshot"]["connectionStatus"] == "disconnected"
    expected_default = json.loads((ROOT / "Configuration" / "default.json").read_text())
    assert set(ready["configuration"].keys()) == set(expected_default.keys())
    assert ready["configuration"]["site"]["name"] == expected_default["site"]["name"]
    assert ready["configuration"]["guidance"]["targetEntryRange"] == 1030000
    assert isinstance(ready["configuration"]["site"]["allowReciprocalRunway"], bool)

    request(process, "config", "getConfiguration")
    config_response = read_response(process, "config")
    assert config_response["type"] == "response"
    assert config_response["id"] == "config"
    assert config_response["ok"] is True
    assert config_response["result"]["configuration"]["guidance"]["targetEntryRange"] == 1030000

    changed = json.loads(json.dumps(expected_default))
    changed["guidance"]["entryRollRate"] = 999
    request(process, "normalize", "updateConfiguration", configuration=changed)
    normalized = read_response(process, "normalize")
    assert normalized["ok"] is True
    assert normalized["result"]["configuration"]["guidance"]["entryRollRate"] == 999

    request(process, "shutdown", "shutdown")
    shutdown = read_response(process, "shutdown")
    assert shutdown["id"] == "shutdown"
    assert shutdown["ok"] is True
    process.wait(timeout=3)
    assert process.returncode == 0
    print("LandingBackend UI protocol tests passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
