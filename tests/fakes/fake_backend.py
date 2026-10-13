#!/usr/bin/env python3
"""Minimal stand-in for CLanding's landing_backend NDJSON protocol (tests only).

Scripted flight: after engageReentry it publishes TAEM -> Final Approach ->
Flare -> Touchdown -> Rollout -> Complete. FAKE_BACKEND_MODE=fault ends in
Fault instead; FAKE_BACKEND_MODE=hold stays in TAEM until abort.
"""
import json
import os
import sys
import time

MODE = os.environ.get("FAKE_BACKEND_MODE", "land")
state = {"phase": "Idle", "connectionStatus": "disconnected", "automationEngaged": False}
alt = 20000.0


def snap(**extra):
    s = dict(state, telemetry={"ut": 100.0 + (20000.0 - alt) / 50, "meanAltitude": alt, "trueAirSpeed": 300.0,
                               "rangeToSite": alt * 3}, statusMessage=f"fake {state['phase']}",
             warningMessage=None, lastError=None, plan=None, command={"targetAoA": 10.0})
    s.update(extra)
    print(json.dumps({"type": "snapshot", "snapshot": s}), flush=True)


def respond(rid, ok=True, error=None):
    msg = {"type": "response", "id": rid, "ok": ok}
    msg.update({"result": {}} if ok else {"error": error})
    print(json.dumps(msg), flush=True)


print(json.dumps({"type": "ready", "protocolVersion": 1, "configuration": {}, "snapshot": state}), flush=True)
for line in sys.stdin:
    req = json.loads(line)
    method, rid = req["method"], req.get("id")
    if os.environ.get("FAKE_BACKEND_REQUESTS"):
        with open(os.environ["FAKE_BACKEND_REQUESTS"], "a") as handle:
            handle.write(method + "\n")
    if method == "connect":
        state["connectionStatus"] = "connected"
        snap()
        respond(rid)
    elif method == "createPlan":
        # A valid physical candidate may fail every forecast/capture gate.
        # Script the actual backend plan response for end-to-end controller tests.
        state.update(phase="Idle", automationEngaged=False)
        plan = {"burnUT": 5000.0, "deltaV": 42.0,
                "executionQualified": MODE != "unqualified-entry",
                "targetCaptureAchieved": MODE not in ("unqualified-entry", "unqualified-route"),
                "terminalInletEnergyMargin": -246415.0,
                "robustnessScenarios": 15, "robustnessPassed": 0}
        snap(plan=plan)
        respond(rid)
    elif method in ("engageReentry", "engage"):
        state.update(phase="TAEM" if method == "engageReentry" else "Coast",
                     automationEngaged=True)
        snap()
        respond(rid)
        script = (["Coast", "Deorbit Burn", "Entry"] if method == "engage" else []) + [
                  "TAEM", "Final Approach", "Flare", "Touchdown", "Rollout",
                  "Fault" if MODE == "fault" else "Complete"]
        if MODE == "hold":
            script = ["TAEM"] * 400
        for phase in script:
            state["phase"] = phase
            alt = max(0.0, alt - 4000.0)
            snap(lastError="fake actuator fault" if phase == "Fault" else None)
            time.sleep(0.05)
            if MODE == "hold":
                import select
                if select.select([sys.stdin], [], [], 0)[0]:
                    break
    elif method == "abort":
        state.update(phase="Abort", automationEngaged=False)
        snap()
        respond(rid)
    elif method == "shutdown":
        respond(rid)
        break
    else:
        respond(rid)  # updateConfiguration, saveCheckpoint, ...
