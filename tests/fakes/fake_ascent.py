#!/usr/bin/env python3
"""Stand-in for ksl.ascent.flight: prints the same @@ksl milestone lines (tests only)."""
import json
import sys
import time

for name in ("ignition", "liftoff", "gravity-turn", "booster-separation", "meco", "et-separation", "insertion"):
    print("@@ksl " + json.dumps({"event": "telemetry", "ut": 1.0, "altitude": 1000.0}), flush=True)
    print("@@ksl " + json.dumps({"event": name}), flush=True)
    print(f"human readable {name}", flush=True)
    time.sleep(0.01)
sys.exit(int(sys.argv[1]) if len(sys.argv) > 1 else 0)
