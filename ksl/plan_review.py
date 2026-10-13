"""Conservative validity checks for a plan held for explicit operator review."""
from __future__ import annotations

import math
from typing import Any


def invalid_reason(plan: dict[str, Any], reference: dict[str, Any], current: dict[str, Any],
                   minimum_lead: float) -> str | None:
    if current.get("scene") != "flight":
        return "The flight scene changed; reconnect and replan"
    for key in ("vesselId", "body"):
        if reference.get(key) is None or current.get(key) != reference[key]:
            return "The active vessel or celestial body changed; reconnect and replan"
    limits = {"apoapsis": 100.0, "periapsis": 100.0, "inclination": 0.05,
              "semiMajorAxis": 100.0, "eccentricity": 0.0001,
              "longitudeOfAscendingNode": 0.001, "argumentOfPeriapsis": 0.001,
              "mass": max(1.0, float(reference.get("mass") or 0) * 0.001)}
    for key, tolerance in limits.items():
        a, b = reference.get(key), current.get(key)
        if not isinstance(a, (int, float)) or not isinstance(b, (int, float)) or not math.isfinite(a + b):
            return f"Cannot verify {key}; refresh and replan"
        delta = abs(b - a)
        # The periapsis direction is undefined for a circular orbit, and the
        # ascending node is undefined in an equatorial plane. Harmless angle
        # noise there must not invalidate a physically unchanged orbit.
        if key == "argumentOfPeriapsis" and reference.get("eccentricity", 1) < 0.0001:
            continue
        if key == "longitudeOfAscendingNode" and abs(math.sin(math.radians(reference.get("inclination", 90)))) < 0.001:
            continue
        if key in ("longitudeOfAscendingNode", "argumentOfPeriapsis"):
            delta = abs((b - a + math.pi) % (2 * math.pi) - math.pi)
        if delta > tolerance:
            return f"Planning state changed ({key}); replan before execution"
    ut, burn = current.get("ut"), plan.get("burnUT")
    if not isinstance(ut, (int, float)) or not isinstance(burn, (int, float)) or not math.isfinite(ut + burn):
        return "Burn timing unavailable; replan"
    if ut < reference.get("ut", ut) - 1:
        return "The game clock moved backwards; reconnect and replan"
    if burn - ut < minimum_lead:
        return "The reviewed burn window is too close or has passed; replan"
    atmosphere = current.get("atmosphereDepth")
    if atmosphere is None or min(current["apoapsis"], current["periapsis"]) <= atmosphere:
        return "The orbit now intersects the atmosphere; select entry takeover"
    return None
