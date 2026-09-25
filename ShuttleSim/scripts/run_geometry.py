"""Independent validation of the native immutable HAC commitment record."""
from __future__ import annotations

import json
import math

def fixed_hac_geometry_evidence(text: str, final_distance: float | None = None) -> dict:
    """Validate the native committed circle; flight completion is checked separately."""
    frozen = None
    count = 0
    for line in text.splitlines():
        if not line.startswith("MM305_ROUTE "):
            continue
        try:
            record=json.loads(line.partition(" ")[2])
            radius=float(record["radius"])
            sweep=float(record["sweep"])
            side=float(record["side"])
            distance=float(record["finalDistance"] if final_distance is None else final_distance)
            points={key:tuple(map(float,record[key])) for key in ("entry","center","exit")}
            if any(len(point)!=2 for point in points.values()) or record["runwayEnd"] not in (0,1):
                raise ValueError("invalid runway point")
            points["runwayStation"]=(-distance,0.0)
            lead,arc=float(record["leadLength"]),float(record["arcLength"])
            if side not in (-1.0,1.0) or not 0.0<sweep*side<=math.tau:
                raise ValueError("invalid signed arc")
        except (KeyError,TypeError,ValueError,IndexError,OverflowError):
            return {"valid": False, "reason": "malformed-native-geometry"}
        values=tuple(x for key in ("runwayStation","center","exit","entry") for x in points[key])
        values+=(lead,arc,radius,sweep,side,distance,record["runwayEnd"])
        if not all(math.isfinite(x) for x in values) or radius<3000.0 or lead<0 or distance<=0:
            return {"valid":False,"reason":"nonfinite-or-out-of-domain"}
        tolerance=64*math.ulp(max(1.0,*(abs(x) for x in values)))
        dx,dy=points["exit"][0]-points["center"][0],points["exit"][1]-points["center"][1]
        expected=(points["center"][0]+dx*math.cos(sweep)+dy*math.sin(sweep),
                  points["center"][1]-dx*math.sin(sweep)+dy*math.cos(sweep))
        if (math.dist(points["exit"],(-distance,0))>tolerance or
                math.dist(points["center"],(-distance,side*radius))>tolerance or
                math.dist(points["entry"],expected)>tolerance or
                abs(arc-radius*abs(sweep))>tolerance):
            return {"valid":False,"reason":"not-runway-anchored-native-circle"}
        if frozen is not None and values!=frozen:
            return {"valid": False, "reason": "committed-geometry-changed"}
        frozen = values
        count += 1
    if frozen is None:
        return {"valid": False, "reason": "no-committed-geometry-record"}
    return {"valid": True, "radius": radius, "records": count,
            "runwayEnd":record["runwayEnd"],"side":side,"sweep":sweep,
            "center": points["center"], "entry": points["entry"],
            "exit": points["exit"], "leadLength": lead, "arcLength": arc}

