"""Independent validation of the native immutable HAC commitment record."""
from __future__ import annotations

import json
import math
import re


def energy_law_runway_end(text: str) -> int | None:
    """Read both archived and timestamped native runway commitments (energy or HAC law)."""
    ends = re.findall(
        r"^MM305 (?:energy-law|HAC) runway end: (?:UT \S+ )?RW\S+ end=([01])\b",
        text, re.MULTILINE,
    )
    return int(ends[-1]) if ends else None

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



def validate_runway_geometry(scenario_path, config):
    """Reject a plant runway displaced from the configured landing segment.

    Defaults match scenario_seed_86km; the offline world is Kerbin (600 km).
    Reversing both threshold and heading describes the same physical runway.
    """
    fields = dict(runway_latitude_deg=-0.0486, runway_longitude_deg=-74.7240,
                  runway_elevation_m=70.0, runway_heading_deg=90.0,
                  runway_length_m=2500.0, runway_width_m=70.0)
    with open(scenario_path, encoding="utf-8") as source:
        for line in source:
            key, sep, value = line.partition("=")
            if sep and key.strip() in fields:
                fields[key.strip()] = float(value.split("#")[0].strip())
    site = config["site"]

    def endpoints(lat, lon, heading, length):
        lat, lon, heading = map(math.radians, (lat, lon, heading))
        arc = length / 600000.0
        end_lat = math.asin(math.sin(lat)*math.cos(arc) +
                            math.cos(lat)*math.sin(arc)*math.cos(heading))
        end_lon = lon + math.atan2(math.sin(heading)*math.sin(arc)*math.cos(lat),
                                  math.cos(arc)-math.sin(lat)*math.sin(end_lat))
        def xyz(a, b):
            return tuple(600000.0*x for x in
                         (math.cos(a)*math.cos(b), math.cos(a)*math.sin(b), math.sin(a)))
        return xyz(lat, lon), xyz(end_lat, end_lon)

    actual = endpoints(fields["runway_latitude_deg"], fields["runway_longitude_deg"],
                       fields["runway_heading_deg"], fields["runway_length_m"])
    expected = endpoints(site["latitude"], site["longitude"],
                         site["runwayHeading"], site["runwayLength"])
    error = min(max(math.dist(a, b) for a, b in zip(actual, order))
                for order in (expected, expected[::-1]))
    # Allow rounded survey coordinates/elevation, never a different runway end.
    if (not math.isfinite(error) or error > 5.0 or
            abs(fields["runway_elevation_m"]-site["altitude"]) > 5.0 or
            abs(fields["runway_width_m"]-site["runwayWidth"]) > 1.0):
        raise ValueError(f"scenario/configuration runway mismatch: endpoint error {error:.1f} m; "
                         "use the same physical runway threshold, heading and dimensions")
