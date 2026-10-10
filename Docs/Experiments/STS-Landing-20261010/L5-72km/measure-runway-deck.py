#!/usr/bin/env python3
"""Measure loaded runway colliders with read-only, downward kRPC raycasts."""
import json
import math
import os
from pathlib import Path
import time

import krpc


def main():
    directory = Path(__file__).resolve().parent
    launch = json.loads((directory / "launch.json").read_text())
    site = json.loads((directory / "configuration.json").read_text())["site"]
    output = directory / "runway-deck-raycasts.json"
    if output.exists():
        raise RuntimeError("Refusing to replace existing measurement")
    conn = krpc.connect(name="STS runway deck read-only measurement")
    try:
        sc = conn.space_center
        body = sc.bodies["Kerbin"]
        frame = body.reference_frame
        radius = body.equatorial_radius
        latitude = site["latitude"]
        while True:
            os.kill(launch["pid"], 0)
            vessel = sc.active_vessel
            flight = vessel.flight()
            delta = math.radians(flight.longitude - site["longitude"])
            separation = abs(delta) * radius * math.cos(math.radians(latitude))
            if separation < 12000 and flight.mean_altitude < 10000:
                rows = []
                for along in (0, 400, 1000, 1400, 2000, 2400):
                    longitude = site["longitude"] + math.degrees(
                        along / (radius * math.cos(math.radians(latitude))))
                    sea = body.position_at_altitude(latitude, longitude, 0, frame)
                    norm = math.sqrt(sum(x*x for x in sea))
                    down = tuple(-x/norm for x in sea)
                    hits = []
                    for height in (200, 400):
                        origin = body.position_at_altitude(latitude, longitude, height, frame)
                        distance = sc.raycast_distance(origin, down, frame)
                        part = sc.raycast_part(origin, down, frame)
                        hits.append(dict(originAltitude=height, distance=distance,
                                         colliderAltitude=height-distance,
                                         vesselPartHit=part is not None))
                    if all(math.isfinite(h["distance"]) and not h["vesselPartHit"] for h in hits):
                        consistent = abs(hits[0]["colliderAltitude"]-hits[1]["colliderAltitude"]) < .03
                        rows.append(dict(along=along, latitude=latitude, longitude=longitude,
                                         hits=hits, consistent=consistent))
                if len(rows) == 6 and all(row["consistent"] for row in rows):
                    result = dict(ut=float(sc.ut), vessel=vessel.name, samples=rows,
                                  configuredSiteAltitude=site["altitude"],
                                  method="Two vertical ray origins per point; static collider hits only, agreement <3 cm. Points use configured Runway09 centerline; terrain/deck identification requires shape and position review.")
                    with output.open("x") as handle:
                        json.dump(result, handle, indent=2, allow_nan=False)
                        handle.write("\n")
                    print(json.dumps(result), flush=True)
                    return
            time.sleep(10)
    finally:
        conn.close()


if __name__ == "__main__":
    main()
