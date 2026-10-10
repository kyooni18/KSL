#!/usr/bin/env python3
"""Observe the new L5 save and record physics without changing live KSP."""
import hashlib
import json
import os
from pathlib import Path
import re
import time

import krpc


def main():
    directory = Path(__file__).resolve().parent
    launch = json.loads((directory / "launch.json").read_text())
    initial = json.loads((directory / "checkpoint-physics.json").read_text())
    name = launch["mm305CheckpointName"]
    save = Path(initial["savePath"]).parent / (name + ".sfs")
    output = directory / "mm305-final.physics.json"
    if output.exists():
        raise RuntimeError("Profile already exists; refusing to overwrite evidence")
    while not save.exists():
        os.kill(launch["pid"], 0)
        time.sleep(5)
    # Wait for the newly created save to finish being written.
    first = save.read_bytes()
    time.sleep(1)
    data = save.read_bytes()
    if data != first:
        raise RuntimeError("Checkpoint still changing; rerun after save completes")
    match = re.search(rb"(?m)^\s*UT = ([0-9.eE+\-]+)\s*$", data)
    if not match:
        raise RuntimeError("Checkpoint universal time missing")
    saved_ut = float(match.group(1))
    connection = krpc.connect(name="STS MM305 checkpoint read-only observer")
    try:
        sc = connection.space_center
        vessel = sc.active_vessel
        flight = vessel.flight()
        observation = dict(ut=float(sc.ut), vessel=vessel.name,
                           mass=float(vessel.mass), dryMass=float(vessel.dry_mass),
                           partCount=len(vessel.parts.all), thrust=float(vessel.thrust),
                           meanAltitude=float(flight.mean_altitude),
                           situation=str(vessel.situation))
    finally:
        connection.close()
    if observation["vessel"] != "STS" or observation["partCount"] != initial["partCount"]:
        raise RuntimeError("Unexpected vessel or damaged checkpoint")
    if abs(observation["dryMass"] - initial["dryMass"]) > 1:
        raise RuntimeError("Checkpoint dry mass does not match intact STS")
    if abs(observation["thrust"]) > 1 or not 0 <= observation["ut"] - saved_ut < 15:
        raise RuntimeError("Observation not a timely unpowered checkpoint crosscheck")
    profile = dict(saveName=name, savePath=str(save),
                   saveSHA256=hashlib.sha256(data).hexdigest(), vessel="STS",
                   mass=observation["mass"], dryMass=observation["dryMass"],
                   partCount=observation["partCount"], savedUT=saved_ut,
                   liveCrosscheck=observation,
                   evidence="Read-only intact unpowered STS observation within 15 s of new MM305 ownership save; reload must verify coherent unpacked physics and normal route/Final gates.")
    with output.open("x") as handle:
        json.dump(profile, handle, indent=2)
        handle.write("\n")
    print(json.dumps(profile), flush=True)


if __name__ == "__main__":
    main()
