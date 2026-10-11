"""Python kRPC connections for the ascent side and mission bookkeeping.

The landing path does not use this: the native backend owns its own C-Nano
kRPC connection. Both are pointed at the same server through the mission
profile's ``connection`` block (and ``KSL_KRPC_*`` for child processes).
"""

from __future__ import annotations

import time
from contextlib import contextmanager
from typing import Any, Iterator

from .config import MissionConfig


class ConnectionFailure(RuntimeError):
    pass


def krpc_env(config: MissionConfig) -> dict[str, str]:
    c = config["connection"]
    return {"KSL_KRPC_ADDRESS": str(c["address"]), "KSL_KRPC_RPC_PORT": str(int(c["rpcPort"])),
            "KSL_KRPC_STREAM_PORT": str(int(c["streamPort"]))}


def endpoint_args(config: MissionConfig) -> list[str]:
    c = config["connection"]
    return ["--address", str(c["address"]), "--rpc-port", str(int(c["rpcPort"])),
            "--stream-port", str(int(c["streamPort"]))]


@contextmanager
def connect(config: MissionConfig, name: str) -> Iterator[Any]:
    try:
        import krpc
    except ImportError as exc:
        raise ConnectionFailure("the krpc Python package is not installed (run `make setup`)") from exc
    c = config["connection"]
    try:
        conn = krpc.connect(name=name, address=str(c["address"]), rpc_port=int(c["rpcPort"]),
                            stream_port=int(c["streamPort"]))
    except Exception as exc:  # krpc raises ConnectionRefusedError, socket errors, protobuf errors
        raise ConnectionFailure(f"cannot reach kRPC at {c['address']}:{c['rpcPort']}: {exc}") from exc
    try:
        yield conn
    finally:
        try:
            conn.close()
        except Exception:
            pass


def probe(config: MissionConfig) -> dict[str, Any]:
    """Read-only snapshot of the game state (no control writes)."""
    with connect(config, "ksl-probe") as conn:
        info: dict[str, Any] = {"scene": str(conn.krpc.current_game_scene).split(".")[-1],
                                "paused": bool(conn.krpc.paused)}
        if info["scene"] == "flight":
            sc = conn.space_center
            vessel = sc.active_vessel
            orbit = vessel.orbit
            flight = vessel.flight(orbit.body.reference_frame)
            info.update({
                "ut": sc.ut, "vessel": vessel.name, "situation": str(vessel.situation).split(".")[-1],
                "body": orbit.body.name, "altitude": flight.mean_altitude, "speed": flight.speed,
                "apoapsis": orbit.apoapsis_altitude, "periapsis": orbit.periapsis_altitude,
                "inclination": orbit.inclination * 57.29577951308232, "mass": vessel.mass,
            })
        return info


def suggest_phase(info: dict[str, Any]) -> str | None:
    """Best-effort entry-phase suggestion from a probe (operator still decides)."""
    if info.get("scene") != "flight":
        return None
    situation = info.get("situation", "")
    if situation in ("pre_launch", "prelaunch"):
        return "prelaunch"
    if situation == "orbiting" and info.get("periapsis", 0) > 70000:
        return "orbit"
    if situation in ("sub_orbital", "flying") and info.get("altitude", 0) > 30000:
        return "mm304"
    if situation == "flying":
        return "mm305"
    return None


def wait_for_flight_scene(config: MissionConfig, timeout: float = 180.0) -> None:
    deadline = time.time() + timeout
    last_error: Exception | None = None
    while time.time() < deadline:
        try:
            if probe(config).get("scene") == "flight":
                return
        except ConnectionFailure as exc:
            last_error = exc
        time.sleep(2.0)
    raise ConnectionFailure(f"KSP did not reach the flight scene within {timeout:.0f} s"
                            + (f" ({last_error})" if last_error else ""))


def save_game(config: MissionConfig, name: str) -> None:
    with connect(config, "ksl-checkpoint") as conn:
        conn.space_center.save(name)


def load_game(config: MissionConfig, name: str) -> None:
    with connect(config, "ksl-checkpoint") as conn:
        conn.space_center.load(name)
