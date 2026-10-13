"""Python kRPC connections for the ascent side and mission bookkeeping.

The native backend owns its C-Nano landing control connection. Python probes
only read vessel identity and planning state when a deorbit plan is held for
operator review. Both use the mission profile's ``connection`` block (and
``KSL_KRPC_*`` for child processes).
"""

from __future__ import annotations

import time
import threading
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
    timeout = float(c["timeoutSeconds"])
    finished = threading.Event()
    lock = threading.Lock()
    result: dict[str, Any] = {}

    def open_connection() -> None:
        try:
            value = krpc.connect(name=name, address=str(c["address"]), rpc_port=int(c["rpcPort"]),
                                 stream_port=int(c["streamPort"]))
        except Exception as exc:
            value = exc
        with lock:
            if result.get("abandoned"):
                if not isinstance(value, Exception):
                    value.close()
                return
            result["value"] = value
            finished.set()

    threading.Thread(target=open_connection, name=f"krpc-connect-{name}", daemon=True).start()
    if not finished.wait(timeout):
        with lock:
            result["abandoned"] = True
            # A reply arriving exactly at the deadline also belongs to the
            # timed-out attempt. Do not leave its connection unowned.
            late = result.pop("value", None)
        if late is not None and not isinstance(late, Exception):
            late.close()
        raise ConnectionFailure(f"kRPC handshake at {c['address']}:{c['rpcPort']} timed out after {timeout:.1f} s")
    value = result["value"]
    if isinstance(value, Exception):
        raise ConnectionFailure(f"cannot reach kRPC at {c['address']}:{c['rpcPort']}: {value}") from value
    conn = value
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
                                "paused": bool(conn.krpc.paused), "observedAt": time.time(),
                                "connection": dict(config["connection"])}
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
                "vesselId": hash(vessel), "atmosphereDepth": orbit.body.atmosphere_depth,
                "launchClamps": len(vessel.parts.launch_clamps),
                "eccentricity": orbit.eccentricity, "semiMajorAxis": orbit.semi_major_axis,
                "longitudeOfAscendingNode": orbit.longitude_of_ascending_node,
                "argumentOfPeriapsis": orbit.argument_of_periapsis,
                "verticalSpeed": flight.vertical_speed, "dynamicPressure": flight.dynamic_pressure,
            })
        info["observedAt"] = time.time()
        return info


def suggest_phase(info: dict[str, Any]) -> str | None:
    """Best-effort entry-phase suggestion from a probe (operator still decides)."""
    if info.get("scene") != "flight":
        return None
    situation = info.get("situation", "")
    if situation in ("pre_launch", "prelaunch"):
        return "prelaunch"
    if situation == "landed" and info.get("launchClamps", 0) >= 2 and info.get("speed", 99) < 1:
        return "prelaunch"
    if situation == "orbiting" and info.get("periapsis", 0) > info.get("atmosphereDepth", 70000):
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
