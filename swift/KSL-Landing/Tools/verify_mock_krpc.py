#!/usr/bin/env python3
"""Independent TCP/protobuf fault fixture. Tests only; never a landing runtime.

Starts an ephemeral loopback server owned by this process, then invokes the
Swift executable against that port. It never connects to a KSP instance.
"""
import argparse
import json
import math
import pathlib
import signal
import socket
import struct
import subprocess
import tempfile
import threading
import time


def varint(n):
    result = bytearray()
    while n > 127:
        result.append((n & 127) | 128)
        n >>= 7
    result.append(n)
    return bytes(result)


def read_varint(data, offset=0):
    value = 0
    for i in range(10):
        b = data[offset]
        offset += 1
        value |= (b & 127) << (7*i)
        if b < 128:
            return value, offset
    raise ValueError("overlong varint")


def field(n, body):
    return varint(n*8+2) + varint(len(body)) + body


def fields(data):
    offset, result = 0, {}
    while offset < len(data):
        tag, offset = read_varint(data, offset)
        number, wire = tag >> 3, tag & 7
        if wire == 0:
            value, offset = read_varint(data, offset)
        elif wire == 2:
            size, offset = read_varint(data, offset)
            value = data[offset:offset+size]
            assert len(value) == size
            offset += size
        else:
            raise ValueError(f"unexpected wire {wire}")
        result.setdefault(number, []).append(value)
    return result


def exact(connection, count):
    chunks = bytearray()
    while len(chunks) < count:
        chunk = connection.recv(count-len(chunks))
        if not chunk:
            raise EOFError()
        chunks.extend(chunk)
    return bytes(chunks)


def packet(connection):
    prefix = bytearray()
    while True:
        b = exact(connection, 1)[0]
        prefix.append(b)
        if b < 128:
            break
        assert len(prefix) <= 5
    size, _ = read_varint(prefix)
    assert size <= 4*1024*1024
    return exact(connection, size)


def number(value):
    return struct.pack("<d", value)


def real(value):
    return struct.pack("<f", value)


def string(value):
    data = value.encode()
    return varint(len(data))+data


def vector(*values):
    return b"".join(field(1, number(v)) for v in values)


def rotation_quaternion(pitch):
    # kRPC vessel (right,forward,down) -> surface (up,north,east).
    c, s = math.cos(pitch), math.sin(pitch)
    matrix = [[0, s, -c], [-1, 0, 0], [0, c, s]]
    trace = sum(matrix[i][i] for i in range(3))
    scale = math.sqrt(1+trace)*2
    return vector((matrix[2][1]-matrix[1][2])/scale,
                  (matrix[0][2]-matrix[2][0])/scale,
                  (matrix[1][0]-matrix[0][1])/scale, scale/4)


class Fixture:
    def __init__(self, mode):
        self.mode, self.calls, self.error = mode, [], None
        self.ut, self.snapshots, self.command_batches = 1000.0, 0, 0
        self.command_seen, self.closed = threading.Event(), threading.Event()
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.listener.settimeout(5)
        self.port = self.listener.getsockname()[1]
        self.worker = threading.Thread(target=self.serve, daemon=True)
        self.worker.start()

    def send(self, connection, body):
        data = varint(len(body))+body
        if self.mode == "fragmented":
            for offset in range(0, len(data), 3):
                connection.sendall(data[offset:offset+3])
        else:
            connection.sendall(data)

    def value(self, name, args, telemetry, last_ut):
        objects = {"get_ActiveVessel": 1, "Vessel_get_Control": 11,
                   "Vessel_get_SurfaceReferenceFrame": 12, "Vessel_get_Orbit": 13,
                   "Vessel_get_Parts": 14, "Vessel_get_AutoPilot": 15,
                   "Orbit_get_Body": 21, "Vessel_Flight": 16,
                   "CelestialBody_get_ReferenceFrame": 22, "ReferenceFrame_static_CreateHybrid": 23}
        if name in objects:
            if name == "get_ActiveVessel" and telemetry and self.mode == "vessel-switch":
                return varint(2)
            return varint(objects[name])
        if name == "GetStatus":
            return field(1, b"0.6.0-independent-test-fixture")
        if name == "Vessel_get_Name":
            return string("Different Vessel" if self.mode == "wrong-vessel" else "STS")
        if name == "CelestialBody_get_Name":
            return string("Kerbin")
        if name == "CelestialBody_get_EquatorialRadius":
            return number(600000)
        if name == "CelestialBody_get_GravitationalParameter":
            return number(3.5316e12)
        if name == "Parts_get_All":
            count = 3 if telemetry and self.mode == "parts-change" else 2
            return b"".join(field(1, varint(101+i)) for i in range(count))
        if name == "get_UT":
            return number(self.ut+(1 if last_ut and self.mode == "snapshot-span" else 0))
        if name == "Vessel_Rotation":
            return rotation_quaternion(math.radians(-16))
        if name == "Vessel_get_Mass":
            return real(25000)
        if name == "Vessel_get_Situation":
            return varint(3*2)  # FLYING, zigzag enum
        if name == "Vessel_get_Thrust":
            return real(0)
        if name == "Control_get_Throttle":
            return real(0)
        if name.startswith("Control_get_") or name == "Control_GetActionGroup":
            return varint(0)
        if name.startswith("Flight_get_"):
            property_name = name[len("Flight_get_"):]
            doubles = {"Latitude": -0.04860436, "Longitude": -75.683263,
                       "MeanAltitude": 4268.44, "SurfaceAltitude": 4268.44}
            floats = {"DynamicPressure": 7500, "TrueAirSpeed": 150,
                      "SpeedOfSound": 330, "StaticPressure": 60000,
                      "StaticAirTemperature": 280}
            if property_name in doubles:
                return number(doubles[property_name])
            if property_name in floats:
                return real(floats[property_name])
            if property_name == "Velocity":
                return vector(-150*math.sin(math.radians(24)), 0, 150*math.cos(math.radians(24)))
            if property_name == "Lift":
                return vector(300000*math.cos(math.radians(24)), 0, 300000*math.sin(math.radians(24)))
            if property_name == "Drag":
                return vector(120000*math.sin(math.radians(24)), 0, -120000*math.cos(math.radians(24)))
        if name == "AutoPilot_set_Engaged":
            assert len(args) == 2 and read_varint(args[1])[0] == 0
            return b""
        if name.startswith("Control_set_") or name == "Control_SetActionGroup":
            assert read_varint(args[0])[0] == 11
            if name in {"Control_set_Throttle", "Control_set_Pitch", "Control_set_Roll", "Control_set_Yaw", "Control_set_WheelSteering"}:
                assert len(args[1]) == 4
                value = struct.unpack("<f", args[1])[0]
                assert -1 <= value <= 1
                if name == "Control_set_Throttle":
                    assert value == 0
            elif name == "Control_SetActionGroup":
                assert read_varint(args[1])[0] == 1 and args[2] == b"\x00"
            elif name in {"Control_set_SAS", "Control_set_RCS", "Control_set_Brakes"}:
                assert args[1] == b"\x00"
            elif name == "Control_set_Gear":
                assert args[1] in {b"\x00", b"\x01"}
            return b""
        raise AssertionError(f"Unexpected RPC procedure {name}")

    def serve(self):
        try:
            connection, _ = self.listener.accept()
            with connection:
                connection.settimeout(5)
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                handshake = fields(packet(connection))
                assert handshake[2][0] == b"KSL Swift Landing"
                self.send(connection, field(3, b"0123456789abcdef"))
                while True:
                    request = fields(packet(connection))
                    if self.mode == "malformed":
                        self.send(connection, b"\x00")
                        return
                    if self.mode == "truncated":
                        connection.sendall(varint(10)+b"abc")
                        return
                    calls = []
                    for raw in request.get(1, []):
                        call = fields(raw)
                        service = call[1][0].decode()
                        assert service in {"SpaceCenter", "KRPC"}
                        name = call[2][0].decode()
                        arguments = {}
                        for raw_arg in call.get(3, []):
                            arg = fields(raw_arg)
                            arguments[arg.get(1, [0])[0]] = arg[2][0]
                        calls.append((name, [arguments[i] for i in range(len(arguments))]))
                    telemetry = any(name == "Flight_get_Latitude" for name, _ in calls)
                    command = any(name == "Control_set_Pitch" for name, _ in calls)
                    if telemetry:
                        self.snapshots += 1
                        self.ut += .1
                        if self.mode == "slow":
                            time.sleep(.65)
                    if command:
                        self.command_batches += 1
                        self.command_seen.set()
                    self.calls.extend(name for name, _ in calls)
                    response = bytearray()
                    for index, (name, args) in enumerate(calls):
                        if self.mode == "command-error" and name == "Control_set_Pitch":
                            error = field(1, b"SpaceCenter")+field(2, b"TestError")+field(3, b"Injected command rejection")
                            response.extend(field(2, field(1, error)))
                        else:
                            value = self.value(name, args, telemetry, index == len(calls)-1)
                            response.extend(field(2, field(2, value)))
                    self.send(connection, response)
        except (EOFError, ConnectionResetError, BrokenPipeError):
            pass
        except Exception as error:
            self.error = repr(error)
        finally:
            self.listener.close()
            self.closed.set()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", default=".build/release/ksl-landing")
    parser.add_argument("--output", default="Validation/krpc-mock.json")
    args = parser.parse_args()
    results = []
    modes = ["inspect", "fragmented", "wrong-vessel", "vessel-switch", "parts-change",
             "snapshot-span", "slow", "malformed", "truncated", "fly", "command-error", "cancel"]
    with tempfile.TemporaryDirectory(prefix="ksl-wire-") as temp:
        for mode in modes:
            fixture = Fixture(mode)
            live = mode in {"fly", "command-error", "cancel"}
            command = [args.executable, "fly" if live else "inspect", "--host", "127.0.0.1",
                       "--port", str(fixture.port), "--duration", "30" if mode == "cancel" else ".3",
                       "--log", str(pathlib.Path(temp)/f"{mode}.jsonl")]
            started = time.monotonic()
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            if mode == "cancel":
                assert fixture.command_seen.wait(4), "No control command before cancellation"
                time.sleep(.03)
                process.send_signal(signal.SIGINT)
            stdout, stderr = process.communicate(timeout=8)
            assert fixture.closed.wait(3), "Socket not explicitly closed"
            fixture.worker.join(timeout=1)
            expected = 0 if mode in {"inspect", "fragmented"} else 2 if mode in {"fly", "cancel"} else 1
            assert process.returncode == expected, (mode, process.returncode, stdout, stderr)
            assert fixture.error is None, (mode, fixture.error)
            if not live:
                assert not any("set_" in name or name in {"Control_SetActionGroup", "AutoPilot_set_Engaged"} for name in fixture.calls), "Read-only inspect mutated vessel"
            if mode == "command-error":
                assert fixture.command_batches == 1, "A failed command was replayed"
            if mode in {"fly", "cancel"}:
                result = json.loads(stdout)
                assert result["outcome"] == ("cancelled" if mode == "cancel" else "timedOut")
                assert fixture.command_batches >= 2, "Neutralisation command missing during cleanup"
            results.append({"case": mode, "passed": True, "exitCode": process.returncode,
                            "rpcCalls": len(fixture.calls), "controlBatches": fixture.command_batches,
                            "socketClosed": True, "elapsedSeconds": round(time.monotonic()-started, 4)})
            print(f"PASS {mode}", flush=True)
    report = {"evidence": "independent loopback protocol fixture; no live KSP", "cases": results}
    pathlib.Path(args.output).write_text(json.dumps(report, indent=2)+"\n")


if __name__ == "__main__":
    main()
