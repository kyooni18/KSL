#!/usr/bin/env python3
"""Translate kRPC C-Nano SerialIO multiplexing to the normal TCP RPC protocol.

This is intentionally synchronous: the landing backend performs request/response RPCs
on one C-Nano RPC connection and does not consume kRPC stream updates here.
"""
from __future__ import annotations

import argparse
import os
import select
import socket
import sys
import termios
import time
from typing import BinaryIO

from krpc.schema import KRPC_pb2 as KRPC


def encode_varint(value: int) -> bytes:
    if value < 0:
        raise ValueError("negative length")
    out = bytearray()
    while value >= 0x80:
        out.append((value & 0x7F) | 0x80)
        value >>= 7
    out.append(value)
    return bytes(out)


def read_exact_fd(fd: int, count: int) -> bytes:
    out = bytearray()
    while len(out) < count:
        ready, _, _ = select.select([fd], [], [], 1.0)
        if not ready:
            continue
        chunk = os.read(fd, count - len(out))
        if not chunk:
            raise EOFError("serial peer closed")
        out.extend(chunk)
    return bytes(out)


def read_exact_socket(sock: socket.socket, count: int) -> bytes:
    out = bytearray()
    while len(out) < count:
        chunk = sock.recv(count - len(out))
        if not chunk:
            raise EOFError("TCP peer closed")
        out.extend(chunk)
    return bytes(out)


def read_varint(read_one) -> int:
    value = 0
    shift = 0
    for _ in range(10):
        b = read_one(1)[0]
        value |= (b & 0x7F) << shift
        if not (b & 0x80):
            return value
        shift += 7
    raise ValueError("oversized protobuf length varint")


def read_delimited_fd(fd: int) -> bytes:
    size = read_varint(lambda n: read_exact_fd(fd, n))
    return read_exact_fd(fd, size)


def read_delimited_socket(sock: socket.socket) -> bytes:
    size = read_varint(lambda n: read_exact_socket(sock, n))
    return read_exact_socket(sock, size)


def write_all_fd(fd: int, data: bytes) -> None:
    view = memoryview(data)
    while view:
        _, ready, _ = select.select([], [fd], [], 1.0)
        if not ready:
            continue
        n = os.write(fd, view)
        if n <= 0:
            raise EOFError("serial write made no progress")
        view = view[n:]


def write_delimited_fd(fd: int, payload: bytes) -> None:
    write_all_fd(fd, encode_varint(len(payload)) + payload)


def write_delimited_socket(sock: socket.socket, payload: bytes) -> None:
    sock.sendall(encode_varint(len(payload)) + payload)


def configure_raw(fd: int) -> None:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = (attrs[2] & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)) | termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def serve(serial_path: str, host: str, port: int, verbose: bool) -> None:
    fd = os.open(serial_path, os.O_RDWR | os.O_NOCTTY)
    configure_raw(fd)
    tcp: socket.socket | None = None
    request_count = 0
    try:
        while True:
            raw = read_delimited_fd(fd)
            envelope = KRPC.MultiplexedRequest()
            envelope.ParseFromString(raw)

            if envelope.HasField("connection_request"):
                if tcp is not None:
                    tcp.close()
                tcp = socket.create_connection((host, port), timeout=5.0)
                tcp.settimeout(None)
                req = envelope.connection_request
                write_delimited_socket(tcp, req.SerializeToString())
                response_raw = read_delimited_socket(tcp)
                response = KRPC.ConnectionResponse()
                response.ParseFromString(response_raw)
                write_delimited_fd(fd, response.SerializeToString())
                if verbose:
                    print(f"connected client={req.client_name!r} status={response.status}", flush=True)
                continue

            if envelope.HasField("request"):
                if tcp is None:
                    raise RuntimeError("RPC request arrived before connection request")
                write_delimited_socket(tcp, envelope.request.SerializeToString())
                response_raw = read_delimited_socket(tcp)
                response = KRPC.Response()
                response.ParseFromString(response_raw)
                wrapped = KRPC.MultiplexedResponse()
                wrapped.response.CopyFrom(response)
                write_delimited_fd(fd, wrapped.SerializeToString())
                request_count += 1
                if verbose and (request_count <= 5 or request_count % 100 == 0):
                    print(f"rpc_requests={request_count} calls={len(envelope.request.calls)} results={len(response.results)}", flush=True)
                continue

            raise RuntimeError("empty or unsupported C-Nano multiplexed request")
    finally:
        if tcp is not None:
            tcp.close()
        os.close(fd)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", default="/tmp/krpc-ksp")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=50000)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    try:
        serve(args.serial, args.host, args.port, args.verbose)
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"bridge error: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
