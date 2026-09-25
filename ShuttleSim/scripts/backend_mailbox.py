"""Bounded-state handoff between the backend reader and runner.

Telemetry is a latest-value stream, not a work queue. Reliable replies are kept
only for outstanding requests, so memory depends on concurrency, not flight time.
"""
from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Any


class BackendMailbox:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._pending: set[str] = set()
        self._responses: dict[str, dict[str, Any]] = {}
        self._ready: dict[str, Any] | None = None
        self._snapshot: dict[str, Any] | None = None
        self._closed = False
        self._error: BaseException | None = None

    def expect(self, request_id: str) -> None:
        with self._condition:
            self._check_locked()
            if request_id in self._pending:
                raise ValueError(f"duplicate outstanding backend request: {request_id}")
            self._pending.add(request_id)

    def cancel(self, request_id: str) -> None:
        with self._condition:
            self._pending.discard(request_id)
            self._responses.pop(request_id, None)

    def publish(self, message: dict[str, Any]) -> None:
        with self._condition:
            kind = message.get("type")
            if kind == "snapshot":
                self._snapshot = message
            elif kind == "ready":
                self._ready = message
            elif kind == "response" and message.get("id") in self._pending:
                self._responses[message["id"]] = message
            self._condition.notify_all()

    def finish(self, error: BaseException | None = None) -> None:
        with self._condition:
            self._closed = True
            self._error = error
            self._condition.notify_all()

    def _check_locked(self) -> None:
        if self._error is not None:
            raise RuntimeError("backend reader failed") from self._error
        if self._closed:
            raise EOFError("backend closed its output stream")

    def check(self) -> None:
        with self._condition:
            self._check_locked()

    def wait(self, predicate: Callable[[dict[str, Any]], bool], timeout: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        with self._condition:
            while True:
                if self._ready is not None and predicate(self._ready):
                    ready, self._ready = self._ready, None
                    return ready
                for request_id, response in self._responses.items():
                    if predicate(response):
                        self._responses.pop(request_id)
                        self._pending.remove(request_id)
                        return response
                if self._snapshot is not None and predicate(self._snapshot):
                    return self._snapshot
                self._check_locked()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("backend response timeout")
                self._condition.wait(remaining)
