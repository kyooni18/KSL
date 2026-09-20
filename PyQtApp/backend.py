from __future__ import annotations

import json
import uuid
from pathlib import Path
from typing import Any

from PyQt6.QtCore import QObject, QProcess, pyqtSignal


class BackendClient(QObject):
    ready = pyqtSignal(dict)
    snapshot = pyqtSignal(dict)
    response = pyqtSignal(dict)
    backend_error = pyqtSignal(str)
    stopped = pyqtSignal()

    def __init__(self, executable: Path, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.executable = Path(executable)
        self.process = QProcess(self)
        self.process.setProgram(str(self.executable))
        self.process.setProcessChannelMode(QProcess.ProcessChannelMode.SeparateChannels)
        self.process.readyReadStandardOutput.connect(self._read_stdout)
        self.process.readyReadStandardError.connect(self._read_stderr)
        self.process.errorOccurred.connect(self._process_error)
        self.process.finished.connect(lambda *_: self.stopped.emit())
        self._stdout_buffer = ""
        self._pending: dict[str, str] = {}

    @property
    def running(self) -> bool:
        return self.process.state() != QProcess.ProcessState.NotRunning

    def start(self) -> None:
        if not self.executable.exists():
            self.backend_error.emit(f"C landing backend not found: {self.executable}")
            return
        self.process.start()

    def send(self, method: str, **payload: Any) -> str:
        request_id = uuid.uuid4().hex
        command = {"id": request_id, "method": method, **payload}
        self._pending[request_id] = method
        data = (json.dumps(command, separators=(",", ":"), allow_nan=False) + "\n").encode()
        if not self.running:
            self.backend_error.emit(f"Backend is not running; could not send {method}.")
            return request_id
        self.process.write(data)
        return request_id

    def shutdown(self) -> None:
        if not self.running:
            return
        self.send("shutdown")
        # The C backend performs a synchronous safe disconnect. Give a
        # currently-running telemetry/apply RPC enough time to hit its own
        # timeout and still issue the kRPC safe command before escalating.
        self.process.waitForFinished(7000)
        if self.running:
            self.process.terminate()
            self.process.waitForFinished(1200)
        if self.running:
            self.process.kill()

    def _read_stdout(self) -> None:
        self._stdout_buffer += bytes(self.process.readAllStandardOutput()).decode("utf-8", "replace")
        while "\n" in self._stdout_buffer:
            line, self._stdout_buffer = self._stdout_buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                message = json.loads(line)
            except json.JSONDecodeError as exc:
                self.backend_error.emit(f"Invalid backend JSON: {exc}: {line[:240]}")
                continue
            event_type = message.get("type")
            if event_type == "ready":
                self.ready.emit(message)
            elif event_type == "snapshot":
                snapshot = message.get("snapshot")
                if isinstance(snapshot, dict):
                    self.snapshot.emit(snapshot)
            elif event_type == "response":
                request_id = message.get("id")
                if isinstance(request_id, str):
                    self._pending.pop(request_id, None)
                if not message.get("ok", False):
                    self.backend_error.emit(str(message.get("error", "Backend request failed")))
                self.response.emit(message)

    def _read_stderr(self) -> None:
        text = bytes(self.process.readAllStandardError()).decode("utf-8", "replace").strip()
        if text:
            self.backend_error.emit(text)

    def _process_error(self, error: QProcess.ProcessError) -> None:
        self.backend_error.emit(f"Backend process error: {error.name}")
