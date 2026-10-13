"""Operator runtime controls shared by the TUI and the flight programs.

The TUI (or any script) writes ``controls.json`` in the run directory; the child
flight programs poll it. The file is the only coupling, so the controls work for
missions started from the TUI, the CLI, or another terminal, and a stale or
missing file simply means "defaults".

    autoWarp   bool     programs may time-warp on their own between burns
    trim       bool     the OMS orbit trim / orbit operations may run
    holdTrim   bool     wait in orbit until ``trimNow`` is triggered
    warpNow    counter  one-shot: warp to the next event even if autoWarp is off
    warpStop   counter  one-shot: drop to 1x immediately
    trimNow    counter  one-shot: start the held orbit operations
"""

from __future__ import annotations

import json
import os
import tempfile
import time
from pathlib import Path
from typing import Any

ENV_VAR = "KSL_CONTROLS"
FILE_NAME = "controls.json"

DEFAULT_CONTROLS: dict[str, Any] = {
    "autoWarp": True,
    "trim": True,
    "holdTrim": False,
    "warpNow": 0,
    "warpStop": 0,
    "trimNow": 0,
}
COUNTERS = ("warpNow", "warpStop", "trimNow")


class Controls:
    """File-backed control set. ``take`` consumes one-shot triggers per reader."""

    def __init__(self, path: str | os.PathLike[str] | None = None, defaults: dict[str, Any] | None = None) -> None:
        self.path = Path(path) if path else None
        self._memory = {**DEFAULT_CONTROLS, **(defaults or {})}
        self._seen: dict[str, int] = {}
        self._cache: dict[str, Any] | None = None
        self._cache_stamp: tuple[float, int] | None = None
        # A trigger already pending when a reader starts is stale (it belongs to
        # an earlier phase); only triggers raised afterwards count.
        for name in COUNTERS:
            self._seen[name] = int(self.read().get(name, 0))

    @classmethod
    def from_env(cls) -> "Controls":
        return cls(os.environ.get(ENV_VAR))

    @classmethod
    def for_run(cls, run_dir: str | os.PathLike[str], defaults: dict[str, Any] | None = None) -> "Controls":
        path = Path(run_dir) / FILE_NAME
        controls = cls(path, defaults)
        if not path.is_file():
            controls.write({**DEFAULT_CONTROLS, **(defaults or {})})
        return controls

    # --- reading ---------------------------------------------------------------
    def read(self) -> dict[str, Any]:
        if self.path is None:
            return dict(self._memory)
        try:
            stat = self.path.stat()
            stamp = (stat.st_mtime, stat.st_size)
            if stamp != self._cache_stamp or self._cache is None:
                data = json.loads(self.path.read_text(encoding="utf-8"))
                self._cache = {**DEFAULT_CONTROLS, **data} if isinstance(data, dict) else dict(DEFAULT_CONTROLS)
                self._cache_stamp = stamp
            return dict(self._cache)
        except (OSError, json.JSONDecodeError):
            return dict(self._cache or DEFAULT_CONTROLS)

    def get(self, key: str) -> Any:
        return self.read().get(key, DEFAULT_CONTROLS.get(key))

    @property
    def auto_warp(self) -> bool:
        return bool(self.get("autoWarp"))

    @property
    def trim_enabled(self) -> bool:
        return bool(self.get("trim"))

    @property
    def hold_trim(self) -> bool:
        return bool(self.get("holdTrim"))

    def take(self, name: str) -> bool:
        """True once for each increment of a one-shot counter."""
        value = int(self.read().get(name, 0))
        if value > self._seen.get(name, 0):
            self._seen[name] = value
            return True
        return False

    # --- writing ---------------------------------------------------------------
    def write(self, data: dict[str, Any]) -> None:
        if self.path is None:
            self._memory = {**DEFAULT_CONTROLS, **data}
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=self.path.parent, prefix=".controls-", suffix=".tmp")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(data, handle, indent=2)
            os.replace(tmp, self.path)
        finally:
            if os.path.exists(tmp):
                os.unlink(tmp)

    def update(self, **changes: Any) -> dict[str, Any]:
        data = self.read()
        data.update(changes)
        self.write(data)
        self._cache, self._cache_stamp = None, None
        return data

    def trigger(self, name: str) -> int:
        if name not in COUNTERS:
            raise KeyError(name)
        count = int(self.read().get(name, 0)) + 1
        self.update(**{name: count})
        return count

    def toggle(self, name: str) -> bool:
        value = not bool(self.get(name))
        self.update(**{name: value})
        return value


# --- time warp ----------------------------------------------------------------------

RAILS_RATES = (1.0, 5.0, 10.0, 50.0, 100.0, 1000.0, 10000.0, 100000.0)


def rails_factor_for(remaining: float, lead: float = 0.0, safety: float = 4.0) -> int:
    """Highest rails warp index whose rate leaves at least ``safety`` real seconds."""
    usable = remaining - lead
    best = 0
    for index, rate in enumerate(RAILS_RATES):
        if usable / rate >= safety:
            best = index
    return best


def warp_until(space_center: Any, ut: float, controls: Controls | None, lead: float = 0.0,
               sleep=time.sleep, announce=None, poll: float = 0.25) -> bool:
    """Wait for game time ``ut`` (minus ``lead``), warping only as the operator allows.

    Returns True when the time was reached. Unlike ``SpaceCenter.warp_to`` this can
    be interrupted: switching auto-warp off, or a ``warpStop`` trigger, drops to 1x
    within one poll interval, and a ``warpNow`` trigger warps even with auto-warp
    off. The warp factor is re-evaluated continuously so it ramps down on approach.
    """
    target = ut - lead
    forced = False
    announced = None
    while space_center.ut < target:
        if controls is not None and controls.take("warpStop"):
            forced = False
            space_center.rails_warp_factor = 0
        if controls is not None and controls.take("warpNow"):
            forced = True
        allowed = controls is None or controls.auto_warp or forced
        want = rails_factor_for(target - space_center.ut) if allowed else 0
        if space_center.rails_warp_factor != want:
            try:
                space_center.rails_warp_factor = want
            except Exception:
                want = 0  # e.g. too low for rails warp; stay at 1x
        if announce is not None and want != announced:
            announced = want
            announce(want, target - space_center.ut)
        sleep(poll)
    try:
        space_center.rails_warp_factor = 0
    except Exception:
        pass
    return True
