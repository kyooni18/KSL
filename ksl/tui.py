"""``ksl tui``: keyboard-first terminal interface for the unified mission system.

Tabs
  Dashboard  phase strip, vehicle state, guidance/status, connection, event log
  Mission    orbit profile (circular, plane, GTO, GEO, rendezvous), inclination /
             altitude, operations (auto-warp, orbit trim), mode / phases; start/abort
  Controls   live operator controls of the watched run (any run, any process):
             f warp now · x stop warp · g auto-warp on/off · o orbit trim on/off ·
             h hold in orbit on/off · t trigger orbit trim
  Replay     recorded runs: summary, phase timeline, events, sample stepping

The dashboard always watches a run *directory* (status.json + events.jsonl),
so it shows missions started here (in a background thread through the same
MissionController the CLI uses) exactly like missions started from the CLI
or another terminal. A run whose status stops updating is flagged as stale
instead of silently freezing the display.
"""

from __future__ import annotations

import json
import asyncio
import time
from pathlib import Path
from typing import Any

from rich.text import Text
from textual import on
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, VerticalScroll
from textual.widgets import (Button, DataTable, Footer, Header, Input, Label, RichLog, Select, Static,
                             Switch, TabbedContent, TabPane)

from .config import DEFAULT_PROFILE, DEFAULTS, PROFILE_KINDS, ConfigError, load_config
from .controls import FILE_NAME as CONTROLS_FILE, Controls
from .controller import MissionController, holdable
from .phases import ENTRY_PHASES, SEQUENCE, MissionState, Phase, PhaseTransitionError, validate_range
from .runlog import Event, MissionStatus, list_runs, read_status
from .scenarios import SCENARIOS

STALE_SECONDS = 15.0
KIND_LABELS = {
    "circular": "Circular orbit (altitude / inclination)",
    "plane": "Plane of a moon/vessel (launch window), e.g. Minmus",
    "gto": "GTO — ET semi-GTO, OMS apogee raise, stay in GTO",
    "geo": "GEO — GTO over a chosen longitude, circularize if OMS dv allows",
    "rendezvous": "Rendezvous — plane launch, parking orbit, Lambert transfer, RCS approach",
}
DEFAULT_MODE = "dry-run"
LEVEL_STYLE = {"error": "bold red", "warning": "yellow", "info": "", "debug": "dim"}
KIND_STYLE = {"phase": "bold cyan", "state": "bold", "milestone": "green", "checkpoint": "magenta"}


def _fmt(value: Any, scale: float = 1.0, unit: str = "", digits: int = 1) -> str:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return f"{value * scale:,.{digits}f} {unit}".strip()
    return "—" if value in (None, "") else str(value)


class RunWatcher:
    """Incrementally reads a run directory (works for in-process and external runs)."""

    def __init__(self, run_dir: Path) -> None:
        self.run_dir = run_dir
        self.offset = 0
        self.status: MissionStatus | None = None

    def poll(self) -> tuple[MissionStatus | None, list[Event]]:
        try:
            self.status = read_status(self.run_dir)
        except (OSError, json.JSONDecodeError, TypeError):
            pass  # mid-replace or deleted; keep the previous status
        events: list[Event] = []
        path = self.run_dir / "events.jsonl"
        try:
            with path.open(encoding="utf-8") as handle:
                handle.seek(self.offset)
                for line in handle:
                    if not line.endswith("\n"):
                        break               # partial write; re-read next poll
                    self.offset += len(line.encode("utf-8"))
                    try:
                        events.append(Event.from_dict(json.loads(line)))
                    except json.JSONDecodeError:
                        continue
        except OSError:
            pass
        return self.status, events


class PhaseStrip(Static):
    def show(self, status: MissionStatus | None) -> None:
        text = Text()
        current = Phase(status.phase) if status else None
        done = set(status.phase_history) if status else set()
        failed = status is not None and status.state in (MissionState.FAILED.value, MissionState.ABORTED.value)
        for i, phase in enumerate(SEQUENCE):
            if i:
                text.append(" › ", style="dim")
            if phase == current:
                text.append(f" {phase.title} ", style="bold white on red" if failed else "bold black on cyan")
            elif phase.value in done:
                text.append(phase.title, style="green")
            else:
                text.append(phase.title, style="dim")
        self.update(text)


class KeyValue(Static):
    def show(self, title: str, rows: list[tuple[str, str]], style: str = "") -> None:
        text = Text(f"{title}\n", style="bold")
        width = max((len(k) for k, _ in rows), default=0)
        for key, value in rows:
            text.append(f"{key:<{width}}  ", style="dim")
            text.append(f"{value}\n", style=style)
        self.update(text)


class KSLApp(App[int]):
    TITLE = "KSL — Shuttle Flight Control"
    CSS = """
    Screen { layout: vertical; }
    #phases { height: auto; padding: 0 1; margin-bottom: 1; }
    #panels { height: auto; }
    .panel { border: round $primary-darken-2; padding: 0 1; width: 1fr; height: auto; min-height: 9; }
    #banner { height: auto; padding: 0 1; }
    #eventlog { height: 1fr; border: round $primary-darken-2; }
    #setup { padding: 1 2; }
    #setup Label { margin-top: 1; }
    #setup Select, #setup Input { width: 60; }
    .switchrow { height: auto; margin-top: 1; }
    .switchrow Label { margin: 1 1 0 0; width: 44; }
    #buttons { height: auto; margin-top: 1; }
    #buttons Button { margin-right: 2; }
    #plan { border: round $primary-darken-2; height: auto; max-height: 14; padding: 0 1; margin-top: 1; }
    #replay-top { height: 1fr; }
    #runs { width: 2fr; }
    #replay-detail { width: 3fr; border: round $primary-darken-2; padding: 0 1; }
    #sample { height: auto; border: round $primary-darken-2; padding: 0 1; }
    """
    BINDINGS = [
        Binding("d", "tab('dashboard')", "Dashboard"),
        Binding("m", "tab('mission')", "Mission"),
        Binding("r", "tab('replay')", "Replay"),
        Binding("s", "start", "Start"),
        Binding("a", "abort", "Abort"),
        Binding("w", "watch_latest", "Watch latest"),
        Binding("ctrl+l", "clear_log", "Clear log"),
        Binding("v", "toggle_verbose", "Raw output"),
        Binding("f", "control('warpNow')", "Warp now"),
        Binding("x", "control('warpStop')", "Stop warp"),
        Binding("g", "control('autoWarp')", "Auto-warp"),
        Binding("o", "control('trim')", "Orbit trim"),
        Binding("h", "control('holdTrim')", "Hold"),
        Binding("t", "control('trimNow')", "Trim now"),
        Binding("q", "quit_safe", "Quit"),
    ]

    def __init__(self, profile: str | None = None, overrides: list[str] | None = None,
                 runs_root: Path | None = None) -> None:
        super().__init__()
        self.profile = profile or DEFAULT_PROFILE
        self.overrides = list(overrides or [])
        self.runs_root = runs_root
        self.controller: MissionController | None = None
        self.watcher: RunWatcher | None = None
        self.verbose = False
        self.abort_armed_at = 0.0
        self.quit_armed_at = 0.0
        self.replay_samples: list[dict[str, Any]] = []
        self.replay_index = 0
        self.vehicle_document = None

    # --- layout -------------------------------------------------------------------
    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with TabbedContent(initial="dashboard", id="tabs"):
            with TabPane("Dashboard", id="dashboard"):
                yield PhaseStrip(id="phases")
                yield Static(id="banner")
                with Horizontal(id="panels"):
                    yield KeyValue(id="mission-panel", classes="panel")
                    yield KeyValue(id="vehicle-panel", classes="panel")
                    yield KeyValue(id="guidance-panel", classes="panel")
                    yield KeyValue(id="controls-panel", classes="panel")
                yield RichLog(id="eventlog", wrap=True, markup=False, max_lines=5000)
            with TabPane("Mission", id="mission"):
                with VerticalScroll(id="setup"):
                    ascent = self._base_ascent()
                    base = self._base()
                    pr = base["profile"]
                    yield Label("Ascent guidance (next mission only)")
                    yield Select([("Legacy STS (default)", "legacy"), ("Intelligent", "intelligent")],
                                 value=ascent["guidanceMode"], allow_blank=False, id="ascent-mode")
                    yield Label("Vehicle configuration (optional; inspect in Vehicle tab)")
                    yield Input(base["intelligentAscent"]["vehicleConfig"], id="vehicle-config")
                    yield Label("Orbit profile")
                    yield Select([(KIND_LABELS[k], k) for k in PROFILE_KINDS], value=pr["kind"],
                                 allow_blank=False, id="kind")
                    yield Label("Plane / rendezvous target (moon or vessel name)")
                    yield Input(pr["planeTarget"], placeholder="e.g. Minmus or Station", id="plane-target")
                    yield Label("GEO longitude (deg east; geo profile)")
                    yield Input(f"{pr['geoLongitudeDeg']:g}", id="geo-longitude")
                    yield Label("GTO perigee (km; gto/geo profiles)")
                    yield Input(f"{pr['gtoPerigeeKm']:g}", id="gto-perigee")
                    yield Label("GEO circularization")
                    yield Select([("Auto — only if OMS delta-v is enough", "auto"), ("Always", "always"),
                                  ("Never — stay in GTO", "never")], value=pr["geoCircularize"],
                                 allow_blank=False, id="geo-circularize")
                    with Horizontal(classes="switchrow"):
                        yield Label("Wait for launch window (plane / rendezvous)")
                        yield Switch(bool(pr["launchWindow"]), id="launch-window")
                    with Horizontal(classes="switchrow"):
                        yield Label("Phase GTO apogee onto the GEO longitude")
                        yield Switch(bool(pr["targetLongitude"]), id="target-longitude")
                    with Horizontal(classes="switchrow"):
                        yield Label("Auto time-warp (live: g)")
                        yield Switch(bool(base["operations"]["autoWarp"]), id="auto-warp")
                    with Horizontal(classes="switchrow"):
                        yield Label("Orbit trim / orbit operations (live: o)")
                        yield Switch(bool(base["orbit"]["trim"]), id="orbit-trim")
                    with Horizontal(classes="switchrow"):
                        yield Label("Hold in orbit until triggered (live: h, t)")
                        yield Switch(bool(base["orbit"]["holdForTrigger"]), id="hold-trim")
                    yield Label("Maximum trim burns")
                    yield Input(str(base["orbit"]["maxBurns"]), id="max-burns")
                    yield Label("Inclination (deg)")
                    yield Input(f"{ascent['inclinationDeg']:g}", id="inclination")
                    yield Label("Target altitude (km, circular orbit)")
                    yield Input(f"{ascent['targetAltitudeKm']:g}", id="altitude")
                    yield Label("Mode")
                    yield Select([("Live KSP", "live"), ("Simulation (ShuttleSim)", "sim"),
                                  ("Dry run (plan only)", "dry-run")], value=DEFAULT_MODE, allow_blank=False, id="mode")
                    yield Label("Start phase (live / dry run)")
                    yield Select([(f"{p.title} — {d}", p.value) for p, d in ENTRY_PHASES.items()],
                                 value=Phase.PRELAUNCH.value, allow_blank=False, id="start",
                                 disabled=DEFAULT_MODE == "sim")
                    yield Label("Stop / hold phase (only phases this mode can hold at)")
                    yield Select(self._stop_options(DEFAULT_MODE), value=Phase.COMPLETE.value,
                                 allow_blank=False, id="stop")
                    yield Label("Simulation scenario (simulation mode only; sets the start phase)")
                    yield Select([(f"{s.name} — from {s.start.title}", s.name) for s in SCENARIOS.values()],
                                 value="l5-mm305", allow_blank=False, id="scenario",
                                 disabled=DEFAULT_MODE != "sim")
                    yield Label("Other overrides (space separated KEY=VALUE, e.g. orbit.trim=false)")
                    yield Input(" ".join(self.overrides), id="overrides")
                    with Horizontal(id="buttons"):
                        yield Button("Validate plan", id="validate")
                        yield Button("Start  [s]", id="start-btn", variant="success")
                        yield Button("Abort  [a]", id="abort-btn", variant="error")
                    yield Label("Save setup as reusable mission profile")
                    yield Input("Configuration/missions/user-ascent.json", id="save-profile-path")
                    yield Button("Save mission setup", id="save-profile")
                    yield Static(id="plan")
            with TabPane("Vehicle", id="vehicle-editor"):
                with VerticalScroll():
                    yield Button("Inspect active vessel (read-only)", id="inspect-vehicle")
                    yield Static("Load a vessel, then inspect its attachment tree.", id="vehicle-tree", markup=False)
                    yield Label("Component")
                    yield Select([], id="component")
                    yield Label("Role")
                    from .ascent.intelligent.vehicle import ROLES
                    yield Select([(role, role) for role in ROLES], value="structure", allow_blank=False, id="component-role")
                    yield Label("Permit depleted engine-less tank separation at this decoupler")
                    yield Switch(False, id="approve-separator")
                    yield Button("Apply component correction", id="apply-component")
                    yield Input("Configuration/vehicles/user-ascent.json", id="vehicle-save-path")
                    yield Button("Save vehicle and use for next mission", id="save-vehicle")
            with TabPane("Replay", id="replay"):
                with Horizontal(id="replay-top"):
                    yield DataTable(id="runs", cursor_type="row", zebra_stripes=True)
                    with VerticalScroll(id="replay-detail"):
                        yield Static(id="replay-summary")
                yield Static("←/→ step samples · enter: open run in dashboard", id="sample")
        yield Footer()

    def on_mount(self) -> None:
        self.query_one("#eventlog", RichLog).write(Text(
            "Keys: d/m/r tabs · s start · a abort (press twice) · w watch latest run · v raw program output · "
            "f warp now · x stop warp · g auto-warp · o orbit trim · h hold · t trim now · q quit", style="dim"))
        table = self.query_one("#runs", DataTable)
        table.add_columns("Run", "Mode", "State", "Phase", "Profile")
        self.refresh_runs()
        self.render_dashboard(None)
        self.set_interval(0.5, self.poll)

    # --- dashboard ------------------------------------------------------------------
    def poll(self) -> None:
        # Textual stops the app before removing dashboard widgets. A queued
        # timer callback during teardown must not query that disappearing tree.
        if not self.is_running or self.watcher is None:
            return
        status, events = self.watcher.poll()
        log = self.query_one("#eventlog", RichLog)
        for event in events:
            if event.kind == "program" and not self.verbose:
                continue
            stamp = time.strftime("%H:%M:%S", time.localtime(event.time))
            style = KIND_STYLE.get(event.kind, "") if event.level == "info" else LEVEL_STYLE.get(event.level, "")
            log.write(Text(f"{stamp}  {event.message}", style=style))
        self.render_dashboard(status)

    def render_dashboard(self, status: MissionStatus | None) -> None:
        self.query_one("#phases", PhaseStrip).show(status)
        banner = self.query_one("#banner", Static)
        mission = self.query_one("#mission-panel", KeyValue)
        vehicle = self.query_one("#vehicle-panel", KeyValue)
        guidance = self.query_one("#guidance-panel", KeyValue)
        if status is None:
            banner.update(Text("No mission selected — start one on the Mission tab (m) or press w to watch "
                               "the latest run.", style="dim"))
            mission.show("Mission", [("state", "—")])
            self.query_one("#controls-panel", KeyValue).show("Controls", [("run", "—")])
            vehicle.show("Vehicle", [("altitude", "—")])
            guidance.show("Guidance", [("program", "—")])
            return
        running = status.state == MissionState.RUNNING.value
        stale = running and time.time() - (status.updated_at or 0) > STALE_SECONDS
        local = self.controller is not None and self.watcher is not None \
            and self.controller.run_dir == self.watcher.run_dir
        if status.error:
            banner.update(Text(f"✖ {status.error}", style="bold red"))
        elif stale:
            banner.update(Text(f"⚠ no status update for {time.time() - status.updated_at:.0f} s — the mission "
                               "process may have exited or lost contact", style="bold yellow"))
        elif status.warning:
            banner.update(Text(f"⚠ {status.warning}", style="yellow"))
        else:
            banner.update(Text(status.status_message or "", style="green" if not running else ""))
        conn_style = {"connected": "green", "simulator": "green", "failed": "bold red"}.get(status.connection, "yellow")
        mission.show("Mission", [
            ("run", status.run_id), ("profile", f"{status.profile} ({status.mode})"),
            ("state", status.state.upper() + ("  (this TUI)" if local else "  (watching)")),
            ("phase", Phase(status.phase).title), ("range", f"{status.start_phase} → {status.stop_phase}"),
            ("program", status.program or "—"),
        ])
        mission.styles.border = ("round", "red" if status.state in ("failed", "aborted") else
                                 "green" if status.state == "complete" else "cyan" if running else "grey")
        t = status.telemetry or {}
        vehicle.show("Vehicle", [
            ("altitude", _fmt(t.get("altitude"), 1e-3, "km", 2)), ("speed", _fmt(t.get("speed"), unit="m/s")),
            ("vert speed", _fmt(t.get("verticalSpeed"), unit="m/s")),
            ("apoapsis", _fmt(t.get("apoapsis"), 1e-3, "km", 2)), ("periapsis", _fmt(t.get("periapsis"), 1e-3, "km", 2)),
            ("dyn press", _fmt(t.get("dynamicPressure"), 1e-3, "kPa", 2)), ("AoA / roll",
             f"{_fmt(t.get('angleOfAttack'))} / {_fmt(t.get('roll'))} deg"),
        ])
        guidance.show("Guidance", [
            ("connection", status.connection), ("backend", str(t.get("backendPhase") or "—")),
            ("range", _fmt(t.get("rangeToSite"), 1e-3, "km")),
            ("along/cross", f"{_fmt(t.get('runwayAlongTrack'), 1e-3, 'km', 2)} / "
                            f"{_fmt(t.get('runwayCrossTrack'), unit='m', digits=0)}"),
            ("target AoA", _fmt(t.get("targetAoA"), unit="deg")), ("target roll", _fmt(t.get("targetRoll"), unit="deg")),
            ("status", (status.status_message or "—")[:60]),
        ], style="")
        guidance.styles.border = ("round", conn_style)
        self._render_controls()

    def _controls(self) -> Controls | None:
        if self.watcher is None:
            return None
        return Controls(self.watcher.run_dir / CONTROLS_FILE)

    def _render_controls(self) -> None:
        panel = self.query_one("#controls-panel", KeyValue)
        controls = self._controls()
        if controls is None or not (self.watcher.run_dir / CONTROLS_FILE).is_file():
            panel.show("Controls", [("run", "— (no live controls)")])
            return
        c = controls.read()
        on = lambda flag: "ON" if flag else "off"  # noqa: E731
        panel.show("Controls", [("auto-warp [g]", on(c["autoWarp"])), ("orbit trim [o]", on(c["trim"])),
                                ("hold [h]", on(c["holdTrim"])), ("warp now [f]", str(c["warpNow"])),
                                ("trim now [t]", str(c["trimNow"]))])

    def action_control(self, name: str) -> None:
        controls = self._controls()
        if controls is None:
            self.notify("Watch a run first (start one, or press w).", severity="warning")
            return
        if name in ("autoWarp", "trim", "holdTrim"):
            value = controls.toggle(name)
            self.notify(f"{name} {'ON' if value else 'off'} for {self.watcher.run_dir.name}")
        else:
            controls.trigger(name)
            self.notify({"warpNow": "Warp to the next event requested", "warpStop": "Warp stop requested",
                         "trimNow": "Orbit operations triggered"}[name])
        self._render_controls()

    def watch_run(self, run_dir: Path) -> None:
        self.watcher = RunWatcher(run_dir)
        self.query_one("#eventlog", RichLog).write(Text(f"— watching {run_dir.name} —", style="bold"))
        self.poll()
        self.query_one("#tabs", TabbedContent).active = "dashboard"
        self.query_one("#eventlog", RichLog).focus()  # keep focus out of the hidden Mission pane

    # --- mission setup ----------------------------------------------------------------
    def _base(self) -> dict[str, Any]:
        try:
            return load_config(self.profile, self.overrides).data
        except ConfigError:
            return DEFAULTS

    def _base_ascent(self) -> dict[str, Any]:
        """Ascent values of the base profile (plus CLI overrides), shown in the Mission fields."""
        try:
            return load_config(self.profile, self.overrides).data["ascent"]
        except ConfigError:
            return DEFAULTS["ascent"]  # the start action reports the broken profile

    def _number(self, wid: str, label: str) -> float:
        text = self.query_one(f"#{wid}", Input).value.strip()
        try:
            return float(text)
        except ValueError:
            raise ConfigError(f"{label} must be a number, not {text!r}") from None

    @staticmethod
    def _stop_options(mode: str) -> list[tuple[str, str]]:
        return [(p.title, p.value) for p in SEQUENCE if holdable(p, mode)]

    @on(Select.Changed, "#mode")
    def mode_changed(self, event: Select.Changed) -> None:
        mode = str(event.value)
        stop = self.query_one("#stop", Select)
        previous = stop.value
        options = self._stop_options(mode)
        stop.set_options(options)
        stop.value = previous if previous in {value for _, value in options} else Phase.COMPLETE.value
        self.query_one("#start", Select).disabled = mode == "sim"
        self.query_one("#scenario", Select).disabled = mode != "sim"

    def _selection(self) -> dict[str, Any]:
        value = lambda wid: self.query_one(f"#{wid}", Select).value  # noqa: E731
        # The inclination and altitude fields are the single source for those two keys, so they
        # are appended last and any typed copy of them in the overrides box is dropped.
        switch = lambda wid: self.query_one(f"#{wid}", Switch).value  # noqa: E731
        owned_values = {
            "ascent.guidanceMode": str(value("ascent-mode")),
            "intelligentAscent.vehicleConfig": self.query_one("#vehicle-config", Input).value.strip(),
            "ascent.inclinationDeg": self._number("inclination", "Inclination"),
            "ascent.targetAltitudeKm": self._number("altitude", "Altitude"),
            "profile.kind": str(value("kind")),
            "profile.planeTarget": self.query_one("#plane-target", Input).value.strip(),
            "profile.geoLongitudeDeg": self._number("geo-longitude", "GEO longitude"),
            "profile.gtoPerigeeKm": self._number("gto-perigee", "GTO perigee"),
            "profile.geoCircularize": str(value("geo-circularize")),
            "profile.launchWindow": switch("launch-window"),
            "profile.targetLongitude": switch("target-longitude"),
            "operations.autoWarp": switch("auto-warp"),
            "orbit.trim": switch("orbit-trim"),
            "orbit.holdForTrigger": switch("hold-trim"),
            "orbit.maxBurns": int(self._number("max-burns", "Maximum trim burns")),
        }
        owned = tuple(owned_values)
        extra = [text for text in self.query_one("#overrides", Input).value.split()
                 if text.split("=", 1)[0] not in owned]
        fields = [f"{key}={json.dumps(val)}" for key, val in owned_values.items()]
        return {"profile": self.profile, "mode": value("mode"), "start": Phase(value("start")),
                "stop": Phase(value("stop")), "scenario": SCENARIOS[str(value("scenario"))],
                "overrides": extra + fields}

    def _build_controller(self, mode_override: str | None = None) -> MissionController:
        sel = self._selection()
        config = load_config(str(sel["profile"]), sel["overrides"])
        mode = mode_override or str(sel["mode"])
        return MissionController(config, mode, sel["start"], sel["stop"],
                                 scenario=sel["scenario"] if mode == "sim" else None,
                                 runs_root=self.runs_root, label="tui")

    def _show_plan(self, message: str, style: str = "") -> None:
        self.query_one("#plan", Static).update(Text(message, style=style))

    @on(Button.Pressed, "#validate")
    def validate_plan(self) -> None:
        try:
            sel = self._selection()
            config = load_config(str(sel["profile"]), sel["overrides"])
            mode = str(sel["mode"])
            probe = MissionController.__new__(MissionController)  # plan without creating a run
            probe.config, probe.mode = config, mode
            probe.start = sel["scenario"].start if mode == "sim" else sel["start"]
            probe.stop = sel["stop"]
            validate_range(probe.start, probe.stop)
            if not holdable(probe.stop, mode):
                raise PhaseTransitionError(f"cannot hold at {probe.stop.value} in {mode} mode")
            problems = config.validate()
            if problems:
                raise ConfigError("; ".join(problems))
            lines = [f"{config.name}: {mode} {probe.start.title} → {probe.stop.title}"]
            lines += [f"  • {program.name} from {phase.title}" for program, phase in probe.plan()]
            if mode == "sim":
                lines.append(f"  scenario {sel['scenario'].name}: {sel['scenario'].description}")
                if sel["scenario"].baseline:
                    lines.append(f"  baseline: {sel['scenario'].baseline}")
            self._show_plan("\n".join(lines), "green")
        except (ConfigError, PhaseTransitionError, ValueError, KeyError) as exc:
            self._show_plan(f"✖ {exc}", "bold red")

    @on(Button.Pressed, "#start-btn")
    def action_start(self) -> None:
        if self.controller is not None and self.controller.status.state == MissionState.RUNNING.value:
            self.notify("A mission is already running from this TUI.", severity="warning")
            return
        try:
            controller = self._build_controller()
        except (ConfigError, PhaseTransitionError, ValueError, KeyError) as exc:
            self._show_plan(f"✖ {exc}", "bold red")
            self.notify(str(exc), title="Cannot start", severity="error")
            return
        if controller.mode == "live":
            self.notify("Live mission started — flight software is now commanding KSP.", severity="warning")
        self.controller = controller
        controller.start_background()
        self.watch_run(controller.run_dir)

    @on(Button.Pressed, "#abort-btn")
    def action_abort(self) -> None:
        if self.controller is None or self.controller.status.state != MissionState.RUNNING.value:
            self.notify("No mission running from this TUI (external runs: Ctrl-C their CLI).")
            return
        if time.monotonic() - self.abort_armed_at > 3.0:
            self.abort_armed_at = time.monotonic()
            self.notify("Press abort again within 3 s to confirm.", severity="warning")
            return
        self.controller.abort()
        self.notify("Abort requested: safe-control path running.", severity="error")

    @on(Button.Pressed, "#save-profile")
    def save_profile(self) -> None:
        from .paths import repo_path
        from .ascent.intelligent.vehicle import save_document
        try:
            selection = self._selection()
            config = load_config(str(selection["profile"]), selection["overrides"])
            problems = config.validate()
            if problems:
                raise ConfigError("; ".join(problems))
            path = repo_path(self.query_one("#save-profile-path", Input).value.strip())
            save_document(path, config.data)
            self.profile = str(path)
            self.notify(f"Saved mission. Reuse with ksl tui -p {path}")
        except (OSError, ValueError, ConfigError) as exc:
            self.notify(str(exc), severity="error")

    @on(Button.Pressed, "#inspect-vehicle")
    async def inspect_vehicle(self) -> None:
        from .krpc_link import connect
        from .ascent.intelligent.vehicle import Vehicle, diagram
        try:
            config = load_config(self.profile, self.overrides)
            def inspect():
                with connect(config, "KSL vehicle inspector") as conn:
                    if str(conn.krpc.current_game_scene).split('.')[-1] != 'flight':
                        raise ValueError("Load a vessel in the flight scene first")
                    return Vehicle(conn.space_center.active_vessel).document()
            document = await asyncio.to_thread(inspect)
            self.vehicle_document = document
            self.query_one("#vehicle-tree", Static).update(diagram(document))
            self.query_one("#component", Select).set_options(
                [(f"{c['key']} {c['name']} ({c['role']})", c['key']) for c in document['components']])
        except Exception as exc:
            self.notify(str(exc), severity="error")

    @on(Select.Changed, "#component")
    def select_component(self, event: Select.Changed) -> None:
        if self.vehicle_document is None or event.value is Select.BLANK:
            return
        key = str(event.value)
        self.query_one("#component-role", Select).value = self.vehicle_document['roles'][key]
        self.query_one("#approve-separator", Switch).value = key in self.vehicle_document['approvedSeparators']

    @on(Button.Pressed, "#apply-component")
    def apply_component(self) -> None:
        from .ascent.intelligent.vehicle import diagram
        document = self.vehicle_document
        key = self.query_one("#component", Select).value
        if document is None or key is Select.BLANK:
            return
        role = str(self.query_one("#component-role", Select).value)
        document['roles'][str(key)] = role
        for c in document['components']:
            if c['key'] == key:
                c['role'] = role
        approved = set(document['approvedSeparators'])
        approved.discard(str(key))
        if self.query_one("#approve-separator", Switch).value:
            approved.add(str(key))
        document['approvedSeparators'] = sorted(approved)
        self.query_one("#vehicle-tree", Static).update(diagram(document))

    @on(Button.Pressed, "#save-vehicle")
    def save_vehicle(self) -> None:
        from .paths import repo_path
        from .ascent.intelligent.vehicle import save_document
        if self.vehicle_document is None:
            self.notify("Inspect a vehicle first", severity="warning")
            return
        try:
            path = repo_path(self.query_one("#vehicle-save-path", Input).value.strip())
            save_document(path, self.vehicle_document)
            self.query_one("#vehicle-config", Input).value = str(path)
            self.notify("Vehicle saved for the next mission. Save mission setup to reuse it.")
        except OSError as exc:
            self.notify(str(exc), severity="error")

    # --- replay -----------------------------------------------------------------------
    def refresh_runs(self) -> None:
        table = self.query_one("#runs", DataTable)
        table.clear()
        for run in reversed(list_runs(self.runs_root)[-200:]):
            try:
                s = read_status(run)
            except (OSError, json.JSONDecodeError, TypeError):
                continue
            state_style = {"failed": "red", "aborted": "red", "complete": "green", "running": "cyan"}.get(s.state, "")
            table.add_row(run.name, s.mode, Text(s.state, style=state_style), s.phase, s.profile, key=str(run))

    @on(DataTable.RowHighlighted, "#runs")
    def show_run(self, event: DataTable.RowHighlighted) -> None:
        if event.row_key is None or event.row_key.value is None:
            return
        from .replay import load
        try:
            recording = load(Path(event.row_key.value))
        except (OSError, ValueError) as exc:
            self.query_one("#replay-summary", Static).update(Text(str(exc), style="red"))
            return
        s = recording.summary()
        text = Text(f"{Path(s['source']).name}\n", style="bold")
        text.append(f"{s['samples']} samples · {s['events']} events · {s['duration_s']:.0f} s\n")
        for key, label, scale, unit in (("max_altitude_m", "max altitude", 1e-3, "km"),
                                        ("max_speed_mps", "max speed", 1, "m/s"),
                                        ("max_dynamic_pressure_pa", "max q", 1e-3, "kPa")):
            if s[key] is not None:
                text.append(f"{label:<13} {s[key] * scale:.1f} {unit}\n")
        text.append("\nTimeline\n", style="bold")
        for t, phase in recording.timeline():
            text.append(f"  T+{t:7.1f} s  {phase}\n", style="cyan")
        notable = [e for e in recording.events
                   if e.kind in ("state", "milestone", "checkpoint") or e.level in ("warning", "error")]
        if notable:
            text.append("\nEvents\n", style="bold")
            for e in notable[-40:]:
                text.append(f"  {time.strftime('%H:%M:%S', time.localtime(e.time))}  {e.message}\n",
                            style=LEVEL_STYLE.get(e.level, ""))
        self.query_one("#replay-summary", Static).update(text)
        self.replay_samples, self.replay_index = recording.samples, 0
        self._show_sample()

    @on(DataTable.RowSelected, "#runs")
    def open_run(self, event: DataTable.RowSelected) -> None:
        if event.row_key.value:
            self.watch_run(Path(event.row_key.value))

    def _show_sample(self) -> None:
        widget = self.query_one("#sample", Static)
        if not self.replay_samples:
            widget.update(Text("no telemetry samples", style="dim"))
            return
        s = self.replay_samples[self.replay_index]
        widget.update(Text(
            f"sample {self.replay_index + 1}/{len(self.replay_samples)}  phase {s.get('phase', '—')}  "
            f"alt {_fmt(s.get('altitude'), 1e-3, 'km', 2)}  v {_fmt(s.get('speed'), unit='m/s')}  "
            f"Ap {_fmt(s.get('apoapsis'), 1e-3, 'km')}  range {_fmt(s.get('rangeToSite'), 1e-3, 'km')}  "
            "(←/→ step, shift for ×10)"))

    def on_key(self, event: Any) -> None:
        if self.query_one("#tabs", TabbedContent).active != "replay" or not self.replay_samples:
            return
        step = {"left": -1, "right": 1, "shift+left": -10, "shift+right": 10}.get(event.key)
        if step:
            self.replay_index = max(0, min(len(self.replay_samples) - 1, self.replay_index + step))
            self._show_sample()
            event.stop()

    # --- actions ------------------------------------------------------------------------
    def action_tab(self, tab: str) -> None:
        if tab == "replay":
            self.refresh_runs()
        self.query_one("#tabs", TabbedContent).active = tab

    def action_watch_latest(self) -> None:
        runs = list_runs(self.runs_root)
        if runs:
            self.watch_run(runs[-1])
        else:
            self.notify("No runs recorded yet.")

    def action_clear_log(self) -> None:
        self.query_one("#eventlog", RichLog).clear()

    def action_toggle_verbose(self) -> None:
        self.verbose = not self.verbose
        self.notify(f"Raw program output {'shown' if self.verbose else 'hidden'} (new lines).")

    async def action_quit_safe(self) -> None:
        running = self.controller is not None and self.controller.status.state == MissionState.RUNNING.value
        if running and time.monotonic() - self.quit_armed_at > 3.0:
            self.quit_armed_at = time.monotonic()
            self.notify("A mission is running: press q again to abort it and quit.", severity="warning")
            return
        if running:
            assert self.controller is not None
            self.controller.abort()
            deadline = time.monotonic() + 30.0
            while self.controller.status.state == MissionState.RUNNING.value and time.monotonic() < deadline:
                await asyncio.sleep(0.2)
            if self.controller.status.state == MissionState.RUNNING.value:
                self.notify("Mission is still stopping; the TUI remains open until controls are released.",
                            severity="warning")
                return
        self.exit(0)


def run_tui(profile: str | None = None, overrides: list[str] | None = None) -> int:
    return KSLApp(profile, overrides).run() or 0
