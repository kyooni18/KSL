"""Persistent flight operations console, shared with the CLI operation API.

The live vessel monitor is read-only and independent of recorded runs. Flight
operations require a reviewed request; deorbit engagement additionally requires
an explicit command for the currently displayed plan revision. Watching and
operator commands work across processes through the same run directory.
"""

from __future__ import annotations

import json
import asyncio
import time
import shlex
from pathlib import Path
from typing import Any

from rich.text import Text
from textual import on
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.widgets import (Button, Collapsible, DataTable, Footer, Header, Input, Label, RichLog, Select, Static,
                             Switch, TabbedContent, TabPane)

from .config import DEFAULT_PROFILE, DEFAULTS, PROFILE_KINDS, ConfigError, available_profiles, config_from_snapshot, load_config
from .controls import FILE_NAME as CONTROLS_FILE, Controls
from .controller import MissionController
from .phases import SEQUENCE, MissionState, Phase, PhaseTransitionError
from .runlog import Event, MissionStatus, list_runs, read_status
from .scenarios import SCENARIOS
from .operations import BY_KEY, OPERATIONS, OperationRequest, blocked_reason
from .commands import Commands
from .ownership import owner
from .plan_review import invalid_reason

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
            title = {Phase.MM304: "Entry", Phase.MM305: "TAEM", Phase.FINAL_APPROACH: "Final"}.get(phase, phase.title)
            if i:
                text.append(" › ", style="dim")
            if phase == current:
                text.append(f" {title} ", style="bold white on red" if failed else "bold black on cyan")
            elif phase.value in done:
                text.append(title, style="green")
            else:
                text.append(title, style="dim")
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
    TITLE = "KSL — Flight Operations"
    CSS = """
    Screen { layout: vertical; }
    #phases { height: auto; padding: 0 1; margin-bottom: 1; }
    #panels { height: auto; max-height: 9; }
    #compact-status { display: none; height: auto; }
    .panel { border: round $primary-darken-2; padding: 0 1; width: 1fr; height: auto; min-height: 6; }
    #banner { height: auto; max-height: 3; padding: 0 1; }
    #eventlog { height: 8; border: round $primary-darken-2; }
    #session-actions, #session-settings, .actionrow { height: auto; }
    #session-actions Button { margin-right: 1; }
    #session-settings Select { width: 1fr; }
    #workspace { height: auto; min-height: 21; }
    #operation-list { width: 2fr; height: auto; padding: 0 1; }
    #operation-settings { width: 3fr; height: auto; max-height: 21; padding: 0 1; border: round $primary-darken-2; }
    #operations { height: 11; }
    #availability, #operation-title, #connection, #command-result { height: auto; }
    #availability { min-height: 2; }
    .actionrow Button { margin-right: 1; }
    .actionrow Input { width: 1fr; }
    .context-fields, #deorbit-review { height: auto; }
    #deorbit-plan { height: auto; border: round $warning; padding: 0 1; }
    #operation-settings Label { margin-top: 1; }
    #settings-hint { height: auto; color: $text-muted; margin-top: 1; }
    .compact #workspace, .compact #panels, .compact #session-settings { layout: vertical; }
    .compact #operation-list, .compact #operation-settings { width: 1fr; }
    .compact #workspace { height: 40; }
    .compact #session-actions Button { min-width: 12; width: 1fr; }
    .compact .panel { width: 1fr; }
    .compact #panels, .compact #phases, .compact #banner { display: none; }
    .compact #compact-status { display: block; }
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
    .quick { padding: 1 2; }
    .quick-status { height: auto; border: round $primary-darken-2; padding: 0 1; margin-bottom: 1; }
    .quick-row { height: auto; margin-bottom: 1; }
    .quick-row Label { width: 18; margin-top: 1; }
    .quick-row Select, .quick-row Input { width: 1fr; max-width: 60; }
    .quick-buttons { height: auto; margin-top: 1; }
    .quick-buttons Button { margin-right: 2; min-width: 16; }
    .quick-msg { height: auto; margin-top: 1; }
    .quick-more { height: auto; border: round $primary-darken-2; padding: 0 1; }
    """
    BINDINGS = [
        Binding("1", "tab('launch')", "Launch"),
        Binding("2", "tab('landing')", "Landing"),
        Binding("d", "tab('dashboard')", "Console"),
        Binding("m", "tab('mission')", "Settings"),
        Binding("r", "tab('replay')", "Runs"),
        Binding("ctrl+enter", "start", "Execute reviewed operation"),
        Binding("ctrl+a", "abort", "Abort"),
        Binding("w", "watch_latest", "Watch latest"),
        Binding("ctrl+l", "clear_log", "Clear log"),
        Binding("v", "toggle_verbose", "Raw output"),
        Binding("f", "control('warpNow')", "Warp now"),
        Binding("x", "control('warpStop')", "Stop warp"),
        Binding("g", "control('autoWarp')", "Coast warp"),
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
        self.selected_operation = "launch"
        self.selected_run: Path | None = None
        self.vessel_info: dict[str, Any] | None = None
        self.connection_error = ""
        self.monitor_vessel = False
        self.probing = False
        self.reviewed_signature: str | None = None
        self.displayed_plan_revision: str | None = None
        self.pending_commands: dict[str, Path] = {}
        self.quick_armed: tuple[str, str, float] | None = None

    # --- layout -------------------------------------------------------------------
    def compose(self) -> ComposeResult:
        base = self._base()
        ascent, pr = base["ascent"], base["profile"]
        yield Header(show_clock=True)
        with TabbedContent(initial="launch", id="tabs"):
            with TabPane("Launch", id="launch"):
                with VerticalScroll(classes="quick"):
                    yield Static(id="q-launch-status", classes="quick-status", markup=False)
                    for label, widget in (
                        ("Mission type", Select([(KIND_LABELS[k], k) for k in PROFILE_KINDS], value=pr["kind"],
                                                allow_blank=False, id="q-kind")),
                        ("Target altitude km", Input(f"{ascent['targetAltitudeKm']:g}", id="q-altitude")),
                        ("Inclination deg", Input(f"{ascent['inclinationDeg']:g}", id="q-inclination")),
                    ):
                        with Horizontal(classes="quick-row"):
                            yield Label(label)
                            yield widget
                    with Horizontal(classes="quick-row"):
                        yield Label("Settings")
                        yield Switch(False, id="q-launch-settings")
                    with Vertical(id="q-launch-more", classes="quick-more"):
                        with Horizontal(classes="quick-row"):
                            yield Label("Ascent guidance")
                            yield Select([("Legacy STS", "legacy"), ("Intelligent · experimental", "intelligent")],
                                         value=ascent["guidanceMode"], allow_blank=False, id="q-ascent-mode")
                        with Horizontal(classes="quick-row"):
                            yield Label("Plane target")
                            yield Input(pr["planeTarget"], placeholder="e.g. Minmus or Station", id="q-plane-target")
                        with Horizontal(classes="quick-row"):
                            yield Label("Continue to orbit ops")
                            yield Switch(False, id="q-automatic")
                        yield Static("Full configuration: Console [d] and Settings [m].", markup=False)
                    with Horizontal(classes="quick-buttons"):
                        yield Button("LAUNCH", id="q-launch", variant="success")
                        yield Button("ABORT", id="q-launch-abort", variant="error")
                    yield Static(id="q-launch-msg", classes="quick-msg", markup=False)
            with TabPane("Landing", id="landing"):
                with VerticalScroll(classes="quick"):
                    yield Static(id="q-land-status", classes="quick-status", markup=False)
                    for label, widget in (
                        ("Mode", Select([("Live KSP", "live"), ("Simulation", "sim")], value="live", allow_blank=False,
                                        id="q-land-mode")),
                        ("Start from", Select([("Auto (from vessel state)", "auto"), ("Deorbit plan", "deorbit"),
                                               ("Entry · MM304", "entry"), ("TAEM · MM305", "taem"),
                                               ("Final approach", "final")], value="auto", allow_blank=False,
                                              id="q-land-from")),
                    ):
                        with Horizontal(classes="quick-row"):
                            yield Label(label)
                            yield widget
                    with Horizontal(classes="quick-row"):
                        yield Label("Settings")
                        yield Switch(False, id="q-land-settings")
                    with Vertical(id="q-land-more", classes="quick-more"):
                        with Horizontal(classes="quick-row"):
                            yield Label("Landing config")
                            yield Input(base["landing"]["backendConfiguration"], id="q-landing-config")
                        with Horizontal(classes="quick-row"):
                            yield Label("Strict plan")
                            yield Switch(bool(base["landing"]["requireStrictPlan"]), id="q-strict-plan")
                        with Horizontal(classes="quick-row"):
                            yield Label("Sim scenario")
                            yield Select([(f"{s.name} — {s.start.title}", s.name) for s in SCENARIOS.values()],
                                         value=base["simulation"]["scenario"] if base["simulation"]["scenario"] in SCENARIOS else "l5-mm305",
                                         allow_blank=False, id="q-scenario")
                        yield Static("Full configuration: Console [d] and Settings [m].", markup=False)
                    with Horizontal(classes="quick-buttons"):
                        yield Button("LAND", id="q-land", variant="success")
                        yield Button("EXECUTE DEORBIT BURN", id="q-execute-plan", variant="warning")
                        yield Button("ABORT", id="q-land-abort", variant="error")
                    yield Static(id="q-land-msg", classes="quick-msg", markup=False)
            with TabPane("Console", id="dashboard"):
                with Horizontal(id="session-actions"):
                    yield Button("Connect / refresh", id="connect-vessel")
                    yield Button("Watch latest run", id="watch-latest")
                    yield Button("Abort / release", id="abort-btn", variant="error")
                yield Static("Disconnected · Connect to inspect the active vessel. No flight controls are sent.", id="connection", markup=False)
                yield PhaseStrip(id="phases")
                yield Static(id="banner", markup=False)
                with Horizontal(id="panels"):
                    yield KeyValue(id="vehicle-panel", classes="panel")
                    yield KeyValue(id="mission-panel", classes="panel")
                yield Static(id="compact-status", markup=False)
                with VerticalScroll(id="console"):
                    with Horizontal(id="session-settings"):
                        yield Select([("Dry run · no flight", "dry-run"), ("Live KSP", "live"),
                                      ("Simulation", "sim")], value=DEFAULT_MODE, allow_blank=False, id="mode")
                        names = list(dict.fromkeys([self.profile, *available_profiles()]))
                        yield Select([(f"Profile: {name}", name) for name in names], value=self.profile,
                                     allow_blank=False, id="mission-profile")
                        yield Select([("Operator-directed", "directed"), ("Automatic mission", "automatic")],
                                     value="directed", allow_blank=False, id="execution-style")
                    with Horizontal(id="workspace"):
                        with Vertical(id="operation-list"):
                            yield DataTable(id="operations", cursor_type="row", zebra_stripes=True)
                            yield Static(id="availability", markup=False)
                            with Horizontal(classes="actionrow"):
                                yield Button("Review", id="validate", variant="primary")
                                yield Button("Execute", id="start-btn", variant="success", disabled=True)
                        with VerticalScroll(id="operation-settings"):
                            yield Static(id="operation-title", markup=False)
                            with Vertical(id="orbit-target-fields", classes="context-fields"):
                                yield Label("Target altitude (km) / inclination (deg)")
                                with Horizontal(classes="actionrow"):
                                    yield Input(f"{ascent['targetAltitudeKm']:g}", id="altitude")
                                    yield Input(f"{ascent['inclinationDeg']:g}", id="inclination")
                                yield Label("Orbit profile")
                                yield Select([(KIND_LABELS[k], k) for k in PROFILE_KINDS], value=pr["kind"],
                                             allow_blank=False, id="kind")
                            with Vertical(id="launch-fields", classes="context-fields"):
                                yield Label("Ascent guidance")
                                yield Select([("Legacy STS", "legacy"), ("Intelligent · experimental", "intelligent")],
                                             value=ascent["guidanceMode"], allow_blank=False, id="ascent-mode")
                            with Vertical(id="landing-fields", classes="context-fields"):
                                yield Label("Landing configuration / model profile")
                                yield Input(base["landing"]["backendConfiguration"], id="landing-config")
                                yield Input(base["landing"]["modelProfile"], id="landing-model")
                                with Horizontal(classes="switchrow"):
                                    yield Label("Require strict plan qualification")
                                    yield Switch(bool(base["landing"]["requireStrictPlan"]), id="strict-plan")
                            with Vertical(id="scenario-fields", classes="context-fields"):
                                yield Label("Simulation scenario")
                                yield Select([(f"{s.name} — {s.start.title}", s.name) for s in SCENARIOS.values()],
                                             value=base["simulation"]["scenario"] if base["simulation"]["scenario"] in SCENARIOS else "l5-mm305",
                                             allow_blank=False, id="scenario")
                            with Vertical(id="continue-fields", classes="context-fields"):
                                yield Static("Select a run in Runs; its saved configuration is restored.", id="resume-source", markup=False)
                                with Horizontal(classes="switchrow"):
                                    yield Label("Load checkpoint's KSP save first")
                                    yield Switch(False, id="use-save")
                            yield Static("More profile settings are available in Settings [m].", id="settings-hint", markup=False)
                            yield Static("Select an operation, then Review.", id="plan", markup=False)
                    with Vertical(id="deorbit-review"):
                        yield Static(id="deorbit-plan", markup=False)
                        with Horizontal(classes="actionrow"):
                            yield Button("Execute reviewed plan", id="execute-plan", variant="success")
                            yield Button("Replan", id="replan")
                    yield Static(id="command-result", markup=False)
                    with Collapsible(title="Guidance and live orbit controls", collapsed=True):
                        with Horizontal():
                            yield KeyValue(id="guidance-panel", classes="panel")
                            yield KeyValue(id="controls-panel", classes="panel")
                        with Horizontal(classes="actionrow"):
                            yield Button("Warp now", id="warp-now")
                            yield Button("Stop warp", id="warp-stop")
                            yield Button("Trim now", id="trim-now")
                    yield RichLog(id="eventlog", wrap=True, markup=False, max_lines=5000)
            with TabPane("Settings", id="mission"):
                with VerticalScroll(id="setup"):
                    yield Label("Profile settings for the next operation; running guidance keeps its configuration.")
                    yield Label("Vehicle configuration (inspect in Vehicle)")
                    yield Input(base["intelligentAscent"]["vehicleConfig"], id="vehicle-config")
                    yield Label("Intelligent periapsis (km; 0 = target altitude)")
                    yield Input(str(base["intelligentAscent"]["targetPeriapsisKm"]), id="intelligent-periapsis")
                    yield Label("Plane / rendezvous target")
                    yield Input(pr["planeTarget"], placeholder="e.g. Minmus or Station", id="plane-target")
                    yield Label("GEO longitude (deg east)")
                    yield Input(f"{pr['geoLongitudeDeg']:g}", id="geo-longitude")
                    yield Label("GTO perigee (km)")
                    yield Input(f"{pr['gtoPerigeeKm']:g}", id="gto-perigee")
                    yield Label("GEO circularization")
                    yield Select([("Auto", "auto"), ("Always", "always"), ("Never", "never")],
                                 value=pr["geoCircularize"], allow_blank=False, id="geo-circularize")
                    for wid, label, val in (
                        ("launch-window", "Wait for plane / rendezvous launch window", pr["launchWindow"]),
                        ("target-longitude", "Phase GTO apogee onto GEO longitude", pr["targetLongitude"]),
                        ("auto-warp", "Allow automatic coast time warp (launch and landing)", base["operations"]["autoWarp"]),
                        ("orbit-trim", "Include orbit operations in automatic missions", base["orbit"]["trim"]),
                        ("hold-trim", "Automatic missions wait for Trim now", base["orbit"]["holdForTrigger"]),
                    ):
                        with Horizontal(classes="switchrow"):
                            yield Label(label)
                            yield Switch(bool(val), id=wid)
                    yield Label("Maximum trim burns")
                    yield Input(str(base["orbit"]["maxBurns"]), id="max-burns")
                    yield Label("Other overrides (shell-style KEY=VALUE; quote JSON and values with spaces)")
                    yield Input(shlex.join(self.overrides), id="overrides")
                    yield Label("Save settings as a reusable mission profile")
                    yield Input("Configuration/missions/user-ascent.json", id="save-profile-path")
                    yield Button("Save mission setup", id="save-profile")
            with TabPane("Vehicle", id="vehicle-editor"):
                with VerticalScroll():
                    yield Button("Inspect active vessel (read-only)", id="inspect-vehicle")
                    yield Static("Load a vessel, then inspect its attachment tree.", id="vehicle-tree", markup=False)
                    yield Label("Component")
                    yield Select([], id="component")
                    yield Label("Role")
                    from .ascent.intelligent.vehicle import ROLES
                    yield Select([(role, role) for role in ROLES], value="structure", allow_blank=False, id="component-role")
                    yield Label("Approve depleted tank / controlled booster release at this decoupler")
                    yield Switch(False, id="approve-separator")
                    yield Button("Apply component correction", id="apply-component")
                    yield Input("Configuration/vehicles/user-ascent.json", id="vehicle-save-path")
                    yield Button("Save vehicle and use for next mission", id="save-vehicle")
            with TabPane("Runs", id="replay"):
                with Horizontal(id="replay-top"):
                    yield DataTable(id="runs", cursor_type="row", zebra_stripes=True)
                    with VerticalScroll(id="replay-detail"):
                        yield Static(id="replay-summary", markup=False)
                        yield Button("Watch selected run", id="watch-selected")
                        yield Button("Continue checkpoint", id="continue-selected")
                yield Static("←/→ step samples · enter: watch run · Continue restores its saved configuration", id="sample", markup=False)
        yield Footer()

    def on_mount(self) -> None:
        self.query_one("#eventlog", RichLog).write(Text(
            "Select an operation → Review → Execute. Keys: d/m/r tabs · Ctrl+Enter execute reviewed operation · "
            "Ctrl+A abort (press twice) · w watch latest run · v raw program output · "
            "f warp now · x stop warp · g auto-warp · o orbit trim · h hold · t trim now · q quit", style="dim"))
        operations = self.query_one("#operations", DataTable)
        operations.add_column("Operation", key="title")
        operations.add_column("Status", key="status")
        for op in OPERATIONS:
            operations.add_row(op.title, "", key=op.key)
        operations.move_cursor(row=1)
        table = self.query_one("#runs", DataTable)
        table.add_columns("Run", "Mode", "State", "Phase", "Profile")
        self.refresh_runs()
        self.render_dashboard(None)
        self.refresh_operations()
        self.query_one("#q-launch-more").display = False
        self.query_one("#q-land-more").display = False
        self.monitor_vessel = True  # the quick tabs need the vessel state; probing is read-only
        self.set_interval(0.5, self.poll)
        self.set_interval(3, self.schedule_probe)

    def on_resize(self, event) -> None:
        self.screen.set_class(event.size.width < 100, "compact")

    @on(Button.Pressed, "#connect-vessel")
    def connect_vessel(self) -> None:
        self.monitor_vessel = True
        self.schedule_probe()

    def schedule_probe(self) -> None:
        if self.monitor_vessel and not self.probing and self.is_running:
            self.probing = True
            self.run_worker(self.refresh_vessel(), group="vessel-probe")

    async def refresh_vessel(self) -> None:
        from .krpc_link import probe
        try:
            config = self._connection_config()
            if config.validate():
                raise ConfigError("; ".join(config.validate()))
            info = await asyncio.to_thread(probe, config)
            # A reply from the old endpoint must not become the new profile's
            # live state when the operator edits settings during the request.
            current = self._connection_config()
            if current["connection"] == config["connection"]:
                self.vessel_info = info
                self.connection_error = ""
        except Exception as exc:
            self.connection_error = str(exc)
            self.vessel_info = None
        finally:
            self.probing = False
        self.poll()

    def _connection_config(self):
        # Monitoring is independent of partly edited operation form fields.
        if self.selected_operation == "continue" and self.selected_run:
            saved = json.loads((self.selected_run / "config.json").read_text())
            return config_from_snapshot(saved["profile"], saved["configuration"])
        return load_config(self.profile, shlex.split(self.query_one("#overrides", Input).value))

    # --- dashboard ------------------------------------------------------------------
    def poll(self) -> None:
        # Textual stops the app before removing dashboard widgets. A queued
        # timer callback during teardown must not query that disappearing tree.
        if not self.is_running:
            return
        status, events = self.watcher.poll() if self.watcher else (None, [])
        log = self.query_one("#eventlog", RichLog)
        for event in events:
            if event.kind == "program" and not self.verbose:
                continue
            stamp = time.strftime("%H:%M:%S", time.localtime(event.time))
            style = KIND_STYLE.get(event.kind, "") if event.level == "info" else LEVEL_STYLE.get(event.level, "")
            log.write(Text(f"{stamp}  {event.message}", style=style))
        self.render_dashboard(status)
        self.render_vessel()
        self.refresh_operations()
        self.render_plan_review(status)
        self.render_quick(status)
        for request_id, run_dir in list(self.pending_commands.items()):
            try:
                command = Commands(run_dir).read(request_id)
                self.query_one("#command-result", Static).update(
                    Text(f"{command['action']}: {command['state']} — {command['message']}",
                         style="red" if command["state"] == "rejected" else "cyan"))
                if command["state"] in ("completed", "rejected"):
                    del self.pending_commands[request_id]
            except (OSError, ValueError):
                continue

    def render_vessel(self) -> None:
        from .krpc_link import suggest_phase
        info = self.vessel_info
        status = self.watcher.status if self.watcher else None
        self.query_one("#compact-status", Static).update(
            f"Vessel: {(info or {}).get('vessel', '—')} · alt {_fmt((info or {}).get('altitude'), 1e-3, 'km')} · "
            f"Ap/Pe {_fmt((info or {}).get('apoapsis'), 1e-3)}/{_fmt((info or {}).get('periapsis'), 1e-3)} km\n"
            + (f"Run: {status.operation} · {status.state} · {status.phase}" if status else "Run: none · guidance disengaged"))
        if info:
            age = time.time() - info["observedAt"]
            message = f"KSP · {info.get('scene')} · {'paused' if info.get('paused') else 'running'} · updated {age:.0f}s ago"
            if age > STALE_SECONDS:
                message += " · STALE"
            suggested = suggest_phase(info)
            if suggested:
                message += f" · suggested entry: {Phase.parse(suggested).title}"
            self.query_one("#connection", Static).update(Text(message, style="yellow" if age > STALE_SECONDS else "green"))
            self.query_one("#vehicle-panel", KeyValue).show("Active vessel · read-only", [
                ("vessel", str(info.get("vessel", "—"))),
                ("body / situation", f"{info.get('body', '—')} / {info.get('situation', '—')}"),
                ("altitude / speed", f"{_fmt(info.get('altitude'), 1e-3, 'km')} / {_fmt(info.get('speed'), unit='m/s')}"),
                ("Ap / Pe", f"{_fmt(info.get('apoapsis'), 1e-3, 'km')} / {_fmt(info.get('periapsis'), 1e-3, 'km')}"),
                ("inclination", _fmt(info.get("inclination"), unit="deg", digits=3)),
                ("sink / dynamic q", f"{_fmt(info.get('verticalSpeed'), unit='m/s')} / "
                                     f"{_fmt(info.get('dynamicPressure'), 1e-3, 'kPa')}"),
            ])
        else:
            message = self.connection_error or ("Connecting…" if self.probing else
                       "Disconnected · Connect to inspect the active vessel. No flight controls are sent.")
            self.query_one("#connection", Static).update(Text(message, style="yellow" if self.connection_error else "dim"))

    def _availability(self, request: OperationRequest) -> str | None:
        busy = self.controller is not None and self.controller.status.finished_at is None
        if request.mode == "live":
            config = request.resolve()[0]
            busy = busy or owner(config) is not None
            if (not request.use_save and self.vessel_info and self.vessel_info.get("connection")
                    and self.vessel_info["connection"] != config["connection"]):
                return "Connect / refresh using this operation's kRPC endpoint"
        return blocked_reason(BY_KEY[request.key], self.vessel_info, request.mode, busy, request.source, request.use_save)

    def refresh_operations(self) -> None:
        mode = str(self.query_one("#mode", Select).value)
        busy = self.controller is not None and self.controller.status.finished_at is None
        guidance_owner = None
        if mode == "live":
            try:
                config = self._connection_config()
                guidance_owner = owner(config)
                busy = busy or guidance_owner is not None
            except (ConfigError, ValueError, OSError, KeyError):
                pass
        table = self.query_one("#operations", DataTable)
        for op in OPERATIONS:
            reason = blocked_reason(op, self.vessel_info, mode, busy, self.selected_run,
                                    self.query_one("#use-save", Switch).value if op.key == "continue" else False)
            table.update_cell(op.key, "status", Text("Blocked" if reason else "Ready", style="dim" if reason else "green"))
        op = BY_KEY[self.selected_operation]
        try:
            reason = self._availability(self._request())
        except (ConfigError, ValueError, KeyError, OSError, PhaseTransitionError) as exc:
            reason = str(exc)
        if guidance_owner:
            reason = f"Guidance owner: {guidance_owner['run']} · select it in Runs to watch or abort"
        self.query_one("#availability", Static).update(Text(reason or "Ready to review. Execution begins only when you choose Execute.",
                                                          style="yellow" if reason else "green"))
        self.query_one("#operation-title", Static).update(Text(op.title + "\n" + op.description, style="bold"))
        for wid, show in {
            "orbit-target-fields": op.key in ("preflight", "launch", "orbit"),
            "launch-fields": op.key in ("preflight", "launch"),
            "landing-fields": op.key in ("deorbit", "entry", "taem", "final", "simulate"),
            "scenario-fields": op.key == "simulate", "continue-fields": op.key == "continue",
        }.items():
            self.query_one(f"#{wid}").display = show
        self.query_one("#execution-style", Select).disabled = op.key not in ("launch", "orbit")
        self.query_one("#validate", Button).disabled = reason is not None
        self.query_one("#start-btn", Button).disabled = reason is not None or self.reviewed_signature is None

    @on(DataTable.RowHighlighted, "#operations")
    def select_operation(self, event: DataTable.RowHighlighted) -> None:
        # A queued table event can arrive after the screen starts unmounting.
        if not self.query("#start-btn"):
            return
        if event.row_key and event.row_key.value:
            self.selected_operation = str(event.row_key.value)
            self.invalidate_review()
            self.refresh_operations()

    def invalidate_review(self) -> None:
        self.reviewed_signature = None
        buttons = self.query("#start-btn")
        if not buttons:
            return
        buttons.first(Button).disabled = True
        self._show_plan("Select an operation, then Review.")

    @on(Input.Changed)
    @on(Switch.Changed)
    def settings_changed(self, event) -> None:
        if self.is_mounted:
            self.invalidate_review()

    @on(Select.Changed)
    def selection_changed(self, event: Select.Changed) -> None:
        if self.is_mounted and event.select.id not in ("component", "component-role", "mission-profile", "mode"):
            self.invalidate_review()

    def render_dashboard(self, status: MissionStatus | None) -> None:
        self.query_one("#phases", PhaseStrip).show(status)
        banner = self.query_one("#banner", Static)
        mission = self.query_one("#mission-panel", KeyValue)
        vehicle = self.query_one("#vehicle-panel", KeyValue)
        guidance = self.query_one("#guidance-panel", KeyValue)
        if status is None:
            banner.update(Text("Select a flight operation below, or watch a recorded run.", style="dim"))
            mission.show("Watched run", [("state", "No run selected"), ("guidance", "Not commanding")])
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
        mission.show("Watched run", [
            ("run", status.run_id), ("profile", f"{status.profile} ({status.mode})"),
            ("state", status.state.upper() + ("  (this TUI)" if local else "  (watching)")),
            ("phase", Phase(status.phase).title), ("range", f"{status.start_phase} → {status.stop_phase}"),
            ("operation", f"{status.operation} · {status.operation_stage}"),
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
        panel.show("Controls", [("coast warp [g]", on(c["autoWarp"])), ("orbit trim [o]", on(c["trim"])),
                                ("hold [h]", on(c["holdTrim"])), ("warp now [f]", str(c["warpNow"])),
                                ("trim now [t]", str(c["trimNow"]))])

    def action_control(self, name: str) -> None:
        controls = self._controls()
        status = self.watcher.status if self.watcher else None
        if (controls is None or status is None or status.state != "running" or status.mode != "live"
                or status.phase != "orbit" or status.program != "orbit-trim"
                or time.time() - status.updated_at > STALE_SECONDS
                or not (self.watcher.run_dir / CONTROLS_FILE).is_file()):
            self.notify("These controls apply to a running live orbit operation.", severity="warning")
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
        self.selected_run = run_dir
        self.abort_armed_at = 0
        self.watcher = RunWatcher(run_dir)
        self.query_one("#eventlog", RichLog).write(Text(f"— watching {run_dir.name} —", style="bold"))
        self.poll()
        tabs = self.query_one("#tabs", TabbedContent)
        if tabs.active not in ("launch", "landing"):
            tabs.active = "dashboard"
            self.query_one("#eventlog", RichLog).focus()  # keep focus out of the hidden Mission pane

    @on(Button.Pressed, "#watch-latest")
    def watch_latest_button(self) -> None:
        self.action_watch_latest()

    @on(Button.Pressed, "#warp-now")
    def warp_now_button(self) -> None:
        self.action_control("warpNow")

    @on(Button.Pressed, "#warp-stop")
    def warp_stop_button(self) -> None:
        self.action_control("warpStop")

    @on(Button.Pressed, "#trim-now")
    def trim_now_button(self) -> None:
        self.action_control("trimNow")

    def render_plan_review(self, status: MissionStatus | None) -> None:
        reviewing = status is not None and status.state == "running" and status.operation_stage == "review"
        self.query_one("#deorbit-review").display = reviewing
        previous_revision = self.displayed_plan_revision
        self.displayed_plan_revision = None
        if not reviewing or self.watcher is None:
            return
        try:
            review = json.loads((self.watcher.run_dir / "plan-review.json").read_text())
            saved = json.loads((self.watcher.run_dir / "config.json").read_text())
            config = config_from_snapshot(saved["profile"], saved["configuration"])
            plan = review["plan"]
            if review["revision"] != status.plan_revision:
                raise ValueError("Waiting for the current plan revision")
            self.displayed_plan_revision = review["revision"]
            if previous_revision != self.displayed_plan_revision:
                self.call_after_refresh(self.query_one("#deorbit-plan").scroll_visible, animate=False)
            reason = None
            if time.time() - status.updated_at > STALE_SECONDS:
                reason = "Controller status is stale; command delivery cannot be confirmed"
            elif self.vessel_info and time.time() - self.vessel_info["observedAt"] <= STALE_SECONDS:
                reason = invalid_reason(plan, review["reference"], self.vessel_info,
                                        float(review["minimumLead"]))
            from .backend import validate_plan_qualification
            try:
                warning = validate_plan_qualification(plan, config["landing"]["requireStrictPlan"])
            except Exception as exc:
                reason, warning = str(exc), None
            text = Text("Deorbit plan · awaiting execution · guidance is disengaged\n", style="bold yellow")
            text.append(f"Burn UT {_fmt(plan.get('burnUT'), digits=1)} · Δv {_fmt(plan.get('deltaV'), unit='m/s')}\n")
            text.append(f"Entry qualified: {plan.get('executionQualified')} · Full-route capture: {plan.get('targetCaptureAchieved')}\n")
            text.append(f"Predicted miss: {_fmt(plan.get('predictedClosestDistance'), unit='m')} · "
                        f"Entry FPA: {_fmt(plan.get('predictedEntryFlightPathAngle'), unit='deg')}\n")
            if warning:
                text.append(warning + "\n", style="yellow")
            text.append(reason or "Execution rechecks the active vessel, orbit, mass and burn window.",
                        style="red" if reason else "dim")
            self.query_one("#deorbit-plan", Static).update(text)
            self.query_one("#execute-plan", Button).disabled = reason is not None
            self.query_one("#replan", Button).disabled = time.time() - status.updated_at > STALE_SECONDS
        except (OSError, ValueError, KeyError, ConfigError) as exc:
            self.query_one("#deorbit-plan", Static).update(Text(str(exc), style="yellow"))
            self.query_one("#execute-plan", Button).disabled = True

    def submit_command(self, action: str) -> None:
        if self.watcher is None:
            return
        try:
            request = Commands(self.watcher.run_dir).submit(action, self.displayed_plan_revision)
            self.pending_commands[request["id"]] = self.watcher.run_dir
            self.query_one("#command-result", Static).update(Text(
                f"{action}: requested — waiting for controller acknowledgement", style="cyan"))
        except (OSError, ValueError) as exc:
            self.notify(str(exc), severity="error")

    @on(Button.Pressed, "#execute-plan")
    def execute_plan(self) -> None:
        self.submit_command("execute-plan")

    @on(Button.Pressed, "#replan")
    def replan(self) -> None:
        self.submit_command("replan")

    # --- simple launch / landing ----------------------------------------------------
    QUICK_FIELDS = {"q-altitude": "altitude", "q-inclination": "inclination", "q-kind": "kind",
                    "q-ascent-mode": "ascent-mode", "q-plane-target": "plane-target",
                    "q-landing-config": "landing-config", "q-strict-plan": "strict-plan", "q-scenario": "scenario"}

    @on(Switch.Changed, "#q-launch-settings")
    @on(Switch.Changed, "#q-land-settings")
    def toggle_quick_settings(self, event: Switch.Changed) -> None:
        self.query_one("#" + event.switch.id.replace("settings", "more")).display = event.value

    def render_quick(self, status: MissionStatus | None) -> None:
        info = self.vessel_info or {}
        vessel = (f"Vessel: {info.get('vessel', '—')} · {info.get('situation', '—')} · "
                  f"alt {_fmt(info.get('altitude'), 1e-3, 'km')} · "
                  f"Ap/Pe {_fmt(info.get('apoapsis'), 1e-3)}/{_fmt(info.get('periapsis'), 1e-3)} km"
                  if info else (self.connection_error or "Vessel: not connected"))
        run = (f"Run: {status.operation} · {status.state.upper()} · {Phase(status.phase).title}"
               + (f" · {status.status_message}" if status.status_message else "") if status else "Run: none")
        text = Text(vessel + "\n" + run, style="red" if status and status.error else "")
        self.query_one("#q-launch-status", Static).update(text)
        self.query_one("#q-land-status", Static).update(text)
        ready = self.displayed_plan_revision is not None and not self.query_one("#execute-plan", Button).disabled
        self.query_one("#q-execute-plan", Button).display = self.displayed_plan_revision is not None
        self.query_one("#q-execute-plan", Button).disabled = not ready
        running = status is not None and status.state == "running"
        self.query_one("#q-launch-abort", Button).disabled = not running
        self.query_one("#q-land-abort", Button).disabled = not running

    def _landing_operation(self, mode: str) -> str:
        if mode == "sim":
            return "simulate"
        chosen = str(self.query_one("#q-land-from", Select).value)
        if chosen != "auto":
            return chosen
        from .krpc_link import suggest_phase
        suggested = suggest_phase(self.vessel_info) if self.vessel_info else None
        operation = {"orbit": "deorbit", "mm304": "entry", "mm305": "taem"}.get(suggested or "")
        if operation is None:
            raise ConfigError("Cannot pick a landing phase from the vessel state; choose Start from")
        return operation

    def quick_start(self, tab: str) -> None:
        message = self.query_one(f"#q-{tab}-msg", Static)
        try:
            # The simple tabs drive the console's fields, so both views share one request path.
            for source, target in self.QUICK_FIELDS.items():
                self.query_one(f"#{target}").value = self.query_one(f"#{source}").value
            # The simple view always flies; dry runs stay in the full console.
            mode = "live" if tab == "launch" else str(self.query_one("#q-land-mode", Select).value)
            self.query_one("#mode", Select).value = mode
            self.query_one("#execution-style", Select).value = (
                "automatic" if self.query_one("#q-automatic", Switch).value else "directed")
            key = "launch" if tab == "launch" else self._landing_operation(mode)
            self.selected_operation = key
            request = self._request()
            reason = self._availability(request)
            if reason:
                raise ConfigError(reason)
            signature = self._signature(request)
        except (OSError, ConfigError, PhaseTransitionError, ValueError, KeyError) as exc:
            self.quick_armed = None
            message.update(Text(f"✖ {exc}", style="bold red"))
            return
        armed = self.quick_armed
        if armed is None or armed[:2] != (key, signature) or time.monotonic() - armed[2] > 5.0:
            self.quick_armed = (key, signature, time.monotonic())
            message.update(Text(request.preview() + f"\n\nPress {tab.upper()} again within 5 s to confirm.",
                                style="yellow"))
            return
        self.quick_armed = None
        try:
            controller = request.build(self.runs_root)
        except (OSError, ConfigError, PhaseTransitionError, ValueError, KeyError) as exc:
            message.update(Text(f"✖ {exc}", style="bold red"))
            return
        if controller.mode == "live":
            self.notify("Live mission started — flight software is now commanding KSP.", severity="warning")
        self.controller = controller
        self.invalidate_review()
        controller.start_background()
        message.update(Text(f"Started {BY_KEY[key].title} ({controller.mode}) · {controller.run_dir.name}",
                            style="green"))
        self.watch_run(controller.run_dir)

    @on(Button.Pressed, "#q-launch")
    def quick_launch(self) -> None:
        self.quick_start("launch")

    @on(Button.Pressed, "#q-land")
    def quick_land(self) -> None:
        self.quick_start("land")

    @on(Button.Pressed, "#q-launch-abort")
    @on(Button.Pressed, "#q-land-abort")
    def quick_abort(self) -> None:
        self.action_abort()

    @on(Button.Pressed, "#q-execute-plan")
    def quick_execute_plan(self) -> None:
        self.submit_command("execute-plan")

    # --- mission setup ----------------------------------------------------------------
    def _base(self) -> dict[str, Any]:
        try:
            return load_config(self.profile, self.overrides).data
        except ConfigError:
            return DEFAULTS

    def _number(self, wid: str, label: str) -> float:
        text = self.query_one(f"#{wid}", Input).value.strip()
        try:
            return float(text)
        except ValueError:
            raise ConfigError(f"{label} must be a number, not {text!r}") from None

    @on(Select.Changed, "#mode")
    def mode_changed(self, event: Select.Changed) -> None:
        mode = str(event.value)
        self.invalidate_review()
        if mode == "sim":
            self.choose_operation("simulate")
        self.refresh_operations()

    def choose_operation(self, key: str) -> None:
        self.selected_operation = key
        self.query_one("#operations", DataTable).move_cursor(row=next(i for i, op in enumerate(OPERATIONS) if op.key == key))
        self.invalidate_review()
        self.refresh_operations()

    @on(Select.Changed, "#mission-profile")
    def profile_changed(self, event: Select.Changed) -> None:
        if event.value is Select.BLANK:
            return
        try:
            config = load_config(str(event.value), self.overrides)
            self.profile = str(event.value)
            self.apply_profile(config.data)
            self.vessel_info = None
            self.connection_error = ""
            self.invalidate_review()
            self.schedule_probe()
        except ConfigError as exc:
            self.notify(str(exc), severity="error")

    def apply_profile(self, data: dict[str, Any]) -> None:
        # All fields come from one resolved document when switching profiles.
        fields = {
            "altitude": ("ascent", "targetAltitudeKm"), "inclination": ("ascent", "inclinationDeg"),
            "ascent-mode": ("ascent", "guidanceMode"), "kind": ("profile", "kind"),
            "vehicle-config": ("intelligentAscent", "vehicleConfig"),
            "intelligent-periapsis": ("intelligentAscent", "targetPeriapsisKm"),
            "plane-target": ("profile", "planeTarget"), "geo-longitude": ("profile", "geoLongitudeDeg"),
            "gto-perigee": ("profile", "gtoPerigeeKm"), "geo-circularize": ("profile", "geoCircularize"),
            "launch-window": ("profile", "launchWindow"), "target-longitude": ("profile", "targetLongitude"),
            "auto-warp": ("operations", "autoWarp"), "orbit-trim": ("orbit", "trim"),
            "hold-trim": ("orbit", "holdForTrigger"), "max-burns": ("orbit", "maxBurns"),
            "landing-config": ("landing", "backendConfiguration"), "landing-model": ("landing", "modelProfile"),
            "strict-plan": ("landing", "requireStrictPlan"),
        }
        for wid, (section, key) in fields.items():
            widget = self.query_one(f"#{wid}")
            widget.value = data[section][key] if isinstance(widget, (Select, Switch)) else str(data[section][key])
        for source, target in self.QUICK_FIELDS.items():
            self.query_one(f"#{source}").value = self.query_one(f"#{target}").value

    def _selection(self) -> dict[str, Any]:
        value = lambda wid: self.query_one(f"#{wid}", Select).value  # noqa: E731
        # The inclination and altitude fields are the single source for those two keys, so they
        # are appended last and any typed copy of them in the overrides box is dropped.
        switch = lambda wid: self.query_one(f"#{wid}", Switch).value  # noqa: E731
        owned_values = {
            "intelligentAscent.targetPeriapsisKm": self._number("intelligent-periapsis", "Periapsis"),
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
            "landing.backendConfiguration": self.query_one("#landing-config", Input).value.strip(),
            "landing.modelProfile": self.query_one("#landing-model", Input).value.strip(),
            "landing.requireStrictPlan": switch("strict-plan"),
        }
        owned = tuple(owned_values)
        extra = [text for text in shlex.split(self.query_one("#overrides", Input).value)
                 if text.split("=", 1)[0] not in owned]
        fields = [f"{key}={json.dumps(val)}" for key, val in owned_values.items()]
        return {"profile": self.profile, "mode": value("mode"), "scenario": SCENARIOS[str(value("scenario"))],
                "overrides": extra + fields}

    def _request(self) -> OperationRequest:
        if self.selected_operation == "continue" and self.selected_run:
            from .operations import resume
            config = resume(self.selected_run)[0]
            return OperationRequest("continue", config, str(self.query_one("#mode", Select).value),
                                    source=self.selected_run, use_save=self.query_one("#use-save", Switch).value)
        sel = self._selection()
        config = load_config(str(sel["profile"]), sel["overrides"])
        return OperationRequest(self.selected_operation, config, str(sel["mode"]),
                                self.selected_operation in ("launch", "orbit") and
                                self.query_one("#execution-style", Select).value == "automatic",
                                sel["scenario"] if self.selected_operation == "simulate" else None,
                                self.selected_run, self.query_one("#use-save", Switch).value)

    @staticmethod
    def _signature(request: OperationRequest) -> str:
        config, start, stop, save = request.resolve()
        return json.dumps({"config": config.data, "key": request.key, "mode": request.mode,
                           "start": start.value, "stop": stop.value, "save": save,
                           "source": str(request.source), "scenario": request.scenario.name if request.scenario else None,
                           "automatic": request.automatic}, sort_keys=True)

    def _show_plan(self, message: str, style: str = "") -> None:
        self.query_one("#plan", Static).update(Text(message, style=style))

    @on(Button.Pressed, "#validate")
    def validate_plan(self) -> None:
        try:
            request = self._request()
            reason = self._availability(request)
            if reason:
                raise ConfigError(reason)
            self._show_plan(request.preview(), "green")
            self.query_one("#operation-settings", VerticalScroll).scroll_to_widget(self.query_one("#plan"), animate=False)
            self.reviewed_signature = self._signature(request)
            self.refresh_operations()
        except (OSError, ConfigError, PhaseTransitionError, ValueError, KeyError) as exc:
            self.reviewed_signature = None
            self._show_plan(f"✖ {exc}", "bold red")
            self.query_one("#start-btn", Button).disabled = True

    @on(Button.Pressed, "#start-btn")
    def action_start(self) -> None:
        try:
            request = self._request()
            if self.reviewed_signature != self._signature(request):
                raise ConfigError("Review the selected operation and its settings before executing")
            reason = self._availability(request)
            if reason:
                raise ConfigError(reason)
            controller = request.build(self.runs_root)
        except (OSError, ConfigError, PhaseTransitionError, ValueError, KeyError) as exc:
            self._show_plan(f"✖ {exc}", "bold red")
            self.notify(str(exc), title="Cannot start", severity="error")
            return
        if controller.mode == "live":
            self.notify("Live mission started — flight software is now commanding KSP.", severity="warning")
        self.controller = controller
        self.invalidate_review()
        controller.start_background()
        self.watch_run(controller.run_dir)

    @on(Button.Pressed, "#abort-btn")
    def action_abort(self) -> None:
        status = self.watcher.status if self.watcher else None
        if status is None or status.state != "running":
            self.notify("Watch a running operation first.")
            return
        if time.monotonic() - self.abort_armed_at > 3.0:
            self.abort_armed_at = time.monotonic()
            self.notify("Press abort again within 3 s to confirm.", severity="warning")
            return
        self.abort_armed_at = 0
        self.submit_command("abort")

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
            selector = self.query_one("#mission-profile", Select)
            selector.set_options([(f"Profile: {name}", name) for name in dict.fromkeys([self.profile, *available_profiles()])])
            selector.value = self.profile
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
                [(f"{i:02d} {c['name']} ({c['role']})", c['key']) for i,c in enumerate(document['components'])])
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
            component = next(c for c in document['components'] if c['key'] == key)
            if not component.get('separator'):
                self.notify("Only a measured decoupler can be approved for tank release", severity="error")
                return
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
        self.selected_run = Path(event.row_key.value)
        self.query_one("#resume-source", Static).update(f"Checkpoint source: {self.selected_run.name}\nUses its saved configuration.")
        self.invalidate_review()
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

    @on(Button.Pressed, "#watch-selected")
    def watch_selected(self) -> None:
        if self.selected_run:
            self.watch_run(self.selected_run)

    @on(Button.Pressed, "#continue-selected")
    def continue_selected(self) -> None:
        if self.selected_run:
            self.query_one("#tabs", TabbedContent).active = "dashboard"
            self.choose_operation("continue")
            self.query_one("#operations", DataTable).focus()

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
