"""``ksl``: single entry point for the unified shuttle flight-control system.

Every flying command goes through ksl.controller.MissionController, the same
backend the TUI uses. Output is human-readable by default; ``--json`` prints
machine-readable results for scripts and agents.
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

from . import __version__
from .config import ConfigError, available_profiles, config_from_snapshot, load_config
from .controller import (EXIT_ABORTED, EXIT_CONNECTION, EXIT_FAILED, EXIT_OK, EXIT_USAGE, MissionController,
                         continue_phase, holdable)
from .paths import BACKEND
from .phases import ENTRY_PHASES, SEQUENCE, Phase, PhaseTransitionError, validate_range
from .runlog import Event, list_runs, read_status, resolve_run
from .scenarios import SCENARIOS, resolve_scenario

COLORS = {"error": "\033[31m", "warning": "\033[33m", "phase": "\033[1;36m", "state": "\033[1m", "reset": "\033[0m"}


class Printer:
    """Console subscriber for mission events."""

    def __init__(self, verbose: bool, color: bool, json_lines: bool) -> None:
        self.verbose = verbose
        self.color = color
        self.json_lines = json_lines
        self.last_telemetry = 0.0

    def _paint(self, key: str, text: str) -> str:
        return f"{COLORS[key]}{text}{COLORS['reset']}" if self.color and key in COLORS else text

    def __call__(self, event: Event) -> None:
        if self.json_lines:
            if event.kind != "program" or self.verbose:
                print(event.to_json(), flush=True)
            return
        if event.kind == "program" and not self.verbose:
            return
        if event.kind == "telemetry":
            if not self.verbose and time.monotonic() - self.last_telemetry < 10.0:
                return
            self.last_telemetry = time.monotonic()
            d = event.data
            parts = [f"{d.get('phase', '?'):>14}"]
            for key, label, scale, unit in (("altitude", "alt", 1e-3, "km"), ("speed", "v", 1, "m/s"),
                                            ("apoapsis", "Ap", 1e-3, "km"), ("periapsis", "Pe", 1e-3, "km"),
                                            ("rangeToSite", "range", 1e-3, "km")):
                if isinstance(d.get(key), (int, float)):
                    parts.append(f"{label} {d[key] * scale:.1f} {unit}")
            print("  · " + " | ".join(parts), flush=True)
            return
        stamp = time.strftime("%H:%M:%S", time.localtime(event.time))
        key = "phase" if event.kind == "phase" else event.level if event.level in ("error", "warning") else \
            "state" if event.kind == "state" else ""
        prefix = {"warning": "WARN ", "error": "ERROR "}.get(event.level, "")
        print(f"{stamp} {self._paint(key, prefix + event.message)}", flush=True)


def _profile(args: argparse.Namespace):
    overrides = list(args.set)
    if getattr(args, "ascent_mode", None):
        overrides.append(f"ascent.guidanceMode={args.ascent_mode}")
    return load_config(args.profile, overrides)


def _install_abort(controller: MissionController) -> None:
    def handler(signum: int, _frame: Any) -> None:
        if controller.stop_event.is_set():
            raise KeyboardInterrupt  # second Ctrl-C: stop waiting
        controller.abort()
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)


def _fly(args: argparse.Namespace, controller: MissionController) -> int:
    printer = Printer(args.verbose, sys.stdout.isatty() and not args.no_color, args.json)
    controller.subscribe(printer)
    if not args.json:
        print(f"Run directory: {controller.run_dir}")
    _install_abort(controller)
    status = controller.run()
    if args.json:
        print(json.dumps({"result": asdict(status), "runDirectory": str(controller.run_dir)}, default=str))
    else:
        print(f"\nResult: {status.state.upper()} at {Phase(status.phase).title} — {status.status_message}")
        print(f"Phases: {' → '.join(Phase(p).title for p in status.phase_history)}")
    return controller.exit_code()


def _mode(args: argparse.Namespace) -> str:
    return "dry-run" if args.dry_run else "live"


def cmd_run(args: argparse.Namespace) -> int:
    config = _profile(args)
    start = Phase.parse(args.start) if args.start else Phase.PRELAUNCH
    stop = Phase.parse(args.stop) if args.stop else Phase.COMPLETE
    validate_range(start, stop)
    if not holdable(stop, _mode(args)):
        raise PhaseTransitionError(f"cannot stop at {stop.value} in {_mode(args)} mode")
    problems = config.validate()
    if problems:
        raise ConfigError("invalid mission profile:\n  " + "\n  ".join(problems))
    if args.load_save and not args.dry_run:
        from .krpc_link import krpc_env
        from .kspguard import load_save_verified
        print(f"Loading KSP save {args.load_save!r}")
        load_save_verified(args.load_save, krpc_env(config))
    controller = MissionController(config, _mode(args), start, stop, label=args.label)
    return _fly(args, controller)


def cmd_launch(args: argparse.Namespace) -> int:
    args.start, args.stop = "prelaunch", args.stop or "orbit"
    return cmd_run(args)


def cmd_land(args: argparse.Namespace) -> int:
    args.start = args.start or "deorbit"
    args.stop = "complete"
    if Phase.parse(args.start).index < Phase.ORBIT.index:
        raise PhaseTransitionError("land starts at orbit or later; use `ksl run` for a full mission")
    return cmd_run(args)


def cmd_continue(args: argparse.Namespace) -> int:
    run_dir = resolve_run(args.run)
    status = read_status(run_dir)
    checkpoint_path = run_dir / "checkpoint.json"
    checkpoint = json.loads(checkpoint_path.read_text()) if checkpoint_path.is_file() else {}
    phase = Phase.parse(checkpoint.get("phase") or status.phase)
    start = continue_phase(phase)
    saved = json.loads((run_dir / "config.json").read_text())
    if args.profile or not isinstance(saved.get("configuration"), dict):
        config = load_config(args.profile or saved.get("profile"), args.set)
    else:
        config = config_from_snapshot(saved.get("profile") or status.profile, saved["configuration"], args.set)
    if args.use_save:
        if not checkpoint.get("kspSave"):
            raise ConfigError(f"run {run_dir.name} has no KSP save to load")
        args.load_save = checkpoint["kspSave"]
    print(f"Continuing {run_dir.name} ({status.state} at {Phase(status.phase).title}) from {start.title}")
    if args.load_save and not args.dry_run:
        from .krpc_link import krpc_env
        from .kspguard import load_save_verified
        problems = config.validate()
        if problems:
            raise ConfigError("invalid mission profile:\n  " + "\n  ".join(problems))
        load_save_verified(args.load_save, krpc_env(config))
    controller = MissionController(config, _mode(args), start, Phase.COMPLETE,
                                   label=f"continue-{status.profile}", resumed_from=run_dir.name)
    return _fly(args, controller)


def cmd_simulate(args: argparse.Namespace) -> int:
    if args.list:
        rows = [{"name": s.name, "start": s.start.value, "profile": s.model_profile,
                 "description": s.description, "baseline": s.baseline} for s in SCENARIOS.values()]
        if args.json:
            print(json.dumps(rows, indent=2))
        else:
            for r in rows:
                print(f"{r['name']:<16} from {r['start']:<15} {r['description']}")
                if r["baseline"]:
                    print(f"{'':<16} baseline: {r['baseline']}")
        return EXIT_OK
    config = _profile(args)
    name = args.scenario or config["simulation"]["scenario"]
    scenario = resolve_scenario(name, Phase.parse(args.start) if args.start else None)
    stop = Phase.parse(args.stop) if args.stop else Phase.COMPLETE
    controller = MissionController(config, "dry-run" if args.dry_run else "sim", scenario.start, stop,
                                   scenario=scenario, label=args.label)
    return _fly(args, controller)


def _status_text(run_dir: Path, status: Any) -> str:
    t = status.telemetry or {}
    lines = [f"Run        {run_dir.name}",
             f"Profile    {status.profile} ({status.mode})",
             f"State      {status.state.upper()}",
             f"Phase      {Phase(status.phase).title}   [{status.start_phase} → {status.stop_phase}]",
             f"Program    {status.program or '-'}   connection: {status.connection}",
             f"Status     {status.status_message or '-'}"]
    if status.warning:
        lines.append(f"Warning    {status.warning}")
    if status.error:
        lines.append(f"Error      {status.error}")
    if t:
        shown = ", ".join(f"{k}={v:.1f}" if isinstance(v, float) else f"{k}={v}"
                          for k, v in t.items() if k in ("altitude", "speed", "apoapsis", "periapsis",
                                                         "rangeToSite", "verticalSpeed", "backendPhase"))
        lines.append(f"Telemetry  {shown}")
    age = time.time() - (status.updated_at or 0)
    lines.append(f"Updated    {age:.0f} s ago")
    lines.append(f"History    {' → '.join(status.phase_history)}")
    return "\n".join(lines)


def cmd_status(args: argparse.Namespace) -> int:
    if args.live:
        from .krpc_link import probe, suggest_phase
        info = probe(_profile(args))
        info["suggestedStartPhase"] = suggest_phase(info)
        print(json.dumps(info, indent=2, default=str) if args.json else
              "\n".join(f"{k:<20} {v}" for k, v in info.items()))
        return EXIT_OK
    run_dir = resolve_run(args.run)
    while True:
        status = read_status(run_dir)
        if args.json:
            print(json.dumps(asdict(status), default=str))
        else:
            print(_status_text(run_dir, status))
        if not args.watch or status.finished_at:
            return EXIT_OK if status.state in ("complete", "holding", "running", "ready") else EXIT_FAILED
        time.sleep(args.watch)
        if not args.json:
            print()


def cmd_runs(args: argparse.Namespace) -> int:
    runs = list_runs()[-args.limit:]
    rows = []
    for run in runs:
        s = read_status(run)
        rows.append({"run": run.name, "mode": s.mode, "state": s.state, "phase": s.phase, "profile": s.profile})
    if args.json:
        print(json.dumps(rows, indent=2))
    else:
        for r in rows:
            print(f"{r['run']:<48} {r['mode']:<8} {r['state']:<9} {r['phase']:<15} {r['profile']}")
    return EXIT_OK


def cmd_replay(args: argparse.Namespace) -> int:
    from .replay import load
    try:
        path = resolve_run(args.source)
    except (FileNotFoundError, LookupError):
        path = Path(args.source)
    recording = load(path)
    summary = recording.summary()
    if args.json:
        print(json.dumps({"summary": summary, "timeline": recording.timeline()}, default=str, indent=2))
        return EXIT_OK
    print(f"{summary['kind']}: {summary['source']}")
    print(f"  {summary['samples']} samples, {summary['events']} events, {summary['duration_s']:.0f} s")
    for key, label, scale, unit in (("max_altitude_m", "max altitude", 1e-3, "km"),
                                    ("max_speed_mps", "max speed", 1, "m/s"),
                                    ("max_dynamic_pressure_pa", "max q", 1e-3, "kPa")):
        if summary[key] is not None:
            print(f"  {label:<13} {summary[key] * scale:.1f} {unit}")
    print("  timeline:")
    for t, phase in recording.timeline():
        print(f"    T+{t:8.1f} s  {Phase(phase).title if phase in [p.value for p in Phase] else phase}")
    if args.events and recording.events:
        print("  events:")
        for e in recording.events:
            if e.kind in ("phase", "state", "milestone", "checkpoint") or e.level in ("warning", "error"):
                print(f"    {time.strftime('%H:%M:%S', time.localtime(e.time))} [{e.level}] {e.message}")
    if args.play:
        rate = max(args.play, 1e-3)
        prev = None
        for sample, t in recording.clocked_samples():
            if prev is not None:
                time.sleep(max(0.0, min(2.0, (t - prev) / rate)))
            prev = t
            print(f"  {sample.get('phase', '?'):>14} alt {float(sample.get('altitude') or 0) / 1000:7.2f} km "
                  f"v {float(sample.get('speed') or 0):7.1f} m/s")
    return EXIT_OK


def cmd_export(args: argparse.Namespace) -> int:
    from .replay import export, load
    try:
        path = resolve_run(args.source)
    except (FileNotFoundError, LookupError):
        path = Path(args.source)
    text = export(load(path), args.format, Path(args.output) if args.output else None)
    if not args.output:
        sys.stdout.write(text)
    else:
        print(f"wrote {args.output}", file=sys.stderr)
    return EXIT_OK


def cmd_validate(args: argparse.Namespace) -> int:
    config = _profile(args)
    checks: list[tuple[str, bool, str]] = []
    problems = config.validate()
    checks.append(("profile", not problems, "; ".join(problems) or f"{config.name} ({config.source})"))
    checks.append(("landing backend", BACKEND.is_file(), str(BACKEND) if BACKEND.is_file() else
                   f"missing {BACKEND} — run `make backend`"))
    try:
        import krpc  # noqa: F401
        checks.append(("krpc package", True, "importable"))
    except ImportError:
        checks.append(("krpc package", False, "not installed — run `make setup`"))
    scenario = SCENARIOS.get(config["simulation"]["scenario"])
    if scenario:
        missing = [str(p) for p in scenario.paths() if not p.is_file()]
        checks.append((f"scenario {scenario.name}", not missing, ", ".join(missing) or "inputs present"))
    if args.connect:
        from .krpc_link import ConnectionFailure, probe, suggest_phase
        try:
            info = probe(config)
            checks.append(("kRPC", True, f"scene {info['scene']}, vessel {info.get('vessel', '-')}, "
                                         f"situation {info.get('situation', '-')}, suggested start "
                                         f"{suggest_phase(info) or '-'}"))
        except ConnectionFailure as exc:
            checks.append(("kRPC", False, str(exc)))
    ok = all(c[1] for c in checks)
    if args.json:
        print(json.dumps({"ok": ok, "checks": [{"check": c, "ok": o, "detail": d} for c, o, d in checks]}, indent=2))
    else:
        for name, passed, detail in checks:
            print(f"{'PASS' if passed else 'FAIL'}  {name:<22} {detail}")
    if ok:
        return EXIT_OK
    return EXIT_CONNECTION if any(c[0] == "kRPC" and not c[1] for c in checks) else EXIT_FAILED


def cmd_checkpoints(args: argparse.Namespace) -> int:
    rows = []
    for run in list_runs():
        path = run / "checkpoint.json"
        if path.is_file():
            cp = json.loads(path.read_text())
            rows.append({"run": run.name, "phase": cp.get("phase"), "kspSave": cp.get("kspSave"),
                         "resumeFrom": _safe_continue(cp.get("phase"))})
    if args.json:
        print(json.dumps(rows, indent=2))
    elif not rows:
        print("no checkpoints yet (live missions write one after orbit trim and at configured phases)")
    else:
        for r in rows:
            print(f"{r['run']:<48} {r['phase']:<15} save={r['kspSave'] or '-':<40} resume→{r['resumeFrom'] or '-'}")
    return EXIT_OK


def _safe_continue(phase: str | None) -> str | None:
    try:
        return continue_phase(Phase.parse(phase or "")).value
    except (ValueError, PhaseTransitionError):
        return None


def cmd_profiles(args: argparse.Namespace) -> int:
    names = available_profiles()
    if args.show:
        config = load_config(args.show, args.set)
        print(json.dumps(config.data, indent=2))
        return EXIT_OK
    for name in names:
        config = load_config(name)
        print(f"{name:<18} {config.data.get('description', '')}")
    return EXIT_OK


def cmd_phases(args: argparse.Namespace) -> int:
    for phase in SEQUENCE:
        entry = ENTRY_PHASES.get(phase)
        print(f"{phase.value:<15} {phase.title:<15} {'start: ' + entry if entry else ''}")
    return EXIT_OK


def cmd_tui(args: argparse.Namespace) -> int:
    try:
        from .tui import run_tui
    except ImportError as exc:
        print(f"the TUI needs the 'textual' package ({exc}); run `make setup`", file=sys.stderr)
        return EXIT_USAGE
    overrides = list(args.set)
    if args.ascent_mode:
        overrides.append(f"ascent.guidanceMode={args.ascent_mode}")
    return run_tui(profile=args.profile, overrides=overrides)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="ksl", description="KSL — unified KSP Space Shuttle flight control: launch, orbit, deorbit, "
        "entry (MM304), TAEM (MM305), final approach, touchdown and rollout.",
        epilog="Exit codes: 0 ok/holding, 1 mission failed, 2 usage/config error, 3 kRPC connection failure, "
               "130 aborted. Run output lives in runs/<run-id>/.")
    parser.add_argument("--version", action="version", version=f"ksl {__version__}")
    sub = parser.add_subparsers(dest="command", metavar="COMMAND")

    def common(p: argparse.ArgumentParser, flying: bool = False) -> argparse.ArgumentParser:
        p.add_argument("-p", "--profile", help="mission profile name or JSON path (default sts-kerbin)")
        p.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                       help="override a profile value, e.g. --set ascent.inclinationDeg=10 (repeatable)")
        p.add_argument("--ascent-mode", choices=("legacy", "intelligent"),
                       help="ascent guidance for this mission (default: profile or legacy)")
        p.add_argument("--json", action="store_true", help="machine-readable output")
        if flying:
            p.add_argument("--dry-run", action="store_true", help="validate and print the plan without flying")
            p.add_argument("-v", "--verbose", action="store_true", help="also show raw program output")
            p.add_argument("--no-color", action="store_true")
            p.add_argument("--label", help="label used in the run id")
        return p

    p = common(sub.add_parser("run", help="fly a mission from any entry phase (live KSP)"), True)
    p.add_argument("--from", dest="start", metavar="PHASE", help="entry phase (default prelaunch); see `ksl phases`")
    p.add_argument("--to", dest="stop", metavar="PHASE", help="hold phase: prelaunch, orbit or complete (default)")
    p.add_argument("--load-save", metavar="NAME", help="load this KSP save before starting")
    p.set_defaults(func=cmd_run)

    p = common(sub.add_parser("launch", help="prelaunch -> orbit (ascent and OMS trim), then hold"), True)
    p.add_argument("--to", dest="stop", metavar="PHASE", help="prelaunch (read-only preflight) or orbit (default)")
    p.add_argument("--load-save", metavar="NAME")
    p.set_defaults(func=cmd_launch, start=None)

    p = common(sub.add_parser("land", help="deorbit (or later) -> rollout on the native landing backend"), True)
    p.add_argument("--from", dest="start", metavar="PHASE", help="orbit, deorbit (default), mm304, mm305, final-approach")
    p.add_argument("--load-save", metavar="NAME")
    p.set_defaults(func=cmd_land, stop=None)

    p = common(sub.add_parser("continue", help="resume a run from its last checkpoint/phase"), True)
    p.add_argument("run", nargs="?", default="latest", help="run id, unique fragment, path or 'latest'")
    p.add_argument("--use-save", action="store_true", help="load the checkpoint's KSP save first")
    p.add_argument("--load-save", metavar="NAME")
    p.set_defaults(func=cmd_continue)

    p = common(sub.add_parser("simulate", help="offline closed-loop landing against ShuttleSim"), True)
    p.add_argument("scenario", nargs="?", help="named scenario or ShuttleSim .ini (default from profile)")
    p.add_argument("--list", action="store_true", help="list named scenarios")
    p.add_argument("--from", dest="start", metavar="PHASE", help="entry phase for a raw .ini scenario")
    p.add_argument("--to", dest="stop", metavar="PHASE", help="stop when this landing phase begins")
    p.set_defaults(func=cmd_simulate)

    p = common(sub.add_parser("status", help="show a run's mission status (or the live game with --live)"))
    p.add_argument("run", nargs="?", default="latest")
    p.add_argument("--watch", type=float, nargs="?", const=2.0, metavar="SECONDS", help="refresh until the run ends")
    p.add_argument("--live", action="store_true", help="read-only probe of the KSP game instead")
    p.set_defaults(func=cmd_status)

    p = common(sub.add_parser("runs", help="list recorded mission runs"))
    p.add_argument("-n", "--limit", type=int, default=20)
    p.set_defaults(func=cmd_runs)

    p = common(sub.add_parser("replay", help="summarize/play back a run, ascent CSV or ShuttleSim recording"))
    p.add_argument("source", nargs="?", default="latest")
    p.add_argument("--events", action="store_true", help="list mission events")
    p.add_argument("--play", type=float, metavar="RATE", help="print samples in (scaled) real time")
    p.set_defaults(func=cmd_replay)

    p = common(sub.add_parser("export", help="export telemetry as CSV or JSONL"))
    p.add_argument("source", nargs="?", default="latest")
    p.add_argument("-f", "--format", choices=("csv", "jsonl"), default="csv")
    p.add_argument("-o", "--output", help="output file (default stdout)")
    p.set_defaults(func=cmd_export)

    p = common(sub.add_parser("validate", help="check profile, build, dependencies (and KSP with --connect)"))
    p.add_argument("--connect", action="store_true", help="also probe the kRPC server (read-only)")
    p.set_defaults(func=cmd_validate)

    p = common(sub.add_parser("checkpoints", help="list phase checkpoints usable by `ksl continue`"))
    p.set_defaults(func=cmd_checkpoints)

    p = common(sub.add_parser("profiles", help="list mission profiles"))
    p.add_argument("--show", metavar="NAME", help="print a resolved profile")
    p.set_defaults(func=cmd_profiles)

    p = common(sub.add_parser("phases", help="list mission phases and valid entry phases"))
    p.set_defaults(func=cmd_phases)

    p = common(sub.add_parser("tui", help="interactive terminal UI (dashboard, setup, replay)"))
    p.set_defaults(func=cmd_tui)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not getattr(args, "func", None):
        parser.print_help()
        return EXIT_USAGE
    try:
        return args.func(args)
    except (ConfigError, PhaseTransitionError, ValueError, KeyError) as exc:
        print(f"ksl: error: {exc}".replace("\\n", "\n"), file=sys.stderr)
        return EXIT_USAGE
    except (FileNotFoundError, LookupError) as exc:
        print(f"ksl: error: {exc}", file=sys.stderr)
        return EXIT_USAGE
    except KeyboardInterrupt:
        print("ksl: interrupted", file=sys.stderr)
        return EXIT_ABORTED
    except Exception as exc:  # connection failures from probe/load paths
        from .krpc_link import ConnectionFailure
        if isinstance(exc, ConnectionFailure):
            print(f"ksl: {exc}", file=sys.stderr)
            return EXIT_CONNECTION
        raise


if __name__ == "__main__":
    sys.exit(main())
