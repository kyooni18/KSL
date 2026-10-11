"""Machine-readable milestone lines from the ascent and OMS trim programs.

The KSL mission controller runs these programs as child processes and reads
lines starting with EVENT_PREFIX (see ksl.phases.ASCENT_EVENTS). Everything
else they print stays human-readable log output.
"""
import json
import time

EVENT_PREFIX = '@@ksl '
_last_telemetry_event = 0.0


def emit_event(name, **data):
    """Print one machine-readable milestone line for the KSL mission controller.

    Purely informational: the controller maps these onto mission phases
    (ksl.phases.ASCENT_EVENTS). Human-readable prints are unchanged.
    """
    print(EVENT_PREFIX + json.dumps({'event': name, **data}, default=float), flush=True)


def emit_telemetry(**sample):
    """Emit a telemetry event at most once per wall-clock second."""
    global _last_telemetry_event
    now = time.monotonic()
    if now - _last_telemetry_event >= 1.0:
        _last_telemetry_event = now
        emit_event('telemetry', **sample)
