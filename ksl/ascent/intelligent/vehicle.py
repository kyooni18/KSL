"""Read-only topology discovery and explicit, fingerprint-bound role corrections.

No part titles are interpreted as engine types. A discovery never writes controls.
Structural paths remain stable across sessions of the same saved craft.
"""
from dataclasses import dataclass, asdict
import hashlib
import json
from pathlib import Path
from ..orbit_math import norm, unit

ROLES = ('engine', 'booster', 'tank', 'external-tank', 'decoupler', 'structure', 'payload', 'ignore')


def optional(obj, name, default=None):
    try:
        return getattr(obj, name)
    except Exception:
        return default


@dataclass
class Component:
    key: str
    parent: str | None
    name: str
    role: str
    stage: int
    separation_stage: int
    mass: float
    position: tuple
    direction: tuple
    thrust: float = 0.0
    isp: float = 0.0
    throttleable: bool = False
    confidence: str = 'measured'


class Vehicle:
    def __init__(self, vessel):
        self.vessel = vessel
        self.parts = {}
        self.engines = {}
        self.separators = {}
        self.components = []
        self.approved = set()
        self.warnings = []
        frame = vessel.reference_frame

        def walk(part, key, parent):
            self.parts[key] = part
            engine = optional(part, 'engine')
            separator = optional(part, 'decoupler')
            resources = optional(part, 'resources')
            role = 'structure'
            if resources and optional(resources, 'names', []):
                role = 'tank'
            if optional(part, 'command'):
                role = 'payload'
            direction = (0., 1., 0.)
            confidence = 'measured'
            if separator:
                self.separators[key] = separator
                role = 'decoupler'
            thrust = isp = 0.
            throttleable = False
            if engine:
                self.engines[key] = engine
                role = 'engine'
                thrust = float(engine.max_thrust)
                isp = float(engine.vacuum_specific_impulse)
                throttleable = bool(engine.throttle_locked is False)
                try:
                    axes = [t.thrust_direction(frame) for t in engine.thrusters]
                    direction = unit(tuple(sum(a[i] for a in axes) for i in range(3)))
                    if norm(direction) < .5:
                        raise ValueError('empty thrust transforms')
                except Exception:
                    direction = tuple(part.direction(frame))
                    confidence = 'estimated axis'
                    self.warnings.append(f'{key}: thrust transforms unavailable; check axis')
            self.components.append(Component(key, parent, part.name, role, int(part.stage),
                int(part.decouple_stage), float(part.mass), tuple(part.position(frame)),
                direction, thrust, isp, throttleable, confidence))
            for i, child in enumerate(part.children):
                walk(child, f'{key}/{i}', key)
        walk(vessel.parts.root, '0', None)
        identity = [(c.key, c.name, c.stage, c.separation_stage) for c in self.components]
        self.fingerprint = hashlib.sha256(json.dumps(identity).encode()).hexdigest()[:24]

    def descendants(self, key):
        return {k for k in self.parts if k == key or k.startswith(key + '/')}

    def apply(self, document):
        if document.get('fingerprint') != self.fingerprint:
            raise ValueError('Vehicle configuration does not match this craft; inspect and save a new configuration')
        corrections = document.get('roles', {})
        if not isinstance(corrections, dict) or any(k not in self.parts or v not in ROLES for k, v in corrections.items()):
            raise ValueError('Invalid vehicle role correction')
        approved = document.get('approvedSeparators', [])
        if not isinstance(approved, list) or any(k not in self.separators for k in approved):
            raise ValueError('Invalid approved separator')
        for c in self.components:
            c.role = corrections.get(c.key, c.role)
        self.approved = set(approved)

    def document(self):
        return {'version': 1, 'fingerprint': self.fingerprint, 'name': self.vessel.name,
                'roles': {c.key: c.role for c in self.components},
                'approvedSeparators': sorted(self.approved),
                'components': [asdict(c) for c in self.components], 'warnings': self.warnings}

    def usable_engines(self):
        return {c.key: self.engines[c.key] for c in self.components
                if c.key in self.engines and c.role in ('engine', 'booster')
                and c.direction[1] > .5}

    def exhausted_branch(self, key):
        """Only positively identified, exhausted propulsion branches may auto-separate.

        A role override alone never grants permission to release a payload. Unknown
        fuel state raises rather than being interpreted as depletion.
        """
        branch = self.descendants(key)
        motors = [e for k, e in self.engines.items() if k in branch]
        if key == '0' or any(optional(self.parts[k], 'command') for k in branch):
            return False
        if any(c.role in ('payload', 'ignore') for c in self.components if c.key in branch):
            return False
        if motors:
            return all(e.active and not e.has_fuel and e.thrust < 1.0 for e in motors)
        if key not in self.approved:
            return False
        # Explicitly approved engine-less tanks require measured resource depletion.
        amounts = []
        for k in branch:
            res = self.parts[k].resources
            for name in res.names:
                if res.max(name) > 0 and name not in ('ElectricCharge', 'IntakeAir'):
                    amounts.append(res.amount(name) / res.max(name))
        return bool(amounts) and max(amounts) < .001


def save_document(path, document):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(document, indent=2) + '\n')
    temp.replace(path)


def diagram(document):
    lines = [f"{document['name']}  [{document['fingerprint']}]", 'Body axes: +Y nose; position in metres; arrows show thrust']
    for c in document['components']:
        indent = '  ' * c['key'].count('/')
        pos = ','.join(f'{x:.1f}' for x in c['position'])
        axis = ','.join(f'{x:+.2f}' for x in c['direction'])
        engine = f"  ↑({axis}) {c['thrust']/1000:.1f} kN" if c['thrust'] else ''
        lines.append(f"{indent}{c['key']} {c['name']} [{c['role']}] ({pos}) S{c['stage']}/D{c['separation_stage']}{engine}")
    return '\n'.join(lines + document.get('warnings', []))
