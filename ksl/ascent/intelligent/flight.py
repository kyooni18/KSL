"""Optional generic ascent process. Legacy flight.py is deliberately not imported."""
import argparse
import csv
import json
import math
import signal
import time
from pathlib import Path

from ...config import config_from_snapshot
from ...krpc_link import connect
from ...paths import repo_path
from ..events import emit_event, emit_telemetry
from ..orbit_math import norm, unit, dot, cross
from .vehicle import Vehicle, diagram
from .planner import Planner, State, Target, achieved, coast_apoapsis


def engage(autopilot, enabled):
    method = getattr(autopilot, 'engage' if enabled else 'disengage', None)
    if callable(method):
        method()
    else:
        autopilot.engaged = enabled


def heading_for(inclination, latitude):
    ratio = math.cos(math.radians(inclination))/max(.000001, math.cos(math.radians(latitude)))
    northbound = math.degrees(math.asin(max(-1., min(1., ratio))))
    return (180.-northbound if inclination < 0 else northbound) % 360


class Propulsion:
    def __init__(self, model):
        self.model = model
        self.separated = set()
        self.ignited = set()
        self.depleted_since = {}

    def present(self):
        attached = set(self.model.vessel.parts.all)
        return {k: e for k, e in self.model.usable_engines().items() if e.part in attached}

    def ignite(self):
        engines = self.present()
        candidates = [(k, e) for k, e in engines.items() if k not in self.ignited and e.has_fuel]
        if not candidates:
            return False
        stage = max(e.part.stage for _, e in candidates)
        selected = [(k, e) for k, e in candidates if e.part.stage == stage]
        for k, e in selected:
            e.active = True
            self.ignited.add(k)
        emit_event('engine-ignition', stage=stage, engines=[k for k, _ in selected])
        return True

    def update(self):
        attached = set(self.model.vessel.parts.all)
        groups = {}
        waiting_for_separation = False
        for c in self.model.components:
            if c.key in self.model.separators and c.key not in self.separated and self.model.parts[c.key] in attached:
                groups.setdefault(c.separation_stage, []).append(c.key)
        for group in groups.values():
            ready = all(self.model.exhausted_branch(key) for key in group)
            token = tuple(group)
            if not ready:
                self.depleted_since.pop(token, None)
                continue
            since = self.depleted_since.setdefault(token, time.monotonic())
            if time.monotonic()-since < 2.:
                waiting_for_separation = True
                continue
            for key in group:
                self.model.separators[key].decouple()
                self.separated.add(key)
                emit_event('stage-separation', component=key)
        engines = list(self.present().values())
        active = [e for e in engines if e.active and e.has_fuel]
        if not active and not waiting_for_separation and self.ignite():
            active = [e for e in self.present().values() if e.active and e.has_fuel]
        return active


def rotate_thrust_to_target(vector, thrust, target):
    """Shortest rigid rotation aligning measured thrust, including large cant angles."""
    source, target = unit(thrust), unit(target)
    cosine = max(-1., min(1., dot(source, target)))
    axis = cross(source, target)
    sine = norm(axis)
    if sine < 1e-8:
        if cosine > 0:
            return vector
        axis = unit(cross(source, (1., 0., 0.) if abs(source[0]) < .9 else (0., 1., 0.)))
    else:
        axis = unit(axis)
    turn = cross(axis, vector)
    return tuple(vector[i]*cosine+turn[i]*sine+axis[i]*dot(axis, vector)*(1-cosine) for i in range(3))


def aim(conn, vessel, frame, direction):
    sc = conn.space_center
    total = [0., 0., 0.]
    for engine in vessel.parts.engines:
        if not engine.active:
            continue
        force = engine.thrust if engine.thrust > 1. else engine.available_thrust
        thrusters = engine.thrusters
        try:
            directions = [t.thrust_direction(vessel.reference_frame) for t in thrusters]
        except Exception:
            directions = []
        if not directions:
            directions = [engine.part.direction(vessel.reference_frame)]
        for axis in directions:
            for i in range(3):
                total[i] += force*axis[i]/len(directions)
    nose = sc.transform_direction((0., 1., 0.), vessel.reference_frame, frame)
    roof = sc.transform_direction((0., 0., -1.), vessel.reference_frame, frame)
    thrust = sc.transform_direction(unit(total), vessel.reference_frame, frame) if norm(total) > 1. else nose
    nose_target = rotate_thrust_to_target(nose, thrust, direction)
    roof_target = rotate_thrust_to_target(roof, thrust, direction)
    ap = vessel.auto_pilot
    vessel.control.sas = False
    if not getattr(ap, 'engaged', True):
        engage(ap, True)
    ap.reference_frame = frame
    if hasattr(ap, 'set_direction_and_up'):
        ap.set_direction_and_up(nose_target, roof_target)
    else:
        ap.target_direction = nose_target
    return math.degrees(math.acos(max(-1., min(1., dot(unit(thrust), unit(direction))))))


def correct_orbit(conn, vessel, propulsion, target, timeout):
    """Replan finite burns from achieved elements, using generic active propulsion."""
    from .insertion import plan_node, recovery_direction
    sc, ap = conn.space_center, vessel.auto_pilot
    frame = vessel.orbit.body.non_rotating_reference_frame
    deadline = time.monotonic()+timeout
    from ...controls import Controls, warp_until
    controls = Controls.from_env()
    for attempt in range(8):
        o = vessel.orbit
        if achieved(o.apoapsis_altitude, o.periapsis_altitude, math.degrees(o.inclination), target):
            return
        engines = propulsion.update()
        thrust = sum(e.available_thrust for e in engines)
        if thrust <= 0:
            raise RuntimeError('No fueled propulsion remains for insertion')
        try:
            result = plan_node(sc, vessel, target)
        except (ArithmeticError, ValueError) as exc:
            emit_event('prediction-warning', error=str(exc))
            result = None
        if result is None:
            # A search failure is not proof of an infeasible orbit. Build energy
            # for a bounded interval and retry from fresh telemetry.
            emit_event('insertion-recovery', attempt=attempt+1)
            until = min(deadline, time.monotonic()+15.)
            try:
                while time.monotonic() < until:
                    direction = recovery_direction(vessel.position(frame), vessel.velocity(frame), target.mu)
                    error = aim(conn, vessel, frame, direction)
                    vessel.control.throttle = min(1., 9.80665*vessel.mass/max(thrust, 1.)) if error < 10 else 0.
                    time.sleep(.1)
            finally:
                vessel.control.throttle = 0.
            continue
        node, prediction = result
        emit_event('insertion-plan', attempt=attempt+1, deltaV=prediction['dv'],
                   burnIn=prediction['t'], exact=prediction['exact'])
        try:
            duration = node.delta_v/(thrust/vessel.mass)
            start = node.ut-duration/2
            aim(conn, vessel, frame, node.burn_vector(frame))
            if vessel.flight().mean_altitude > target.atmosphere+1000 and start-sc.ut > 40:
                warp_until(sc, start, controls, lead=20.)
            while sc.ut < start:
                if time.monotonic() > deadline:
                    raise RuntimeError('Insertion coast timed out')
                if vessel.flight().mean_altitude < target.atmosphere and vessel.flight(vessel.orbit.body.reference_frame).vertical_speed < 0:
                    raise RuntimeError('Insertion coast is descending into atmosphere')
                time.sleep(.1)
            initial = norm(node.remaining_burn_vector(frame))
            previous = initial
            while time.monotonic() < deadline:
                dv = node.remaining_burn_vector(frame)
                remaining = norm(dv)
                if remaining < .15 or (remaining > previous+.2 and previous < 1.):
                    break
                previous = remaining
                error = aim(conn, vessel, frame, dv)
                engines = propulsion.update()
                accel = sum(e.available_thrust for e in engines)/vessel.mass
                if accel <= .001:
                    raise RuntimeError('Propulsion lost during insertion')
                vessel.control.throttle = min(1., remaining/max(accel*2., .1)) if error < 8 else 0.
                emit_telemetry(phase='insertion', apoapsis=vessel.orbit.apoapsis_altitude,
                               periapsis=vessel.orbit.periapsis_altitude, remainingDeltaV=remaining,
                               thrustAlignmentError=error, attitudeError=ap.error, throttle=vessel.control.throttle)
                time.sleep(.05)
            else:
                raise RuntimeError('Insertion burn timed out')
        finally:
            vessel.control.throttle = 0.
            node.remove()
        time.sleep(.3)
    raise RuntimeError('Orbital correction budget exhausted; measured orbit outside tolerance')


def fly(conn, config, output, launch):
    sc = conn.space_center
    if str(conn.krpc.current_game_scene).split('.')[-1] != 'flight':
        raise RuntimeError('Load a vessel in the flight scene before vehicle inspection or launch')
    vessel = sc.active_vessel
    model = Vehicle(vessel)
    settings = config['intelligentAscent']
    if settings['vehicleConfig']:
        model.apply(json.loads(repo_path(settings['vehicleConfig']).read_text()))
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.with_suffix('.vehicle.json').write_text(json.dumps(model.document(), indent=2))
    print(diagram(model.document()), flush=True)
    if not model.usable_engines():
        raise RuntimeError('No forward propulsion identified; inspect engine axes and component roles')
    body = vessel.orbit.body
    altitude = config.target_altitude_m
    periapsis = settings["targetPeriapsisKm"]*1000 or altitude
    target = Target(body.equatorial_radius, body.gravitational_parameter, body.atmosphere_depth,
                    altitude, periapsis, config['ascent']['inclinationDeg'],
                    settings['altitudeToleranceM'], settings['inclinationToleranceDeg'])
    if target.periapsis <= target.atmosphere+target.altitude_tolerance:
        raise RuntimeError('Requested orbit intersects the atmosphere')
    if config['profile']['kind'] != 'circular':
        raise RuntimeError('Intelligent ascent currently requires a circular profile')
    emit_event('preflight', guidance='intelligent', fingerprint=model.fingerprint)
    if not launch:
        return
    if str(vessel.situation).split('.')[-1] not in ('pre_launch', 'landed'):
        raise RuntimeError('Intelligent launch requires a vessel on the pad; refusing in-flight takeover')
    propulsion = Propulsion(model)
    planner = Planner(target, settings['maxDynamicPressurePa'], settings['maxG'])
    ctl, ap = vessel.control, vessel.auto_pilot
    start = sc.ut
    wall_deadline = time.monotonic()+settings['timeoutSeconds']
    heading = heading_for(target.inclination, vessel.flight().latitude)
    last = start
    turn_announced = False
    coasting = False
    no_thrust_since = None
    with output.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=['ut', 'altitude', 'verticalSpeed', 'horizontalSpeed',
            'apoapsis', 'periapsis', 'inclination', 'mass', 'dynamicPressure', 'pitchCommand',
            'throttle', 'predictedApoapsis'])
        writer.writeheader()
        try:
            ap.reference_frame = vessel.surface_reference_frame
            ap.target_direction = vessel.direction(vessel.surface_reference_frame)
            ctl.sas = False
            engage(ap, True)
            if not propulsion.ignite():
                raise RuntimeError('No fueled launch engines')
            ctl.throttle = 1.
            # Clamp release is explicit and never invokes the staging stack.
            ignition_deadline = time.monotonic()+30.
            while True:
                thrust = sum(e.thrust for e in propulsion.present().values() if e.active)
                weight = vessel.mass*target.mu/(target.radius+vessel.flight().mean_altitude)**2
                if thrust >= weight:
                    break
                if time.monotonic() >= ignition_deadline:
                    raise RuntimeError('Measured thrust remained below lift-off requirement after ignition settling')
                time.sleep(.2)
            for clamp in vessel.parts.launch_clamps:
                clamp.release()
            emit_event('liftoff', guidance='intelligent')
            while time.monotonic() < wall_deadline:
                now = sc.ut
                if now <= last:
                    time.sleep(.1)
                    continue
                dt, last = min(2., now-last), now
                if sc.active_vessel != vessel:
                    raise RuntimeError('Active vessel changed; releasing guidance')
                engines = propulsion.update()
                thrust = sum(e.available_thrust for e in engines)
                isp = sum(e.specific_impulse*e.available_thrust for e in engines)/max(1., thrust)
                f = vessel.flight(body.reference_frame)
                inertial = vessel.flight(body.non_rotating_reference_frame)
                drag = norm(f.drag)/vessel.mass
                state = State(f.mean_altitude, f.vertical_speed, inertial.horizontal_speed,
                              vessel.mass, thrust, isp, f.dynamic_pressure, drag,
                              sum(e.thrust for e in engines if e.throttle_locked), f.horizontal_speed)
                if thrust <= 0 and not coasting:
                    no_thrust_since = no_thrust_since or time.monotonic()
                    if time.monotonic()-no_thrust_since > 15.:
                        raise RuntimeError('No fueled propulsion remains during ascent after staging grace')
                    ctl.throttle = 0.
                    time.sleep(.1)
                    continue
                no_thrust_since = None
                # Bound command slew by live rotational authority and inertia.
                positive, negative = vessel.available_torque
                inertia = vessel.moment_of_inertia
                authority = min(max(positive[i], negative[i])/max(inertia[i], 1.) for i in (0, 2))
                planner.slew = min(2., max(.1, math.degrees(math.sqrt(max(0., authority)))*.2))
                command = planner.command(state, dt)
                if not turn_announced and command.pitch < 85:
                    emit_event('gravity-turn', guidance='intelligent')
                    turn_announced = True
                # A safe ballistic arc is handed to the generic finite-burn insertion solver.
                if coasting:
                    ctl.throttle = 0.
                    if state.altitude > target.atmosphere+1000:
                        break
                    if state.radial > 0:
                        emit_telemetry(phase='coast', altitude=state.altitude, verticalSpeed=state.radial,
                                       apoapsis=vessel.orbit.apoapsis_altitude, periapsis=vessel.orbit.periapsis_altitude)
                        time.sleep(.1)
                        continue
                    emit_event('coast-recovery', reason='atmospheric climb ended before clearance')
                    coasting = False
                    planner.pitch = max(30., planner.pitch)
                if state.fixed_thrust < 1. and coast_apoapsis(state, target) >= target.apoapsis-100 and state.radial > 0:
                    ctl.throttle = 0.
                    coasting = True
                    emit_event('ascent-cutoff', guidance='intelligent')
                    continue
                pitch, yaw = math.radians(command.pitch), math.radians(heading)
                direction = (math.sin(pitch), math.cos(pitch)*math.cos(yaw), math.cos(pitch)*math.sin(yaw))
                error = aim(conn, vessel, vessel.surface_reference_frame, direction)
                ctl.throttle = command.throttle
                if error > 30 and f.mean_altitude > 1000:
                    ctl.throttle = min(ctl.throttle, .25)
                sample = dict(ut=now, altitude=f.mean_altitude, verticalSpeed=f.vertical_speed,
                    horizontalSpeed=inertial.horizontal_speed, apoapsis=vessel.orbit.apoapsis_altitude,
                    periapsis=vessel.orbit.periapsis_altitude, inclination=math.degrees(vessel.orbit.inclination),
                    mass=vessel.mass, dynamicPressure=f.dynamic_pressure, pitchCommand=command.pitch,
                    throttle=ctl.throttle, predictedApoapsis=command.predicted_apoapsis)
                writer.writerow(sample)
                stream.flush()
                emit_telemetry(**sample)
                if now-start > 15 and f.mean_altitude < 100 and f.vertical_speed < -1:
                    raise RuntimeError('Ascent lost altitude near the surface')
                time.sleep(.1)
            else:
                raise RuntimeError('Ascent timed out')
            correct_orbit(conn, vessel, propulsion, target, max(1., wall_deadline-time.monotonic()))
            o = vessel.orbit
            report = dict(apoapsis=o.apoapsis_altitude, periapsis=o.periapsis_altitude,
                          inclination=math.degrees(o.inclination))
            report['achieved'] = achieved(**report, target=target)
            output.with_suffix('.result.json').write_text(json.dumps(report, indent=2))
            if not report['achieved']:
                raise RuntimeError('Measured final orbit outside tolerance')
            emit_event('insertion', guidance='intelligent', **report)
        finally:
            for action in (lambda: setattr(ctl, 'throttle', 0.), lambda: engage(ap, False),
                           lambda: setattr(ctl, 'sas', True),
                           lambda: setattr(sc, 'rails_warp_factor', 0)):
                try:
                    action()
                except Exception:
                    pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--telemetry', required=True)
    parser.add_argument('--launch', action='store_true')
    args = parser.parse_args()
    data = json.loads(Path(args.config).read_text())
    config = config_from_snapshot(data.get('profile', 'intelligent'), data.get('configuration', data))
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        with connect(config, 'KSL intelligent ascent') as conn:
            fly(conn, config, args.telemetry, args.launch)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f'Intelligent ascent failed: {exc}', flush=True)
        emit_event('guidance-failure', error=str(exc))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
