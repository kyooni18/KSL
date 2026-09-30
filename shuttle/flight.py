"""kRPC adapter. Preflight is read-only; --launch explicitly enables control."""
import argparse
import csv
import math
import time
from .allocation import allocate


def descendants(part):
    result = [part]
    for child in part.children:
        result.extend(descendants(child))
    return result


def one(parts, name):
    found = [p for p in parts if p.name == name]
    if len(found) != 1:
        raise ValueError(f'Expected one {name}, found {len(found)}')
    return found[0]


def set_autopilot_engaged(autopilot, engaged):
    """Support kRPC clients exposing engagement as either methods or a property."""
    method = getattr(autopilot, 'engage' if engaged else 'disengage', None)
    if callable(method):
        method()
    elif hasattr(autopilot, 'engaged'):
        autopilot.engaged = engaged
    else:
        raise AttributeError('AutoPilot has no engagement control')


class Shuttle:
    def __init__(self, conn):
        self.conn = conn
        self.sc = conn.space_center
        self.vessel = self.sc.active_vessel
        parts = self.vessel.parts.all
        self.skiff = one(parts, 'LiquidEngineRE-I2').engine
        self.cheetah = one(parts, 'LiquidEngineLV-T91').engine
        self.boosters = [p.engine for p in parts if p.name == 'Pollux']
        bobcats = [p.engine for p in parts if p.name == 'LiquidEngineLV-TX87']
        if len(bobcats) != 4 or len(self.boosters) != 2:
            raise ValueError('Expected four Bobcats and two Pollux boosters')
        self.tanks = [p for p in parts if p.name == 'Size3LargeTank']
        self.shuttle_tank = one(parts, 'mk3FuselageLFO.25')
        if len(self.tanks) != 2:
            raise ValueError('Expected two external tanks')
        frame = self.vessel.reference_frame
        s = self.skiff.part.position(frame)
        c = self.cheetah.part.position(frame)
        length = math.hypot(s[0]-c[0], s[2]-c[2])
        if length < 1:
            raise ValueError('Invalid shuttle/ET geometry')
        self.radial = ((s[0]-c[0])/length, (s[2]-c[2])/length)
        bobcats.sort(key=lambda e: self.project(e.part.position(frame)), reverse=True)
        self.groups = [[self.skiff], bobcats[:2], bobcats[2:], [self.cheetah]]
        self.booster_decouplers = []
        self.et_decoupler = None
        for p in parts:
            if p.name != 'radialDecoupler1-2':
                continue
            subtree = descendants(p)
            if all(t in subtree for t in self.tanks) and self.skiff.part not in subtree:
                self.et_decoupler = p.decoupler
            elif sum(e.part in subtree for e in self.boosters) == 1 and not any(t in subtree for t in self.tanks):
                self.booster_decouplers.append(p.decoupler)
        if len(self.booster_decouplers) != 2 or self.et_decoupler is None:
            raise ValueError('Could not uniquely identify booster and ET separators')
        self.sepmotors = [p.engine for p in parts if p.name == 'sepMotor1']
        self.transfers = {}
        self.separated = False
        self.et_attached = True

    def project(self, position):
        return position[0]*self.radial[0] + position[2]*self.radial[1]

    def allocation(self):
        frame = self.vessel.reference_frame
        # kRPC's vessel frame follows the instantaneous center of mass.
        com = 0.0
        thrusts, arms = [], []
        for group in self.groups:
            values = [(e.max_thrust if e.has_fuel else 0,
                       self.project(e.part.position(frame))-com) for e in group]
            thrust = sum(t for t, _ in values)
            thrusts.append(thrust)
            arms.append(sum(t*a for t, a in values)/thrust if thrust else 0)
        fixed = 0
        if not self.separated:
            for e in self.boosters:
                # Before ignition use pressure-corrected full thrust; after ignition
                # actual thrust includes the Pollux burn curve and burnout.
                thrust = e.thrust if e.active else (e.max_thrust if e.has_fuel else 0)
                fixed += thrust*(self.project(e.part.position(frame))-com)
        return allocate(thrusts, arms, fixed)

    def top_off(self):
        for resource in ('LiquidFuel', 'Oxidizer'):
            previous = self.transfers.get(resource)
            if previous is not None and not previous.complete:
                continue
            amount = self.shuttle_tank.resources.max(resource)-self.shuttle_tank.resources.amount(resource)
            if amount < .1:
                continue
            source = max(self.tanks, key=lambda p: p.resources.amount(resource))
            if source.resources.amount(resource) > .1:
                self.transfers[resource] = self.sc.ResourceTransfer.start(
                    source, self.shuttle_tank, resource, amount)

    def update(self):
        if self.et_attached:
            self.top_off()
        if not self.separated and all(e.part.resources.amount('SolidFuel') < .1 for e in self.boosters):
            for e in self.sepmotors:
                e.active = True
            for d in self.booster_decouplers:
                d.decouple()
            self.separated = True
            print('Pollux burnout: boosters separated')
        result = self.allocation()
        for group, u in zip(self.groups, result.throttles):
            for e in group:
                e.thrust_limit = u
        return result

    def release_et(self):
        self.vessel.control.throttle = 0
        if any(not transfer.complete for transfer in self.transfers.values()):
            return False
        for group in self.groups[1:]:
            for e in group:
                e.active = False
        self.et_decoupler.decouple()
        self.et_attached = False
        self.groups = [[self.skiff]]
        self.skiff.thrust_limit = 1
        print('External tank separated')
        return True


def run(args):
    import krpc
    with krpc.connect(name='STS Launch Program', address=args.address,
                      rpc_port=args.rpc_port, stream_port=args.stream_port) as conn:
        shuttle = Shuttle(conn)
        v = shuttle.vessel
        result = shuttle.allocation()
        print(f'{v.name}: {v.mass/1000:.2f} t; engine groups Skiff/Near/Far/Cheetah')
        print(f'Preflight throttles: {result.throttles}; residual {result.residual:.0f} N m')
        if not args.launch:
            print('Read-only preflight complete. Use --launch to fly.')
            return
        if v.name != 'STS':
            raise ValueError('Expected the active vessel to be named STS')
        if v.situation != conn.space_center.VesselSituation.pre_launch:
            raise ValueError('Launch requires an STS on the launch pad')
        body = v.orbit.body
        if body.name != 'Kerbin':
            raise ValueError('Default ascent guidance is designed for Kerbin')
        if args.orbit <= body.atmosphere_depth + 10000:
            raise ValueError('Target orbit must be at least 10 km above the atmosphere')
        flight = v.flight(body.reference_frame)
        ap = v.auto_pilot
        control = v.control
        started = conn.space_center.ut
        state = 'ASCENT'
        next_report = 0
        try:
            control.throttle = 0
            control.sas = False
            ap.reference_frame = v.surface_reference_frame
            ap.target_pitch_and_heading(90, args.heading)
            set_autopilot_engaged(ap, True)
            for group, u in zip(shuttle.groups, result.throttles):
                for e in group:
                    e.thrust_limit = u
                    e.active = True
            for e in shuttle.boosters:
                e.active = True
            for clamp in v.parts.launch_clamps:
                clamp.release()
            control.throttle = 1
            with open(args.telemetry, 'w', newline='') as log:
                writer = csv.writer(log)
                writer.writerow(['ut', 'state', 'altitude_m', 'apoapsis_m', 'periapsis_m',
                                 'throttles', 'residual_Nm', 'feasible'])
                while True:
                    tick = time.monotonic()
                    if conn.space_center.active_vessel != v:
                        raise RuntimeError('Active vessel changed')
                    if conn.space_center.ut-started > args.timeout:
                        raise RuntimeError('Mission timeout')
                    altitude = flight.mean_altitude
                    if state == 'ASCENT':
                        pitch = max(5, 90*(1-min(1, max(0, (altitude-args.turn_start)/(args.turn_end-args.turn_start)))))
                        ap.target_pitch_and_heading(pitch, args.heading)
                        if v.orbit.apoapsis_altitude >= args.orbit:
                            control.throttle = 0
                            state = 'COAST'
                        elif shuttle.separated and any(sum(t.resources.amount(r) for t in shuttle.tanks) < 1 for r in ('LiquidFuel', 'Oxidizer')):
                            if shuttle.release_et():
                                state = 'ORBITER_ASCENT'
                    elif state == 'ORBITER_ASCENT':
                        ap.target_pitch_and_heading(5, args.heading)
                        control.throttle = min(1, max(.05, (args.orbit-v.orbit.apoapsis_altitude)/5000))
                        if v.orbit.apoapsis_altitude >= args.orbit:
                            control.throttle = 0
                            state = 'COAST'
                    elif state == 'COAST':
                        control.throttle = 0
                        if not shuttle.separated:
                            raise RuntimeError('Target reached before SRB burnout; coast cannot stop SRBs')
                        if shuttle.et_attached and not shuttle.release_et():
                            time.sleep(1/args.hz)
                            continue
                        ap.reference_frame = v.orbital_reference_frame
                        ap.target_direction = (0, 1, 0)
                        radius = body.equatorial_radius+v.orbit.apoapsis_altitude
                        dv = max(0, math.sqrt(body.gravitational_parameter/radius)-math.sqrt(body.gravitational_parameter*(2/radius-1/v.orbit.semi_major_axis)))
                        acceleration = shuttle.skiff.max_thrust/v.mass
                        if acceleration <= 0:
                            raise RuntimeError('Skiff has no thrust for insertion')
                        if v.orbit.time_to_apoapsis <= dv/acceleration/2+1:
                            state = 'INSERTION'
                    elif state == 'INSERTION':
                        control.throttle = min(1, max(.03, (args.orbit-v.orbit.periapsis_altitude)/10000))
                        if v.orbit.periapsis_altitude >= args.orbit-1000:
                            print('Orbital insertion complete')
                            return
                        if flight.mean_altitude < body.atmosphere_depth:
                            raise RuntimeError('Insertion trajectory reentered atmosphere')
                    if shuttle.et_attached:
                        result = shuttle.update()
                    else:
                        shuttle.skiff.thrust_limit = 1
                    if conn.space_center.ut >= next_report:
                        writer.writerow([conn.space_center.ut, state, altitude,
                                         v.orbit.apoapsis_altitude, v.orbit.periapsis_altitude,
                                         result.throttles if shuttle.et_attached else (1,),
                                         result.residual if shuttle.et_attached else 0,
                                         result.feasible if shuttle.et_attached else True])
                        log.flush()
                        print(f'{state}: altitude {altitude/1000:.1f} km; Ap {v.orbit.apoapsis_altitude/1000:.1f} km; residual {result.residual:.0f} N m')
                        next_report = conn.space_center.ut+2
                    if ap.error > 45 and altitude > 1000:
                        raise RuntimeError('Attitude error exceeds 45 degrees')
                    if not shuttle.skiff.has_fuel:
                        raise RuntimeError('Skiff out of fuel')
                    time.sleep(max(0, 1/args.hz-(time.monotonic()-tick)))
        finally:
            control.throttle = 0
            try:
                set_autopilot_engaged(ap, False)
            finally:
                control.sas = True
            print('Control released; liquid throttle zero. Burning SRBs cannot be stopped.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--launch', action='store_true')
    parser.add_argument('--address', default='127.0.0.1')
    parser.add_argument('--rpc-port', type=int, default=50000)
    parser.add_argument('--stream-port', type=int, default=50001)
    parser.add_argument('--orbit', type=float, default=100000, help='Target altitude in m')
    parser.add_argument('--heading', type=float, default=90)
    parser.add_argument('--turn-start', type=float, default=1000)
    parser.add_argument('--turn-end', type=float, default=45000)
    parser.add_argument('--hz', type=float, default=10)
    parser.add_argument('--timeout', type=float, default=1800)
    parser.add_argument('--telemetry', default='telemetry.csv')
    args = parser.parse_args()
    if not 0 < args.hz <= 50 or not 0 <= args.turn_start < args.turn_end or args.timeout <= 0:
        parser.error('Invalid loop rate, turn altitudes, or timeout')
    run(args)


if __name__ == '__main__':
    main()
