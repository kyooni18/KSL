#!/usr/bin/env python3
"""Read-only MM304/MM305 forensic reconstruction (stdlib; no flight control).

Uses the recorded configuration, not current defaults. Distances are spherical
surface distances. Chart derivatives are measured finite differences; the
controller's orthonormal runway-right projection is retained separately.
"""
import argparse
import base64
import csv
import hashlib
import json
import math
from pathlib import Path


def dot(a, b): return sum(x*y for x, y in zip(a, b))
def mul(a, s): return [x*s for x in a]
def sub(a, b): return [x-y for x, y in zip(a, b)]
def add(a, b): return [x+y for x, y in zip(a, b)]
def norm(a): return math.sqrt(dot(a, a))
def unit(a): return mul(a, 1/max(1e-30, norm(a)))
def cross(a, b): return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
def vec(a): return [a[k] for k in ('x', 'y', 'z')]
def wrap(a): return math.atan2(math.sin(a), math.cos(a))
def project(a, n): return sub(a, mul(n, dot(a, n)))


class Geometry:
    def __init__(self, c):
        self.c = c
        r = c['runway']; lat, lon = r['latitude'], r['longitude']
        self.R = c['planet']['radius']
        self.up = [math.cos(lat)*math.cos(lon), math.cos(lat)*math.sin(lon), math.sin(lat)]
        east = [-math.sin(lon), math.cos(lon), 0]
        north = [-math.sin(lat)*math.cos(lon), -math.sin(lat)*math.sin(lon), math.cos(lat)]
        self.along = add(mul(north, math.cos(r['heading'])), mul(east, math.sin(r['heading'])))
        self.right = cross(self.along, self.up)

    def coordinates(self, p):
        n = unit(p); t = project(n, self.up)
        chart = mul(unit(t), self.R*math.atan2(norm(t), dot(n, self.up)))
        return dot(chart, self.along), dot(chart, self.right)

    def position(self, along, right):
        r = math.hypot(along, right)
        tangent = unit(add(mul(self.along, along), mul(self.right, right)))
        return add(mul(self.up, math.cos(r/self.R)), mul(tangent, math.sin(r/self.R)))

    def distance(self, p, q):
        p, q = unit(p), unit(q)
        return self.R*math.atan2(norm(cross(p, q)), dot(p, q))

    @staticmethod
    def heading(p, direction):
        u = unit(p); east = unit([-u[1], u[0], 0]); north = cross(u, east)
        return math.atan2(dot(direction, east), dot(direction, north))


def reconstruct(run):
    config = run/'live.jsonl.configuration.json'
    c = json.loads(config.read_text()); g = Geometry(c)
    e, guide = c['entry'], c['guidance']
    aim = g.position(-e.get('captureTargetRange', e['terminalTargetRange']), e['handoffCross'])
    # There is no separate KSC/map-marker coordinate in this run. Label this
    # reference explicitly: it is KSLCore's default 09 threshold, not "KSC".
    ksc_lat, ksc_lon = math.radians(-.0486111111), math.radians(-74.7283333333)
    ksc = [math.cos(ksc_lat)*math.cos(ksc_lon), math.cos(ksc_lat)*math.sin(ksc_lon), math.sin(ksc_lat)]
    omega = [0, 0, c['planet']['rotationRate']]
    polar = Path(__file__).resolve().parents[1]/'Sources/KSLPhysics/Resources/STSAerodynamics.csv'
    cells = [list(map(float, line.split(','))) for line in polar.read_text().splitlines()
             if line and not line.startswith(('#', 'mach'))]
    best_ld = max(max(0, x[2])/max(1e-8, x[3]) for x in cells)
    optimistic_ld = best_ld*(1+c['vehicle']['aerodynamicUncertainty'])/(1-c['vehicle']['aerodynamicUncertainty'])
    rows, proposals, raw = [], [], []
    objective_altitude, objective_speed = None, None
    for line_number, line in enumerate((run/'live.jsonl').open(), 1):
        a = json.loads(line); s = a.get('state')
        if s is None: continue
        raw.append(a)
        p, v = vec(s['position']), vec(s['velocity']); up = unit(p)
        hvel = project(v, up); vh = norm(hvel); along, right = g.coordinates(p)
        forward = unit(project(sub(g.position(along+1, right), g.position(along, right)), up))
        orthogonal_right = unit(cross(forward, up))
        course = g.heading(p, v); runway_course = g.heading(p, forward)
        bearing = g.heading(p, project(g.up, up)); aim_bearing = g.heading(p, project(aim, up))
        alignment = dot(unit(hvel), forward)
        d = a['guidance']; ae = s['aerodynamics']; en = d.get('entry', {})
        lift = ae['lift']/s['mass']; drag = ae['drag']/s['mass']
        vertical = dot(v, up); t = s['time']
        if not rows: t0 = t
        T = max(guide['pathLookaheadTime'], abs(-e['captureTargetRange']-along)/max(1, vh*abs(alignment)))
        cross_rate = -dot(v, orthogonal_right)
        demand = 6*(-right)/T**2+4*cross_rate/T
        pursuit = 2*vh*math.sin(wrap(aim_bearing-course))/T
        signed = alignment*demand+(1-alignment**2)*pursuit
        accel = mul(vec(ae['force']), 1/s['mass'])
        rotating = sub(mul(cross(omega, v), -2), cross(omega, cross(omega, p)))
        ground_right = unit(cross(unit(hvel), up))
        energy = dot(v, v)/2-c['planet']['gravitationalParameter']/norm(p)-norm(cross(omega, p))**2/2
        if d['phase'] == 'entry' or objective_altitude is None:
            objective_altitude = d['target']['desiredHeight']
            objective_speed = d['target']['desiredSpeed']
        op = mul(aim, g.R+objective_altitude)
        objective_energy = objective_speed**2/2-c['planet']['gravitationalParameter']/norm(op)-norm(cross(omega, op))**2/2
        gravity = c['planet']['gravitationalParameter']/norm(p)**2
        height = norm(p)-g.R-c['runway']['elevation']
        optimistic_range = optimistic_ld*max(0, height)+(optimistic_ld/2+1)*dot(v, v)/gravity
        stopping = vh**2/(2*c['vehicle']['brakeFriction']*gravity)
        reachable_along = min(max(0, along), max(0, c['runway']['length']-stopping))
        pavement_distance = g.distance(p, g.position(reachable_along, 0))
        row = dict(line=line_number, time=t, elapsed=t-t0, phase=d['phase'], altitude=norm(p)-g.R,
            mach=ae['mach'], airspeed=ae['airspeed'], horizontalSpeed=vh, verticalSpeed=vertical,
            specificEnergy=energy, objectiveEnergyMargin=energy-objective_energy,
            captureEnergyMargin=en.get('captureEnergyMargin'),
            optimisticRange=optimistic_range, pavementDistance=pavement_distance,
            referenceDrag=en.get('referenceDrag'), drag=drag, lift=lift,
            runwayRange=g.distance(p, g.up), targetRange=g.distance(p, aim), runway09ReferenceRange=g.distance(p, ksc),
            runway09ReferenceBearing=math.degrees(g.heading(p, project(ksc, up))),
            thresholdBearing=math.degrees(bearing), targetBearing=math.degrees(aim_bearing),
            along=along, cross=right, controllerCrossRate=cross_rate,
            course=math.degrees(course), runwayHeadingError=math.degrees(wrap(runway_course-course)),
            runwayBearingError=math.degrees(wrap(bearing-course)), targetBearingError=math.degrees(wrap(aim_bearing-course)),
            alignment=alignment, horizon=T, crossDemand=demand, signedLateral=signed,
            bankTarget=math.degrees(d['target']['bank']), bankActual=math.degrees(ae['bank']),
            bankLaw=math.degrees(en.get('commandedBank', d['target']['bank'])),
            alphaTarget=math.degrees(d['target']['angleOfAttack']), alphaActual=math.degrees(ae['angleOfAttack']),
            energyBank=math.degrees(en.get('energyBank', 0)), verticalLift=en.get('verticalLift'),
            forceRight=dot(accel, ground_right), liftBankRight=lift*math.sin(ae['bank']),
            commandLiftRight=lift*math.sin(d['target']['bank']),
            rotatingRight=dot(rotating, ground_right), crossForce=dot(accel, orthogonal_right),
            q=ae['dynamicPressure'], searched=d.get('capture', {}).get('mm305Searched', False),
            planID=d.get('planID'), pathRemaining=d.get('pathRemaining'),
            rollResponse=a.get('control', {}).get('rollResponseSeconds'),
            pitchResponse=a.get('control', {}).get('pitchResponseSeconds'))
        rows.append(row)
        if a.get('planningProposal'):
            z = json.loads(base64.b64decode(a['planningProposal']))
            proposals.append(dict(received=t-t0, requested=z['time']-t0, age=t-z['time'], proposal=z))
    # Symmetric measured derivatives. Do not label velocity projection a chart derivative.
    for i, r in enumerate(rows):
        lo, hi = rows[max(0, i-1)], rows[min(len(rows)-1, i+1)]
        dt = hi['time']-lo['time']
        for k in ('along', 'cross'):
            r[k+'Rate'] = (hi[k]-lo[k])/dt if dt else 0
        r['courseTurnAcceleration'] = wrap(math.radians(hi['course']-lo['course']))/max(dt, 1e-30)*r['horizontalSpeed']
        # d(course)/dt includes meridian transport. The vector derivative is
        # the force-comparable acceleration in the rotating Cartesian frame.
        state = raw[i]['state']; up = unit(vec(state['position']))
        ground_right = unit(cross(unit(project(vec(state['velocity']), up)), up))
        dv = sub(vec(raw[min(len(raw)-1, i+1)]['state']['velocity']),
                 vec(raw[max(0, i-1)]['state']['velocity']))
        r['measuredTurnAcceleration'] = dot(dv, ground_right)/max(dt, 1e-30)
    return c, rows, proposals, raw


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('run', type=Path)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    c, rows, proposals, raw = reconstruct(args.run)
    def first(predicate): return next((r for r in rows if predicate(r)), None)
    def near(t): return min(rows, key=lambda r: abs(r['elapsed']-t))
    target_along = -c['entry']['captureTargetRange']
    events = {'initial': rows[0], 'firstTargetAlongCrossing': first(lambda r: r['along'] <= target_along),
        'closestTarget': min(rows, key=lambda r: r['targetRange']),
        'closestRunway': min(rows, key=lambda r: r['runwayRange']),
        'firstOptimisticRangeExhausted': first(lambda r: r['airspeed'] < 500 and r['altitude'] < 20000
            and r['pavementDistance'] > r['optimisticRange']),
        'firstMM305Diagnostic': first(lambda r: r['searched']),
        'firstAcquisition': first(lambda r: r['phase'] == 'acquisition'),
        'firstSpiral': first(lambda r: r['phase'] == 'headingAlignment'), 'last': rows[-1]}
    terminal = [p for p in proposals if p['proposal'].get('terminal')]
    if terminal:
        events['firstMM305Request'] = near(terminal[0]['requested'])
        events['firstMM305Completion'] = near(terminal[0]['received'])
    survey_path = args.output/'survey.json'
    if survey_path.exists():
        survey = json.loads(survey_path.read_text())
        opportunity = next((s for s in survey if s['envelope']['feasible'] > 0), None)
        if opportunity: events['firstSurveyFamilyOpportunity'] = near(opportunity['elapsed'])
    for t in [60, 120, 180, 240, 300, 360, 400, 420, 440, 460, 480, 500, 540, 600, 700, 800]:
        events[f'sample{t}'] = near(t)
    args.output.joinpath('events.json').write_text(json.dumps(events, indent=2)+'\n')
    args.output.joinpath('proposals.json').write_text(json.dumps(proposals, indent=2)+'\n')
    with args.output.joinpath('geometry.csv').open('w') as f:
        w = csv.DictWriter(f, fieldnames=rows[0]); w.writeheader(); w.writerows(rows)
    hashes = {n: hashlib.sha256(args.run.joinpath(n).read_bytes()).hexdigest()
              for n in ['live.jsonl', 'live.jsonl.configuration.json', 'source.patch']}
    args.output.joinpath('source-hashes.json').write_text(json.dumps(hashes, indent=2)+'\n')
    keys = ['elapsed', 'altitude', 'mach', 'airspeed', 'verticalSpeed', 'along', 'cross', 'targetRange',
            'runwayRange', 'runwayHeadingError', 'targetBearingError', 'bankTarget', 'bankActual',
            'alphaTarget', 'alphaActual', 'objectiveEnergyMargin', 'captureEnergyMargin']
    for name, r in events.items():
        if r is None:
            print(name, 'not observed')
            continue
        print(name, ' '.join(f'{k}={r[k]:.3f}' if isinstance(r[k], (int, float)) else f'{k}={r[k]}' for k in keys))
    print('terminal proposals:', len(terminal))
    for p in terminal[:4]:
        z = p['proposal']['terminal']; plan = z.get('plan') or {}
        print('terminal', p['requested'], p['received'], p['age'], 'fit', z.get('fit'),
              'envelope', z.get('envelope'), 'assessment', plan.get('assessment'), 'spiral', plan.get('spiral'))


if __name__ == '__main__': main()
