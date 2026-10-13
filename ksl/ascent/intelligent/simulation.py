"""Independent RK4 spherical ascent plant for offline guidance experiments.

This deliberately uses a separate integrator, atmosphere and mass depletion from
planner.step. It is an ideal-attitude point mass, not a KSP aerodynamic validation.
"""
import json
import math
from dataclasses import replace
from .planner import State, Target, Planner, apsides, coast_apoapsis


def simulate(mass=30000., thrust=600000., isp=330., dry=5000., altitude=90000., dt=.25, periapsis=None):
    periapsis = altitude if periapsis is None else periapsis
    target = Target(600000., 3.5316e12, 70000., altitude, periapsis)
    planner = Planner(target)
    # No planetary rotation: the harder zero-horizontal-speed launch case.
    y = [0., 0., 0., mass]
    peak_q = 0.
    phase = 'ascent'
    angle = 0.
    maneuver = None
    attempts = 0
    from .. import orbit_math as om
    for tick in range(int(2400/dt)):
        h, vr, vt, m = y
        speed = math.hypot(vr, vt)
        density = 1.225*math.exp(-max(0., h)/5600.) if h < 70000 else 0.
        q = .5*density*speed*speed
        peak_q = max(peak_q, q)
        drag = q*4./m
        state = State(h, vr, vt, m, thrust if m > dry else 0., isp, q, drag)
        ap, pe = apsides(state, target)
        # Check achieved orbital elements of the independent plant.
        if abs(pe-periapsis) < 1000 and abs(ap-altitude) < 1000:
            return dict(success=True, time=tick*dt, apoapsis=ap, periapsis=pe, mass=m, peakQ=peak_q)
        if (tick > 10 and h < 0) or m <= dry:
            return dict(success=False, time=tick*dt, altitude=h, apoapsis=ap, periapsis=pe,
                        mass=m, peakQ=peak_q, reason='impact' if h < 0 else 'fuel')
        if phase == 'ascent' and coast_apoapsis(state, target) >= altitude-100:
            phase = 'coast'
        now = tick*dt
        pitch, throttle = 0., 0.
        radial = (math.cos(angle), math.sin(angle), 0.)
        tangent = (-math.sin(angle), math.cos(angle), 0.)
        position = om.scale(radial, target.radius+h)
        velocity = om.add(om.scale(radial, vr), om.scale(tangent, vt))
        if phase == 'ascent':
            cmd = planner.command(state, dt)
            pitch, throttle = math.radians(cmd.pitch), cmd.throttle
        elif h > target.atmosphere+1000 and maneuver is None:
            solution = om.solve_one_burn(position, velocity, target.mu, altitude, periapsis, 0.,
                                         target.radius, min_lead=5., deadline=2000. if pe > target.atmosphere else 500.)
            if solution is None:
                return dict(success=False, reason='no correction', apoapsis=ap, periapsis=pe)
            attempts += 1
            if attempts > 16:
                return dict(success=False, reason='correction budget', apoapsis=ap, periapsis=pe)
            dv = solution['dv_vec']
            duration = om.norm(dv)/(thrust/m)
            maneuver = [now+solution['t']-duration/2, dv]
        if maneuver is not None and now >= maneuver[0]:
            dv = maneuver[1]
            remaining = om.norm(dv)
            if remaining < .15:
                maneuver = None
            else:
                direction = om.unit(dv)
                pitch = math.atan2(om.dot(direction, radial), om.dot(direction, tangent))
                throttle = min(1., remaining/max(thrust/m*2., .1))
                maneuver[1] = om.add(dv, direction, -thrust/m*throttle*dt)
        if phase == 'coast' and h < target.atmosphere and vr < 0:
            return dict(success=False, reason='reentry', apoapsis=ap, periapsis=pe)
        if __import__('os').environ.get('KSL_SIM_TRACE') and tick % int(10/dt) == 0:
            print(round(now), phase, *(round(x,1) for x in (h,vr,vt,ap,pe,math.degrees(pitch),throttle)))
        def derivative(v):
            h, vr, vt, m = v
            r = target.radius+h
            speed = max(1., math.hypot(vr, vt))
            rho = 1.225*math.exp(-max(0., h)/5600.) if h < 70000 else 0.
            drag = .5*rho*speed*speed*4./m
            a = thrust*throttle/m
            return [vr, a*math.sin(pitch)-target.mu/r**2+vt*vt/r-drag*vr/speed,
                    a*math.cos(pitch)-vr*vt/r-drag*vt/speed, -thrust*throttle/(isp*9.80665)]
        k1 = derivative(y)
        k2 = derivative([a+dt*b/2 for a,b in zip(y,k1)])
        k3 = derivative([a+dt*b/2 for a,b in zip(y,k2)])
        k4 = derivative([a+dt*b for a,b in zip(y,k3)])
        angle += vt/(target.radius+h)*dt
        y = [a+dt*(b+2*c+2*d+e)/6 for a,b,c,d,e in zip(y,k1,k2,k3,k4)]
    return dict(success=False, reason='timeout', altitude=y[0], apoapsis=ap, periapsis=pe)


if __name__ == '__main__':
    for mass, thrust, dry in ((30000.,600000.,5000.), (80000.,1200000.,12000.), (10000.,250000.,1500.)):
        for altitude in (80000.,120000.,200000.):
            print(json.dumps(dict(initialMass=mass, thrust=thrust, target=altitude,
                                  **simulate(mass,thrust,340.,dry,altitude))), flush=True)
