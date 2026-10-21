"""Read-only kRPC snapshot for the STS solid-booster trajectory model."""
import math
from .preplan import MassPoint, Motor, VehicleModel


def snapshot(shuttle, args):
    if shuttle.vehicle_kind != 'orbiter-shuttle' or not shuttle.solid_boosters:
        raise ValueError('Preplanned ascent currently requires STS with solid boosters')
    v, body = shuttle.vessel, shuttle.vessel.orbit.body
    frame = v.reference_frame
    clamps = {c.part for c in v.parts.launch_clamps}
    booster_parts = {p for branch in shuttle.booster_branch_parts for p in branch}
    solid_parts = {e.part for e in shuttle.solid_boosters}
    points = []

    def fuel_mass(part, resources):
        # Missing telemetry is a planning error, not an empty tank. The legacy
        # runtime helper intentionally tolerates stale handles after staging.
        res = part.resources
        names = set(res.names)
        return sum(res.amount(name)*res.density(name) for name in resources if name in names)

    for part in v.parts.all:
        if part in clamps:
            continue
        position = tuple(part.position(frame))
        pool = 'solid' if part in booster_parts else 'liquid'
        if pool == 'solid':
            fuel = fuel_mass(part, ('SolidFuel',)) if part in solid_parts else 0.
        else:
            fuel = fuel_mass(part, ('LiquidFuel', 'Oxidizer'))
        if fuel > part.mass+1.:
            raise ValueError(f'Inconsistent propellant/part mass for {part.name}')
        if fuel:
            points.append(MassPoint(fuel, position, pool))
        points.append(MassPoint(max(0., part.mass-fuel), position,
                                'booster-dry' if part in booster_parts else 'dry'))
    def motor(engine):
        # Part mounting axis intentionally excludes live gimbal deflection.
        return Motor(float(engine.max_vacuum_thrust), float(engine.vacuum_specific_impulse),
                     float(engine.kerbin_sea_level_specific_impulse),
                     tuple(engine.part.position(frame)), tuple(engine.part.direction(frame)),
                     math.radians(engine.gimbal_range)*engine.gimbal_limit if engine.gimballed else 0.)
    atmosphere = tuple((float(h), float(body.density_at(h)), float(body.pressure_at(h))/101325.)
                       for h in range(0, int(body.atmosphere_depth)+2000, 2000))
    flight = v.flight(body.reference_frame)
    r = body.equatorial_radius+flight.mean_altitude
    latitude = math.radians(flight.latitude)
    vrot = body.rotational_speed*r*math.cos(latitude)
    # Project eastward rotation into the launch orbital plane, not its magnitude.
    inclination = math.radians(args.inclination)
    east_fraction = math.cos(inclination)/max(.01, math.cos(latitude))
    if args.heading is not None:
        east_fraction = math.sin(math.radians(args.heading))
    elif abs(math.cos(inclination)) > math.cos(latitude):
        raise ValueError('Target inclination is below the launch latitude')
    rotation = vrot*max(-1., min(1., east_fraction))
    low, high = v.bounding_box(frame)
    # Cd*A cannot be measured on the stationary pad. Save its provenance and
    # permit an explicit calibrated value rather than pretending geometry is Cd.
    drag_area = args.plan_drag_area
    source = 'configured Cd*A' if drag_area is not None else 'estimated: 0.3 * frontal bounding box'
    if drag_area is None:
        drag_area = .3*abs(high[0]-low[0])*abs(high[2]-low[2])
    expected_mass = v.mass-sum(p.mass for p in clamps)
    if abs(sum(p.mass for p in points)-expected_mass) > max(10., .01*expected_mass):
        raise ValueError('Part-mass snapshot does not match live vessel mass')
    inertia = float(v.moment_of_inertia[0])-sum(
        p.mass*(p.position(frame)[1]**2+p.position(frame)[2]**2) for p in clamps)
    roll_angle = 180.  # conservative allowance for heads-down roll plus settling
    return VehicleModel(tuple(points), tuple(motor(e) for group in shuttle.core_groups for e in group),
                        tuple(motor(e) for e in shuttle.solid_boosters), atmosphere,
                        radius=body.equatorial_radius, mu=body.gravitational_parameter,
                        atmosphere_depth=body.atmosphere_depth, pad_altitude=flight.mean_altitude,
                        rotation_speed=rotation, inertia_pitch=inertia,
                        drag_area=drag_area, drag_source=source,
                        booster_limit=shuttle.solid_boosters[0].thrust_limit,
                        initial_core_limit=args.ssme_limit, max_q=args.max_q,
                        max_g=args.max_g, upper_g=min(args.max_g, args.upper_g),
                        pitch_slew=args.pitch_slew, hold_slew=args.post_sep_hold_slew,
                        throttle_slew=args.throttle_slew, turn_start=args.turn_start,
                        roll_start=args.roll_start,
                        roll_duration=max(args.roll_time, roll_angle/args.roll_rate)+args.roll_settle_time,
                        upright_delay=args.roll_reset_delay,
                        upright_duration=max(args.upright_roll_time, 180./args.upright_roll_rate)+args.roll_settle_time,
                        upright_max_q=args.upright_max_q, separation_max_q=args.booster_separation_max_q,
                        post_booster_coast=args.post_booster_coast_seconds,
                        prelaunch_core_seconds=args.ssme_lead_time)
