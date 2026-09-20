#include "../CLanding/guidance.c"
#include <assert.h>

int main(void){
    LandingConfiguration cfg=landing_configuration_default();
    landing_configuration_normalize(&cfg);
    cfg.site.runway_heading=90;cfg.guidance.taem_interface_altitude=20000;
    cfg.guidance.taem_force_handoff_speed=1300;
    PlanetModel p={.radius=600000,.gravitational_parameter=3531600000000.0,
        .rotational_speed=.000291570900559802,.atmosphere_depth=70000,
        .surface_density=1.225,.atmosphere_adiabatic_index=1.4};
    AerodynamicModel aero={.lift_to_drag=.8,.ballistic_coefficient=700,.confidence=.8};
    /* MM304 owns the fixed final-alignment station and publishes one latched
       course inside the explicit perpendicular-heading contract. Speed is
       constrained by the structural atmosphere envelope and unpowered energy. */
    GuidanceMachine tangent={0};
    tangent.s_turn_sign=1.0;
    Telemetry ingress;telemetry_init(&ingress);
    ingress.ut=50.0;ingress.mean_altitude=50000.0;ingress.radar_altitude=50000.0;
    ingress.true_air_speed=2050.0;ingress.horizontal_speed=2030.0;
    ingress.vertical_speed=-60.0;ingress.flight_path_angle=-1.7;
    ingress.ground_track_heading=90.0;ingress.heading=90.0;
    ingress.runway_along_track=-400000.0;ingress.runway_cross_track=0.0;
    entry_publish_taem_tangent_target(&tangent,&ingress,90.0,&p,aero,&cfg);
    assert(tangent.taem_interface_target.valid);
    assert(entry_taem_tangent_target_geometry(&tangent.taem_interface_target,&cfg));
    double station=-cfg.guidance.final_approach_distance;
    assert(fabs(tangent.taem_interface_target.along_track-station)<1e-6);
    assert(fabs(tangent.taem_interface_target.cross_track)<1e-6);
    assert(fabs(hypot(tangent.taem_interface_target.along_track-station,
        tangent.taem_interface_target.cross_track))<1e-6);
    TaemSpeedEnvelope speed_envelope=decision_taem_speed_envelope(
        &cfg.vehicle,&p,tangent.taem_interface_target.altitude);
    assert(speed_envelope.valid&&speed_envelope.feasible);
    assert(tangent.taem_interface_target.speed>=
        speed_envelope.minimum_speed_mps-1e-6);
    assert(tangent.taem_interface_target.speed<=
        speed_envelope.maximum_speed_mps+1e-6);

    double rh=cfg.site.runway_heading*DEG2RAD;
    GeoPoint origin={cfg.site.latitude,cfg.site.longitude,cfg.site.altitude};
    GeoPoint target_point=local_point(origin,station*sin(rh),station*cos(rh),
        p.radius,tangent.taem_interface_target.altitude);
    double current_energy=rotating_specific_energy(
        ingress.latitude,ingress.mean_altitude,ingress.true_air_speed,&p);
    double zero_target_energy=rotating_specific_energy(
        target_point.latitude,tangent.taem_interface_target.altitude,0.0,&p);
    double energy_speed=sqrt(fmax(0.0,2.0*(current_energy-zero_target_energy)));
    double expected_speed=fmin(speed_envelope.maximum_speed_mps,energy_speed);
    if(expected_speed<speed_envelope.minimum_speed_mps)
        expected_speed=speed_envelope.minimum_speed_mps;

    double expected_course=norm_deg(
        cfg.site.runway_heading-90.0); /* decision-literal-ok: perpendicular runway geometry */
    assert(fabs(norm_signed_deg(
        tangent.taem_interface_target.course-expected_course))<1e-6);
    assert(fabs(tangent.taem_interface_target.speed-expected_speed)<1e-6);
    double initial_offset=fabs(norm_signed_deg(
        tangent.taem_interface_target.course-cfg.site.runway_heading));
    assert(fabs(initial_offset-90.0)<1e-6); /* decision-literal-ok: perpendicular runway geometry */

    /* A measured arc may prove that a shallower outlet would be easier to reach,
       but that is not permission to rewrite the terminal contract merely to make
       the final reversal admissible. Construct a state exactly on a 60 deg setup
       circle while the published inlet remains perpendicular: the helper must keep
       the requested course unchanged and leave the 90 deg endpoint unproven. */
    GuidanceMachine relaxed=tangent;
    relaxed.taem_interface_target.course=norm_deg(cfg.site.runway_heading-90.0);
    relaxed.taem_interface_target.speed=1300.0;
    Telemetry relax_arc=ingress;
    const double arc_radius=50000.0;
    relax_arc.mass=40000.0;relax_arc.true_air_speed=1300.0;relax_arc.horizontal_speed=1300.0;
    relax_arc.ground_track_heading=relax_arc.heading=cfg.site.runway_heading;
    relax_arc.dynamic_pressure=12000.0;relax_arc.g_force=1.0;relax_arc.bank_effectiveness=1.0;
    relax_arc.stall_fraction=0.0;relax_arc.stall_fraction_is_measured=true;
    double lateral=relax_arc.true_air_speed*relax_arc.true_air_speed/arc_radius;
    double proof_bank=fmin(55.0,dynamic_bank_limit(&relax_arc,&cfg.vehicle));
    relax_arc.lift_force=relax_arc.mass*lateral/sin(proof_bank*DEG2RAD);
    double final_sign=-1.0;
    double relax_psi0=0.0,psi60=-60.0*DEG2RAD;
    relax_arc.runway_along_track=station-arc_radius/final_sign*(sin(psi60)-sin(relax_psi0));
    relax_arc.runway_cross_track=-arc_radius/final_sign*(cos(relax_psi0)-cos(psi60));
    double published_course=relaxed.taem_interface_target.course;
    assert(!entry_program_final_reversal_geometry_ready(&relaxed,&relax_arc,&cfg,final_sign));
    assert(!entry_program_relax_terminal_course_if_ready(&relaxed,&relax_arc,&cfg,final_sign));
    assert(fabs(norm_signed_deg(relaxed.taem_interface_target.course-published_course))<1e-6);
    assert(!entry_program_final_reversal_geometry_ready(&relaxed,&relax_arc,&cfg,final_sign));
    assert(entry_taem_tangent_target_geometry(&relaxed.taem_interface_target,&cfg));

    /* Turn admission must respect the measured current-speed physical radius.
       This 50 km circle is exactly reachable geometrically, but at 2000 m/s the
       measured lift cannot generate it, so a future lower TAEM speed must not make
       the current endpoint appear flyable.  After deceleration into the actual
       radius envelope, the identical geometry becomes admissible. */
    GuidanceMachine high_speed=relaxed;
    high_speed.taem_interface_target.course=norm_deg(cfg.site.runway_heading-60.0);
    Telemetry fast_arc=relax_arc;
    fast_arc.true_air_speed=fast_arc.horizontal_speed=2000.0;
    fast_arc.runway_along_track=relax_arc.runway_along_track;
    fast_arc.runway_cross_track=relax_arc.runway_cross_track;
    assert(!entry_program_terminal_course_endpoint_ready(&high_speed,&fast_arc,&cfg,
        final_sign,high_speed.taem_interface_target.course,NULL,NULL));
    fast_arc.true_air_speed=fast_arc.horizontal_speed=1250.0;
    assert(entry_program_terminal_course_endpoint_ready(&high_speed,&fast_arc,&cfg,
        final_sign,high_speed.taem_interface_target.course,NULL,NULL));

    /* A maximum-bank radius is a lower bound, not the only flyable circle. Reproduce
       the live 25 km geometry that previously missed by ~30 km when forced onto the
       propagated ~31 km radius: a wider ~50 km-class arc is inside the same endpoint
       tube and must be accepted rather than delaying the reversal until energy is gone. */
    GuidanceMachine wide_arc=tangent;
    wide_arc.taem_interface_target.course=norm_deg(cfg.site.runway_heading-45.0);
    wide_arc.taem_interface_target.speed=820.0;
    Telemetry wide_state=ingress;
    wide_state.true_air_speed=wide_state.horizontal_speed=1103.0;
    wide_state.ground_track_heading=wide_state.heading=103.8;
    wide_state.runway_along_track=-65866.0;
    wide_state.runway_cross_track=17769.0;
    wide_state.mass=43603.0;wide_state.lift_force=wide_state.mass*35.0;
    wide_state.dynamic_pressure=9000.0;wide_state.g_force=1.0;wide_state.bank_effectiveness=1.0;
    wide_state.stall_fraction=0.0;wide_state.stall_fraction_is_measured=true;
    double wide_error=NAN,wide_radius=NAN;
    assert(entry_program_terminal_course_endpoint_ready(&wide_arc,&wide_state,&cfg,-1.0,
        wide_arc.taem_interface_target.course,&wide_error,&wide_radius));
    assert(wide_radius>45000.0);
    assert(wide_error<wide_radius*.25);

    /* The current-speed physical radius may be much wider than the eventual HAC.
       With this measured lift and ~1.22 km/s horizontal speed the minimum flyable
       circle is ~235 km; the endpoint is still far outside the bounded admission tube,
       so turn-start must remain false.  The final <=1 km capture contract is separate. */
    GuidanceMachine v20_arc=tangent;
    v20_arc.taem_interface_target.course=norm_deg(cfg.site.runway_heading-45.0);
    v20_arc.taem_interface_target.speed=640.3;
    Telemetry v20_state=ingress;
    v20_state.true_air_speed=1227.4;v20_state.horizontal_speed=1219.2;
    v20_state.ground_track_heading=v20_state.heading=102.0;
    v20_state.runway_along_track=-107562.5;v20_state.runway_cross_track=13871.3;
    v20_state.mass=43729.0;v20_state.lift_force=v20_state.mass*8.0;
    v20_state.dynamic_pressure=9000.0;v20_state.g_force=1.0;v20_state.bank_effectiveness=.97;
    v20_state.stall_fraction=0.0;v20_state.stall_fraction_is_measured=true;
    double v20_error=NAN,v20_radius=NAN;
    bool v20_ready=entry_program_terminal_course_endpoint_ready(&v20_arc,&v20_state,&cfg,-1.0,
        v20_arc.taem_interface_target.course,&v20_error,&v20_radius);
    assert(v20_radius>220000.0&&v20_radius<250000.0);
    double v20_tolerance=fmax(2500.0,fmin(10000.0,v20_radius*.12));
    assert(v20_error>v20_tolerance);
    assert(!v20_ready);
    double nominal_taem_speed=entry_taem_speed_target(&cfg.vehicle,&cfg.guidance,&p);
    /* The bootstrap inlet deliberately trades the old high-speed TAEM target for
       a low-energy HAC-compatible state while retaining the fixed legal inlet pose. */
    assert(tangent.taem_interface_target.speed>=taem_speed_min-1e-6);
    assert(tangent.taem_interface_target.speed<=fmin(500.0,taem_speed_max)+1e-6);
    assert(tangent.taem_interface_target.speed<=nominal_taem_speed+1e-6);
    assert(fabs(fabs(norm_signed_deg(tangent.taem_interface_target.course-
        cfg.site.runway_heading))-90.0)<1e-6);
    double final_alignment_altitude=cfg.site.altitude+cfg.guidance.final_approach_distance*
        tan(cfg.guidance.final_glide_slope*DEG2RAD);
    assert(tangent.taem_interface_target.altitude>final_alignment_altitude+1000.0);
    assert(tangent.taem_interface_target.altitude<p.atmosphere_depth);
    assert(tangent.taem_interface_target.flight_path_angle<=-7.0);
    assert(tangent.taem_interface_target.flight_path_angle>=-20.0);

    /* Lateral offset is now an independent geometry demand: even when longitudinal
       energy wants wings-level, MM304 must create only the curvature needed to reach
       the latched tangent point, without using geometry to trigger an unplanned side flip. */
    Telemetry gate_guidance=ingress;
    gate_guidance.mass=40000.0;gate_guidance.lift_force=gate_guidance.mass*2.4;
    gate_guidance.dynamic_pressure=1400.0;gate_guidance.g_force=1.0;
    gate_guidance.bank_effectiveness=1.0;gate_guidance.stall_fraction=0.0;
    gate_guidance.stall_fraction_is_measured=true;
    double gate_bank=entry_taem_gate_acquisition_bank(&tangent,&gate_guidance,&p,aero,&cfg);
    assert(gate_bank>0.1&&gate_bank<=dynamic_bank_limit(&gate_guidance,&cfg.vehicle)+1e-6);
    gate_guidance.ground_track_heading=100.0;gate_guidance.heading=100.0;
    double no_flip_bank=entry_taem_gate_acquisition_bank(&tangent,&gate_guidance,&p,aero,&cfg);
    assert(no_flip_bank*tangent.s_turn_sign>=-1e-9);
    assert(!tangent.entry_reversal_scheduled);

    /* Once the fixed station has been crossed without capture, the tangent line
       must not become an infinite straight-away. Continue the owned terminal
       turn, then home back onto the point. */
    Telemetry missed=gate_guidance;
    double tangent_rel=norm_signed_deg(tangent.taem_interface_target.course-
        cfg.site.runway_heading)*DEG2RAD;
    missed.runway_along_track=tangent.taem_interface_target.along_track+3000.0*cos(tangent_rel);
    missed.runway_cross_track=tangent.taem_interface_target.cross_track+3000.0*sin(tangent_rel);
    missed.range_to_site=hypot(missed.runway_along_track,missed.runway_cross_track);
    missed.true_air_speed=tangent.taem_interface_target.speed+40.0;
    missed.horizontal_speed=missed.true_air_speed;
    missed.ground_track_heading=missed.heading=tangent.taem_interface_target.course;
    GuidanceMachine missed_machine=tangent;
    missed_machine.entry_control_reversals=1;
    missed_machine.entry_final_reversal_completed=true;
    double missed_bearing=NAN;
    assert(entry_taem_alignment_station_missed(&missed_machine,&missed,&cfg,&missed_bearing));
    assert(isfinite(missed_bearing));
    double recovery_heading=entry_taem_tangent_capture_heading(&missed_machine,&missed,&cfg);
    assert(fabs(norm_signed_deg(recovery_heading-missed_machine.taem_interface_target.course))>30.0);
    double recovery_bank=entry_taem_tangent_capture_bank(&missed_machine,&missed,aero,&cfg,recovery_heading);
    assert(recovery_bank*missed_machine.s_turn_sign>0.1);
    assert(fabs(recovery_bank)<=dynamic_bank_limit(&missed,&cfg.vehicle)+1e-6);

    /* Diagonal passage must also count as a missed fixed point. This is the live-map
       failure mode: the vehicle crossed the -8 km station mostly along the runway axis,
       so tangent-axis progress stayed near zero and the old detector flew straight on. */
    Telemetry diagonal=missed;
    diagonal.runway_along_track=tangent.taem_interface_target.along_track+2500.0;
    diagonal.runway_cross_track=tangent.taem_interface_target.cross_track;
    diagonal.range_to_site=hypot(diagonal.runway_along_track,diagonal.runway_cross_track);
    diagonal.ground_track_heading=diagonal.heading=cfg.site.runway_heading;
    double diagonal_bearing=NAN;
    assert(entry_taem_alignment_station_missed(&missed_machine,&diagonal,&cfg,&diagonal_bearing));
    double diagonal_recovery=entry_taem_tangent_capture_heading(&missed_machine,&diagonal,&cfg);
    assert(fabs(norm_signed_deg(diagonal_recovery-diagonal.ground_track_heading))>30.0);

    /* The same tangent-state contract defines MM304's upstream vertical corridor.
       Being below that corridor must never cause the legacy endpoint schedule to add
       sink/bank; being materially above it may still invoke vertical capture. */
    Telemetry corridor=ingress;
    double corridor_axis=norm_signed_deg(tangent.taem_interface_target.course-cfg.site.runway_heading)*DEG2RAD;
    corridor.runway_along_track=tangent.taem_interface_target.along_track-95000.0*cos(corridor_axis);
    corridor.runway_cross_track=tangent.taem_interface_target.cross_track-95000.0*sin(corridor_axis);
    corridor.dynamic_pressure=1200.0;corridor.g_force=1.0;
    corridor.true_air_speed=tangent.taem_interface_target.speed+120.0;
    corridor.horizontal_speed=corridor.true_air_speed;
    corridor.flight_path_angle=tangent.taem_interface_target.flight_path_angle;
    corridor.stall_fraction=0.0;corridor.stall_fraction_is_measured=true;
    double corridor_h=entry_program_taem_corridor_altitude(&tangent,&corridor,&cfg);
    double corridor_lead=fmax(fmax(0.0,tangent.taem_interface_target.acquisition_lead),
        fmax(0.0,tangent.taem_interface_target.speed)*
        fmax(4.0,tangent.taem_interface_target.response_time));
    double corridor_slope=clampd(-tangent.taem_interface_target.flight_path_angle,3.0,25.0)*DEG2RAD;
    double expected_corridor_h=tangent.taem_interface_target.altitude+
        fmin(95000.0,corridor_lead)*tan(corridor_slope);
    assert(isfinite(corridor_h)&&fabs(corridor_h-expected_corridor_h)<1e-6);
    assert(corridor_h>tangent.taem_interface_target.altitude+100.0);
    corridor.mean_altitude=corridor_h-2500.0;
    double corridor_bank=5.0,corridor_aoa=cfg.vehicle.entry_angle_of_attack;
    assert(!entry_program_altitude_capture(&tangent,&corridor,&p,&cfg,&corridor_bank,&corridor_aoa));
    assert(fabs(corridor_bank-5.0)<1e-9);

    corridor.mean_altitude=corridor_h+2500.0;
    corridor.flight_path_angle=-1.0;
    corridor_bank=5.0;corridor_aoa=cfg.vehicle.entry_angle_of_attack;
    assert(entry_program_altitude_capture(&tangent,&corridor,&p,&cfg,&corridor_bank,&corridor_aoa));
    assert(fabs(corridor_bank)>5.0);


    /* With the demonstrated ~21.8 km TAEM seam, the same v25/v32 upstream state now
       lies close to the real line-of-sight descent path instead of above a fictitious
       +6.5 km inlet reserve. Vertical protection must therefore leave broad-arc bank
       authority available rather than forcing wings-level flight. */
    Telemetry steep=ingress;
    steep.mass=40000.0;steep.lift_force=steep.mass*7.0;
    steep.mean_altitude=37700.0;steep.radar_altitude=37700.0;
    steep.true_air_speed=1834.0;steep.horizontal_speed=1825.0;
    steep.flight_path_angle=-4.5;steep.vertical_speed=1834.0*sin(-4.5*DEG2RAD);steep.dynamic_pressure=5000.0;steep.g_force=1.0;
    steep.bank_effectiveness=1.0;steep.stall_fraction=0.0;steep.stall_fraction_is_measured=true;
    steep.runway_along_track=tangent.taem_interface_target.along_track-208000.0;
    steep.runway_cross_track=tangent.taem_interface_target.cross_track-28000.0;
    double steep_distance=NAN;
    double steep_path_fpa=entry_program_taem_path_fpa(&tangent,&steep,&cfg,&steep_distance);
    double steep_limit=entry_program_vertical_bank_ceiling(&tangent,&steep,&p,aero,&cfg);
    assert(steep_distance>200000.0&&steep_distance<220000.0);
    double steep_los_fpa=atan2(tangent.taem_interface_target.altitude-steep.mean_altitude,
        steep_distance)*RAD2DEG;
    assert(fabs(steep_path_fpa-clampd(steep_los_fpa,-24.0,6.0))<0.1);
    assert(steep_limit>40.0&&steep_limit<=dynamic_bank_limit(&steep,&cfg.vehicle));
    steep.flight_path_angle=-2.0;steep.vertical_speed=1834.0*sin(-2.0*DEG2RAD);
    double shallow_limit=entry_program_vertical_bank_ceiling(&tangent,&steep,&p,aero,&cfg);
    assert(shallow_limit>40.0&&shallow_limit<=dynamic_bank_limit(&steep,&cfg.vehicle));

    /* Geometry demand is still subordinate to a genuine vertical-path emergency.
       Use an intentionally over-steep state to prove the safety ceiling can clip the
       lateral request, then verify that ordinary shallow flight retains the arc. */
    Telemetry geometry_hold=ingress;
    geometry_hold.mass=40000.0;geometry_hold.lift_force=geometry_hold.mass*4.0;
    geometry_hold.mean_altitude=41400.0;geometry_hold.radar_altitude=41400.0;
    geometry_hold.true_air_speed=1923.0;geometry_hold.horizontal_speed=1900.0;
    geometry_hold.flight_path_angle=-7.0;geometry_hold.vertical_speed=1923.0*sin(-7.0*DEG2RAD);geometry_hold.dynamic_pressure=3000.0;geometry_hold.g_force=1.0;
    geometry_hold.bank_effectiveness=1.0;geometry_hold.stall_fraction=0.0;geometry_hold.stall_fraction_is_measured=true;
    double geometry_axis=norm_signed_deg(tangent.taem_interface_target.course-cfg.site.runway_heading)*DEG2RAD;
    double geometry_along=270000.0,geometry_cross=30675.0;
    double geometry_da=geometry_along*cos(geometry_axis)-geometry_cross*sin(geometry_axis);
    double geometry_dc=geometry_along*sin(geometry_axis)+geometry_cross*cos(geometry_axis);
    geometry_hold.runway_along_track=tangent.taem_interface_target.along_track-geometry_da;
    geometry_hold.runway_cross_track=tangent.taem_interface_target.cross_track-geometry_dc;
    double hold_path_distance=NAN;
    double hold_path_fpa=entry_program_taem_path_fpa(&tangent,&geometry_hold,&cfg,&hold_path_distance);
    assert(isfinite(hold_path_fpa)&&isfinite(hold_path_distance));
    geometry_hold.flight_path_angle=hold_path_fpa-3.5;
    geometry_hold.vertical_speed=geometry_hold.true_air_speed*sin(geometry_hold.flight_path_angle*DEG2RAD);
    geometry_hold.ground_track_heading=92.4;geometry_hold.heading=92.4;
    double vertical_ceiling=entry_program_vertical_bank_ceiling(&tangent,&geometry_hold,&p,aero,&cfg);
    assert(isfinite(vertical_ceiling));
    double geometry_floor=fmin(dynamic_bank_limit(&geometry_hold,&cfg.vehicle),vertical_ceiling+10.0);
    assert(geometry_floor>vertical_ceiling+1.0);
    EntryControlPlan protected={.valid=true,.target_bank=70.0,
        .target_aoa=cfg.vehicle.entry_angle_of_attack,.bank_cap=70.0};
    assert(entry_program_shape_vertical_capture_plan(&tangent,&geometry_hold,&p,aero,&cfg,&protected));
    assert(fabs(protected.target_bank)<=vertical_ceiling+.25);
    assert(fabs(protected.target_bank)<fabs(geometry_floor));
    double protected_bank=protected.target_bank;
    entry_program_apply_geometry_bank_demand(&tangent,&geometry_hold,&p,aero,&cfg,geometry_floor,&protected);
    assert(fabs(protected.target_bank-protected_bank)<1e-6);
    geometry_hold.flight_path_angle=-2.0;geometry_hold.vertical_speed=1923.0*sin(-2.0*DEG2RAD);
    double shallow_ceiling=entry_program_vertical_bank_ceiling(&tangent,&geometry_hold,&p,aero,&cfg);
    assert(isfinite(shallow_ceiling)&&shallow_ceiling>10.0);
    double permissive_demand=fmin(20.0,shallow_ceiling-1.0);
    EntryControlPlan permissive={.valid=true,.target_bank=0.0,
        .target_aoa=cfg.vehicle.entry_angle_of_attack,.bank_cap=70.0};
    entry_program_apply_geometry_bank_demand(&tangent,&geometry_hold,&p,aero,&cfg,permissive_demand,&permissive);
    assert(permissive.target_bank>0.0);
    assert(fabs(permissive.target_bank-permissive_demand)<1e-6);

    /* High specific-energy reserve alone is not permission to spend vertical lift on
       ineffective lateral bank. v21b showed 60-70 deg requests at 37-30 km while the
       demonstrated turn radius was still hundreds to thousands of kilometres. In this
       high-speed fixture, keep at least the vertical-safe authority but clip the surplus
       request when its live radius cannot close the remaining pose debt efficiently. */
    GuidanceMachine energy_machine=tangent;
    energy_machine.entry_reversal_scheduled=false;
    energy_machine.entry_reversal_is_final=false;
    energy_machine.entry_final_reversal_pending=false;
    energy_machine.entry_final_reversal_completed=false;
    Telemetry energy_shaping=geometry_hold;
    energy_shaping.flight_path_angle=-3.1;
    energy_shaping.vertical_speed=energy_shaping.true_air_speed*sin(-3.1*DEG2RAD);
    double energy_ceiling=entry_program_vertical_bank_ceiling(&energy_machine,&energy_shaping,&p,aero,&cfg);
    assert(isfinite(energy_ceiling));
    double energy_request=fmin(dynamic_bank_limit(&energy_shaping,&cfg.vehicle),energy_ceiling+10.0);
    assert(energy_request>energy_ceiling+1.0);
    EntryControlPlan energy_plan={.valid=true,.target_bank=0.0,
        .target_aoa=cfg.vehicle.entry_angle_of_attack,.bank_cap=70.0};
    entry_program_apply_geometry_bank_demand(&energy_machine,&energy_shaping,&p,aero,&cfg,energy_request,&energy_plan);
    assert(fabs(energy_plan.target_bank)>=energy_ceiling-.25);
    assert(fabs(energy_plan.target_bank)<=energy_request+1e-6);

    /* Recorded v21b-like thin-air state: at ~37.5 km / 1.84 km/s the measured
       lift was only about 2.1 m/s2, so a near-70 deg bank had a multi-thousand-km
       radius and mostly discarded vertical lift. The same pose debt must retain the
       vertical-safe bank but reject most surplus-energy authority. */
    Telemetry ineffective_turn=energy_shaping;
    ineffective_turn.mean_altitude=ineffective_turn.radar_altitude=37490.0;
    ineffective_turn.true_air_speed=1840.4;ineffective_turn.horizontal_speed=1837.0;
    ineffective_turn.flight_path_angle=-3.3;
    ineffective_turn.vertical_speed=ineffective_turn.true_air_speed*sin(-3.3*DEG2RAD);
    ineffective_turn.dynamic_pressure=2530.0;
    ineffective_turn.lift_force=ineffective_turn.mass*2.10;
    ineffective_turn.drag_force=ineffective_turn.mass*3.80;
    ineffective_turn.ground_track_heading=ineffective_turn.heading=91.0;
    double ineffective_ceiling=entry_program_vertical_bank_ceiling(&energy_machine,&ineffective_turn,&p,aero,&cfg);
    assert(isfinite(ineffective_ceiling)&&ineffective_ceiling<50.0);
    double ineffective_request=fmin(dynamic_bank_limit(&ineffective_turn,&cfg.vehicle),70.0);
    EntryControlPlan ineffective_plan={.valid=true,.target_bank=0.0,
        .target_aoa=cfg.vehicle.entry_angle_of_attack,.bank_cap=70.0};
    entry_program_apply_geometry_bank_demand(&energy_machine,&ineffective_turn,&p,aero,&cfg,
        ineffective_request,&ineffective_plan);
    assert(fabs(ineffective_plan.target_bank)>=ineffective_ceiling-.25);
    assert(fabs(ineffective_plan.target_bank)<ineffective_request-5.0);
    /* v23 exposed the opposite thin-air failure: useful curvature is still weak near
       46 km, but staying at ~2 deg bank leaves the vehicle unshaped when q finally
       arrives. With ample energy and no FPA debt, pre-position bank while diverting
       no more than the bounded vertical-lift fraction. */
    Telemetry thin_air=energy_shaping;
    thin_air.mean_altitude=thin_air.radar_altitude=46000.0;
    thin_air.true_air_speed=2000.0;thin_air.horizontal_speed=1995.0;
    thin_air.dynamic_pressure=900.0;thin_air.lift_force=thin_air.mass*.35;
    thin_air.drag_force=thin_air.mass*1.5;thin_air.bank_effectiveness=1.0;
    thin_air.ground_track_heading=thin_air.heading=90.0;
    thin_air.runway_along_track=tangent.taem_interface_target.along_track-380000.0;
    thin_air.runway_cross_track=tangent.taem_interface_target.cross_track;
    double thin_distance=NAN;
    double thin_path=entry_program_taem_path_fpa(&energy_machine,&thin_air,&cfg,&thin_distance);
    assert(isfinite(thin_path)&&thin_distance>300000.0);
    thin_air.flight_path_angle=thin_path;
    thin_air.vertical_speed=thin_air.true_air_speed*sin(thin_path*DEG2RAD);
    double thin_ceiling=entry_program_vertical_bank_ceiling(&energy_machine,&thin_air,&p,aero,&cfg);
    assert(isfinite(thin_ceiling)&&thin_ceiling<5.0);
    EntryControlPlan thin_plan={.valid=true,.target_bank=0.0,
        .target_aoa=cfg.vehicle.entry_angle_of_attack,.bank_cap=70.0};
    entry_program_apply_geometry_bank_demand(&energy_machine,&thin_air,&p,aero,&cfg,45.0,&thin_plan);
    assert(fabs(thin_plan.target_bank)>18.0&&fabs(thin_plan.target_bank)<30.0);
    Telemetry moderate_debt=energy_shaping;
    double moderate_distance=NAN;
    double moderate_path=entry_program_taem_path_fpa(&energy_machine,&moderate_debt,&cfg,&moderate_distance);
    assert(isfinite(moderate_path));
    moderate_debt.flight_path_angle=moderate_path-1.0;
    moderate_debt.vertical_speed=moderate_debt.true_air_speed*sin(moderate_debt.flight_path_angle*DEG2RAD);
    double moderate_ceiling=entry_program_vertical_bank_ceiling(&energy_machine,&moderate_debt,&p,aero,&cfg);
    double moderate_request=fmin(dynamic_bank_limit(&moderate_debt,&cfg.vehicle),moderate_ceiling+10.0);
    EntryControlPlan moderate_plan={.valid=true,.target_bank=0.0,
        .target_aoa=cfg.vehicle.entry_angle_of_attack,.bank_cap=70.0};
    entry_program_apply_geometry_bank_demand(&energy_machine,&moderate_debt,&p,aero,&cfg,
        moderate_request,&moderate_plan);
    assert(fabs(moderate_plan.target_bank)>=moderate_ceiling-.25);
    assert(fabs(moderate_plan.target_bank)<=moderate_request+1e-6);
    assert(fabs(moderate_plan.target_bank)<moderate_request-1.0);

    /* The final reversal must create one monotonic arc into the tangent line.  The
       v25 geometry was still short of crossrange: the gate bearing was to the right
       while the scheduled final reversal was to the left.  Hold the first leg until
       continued same-side turning moves the gate onto the final-reversal side. */
    Telemetry arc=ingress;
    arc.ground_track_heading=102.0;arc.heading=102.0;
    double final_relative=norm_signed_deg(tangent.taem_interface_target.course-cfg.site.runway_heading);
    double final_side=final_relative>=0.0?1.0:-1.0;
    double ignored_endpoint_error=INFINITY,final_radius=INFINITY;
    (void)entry_program_terminal_course_endpoint_ready(&tangent,&arc,&cfg,final_side,
        tangent.taem_interface_target.course,&ignored_endpoint_error,&final_radius);
    assert(isfinite(final_radius)&&final_radius>1000.0);
    double psi0=norm_signed_deg(arc.ground_track_heading-cfg.site.runway_heading)*DEG2RAD;
    double psi1=norm_signed_deg(tangent.taem_interface_target.course-cfg.site.runway_heading)*DEG2RAD;
    double arc_ua=(sin(psi1)-sin(psi0))/final_side;
    double arc_uc=(cos(psi0)-cos(psi1))/final_side;
    double arc_unorm=hypot(arc_ua,arc_uc);
    assert(arc_unorm>1e-6);
    double ready_along=tangent.taem_interface_target.along_track-final_radius*arc_ua;
    double ready_cross=tangent.taem_interface_target.cross_track-final_radius*arc_uc;
    /* A change along the circle's radius vector can be absorbed by selecting a wider
       flyable turn radius. Create a true negative fixture by moving 30 km normal to
       that vector; no radius choice can remove this residual, so the endpoint must
       remain outside the <=15 km MM304 setup tube. */
    double bad_along=ready_along-30000.0*arc_uc/arc_unorm;
    double bad_cross=ready_cross+30000.0*arc_ua/arc_unorm;
    arc.runway_along_track=bad_along;
    arc.runway_cross_track=bad_cross;
    assert(!entry_program_final_reversal_geometry_ready(&tangent,&arc,&cfg,final_side));
    double arc_heading=entry_taem_tangent_capture_heading(&tangent,&arc,&cfg);
    double commanded_turn=norm_signed_deg(arc_heading-arc.ground_track_heading);
    assert(final_side*commanded_turn>1.0);
    arc.runway_along_track=ready_along;
    arc.runway_cross_track=ready_cross;
    assert(entry_program_final_reversal_geometry_ready(&tangent,&arc,&cfg,final_side));

    GuidanceMachine arc_event=tangent;
    arc_event.s_turn_sign=1.0;
    arc_event.entry_reversal_scheduled=true;arc_event.entry_reversal_is_final=true;
    arc_event.entry_reversal_ut=100.0;arc_event.entry_reversal_sign=final_side;
    arc_event.entry_reversal_bank=25.0;arc_event.entry_control_bank=20.0;
    arc_event.entry_s_turn_plan.valid=true;arc_event.has_s_turn_leg_started=true;
    arc_event.s_turn_leg_started_ut=100.0-cfg.guidance.s_turn_minimum_leg_duration-2.0;
    arc.ut=100.0;arc.runway_along_track=bad_along;arc.runway_cross_track=bad_cross;
    assert(!entry_program_execute_planned_reversal(&arc_event,&arc,&p,&cfg));
    assert(arc_event.s_turn_sign>0.0&&arc_event.entry_reversal_scheduled);
    arc.runway_along_track=ready_along;arc.runway_cross_track=ready_cross;
    assert(entry_program_execute_planned_reversal(&arc_event,&arc,&p,&cfg));
    assert(arc_event.s_turn_sign<0.0&&arc_event.entry_control_reversals==1);

    /* Persistent MM304 bank topology must be refreshed when the state-consistent
       longitudinal demand has materially moved, but not on sub-interval noise. */
    GuidanceMachine drift={0};
    drift.entry_s_turn_plan.valid=true;
    drift.entry_s_turn_plan.planned_ut=10.0;
    drift.entry_s_turn_plan.target_bank=60.0;
    EntryControlPlan drift_demand={.valid=true,.target_bank=20.0};
    EntryLateralLimits drift_limits=entry_lateral_default_limits();
    drift_limits.maximum_bank_deg=60.0;
    GuidanceSettings drift_settings=cfg.guidance;
    drift_settings.prediction_interval=1.5;
    assert(!entry_program_longitudinal_bank_replan_due(&drift,&drift_demand,12.0,
        &drift_settings,&drift_limits));
    assert(entry_program_longitudinal_bank_replan_due(&drift,&drift_demand,14.0,
        &drift_settings,&drift_limits));
    drift_demand.target_bank=56.0;
    assert(!entry_program_longitudinal_bank_replan_due(&drift,&drift_demand,20.0,
        &drift_settings,&drift_limits));


    /* A terminal tangent-capture event may stay on the already-correct S-turn side.
       It must enter the final capture state without manufacturing a reversal count or
       forcing a roll through the opposite bank. */
    GuidanceMachine same_side={0};
    same_side.diagnostic_shadow=true;
    same_side.s_turn_sign=1.0;
    same_side.entry_reversal_scheduled=true;
    same_side.entry_reversal_is_final=true;
    same_side.entry_reversal_ut=100.0;
    same_side.entry_reversal_sign=1.0;
    same_side.entry_reversal_bank=18.0;
    same_side.entry_control_bank=16.0;
    same_side.entry_s_turn_plan.valid=true;
    same_side.control_plan_sequence=7;same_side.entry_s_turn_plan.plan_id=7;
    same_side.entry_s_turn_plan.plan_version=1;
    same_side.has_s_turn_leg_started=true;
    same_side.s_turn_leg_started_ut=100.0-cfg.guidance.s_turn_minimum_leg_duration-1.0;
    Telemetry same_event;telemetry_init(&same_event);same_event.ut=100.0;
    assert(entry_program_execute_planned_reversal(&same_side,&same_event,&p,&cfg));
    assert(same_side.s_turn_sign>0.0);
    assert(same_side.entry_control_bank>0.0);
    assert(same_side.entry_control_reversals==0);
    assert(same_side.entry_final_reversal_pending&&!same_side.entry_final_reversal_completed);
    assert(!same_side.entry_reversal_scheduled);
    assert(same_side.entry_s_turn_plan.valid&&same_side.entry_s_turn_plan.final_heading_lock);
    assert(!same_side.entry_s_turn_plan.has_planned_reversal);
    assert(same_side.entry_s_turn_plan.plan_id==8&&same_side.entry_s_turn_plan.parent_plan_id==7);

    /* Incoming inclination does not move or relax the MM304 ownership station.
       The S-turn side selects the same fixed legal outlet; measured geometry must
       satisfy that latched contract rather than changing course to force release. */
    GuidanceMachine inclined={0};inclined.s_turn_sign=1.0;
    Telemetry inclined_ingress=ingress;
    inclined_ingress.ground_track_heading=35.0;inclined_ingress.heading=35.0;
    entry_publish_taem_tangent_target(&inclined,&inclined_ingress,35.0,&p,aero,&cfg);
    assert(inclined.taem_interface_target.valid);
    assert(entry_taem_tangent_target_geometry(&inclined.taem_interface_target,&cfg));
    double inclined_expected_speed=NAN,inclined_course=NAN;
    entry_taem_turnability_target(&inclined,&inclined_ingress,35.0,aero,&cfg,ordinary,
        &inclined_expected_speed,&inclined_course);
    assert(fabs(inclined.taem_interface_target.along_track-station)<1e-6);
    assert(fabs(inclined.taem_interface_target.cross_track)<1e-6);
    assert(fabs(norm_signed_deg(inclined.taem_interface_target.course-inclined_course))<1e-6);
    double inclined_offset=fabs(norm_signed_deg(inclined.taem_interface_target.course-cfg.site.runway_heading));
    assert(fabs(inclined_offset-90.0)<1e-6);
    assert(fabs(inclined.taem_interface_target.speed-ordinary)<1e-6);
    /* The selected gate is mission-program state.  A changing live course must not
       turn it into another moving-target problem during the S-turn. */
    TaemInterfaceTarget latched=tangent.taem_interface_target;
    ingress.ground_track_heading=50.0;
    entry_publish_taem_tangent_target(&tangent,&ingress,50.0,&p,aero,&cfg);
    assert(fabs(tangent.taem_interface_target.along_track-latched.along_track)<1e-9);
    assert(fabs(tangent.taem_interface_target.cross_track-latched.cross_track)<1e-9);
    assert(fabs(norm_signed_deg(tangent.taem_interface_target.course-latched.course))<1e-9);

    GuidanceMachine g={0};
    g.terminal_candidate.valid=true;
    g.terminal_candidate.selected_ut=90;g.terminal_candidate.arrival_ut=110;g.terminal_candidate.response=4;
    g.terminal_candidate.join=(HACTransitionPlan){.valid=true,
        .p0={-25000,0},.p1={-20000,0},.p2={-15000,0},.p3={-10000,0}};
    g.taem_interface_target=(TaemInterfaceTarget){.valid=true,.along_track=-38000,
        .cross_track=30000,.course=0,.altitude=27000,.speed=500,.flight_path_angle=-10,
        .acquisition_lead=12000,.remaining_path=36000,.response_time=8};
    g.taem_interface_target.specific_energy=rotating_specific_energy(0,27000,500,&p);
    Telemetry t;telemetry_init(&t);
    t.ut=100;t.latitude=0;t.mean_altitude=30000;t.radar_altitude=30000;
    t.true_air_speed=500;t.horizontal_speed=490;t.vertical_speed=-80;
    t.flight_path_angle=-9.2;t.runway_along_track=-38200;t.runway_cross_track=30000;
    t.mass=40231;t.lift_force=t.mass*17;t.drag_force=t.mass*3;
    t.dynamic_pressure=5000;t.g_force=1.5;t.bank_effectiveness=1;t.stall_fraction=0;
    TaemInterfaceCapture a=entry_dynamic_interface_capture(&g,&t,2,&p,aero,&cfg);
    printf("nominal capture valid=%d veto=%u dh=%.1f turn=%.1f energy=%.1f\n",a.valid,a.veto,a.altitude_error,a.turn_margin,a.energy_margin);
    assert(a.valid&&a.ready&&!g.terminal_path_committed);
    assert(!entry_taem_handoff_geometry_ready(t.mean_altitude,t.runway_along_track,
        t.vertical_speed,t.horizontal_speed,&cfg.guidance));
    cfg.guidance.taem_interface_altitude=16500;
    assert(entry_dynamic_interface_capture(&g,&t,2,&p,aero,&cfg).ready);
    cfg.guidance.taem_interface_altitude=20000;
    assert((entry_dynamic_interface_capture(&g,&t,29,&p,aero,&cfg).veto&4u)==0);
    assert(entry_dynamic_interface_capture(&g,&t,31,&p,aero,&cfg).veto&4u);
    Telemetry point_edge=t;
    point_edge.runway_along_track=g.taem_interface_target.along_track;
    point_edge.runway_cross_track=g.taem_interface_target.cross_track-999.0;
    assert((entry_dynamic_interface_capture(&g,&point_edge,2,&p,aero,&cfg).veto&2u)==0);
    point_edge.runway_cross_track=g.taem_interface_target.cross_track-1001.0;
    assert(entry_dynamic_interface_capture(&g,&point_edge,2,&p,aero,&cfg).veto&2u);
    Telemetry bad=t;bad.runway_cross_track=20000;
    assert(entry_dynamic_interface_capture(&g,&bad,2,&p,aero,&cfg).veto&2u);
    bad=t;bad.mean_altitude=27500;bad.radar_altitude=27500;bad.true_air_speed=1350;
    assert(entry_dynamic_interface_capture(&g,&bad,2,&p,aero,&cfg).veto&1u);
    bad=t;bad.true_air_speed=g.taem_interface_target.speed*.95;
    bad.horizontal_speed=bad.true_air_speed;
    assert(entry_dynamic_interface_capture(&g,&bad,2,&p,aero,&cfg).veto&1u); /* <96% target is not enough HAC reserve */
    bad=t;bad.mean_altitude=27500;bad.radar_altitude=27500;
    bad.true_air_speed=g.taem_interface_target.speed*.90;bad.horizontal_speed=bad.true_air_speed;
    TaemInterfaceCapture low_energy=entry_dynamic_interface_capture(&g,&bad,2,&p,aero,&cfg);
    assert(low_energy.veto&1u); /* normal ownership must retain TAEM kinetic reserve */
    bad=t;bad.flight_path_angle=-36;
    assert(entry_dynamic_interface_capture(&g,&bad,2,&p,aero,&cfg).veto&32u);
    bad=t;bad.flight_path_angle=g.taem_interface_target.flight_path_angle-
        (TAEM_INTERFACE_FPA_DEBT_LIMIT_DEG-.1);
    assert((entry_dynamic_interface_capture(&g,&bad,2,&p,aero,&cfg).veto&32u)==0);
    bad=t;bad.flight_path_angle=g.taem_interface_target.flight_path_angle-
        (TAEM_INTERFACE_FPA_DEBT_LIMIT_DEG+.1);
    assert(entry_dynamic_interface_capture(&g,&bad,2,&p,aero,&cfg).veto&32u);
    bad=t;bad.lift_force=t.mass*.1;
    assert(entry_dynamic_interface_capture(&g,&bad,2,&p,aero,&cfg).veto&4u);
    bad=t;bad.dynamic_pressure=NAN;
    assert(!entry_dynamic_interface_capture(&g,&bad,2,&p,aero,&cfg).valid);
    g.entry_exec.entry_complete=true;g.taem_interface_captured=true;
    t.energy_excess_range=101000;
    TaemExecInputs inputs=taem_exec_inputs_live(&g,&t,&cfg.guidance);
    assert(!inputs.energy_valid);
    TaemExecutive exec={0};
    TaemExecObservation obs={.ut=t.ut,.relative_velocity=t.true_air_speed};
    TaemExecProfile profile=taem_exec_profile_production(&cfg.guidance);
    assert(taem_exec_initialize(&exec,&obs,&inputs,&profile));
    assert(exec.phase==TAEM_PHASE_PATH_ACQUISITION&&!g.terminal_path_committed);
    /* Recorded v6 eligibility state: target construction must reserve the
       long acquisition leg rather than request 695 m/s at a 15 km lead. */
    GuidanceMachine real={0};
    cfg.site.altitude=70;cfg.guidance.hac_radius=12000;
    cfg.guidance.final_approach_distance=8000;cfg.guidance.final_glide_slope=20;
    cfg.guidance.taem_interface_range=35000;
    cfg.vehicle.entry_angle_of_attack=18;cfg.vehicle.final_approach_speed=115;
    cfg.vehicle.minimum_safe_speed=85;
    real.terminal_candidate=(TerminalCandidate){.valid=true,.kind=TERMINAL_PATH_SPLINE,
        .radius=12000,.slope=22,.final_distance=8000,.response=7,
        .join={.valid=true,.p0={-40000,0},.p1={-30000,0},.p2={-20000,0},.p3={-8000,0}}};
    Telemetry live=t;live.ut=67544.14;live.mean_altitude=27134.47;
    live.true_air_speed=1299.918;live.horizontal_speed=1294.78;live.vertical_speed=-115.55;
    live.flight_path_angle=-5.0996;live.runway_along_track=-70354.91;live.runway_cross_track=4110.38;
    live.ground_track_heading=88.15;live.heading=97.09;live.angle_of_attack=18.6914;live.mach=4.1;
    live.dynamic_pressure=8439.98;live.lift_force=live.mass*6.698803;live.drag_force=live.mass*11.94044;
    real.terminal_candidate.selected_ut=live.ut-6.0;
    real.terminal_candidate.arrival_ut=live.ut+10.0;
    terminal_publish_interface_target(&real,&live,&p,aero,&cfg);
    TaemInterfaceTarget target=real.taem_interface_target;
    TaemInterfaceCapture actual=entry_dynamic_interface_capture(&real,&live,88.15,&p,aero,&cfg);
    printf("recorded inlet: h=%.1f V=%.1f along=%.1f lead=%.1f FPA=%.1f veto=%u dh=%.1f E=%.1f turn=%.1f\n",
        target.altitude,target.speed,target.along_track,target.acquisition_lead,target.flight_path_angle,
        actual.veto,actual.altitude_error,actual.energy_margin,actual.turn_margin);
    /* v12 demonstrated that the old inlet contract could be vertically
       impossible even when position/speed capture was nominal: a ~7 deg inlet
       was paired with an ~19 deg average descent to the terminal merge, and the
       real shuttle exhausted the lateral curve while still ~17 km high.  Keep
       the measured-energy lead, but require the inlet to carry the geometric
       descent up to the established 16 deg high-speed envelope and leave no
       more than the bounded downstream response reserve. */
    double gate_height=clampd(fmax(500.0,cfg.guidance.flare_altitude*8.0),450.0,750.0);
    double final_slope=clampd(cfg.guidance.final_glide_slope,5.0,35.0)*DEG2RAD;
    double gate_ground=gate_height/fmax(tan(final_slope),1e-3);
    double final_part=fmax(0.0,real.terminal_candidate.final_distance-gate_ground);
    double merge_altitude=cfg.site.altitude+gate_height+
        fmax(0.0,real.terminal_candidate.join.arc_remaining)*
            tan(clampd(cfg.guidance.taem_glide_slope,6.0,22.0)*DEG2RAD)+
        final_part*tan(final_slope);
    double required_average_slope=atan2(target.altitude-merge_altitude,
        target.acquisition_lead)*RAD2DEG;
    printf("vertical inlet closure: average %.2f deg vs inlet %.2f deg\n",
        required_average_slope,-target.flight_path_angle);
    assert(target.valid&&target.acquisition_lead>35000&&target.altitude>24000);
    assert(required_average_slope-(-target.flight_path_angle)<=TAEM_INTERFACE_FPA_DEBT_LIMIT_DEG+1e-6);
    assert(fabs(-target.flight_path_angle-fmin(required_average_slope,16.0))<0.25);
    /* Do not pay for the vertical fix by silently consuming the whole TAEM
       speed ceiling; this recorded case had a real energy margin before v12. */
    assert(target.speed<cfg.guidance.taem_force_handoff_speed-10.0);
    /* The recorded v6/v12-like state is deliberately no longer a legal
       handoff: MM304 must deliver the deeper FPA before TAEM takes ownership. */
    assert(!actual.ready&&(actual.veto&32u)!=0);
    /* The old v12 FPA-only regression expected still more sink here.  Against the
       shared tangent tube this replay is already *below* the upstream -16 deg
       corridor, so more bank would make the position/altitude contract worse.  The
       same shallow-FPA state above the corridor must still invoke vertical capture. */
    GuidanceMachine delivery={0};delivery.taem_interface_target=target;delivery.s_turn_sign=1.0;
    Telemetry replay=live;
    replay.ut=67533.9836423648;replay.mean_altitude=27964.2551337216;
    replay.true_air_speed=1381.54284667969;replay.surface_speed=replay.true_air_speed;
    replay.horizontal_speed=1375.41469359712;replay.vertical_speed=-129.98119327655;
    replay.flight_path_angle=-5.39860550223131;replay.dynamic_pressure=8100.8193359375;
    replay.g_force=1.44336187839508;replay.range_to_site=84635.5963284118;
    replay.runway_along_track=-84507.0537937423;replay.runway_cross_track=4662.83443491395;
    replay.angle_of_attack=20.5064792633057;
    double replay_corridor=entry_program_taem_corridor_altitude(&delivery,&replay,&cfg);
    assert(isfinite(replay_corridor)&&replay.mean_altitude<replay_corridor);
    double delivery_bank=3.33421553685213,delivery_aoa=20.7470854234972;
    bool delivery_active=entry_program_altitude_capture(&delivery,&replay,&p,&cfg,
        &delivery_bank,&delivery_aoa);
    assert(!delivery_active);

    GuidanceMachine delivery_high={0};delivery_high.taem_interface_target=target;delivery_high.s_turn_sign=1.0;
    Telemetry replay_high=replay;replay_high.mean_altitude=replay_corridor+2200.0;
    double high_bank=3.33421553685213,high_aoa=20.7470854234972;
    bool high_active=entry_program_altitude_capture(&delivery_high,&replay_high,&p,&cfg,
        &high_bank,&high_aoa);
    printf("v12 inlet delivery: below active=%d h=%.0f/%.0f, above active=%d bank=%.2f aoa=%.2f FPA=%.2f latest=%.2f\n",
        delivery_active,replay.mean_altitude,replay_corridor,high_active,high_bank,high_aoa,
        replay_high.flight_path_angle,target.flight_path_angle+TAEM_INTERFACE_FPA_DEBT_LIMIT_DEG);
    assert(high_active);
    assert(high_bank>13.5);
    assert(high_aoa<20.0);
    /* A shallow TAEM-delivery state should naturally buy a much wider lateral
       S-turn before reversing.  This is debt/geometry driven, not a requested
       number of reversals. */
    double delivery_debt=entry_s_turn_fpa_delivery_debt(replay.flight_path_angle,&target);
    double join_range=entry_taem_range_target(&p,&cfg.guidance);
    double no_target_corridor=entry_s_turn_reversal_corridor(cfg.guidance.hac_look_ahead_angle,
        170000.0,join_range,1.0,0.0);
    double wide_corridor=entry_s_turn_reversal_corridor(cfg.guidance.hac_look_ahead_angle,
        170000.0,join_range,1.0,delivery_debt);
    assert(delivery_debt>.5);
    assert(no_target_corridor<cfg.guidance.hac_look_ahead_angle+8.0);
    assert(wide_corridor>cfg.guidance.hac_look_ahead_angle+15.0);

    /* MM304 tangent ownership must not depend on speculative terminal-candidate
       timing.  Any HAC-radius veto belongs to the already-published inlet/live
       state, not to a later candidate arrival timestamp. */
    real.terminal_candidate.arrival_ut=live.ut+2.0;
    TaemInterfaceCapture stale_timing=entry_dynamic_interface_capture(&real,&live,88.15,&p,aero,&cfg);
    assert((stale_timing.veto&128u)==(actual.veto&128u));
    real.terminal_candidate.arrival_ut=live.ut+10.0;
    /* Changing the forecast's future state must not change its demand. */
    real.terminal_candidate.altitude=500;real.terminal_candidate.speed=200;
    real.terminal_candidate.slope=8.0;
    terminal_publish_interface_target(&real,&live,&p,aero,&cfg);
    assert(fabs(real.taem_interface_target.along_track-target.along_track)<1e-6);
    assert(fabs(real.taem_interface_target.speed-target.speed)<1e-6);
    assert(fabs(real.taem_interface_target.altitude-target.altitude)<1e-6);
    assert(fabs(real.taem_interface_target.flight_path_angle-target.flight_path_angle)<1e-6);
    real.terminal_candidate.join.p2=(HACPoint2){10000,20000};
    real.terminal_candidate.join.p3=(HACPoint2){5000,20000};
    terminal_publish_interface_target(&real,&live,&p,aero,&cfg);
    assert(fabs(norm_signed_deg(real.taem_interface_target.course-target.course))<1e-6);
    /* A candidate selected during MM304 may publish this acquisition demand,
       but its response clock must be rebased when TAEM actually takes over. */
    real.terminal_candidate.valid=true;
    real.terminal_candidate.selected_ut=live.ut-34.2;
    real.terminal_candidate.arrival_ut=live.ut+9.8;
    real.terminal_prediction_valid=true;
    real.terminal_prediction_ut=live.ut;
    real.terminal_prediction_altitude=26770.0;
    real.terminal_prediction_speed=1287.0;
    real.terminal_prediction_time=9.8;
    real.taem_interface_target=target;
    real.terminal_energy_loss_accel_ema=14.75;
    real.terminal_speed_loss_accel_ema=12.0;
    /* Before the one-way ownership transfer, a terminal candidate is advisory.
       Its reference controls must not replace the MM304 segment that is actually
       being flown in the planning-state projection.  Otherwise a preview can
       poison its own replacement search by projecting terminal roll/FPA/AoA while
       Entry still owns the vehicle. */
    real.entry_control_plan_valid=true;
    real.entry_control_segment_until_ut=live.ut+30.0;
    real.entry_control_bank=35.0;
    GuidanceMachine no_preview=real;
    no_preview.terminal_candidate.valid=false;
    Telemetry preview_future={0},entry_future={0};
    double preview_e=0,preview_n=0,preview_course=0;
    double entry_e=0,entry_n=0,entry_course=0;
    terminal_project_planning_state(&real,&live,live.ground_track_heading,&p,aero,&cfg,12.0,
        &preview_future,&preview_e,&preview_n,&preview_course);
    terminal_project_planning_state(&no_preview,&live,live.ground_track_heading,&p,aero,&cfg,12.0,
        &entry_future,&entry_e,&entry_n,&entry_course);
    printf("MM304 advisory-preview projection: bank %.1f/%.1f FPA %.2f/%.2f AoA %.2f/%.2f\n",
        preview_future.roll,entry_future.roll,preview_future.flight_path_angle,entry_future.flight_path_angle,
        preview_future.angle_of_attack,entry_future.angle_of_attack);
    assert(fabs(preview_future.roll-entry_future.roll)<1e-9);
    assert(fabs(preview_future.flight_path_angle-entry_future.flight_path_angle)<1e-9);
    assert(fabs(preview_future.angle_of_attack-entry_future.angle_of_attack)<1e-9);

    /* Runway-local distances live on the body's reference sphere. A high-altitude
       vehicle's horizontal velocity is tangential at R+h, so surface-map travel
       over a prediction horizon must be scaled by R/(R+h). This reproduces the
       v29 45 km lookahead seam without depending on live KSP. */
    Telemetry map_rate={0};
    GeoPoint map_origin={cfg.site.latitude,cfg.site.longitude,cfg.site.altitude};
    GeoPoint map_start=local_point(map_origin,-350000.0,-1000.0,p.radius,45000.0);
    map_rate.latitude=map_start.latitude;map_rate.longitude=map_start.longitude;
    map_rate.mean_altitude=45000.0;map_rate.radar_altitude=45000.0;
    map_rate.true_air_speed=2000.0;map_rate.horizontal_speed=2000.0;
    map_rate.vertical_speed=0.0;map_rate.flight_path_angle=0.0;
    map_rate.roll=0.0;map_rate.course_rate=0.0;map_rate.bank_effectiveness=1.0;
    double map_start_e=0,map_start_n=0,map_end_e=0,map_end_n=0,map_end_course=0;
    local_offsets(map_origin,map_start,p.radius,&map_start_e,&map_start_n);
    hac_projected_local_state(&map_rate,&cfg.site,p.radius,90.0,20.0,0.0,6.0,2.0,
        0.0,1.0,0.0,&map_end_e,&map_end_n,&map_end_course);
    double expected_surface_travel=map_rate.horizontal_speed*p.radius/
        (p.radius+map_rate.mean_altitude)*20.0;
    assert(fabs((map_end_e-map_start_e)-expected_surface_travel)<1e-6);
    assert(fabs(map_end_n-map_start_n)<1e-6);
    assert(fabs(norm_signed_deg(map_end_course-90.0))<1e-9);
    real.entry_control_plan_valid=false;

    /* A predictor-classified final S-turn reversal is an executable MM304 mission
       event, not advisory metadata.  Preserve the final bit through durable commit,
       execute the opposite-side roll, then latch tangent capture only after the
       measured bank has physically crossed onto that side. */
    GuidanceMachine exit={0};
    exit.diagnostic_shadow=true;
    exit.s_turn_sign=1.0;
    exit.entry_control_bank=45.0;
    exit.has_s_turn_leg_started=true;
    Telemetry exit_t=ingress;
    exit_t.ut=200.0;
    exit.s_turn_leg_started_ut=exit_t.ut-cfg.guidance.s_turn_minimum_leg_duration-2.0;
    EntryControlPlan exit_plan={0};
    exit_plan.valid=true;
    exit_plan.target_bank=45.0;
    exit_plan.target_aoa=cfg.vehicle.entry_angle_of_attack;
    exit_plan.has_planned_reversal=true;
    exit_plan.planned_reversal_is_final=true;
    exit_plan.planned_reversal_ut=exit_t.ut;
    exit_plan.planned_reversal_range=80000.0;
    exit_plan.planned_reversal_sign=-1.0;
    entry_program_commit_planned_reversal(&exit,&exit_plan,exit_t.ut-1.0,false);
    assert(exit.entry_reversal_scheduled&&exit.entry_reversal_is_final);
    assert(exit_plan.planned_reversal_is_final);
    assert(entry_program_execute_planned_reversal(&exit,&exit_t,&p,&cfg));
    assert(exit.s_turn_sign<0.0&&exit.entry_final_reversal_pending);
    assert(!exit.entry_final_reversal_completed&&!exit.entry_reversal_scheduled);

    exit_t.roll=-25.0;
    exit_t.dynamic_pressure=5000.0;
    exit_t.true_air_speed=1000.0;
    exit_t.stall_fraction=0.0;
    exit_t.g_force=1.0;
    exit.entry_control_bank=-45.0;
    assert(entry_program_tangent_reversal_captured(&exit,&exit_t,&cfg.vehicle));
    assert(!exit.entry_final_reversal_pending&&exit.entry_final_reversal_completed);

    exit.taem_interface_target=tangent.taem_interface_target;
    exit_t.runway_along_track=exit.taem_interface_target.along_track-50000.0;
    exit_t.runway_cross_track=exit.taem_interface_target.cross_track-20000.0;
    exit_t.ground_track_heading=105.0;
    exit_t.heading=105.0;
    exit_t.horizontal_speed=990.0;
    exit_t.bank_effectiveness=1.0;
    double exit_heading=entry_taem_tangent_capture_heading(&exit,&exit_t,&cfg);
    double exit_bank=entry_taem_tangent_capture_bank(&exit,&exit_t,aero,&cfg,exit_heading);
    assert(isfinite(exit_heading)&&isfinite(exit_bank));
    assert(fabs(exit_bank)<=dynamic_bank_limit(&exit_t,&cfg.vehicle)+1e-6);
    /* v9b live handoff state: the airframe was still deeply banked when TAEM
       latched ownership, so the predictor must model the commanded roll-out
       rather than holding that bank through every future sample. */
    live.roll=55.826;live.ground_track_heading=89.6;live.flight_path_angle=-4.976;
    live.angle_of_attack=18.72;live.runway_along_track=-72937.35;live.runway_cross_track=3503.64;
    terminal_force_acquisition(&real,&live,&cfg.vehicle,&cfg.guidance,false);
    assert(real.terminal_region_entered&&real.terminal_test_capture_active);
    assert(!real.terminal_candidate.valid&&!real.terminal_prediction_valid);
    assert(isinf(real.terminal_prediction_ut)&&real.terminal_prediction_ut<0.0);
    assert(isnan(real.terminal_prediction_altitude)&&isnan(real.terminal_prediction_speed));
    assert(real.taem_interface_target.valid);
    assert(fabs(real.terminal_energy_loss_accel_ema-14.75)<1e-9);
    assert(fabs(real.terminal_speed_loss_accel_ema-12.0)<1e-9);

    assert(fabs(real.terminal_reference_bank)<1e-9);
    assert(fabs(real.terminal_reference_fpa-live.flight_path_angle)<1e-9);
    assert(fabs(real.terminal_reference_aoa-live.angle_of_attack)<1e-9);
    Telemetry rollout_future={0};double rollout_e=0,rollout_n=0,rollout_course=0;
    terminal_project_planning_state(&real,&live,live.ground_track_heading,&p,aero,&cfg,20.0,
        &rollout_future,&rollout_e,&rollout_n,&rollout_course);
    printf("handoff rollout projection: bank %.1f -> %.1f course %.1f -> %.1f\n",
        live.roll,rollout_future.roll,live.ground_track_heading,rollout_course);
    assert(fabs(rollout_future.roll)<1e-9);
    assert(fabs(norm_signed_deg(rollout_course-live.ground_track_heading))<15.0);
    puts("PASS: dynamic high-altitude capture, shell independence, state/reserve vetoes, and no repeat shell S-turn.");
    return 0;
}
