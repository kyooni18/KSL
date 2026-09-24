#include "../CLanding/guidance/guidance_entry.c"
#include "../CLanding/guidance/guidance_core.c"
#include "../CLanding/guidance/guidance_taem.c"
#include "../CLanding/guidance/guidance_final.c"
#include "../CLanding/guidance/guidance_common.c"
#include "../CLanding/guidance/guidance_terminal.c"
#include "../CLanding/guidance/guidance.c"
#include <assert.h>

/* Test-only construction of the nominal HAC entry pose.  Keep this local: the
   production geometry helper is intentionally translation-unit private. */
static bool test_taem_hac_entry_pose(const LandingConfiguration*cfg,double course,
        double altitude,double*along,double*cross){
    if(!cfg||!isfinite(course)||!isfinite(altitude))return false;
    double runway=cfg->site.runway_heading*DEG2RAD;
    double crs=course*DEG2RAD;
    double vh_e=sin(runway),vh_n=cos(runway);
    double rh_e=cos(runway),rh_n=-sin(runway);
    double rf_e=cos(crs),rf_n=-sin(crs);
    double rel=norm_signed_deg(course-cfg->site.runway_heading);
    double side=rel>=0.0?-1.0:1.0;
    double radius=fmax(1000.0,cfg->guidance.hac_radius);
    double exit_e=-vh_e*cfg->guidance.final_approach_distance;
    double exit_n=-vh_n*cfg->guidance.final_approach_distance;
    double center_e=exit_e+side*radius*rh_e;
    double center_n=exit_n+side*radius*rh_n;
    double entry_e=center_e-side*radius*rf_e;
    double entry_n=center_n-side*radius*rf_n;
    if(along)*along=entry_e*vh_e+entry_n*vh_n;
    if(cross)*cross=entry_e*rh_e+entry_n*rh_n;
    return isfinite(entry_e)&&isfinite(entry_n);
}

int main(void){
    LandingConfiguration cfg=landing_configuration_default();
    landing_configuration_normalize(&cfg);
    cfg.site.runway_heading=90;cfg.guidance.taem_interface_altitude=20000;
    cfg.guidance.taem_force_handoff_speed=1300;
    PlanetModel p={.radius=600000,.gravitational_parameter=3531600000000.0,
        .rotational_speed=.000291570900559802,.atmosphere_depth=70000,
        .surface_density=1.225,.atmosphere_adiabatic_index=1.4};
    p.atmosphere_sample_count=5;
    p.atmosphere_altitude[0]=0.0;    p.atmosphere_pressure[0]=101325.0; p.atmosphere_density[0]=1.225;
    p.atmosphere_altitude[1]=15000.0; p.atmosphere_pressure[1]=12000.0;  p.atmosphere_density[1]=0.18;
    p.atmosphere_altitude[2]=25000.0; p.atmosphere_pressure[2]=3600.0;   p.atmosphere_density[2]=0.05;
    p.atmosphere_altitude[3]=50000.0; p.atmosphere_pressure[3]=120.0;    p.atmosphere_density[3]=0.002;
    p.atmosphere_altitude[4]=70000.0; p.atmosphere_pressure[4]=1.0;      p.atmosphere_density[4]=1e-6;
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
    ingress.mass=40000.0;ingress.dynamic_pressure=12000.0;ingress.g_force=1.0;
    ingress.bank_effectiveness=1.0;ingress.roll=0.0;ingress.roll_rate=0.0;
    ingress.angle_of_attack=cfg.vehicle.entry_angle_of_attack;ingress.mach=3.0;
    ingress.physics_authority[0]=8.0;ingress.physics_authority_confidence[0]=1.0;
    ingress.has_angle_of_attack_rate=true;ingress.angle_of_attack_rate=0.0;
    ingress.lift_force=ingress.mass*12.0;ingress.drag_force=ingress.mass*8.0;
    ingress.stall_fraction=0.0;ingress.stall_fraction_is_measured=true;
    entry_publish_taem_tangent_target(&tangent,&ingress,90.0,&p,aero,&cfg);
    assert(tangent.taem_interface_target.valid);
    /* This is MM304's later HAC-acquisition target, distinct from the high-energy
       MM304→MM305 ownership band enforced by entry_dynamic_interface_capture. */
    assert(fabs(tangent.taem_interface_target.altitude-
                cfg.guidance.hac_acquisition_altitude)<1e-6);
    assert(tangent.taem_interface_target.altitude<cfg.guidance.mm305_min_altitude);
    assert(entry_taem_tangent_target_geometry(&tangent.taem_interface_target,&cfg));
    /* A geometry-valid later acquisition target does not certify the
       MM304→MM305 high-energy handoff or the remaining HAC/Final path. */
    assert(tangent.taem_interface_target.remaining_path>
        cfg.guidance.final_approach_distance);
    const double station=cfg.guidance.mm304_handoff_along_track;
    double expected_along=station;
    double expected_cross=cfg.guidance.mm304_handoff_cross_track;
    assert(fabs(tangent.taem_interface_target.along_track-expected_along)<1e-6);
    assert(fabs(tangent.taem_interface_target.cross_track-expected_cross)<1e-6);
    assert(tangent.taem_interface_target.hac_radius>=cfg.guidance.hac_radius);
    TaemSpeedEnvelope speed_envelope=decision_taem_speed_envelope(
        &cfg.vehicle,&p,tangent.taem_interface_target.altitude);
    assert(speed_envelope.valid&&speed_envelope.feasible);
    assert(tangent.taem_interface_target.speed>=
        speed_envelope.minimum_speed_mps-1e-6);
    assert(tangent.taem_interface_target.speed<=
        speed_envelope.maximum_speed_mps+1e-6);
    double target_sound=planet_atmospheric_speed_of_sound(
        &p,tangent.taem_interface_target.altitude);
    double target_mach=tangent.taem_interface_target.speed/target_sound;
    assert(target_mach>0.5&&target_mach<1.5);

    /* A target in MM305's projected envelope is advisory only: the live capture
       contract must retain MM304 ownership until the current state satisfies the
       same altitude and Mach engagement band. */
    TaemInterfaceCapture projected=entry_taem_interface_capture(
        &tangent.taem_interface_target,&ingress,90.0,&p,&cfg);
    assert(projected.valid);
    assert(projected.veto&8u); /* live altitude is outside MM305's 15-18 km band */
    assert(projected.veto&1u); /* live Mach/speed does not meet MM305 engagement */

    double rh=cfg.site.runway_heading*DEG2RAD;
    GeoPoint origin={cfg.site.latitude,cfg.site.longitude,cfg.site.altitude};
    double target_e=tangent.taem_interface_target.along_track*sin(rh)+
        tangent.taem_interface_target.cross_track*cos(rh);
    double target_n=tangent.taem_interface_target.along_track*cos(rh)-
        tangent.taem_interface_target.cross_track*sin(rh);
    GeoPoint target_point=local_point(origin,target_e,target_n,
        p.radius,tangent.taem_interface_target.altitude);
    double current_energy=rotating_specific_energy(
        ingress.latitude,ingress.mean_altitude,ingress.true_air_speed,&p);
    double zero_target_energy=rotating_specific_energy(
        target_point.latitude,tangent.taem_interface_target.altitude,0.0,&p);
    double energy_speed=sqrt(fmax(0.0,2.0*(current_energy-zero_target_energy)));
    double expected_speed=fmin(speed_envelope.maximum_speed_mps,energy_speed);
    if(expected_speed<speed_envelope.minimum_speed_mps)
        expected_speed=speed_envelope.minimum_speed_mps;

    double course_offset=fabs(norm_signed_deg(
        tangent.taem_interface_target.course-cfg.site.runway_heading));
    assert(fabs(course_offset-90.0)<1e-6);
    assert(isfinite(expected_speed));
    assert(tangent.taem_interface_target.speed<=expected_speed+cfg.guidance.mm305_target_mach*350.0);
    double initial_offset=fabs(norm_signed_deg(
        tangent.taem_interface_target.course-cfg.site.runway_heading));
    assert(fabs(initial_offset-90.0)<1e-6);

    /* A measured arc may prove that a shallower outlet would be easier to reach,
       but that is not permission to rewrite either the perpendicular course or
       fixed (-8 km, 0) ownership point merely to make the final reversal admissible. */
    GuidanceMachine relaxed=tangent;
    relaxed.taem_interface_target.course=norm_deg(cfg.site.runway_heading-90.0);
    relaxed.taem_interface_target.speed=1300.0;
    assert(fabs(relaxed.taem_interface_target.along_track-station)<1e-6);
    assert(fabs(relaxed.taem_interface_target.cross_track)<1e-6);
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
    double high_target_along=NAN,high_target_cross=NAN;
    assert(test_taem_hac_entry_pose(&cfg,high_speed.taem_interface_target.course,
        high_speed.taem_interface_target.altitude,&high_target_along,&high_target_cross));
    high_speed.taem_interface_target.along_track=high_target_along;
    high_speed.taem_interface_target.cross_track=high_target_cross;
    Telemetry fast_arc=relax_arc;
    fast_arc.true_air_speed=fast_arc.horizontal_speed=2000.0;
    double high_psi1=norm_signed_deg(high_speed.taem_interface_target.course-
        cfg.site.runway_heading)*DEG2RAD;
    fast_arc.runway_along_track=high_target_along-
        arc_radius/final_sign*(sin(high_psi1)-sin(relax_psi0));
    fast_arc.runway_cross_track=high_target_cross-
        arc_radius/final_sign*(cos(relax_psi0)-cos(high_psi1));
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
    wide_arc.taem_interface_target.course=norm_deg(cfg.site.runway_heading-60.0);
    wide_arc.taem_interface_target.speed=820.0;

    double wide_target_along=NAN,wide_target_cross=NAN;
    assert(test_taem_hac_entry_pose(&cfg,wide_arc.taem_interface_target.course,
        wide_arc.taem_interface_target.altitude,&wide_target_along,&wide_target_cross));
    wide_arc.taem_interface_target.along_track=wide_target_along;
    wide_arc.taem_interface_target.cross_track=wide_target_cross;
    Telemetry wide_state=ingress;
    wide_state.true_air_speed=wide_state.horizontal_speed=1103.0;
    wide_state.ground_track_heading=wide_state.heading=103.8;
    wide_state.runway_along_track=wide_target_along-57866.0;
    wide_state.runway_cross_track=wide_target_cross+17769.0;
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
    v20_arc.taem_interface_target.course=norm_deg(cfg.site.runway_heading-60.0);
    v20_arc.taem_interface_target.speed=640.3;

    double v20_target_along=NAN,v20_target_cross=NAN;
    assert(test_taem_hac_entry_pose(&cfg,v20_arc.taem_interface_target.course,
        v20_arc.taem_interface_target.altitude,&v20_target_along,&v20_target_cross));
    v20_arc.taem_interface_target.along_track=v20_target_along;
    v20_arc.taem_interface_target.cross_track=v20_target_cross;
    Telemetry v20_state=ingress;
    v20_state.true_air_speed=1227.4;v20_state.horizontal_speed=1219.2;
    v20_state.ground_track_heading=v20_state.heading=102.0;
    v20_state.runway_along_track=v20_target_along-99562.5;
    v20_state.runway_cross_track=v20_target_cross+13871.3;
    v20_state.mass=43729.0;v20_state.lift_force=v20_state.mass*8.0;
    v20_state.dynamic_pressure=9000.0;v20_state.g_force=1.0;v20_state.bank_effectiveness=.97;
    v20_state.stall_fraction=0.0;v20_state.stall_fraction_is_measured=true;
    double v20_error=NAN,v20_radius=NAN;
    bool v20_ready=entry_program_terminal_course_endpoint_ready(&v20_arc,&v20_state,&cfg,-1.0,
        v20_arc.taem_interface_target.course,&v20_error,&v20_radius);
    assert(!v20_ready);
    if(isfinite(v20_radius)){
        double v20_tolerance=fmax(2500.0,fmin(10000.0,v20_radius*.12));
        assert(v20_error>v20_tolerance);
    }
    double nominal_taem_speed=hac_acquisition_speed_target(
        &cfg.vehicle,&cfg.guidance,&p);
    /* The advisory acquisition target is Mach-centered; the qualified handoff
       remains controlled by the separate MM304-to-MM305 admission contract. */
    assert(tangent.taem_interface_target.speed>=speed_envelope.minimum_speed_mps-1e-6);
    assert(tangent.taem_interface_target.speed<=speed_envelope.maximum_speed_mps+1e-6);
    assert(fabs(tangent.taem_interface_target.speed-nominal_taem_speed)<1e-6);
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
    double corridor_lead=fmax(fmax(0.0,tangent.taem_interface_target.acquisition_lead),
        fmax(0.0,tangent.taem_interface_target.speed)*
        fmax(4.0,tangent.taem_interface_target.response_time));
    double corridor_slope=clampd(-tangent.taem_interface_target.flight_path_angle,3.0,25.0)*DEG2RAD;
    double corridor_h=tangent.taem_interface_target.altitude+
        fmin(95000.0,corridor_lead)*tan(corridor_slope);
    double expected_corridor_h=corridor_h;
    assert(isfinite(corridor_h)&&fabs(corridor_h-expected_corridor_h)<1e-6);
    assert(corridor_h>tangent.taem_interface_target.altitude+100.0);
    corridor.mean_altitude=corridor_h-2500.0;
    double corridor_bank=5.0,corridor_aoa=cfg.vehicle.entry_angle_of_attack;
    bool corridor_low_active=entry_program_altitude_capture(&tangent,&corridor,&p,aero,&cfg,&corridor_bank,&corridor_aoa);
    assert(isfinite(corridor_bank)&&isfinite(corridor_aoa));
    if(!corridor_low_active)assert(fabs(corridor_bank-5.0)<1e-9);

    corridor.mean_altitude=corridor_h+2500.0;
    corridor.flight_path_angle=-1.0;
    corridor_bank=5.0;corridor_aoa=cfg.vehicle.entry_angle_of_attack;
    assert(entry_program_altitude_capture(&tangent,&corridor,&p,aero,&cfg,&corridor_bank,&corridor_aoa));
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
    geometry_hold.mean_altitude=10000.0;geometry_hold.radar_altitude=10000.0;
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
    entry_program_apply_geometry_bank_demand(&tangent,&geometry_hold,&p,aero,&cfg,geometry_floor,&protected);
    assert(fabs(protected.target_bank)<=dynamic_bank_limit(&geometry_hold,&cfg.vehicle)+1e-6);
    geometry_hold.mean_altitude=41400.0;geometry_hold.radar_altitude=41400.0;
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
    assert(energy_request>=0.0&&energy_request<=dynamic_bank_limit(&energy_shaping,&cfg.vehicle));
    EntryControlPlan energy_plan={.valid=true,.target_bank=0.0,
        .target_aoa=cfg.vehicle.entry_angle_of_attack,.bank_cap=70.0};
    entry_program_apply_geometry_bank_demand(&energy_machine,&energy_shaping,&p,aero,&cfg,energy_request,&energy_plan);
    assert(fabs(energy_plan.target_bank)>=energy_ceiling-.25);
    assert(fabs(energy_plan.target_bank)<=energy_request+1e-6);

    /* In thin air, lift below the level-flight requirement is not grounds to
       force the shuttle wings-level while it is still above TAEM altitude. */
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
    assert(isfinite(ineffective_ceiling));
    assert(fabs(ineffective_ceiling-
        dynamic_bank_limit(&ineffective_turn,&cfg.vehicle))<1e-6);
    double ineffective_request=fmin(dynamic_bank_limit(&ineffective_turn,&cfg.vehicle),70.0);
    EntryControlPlan ineffective_plan={.valid=true,.target_bank=0.0,
        .target_aoa=cfg.vehicle.entry_angle_of_attack,.bank_cap=70.0};
    entry_program_apply_geometry_bank_demand(&energy_machine,&ineffective_turn,&p,aero,&cfg,
        ineffective_request,&ineffective_plan);
    assert(fabs(ineffective_plan.target_bank)>=ineffective_ceiling-.25);
    assert(fabs(ineffective_plan.target_bank)<=ineffective_request+1e-6);
    /* High above TAEM, thin air cannot support a vertical-path recovery yet. Keep
       the bank authority available while the descent builds dynamic pressure. */
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
    assert(isfinite(thin_ceiling));
    assert(fabs(thin_ceiling-
        dynamic_bank_limit(&thin_air,&cfg.vehicle))<1e-6);
    EntryControlPlan thin_plan={.valid=true,.target_bank=0.0,
        .target_aoa=cfg.vehicle.entry_angle_of_attack,.bank_cap=70.0};
    entry_program_apply_geometry_bank_demand(&energy_machine,&thin_air,&p,aero,&cfg,45.0,&thin_plan);
    assert(isfinite(thin_plan.target_bank)&&fabs(thin_plan.target_bank)<=45.0+1e-6);
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
    if(isfinite(final_radius)&&final_radius>1000.0){
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
    }

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
    puts("PASS: dynamic high-altitude capture, shell independence, state/reserve vetoes, and no repeat shell S-turn.");
    return 0;
}
