#include "../CLanding/guidance.c"
#include <assert.h>
#include <stdio.h>

static double heading_for_angle(double angle,double side){
    return norm_deg(atan2(side*sin(angle),-side*cos(angle))*RAD2DEG);
}

int main(void){
    LandingConfiguration cfg=landing_configuration_default();
    landing_configuration_normalize(&cfg);
    cfg.site.latitude=0.0;
    cfg.site.longitude=0.0;
    cfg.site.altitude=70.0;
    cfg.site.runway_heading=90.0;
    cfg.guidance.final_approach_distance=8000.0;

    /* Mission-fixed RW09 HAC: side +1 is the south-side / positive-bank
       geometry.  The runway outer-final station is the analytic-circle exit,
       and therefore the exact northern tip of the 12 km circle.  The center
       is one radius south; the southbound tangent entry is one radius east of
       that center.  Preserving side +1 means the committed 270-degree partial
       arc increases heading 180 -> 190 -> 200 ... before reaching RW09 90. */
    const double fixed_radius=12000.0;
    const double fixed_entry_e=4000.0;
    const double fixed_entry_n=-12000.0;
    HACTransitionPlan fixed={0};
    assert(hac_fixed_alignment_plan(&fixed,fixed_entry_e,fixed_entry_n,180.0,180.0,
        &cfg.site,&cfg.guidance,fixed_radius,1.0));
    assert(fixed.valid&&fixed.heading_cone&&!fixed.lead_curve);
    assert(fabs(fixed.p3.e+8000.0)<1e-9);
    assert(fabs(fixed.p3.n)<1e-9);
    assert(fabs(fixed.cone_center.e+8000.0)<1e-9);
    assert(fabs(fixed.cone_center.n+fixed_radius)<1e-9);
    assert(fabs((fixed.cone_center.n+fixed_radius)-fixed.p3.n)<1e-9);
    assert(fabs(fixed.p0.e-fixed_entry_e)<1e-9);
    assert(fabs(fixed.p0.n-fixed_entry_n)<1e-9);
    assert(fabs(hypot(fixed.p0.e-fixed.cone_center.e,
        fixed.p0.n-fixed.cone_center.n)-fixed_radius)<1e-8);
    assert(fabs(hypot(fixed.p3.e-fixed.cone_center.e,
        fixed.p3.n-fixed.cone_center.n)-fixed_radius)<1e-8);
    assert(fabs((fixed.p0.e-fixed.cone_center.e)-fixed_radius)<1e-8);
    assert(fabs(fixed.p0.e-4000.0)<1e-9);
    assert(fabs(fixed.cone_arc_length-fixed_radius*1.5*LANDER_PI)<1e-6);
    assert(fabs(fixed.length-fixed.cone_arc_length)<1e-9);
    assert(fabs(fixed.arc_remaining)<1e-12);
    assert(fabs(fixed.lead_length)<1e-12);

    HACTransitionPlan fixed_zero_lead=hac_lead_bezier_from_plan(&fixed);
    HACPoint2 zld0=hac_bezier_derivative(&fixed_zero_lead,0.0);
    HACPoint2 zld1=hac_bezier_derivative(&fixed_zero_lead,1.0);
    assert(hypot(zld0.e,zld0.n)<1e-12);
    assert(hypot(zld1.e,zld1.n)<1e-12);
    assert(fabs(norm_signed_deg(heading_for_angle(fixed.cone_start_angle,1.0)-180.0))<1e-8);
    double fixed_ten=fixed.cone_start_angle-10.0*DEG2RAD;
    assert(fabs(norm_signed_deg(heading_for_angle(fixed_ten,1.0)-190.0))<1e-8);
    assert(fabs(norm_signed_deg(heading_for_angle(fixed.cone_end_angle,1.0)-90.0))<1e-8);

    /* Variant B is simulator-only and deliberately does not change the
       default runway-station construction above.  For RW09/+1 its actual
       alignment exit is 3.5 km behind threshold, with the exact 12 km circle
       tangent entry above it. */
    setenv("KSP_LANDER_SIMULATOR","1",1);
    setenv("KSP_LANDER_HAC_VARIANT_B","1",1);
    HACTransitionPlan variant_b={0};
    assert(hac_fixed_alignment_plan(&variant_b,8500.0,-12000.0,180.0,180.0,
        &cfg.site,&cfg.guidance,fixed_radius,1.0));
    assert(variant_b.valid&&variant_b.heading_cone&&!variant_b.lead_curve);
    assert(fabs(variant_b.p3.e+3500.0)<1e-9);
    assert(fabs(variant_b.p3.n)<1e-9);
    assert(fabs(variant_b.cone_center.e+3500.0)<1e-9);
    assert(fabs(variant_b.cone_center.n+fixed_radius)<1e-9);
    assert(fabs(variant_b.p0.e-8500.0)<1e-9);
    assert(fabs(variant_b.p0.n+fixed_radius)<1e-9);
    assert(fabs(variant_b.cone_arc_length-fixed_radius*1.5*LANDER_PI)<1e-6);
    assert(fabs(norm_signed_deg(heading_for_angle(
        variant_b.cone_start_angle,1.0)-180.0))<1e-8);
    assert(fabs(norm_signed_deg(heading_for_angle(
        variant_b.cone_end_angle,1.0)-90.0))<1e-8);
    unsetenv("KSP_LANDER_HAC_VARIANT_B");

    /* The focused fixture is not at the anchored circle tangent.  Acquisition
       is therefore a finite C1 Hermite lead from the live MM305 state; the
       circle itself remains frozen at exactly 12 km. */
    HACTransitionPlan fixed_live={0};
    assert(hac_fixed_alignment_plan(&fixed_live,-8000.0,0.0,180.8,180.0,
        &cfg.site,&cfg.guidance,fixed_radius,1.0));
    assert(fixed_live.valid&&fixed_live.heading_cone&&fixed_live.lead_curve);
    assert(fabs(fixed_live.lead_start.e+8000.0)<1e-9);
    assert(fabs(fixed_live.lead_start.n)<1e-9);
    assert(fixed_live.p0.e>fixed_live.lead_start.e);
    assert(fixed_live.p0.n<fixed_live.lead_start.n);
    assert(fabs(fixed_live.p0.e-fixed.p0.e)<1e-9);
    assert(fabs(fixed_live.p0.n-fixed.p0.n)<1e-9);
    assert(fabs(fixed_live.cone_center.e-fixed.cone_center.e)<1e-9);
    assert(fabs(fixed_live.cone_center.n-fixed.cone_center.n)<1e-9);
    assert(fabs(fixed_live.cone_arc_length-fixed.cone_arc_length)<1e-9);
    double fixed_lead_chord=hypot(fixed_live.p0.e-fixed_live.lead_start.e,
        fixed_live.p0.n-fixed_live.lead_start.n);
    assert(fixed_live.lead_length>=fixed_lead_chord);
    HACTransitionPlan fixed_lead=hac_lead_bezier_from_plan(&fixed_live);
    HACPoint2 ld0=hac_bezier_derivative(&fixed_lead,0.0);
    HACPoint2 ld1=hac_bezier_derivative(&fixed_lead,1.0);
    assert(hypot(ld0.e,ld0.n)>1.0);
    assert(hypot(ld1.e,ld1.n)>1.0);
    assert(fabs(norm_signed_deg(norm_deg(atan2(ld0.e,ld0.n)*RAD2DEG)-180.8))<1e-8);
    assert(fabs(norm_signed_deg(norm_deg(atan2(ld1.e,ld1.n)*RAD2DEG)-180.0))<1e-8);

    GuidanceMachine fixed_store;
    guidance_machine_init(&fixed_store);
    hac_transition_store(&fixed_store,&fixed_live,3.0);
    assert(fabs(fixed_store.hac_transition_lead_p1_e-fixed_live.lead_p1.e)<1e-12);
    assert(fabs(fixed_store.hac_transition_lead_p1_n-fixed_live.lead_p1.n)<1e-12);
    assert(fabs(fixed_store.hac_transition_lead_p2_e-fixed_live.lead_p2.e)<1e-12);
    assert(fabs(fixed_store.hac_transition_lead_p2_n-fixed_live.lead_p2.n)<1e-12);
    assert(fabs(fixed_store.hac_transition_length-fixed_live.cone_arc_length)<1e-12);
    assert(fabs(fixed_store.hac_transition_lead_length-fixed_live.lead_length)<1e-12);
    fixed_store.fixed_alignment_hac_latched=true;
    fixed_store.hac_radius=fixed_radius;
    fixed_store.hac_side=1.0;
    fixed_store.hac_side_selected=true;
    fixed_store.hac_transition_lead_progress=.65;
    fixed_store.terminal_path_committed=true;
    Telemetry fixed_exec_telemetry;telemetry_init(&fixed_exec_telemetry);
    fixed_exec_telemetry.true_air_speed=500.0;
    fixed_exec_telemetry.mean_altitude=cfg.guidance.taem_interface_altitude;
    TaemExecInputs fixed_exec_inputs=taem_exec_inputs_live(&fixed_store,
        &fixed_exec_telemetry,&cfg.guidance);
    assert(fixed_exec_inputs.terminal_path_selected);
    assert(fixed_exec_inputs.terminal_path_captured);
    GuidanceMachine ordinary_transition=fixed_store;
    ordinary_transition.fixed_alignment_hac_latched=false;
    ordinary_transition.hac_captured=false;
    TaemExecInputs ordinary_exec_inputs=taem_exec_inputs_live(&ordinary_transition,
        &fixed_exec_telemetry,&cfg.guidance);
    assert(!ordinary_exec_inputs.terminal_path_captured);
    fixed_store.fixed_hac_reference_valid=true;
    fixed_store.fixed_hac_reference_start_altitude=20000.0;
    fixed_store.fixed_hac_reference_final_altitude=cfg.site.altitude+
        cfg.guidance.final_approach_distance*tan(
            cfg.guidance.final_glide_slope*DEG2RAD);
    fixed_store.fixed_hac_reference_slope_deg=atan2(
        fixed_store.fixed_hac_reference_start_altitude-
            fixed_store.fixed_hac_reference_final_altitude,
        fixed_live.lead_length+fixed_live.cone_arc_length)*RAD2DEG;
    Trajectory fixed_reference;trajectory_init(&fixed_reference);
    reference_trajectory_fixed_hac(&fixed_reference,&cfg.site,&cfg.guidance,600000.0,
        &fixed_store);
    assert(fixed_reference.count>72);
    GeoPoint fixed_first={fixed_reference.points[0].latitude,
        fixed_reference.points[0].longitude,fixed_reference.points[0].altitude};
    GeoPoint fixed_last={fixed_reference.points[fixed_reference.count-1].latitude,
        fixed_reference.points[fixed_reference.count-1].longitude,
        fixed_reference.points[fixed_reference.count-1].altitude};
    GeoPoint fixed_origin={cfg.site.latitude,cfg.site.longitude,cfg.site.altitude};
    double first_e=0.0,first_n=0.0,last_e=0.0,last_n=0.0;
    local_offsets(fixed_origin,fixed_first,600000.0,&first_e,&first_n);
    local_offsets(fixed_origin,fixed_last,600000.0,&last_e,&last_n);
    assert(fabs(fixed_reference.points[0].altitude-
        fixed_store.fixed_hac_reference_start_altitude)<1e-6);
    assert(fixed_reference.points[1].altitude<
        fixed_reference.points[0].altitude);
    assert(fabs(fixed_last.altitude-
        fixed_store.fixed_hac_reference_final_altitude)<1e-6);
    assert(fabs(first_e-fixed_live.lead_start.e)<1e-4);
    assert(fabs(first_n-fixed_live.lead_start.n)<1e-4);
    assert(fabs(last_e-fixed_live.p3.e)<1e-4);
    assert(fabs(last_n-fixed_live.p3.n)<1e-4);
    trajectory_clear(&fixed_reference);

    const double radius=10000.0;
    const double side=-1.0;
    HACTransitionPlan p={0};
    assert(hac_transition_plan(&p,
        -30000.0,0.0,
        -30000.0,0.0,
        180.0,180.0,&cfg.site,&cfg.guidance,
        radius,side,300.0,300.0,0.0,100.0,
        25000.0,1000.0,50000.0));
    assert(p.valid&&p.heading_cone);
    double lead_chord=hypot(p.p0.e+30000.0,p.p0.n);
    assert(p.lead_curve);
    assert(p.lead_length>=lead_chord);
    assert(p.lead_length<lead_chord*1.25);
    assert(fabs(norm_signed_deg(p.lead_start_course-180.0))<1e-12);
    assert(fabs(norm_signed_deg(p.lead_end_course-180.0))<1e-12);
    assert(fabs(hypot(p.p0.e-p.cone_center.e,p.p0.n-p.cone_center.n)-radius)<1e-6);
    assert(fabs(hypot(p.p3.e-p.cone_center.e,p.p3.n-p.cone_center.n)-radius)<1e-6);

    double runway=cfg.site.runway_heading*DEG2RAD;
    double exit_cross=p.p3.e*cos(runway)-p.p3.n*sin(runway);
    assert(fabs(exit_cross)<1e-6);
    assert(fabs(norm_signed_deg(heading_for_angle(p.cone_end_angle,side)-
        cfg.site.runway_heading))<1e-8);
    assert(fabs(p.end_angle-p.cone_end_angle)<1e-12);
    assert(fabs(p.arc_remaining)<1e-12);
    assert(hac_handoff_geometry_valid(&p,-30000.0,0.0,180.0,&cfg.site,
        &cfg.guidance,radius,side));

    /* A real projected inlet is not exactly cardinal.  Normal HAC must still
       take the shortest partial alignment arc and preserve exact runway-heading
       tangency at the exit. */
    const double offset_course=180.932424109;
    HACTransitionPlan offset={0};
    assert(hac_transition_plan(&offset,
        -30000.0,0.0,
        -30000.0,0.0,
        offset_course,offset_course,&cfg.site,&cfg.guidance,
        radius,side,300.0,300.0,0.0,100.0,
        25000.0,1000.0,50000.0));
    assert(offset.valid&&offset.heading_cone);
    assert(offset.cone_arc_length/radius<LANDER_PI);
    assert(offset.cone_arc_length/radius>LANDER_PI*.45);
    assert(fabs(norm_signed_deg(heading_for_angle(offset.cone_end_angle,side)-
        cfg.site.runway_heading))<1e-8);
    assert(hac_handoff_geometry_valid(&offset,-30000.0,0.0,offset_course,&cfg.site,
        &cfg.guidance,radius,side));

    GuidanceMachine g;
    guidance_machine_init(&g);
    g.hac_radius=radius;
    g.hac_side=side;
    g.hac_side_selected=true;
    hac_transition_store(&g,&p,3.0);

    PlanetModel planet={.radius=600000.0,
        .gravitational_parameter=3531600000000.0};
    Telemetry t;
    telemetry_init(&t);
    t.horizontal_speed=300.0;
    t.true_air_speed=300.0;
    t.mean_altitude=12000.0;

    /* If the aircraft leaves the original acquisition bridge, the fixed circle
       stays frozen but the one-shot replacement lead must begin at the live
       vector rather than at a stale projected station. */
    GuidanceMachine rebased=fixed_store;
    rebased.fixed_alignment_hac_latched=true;
    rebased.hac_radius=fixed_radius;
    rebased.hac_side=1.0;
    rebased.hac_transition_lead_rebase_attempted=false;
    assert(hac_fixed_lead_rebase_from_vector(&rebased,&t,180.0,&planet,&cfg,
        -6000.0,4000.0));
    assert(fabs(rebased.hac_transition_lead_start_e+6000.0)<1e-9);
    assert(fabs(rebased.hac_transition_lead_start_n-4000.0)<1e-9);
    assert(fabs(rebased.hac_transition_p3_e-fixed_live.p3.e)<1e-9);
    assert(fabs(rebased.hac_transition_p3_n-fixed_live.p3.n)<1e-9);
    assert(fabs(rebased.hac_transition_cone_arc_length-
        fixed_live.cone_arc_length)<1e-9);

    GeoPoint origin={cfg.site.latitude,cfg.site.longitude,cfg.site.altitude};

    /* A non-tangent fixed-HAC lead must reach the runway-anchored circle
       without overwriting its C1 tangent.  The lead is allowed to use its own
       signed curvature; side +1 applies only after the analytic circle is
       reached. */
    GeoPoint fixed_lead_start=local_point(origin,fixed_live.lead_start.e,
        fixed_live.lead_start.n,planet.radius,t.mean_altitude);
    t.latitude=fixed_lead_start.latitude;t.longitude=fixed_lead_start.longitude;
    HACGuidance fixed_lead_command=hac_path_guidance(&fixed_store,&t,&cfg.site,
        &cfg.guidance,planet.radius,1.0,9.81,180.8,fixed_radius);
    assert(isfinite(fixed_lead_command.heading));
    assert(isfinite(fixed_lead_command.lateral_acceleration));
    assert(isfinite(fixed_lead_command.bank));
    assert(fabs(norm_signed_deg(fixed_lead_command.heading-180.0))<1.0);
    assert(fixed_lead_command.arc_remaining>fixed_live.cone_arc_length);

    /* The frozen south-side (+1) circle must also keep same-direction bank
       demand when the vehicle is outside the 12 km radius. */
    fixed_store.hac_transition_lead_progress=1.0;
    double fixed_probe_angle=fixed.cone_start_angle-30.0*DEG2RAD;
    double fixed_probe_heading=heading_for_angle(fixed_probe_angle,1.0);
    GeoPoint fixed_probe=local_point(origin,
        fixed.cone_center.e+fixed_radius*cos(fixed_probe_angle),
        fixed.cone_center.n+fixed_radius*sin(fixed_probe_angle),
        planet.radius,t.mean_altitude);
    t.latitude=fixed_probe.latitude;t.longitude=fixed_probe.longitude;
    HACGuidance fixed_nominal=hac_heading_cone_path_guidance(&fixed_store,&t,
        &cfg.site,&cfg.guidance,planet.radius,9.81,fixed_probe_heading);
    GeoPoint fixed_outside_point=local_point(origin,
        fixed.cone_center.e+(fixed_radius+2000.0)*cos(fixed_probe_angle),
        fixed.cone_center.n+(fixed_radius+2000.0)*sin(fixed_probe_angle),
        planet.radius,t.mean_altitude);
    t.latitude=fixed_outside_point.latitude;t.longitude=fixed_outside_point.longitude;
    HACGuidance fixed_outward=hac_heading_cone_path_guidance(&fixed_store,&t,
        &cfg.site,&cfg.guidance,planet.radius,9.81,fixed_probe_heading);
    assert(fixed_outward.radial_error>1999.0);
    assert(fixed_outward.lateral_acceleration>0.0);
    assert(fixed_outward.lateral_acceleration>=fixed_nominal.lateral_acceleration-1e-8);
    assert(fixed_outward.lateral_acceleration>=
        t.horizontal_speed*t.horizontal_speed/fixed_radius-1e-8);

    GeoPoint at_start=local_point(origin,p.p0.e,p.p0.n,planet.radius,
        t.mean_altitude);
    t.latitude=at_start.latitude;
    t.longitude=at_start.longitude;
    HACGuidance start=hac_path_guidance(&g,&t,&cfg.site,
        &cfg.guidance,planet.radius,side,9.81,180.0,radius);
    assert(start.arc_remaining>p.cone_arc_length-1e-3);
    assert(fabs(norm_signed_deg(start.heading-180.0))<1e-6);
    assert(fabs(start.arc_remaining-p.cone_arc_length)<1e-3);
    assert(start.lateral_acceleration*side>0.0);
    g.hac_transition_lead_progress=1.0;

    double half_angle=p.cone_start_angle-
        side*(p.cone_arc_length/radius)*0.5;
    HACPoint2 half={
        p.cone_center.e+radius*cos(half_angle),
        p.cone_center.n+radius*sin(half_angle)};
    GeoPoint at_half=local_point(origin,half.e,half.n,planet.radius,
        t.mean_altitude);
    t.latitude=at_half.latitude;
    t.longitude=at_half.longitude;
    double half_heading=heading_for_angle(half_angle,side);
    HACGuidance mid=hac_path_guidance(&g,&t,&cfg.site,
        &cfg.guidance,planet.radius,side,9.81,half_heading,radius);
    assert(fabs(mid.arc_remaining/p.cone_arc_length-0.5)<1e-6);
    assert(fabs(mid.course_error)<1e-6);
    assert(fabs(mid.arc_remaining-p.cone_arc_length*0.5)<1e-3);

    /* The execution station is monotonic. A lagging course can cap future
       angular advancement, but it must not rewind a station already flown;
       otherwise the vertical scheduler sees arc_remaining grow near p3. */
    GuidanceMachine committed=g;
    committed.hac_transition_progress=.75;
    HACGuidance lagged=hac_heading_cone_path_guidance(&committed,&t,&cfg.site,
        &cfg.guidance,planet.radius,9.81,half_heading);
    assert(lagged.arc_remaining<=p.cone_arc_length*.25+1e-3);

    /* An orbiter outside a clockwise circle is LEFT of its tangent, not
       right. The Frenet law subtracts signed cross-track error; both turn
       directions must therefore increase same-direction demand outside. */
    g.fixed_alignment_hac_latched=true;
    HACGuidance nominal=hac_heading_cone_path_guidance(&g,&t,&cfg.site,
        &cfg.guidance,planet.radius,9.81,half_heading);
    GeoPoint outside=local_point(origin,
        p.cone_center.e+(radius+2000.0)*cos(half_angle),
        p.cone_center.n+(radius+2000.0)*sin(half_angle),planet.radius,t.mean_altitude);
    t.latitude=outside.latitude;t.longitude=outside.longitude;
    HACGuidance outward=hac_heading_cone_path_guidance(&g,&t,&cfg.site,
        &cfg.guidance,planet.radius,9.81,half_heading);
    assert(outward.radial_error>1999.0);
    assert(outward.lateral_acceleration*side>=nominal.lateral_acceleration*side-1e-8);
    assert(outward.lateral_acceleration*side>=t.horizontal_speed*t.horizontal_speed/radius);
    HACGuidance adverse_course=hac_heading_cone_path_guidance(&g,&t,&cfg.site,
        &cfg.guidance,planet.radius,9.81,norm_deg(half_heading+side*60.0));
    assert(adverse_course.radial_error>1999.0);
    assert(adverse_course.lateral_acceleration*side>=
        t.horizontal_speed*t.horizontal_speed/radius-1e-8);
    g.fixed_alignment_hac_latched=false;

    GeoPoint at_end=local_point(origin,p.p3.e,p.p3.n,planet.radius,
        t.mean_altitude);
    t.latitude=at_end.latitude;
    t.longitude=at_end.longitude;
    HACGuidance end=hac_path_guidance(&g,&t,&cfg.site,
        &cfg.guidance,planet.radius,side,9.81,cfg.site.runway_heading,radius);
    assert(end.arc_remaining<1e-3);
    assert(fabs(norm_signed_deg(end.heading-cfg.site.runway_heading))<1e-6);
    assert(end.arc_remaining<1e-3);
    assert(fabs(end.radial_error)<1e-3);

    /* Being in the endpoint half-plane is not completion.  An off-circle
       point at the analytic exit angle must retain a live arc reference until
       radial/endpoint/course validity is simultaneously satisfied. */
    g.fixed_alignment_hac_latched=true;
    g.hac_transition_progress=0.0;
    double exit_dx=p.p3.e-p.cone_center.e;
    double exit_dy=p.p3.n-p.cone_center.n;
    double exit_norm=hypot(exit_dx,exit_dy);
    GeoPoint off_endpoint=local_point(origin,
        p.p3.e+exit_dx/exit_norm*2000.0,
        p.p3.n+exit_dy/exit_norm*2000.0,
        planet.radius,t.mean_altitude);
    t.latitude=off_endpoint.latitude;t.longitude=off_endpoint.longitude;
    HACGuidance off_end=hac_heading_cone_path_guidance(&g,&t,&cfg.site,
        &cfg.guidance,planet.radius,9.81,cfg.site.runway_heading);
    assert(off_end.radial_error>1999.0);
    assert(off_end.arc_remaining>1.0);
    g.fixed_alignment_hac_latched=false;


    /* Nearest-point progress alone must never hand a fixed HAC lead to the
       analytic circle.  The entry capture set requires circle radial, endpoint,
       and tangent/course validity simultaneously. */
    GuidanceMachine lead_probe=fixed_store;
    lead_probe.hac_transition_lead_progress=.65;
    HACPoint2 lead_end_dir=hac_bezier_derivative(&fixed_lead,1.0);
    double lead_end_norm=hypot(lead_end_dir.e,lead_end_dir.n);
    assert(lead_end_norm>1.0);
    double lead_end_course=norm_deg(atan2(lead_end_dir.e,lead_end_dir.n)*RAD2DEG);
    double far_e=fixed_live.p0.e+lead_end_dir.e/lead_end_norm*5000.0;
    double far_n=fixed_live.p0.n+lead_end_dir.n/lead_end_norm*5000.0;
    assert(!fixed_hac_lead_capture_update(&lead_probe,&t,lead_end_course,
        fixed_radius,far_e,far_n,&fixed_lead));
    assert(lead_probe.hac_transition_lead_progress<.995);

    GuidanceMachine wrong_course_probe=fixed_store;
    wrong_course_probe.hac_transition_lead_progress=.65;
    assert(!fixed_hac_lead_capture_update(&wrong_course_probe,&t,
        norm_deg(lead_end_course+45.0),fixed_radius,
        fixed_live.p0.e,fixed_live.p0.n,&fixed_lead));
    assert(wrong_course_probe.hac_transition_lead_progress<.995);

    GuidanceMachine valid_capture_probe=fixed_store;
    valid_capture_probe.hac_transition_lead_progress=.65;
    assert(fixed_hac_lead_capture_update(&valid_capture_probe,&t,lead_end_course,
        fixed_radius,fixed_live.p0.e,fixed_live.p0.n,&fixed_lead));
    assert(fabs(valid_capture_probe.hac_transition_lead_progress-1.0)<1e-12);
    GuidanceMachine planner;
    guidance_machine_init(&planner);
    terminal_glide_initialize(&planner,&cfg.vehicle,&cfg.guidance);
    Telemetry live;
    telemetry_init(&live);
    live.ut=66564.289814;
    live.mean_altitude=19998.442;
    live.radar_altitude=19928.442;
    live.mass=40252.917969;
    live.true_air_speed=725.767;
    live.horizontal_speed=721.579;
    live.vertical_speed=-77.855;
    live.flight_path_angle=-6.158087;
    live.bank_effectiveness=1.0;
    live.lift_force=443563.385621;
    live.drag_force=626650.101017;
    live.angle_of_attack=18.0;
    live.dynamic_pressure=10517.949802;
    live.mach=2.433371;
    live.runway_along_track=-cfg.guidance.final_approach_distance;
    live.runway_cross_track=0.0;
    live.range_to_site=cfg.guidance.final_approach_distance;
    GeoPoint live_point=local_point(origin,
        -cfg.guidance.final_approach_distance,0.0,planet.radius,
        live.mean_altitude);
    live.latitude=live_point.latitude;
    live.longitude=live_point.longitude;
    live.ground_track_heading=offset_course;
    live.heading=offset_course;
    AerodynamicModel aero={.lift_to_drag=.4,
        .ballistic_coefficient=700.0,.confidence=1.0};
    double required_lateral=0.0,available_lateral=0.0,minimum_radius=0.0;
    assert(!terminal_fixed_hac_authority(&live,&planet,aero,&cfg.vehicle,
        &cfg.guidance,fixed_radius,&required_lateral,&available_lateral,
        &minimum_radius));
    assert(required_lateral>available_lateral);
    assert(minimum_radius>fixed_radius);
    assert(terminal_test_start_spiral(&planner,&live,offset_course,&planet,aero,&cfg));
    assert(planner.hac_transition_heading_cone);
    assert(planner.hac_transition_active);
    assert(planner.hac_transition_cone_arc_length>0.0);
    assert(planner.terminal_test_final_approach_distance>0.0);
    assert(fabs(planner.hac_remaining)<1e-6);

    puts("HAC finite heading-cone geometry/tracking/planning tests passed.");
    return 0;
}
