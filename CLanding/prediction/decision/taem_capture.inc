TaemInterfaceCapture entry_taem_interface_capture(const TaemInterfaceTarget*q,
        const Telemetry*t,double course,const PlanetModel*p,const LandingConfiguration*cfg){
    TaemInterfaceCapture out={0};
    if(!q||!t||!p||!cfg)return out;
    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;
    if(!q->valid||!isfinite(q->along_track)||!isfinite(q->cross_track)||!isfinite(q->course)||
       !isfinite(q->altitude)||!isfinite(q->speed)||q->speed<=0||
       !isfinite(q->flight_path_angle)||!isfinite(q->specific_energy)||
       !isfinite(q->acquisition_lead)||q->acquisition_lead<=0||
       !isfinite(q->remaining_path)||q->remaining_path<q->acquisition_lead||
       !isfinite(q->response_time)||q->response_time<=0||
       !isfinite(t->latitude)||!isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track)||
       !isfinite(t->true_air_speed)||!isfinite(t->mean_altitude)||
       !isfinite(t->flight_path_angle)||!isfinite(course)||
       !isfinite(t->horizontal_speed)||!isfinite(t->vertical_speed)||
       !isfinite(t->dynamic_pressure)||!isfinite(t->g_force)||
       (t->stall_fraction_is_measured&&!isfinite(t->stall_fraction)))return out;
    double h=norm_signed_deg(q->course-cfg->site.runway_heading)*DEG2RAD;
    double da=q->along_track-t->runway_along_track,dc=q->cross_track-t->runway_cross_track;
    out.along=da*cos(h)+dc*sin(h);out.cross=-da*sin(h)+dc*cos(h);
    out.course_error=norm_signed_deg(q->course-course);

    /* The handoff evaluator uses the same bounded-curvature geometry as the
       rest of the decision layer.  No minimum speed/time or pre-gate distance
       is manufactured here. */
    double horizontal=t->horizontal_speed;
    double response=q->response_time;
    ControlAuthorityEnvelope control=
        decision_control_authority_envelope(NULL,t,p,cfg);
    if(!control.valid||!control.controllable)return out;
    double minimum_turn_radius=INFINITY;
    double path_length=decision_target_path_length(NULL,t,course,
        q->along_track,q->cross_track,q->course,NAN,p,cfg,
        &minimum_turn_radius);

    double slope=-q->flight_path_angle*DEG2RAD;
    bool slope_valid=q->flight_path_angle<0.0&&q->flight_path_angle>-90.0&&
        isfinite(slope)&&cos(slope)>DBL_EPSILON;
    double corridor_altitude=slope_valid?
        q->altitude+out.along*tan(slope):NAN;
    out.altitude_error=isfinite(corridor_altitude)?
        t->mean_altitude-corridor_altitude:NAN;

    /* Energy ownership is evaluated against the downstream unpowered
       alignment/final maneuver below.  q->speed remains a planning prediction,
       not a nominal speed that the live vehicle must chase. */
    out.energy_margin=NAN;

    /* The MM304 boundary is a capture set, not an exact nonholonomic
       waypoint.  Asking the exact-point Dubins solver to end at the same
       coordinates with a sub-degree residual heading error produces a full
       2*pi turn, even though the mission contract already accepts the vehicle
       anywhere inside the 1 km tube and within the perpendicular heading
       sector.  Charge the bounded distance to that explicit set instead. */
    HandoffSetEnvelope handoff_set=decision_mm304_handoff_set_envelope(
        NULL,t,course,q->along_track,q->cross_track,p,cfg);
    double capture_path=handoff_set.valid?handoff_set.path_to_set_m:path_length;
    double required_time=isfinite(capture_path)&&horizontal>DBL_MIN?
        response+capture_path/horizontal:INFINITY;
    double available_time=isfinite(q->arrival_ut)?
        q->arrival_ut-t->ut:NAN;
    out.path_length_m=capture_path;
    out.required_time_s=required_time;
    out.available_time_s=available_time;
    out.minimum_turn_radius_m=minimum_turn_radius;
    out.turn_margin=isfinite(required_time)&&isfinite(available_time)?
        (available_time-required_time)*horizontal:NAN;

    out.valid=isfinite(out.along)&&isfinite(out.cross)&&
        isfinite(out.turn_margin)&&isfinite(minimum_turn_radius)&&slope_valid;
    if(!out.valid)return out;

    /*
     * Mission ownership contract:
     * - horizontal distance <= 1 km at the fixed rear-alignment point;
     * - ground track within 30 deg of either runway-perpendicular direction;
     * - positive usable mechanical-energy reserve;
     * - live lift/turn/height state can execute the remaining HAC/alignment.
     *
     * The first two numbers are explicit user mission requirements, not tuned
     * controller thresholds. Everything else is computed from live physics.
     */
    TaemHandoffContract handoff=taem_handoff_contract(s);
    double position_error=hypot(out.along,out.cross);
    bool inside_spatial=isfinite(position_error)&&
        position_error<=handoff.horizontal_radius_m;
    if(!inside_spatial)out.veto|=2u;

    double runway_offset=fabs(norm_signed_deg(course-cfg->site.runway_heading));
    bool heading_contract_ready=fabs(runway_offset-90.0)<=
        handoff.perpendicular_heading_half_width_deg;
    if(!heading_contract_ready)out.veto|=4u;

    if(t->true_air_speed<v->minimum_safe_speed)out.veto|=1u;
    if(!(t->vertical_speed<0.0))out.veto|=32u;

    /*
     * MM305 is a deliberate TAEM engagement state, not merely "some state that
     * can still turn." MM304/S-turn therefore retains ownership until the
     * shuttle is inside the configured altitude band and Mach-centered engagement
     * window. Speed of sound is recomputed from the live atmosphere if telemetry
     * Mach is unavailable.
     */
    if(t->mean_altitude<s->mm305_min_altitude||
       t->mean_altitude>s->mm305_max_altitude)
        out.veto|=8u;
    double sound=planet_atmospheric_speed_of_sound(p,t->mean_altitude);
    double live_mach=isfinite(t->mach)&&t->mach>0.0?t->mach:
        (isfinite(sound)&&sound>DBL_MIN?t->true_air_speed/sound:NAN);
    double mach_min=s->mm305_target_mach-s->mm305_mach_half_width;
    double mach_max=s->mm305_target_mach+s->mm305_mach_half_width;
    if(!isfinite(live_mach)||live_mach<mach_min||live_mach>mach_max)
        out.veto|=1u;

    /* Reuse the same measured turn authority that generated the bounded-curvature
       path above; handoff geometry and downstream maneuver proof must not carry
       separate turn-radius models. */
    double terminal_radius=fmax(s->hac_radius,minimum_turn_radius);

    double maneuver_path=0.0,maneuver_slope=0.0,final_altitude=0.0;
    bool maneuver_geometry=taem_alignment_maneuver_geometry(
        t->mean_altitude,terminal_radius,minimum_turn_radius,
        fabs(norm_signed_deg(q->course-cfg->site.runway_heading)),
        &cfg->site,s,&maneuver_path,&maneuver_slope,&final_altitude)&&
        t->true_air_speed>=v->minimum_safe_speed;

    double final_speed=fmax(v->final_approach_speed,v->minimum_safe_speed);
    double available_energy=entry_remaining_specific_energy(
        t->latitude,t->mean_altitude,t->true_air_speed,
        cfg->site.latitude,final_altitude,final_speed,p);
    UnpoweredPathProjection maneuver_projection={0};
    if(maneuver_geometry)
        maneuver_projection=decision_unpowered_path_projection(
            NULL,t,p,cfg,final_altitude,maneuver_path);
    EnergyPathEnvelope maneuver_energy={0};
    if(maneuver_projection.valid)
        maneuver_energy=decision_energy_path_envelope(t,p,
            available_energy,maneuver_projection.modeled_drag_work,
            maneuver_projection.air_path_m);
    out.energy_margin=maneuver_energy.valid?maneuver_energy.energy.margin:0.0;
    out.energy_available=maneuver_energy.valid?
        maneuver_energy.available_specific_energy:0.0;
    out.energy_drag_work=maneuver_energy.valid?
        maneuver_energy.modeled_drag_work:0.0;
    out.energy_uncertainty=maneuver_energy.valid?
        maneuver_energy.uncertainty_specific_energy:0.0;

    /* A capture that cannot complete the bounded path before its committed
       arrival time is unsafe even when it is already inside the spatial tube.
       Keep the acquisition-time failure in the maneuver/energy veto family so
       planning and live diagnostics report the same contract violation. */
    if(maneuver_geometry&&(!maneuver_energy.valid||out.energy_margin<0.0||
                           out.turn_margin<0.0))out.veto|=16u;

    /* The historical generic alignment maneuver is retained only to publish
       diagnostic energy numbers.  It is not a handoff proof: MM305 must accept
       only the actual live-origin partial Heading Alignment Cone generated from
       the current vector, which entry_dynamic_interface_capture() verifies with
       terminal_live_hac_handoff_veto(). */

    /* Structural margins use the configured physical limits directly. */
    if(t->dynamic_pressure>v->maximum_dynamic_pressure||
       t->g_force>v->maximum_g_load||
       (t->stall_fraction_is_measured&&t->stall_fraction>=1.0)) /* decision-literal-ok: stall fraction normalized domain */
        out.veto|=64u;

    (void)maneuver_path;
    (void)maneuver_slope;
    out.ready=out.veto==0;
    return out;
}

TaemInterfaceCapture entry_mm305_acquisition_capture(const GuidanceMachine*g,
        const TaemInterfaceTarget*q,const Telemetry*t,double course,
        const PlanetModel*p,const LandingConfiguration*cfg){
    TaemInterfaceCapture out={0};
    if(!q||!t||!p||!cfg||!q->valid||
       !isfinite(q->along_track)||!isfinite(q->cross_track)||!isfinite(q->course)||
       !isfinite(t->latitude)||!isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track)||
       !isfinite(t->true_air_speed)||!isfinite(t->horizontal_speed)||
       !isfinite(t->vertical_speed)||!isfinite(t->mean_altitude)||!isfinite(course)||
       !isfinite(t->dynamic_pressure)||!isfinite(t->g_force)||
       (t->stall_fraction_is_measured&&!isfinite(t->stall_fraction)))return out;
    const GuidanceSettings*s=&cfg->guidance;
    const VehicleProfile*v=&cfg->vehicle;
    ControlAuthorityEnvelope control=decision_control_authority_envelope(g,t,p,cfg);
    if(!control.valid||!control.controllable||
       !(control.maximum_lateral_accel_mps2>DBL_MIN)||
       !(t->horizontal_speed>DBL_MIN))return out;
    double qh=norm_signed_deg(q->course-cfg->site.runway_heading)*DEG2RAD;
    double da=q->along_track-t->runway_along_track;
    double dc=q->cross_track-t->runway_cross_track;
    out.along=da*cos(qh)+dc*sin(qh);
    out.cross=-da*sin(qh)+dc*cos(qh);
    out.course_error=norm_signed_deg(q->course-course);
    out.altitude_error=t->mean_altitude-s->taem_interface_altitude;
    out.minimum_turn_radius_m=t->horizontal_speed*t->horizontal_speed/
        control.maximum_lateral_accel_mps2;
    double path_length=decision_target_path_length(g,t,course,
        q->along_track,q->cross_track,q->course,
        control.maximum_lateral_accel_mps2,p,cfg,NULL);
    out.path_length_m=isfinite(path_length)?path_length:hypot(da,dc);
    out.required_time_s=isfinite(out.path_length_m)?
        control.control_response_time_s+out.path_length_m/t->horizontal_speed:NAN;
    out.available_time_s=isfinite(q->arrival_ut)?q->arrival_ut-t->ut:NAN;
    out.turn_margin=isfinite(out.required_time_s)&&isfinite(out.available_time_s)?
        (out.available_time_s-out.required_time_s)*t->horizontal_speed:NAN;
    double final_altitude=cfg->site.altitude+
        s->final_approach_distance*tan(s->final_glide_slope*DEG2RAD);
    double final_speed=fmax(v->final_approach_speed,v->minimum_safe_speed);
    out.energy_margin=entry_remaining_specific_energy(t->latitude,t->mean_altitude,
        t->true_air_speed,cfg->site.latitude,final_altitude,final_speed,p);
    out.energy_available=out.energy_margin;
    out.valid=isfinite(out.along)&&isfinite(out.cross)&&
        isfinite(out.minimum_turn_radius_m)&&isfinite(out.energy_margin);
    if(!out.valid)return out;
    double range=hypot(t->runway_along_track,t->runway_cross_track);
    double max_range=entry_taem_range_target(p,s);
    double min_range=fmax(s->final_approach_distance*1.5,
        s->final_approach_distance+fmax(0.0,s->hac_radius));
    if(!isfinite(range)||!isfinite(max_range)||range<min_range||range>max_range)
        out.veto|=2u;
    double runway=cfg->site.runway_heading*DEG2RAD;
    double de=-t->runway_along_track*sin(runway)-t->runway_cross_track*cos(runway);
    double dn=-t->runway_along_track*cos(runway)+t->runway_cross_track*sin(runway);
    double site_course=norm_deg(atan2(de,dn)*RAD2DEG);
    double closing_error=norm_signed_deg(site_course-course);
    if(!isfinite(closing_error)||cos(closing_error*DEG2RAD)<=0.0)out.veto|=4u;
    double sound=planet_atmospheric_speed_of_sound(p,t->mean_altitude);
    double mach=isfinite(t->mach)&&t->mach>0.0?t->mach:
        (isfinite(sound)&&sound>DBL_MIN?t->true_air_speed/sound:NAN);
    if(t->true_air_speed<v->minimum_safe_speed||
       !isfinite(mach)||mach<s->mm305_target_mach-s->mm305_mach_half_width||
       mach>s->mm305_target_mach+s->mm305_mach_half_width)out.veto|=1u;
    if(t->mean_altitude<s->mm305_min_altitude||
       t->mean_altitude>s->mm305_max_altitude)out.veto|=8u;
    if(!(out.energy_margin>0.0))out.veto|=16u;
    if(!(t->vertical_speed<0.0))out.veto|=32u;
    if(t->dynamic_pressure>v->maximum_dynamic_pressure||
       t->g_force>v->maximum_g_load||
       (t->stall_fraction_is_measured&&t->stall_fraction>=1.0))out.veto|=64u;
    out.ready=out.veto==0u;
    return out;
}


TaemInterfaceCapture entry_mm305_admission_envelope(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,
        const LandingConfiguration*cfg){
    TaemInterfaceCapture out={0};
    if(!t||!p||!cfg||
       !isfinite(t->latitude)||!isfinite(t->runway_along_track)||
       !isfinite(t->runway_cross_track)||!isfinite(t->true_air_speed)||
       !isfinite(t->horizontal_speed)||!isfinite(t->vertical_speed)||
       !isfinite(t->mean_altitude)||!isfinite(course)||
       !isfinite(t->dynamic_pressure)||!isfinite(t->g_force)||
       (t->stall_fraction_is_measured&&!isfinite(t->stall_fraction)))
        return out;

    const GuidanceSettings*s=&cfg->guidance;
    const VehicleProfile*v=&cfg->vehicle;
    ControlAuthorityEnvelope control=decision_control_authority_envelope(g,t,p,cfg);
    /* Admission must stay diagnosable when the measured state violates a
       survivability limit.  q/g/stall are ownership vetoes below, not reasons
       to make the evaluator disappear.  Only missing/degenerate physical
       authority prevents computing the state-set margins at all. */
    if(!control.valid||
       !(control.maximum_normal_accel_mps2>0.0)||
       !(control.maximum_lateral_accel_mps2>DBL_MIN)||
       !(t->horizontal_speed>DBL_MIN))
        return out;

    /*
     * MM304 transfers ownership into a state set, not a waypoint.  Admission is
     * deliberately a lower-bound test: the exact lead/HAC/final path belongs to
     * MM305 and is replay-qualified there from the measured handoff state.
     *
     * Freezing the instantaneous high-Mach turn radius through the entire TAEM
     * descent is physically wrong: density rises, speed falls, and available
     * curvature changes continuously.  It can inflate a 30-50 km terminal
     * problem into a fictitious several-hundred-kilometre Dubins path and keep
     * MM304 in control until the vehicle is already below the TAEM envelope.
     *
     * The straight runway-frame distance to Final's alignment interface is the
     * true geometric lower bound.  If the vehicle lacks energy even for this
     * path, handoff is impossible.  If it passes, native MM305 still has to
     * construct and replay-qualify a real bounded-control route before commit.
     */
    double final_along=-s->final_approach_distance;
    double final_cross=0.0;
    double final_course=cfg->site.runway_heading;
    double da=final_along-t->runway_along_track;
    double dc=final_cross-t->runway_cross_track;
    double minimum_path=hypot(da,dc);
    double minimum_turn_radius=t->horizontal_speed*t->horizontal_speed/
        control.maximum_lateral_accel_mps2;
    if(!isfinite(minimum_path)||!isfinite(minimum_turn_radius)||
       !(minimum_turn_radius>0.0))
        return out;

    double final_altitude=cfg->site.altitude+
        s->final_approach_distance*tan(s->final_glide_slope*DEG2RAD);
    double final_speed=fmax(v->final_approach_speed,v->minimum_safe_speed);
    double available_energy=entry_remaining_specific_energy(
        t->latitude,t->mean_altitude,t->true_air_speed,
        cfg->site.latitude,final_altitude,final_speed,p);
    UnpoweredPathProjection projection=decision_unpowered_path_projection(
        g,t,p,cfg,final_altitude,minimum_path);
    EnergyPathEnvelope energy={0};
    if(projection.valid&&isfinite(available_energy))
        energy=decision_energy_path_envelope(t,p,available_energy,
            projection.modeled_drag_work,projection.air_path_m);

    out.along=final_along-t->runway_along_track;
    out.cross=final_cross-t->runway_cross_track;
    out.course_error=norm_signed_deg(final_course-course);
    out.altitude_error=t->mean_altitude-
        clampd(t->mean_altitude,s->mm305_min_altitude,s->mm305_max_altitude);
    out.minimum_turn_radius_m=minimum_turn_radius;
    out.path_length_m=minimum_path;
    out.required_time_s=control.control_response_time_s+
        minimum_path/t->horizontal_speed;
    out.available_time_s=projection.valid?projection.travel_time_s:NAN;
    /* Geometry is already proved by the finite bounded-curvature solution.
       Keep turn_margin as a signed reachability diagnostic rather than inventing
       an arrival timestamp. */
    out.turn_margin=isfinite(minimum_path)?0.0:-INFINITY;
    /* MM304 admission is a measured-state ownership boundary, not a confidence
       certification of the predictor.  Require positive nominal physical energy
       after the lower-bound path loss, but never turn model uncertainty itself
       into a hard handoff veto.  MM305 immediately replans/replays from the
       measured state and closed-loop feedback remains responsible for residual
       model error. Keep the uncertainty term explicit for telemetry and planning. */
    out.energy_available=energy.valid?energy.available_specific_energy:NAN;
    out.energy_drag_work=energy.valid?energy.modeled_drag_work:NAN;
    out.energy_uncertainty=energy.valid?energy.uncertainty_specific_energy:NAN;
    out.energy_margin=energy.valid?
        energy.available_specific_energy-energy.modeled_drag_work:NAN;
    out.valid=isfinite(out.along)&&isfinite(out.cross)&&
        isfinite(out.minimum_turn_radius_m)&&energy.valid;
    if(!out.valid)return out;

    double range=hypot(t->runway_along_track,t->runway_cross_track);
    double maximum_entry_range=entry_taem_range_target(p,s);
    if(!isfinite(range)||!isfinite(maximum_entry_range)||
       range>maximum_entry_range)
        out.veto|=2u;

    double sound=planet_atmospheric_speed_of_sound(p,t->mean_altitude);
    double mach=isfinite(t->mach)&&t->mach>0.0?t->mach:
        (isfinite(sound)&&sound>DBL_MIN?t->true_air_speed/sound:NAN);
    if(t->true_air_speed<v->minimum_safe_speed||
       !isfinite(mach)||
       mach<s->mm305_target_mach-s->mm305_mach_half_width||
       mach>s->mm305_target_mach+s->mm305_mach_half_width)
        out.veto|=1u;
    if(t->mean_altitude<s->mm305_min_altitude||
       t->mean_altitude>s->mm305_max_altitude)
        out.veto|=8u;
    if(!(out.energy_margin>0.0))
        out.veto|=16u;
    if(!(t->vertical_speed<0.0))
        out.veto|=32u;
    /* Closure: MM305 can turn a vehicle onto the HAC, but a vehicle flying
       away from the site at the TAEM window has not finished the MM304
       heading problem, and the energy test above assumed a straight path. */
    if(isfinite(t->course_to_site_error)&&fabs(t->course_to_site_error)>90.0)
        out.veto|=128u;
    if(t->dynamic_pressure>v->maximum_dynamic_pressure||
       t->g_force>v->maximum_g_load||
       (t->stall_fraction_is_measured&&t->stall_fraction>=1.0))
        out.veto|=64u;

    out.ready=out.veto==0u;
    return out;
}
