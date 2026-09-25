/* A segment can be committed while thin-air/stall guards correctly deny usable
   bank authority. Once that physical authority first becomes available, the
   pre-leg segment may be reshaped immediately, but the child must not postpone
   the parent's already-scheduled ordinary refresh. Latch the acquisition so
   transient stall chatter cannot turn this into another level-triggered loop. */


/* The MM304 outlet is a pose constraint, not merely a waypoint. Before the final
   reversal, aim the owned S-turn leg at the start of the nominal terminal circle
   that would finish at the fixed TAEM point/course. This converts the required
   crossrange into an explicit setup target instead of relying on a fixed bearing
   lead. The live final-reversal gate still proves the measured-radius arc. */
static bool entry_taem_pose_setup_point(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg,double*setup_along,double*setup_cross,double*final_side){
    if(setup_along)*setup_along=NAN;if(setup_cross)*setup_cross=NAN;if(final_side)*final_side=0.0;
    if(!g||!t||!p||!cfg||!g->taem_interface_target.valid||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return false;
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double course=isfinite(t->ground_track_heading)?norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(course)||!isfinite(q->course))return false;
    double target_relative=norm_signed_deg(q->course-cfg->site.runway_heading);
    if(fabs(target_relative)<30.0)return false;
    double side=target_relative>=0.0?1.0:-1.0;
    double radius=entry_taem_shaping_setup_radius(q,t,p,cfg);
    if(!isfinite(radius)||radius<1000.0)return false;
    double psi0=norm_signed_deg(course-cfg->site.runway_heading)*DEG2RAD;
    double psi1=target_relative*DEG2RAD;
    double along=q->along_track-radius/side*(sin(psi1)-sin(psi0));
    double cross=q->cross_track-radius/side*(cos(psi0)-cos(psi1));
    if(!isfinite(along)||!isfinite(cross))return false;
    if(setup_along)*setup_along=along;if(setup_cross)*setup_cross=cross;if(final_side)*final_side=side;
    return true;
}




double entry_taem_tangent_target_hac_side(const TaemInterfaceTarget*q,
        const LandingConfiguration*cfg){
    if(!q||!cfg||!q->valid||!isfinite(q->along_track)||!isfinite(q->cross_track)||
       !isfinite(q->course)||!isfinite(q->hac_radius)||!(q->hac_radius>0.0))
        return 0.0;
    TaemHandoffContract contract=taem_handoff_contract(&cfg->guidance);
    double runway=cfg->site.runway_heading*DEG2RAD;
    double best_side=0.0,best_error=INFINITY;
    for(int turn=-1;turn<=1;turn+=2){
        HACPoint2 entry={0},center={0},exit={0};
        if(!hac_fixed_alignment_geometry(&entry,&center,&exit,&cfg->site,
                &cfg->guidance,q->hac_radius,q->course,(double)turn))continue;
        double expected_along=entry.e*sin(runway)+entry.n*cos(runway);
        double expected_cross=entry.e*cos(runway)-entry.n*sin(runway);
        double error=hypot(q->along_track-expected_along,
            q->cross_track-expected_cross);
        if(isfinite(error)&&error<best_error){
            best_error=error;
            best_side=(double)turn;
        }
    }
    return isfinite(best_error)&&best_error<=contract.horizontal_radius_m?
        best_side:0.0;
}

bool entry_taem_tangent_target_geometry(const TaemInterfaceTarget*q,
        const LandingConfiguration*cfg){
    if(!q||!cfg||!q->valid||!isfinite(q->along_track)||!isfinite(q->cross_track)||
       !isfinite(q->course)||!isfinite(q->speed)||q->speed<=0.0||
       !isfinite(q->hac_radius)||!(q->hac_radius>0.0))return false;
    TaemHandoffContract contract=taem_handoff_contract(&cfg->guidance);
    double relative=norm_signed_deg(q->course-cfg->site.runway_heading);
    const double perpendicular=90.0; /* decision-literal: mathematical-numerical-requirement | MM304 inlet tangent is perpendicular to runway */
    double perpendicular_error=fabs(fabs(relative)-perpendicular);
    if(!isfinite(perpendicular_error)||
       perpendicular_error>contract.perpendicular_heading_half_width_deg)return false;

    /* The target is the fixed MM304 handoff station, while HAC-entry geometry
       remains a downstream MM305 responsibility. */
    double along=NAN,cross=NAN;
    mm304_handoff_station(&cfg->guidance,&along,&cross);
    return isfinite(along)&&isfinite(cross)&&
        hypot(q->along_track-along,q->cross_track-cross)<=
            contract.horizontal_radius_m;
}

static bool entry_taem_hac_inlet_target(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,const LandingConfiguration*cfg,double reference_course,
        double hac_radius,double*target_along,double*target_cross,
        double*target_course,double*target_side){
    if(target_along)*target_along=NAN;
    if(target_cross)*target_cross=NAN;
    if(target_course)*target_course=NAN;
    if(target_side)*target_side=NAN;
    if(!g||!t||!p||!cfg||!(hac_radius>0.0)||!isfinite(reference_course)||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return false;

    /*
     * Choose the inlet by the live bounded-curvature path, not by Euclidean
     * position alone.  The old selector could choose an inlet a few hundred
     * metres away whose tangent course was 180 degrees opposite the velocity,
     * then freeze that choice.  A shuttle already flying toward the opposite
     * inlet would consequently pass KSC while MM304 waited for an impossible
     * reversal.
     */
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    if(!authority.valid||
       !(authority.maximum_lateral_accel_mps2>DBL_MIN)||
       !(t->horizontal_speed>DBL_MIN))return false;
    const double perpendicular=90.0; /* decision-literal: mathematical-numerical-requirement | perpendicular HAC inlet tangent */
    bool have=false;
    double best_score=INFINITY,best_along=NAN,best_cross=NAN,best_course=NAN,best_side=NAN;
    double retained_course=g->taem_interface_target.valid&&
        isfinite(g->taem_interface_target.course)?g->taem_interface_target.course:NAN;
    double retained_side=entry_taem_tangent_target_hac_side(
        &g->taem_interface_target,cfg);

    /*
     * A perpendicular inlet course does not determine the HAC turn side.  The
     * same course has both a short-arc and long-arc runway-anchored solution.
     * Evaluate all four physical inlet poses and let live bounded-curvature
     * reachability choose the one MM304 can actually deliver.
     */
    for(int direction=-1;direction<=1;direction+=2){
        double course=norm_deg(cfg->site.runway_heading+direction*perpendicular);
        for(int turn=-1;turn<=1;turn+=2){
            double side=(double)turn;
            HACPoint2 entry={0},center={0},exit={0};
            if(!hac_fixed_alignment_geometry(&entry,&center,&exit,&cfg->site,
                    &cfg->guidance,hac_radius,course,side))continue;
            /* MM304 owns the vehicle only until the declared handoff set.
               The HAC entry computed above is a downstream MM305 pose and is
               not the MM304/MM305 interface point. */
            double along=NAN,cross=NAN;
            mm304_handoff_station(&cfg->guidance,&along,&cross);
            if(!isfinite(along)||!isfinite(cross))continue;

            /* A terminal tangent is a delivery course, not an arbitrary heading
               that happens to be 90 degrees from the runway.  Reject a candidate
               whose course points away from the current station: a Dubins path
               can otherwise win by adding a large loop, turning an otherwise
               recoverable closure into an energetically destructive excursion. */
            double da_station=along-t->runway_along_track;
            double dc_station=cross-t->runway_cross_track;
            double station_range=hypot(da_station,dc_station);
            if(!(station_range>DBL_MIN))continue;
            double station_bearing=norm_deg(cfg->site.runway_heading+
                atan2(dc_station,da_station)*RAD2DEG);
            double closure_error=norm_signed_deg(course-station_bearing);
            const char*candidate_diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
            if(candidate_diag&&strcmp(candidate_diag,"1")==0)
                fprintf(stderr,"MM304 candidate course: UT %.2f course %.1f candidate %.1f stationBearing %.1f closureErr %.1f range %.0f side %+.0f\n",
                    t->ut,reference_course,course,station_bearing,closure_error,
                    station_range,side);
            if(fabs(closure_error)>90.0)continue;

            /*
             * Evaluate the complete bounded-curvature connection to the candidate
             * pose. Euclidean distance plus a heading-debt scalar is not equivalent
             * to a Dubins path and can select an inlet behind the vehicle.
             */
            double candidate_minimum_radius=INFINITY;
            double score=decision_target_path_length(g,t,reference_course,
                along,cross,course,authority.maximum_lateral_accel_mps2,
                p,cfg,&candidate_minimum_radius);
            if(!isfinite(score)||!isfinite(candidate_minimum_radius))continue;
            if(candidate_diag&&strcmp(candidate_diag,"1")==0)
                fprintf(stderr,"MM304 candidate: UT %.2f course %.1f side %+.0f range %.0f score %.0f Rmin %.0f target %.0f,%.0f -> %.0f,%.0f\n",
                    t->ut,reference_course,side,hypot(along-t->runway_along_track,cross-t->runway_cross_track),
                    score,candidate_minimum_radius,along,cross,along,cross);

            bool retained=isfinite(retained_course)&&fabs(retained_side)==1.0&&
                fabs(norm_signed_deg(course-retained_course))<=
                    sqrt(DBL_EPSILON)*RAD2DEG&&
                side==retained_side;
            double scale=fmax(1.0,fmax(fabs(score),fabs(best_score)));
            bool tied=have&&isfinite(best_score)&&
                fabs(score-best_score)<=sqrt(DBL_EPSILON)*scale;
            if(!have||score<best_score||(tied&&retained)){
                have=true;best_score=score;best_along=along;best_cross=cross;
                best_course=course;best_side=side;
            }
        }
    }
    if(!have)return false;
    if(target_along)*target_along=best_along;
    if(target_cross)*target_cross=best_cross;
    if(target_course)*target_course=best_course;
    if(target_side)*target_side=best_side;
    return true;
}

void entry_publish_taem_tangent_target(GuidanceMachine*g,const Telemetry*t,
        double course,const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||g->terminal_region_entered)return;
    (void)aero; /* Target geometry is qualified by the shared live-state envelopes. */
    /* Keep a geometry-only candidate refreshable.  Its fixed inlet remains the
       same, but ETA and energy qualification are state-dependent; freezing the
       first high-altitude projection preserved a billion-metre turn radius all
       the way into the atmosphere. */
    if(entry_taem_tangent_target_geometry(&g->taem_interface_target,cfg)&&
       g->taem_interface_target.energy_qualified)return;

    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;
    double reference_course=isfinite(course)?norm_deg(course):
        (isfinite(t->ground_track_heading)?norm_deg(t->ground_track_heading):norm_deg(t->heading));
    if(!isfinite(reference_course)||!(t->horizontal_speed>DBL_MIN))return;

    double compatibility_speed=hac_acquisition_speed_target(v,s,p);

    double slope=clampd(fabs(s->taem_glide_slope),
        DBL_EPSILON,nextafter(90.0,0.0)); /* decision-literal-ok: tangent singularity */
    double target_fpa=-slope;
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    if(!authority.valid||!isfinite(authority.control_response_time_s)||
       authority.control_response_time_s<0.0)return;
    double response=authority.control_response_time_s;

    double final_altitude=cfg->site.altitude+
        s->final_approach_distance*tan(s->final_glide_slope*DEG2RAD);

    /* The published tangent is the later Phase-1/Phase-2 acquisition target
     * that MM305 flies toward after entering TAEM at the high-energy interface.
     * Clamp only the ownership floor; the test/configuration may intentionally
     * place the interface altitude below that floor. */
    double target_altitude=fmax(s->hac_acquisition_altitude,
        s->mm305_min_altitude);
    if(!isfinite(target_altitude)||target_altitude<=final_altitude)return;
    TaemSpeedEnvelope speed_envelope=
        decision_taem_speed_envelope(v,p,target_altitude);
    if(!speed_envelope.valid||!speed_envelope.feasible)return;
    double target_sound=planet_atmospheric_speed_of_sound(p,target_altitude);
    double target_speed=compatibility_speed;
    if(!isfinite(target_sound)||target_sound<=DBL_MIN||
       !isfinite(target_speed)||target_speed<=0.0)return;

    /* The HAC radius is geometric mission intent.  Do not freeze the MM304
       handoff state's instantaneous minimum-turn radius into MM305 geometry: the
       native finite lead is allowed to decelerate and build density before joining
       the HAC, and full native replay proves whether that join is physically
       executable.  Inflating this radius at Mach ~2-3 produced 50-70 km HACs and
       hundreds of kilometres of unnecessary arc, causing the observed KSC
       overshoot. */
    double terminal_radius=s->hac_radius;
    if(!isfinite(terminal_radius)||!(terminal_radius>=3000.0))return;
    double target_along=NAN,target_cross=NAN,target_course=NAN,target_side=NAN;
    if(!entry_taem_hac_inlet_target(g,t,p,cfg,reference_course,
            terminal_radius,&target_along,&target_cross,&target_course,
            &target_side))return;
    (void)target_side;

    double rh=cfg->site.runway_heading*DEG2RAD;
    GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
    double target_e=target_along*sin(rh)+target_cross*cos(rh);
    double target_n=target_along*cos(rh)-target_cross*sin(rh);
    GeoPoint point=local_point(origin,target_e,target_n,
        p->radius,target_altitude);

    HandoffSetEnvelope handoff_set=decision_mm304_handoff_set_envelope(
        g,t,reference_course,target_along,target_cross,p,cfg);
    double minimum_turn_radius=handoff_set.minimum_turn_radius_m;
    double approach_path=handoff_set.path_to_set_m;
    const char*decision_diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    bool decision_diag_on=decision_diag&&strcmp(decision_diag,"1")==0;
    if(!handoff_set.valid||!isfinite(approach_path)){
        if(decision_diag_on)fprintf(stderr,
            "MM304 target reject: handoff-set geometry invalid at UT %.2f course %.1f.\n",
            t->ut,reference_course);
        return;
    }
    double relative=fabs(norm_signed_deg(target_course-cfg->site.runway_heading));
    double alignment_arc=relative*DEG2RAD*terminal_radius;
    double remaining=s->final_approach_distance+alignment_arc;
    double distance=hypot(target_along-t->runway_along_track,
        target_cross-t->runway_cross_track);

    /*
     * Keep longitudinal energy accounting independent from the instantaneous
     * lateral-authority proof. handoff_set.path_to_set_m deliberately inflates
     * the route by the turn radius available at the current q/lift. In the upper
     * atmosphere that radius can be thousands of kilometres even though the
     * orbiter will acquire most of its turning authority later in the descent.
     * Feeding that conservative lateral upper bound into the drag integrator made
     * a physically high-energy entry project to zero speed and prevented the
     * topology planner from ever being allowed to schedule its first reversal.
     *
     * Energy qualification therefore follows the finite geometric displacement
     * to the published inlet, plus the published terminal alignment arc. This is
     * not lateral certification: handoff_set, the propagated topology and the live
     * capture contract still independently prove that the path can actually be
     * flown before MM304 can release ownership.
     */
    double energy_path_to_inlet=distance;
    UnpoweredPathProjection arrival=decision_unpowered_path_projection(
        g,t,p,cfg,target_altitude,energy_path_to_inlet);
    if(!arrival.valid){
        if(decision_diag_on)fprintf(stderr,
            "MM304 target reject: energy projection invalid at UT %.2f energyPath %.0f proofPath %.0f Rmin %.0f.\n",
            t->ut,energy_path_to_inlet,approach_path,minimum_turn_radius);
        return;
    }

    /* The published inlet only has to prove that MM304 can deliver an
       unpowered vehicle into the actual MM305 engagement energy band. The
       complete TAEM/HAC/final maneuver is re-propagated by entry_topology_trial()
       and rechecked by entry_taem_interface_capture() at the ownership boundary.
       Requiring a second current-state projection all the way to final approach
       here duplicated that downstream proof and, worse, forced one geometric
       descent profile before the S-turn topology even existed. That circular
       gate prevented every reversal from being planned. */
    double mm305_min_speed=speed_envelope.minimum_speed_mps;
    double mm305_max_speed=speed_envelope.maximum_speed_mps;
    bool energy_qualified=arrival.valid&&
        arrival.terminal_speed_mps>=mm305_min_speed&&
        arrival.terminal_speed_mps<=mm305_max_speed;
    /*
     * A geometry-only target is an acquisition preview, not a certification of
     * the exact fixed-pose turn. At the upper atmosphere the live lateral
     * authority can make handoff_set.path_to_set_m enormous (the instantaneous
     * minimum-turn radius tends toward infinity), even though the vehicle still
     * has a finite forward distance to the handoff station and a finite terminal
     * HAC reserve after that station. Feeding that conservative proof path into
     * arrival_ut makes the preview effectively millions of seconds away and
     * contaminates the first-leg planner. Keep the strict capture veto on the
     * live envelope, but give the advisory target a finite geometric ETA derived
     * from the current station distance plus the configured terminal reserve.
     */
    double geometry_path=isfinite(distance)&&distance>=0.0&&isfinite(remaining)&&remaining>=0.0?
        distance+remaining:approach_path;
    double arrival_time=energy_qualified?arrival.travel_time_s:
        response+geometry_path/fmax(t->horizontal_speed,DBL_MIN);
    if(!isfinite(arrival_time)||arrival_time<0.0)return;
    if(decision_diag_on){
        if(energy_qualified)
            fprintf(stderr,
                "MM304 target accept: inletV %.1f inside MM305 [%.1f, %.1f], guidanceV %.1f structural [%.1f, %.1f], energyPath %.0f proofPath %.0f Rmin %.0f dragWork %.0f travel %.1f.\n",
                arrival.terminal_speed_mps,mm305_min_speed,mm305_max_speed,target_speed,
                speed_envelope.minimum_speed_mps,speed_envelope.maximum_speed_mps,
                energy_path_to_inlet,approach_path,minimum_turn_radius,
                arrival.modeled_drag_work,arrival_time);
        else
            fprintf(stderr,
                "MM304 geometry target: inletV %.1f outside MM305 [%.1f, %.1f], guidanceV %.1f structural [%.1f, %.1f], energyPath %.0f proofPath %.0f Rmin %.0f ETA %.1f.\n",
                arrival.valid?arrival.terminal_speed_mps:NAN,mm305_min_speed,mm305_max_speed,target_speed,
                speed_envelope.minimum_speed_mps,speed_envelope.maximum_speed_mps,
                energy_path_to_inlet,approach_path,minimum_turn_radius,arrival_time);
    }

    double lead=t->horizontal_speed*response;
    TaemHandoffContract contract=taem_handoff_contract(&cfg->guidance);

    TaemInterfaceTarget next={0};
    next.valid=isfinite(point.latitude)&&isfinite(point.longitude)&&
        isfinite(target_altitude)&&isfinite(target_speed)&&target_speed>0.0&&
        isfinite(target_course)&&isfinite(arrival_time)&&
        contract.horizontal_radius_m>0.0&&
        contract.perpendicular_heading_half_width_deg>0.0;
    if(!next.valid)return;
    next.energy_qualified=energy_qualified;
    next.along_track=target_along;next.cross_track=target_cross;
    next.course=target_course;next.hac_radius=terminal_radius;
    next.altitude=target_altitude;next.speed=target_speed;next.flight_path_angle=target_fpa;
    next.specific_energy=rotating_specific_energy(point.latitude,target_altitude,target_speed,p);
    next.selected_ut=t->ut;next.arrival_ut=t->ut+arrival_time;
    next.quality_score=
        fabs(norm_signed_deg(next.course-reference_course))/
            contract.perpendicular_heading_half_width_deg+
        distance/contract.horizontal_radius_m;
    next.acquisition_lead=lead;
    next.remaining_path=fmax(lead,remaining);
    next.response_time=response;
    g->taem_interface_target=next;
    /* A provisional inlet intentionally suppresses the expensive propagated
       topology search.  The instant its unpowered arrival prediction enters
       the actual MM305 Mach/structural window is therefore a planning event: if
       we do not re-arm the worker here, the last provisional no-topology result
       can leave MM304 flying the fixed inlet with no route to it. */
    if(next.energy_qualified&&!g->entry_topology.valid)
        g->entry_planning_needed=true;
}

static bool entry_taem_alignment_station_missed(const GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg,double*point_bearing){
    if(point_bearing)*point_bearing=NAN;
    if(!g||!t||!cfg||!g->taem_interface_target.valid||
       !isfinite(t->runway_along_track)||!isfinite(t->runway_cross_track))return false;
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double rel=norm_signed_deg(q->course-cfg->site.runway_heading)*DEG2RAD;
    double tangent_along=cos(rel),tangent_cross=sin(rel);
    double normal_along=-tangent_cross,normal_cross=tangent_along;
    double from_along=t->runway_along_track-q->along_track;
    double from_cross=t->runway_cross_track-q->cross_track;
    double progress=from_along*tangent_along+from_cross*tangent_cross;
    double line_dist=fabs(from_along*normal_along+from_cross*normal_cross);
    double distance=hypot(from_along,from_cross);
    double to_along=-from_along,to_cross=-from_cross;
    double bearing=norm_deg(cfg->site.runway_heading+atan2(to_cross,to_along)*RAD2DEG);
    if(point_bearing)*point_bearing=bearing;

    TaemHandoffContract contract=taem_handoff_contract(&cfg->guidance);
    if(!isfinite(distance)||distance<=contract.horizontal_radius_m)return false;

    double live=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(live))return false;
    double live_rel=norm_signed_deg(live-cfg->site.runway_heading)*DEG2RAD;
    double forward_to_point=to_along*cos(live_rel)+to_cross*sin(live_rel);
    bool passed_tangent=line_dist<=fmax(5000.0,contract.horizontal_radius_m*2.0)&&
        isfinite(progress)&&progress>contract.horizontal_radius_m;
    bool point_behind_velocity=isfinite(forward_to_point)&&forward_to_point<0.0;
    return passed_tangent||point_behind_velocity;
}

static double entry_taem_tangent_capture_heading(const GuidanceMachine*g,const Telemetry*t,
        const LandingConfiguration*cfg){
    if(!g||!t||!cfg)return 0.0;
    if(!g->taem_interface_target.valid||!isfinite(t->runway_along_track)||
       !isfinite(t->runway_cross_track)){
        double live=isfinite(t->ground_track_heading)?t->ground_track_heading:t->heading;
        return isfinite(live)?norm_deg(live):0.0;
    }
    double point_bearing=NAN;
    if(entry_taem_alignment_station_missed(g,t,cfg,&point_bearing)){
        double live=isfinite(t->ground_track_heading)?
            norm_deg(t->ground_track_heading):norm_deg(t->heading);
        if(!isfinite(live)||!isfinite(point_bearing))
            return g->taem_interface_target.course;
        double point_turn=norm_signed_deg(point_bearing-live);
        double course_turn=norm_signed_deg(g->taem_interface_target.course-live);
        double resolution=sqrt(DBL_EPSILON)*RAD2DEG;
        double side=fabs(course_turn)>resolution?
            (course_turn>=0.0?1.0:-1.0):(g->s_turn_sign>=0.0?1.0:-1.0);
        if(side*point_turn>0.0)return point_bearing;
        /* The point is aft of the committed turn. Continue on the geometric
           quarter-turn tangent until it moves onto the owned side; do not
           reverse S-turn ownership merely to chase an aft waypoint. */
        return norm_deg(live+side*90.0); /* decision-literal-ok: quarter-turn tangent geometry */
    }
    return taem_interface_line_heading(t->runway_along_track,t->runway_cross_track,
        cfg->site.runway_heading,&g->taem_interface_target);
}


static double entry_taem_tangent_capture_bank(const GuidanceMachine*g,const Telemetry*t,
        AerodynamicModel aero,const LandingConfiguration*cfg,double desired_heading){
    if(!g||!t||!cfg||!g->taem_interface_target.valid)return 0.0;
    double course=isfinite(t->ground_track_heading)?norm_deg(t->ground_track_heading):norm_deg(t->heading);
    double bank_limit=fmin(45.0,dynamic_bank_limit(t,&cfg->vehicle));
    return taem_course_rate_bank_command(t->horizontal_speed,live_lift_accel(t,aero,&cfg->vehicle),
        t->bank_effectiveness>0?t->bank_effectiveness:1.0,bank_limit,
        course,desired_heading,g->taem_interface_target.response_time);
}

static double entry_taem_minimum_bank_for_capture(const GuidanceMachine*g,
        const Telemetry*t,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg,double target_along,double target_cross,
        double target_course,double side){
    if(!g||!t||!p||!cfg||!isfinite(target_along)||!isfinite(target_cross)||
       !isfinite(target_course)||!(t->horizontal_speed>DBL_MIN))return 0.0;

    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double course=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(course))return 0.0;

    double distance=hypot(target_along-t->runway_along_track,
        target_cross-t->runway_cross_track);
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    if(!authority.valid||!authority.controllable)return 0.0;

    double available_time=isfinite(q->arrival_ut)&&q->arrival_ut>t->ut?
        q->arrival_ut-t->ut:
        (isfinite(q->remaining_path)&&q->remaining_path>0.0?
            authority.control_response_time_s+q->remaining_path/t->horizontal_speed:
            authority.control_response_time_s+distance/t->horizontal_speed);
    if(!isfinite(available_time))return 0.0;

    double lift=live_lift_accel(t,aero,&cfg->vehicle);
    double effectiveness=isfinite(t->bank_effectiveness)&&
        t->bank_effectiveness>0.0?t->bank_effectiveness:1.0;
    double limit=dynamic_bank_limit(t,&cfg->vehicle);
    if(!(lift>DBL_MIN)||!isfinite(limit)||limit<=0.0)return 0.0;

    double signed_side=side>=0.0?1.0:-1.0;
    double resolution=sqrt(DBL_EPSILON)*fmax(1.0,limit);
    double max_effective=fmin(limit*effectiveness,nextafter(90.0,0.0));
    double max_lateral=lift*fabs(sin(max_effective*DEG2RAD));
    TargetCaptureEnvelope maximum=
        decision_target_capture_envelope_with_lateral_accel(
            g,t,course,target_along,target_cross,target_course,
            available_time,max_lateral,p,cfg);
    if(!maximum.valid||!maximum.reachable)return signed_side*limit;

    double low=0.0,high=limit;
    unsigned guard=0;
    while(high-low>resolution&&guard++<(unsigned)(4*DBL_MANT_DIG)){
        double mid=.5*(low+high); /* decision-literal-ok: bisection midpoint */
        double effective=fmin(mid*effectiveness,nextafter(90.0,0.0));
        double lateral=lift*fabs(sin(effective*DEG2RAD));
        TargetCaptureEnvelope capture=
            decision_target_capture_envelope_with_lateral_accel(
                g,t,course,target_along,target_cross,target_course,
                available_time,lateral,p,cfg);
        if(capture.valid&&capture.reachable)high=mid;
        else low=mid;
    }
    return signed_side*high;
}

static double entry_taem_gate_required_bank(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    double course=isfinite(t->ground_track_heading)?
        norm_deg(t->ground_track_heading):norm_deg(t->heading);
    if(!isfinite(course))return 0.0;

    TaemInterfaceCapture handoff=
        entry_taem_interface_capture(q,t,course,p,cfg);
    if(handoff.valid&&handoff.ready)return 0.0;

    /* The MM304 turn side is a geometric property of the current station
       bearing, not the downstream HAC side.  The two can legitimately be
       opposite: an orbiter may have to turn right to reach a HAC whose entry
       side is left.  Using the latched HAC side here produced a geometrically
       valid but unreachable setup point behind the vehicle, followed by a
       wings-level excursion. */
    /* The fixed station is normally on the runway-parallel centerline.  A
       small course perturbation must not reinterpret that zero crossrange as
       a request to flip the already-owned S-turn side: that would manufacture
       a second turn from measurement noise or a few degrees of heading error.
       When there is a real lateral offset, its sign is the physical turn
       demand; for a centered station retain the latched topology side and let
       the heading-lock/terminal capture law resolve the residual course error. */
    double side=g->s_turn_sign>=0.0?1.0:-1.0;
    double target_along,target_cross,target_course,final_side=0.0;
    double setup_along=NAN,setup_cross=NAN;
    bool have_setup=entry_taem_pose_setup_point(g,t,p,cfg,
        &setup_along,&setup_cross,&final_side);
    bool shaping_leg=have_setup&&side*final_side<0.0&&
        !entry_program_final_reversal_geometry_ready(g,t,cfg,final_side);
    if(shaping_leg){
        target_along=setup_along;
        target_cross=setup_cross;
        target_course=course;
    }else{
        /* Before MM305 admission, retain the published inlet tangent. The
           geometric side is selected below from the measured crossrange sign;
           the simulator's positive-roll frame is opposite that sign. */
        target_along=q->along_track;
        target_cross=q->cross_track;
        double to_along=target_along-t->runway_along_track;
        double to_cross=target_cross-t->runway_cross_track;
        target_course=norm_deg(cfg->site.runway_heading+
            atan2(to_cross,to_along)*RAD2DEG);
        double course_error=norm_signed_deg(target_course-course);
        if(fabs(course_error)>sqrt(DBL_EPSILON))
            side=course_error>=0.0?1.0:-1.0;
    }
    return entry_taem_minimum_bank_for_capture(g,t,p,aero,cfg,
        target_along,target_cross,target_course,side);
}
static double entry_taem_gate_acquisition_bank(const GuidanceMachine*g,const Telemetry*t,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||!g->taem_interface_target.valid)return 0.0;
    double bank=entry_taem_gate_required_bank(g,t,p,aero,cfg);
    /* Preserve the signed physical demand even when it opposes the currently
     * latched S-turn side; the reversal supervisor must see that demand and
     * schedule a real turn.  Collapsing it to zero here is what made large
     * crossrange states fly straight away from the handoff station. */
    if(isfinite(bank)&&fabs(bank)>sqrt(DBL_EPSILON))return bank;

    double side=g->s_turn_sign>=0.0?1.0:-1.0;
    double recovery_bearing=NAN;
    bool missed=entry_taem_alignment_station_missed(g,t,cfg,&recovery_bearing);
    bool blocked=entry_program_terminal_geometry_blocked(g,t,cfg);
    if(!missed&&!blocked)return 0.0;
    const char *diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
    if(diag&&strcmp(diag,"1")==0)
        fprintf(stderr,"MM304 gate bank: UT %.2f side %+.0f bank %.3f missed %d blocked %d station %.0f/%.0f target %.0f/%.0f roll %.1f\n",
                t->ut,side,bank,missed?1:0,blocked?1:0,
                t->runway_along_track,t->runway_cross_track,
                g->taem_interface_target.along_track,
                g->taem_interface_target.cross_track,norm_signed_deg(t->roll));
    /* Missed-point recovery and an overdue blocked endpoint are topological
     * requirements, not invitations to invent a tuned bank floor. Request the
     * full live envelope; the coupled maximin allocator decides what portion is
     * executable after vertical and energy margins are included. */
    return side*dynamic_bank_limit(t,&cfg->vehicle);
}


/* After the first energy-management reversal, keep the acquired setup-side
   crossrange and carry it downstream on a runway-parallel leg until the measured
   endpoint arc becomes feasible. This prevents the shaping reversal and the final
   inlet turn from collapsing into one long maximum-bank maneuver. */
static bool entry_program_tangent_reversal_captured(GuidanceMachine*g,const Telemetry*t,
        const VehicleProfile*v){
    if(!g||!t||!v||!g->entry_final_reversal_pending)return g&&g->entry_final_reversal_completed;
    bool authority=entry_s_turn_bank_authority_available(t->dynamic_pressure,t->true_air_speed,
        t->stall_fraction,t->g_force,v);
    double actual=norm_signed_deg(t->roll);
    double roll_rate=controlled_roll_rate(t);
    double angle_resolution=sqrt(DBL_EPSILON)*fmax(1.0,fabs(actual));
    double rate_resolution=sqrt(DBL_EPSILON)*
        fmax(1.0,isfinite(roll_rate)?fabs(roll_rate):1.0);
    bool crossed=g->s_turn_sign*actual>angle_resolution;
    bool not_rolling_back=!isfinite(roll_rate)||
        g->s_turn_sign*roll_rate>=-rate_resolution;
    if(authority&&crossed&&not_rolling_back){
        g->entry_final_reversal_pending=false;
        g->entry_final_reversal_completed=true;
        return true;
    }
    return false;
}

TaemInterfaceCapture entry_dynamic_interface_capture(const GuidanceMachine*g,
        const Telemetry*t,double course,const PlanetModel*p,AerodynamicModel aero,
        const LandingConfiguration*cfg){
    TaemInterfaceCapture out={0};
    if(!g||!t||!p||!cfg)return out;
    (void)aero;
    const TaemInterfaceTarget*q=&g->taem_interface_target;
    if(!entry_taem_tangent_target_geometry(q,cfg))return out;
    return entry_mm305_acquisition_capture(g,q,t,course,p,cfg);
}
