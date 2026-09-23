bool guidance_terminal_preview_allowed(const GuidanceMachine *g){
    if(!g)return false;
    if(g->terminal_glide_mode||g->terminal_rehearsal_mode)return true;
    if(g->terminal_region_entered||taem_exec_owns_vehicle(&g->taem_exec))return true;
    return g->entry_final_reversal_completed;
}

static double terminal_prediction_refresh_period(const GuidanceMachine*g,
        const GuidanceSettings*s){
    double base=fmax(s?s->prediction_interval:0.0,DBL_EPSILON);
    if(!g||!s)return base;
    bool terminal_owned=g->terminal_region_entered||
        taem_exec_owns_vehicle(&g->taem_exec);
    bool margin_critical=!g->terminal_candidate.valid||
        g->terminal_candidate.geometry_degraded||
        g->terminal_candidate.energy_degraded||
        (g->terminal_candidate.execution_evaluated&&
         isfinite(g->terminal_candidate.execution_margin)&&
         g->terminal_candidate.execution_margin<10.0);
    /* A regular spline window can be shorter than the normal ~1.5 s planning
       cadence.  Once TAEM owns the vehicle, search at guidance-rate-adjacent
       cadence only while no fully healthy candidate exists.  This changes
       observation cadence, not any geometry, energy, or control limit. */
    if(terminal_owned&&margin_critical)
        return fmin(base,0.25);
    return base;
}

       void terminal_predict(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double dt){
    const GuidanceSettings*s=&cfg->guidance;const VehicleProfile*v=&cfg->vehicle;
    g->terminal_reference_path_lateral_acceleration=0.0;
    g->terminal_reference_path_bank=0.0;
    g->terminal_reference_path_course_error=0.0;
    g->terminal_reference_path_arc_remaining=0.0;
    g->terminal_reference_path_transition_active=false;
    if(!guidance_terminal_preview_allowed(g)){
        g->terminal_candidate.valid=false;
        g->terminal_prediction_valid=false;
        g->terminal_prediction_ut=-INFINITY;
        return;
    }
    ControlAuthorityEnvelope authority=
        decision_control_authority_envelope(g,t,p,cfg);
    if(g->terminal_path_committed||(g->hac_side_selected&&!g->terminal_test_capture_active)||
       g->final_approach_captured||t->vertical_speed>=0.0||
       !authority.valid||!authority.controllable)return;
    /* Preview geometry is disposable.  Once its propagated intercept is in
       the past (or the live unpowered vehicle has dropped below both its
       altitude and speed), remove it before scoring replacements.  Otherwise
       the refinement hysteresis can preserve a once-nominal but physically
       unreachable HAC forever simply because newer candidates are degraded. */
    if(g->terminal_candidate.valid&&terminal_candidate_expired(g,&g->terminal_candidate,t,course,p,cfg)){
        g->terminal_candidate.valid=false;
        g->terminal_mix=0.0;
    }
    /* A geometric acquisition demand does not expire with a forecast clock.
       It is replaced by a reachable better topology or reset with the mission.
       There is no separate staging dogleg: Entry/MM304 owns lateral energy
       management, and MM305 publishes one acquisition target. */
    double staging_along=g->taem_interface_target.valid?
        g->taem_interface_target.along_track:-s->final_approach_distance;
    double staging_cross=g->taem_interface_target.valid?
        g->taem_interface_target.cross_track:0.0;
    double staging_gap=staging_along-t->runway_along_track;
    /* Until a terminal path is frozen, vertical energy must remain an owned
       state rather than whatever descent angle the airframe happens to
       acquire while banking for staging geometry.  Use the same speed-scaled
       path-slope envelope as the HAC energy solver, and aim at the TAEM shell
       over the *remaining two-dimensional staging path* (along-track plus the
       lateral reserve needed for the dogleg).  This gives roughly 8-12 deg
       while the shuttle is still hot and naturally steepens toward the
       18-ish degree cool terminal corridor as speed falls.

       The previous fallback simply copied the measured FPA into the
       reference.  Once lift fell during the dogleg, -20/-30 deg became the
       new target every frame, so TAEM preserved crossrange while spending all
       of its altitude and could never make an energy-degraded HAC nominal. */
    double minimum_staging_slope=0.0,maximum_staging_slope=0.0;
    terminal_path_slope_bounds(t,v,s,&minimum_staging_slope,&maximum_staging_slope);
    double staging_cross_gap=staging_cross-t->runway_cross_track;
    double staging_vertical_path=hypot(fmax(0.0,staging_gap),staging_cross_gap);
    double response_distance=t->horizontal_speed*
        fmax(0.0,authority.control_response_time_s);
    double staging_height=fmax(0.0,t->mean_altitude-s->taem_interface_altitude);
    double staging_geometric_slope=atan2(staging_height,
        fmax(DBL_MIN,fmax(staging_vertical_path,response_distance)))*RAD2DEG;
    double staging_reference_fpa=-clampd(staging_geometric_slope,
        minimum_staging_slope,maximum_staging_slope);
    /* TAEM energy is controlled by drag work, not by trying to cancel drag
       with a steeper FPA.  The latter failed in the live replay because diving
       sooner moved the shuttle into denser air, which increased D/m faster
       than gravity could replenish speed.  Use the integrated atmosphere/
       drag model to choose the vertical path and reserve a per-metre drag
       budget all the way to preflare.  AoA is clamped to this budget below. */
    double staging_energy_fpa=staging_reference_fpa;
    bool staging_energy_priority=false;
    double staging_allowed_drag=INFINITY;
    if(g->terminal_region_entered||g->terminal_glide_mode){
        double actual_gate_air_path=terminal_uncommitted_gate_geometry(g,t,cfg,
            &staging_energy_fpa);
        if(fabs(staging_energy_fpa-staging_reference_fpa)>1.0)
            staging_energy_priority=true;
        /* The integrated solve owns the vertical reference once terminal
           guidance owns the vehicle.  This can be either shallower or steeper
           than the staging-shell geometry depending on actual energy. */
        staging_reference_fpa=staging_energy_fpa;

        double gate_altitude=cfg->site.altitude+fmax(0.0,g->terminal_test_preflare_altitude);
        double gate_speed=fmax(g->terminal_test_preflare_target_speed,
            g->terminal_test_preflare_min_speed);
        double available=entry_remaining_specific_energy(t->latitude,t->mean_altitude,
            t->true_air_speed,cfg->site.latitude,gate_altitude,gate_speed,p);
        double reserve=.5*pow(fmax(8.0,.08*gate_speed),2);
        double budget_energy=fmax(0.0,available-reserve);
        double budget_path=actual_gate_air_path;
        staging_allowed_drag=budget_energy/budget_path;
        /* Keep a model-error reserve.  A low-confidence/rapidly changing drag
           history should preserve more kinetic energy, never less. */
        double confidence=clampd(t->trajectory_calibration_confidence,0.0,1.0);
        staging_allowed_drag*=.82+.10*confidence;
    }
    /* Once the uncommitted vehicle reaches the upstream staging station, its
       remaining path is no longer the few kilometres of along-runway closure.
       It must spend energy laterally until a flyable HAC exists.  Use that
       crossrange path in the vertical schedule as well, otherwise the old
       4 km ground-distance floor commands a needless 30 deg dive while the
       dogleg is trying to preserve maneuver room. */
    if(!g->terminal_prediction_valid&&
       (!g->terminal_region_entered||!isfinite(g->terminal_reference_fpa)||
        fabs(g->terminal_reference_fpa)<=DBL_EPSILON))
        g->terminal_reference_fpa=t->flight_path_angle;
    g->terminal_prediction_valid=true;
    /* The predictive slope can move promptly, but no longer has to slam to a
       -35 deg gravity-recovery dive because AoA/drag is now the primary energy
       actuator. */
    double max_change=fmax(0,dt)*(staging_energy_priority?1.4:.7);
    /* The turn candidate can need more response time than a wings-level
       planning probe. Its own publication window is authoritative. Include
       the selection debounce and the two-second planning refresh so a new
       candidate cannot start with an already impossible lead deadline. */
    ControlAuthorityEnvelope planning_authority=
        decision_control_authority_envelope(g,t,p,cfg);
    double candidate_response=hac_response_lead_time(t,s,0.0);
    if(g->terminal_candidate.valid&&isfinite(g->terminal_candidate.response))
        candidate_response=fmax(candidate_response,g->terminal_candidate.response);
    if(planning_authority.valid)
        candidate_response=fmax(candidate_response,
            planning_authority.control_response_time_s);
    if(!(candidate_response>=0.0)||!isfinite(candidate_response))
        candidate_response=0.0;

    double horizontal=fmax(t->horizontal_speed,DBL_MIN);
    double staging_eta=staging_gap>0.0?staging_gap/horizontal:0.0;
    double horizon=fmax(candidate_response,staging_eta);
    double prediction_refresh=terminal_prediction_refresh_period(g,s);

    /* Keep the acquisition boundary observable.  A zero HAC result used to
       look identical whether the predictor was disabled, every future origin
       was unreachable, the candidate builder rejected the state, or the
       pre-handoff timing contract rejected an otherwise regular path.  This
       is diagnostic state only; it does not affect ranking or admission. */
    int acquisition_samples=0;
    int acquisition_reachability_rejects=0;
    int acquisition_builder_rejects=0;
    int acquisition_handoff_timing_rejects=0;

    if(!g->terminal_planning_deferred&&
       t->ut-g->terminal_prediction_ut>=prediction_refresh){
        g->terminal_prediction_ut=t->ut;

        /*
         * Bound the search horizon by actual remaining ground time.  Speed-loss
         * time is another physically meaningful event horizon, not a weighting:
         * it marks when the current aerodynamic state reaches the configured
         * TAEM/minimum-safe speed domain.
         */
        double geometric_limit=fmax(0.0,t->range_to_site)/horizontal;
        double terminal_speed=fmax(v->minimum_safe_speed,
            v->final_approach_speed);
        double speed_loss=terminal_expected_speed_loss_accel(g,t,p,aero,v);
        double speed_eta=INFINITY;
        if(t->true_air_speed<=terminal_speed) speed_eta=0.0;
        else if(speed_loss>DBL_MIN)
            speed_eta=(t->true_air_speed-terminal_speed)/speed_loss;

        double end_horizon=fmax(horizon,candidate_response);
        if(isfinite(speed_eta))
            end_horizon=fmax(end_horizon,fmin(speed_eta,geometric_limit));
        else
            end_horizon=fmax(end_horizon,geometric_limit);
        if(g->terminal_candidate.valid&&
           isfinite(g->terminal_candidate.arrival_ut))
            end_horizon=fmax(end_horizon,
                fmax(candidate_response,
                    g->terminal_candidate.arrival_ut-t->ut));
        if(geometric_limit>=horizon)
            end_horizon=fmin(end_horizon,geometric_limit);
        end_horizon=fmax(end_horizon,horizon);

        /*
         * Before MM305 owns the orbiter, estimate ownership time from the
         * fixed MM304 handoff target itself.  This uses bounded-curvature
         * capture and measured control authority; it does not infer handoff
         * from a speed threshold.
         */
        double handoff_eta=0.0;
        bool handoff_time_known=g->terminal_region_entered;
        if(!g->terminal_region_entered&&g->taem_interface_target.valid){
            TargetCaptureEnvelope handoff=decision_target_capture_envelope(
                g,t,course,g->taem_interface_target.along_track,
                g->taem_interface_target.cross_track,
                g->taem_interface_target.course,
                fmax(geometric_limit,horizon),p,cfg);
            if(handoff.valid&&isfinite(handoff.required_time_s)){
                handoff_eta=handoff.required_time_s;
                handoff_time_known=true;
            }
        }

        TerminalCandidate best={0};
        Telemetry best_future={0};
        double best_horizon=0.0;
        const int samples=7; /* numerical search resolution only */
        for(int i=0;i<samples;i++){
            double f=(double)i/(double)(samples-1);
            double sample_horizon=horizon+(end_horizon-horizon)*f;
            if(staging_eta>=horizon&&staging_eta<=end_horizon&&i==samples/2)
                sample_horizon=staging_eta;

            Telemetry future;
            acquisition_samples++;
            double e=0.0,n=0.0,future_course=course;
            terminal_project_planning_state(g,t,course,p,aero,cfg,
                sample_horizon,&future,&e,&n,&future_course);

            /* Do not retain a future-origin candidate whose own bounded path
               time cannot reach that origin before the forecast sample. */
            if(!terminal_future_origin_reachable(g,t,course,&future,
                    future_course,sample_horizon,p,cfg))
            {
                acquisition_reachability_rejects++;
                continue;
            }

            TerminalCandidate next={0};
            if(!terminal_candidate_from_future(g,t,&future,future_course,
                    p,aero,cfg,&next))
            {
                acquisition_builder_rejects++;
                continue;
            }

            if(!g->terminal_region_entered){
                if(!handoff_time_known)continue;
                double post_handoff_lead=sample_horizon-handoff_eta;
                if(post_handoff_lead<next.response){
                    acquisition_handoff_timing_rejects++;
                    continue;
                }
            }

            bool next_ready=terminal_candidate_probe_commit_ready(g,&next,t,p,aero,cfg);
            bool best_ready=terminal_candidate_probe_commit_ready(g,&best,t,p,aero,cfg);
            bool next_geometry_ok=terminal_candidate_geometry_regular(&next);
            bool best_geometry_ok=terminal_candidate_geometry_regular(&best);
            bool choose_next=false;
            if(next_geometry_ok&&!best_geometry_ok)
                choose_next=true;
            else if(next_geometry_ok==best_geometry_ok)
                choose_next=(next_ready&&!best_ready)||
                    (next_ready==best_ready&&terminal_candidate_better(&best,&next));
            if(choose_next){
                best=next;
                best_future=future;
                best_horizon=sample_horizon;
            }
        }

        /* A future-origin route may be geometrically regular yet fail the
           current-state energy contract once its response lead is charged.
           In that case, prefer a complete live-origin HAC only when it passes
           the same candidate-specific energy gate.  This preserves the
           physical response lead and never promotes a merely shorter or
           degraded path. */
        if(!g->diagnostic_shadow&&getenv("KSP_LANDER_HAC_DIAGNOSTICS")&&
           strcmp(getenv("KSP_LANDER_HAC_DIAGNOSTICS"),"1")==0){
            fprintf(stderr,
                "TERMINAL_REAL_BEST ut=%.3f valid=%d kind=%d geom=%d energy=%d execEval=%d execMargin=%.3f arrival=%.3f violation=%.6f len=%.1f horizon=%.3f\n",
                t->ut,best.valid?1:0,(int)best.kind,best.geometry_degraded?1:0,
                best.energy_degraded?1:0,best.execution_evaluated?1:0,
                best.execution_margin,best.arrival_ut,
                best.join.violation_score,best.join.length,best_horizon);
        }
        if(g->terminal_region_entered){
            TerminalCandidate live_origin={0};
            bool have_live_origin=terminal_live_origin_hac_candidate(
                g,t,course,p,aero,cfg,&live_origin);
            bool live_energy_ready=have_live_origin&&
                !live_origin.geometry_degraded&&
                terminal_candidate_live_energy_ready(g,t,p,aero,cfg,&live_origin);
            bool best_energy_ready=best.valid&&
                terminal_candidate_live_energy_ready(g,t,p,aero,cfg,&best);
            bool live_geometry_ready=have_live_origin&&
                terminal_candidate_geometry_executable(&live_origin);
            /* After MM304, a future-origin candidate with an unqualified live
               energy margin is only a forecast.  Prefer a regular candidate
               anchored at the actual vehicle state so the first turn begins
               now; otherwise the preview keeps moving its lead point ahead of
               the vehicle and can spend the entire handoff reserve flying a
               tangent.  A future candidate still wins when it has already
               passed the same live energy gate. */
            if(live_geometry_ready&&terminal_candidate_better(&best,&live_origin)&&
               (!best.valid||best.geometry_degraded||!best_energy_ready)){
                best=live_origin;
                best_future=*t;
                best_horizon=0.0;
            }
            if(getenv("KSP_LANDER_HAC_DIAGNOSTICS")&&
               strcmp(getenv("KSP_LANDER_HAC_DIAGNOSTICS"),"1")==0&&
               have_live_origin){
                fprintf(stderr,
                    "terminal topology live-origin: UT %.1f geom=%d energy=%d degraded=%d ready=%d margin %.0f q %.2f len %.0f arc %.0f\n",
                    t->ut,live_origin.geometry_degraded,live_origin.energy_degraded,
                    live_origin.degraded,live_energy_ready,
                    g->terminal_candidate_live_energy_margin,
                    live_origin.quality_score,live_origin.join.length,
                    live_origin.join.arc_remaining);
            }
        }

        bool timing_stale=false;
        bool timing_refresh=false;
        if(!g->terminal_region_entered&&g->terminal_candidate.valid&&
           handoff_time_known){
            double retained_post_handoff_lead=
                g->terminal_candidate.arrival_ut-t->ut-handoff_eta;
            timing_stale=retained_post_handoff_lead<
                g->terminal_candidate.response;
            if(timing_stale&&best.valid){
                double best_post_handoff_lead=
                    best.arrival_ut-t->ut-handoff_eta;
                timing_refresh=best_post_handoff_lead>=best.response&&
                    terminal_candidate_constraint_order(
                        &g->terminal_candidate,&best)<=0;
            }
        }

        if(g->terminal_candidate.valid)
            terminal_candidate_execution_cost(&g->terminal_candidate,g,t,course,p,aero,cfg);
        bool best_commit_ready=terminal_candidate_probe_commit_ready(g,&best,t,p,aero,cfg);
        bool retained_commit_ready=terminal_candidate_probe_commit_ready(
            g,&g->terminal_candidate,t,p,aero,cfg);
        bool best_geometry_ok=terminal_candidate_geometry_regular(&best);
        bool retained_geometry_ok=terminal_candidate_geometry_regular(
            &g->terminal_candidate);
        bool replace=false;
        if(best.valid){
            if(!g->terminal_candidate.valid)
                replace=true;
            else if(best_geometry_ok&&!retained_geometry_ok)
                replace=true;
            else if(best_geometry_ok==retained_geometry_ok)
                replace=(best_commit_ready&&!retained_commit_ready)||timing_refresh||
                    terminal_candidate_refresh_allowed(g,
                        &g->terminal_candidate,&best);
        }

        if(!g->diagnostic_shadow&&getenv("KSP_LANDER_HAC_DIAGNOSTICS")&&
           strcmp(getenv("KSP_LANDER_HAC_DIAGNOSTICS"),"1")==0){
            fprintf(stderr,
                "TERMINAL_REAL_REFRESH ut=%.3f retainedValid=%d retainedKind=%d retainedGeom=%d retainedExec=%.3f bestValid=%d bestKind=%d bestGeom=%d bestExec=%.3f bestReady=%d retainedReady=%d replace=%d\n",
                t->ut,g->terminal_candidate.valid?1:0,(int)g->terminal_candidate.kind,
                g->terminal_candidate.geometry_degraded?1:0,
                g->terminal_candidate.execution_margin,
                best.valid?1:0,(int)best.kind,best.geometry_degraded?1:0,
                best.execution_margin,best_commit_ready?1:0,
                retained_commit_ready?1:0,replace?1:0);
        }
        if(replace){
            if(!g->diagnostic_shadow){
                fprintf(stderr,"HAC selected (post-MM304 interface; +1=south -1=north): UT %.2f side %+.0f R %.3f lead %.3f join %.3f arc %.3f execution %.3fm %.3fs courseError %.3fdeg radialClosure %+.3fm/s (%s) margin %.3fs horizon %.3fs degraded=%d reason=%s priorSide=%+.0f\n",
                    t->ut,best.side,best.radius,best.join.lead_length,best.join.length,
                    best.join.arc_remaining,best.execution_distance,best.execution_time,
                    best.execution_course_error,best.execution_radial_closure,
                    best.execution_radial_closure>=0.0?"toward":"away",
                    best.execution_margin,best.execution_horizon,best.degraded,
                    !g->terminal_candidate.valid?"initial":
                    terminal_candidate_constraint_order(&g->terminal_candidate,&best)<0?
                    "physical feasibility improvement":"acquisition saving exceeds response cost",
                    g->terminal_candidate.valid?g->terminal_candidate.side:0.0);
            }
            g->terminal_candidate=best;
            /*
             * Before MM305 ownership the terminal planner is advisory only.
             * MM304's fixed mission handoff target remains authoritative.
             */
            if(g->terminal_region_entered){
                if(best.join.valid&&hac_bezier_regular(&best.join))
                    terminal_publish_interface_target(g,t,p,aero,cfg);
                else
                    g->taem_interface_target.valid=false;
            }
            g->terminal_prediction_altitude=best_future.mean_altitude;
            g->terminal_prediction_speed=best_future.true_air_speed;
            g->terminal_prediction_time=best_horizon;
        }else if(g->terminal_candidate.valid){
            double retained_horizon=fmax(candidate_response,
                g->terminal_candidate.arrival_ut-t->ut);
            Telemetry future;
            double e=0.0,n=0.0,future_course=course;
            terminal_project_planning_state(g,t,course,p,aero,cfg,
                retained_horizon,&future,&e,&n,&future_course);
            g->terminal_prediction_altitude=future.mean_altitude;
            g->terminal_prediction_speed=future.true_air_speed;
            g->terminal_prediction_time=retained_horizon;
        }else{
            Telemetry future;
            double e=0.0,n=0.0,future_course=course;
            terminal_project_planning_state(g,t,course,p,aero,cfg,
                horizon,&future,&e,&n,&future_course);
            g->terminal_prediction_altitude=future.mean_altitude;
            g->terminal_prediction_speed=future.true_air_speed;
        g->terminal_prediction_time=horizon;
        }

        if(!g->diagnostic_shadow&&getenv("KSP_LANDER_HAC_DIAGNOSTICS")&&
           strcmp(getenv("KSP_LANDER_HAC_DIAGNOSTICS"),"1")==0){
            fprintf(stderr,
                "HAC_ACQUISITION_WINDOW UT %.3f allowed=1 region=%d interface=%d handoffKnown=%d samples=%d reachabilityRejects=%d builderRejects=%d handoffTimingRejects=%d best=%d retained=%d committed=%d candidateKind=%d candidateValid=%d geomDegraded=%d energyDegraded=%d arrival=%.3f predictionTime=%.3f\n",
                t->ut,g->terminal_region_entered?1:0,
                g->taem_interface_target.valid?1:0,handoff_time_known?1:0,
                acquisition_samples,acquisition_reachability_rejects,
                acquisition_builder_rejects,acquisition_handoff_timing_rejects,
                best.valid?1:0,g->terminal_candidate.valid?1:0,
                g->terminal_path_committed?1:0,
                g->terminal_candidate.kind,g->terminal_candidate.valid?1:0,
                g->terminal_candidate.geometry_degraded?1:0,
                g->terminal_candidate.energy_degraded?1:0,
                g->terminal_candidate.valid?g->terminal_candidate.arrival_ut:NAN,
                g->terminal_prediction_time);
        }
    }
    bool candidate_operationally_usable=g->terminal_candidate.valid&&
        terminal_candidate_operationally_usable(g,&g->terminal_candidate,t,course,p,cfg);
    bool candidate_energy_ready=candidate_operationally_usable&&
        terminal_candidate_live_energy_ready(g,t,p,aero,cfg,&g->terminal_candidate);
    bool candidate_vertical_ready=candidate_operationally_usable&&
        terminal_candidate_vertical_response_ready_live(g,t,course,p,aero,cfg,
            &g->terminal_candidate);
    /* A geometry-clean path may be preview-tracked while its local vertical
       response catches up.  Requiring that response envelope before issuing
       the first path command creates a deadlock when the measured FPA is
       steeper than the candidate: the vehicle must track the candidate to
       become vertically ready.  Vertical response and energy still authorize
       commitment; geometry remains the hard prerequisite for any lateral
       preview. */
    bool operational_preview=candidate_operationally_usable&&
        (candidate_vertical_ready||
         (g->terminal_region_entered&&!g->terminal_candidate.geometry_degraded));
    /* Before a fully qualified join exists, there is no certified lateral path
       to fly.  Hold the measured ground course and wings level while the
       selector searches; converging toward a fixed station here would spend
       the handoff reserve on an uncommitted turn and can create the very
       cross-track debt the next candidate is meant to avoid. */
    double heading=operational_preview?
        ((g->terminal_region_entered||g->terminal_glide_mode)?
            terminal_uncommitted_alignment_heading(t,cfg):
            norm_deg(cfg->site.runway_heading+
                atan2(staging_cross_gap,staging_gap)*RAD2DEG)):
        norm_deg(course);
    double bank=operational_preview?
        clampd(norm_signed_deg(heading-course)*.65,-35,35):0.0;
    if(operational_preview){
        GuidanceMachine preview=*g;terminal_publish_candidate(&preview);
        HACGuidance path=hac_path_guidance(&preview,t,&cfg->site,s,p->radius,
            preview.hac_side,planet_surface_gravity(p),course,preview.hac_radius);
        g->terminal_reference_path_lateral_acceleration=path.lateral_acceleration;
        g->terminal_reference_path_bank=path.bank;
        g->terminal_reference_path_course_error=path.course_error;
        g->terminal_reference_path_arc_remaining=path.arc_remaining;
        g->terminal_reference_path_transition_active=preview.hac_transition_active;
        heading=path.heading;bank=taem_bank_demand(t,&path,preview.hac_radius,preview.hac_side,aero,v);
        GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
        GeoPoint point={t->latitude,t->longitude,t->mean_altitude};double e,n;
        local_offsets(origin,point,p->radius,&e,&n);
        double distance=hypot(e-g->terminal_candidate.join.lead_start.e,n-g->terminal_candidate.join.lead_start.n);
        double fpa=-atan2(t->mean_altitude-g->terminal_candidate.altitude,fmax(distance,500))*RAD2DEG;
        double kinetic_loss=(t->true_air_speed*t->true_air_speed-
            g->terminal_candidate.speed*g->terminal_candidate.speed)/(2*fmax(distance,500));
        double energy_fpa=asin(clampd((kinetic_loss-live_drag_accel(t,aero,v))/
            fmax(planet_surface_gravity(p),.1),-1,1))*RAD2DEG;
        fpa+=clampd((energy_fpa-fpa)*.25,-4,4);
        /* A path that is dynamically flyable but too long for the current
           energy state needs TAEM to preserve energy while geometry
           converges.  Never let such a preview pull the vehicle steeper than
           the staging glide law; otherwise the path budget shrinks at least
           as quickly as the geometry and the candidate can never become
           committable. */
        if(g->terminal_region_entered||g->terminal_glide_mode){
            /* This preview already passed live energy and control checks.
               Fly its planned slope so the actual state can converge to the
               exact FPA contract used at commit. The straight-to-runway slope
               commanded -35 degrees while a valid HAC required -18, making
               capture impossible even with sufficient altitude and energy. */
            fpa=-clampd(g->terminal_candidate.slope,5.0,35.0);
        }else if(g->terminal_candidate.energy_degraded)
            fpa=fmax(fpa,staging_reference_fpa);
        g->terminal_reference_fpa+=clampd(clampd(fpa,-35,-5)-g->terminal_reference_fpa,-max_change,max_change);
    }else g->terminal_reference_fpa+=clampd(staging_reference_fpa-g->terminal_reference_fpa,
        -max_change,max_change);

    /* If actual descent still runs materially steeper than the uncommitted
       energy corridor, trade some lateral bank for vertical lift.  This is a
       bounded staging priority only; once a nominal HAC is committed, the
       frozen path owns bank normally. */
    if(!g->terminal_path_committed&&
       (!operational_preview||g->terminal_candidate.energy_degraded)){
        double descent_excess=staging_reference_fpa-t->flight_path_angle;
        if(descent_excess>3.0){
            double vertical_priority=clampd(1.0-(descent_excess-3.0)/14.0,.35,1.0);
            bank*=vertical_priority;
        }
    }
    double requested_aoa=terminal_fpa_force_aoa(t,p,aero,v,
        g->terminal_reference_fpa);
    /* A nominal-geometry candidate whose live drag budget is not yet ready is
       a conservation problem, not a reason to keep demanding the lift needed
       for the old staging FPA ramp.  Bound incidence by the demonstrated
       terminal energy AoA until the live path budget catches up.  The
       candidate still cannot be committed without the full energy gate above. */
    if(g->terminal_region_entered&&g->terminal_candidate.valid&&
       !g->terminal_candidate.geometry_degraded&&!candidate_energy_ready)
        requested_aoa=fmin(requested_aoa,terminal_hac_energy_aoa(v));
    if(isfinite(staging_allowed_drag)){
        double budget_aoa=terminal_drag_budget_aoa(t,aero,v,staging_allowed_drag,
            requested_aoa);
        requested_aoa=terminal_preview_recovery_aoa(requested_aoa,budget_aoa,
            g->terminal_reference_fpa,t->flight_path_angle);
        /* When even the present incidence is spending energy faster than the
           remaining path permits, lateral geometry is secondary to staying
           airborne.  Ease bank continuously instead of waiting for a later
           low-speed/stall recovery to undo the damage. */
        double measured_drag=live_drag_accel(t,aero,v);
        if(staging_allowed_drag>.05&&measured_drag>staging_allowed_drag*1.05){
            double ratio=measured_drag/fmax(staging_allowed_drag,.05);
            bank*=clampd(1.0-(ratio-1.0)*.18,.55,1.0);
        }
    }
    g->terminal_reference_heading=heading;
    double reference_bank=clampd(bank,-40,40);
    /* Candidate publication can replace a future-origin join with a different
       path tangent on the next sample.  A raw reference jump from one side of
       the vehicle to the other is not a valid preview: the native attitude
       plant can only change bank at its measured roll-rate limit.  Apply that
       same response envelope to the high-level reference so selector updates
       cannot manufacture an S-turn reversal that the airframe cannot follow.
       This is a command-bandwidth contract, not a trajectory preference. */
    if(g->terminal_region_entered&&dt>0.0&&isfinite(g->terminal_reference_bank)){
        double reference_rate=t->attitude_response.roll_valid&&
            isfinite(t->attitude_response.maximum_roll_rate_deg_s)?
            fabs(t->attitude_response.maximum_roll_rate_deg_s):
            fmax(1.0,s->taem_roll_rate);
        reference_bank=g->terminal_reference_bank+
            clampd(reference_bank-g->terminal_reference_bank,
                -reference_rate*dt,reference_rate*dt);
    }
    g->terminal_reference_bank=reference_bank;
    g->terminal_reference_aoa=clampd(requested_aoa,0.0,20.0);
    double target_mix=g->terminal_region_entered&&operational_preview?1.0:0.0;
    double convergence_time=fmax(candidate_response,DBL_MIN);
    double mix_alpha=clampd(fmax(0.0,dt)/convergence_time,0.0,1.0);
    g->terminal_mix+=(target_mix-g->terminal_mix)*mix_alpha;
}


/* Native planner worker entry point. This mutates only its private snapshot;
   execution ownership, command filters and control output remain on the live
   control thread. Offline shadows execute the same search synchronously. */
bool guidance_plan_terminal_preview(GuidanceMachine*g,const Telemetry*t,const PlanetModel*p,
        AerodynamicModel aero,const LandingConfiguration*cfg){
    if(!g||!t||!p||!cfg||!guidance_terminal_preview_allowed(g))return false;
    double previous=g->terminal_prediction_ut;
    g->terminal_planning_deferred=false;
    double course=isfinite(t->ground_track_heading)?t->ground_track_heading:t->heading;
    terminal_predict(g,t,course,p,aero,cfg,0.0);
    return isfinite(g->terminal_prediction_ut)&&
        (!isfinite(previous)||g->terminal_prediction_ut>previous);
}

bool guidance_accept_terminal_preview(GuidanceMachine*g,const GuidanceMachine*request,
        const GuidanceMachine*result,const Telemetry*t,const LandingConfiguration*cfg){
    if(!g||!request||!result||!t||!cfg||!guidance_terminal_preview_allowed(g)||
       !g->automation_engaged||g->paused||g->aborted||
       g->attitude_recovery||g->phase==PHASE_ATTITUDE_RECOVERY||g->terminal_path_committed||
       g->final_approach_captured||(g->hac_side_selected&&!g->terminal_test_capture_active)||
       g->phase!=request->phase||g->terminal_region_entered!=request->terminal_region_entered||
       g->terminal_reentry_after_ut!=request->terminal_reentry_after_ut||
       g->s_turn_sign!=request->s_turn_sign)return false;
    double age=t->ut-result->terminal_prediction_ut;
    if(!isfinite(age)||age<-.001||age>fmax(3.0,cfg->guidance.prediction_interval*2.0)||
       result->terminal_prediction_ut<=g->terminal_prediction_ut)return false;
    /* Never resurrect an intercept that the live vehicle has already passed.
       Candidate validity is not authority to commit: all existing live geometry,
       energy, response and capture gates remain in terminal_guidance(). */
    bool candidate_is_future=result->terminal_candidate.valid&&
        isfinite(result->terminal_candidate.arrival_ut)&&
        result->terminal_candidate.arrival_ut>t->ut;
    /* An asynchronous preview is advisory, but a stale propagated origin is
       still dangerous: copying it into the retained preview lets hysteresis
       preserve a path whose acquisition point is already behind the live
       vehicle.  Accept only a genuinely future candidate.  Prediction timing
       may still advance without replacing the retained path. */
    if(result->terminal_candidate.valid&&!candidate_is_future){
        const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
        if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow)
            fprintf(stderr,
                "HAC_PREVIEW_REJECT stale-origin liveUt=%.3f resultUt=%.3f arrival=%.3f selected=%.3f\n",
                t->ut,result->terminal_prediction_ut,
                result->terminal_candidate.arrival_ut,
                result->terminal_candidate.selected_ut);
        /* The newer prediction is still publishable, but its stale path is
           not.  Clear the retained candidate so preview hysteresis cannot
           keep flying an origin that has already been passed. */
        g->terminal_candidate.valid=false;
        g->terminal_prediction_valid=false;
    }
    if(candidate_is_future){
        g->terminal_candidate=result->terminal_candidate;
        if(g->terminal_region_entered)g->taem_interface_target=result->taem_interface_target;
    }
    g->terminal_prediction_ut=result->terminal_prediction_ut;
    g->terminal_prediction_altitude=result->terminal_prediction_altitude;
    g->terminal_prediction_speed=result->terminal_prediction_speed;
    g->terminal_prediction_time=result->terminal_prediction_time;
    return true;
}

