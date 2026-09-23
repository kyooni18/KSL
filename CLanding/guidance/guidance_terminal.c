#include "guidance_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Translation-unit private helpers. */
static GuidanceResult terminal_final_test_guidance(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double dt);
static bool fixed_hac_lead_capture_update(GuidanceMachine *g,const Telemetry *t,
        double course,double radius,double e,double n,const HACTransitionPlan *lead);
static GuidanceResult terminal_guidance(GuidanceMachine *g, const Telemetry *t, const VehicleState *state, double course,
        const PlanetModel *p, AerodynamicModel aero, const LandingConfiguration *cfg, double dt);

static GuidanceResult terminal_final_test_guidance(GuidanceMachine*g,const Telemetry*t,double course,
        const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg,double dt){
    const GuidanceSettings*s=&cfg->guidance;
    const VehicleProfile*v=&cfg->vehicle;
    TerminalPreflarePlan approach_plan={0};
    bool approach=terminal_outer_gate(g,t,course,p,aero,cfg,&approach_plan);
    TaemTerminalContract contract=terminal_delivery_contract(g,t,course,p,cfg,&approach_plan);
    TaemTerminalEvaluation evaluation=taem_exec_evaluate_terminal_contract(&contract);
    if(!approach||!evaluation.valid||!evaluation.feasible){
        EnergyPathEnvelope conditioning_energy={0};
        double projected_end_speed=NAN,conditioning_slope=NAN;
        if(!terminal_final_conditioning_reachable(g,t,course,p,aero,cfg,
                &conditioning_energy,&projected_end_speed,&conditioning_slope)){
            char reason[320];
            snprintf(reason,sizeof(reason),
                "Post-HAC final alignment exhausted its physical conditioning envelope: approach=%d preflare=%s terminal=%s.",
                approach?1:0,approach_plan.reject_reason?approach_plan.reject_reason:"none",
                taem_terminal_block_reason_string(evaluation.block_reason));
            return terminal_abort(g,reason);
        }
        Trajectory conditioning_ref;trajectory_init(&conditioning_ref);
        GuidanceResult result=final_guidance(g,t,course,p,cfg,&conditioning_ref,dt);
        trajectory_clear(&conditioning_ref);
        result.phase=PHASE_TAEM;
        g->phase=PHASE_TAEM;
        snprintf(result.status,sizeof(result.status),
            "Post-HAC final alignment energy conditioning: V %.1f m/s, projected %.1f m/s at flare boundary, minimum %.1f m/s, slope %.1f deg, energy margin %+.0f J/kg.",
            t->true_air_speed,projected_end_speed,v->minimum_safe_speed,conditioning_slope,
            conditioning_energy.energy.margin);
        return result;
    }
    (void)taem_exec_sync_with_contract(g,t,s,p,aero,cfg,&contract,false,false);
    evaluation=g->taem_exec.terminal_evaluation;
    if(!g->final_approach_captured&&t->runway_along_track<0.0&&
       g->taem_exec.phase==TAEM_PHASE_FINAL_INTERCEPT){
        (void)taem_exec_sync_with_contract(g,t,s,p,aero,cfg,&contract,true,true);
        if(g->taem_exec.taem_complete){
            g->final_approach_captured=true;
            terminal_store_preflare_plan(g,&approach_plan);
            terminal_set_stage(g,TERMINAL_TRAJECTORY_CAPTURE,t->ut);
            robust_pid_reset(&g->final_altitude_pid);
            robust_pid_reset(&g->speed_pid);
            robust_pid_reset(&g->flare_sink_pid);
        }
    }
    Trajectory ref;
    trajectory_init(&ref);
    GuidanceResult result;
    if(g->final_approach_captured&&g->taem_exec.taem_complete){
        result=terminal_approach_sequence(g,t,course,p,aero,cfg,&ref,dt);
    }else{
        HACGuidance line=terminal_runway_line_path_guidance(t,&cfg->site,s,planet_surface_gravity(p),course);
        GuidanceCommand command=atmospheric(t,line.heading,line.bank,v,0.0,true,PROFILE_TAEM);
        command.heading_control_enabled=true;
        g->phase=PHASE_TAEM;
        result=stabilized(g,result_make(PHASE_TAEM,command,
            "Post-HAC final alignment: runway-line capture and TAEM final-intercept admission.",
            NULL),t,v,s,dt);
    }
    trajectory_clear(&ref);
    return result;
}
static bool fixed_hac_lead_capture_update(GuidanceMachine *g,const Telemetry *t,
        double course,double radius,double e,double n,const HACTransitionPlan *lead){
    HACPoint2 chord={lead->p3.e-lead->p0.e,lead->p3.n-lead->p0.n};
    double chord_len=fmax(hypot(chord.e,chord.n),1.0);
    double projected=((e-lead->p0.e)*chord.e+(n-lead->p0.n)*chord.n)/
        (chord_len*chord_len);
    double nearest=g->hac_transition_lead_curve?
        hac_bezier_nearest_u(lead,e,n):clampd(projected,0.0,1.0);
    double endpoint=hypot(e-lead->p3.e,n-lead->p3.n);
    HACPoint2 end_dir=g->hac_transition_lead_curve?
        hac_bezier_derivative(lead,1.0):chord;
    double end_norm=hypot(end_dir.e,end_dir.n);
    double circle_radial=fabs(hypot(e-g->hac_transition_cone_center_e,
        n-g->hac_transition_cone_center_n)-radius);
    double end_course=end_norm>1e-6?
        norm_deg(atan2(end_dir.e,end_dir.n)*RAD2DEG):NAN;
    bool radial_valid=circle_radial<=fmax(250.0,radius*.05);
    bool endpoint_valid=endpoint<=fmax(260.0,t->true_air_speed*2.8);
    bool tangent_valid=isfinite(end_course)&&
        fabs(norm_signed_deg(course-end_course))<=22.0;
    bool capture_valid=radial_valid&&endpoint_valid&&tangent_valid;

    /* Nearest-point projection is a tracking scalar, not proof that the lead was
       traversed.  A point beyond p3 can otherwise project to u=1 and expose the
       analytic circle controller while the vehicle is still kilometers off-circle. */
    double measured=capture_valid?nearest:fmin(nearest,.994);
    if(measured>g->hac_transition_lead_progress)
        g->hac_transition_lead_progress=measured;
    if(capture_valid)g->hac_transition_lead_progress=1.0;
    else if(g->hac_transition_lead_progress>=.995)
        g->hac_transition_lead_progress=.994;
    return capture_valid;
}

static GuidanceResult terminal_guidance(GuidanceMachine *g, const Telemetry *t, const VehicleState *state, double course,
        const PlanetModel *p, AerodynamicModel aero, const LandingConfiguration *cfg, double dt) {
    const GuidanceSettings *s = &cfg->guidance;
    const VehicleProfile *v = &cfg->vehicle;
    if(g->terminal_final_test_mode){
        /* terminal_glide_mode is only the outer guidance_update router.  Clear it
           while executing this test so every final gate, target and actuator
           limit is exactly the production path, not the HAC-rehearsal path. */
        g->terminal_glide_mode=false;
        GuidanceResult final_test=terminal_final_test_guidance(g,t,course,p,aero,cfg,dt);
        g->terminal_glide_mode=true;
        return final_test;
    }
    /* Ground rollout owns wheel control; airborne flare limits no longer apply.
       A generic KSP `landed` state is not runway success: reject a grass/terrain
       touchdown rather than allowing the rollout state to count it as complete. */
    bool reported_landed=!strcasecmp(t->vessel_situation,"landed");
    if (g->final_approach_captured &&
        (g->phase==PHASE_ROLLOUT || g->phase==PHASE_COMPLETE || reported_landed)) {
        if(reported_landed&&!terminal_runway_contact_position(t,cfg))
            return terminal_abort(g,"Touchdown occurred outside the runway 09 contact envelope.");
        if(reported_landed&&!t->gear)
            return terminal_abort(g,"Touchdown occurred without confirmed landing-gear deployment.");
        if(reported_landed){
            g->ground_contact_latched=true;g->gear_command_latched=true;
            terminal_set_stage(g,TERMINAL_GROUND,t->ut);
        }
        Trajectory ref; trajectory_init(&ref);
        GuidanceResult r=touchdown(g,t,cfg,&ref); trajectory_clear(&ref); return r;
    }
    bool recovery_was_active=g->attitude_recovery;
    if (control_recovery_needed(g, t, s, v, dt)) {
        if(!recovery_was_active&&(g->terminal_path_committed||g->hac_side_selected))
            terminal_invalidate_frozen_path(g);
        if (g->recovery_duration >= 15)
            return terminal_abort(g, "Control departure: attitude recovery exceeded 15 s without stabilizing.");
        if (t->radar_altitude < fmax(300, -t->vertical_speed * 8))
            return terminal_abort(g, "Control departure: insufficient height remains to complete attitude recovery.");
        g->phase = PHASE_ATTITUDE_RECOVERY;
        GuidanceCommand c = atmospheric(t, g->recovery_heading, 0, v, 0, false, PROFILE_RECOVERY);
        /* Recovery is an unload/level maneuver, not a heading-capture
           maneuver.  At useful dynamic pressure, rudder/RCS heading pursuit
           can couple directly into roll.  The failed 50 km replay held nearly
           full yaw while trying to preserve the pre-upset ground track and
           drove beta beyond 50 degrees.  Let roll/pitch stabilize the airframe
           first; ordinary entry guidance will reacquire course after release. */
        c.heading_control_enabled = false;
        double recovery_aoa = g->recovery_aoa>0?g->recovery_aoa:
            clampd(v->entry_angle_of_attack, 10, 18);
        c.has_target_aoa = true;
        c.target_aoa = recovery_aoa;
        c.target_pitch = clampd(t->flight_path_angle + recovery_aoa, -12, 25);
        return stabilized(g, result_make(g->phase, c, "Attitude recovery: unloading and leveling wings; terminal progression inhibited.",
            "Sustained saturation/attitude error or excessive roll rate."), t, v, s, dt);
    }
    if(recovery_was_active&&!g->attitude_recovery){
        /* A successful recovery has changed position, energy and usually bank
           enough that the old terminal join is stale.  With adequate margin,
           hand authority back to Entry/S-turn for a clean replan; otherwise
           remain in terminal mode but require a brand-new HAC candidate. */
        if(terminal_entry_recovery_available(g,t,v,s))
            return terminal_return_to_entry(g,t,state,course,p,aero,cfg,dt);
        terminal_invalidate_frozen_path(g);
    }
    terminal_energy_observe(g,t,p);
    if(g->terminal_test_upstream_staging&&hac_variant_b_requested()){
        bool upstream_ready=false;
        GuidanceResult staging=terminal_variant_b_upstream_guidance(g,t,course,p,
            aero,cfg,dt,&upstream_ready);
        if(upstream_ready&&!g->aborted){
            /* The helper marked the exact p0 window.  Keep that marker only
               for this one admission call; the committed plan starts at the
               measured capture state and excludes all earlier staging. */
            g->terminal_test_upstream_capture_ready=true;
            bool fixed_hac_acquired=terminal_force_acquisition_with_aero(
                g,t,course,p,aero,cfg);
            g->terminal_test_upstream_capture_ready=false;
            if(fixed_hac_acquired){
                g->terminal_test_upstream_staging=false;
                guidance_result_clear(&staging);
            }else if(g->terminal_test_upstream_energy_rejected&&!g->aborted){
                g->terminal_test_upstream_staging=false;
                return terminal_abort(g,
                    "MM305 dynamic HAC admission rejected by energy-radius budget; no safe committed path remains.");
            }else if(!g->aborted){
                /* The unchanged authority gate rejected this state.  Keep
                   staging until the one-way p0 miss predicate fires; never
                   replace it with a generic HAC/spline candidate. */
                return staging;
            }
        }
        if(g->terminal_test_upstream_staging)
            return staging;
    }
    double debug_capture_course=NAN;
    bool debug_fixture_state=terminal_hac_debug_fixture_state(t,course,cfg,
        &debug_capture_course);
    bool explicit_mm305_fixture=hac_mm305_dynamic_fixture_requested();
    bool fixed_fixture_mode=!g->fixed_alignment_hac_latched&&
        (g->terminal_fixed_hac_fixture_latched||
         (g->terminal_test_capture_active&&g->terminal_rehearsal_mode&&
          (explicit_mm305_fixture||debug_fixture_state)));
    if(fixed_fixture_mode){
        g->terminal_fixed_hac_fixture_latched=true;
        bool fixed_hac_acquired=terminal_force_acquisition_with_aero(
            g,t,course,p,aero,cfg);
        if(!fixed_hac_acquired){
            /* This fixture is an authority-gate diagnostic.  Do not let the
               generic terminal planner replace the rejected dynamic circle
               with a spline or an unqualified resized HAC. */
            double final_station=g->terminal_test_final_approach_distance>0.0?
                g->terminal_test_final_approach_distance:s->final_approach_distance;
            if(terminal_capture_margin_exhausted(g,t,course,p,aero,cfg)||
               t->runway_along_track>=-final_station)
                return terminal_abort(g,
                    "MM305 fixed-HAC admission found no lead, authority, and energy qualified circle before the runway maneuver margin was exhausted.");
            g->terminal_candidate.valid=false;
            g->terminal_prediction_valid=false;
            g->terminal_path_kind=TERMINAL_PATH_NONE;
            g->terminal_path_committed=false;
            g->hac_side_selected=false;
            g->hac_transition_active=false;
            g->hac_transition_heading_cone=false;
            g->hac_transition_lead_curve=false;
            g->hac_remaining=0.0;
            GuidanceCommand hold=atmospheric(t,course,0.0,v,0.0,
                false,PROFILE_TAEM);
            hold.heading_control_enabled=false;
            hold.has_target_aoa=true;
            hold.target_aoa=clampd(t->angle_of_attack,0.0,
                v->maximum_angle_of_attack);
            hold.target_pitch=t->flight_path_angle+hold.target_aoa;
            return stabilized(g,result_make(PHASE_TAEM,hold,
                "MM305 fixed-HAC admission has no qualified lead, authority, and energy tuple; holding the current tangent.",
                "The runway-anchored HAC candidate remains uncommitted; no fallback spline or unqualified resize is permitted."),
                t,v,s,dt);
        }
    }
    bool fixed_alignment_hac_mode=g->fixed_alignment_hac_latched&&!g->final_approach_captured;
    bool active_committed_heading_cone=g->terminal_path_committed&&
        g->terminal_path_kind==TERMINAL_PATH_HAC&&
        g->hac_transition_heading_cone&&g->hac_transition_active;
    if(!fixed_alignment_hac_mode&&!active_committed_heading_cone&&g->terminal_region_entered&&
       entry_taem_tangent_target_geometry(&g->taem_interface_target,cfg)&&
       !g->final_approach_captured){
        TaemHandoffContract reacq_contract=taem_handoff_contract(&cfg->guidance);
        double reacq_position_error=hypot(
            t->runway_along_track-g->taem_interface_target.along_track,
            t->runway_cross_track-g->taem_interface_target.cross_track);
        double reacq_course_error=fabs(norm_signed_deg(
            course-g->taem_interface_target.course));
        bool in_fixed_alignment_tube=isfinite(reacq_position_error)&&
            reacq_position_error<=fmax(reacq_contract.horizontal_radius_m,
                fmax(250.0,t->true_air_speed*.75))&&
            reacq_course_error<=fmax(reacq_contract.perpendicular_heading_half_width_deg,8.0);
        if(in_fixed_alignment_tube){
            terminal_force_acquisition_with_aero(g,t,course,p,aero,cfg);
            active_committed_heading_cone=g->terminal_path_committed&&
                g->terminal_path_kind==TERMINAL_PATH_HAC&&
                g->hac_transition_heading_cone&&g->hac_transition_active;
        }
    }
    if(fixed_alignment_hac_mode||active_committed_heading_cone)
        g->terminal_test_capture_active=false;
    else
        terminal_predict(g,t,course,p,aero,cfg,dt);

    /* MM304 -> MM305 ownership is evaluated only on the current strict
       interface contract below.  No speed/altitude/stability fallback may
       create terminal ownership independently of that contract. */
    /* The path provider runs first so the executive can decide with the
       current compact terminal candidate.  Then synchronize on every MM305
       frame, not only while already in S-turn: this is what lets a 500+ m/s
       Path-Acquisition start transition into physically propagated energy
       management before the capture-preview branch returns. */
    if(taem_exec_owns_vehicle(&g->taem_exec)){
        taem_exec_sync(g,t,s,p,aero,cfg);
        if(g->taem_exec.phase==TAEM_PHASE_S_TURN)
            return taem_s_turn_guidance(g,t,course,p,aero,cfg,dt);
    }
    if(g->terminal_glide_mode&&g->terminal_test_capture_active&&!fixed_alignment_hac_mode&&!active_committed_heading_cone){
        bool margin_exhausted=terminal_capture_margin_exhausted(g,t,course,p,aero,cfg);
        bool executable_preview=terminal_candidate_operationally_usable(
            g,&g->terminal_candidate,t,course,p,cfg)&&
            !g->terminal_candidate.geometry_degraded;
        bool vertical_ready=terminal_candidate_vertical_response_ready_live(
            g,t,course,p,aero,cfg,&g->terminal_candidate);
        bool planning_blocked=!terminal_candidate_operationally_usable(
            g,&g->terminal_candidate,t,course,p,cfg)||
            !vertical_ready;
        g->hac_capture_lost_duration=planning_blocked?g->hac_capture_lost_duration+dt:
            fmax(0.0,g->hac_capture_lost_duration-dt*2.0);
        double planning_grace=g->terminal_candidate.valid?
            fmax(8.0,g->terminal_candidate.response*1.5):10.0;
        /* A dynamically regular path may be tracked before its energy budget
           is nominal. Freeze it only after both geometry/control and energy
           convergence are qualified. */
        if(terminal_candidate_commit_ready(g,t,p,aero,cfg)){
            terminal_publish_candidate(g);
            g->terminal_path_committed=true;g->terminal_test_capture_active=false;
            g->terminal_energy_mismatch_duration=0.0;
            g->hac_commit_blend=0.0;
            /* Preserve all command limiter state across the commit. */
        }else{
            bool spline_candidate=g->terminal_candidate.valid&&
                g->terminal_candidate.kind==TERMINAL_PATH_SPLINE;
            if(!executable_preview&&!margin_exhausted&&
               g->hac_capture_lost_duration>=planning_grace&&
               terminal_entry_recovery_available(g,t,v,s))
                return terminal_return_to_entry(g,t,state,course,p,aero,cfg,dt);
            if(margin_exhausted){
                bool forced_spline=spline_candidate&&executable_preview&&
                    !g->terminal_candidate.geometry_degraded;
                if(forced_spline){
                    /* No fully certified topology remains, but this spline is a
                       regular continuous path inside the measured control/rate
                       envelope.  Per the MM305 fallback contract, freeze the
                       least-violating geometry rather than abandon guidance.
                       Energy/vertical deficits remain explicit degradation; the
                       normal minimum-speed, load, q and actuator limits are NOT
                       relaxed by this ownership decision. */
                    bool energy_ready=g->terminal_candidate_live_energy_valid&&
                        isfinite(terminal_energy_commit_tolerance)&&
                        g->terminal_candidate_live_energy_margin>=
                            -terminal_energy_commit_tolerance;
                    g->terminal_candidate.degraded=true;
                    if(!energy_ready)g->terminal_candidate.energy_degraded=true;
                    terminal_publish_candidate(g);
                    g->terminal_path_committed=true;
                    g->terminal_test_capture_active=false;
                    g->terminal_energy_mismatch_duration=0.0;
                    g->hac_commit_blend=0.0;
                    g->hac_plan_degraded=true;
                    if(!energy_ready)g->hac_plan_energy_degraded=true;
                    if(getenv("KSP_LANDER_HAC_DIAGNOSTICS")&&
                       strcmp(getenv("KSP_LANDER_HAC_DIAGNOSTICS"),"1")==0&&
                       !g->diagnostic_shadow)
                        fprintf(stderr,
                            "HAC_FORCED_SPLINE_COMMIT ut=%.3f energyReady=%d verticalReady=%d energyMargin=%.1f violation=%.6f len=%.1f peakLat=%.3f rateRatio=%.3f\n",
                            t->ut,energy_ready?1:0,vertical_ready?1:0,
                            g->terminal_candidate_live_energy_margin,
                            g->hac_plan_violation_score,
                            g->terminal_candidate.join.length,
                            g->terminal_candidate.join.peak_lateral,
                            g->terminal_candidate.join.peak_course_rate_ratio);
                }else{
                    const char*reason="Terminal planning exhausted maneuver margin without a current executable HAC or spline candidate.";
                    if(g->terminal_candidate.valid&&!g->terminal_candidate.geometry_degraded){
                        bool energy_ready=g->terminal_candidate_live_energy_valid&&
                            isfinite(terminal_energy_commit_tolerance)&&
                            g->terminal_candidate_live_energy_margin>=
                                -terminal_energy_commit_tolerance;
                        if(!vertical_ready&&!energy_ready)
                            reason="Terminal path geometry was flyable, but vertical response and energy margins did not converge before maneuver margin was exhausted.";
                        else if(!vertical_ready)
                            reason="Terminal path geometry was flyable, but the vertical response could not reach the flare gate before maneuver margin was exhausted.";
                        else if(!energy_ready)
                            reason="Terminal path geometry was flyable, but its energy/path budget did not converge before maneuver margin was exhausted.";
                        else
                            reason="Terminal path geometry was flyable, but its forecast origin could not be captured before maneuver margin was exhausted.";
                    }
                    return terminal_abort(g,reason);
                }
            }
            if(!executable_preview){
                GuidanceCommand c=atmospheric(t,g->terminal_reference_heading,g->terminal_reference_bank,v,0,false,PROFILE_TAEM);
                c.heading_control_enabled=false;c.has_target_aoa=true;c.target_aoa=g->terminal_reference_aoa;
                c.target_pitch=t->flight_path_angle+c.target_aoa;
                return stabilized(g,result_make(PHASE_TAEM,c,
                    g->terminal_candidate.valid?
                        (g->terminal_candidate.geometry_degraded?"TAEM converging on the best available terminal path.":
                         g->terminal_candidate.energy_degraded?(spline_candidate?
                            "TAEM tracking a direct spline while its energy/path budget converges.":
                            "TAEM tracking the selected HAC while its energy/path budget converges."):
                         g->terminal_candidate.shell_degraded?"TAEM converging on a qualified terminal path outside the preferred altitude shell.":
                         spline_candidate?"TAEM converging on the selected direct spline.":
                            "TAEM converging on the selected HAC join."):
                        "TAEM optimizing terminal path topology.",
                    g->terminal_candidate.geometry_degraded?
                        (planning_blocked?
                            "The current terminal candidate is still outside the bounded executable envelope; TAEM is searching for a better regular path.":
                            "A regular terminal candidate is retained while geometry converges."):
                        g->terminal_candidate.energy_degraded?
                            "The selected terminal curve is dynamically flyable, but its predicted energy/path budget is not nominal yet; commit remains inhibited while TAEM replans.":
                        NULL),t,v,s,dt);
            }
            /* A regular candidate is safe to preview even when its energy or
               vertical-response margin has not yet converged.  Fall through
               to the common path-preview law below so the vehicle can spend
               the physically modeled response lead moving onto that path.
               The commit gate still requires both margins; this branch only
               prevents a neutral tangent hold from creating cross-track debt. */
        }
    }
    if (!g->terminal_region_entered && !g->final_approach_captured) {
        g->phase = PHASE_ENTRY_ENERGY;
        GuidanceResult entry=entry_program_guidance(g,t,state,course,p,aero,cfg,dt);
        TaemInterfaceCapture strict_capture=entry_dynamic_interface_capture(g,t,course,p,aero,cfg);
        double debug_capture_course=NAN;
        bool diagnostic_handoff=terminal_hac_debug_force_requested()&&
            terminal_hac_debug_fixture_state(t,course,cfg,&debug_capture_course);
        (void)debug_capture_course;
        bool strict_handoff=(strict_capture.ready&&g->entry_exec.entry_complete)||
            diagnostic_handoff;
        g->taem_interface_captured=strict_handoff;
        if(strict_handoff){
            /* The strict fixed-point MM304 contract plus the Entry executive's
               explicit qualified-handoff event is the sole ownership transfer.
               Neither layer can manufacture MM305 ownership independently.  A
               dynamically selected HAC also needs measured lateral-force authority at
               this exact handoff state; if it is not executable, remain in
               MM304 and spend energy before retrying the one-way transfer. */
            /* Acquisition is transactional.  Several fixed-HAC admission
               checks occur after terminal state is initialized; a rejected
               candidate must not leak that partial MM305 state back into the
               MM304 controller that retains ownership. */
            GuidanceMachine acquisition_before=*g;
            bool fixed_hac_acquired=terminal_force_acquisition_with_aero(
                g,t,course,p,aero,cfg);
            if(!fixed_hac_acquired){
                *g=acquisition_before;
                g->taem_interface_captured=false;
                entry.has_warning=true;
                snprintf(entry.warning,sizeof(entry.warning),
                    "MM304 interface is geometrically qualified, but the selected HAC is not yet executable at measured lift/speed; retaining MM304 ownership.");
                return entry;
            }
            guidance_result_clear(&entry);
            /* If the newly latched TAEM plan begins with an energy-management
               S-turn, execute it on this same frame so ownership and commands are
               continuous across MM304 -> MM305. */
            if(taem_exec_owns_vehicle(&g->taem_exec)&&g->taem_exec.phase==TAEM_PHASE_S_TURN)
                return taem_s_turn_guidance(g,t,course,p,aero,cfg,dt);
            if(g->terminal_path_committed&&g->terminal_path_kind==TERMINAL_PATH_HAC&&
               g->hac_transition_heading_cone&&g->hac_transition_active){
                HACGuidance immediate_path=hac_path_guidance(g,t,&cfg->site,s,p->radius,
                    g->hac_side,planet_surface_gravity(p),course,g->hac_radius);
                double immediate_bank=taem_bank_demand(t,&immediate_path,g->hac_radius,
                    g->hac_side,aero,v);
                const char*diag=getenv("KSP_LANDER_HAC_DIAGNOSTICS");
                if(diag&&strcmp(diag,"1")==0&&!g->diagnostic_shadow)
                    fprintf(stderr,"MM305 fixed-HAC command: UT %.2f true %.3f horiz %.3f surf %.3f fpa %.3f latSpeed %.3f lateral %.6f bank %.6f side %.0f R %.1f.\n",
                        t->ut,t->true_air_speed,t->horizontal_speed,t->surface_speed,
                        t->flight_path_angle,guidance_lateral_speed(t),
                        immediate_path.lateral_acceleration,immediate_bank,
                        g->hac_side,g->hac_radius);
                GuidanceCommand immediate=atmospheric(t,
                    isfinite(t->ground_track_heading)?t->ground_track_heading:course,
                    immediate_bank,v,0.0,false,PROFILE_TAEM);
                immediate.heading_control_enabled=false;
                immediate.has_target_aoa=true;
                immediate.target_aoa=clampd(g->terminal_reference_aoa,0.0,
                    v->maximum_angle_of_attack);
                immediate.target_pitch=t->flight_path_angle+immediate.target_aoa;
                const char*fixed_status=g->hac_transition_lead_length>1.0&&
                    g->hac_transition_lead_progress<.995?
                    "MM305 fixed-HAC committed; acquiring the runway-anchored circle through its finite C1 lead.":
                    "MM305 fixed-HAC committed; flying the runway-anchored heading-alignment arc.";
                return stabilized(g,result_make(PHASE_TAEM,immediate,fixed_status,NULL),t,v,s,dt);
            }
            /*
             * Ownership has just crossed the one-way boundary, but the first
             * terminal candidate is still only a preview.  Do not fall through
             * into the default HAC law on this same frame: that law has no
             * published join and can command a large bank before the selector
             * has established an executable path.  Give the next sample a
             * neutral tangent/response frame so the candidate search and live
             * vehicle state remain on the same side of the contract.
            */
            terminal_predict(g,t,course,p,aero,cfg,dt);
            /* The response-projected candidate already carries the next
               ownership boundary. If the shared forecast, vertical, and
               energy envelopes all agree on this handoff sample, commit it
               before the forecast clock becomes stale on the next tick. */
            if(g->terminal_test_capture_active&&
               terminal_candidate_commit_ready(g,t,p,aero,cfg)){
                terminal_publish_candidate(g);
                g->terminal_path_committed=true;
                g->terminal_test_capture_active=false;
                g->terminal_energy_mismatch_duration=0.0;
                g->hac_commit_blend=0.0;
            }else{
            GuidanceCommand handoff_hold=atmospheric(t,course,0.0,v,0.0,
                false,PROFILE_TAEM);
            handoff_hold.heading_control_enabled=false;
            handoff_hold.has_target_aoa=true;
            handoff_hold.target_aoa=clampd(g->terminal_reference_aoa,0.0,
                v->maximum_angle_of_attack);
            handoff_hold.target_pitch=t->flight_path_angle+
                handoff_hold.target_aoa;
            return stabilized(g,result_make(PHASE_TAEM,handoff_hold,
                "TAEM ownership latched; holding the current tangent while terminal geometry is qualified.",
                "No executable terminal path has been published; lateral turn is inhibited."),
                t,v,s,dt);
            }
        }else{
            /* MM304 -> TAEM ownership is now exclusively the strict fixed-point
               contract: live course 330..030 or 150..210 for KSC 09, positive
               energy/speed/altitude/HAC suitability, and <=1 km horizontal
               error at the final rear alignment point.  Do not use low-speed
               or altitude safety fallbacks to enter TAEM; those produced TAEM
               phases without a real capture_ready handoff. */
            return entry;
        }
    }

    /* The runway-capture envelope is the fallback reserve for an uncommitted
       terminal state.  Once MM305 has frozen a regular HAC/spline, its route
       is the lateral capture contract; applying the point-to-runway bound
       here would reject a valid long-path turn on the very next sample simply
       because the vehicle is intentionally still perpendicular to the strip. */
    if(g->terminal_region_entered&&!g->terminal_path_committed&&
       !g->final_approach_captured&&
       terminal_capture_margin_exhausted(g,t,course,p,aero,cfg))
        return terminal_abort(g,
            "Terminal trajectory lost: the modeled runway-capture reserve is physically exhausted.");
    GuidanceSettings path_settings=terminal_path_settings(g,s);
    const GuidanceSettings*hs=&path_settings;
    double radius=g->terminal_glide_mode?fmax(3000.0,g->hac_radius):fmax(s->hac_radius,g->hac_radius);
    bool fixed_alignment_hac_path=g->fixed_alignment_hac_latched&&
        g->hac_transition_active&&!g->hac_completed&&!g->final_approach_captured;
    bool spline_path=!fixed_alignment_hac_path&&
        g->terminal_path_kind==TERMINAL_PATH_SPLINE;
    bool heading_cone_path=g->hac_transition_heading_cone;
    bool runway_line_path=!fixed_alignment_hac_path&&
        (spline_path||(heading_cone_path&&!g->hac_transition_active));
    HACGuidance h=runway_line_path?
        terminal_runway_line_path_guidance(t,&cfg->site,hs,planet_surface_gravity(p),course):
        hac_guidance_radius(t,&cfg->site,hs,p->radius,g->hac_side,planet_surface_gravity(p),course,radius);
    double nominal_bank=fmin(55.0,dynamic_bank_limit(t,v));
    g->minimum_turn_radius=live_turn_radius(t,aero,v,nominal_bank);
    bool feasible=runway_line_path?true:
        (isfinite(g->minimum_turn_radius)&&g->minimum_turn_radius<=radius/1.15);
    bool transition_block=g->hac_transition_active;
    bool audit_was_lead=transition_block&&
        g->hac_transition_lead_length>1.0&&
        g->hac_transition_lead_progress<.995;
    if(transition_block){
        GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
        GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
        double e=0,n=0;local_offsets(origin,current,p->radius,&e,&n);
        if(fixed_alignment_hac_path)
            (void)hac_fixed_lead_rebase_from_vector(g,t,course,p,cfg,e,n);
        HACTransitionPlan transition={.valid=true,
            .p0={g->hac_transition_p0_e,g->hac_transition_p0_n},.p1={g->hac_transition_p1_e,g->hac_transition_p1_n},
            .p2={g->hac_transition_p2_e,g->hac_transition_p2_n},.p3={g->hac_transition_p3_e,g->hac_transition_p3_n},
            .length=g->hac_transition_length,.end_angle=g->hac_transition_end_angle};
        bool lead_done=g->hac_transition_lead_length<=1.0||g->hac_transition_lead_progress>=.995;
        if(!lead_done){
            HACTransitionPlan lead={0};
            lead.valid=true;
            lead.p0=(HACPoint2){g->hac_transition_lead_start_e,g->hac_transition_lead_start_n};
            lead.p1=(HACPoint2){g->hac_transition_lead_p1_e,g->hac_transition_lead_p1_n};
            lead.p2=(HACPoint2){g->hac_transition_lead_p2_e,g->hac_transition_lead_p2_n};
            lead.p3=transition.p0;lead.length=g->hac_transition_lead_length;
            if(fixed_alignment_hac_path&&g->hac_transition_heading_cone){
                lead_done=fixed_hac_lead_capture_update(g,t,course,radius,e,n,&lead);
            }else{
                HACPoint2 chord={lead.p3.e-lead.p0.e,lead.p3.n-lead.p0.n};
                double chord_len=fmax(hypot(chord.e,chord.n),1.0);
                double chord_len2=chord_len*chord_len;
                double projected=((e-lead.p0.e)*chord.e+(n-lead.p0.n)*chord.n)/chord_len2;
                double nearest=g->hac_transition_lead_curve?
                    hac_bezier_nearest_u(&lead,e,n):clampd(projected,0.0,1.0);
                if(nearest>g->hac_transition_lead_progress)
                    g->hac_transition_lead_progress=nearest;
                double endpoint=hypot(e-lead.p3.e,n-lead.p3.n);
                HACPoint2 end_dir=g->hac_transition_lead_curve?
                    hac_bezier_derivative(&lead,1.0):chord;
                double end_norm=fmax(hypot(end_dir.e,end_dir.n),1.0);
                double passed=((e-lead.p3.e)*end_dir.e+(n-lead.p3.n)*end_dir.n)/end_norm;
                double cross=fabs((e-lead.p3.e)*end_dir.n-(n-lead.p3.n)*end_dir.e)/end_norm;
                bool close_endpoint=endpoint<=fmax(120.0,t->true_air_speed*.65);
                bool passed_in_corridor=passed>=0.0&&cross<=fmax(350.0,radius*.12);
                if(close_endpoint||passed_in_corridor)
                    g->hac_transition_lead_progress=1.0;
                lead_done=g->hac_transition_lead_progress>=.995;
            }
        }
        if(lead_done){
            if(g->hac_transition_heading_cone){
                h=hac_heading_cone_path_guidance(g,t,&cfg->site,hs,p->radius,
                    planet_surface_gravity(p),course);
                double nearest=1.0-h.arc_remaining/
                    fmax(g->hac_transition_cone_arc_length,1.0);
                if(nearest>g->hac_transition_progress)
                    g->hac_transition_progress=nearest;
                g->hac_transition_progress=clampd(g->hac_transition_progress,0.0,1.0);
                double endpoint=hypot(e-g->hac_transition_p3_e,
                    n-g->hac_transition_p3_n);
                double runway=cfg->site.runway_heading*DEG2RAD;
                double de=e-g->hac_transition_p3_e,dn=n-g->hac_transition_p3_n;
                double passed=de*sin(runway)+dn*cos(runway);
                double cross=fabs(de*cos(runway)-dn*sin(runway));
                double heading_error=fabs(norm_signed_deg(
                    cfg->site.runway_heading-course));
                bool close=endpoint<=fmax(260.0,t->true_air_speed*2.8);
                double fixed_capture_distance=fmax(350.0,
                    fmin(1000.0,radius*.12));
                bool fixed_close=endpoint<=fmax(fixed_capture_distance,
                    t->true_air_speed*2.8);
                bool passed_corridor=passed>=0.0&&cross<=fmax(300.0,radius*.12);
                bool exhausted=g->hac_transition_progress>=.995&&
                    (passed>=-fmax(250.0,radius*.05)||
                     endpoint<=fmax(350.0,t->true_air_speed*2.5));
                bool fixed_endpoint_valid=fixed_alignment_hac_path&&
                    g->hac_transition_progress>=.975&&
                    fabs(h.radial_error)<=fmax(250.0,radius*.05)&&
                    fixed_close&&heading_error<=22.0;
                bool ordinary_endpoint_valid=!fixed_alignment_hac_path&&
                    g->hac_transition_progress>=.975&&heading_error<=22.0&&
                    (close||passed_corridor||exhausted);
                if(fixed_endpoint_valid||ordinary_endpoint_valid){
                    g->hac_transition_active=false;
                    g->hac_remaining=0.0;
                    g->hac_progress_valid=true;
                    g->hac_captured=true;
                    if(fixed_endpoint_valid){
                        /* The analytic 270-degree circle ends here exactly once.
                           Record that geometric fact now; HAC completion does not
                           grant Final ownership.  If the live Final contract is not
                           yet admissible, TAEM remains on the runway-aligned
                           conditioning line until the unchanged Final gate passes. */
                        g->hac_completed=true;
                        g->terminal_test_spiral_active=false;
                        g->terminal_test_revolution_remaining=0.0;
                    }
                    h=terminal_runway_line_path_guidance(t,&cfg->site,hs,
                        planet_surface_gravity(p),course);
                }
                else if(!fixed_alignment_hac_path&&
                         passed>fmax(350.0,t->horizontal_speed*
                             fmax(1.0,g->hac_transition_response_time))&&
                         cross>fmax(300.0,radius*.12)){
                    /* A closest-point projection can saturate before the
                       runway tangent while the vehicle flies past it.  Once
                       the endpoint is behind the response horizon and the
                       measured lateral miss is outside its capture corridor,
                       this frozen join cannot still deliver Final. */
                    if(terminal_entry_recovery_available(g,t,v,s))
                        return terminal_return_to_entry(g,t,state,course,p,aero,cfg,dt);
                    return terminal_abort(g,
                        "HAC join missed its runway tangent outside the capture corridor.");
                }
            }else{
            double nearest=hac_transition_nearest_u(g,e,n);
            /* Progress is a measured path coordinate, not a command reference.
               Rate-limiting it by aircraft heading response made the stored
               station lag behind the real closest point, so altitude scheduling
               and transition completion were based on a fictitious location.
               Keep it monotonic, but otherwise take the geometric projection
               directly. Command bandwidth remains enforced on bank/curvature. */
            if(nearest>g->hac_transition_progress)g->hac_transition_progress=nearest;
            g->hac_transition_progress=clampd(g->hac_transition_progress,0,1);
            double endpoint=hypot(e-g->hac_transition_p3_e,n-g->hac_transition_p3_n);
            HACPoint2 end_dir=hac_bezier_derivative(&transition,1.0);
            double end_heading=norm_deg(atan2(end_dir.e,end_dir.n)*RAD2DEG);
            double end_heading_error=fabs(norm_signed_deg(end_heading-course));
            double end_norm=fmax(hypot(end_dir.e,end_dir.n),1.0);
            double passed_endpoint=((e-g->hac_transition_p3_e)*end_dir.e+
                (n-g->hac_transition_p3_n)*end_dir.n)/end_norm;
            double endpoint_cross=fabs((e-g->hac_transition_p3_e)*end_dir.n-
                (n-g->hac_transition_p3_n)*end_dir.e)/end_norm;
            bool close_endpoint=endpoint<=fmax(260.0,t->true_air_speed*2.8);
            if(spline_path){
                /* A free spline terminates on the runway-aligned outer-final
                   station, so there is no circle to capture after its endpoint.
                   Accept a small passed-end corridor and hand residual error to
                   the runway-line Frenet law. */
                bool passed_in_corridor=passed_endpoint>=0&&
                    endpoint_cross<=fmax(300.0,radius*.16);
                bool exhausted=g->hac_transition_progress>=.995&&
                    (passed_endpoint>=-fmax(250.0,radius*.06)||
                     endpoint<=fmax(350.0,t->true_air_speed*2.5));
                if(!fixed_alignment_hac_path&&
                   g->hac_transition_progress>=.975&&end_heading_error<=22.0&&
                   (close_endpoint||passed_in_corridor||exhausted)){
                    g->hac_transition_active=false;g->hac_remaining=0.0;
                    g->hac_progress_valid=true;g->hac_captured=true;
                    h=terminal_runway_line_path_guidance(t,&cfg->site,hs,
                        planet_surface_gravity(p),course);
                }
            }else{
                /* Handoff after crossing the tangent endpoint in the circle capture
                   corridor. Do not strand a completed join on its final tangent. */
                bool passed_in_corridor=passed_endpoint>=0&&endpoint_cross<=radius*.45&&
                    fabs(h.radial_error)<=radius*.45;
                bool circle_capture_corridor=fabs(h.radial_error)<=radius*.55&&
                    fabs(h.course_error)<=65.0;
                bool exhausted=g->hac_transition_progress>=.995&&
                    (passed_endpoint>=-radius*.10||endpoint<=radius*.45);
                if(g->hac_transition_progress>=.975&&
                   (((close_endpoint||passed_in_corridor)&&end_heading_error<=28.0)||
                    (exhausted&&circle_capture_corridor))){
                    g->hac_transition_active=false;
                    double join_advance=-g->hac_side*norm_signed_deg(
                        (h.angle-g->hac_transition_end_angle)*RAD2DEG)*DEG2RAD;
                    g->hac_remaining-=join_advance*radius;
                    g->hac_previous_angle=h.angle;g->hac_progress_valid=true;
                }
            }
            }
        }
    }
    if(g->hac_energy_audit_active){
        hac_energy_audit_observe(g,t,p,aero,v,dt);
        /* Close on the actual execution stage transition, not on a
           threshold that can be crossed before a rebase or revisited by a
           second acquisition call. */
        bool audit_now_arc=g->hac_transition_active&&
            g->hac_transition_lead_length>1.0&&
            g->hac_transition_lead_progress>=.995;
        if(audit_was_lead&&audit_now_arc)
            hac_energy_audit_close(g,t,p);
    }
    if(transition_block){
        /* The cubic join already carries its own curvature. Do not consume
           circle arc from raw polar-angle motion until the aircraft has
           actually reached the tangent endpoint. */
    }else if(runway_line_path){
        h=terminal_runway_line_path_guidance(t,&cfg->site,hs,planet_surface_gravity(p),course);
        g->hac_remaining=0.0;
    }else if(g->terminal_glide_mode&&g->terminal_test_spiral_active&&!g->hac_completed){
        h=terminal_test_update_spiral(g,t,course,p,aero,cfg,dt);
        radius=g->hac_radius;
        feasible=isfinite(g->minimum_turn_radius)&&g->minimum_turn_radius<=radius/1.06;
    } else if (g->hac_progress_valid && !g->hac_completed) {
        /* Integrate signed angular progress through atan2's branch cut. The
           remaining arc can never jump back to a fresh positive revolution.
           Reverse motion restores distance so repeated oscillations cannot
           manufacture progress. Once established, retain signed position even
           during loss of capture so the physical exit cannot disappear. */
        double progress = -g->hac_side * norm_signed_deg((h.angle - g->hac_previous_angle) * RAD2DEG) * DEG2RAD;
        g->hac_remaining -= progress * radius;
    } else if (!g->hac_progress_valid && (!g->terminal_glide_mode || g->hac_remaining <= 0)) {
        g->hac_remaining = h.arc_remaining;
    }
    g->hac_previous_angle = h.angle;
    TerminalPreflarePlan approach_plan={0};
    bool approach=terminal_outer_gate(g,t,course,p,aero,cfg,&approach_plan);
    if(runway_line_path&&!transition_block&&!approach){
        TerminalPreflarePlan spline_plan={0};
        if(terminal_outer_capture_admissible(g,t,course,p,aero,cfg,&spline_plan)){
            approach=true;approach_plan=spline_plan;
        }
    }
    double exit_window=fmax(200,radius*.012);
    if(transition_block&&getenv("KSP_LANDER_HAC_DIAGNOSTICS")&&
       strcmp(getenv("KSP_LANDER_HAC_DIAGNOSTICS"),"1")==0&&!g->diagnostic_shadow){
        static double last_join_diag_ut=-INFINITY;
        if(t->ut-last_join_diag_ut>=1.0){
            GeoPoint origin={cfg->site.latitude,cfg->site.longitude,cfg->site.altitude};
            GeoPoint current={t->latitude,t->longitude,t->mean_altitude};
            double e=0.0,n=0.0;
            local_offsets(origin,current,p->radius,&e,&n);
            fprintf(stderr,"HAC_JOIN ut=%.3f kind=%d cone=%d leadProgress=%.5f progress=%.5f pos=(%.1f,%.1f) p0=(%.1f,%.1f) p3=(%.1f,%.1f) radial=%.1f courseError=%.1f course=%.1f remaining=%.1f\n",
                t->ut,g->terminal_path_kind,g->hac_transition_heading_cone?1:0,
                g->hac_transition_lead_progress,g->hac_transition_progress,e,n,
                g->hac_transition_p0_e,g->hac_transition_p0_n,
                g->hac_transition_p3_e,g->hac_transition_p3_n,
                h.radial_error,h.course_error,course,g->hac_remaining);
            last_join_diag_ut=t->ut;
        }
    }
    double station=-hs->final_approach_distance;
    double lead=clampd(t->horizontal_speed*hac_response_lead_time(t,s,g->hac_side*25.0),180.0,2400.0);
    bool at_exit=t->runway_along_track>=station-lead&&t->runway_along_track<=station+exit_window&&
        fabs(t->runway_cross_track)<fmax(250.0,fmin(1000.0,radius*.10))&&
        fabs(norm_signed_deg(course-cfg->site.runway_heading))<12.0;
    if(!runway_line_path&&!g->hac_completed&&!g->final_approach_captured&&!transition_block&&!approach&&at_exit&&
       (!g->hac_progress_valid||g->hac_remaining<=lead+exit_window)){
        GuidanceSettings circuit_settings=*hs;
        circuit_settings.taem_glide_slope=s->taem_glide_slope;
        if(g->terminal_glide_mode)circuit_settings.final_glide_slope=g->terminal_test_glide_slope;
        double arc=0,slope=0;
        /* At the first crossing the geometric arc may already have wrapped.
           Convert it to signed distance past this exit before budgeting ONE circuit. */
        double remaining=g->hac_remaining;
        if(!g->hac_progress_valid&&remaining>LANDER_PI*radius)remaining-=2*LANDER_PI*radius;
        if(hac_high_pass_circuit(t->mean_altitude,t->true_air_speed,radius,remaining,
            g->minimum_turn_radius,live_drag_accel(t,aero,v),p,&cfg->site,v,&circuit_settings,&arc,&slope)){
            g->hac_remaining=arc;g->hac_circuit_slope=slope;g->hac_circuit_count++;
            g->hac_progress_valid=true;g->hac_captured=true;g->hac_previous_angle=h.angle;
            path_settings.taem_glide_slope=slope;
            h=hac_guidance_radius(t,&cfg->site,hs,p->radius,g->hac_side,planet_surface_gravity(p),course,radius);
            robust_pid_reset(&g->taem_altitude_pid);
        }
    }
    /* HAC capture is a lateral-path state, not an energy-state gate.  Energy
       still determines the vertical schedule, final-approach validity and
       whether an explicit high-pass circuit is feasible, but it must not keep
       a geometrically captured aircraft labelled as TAEM. */
    if(runway_line_path){
        double line_capture=fmax(220.0,fmin(850.0,hs->final_approach_distance*.085));
        g->hac_captured=!g->hac_transition_active&&
            fabs(t->runway_cross_track)<=line_capture&&
            fabs(norm_signed_deg(cfg->site.runway_heading-course))<24.0;
    }else g->hac_captured=!g->hac_transition_active&&feasible&&
        fabs(h.radial_error)<=fmax(250,fmin(1500.0,radius*.10))&&fabs(h.course_error)<24;

    bool lost=runway_line_path?
        (fabs(t->runway_cross_track)>fmax(1200.0,hs->final_approach_distance*.20)||
         fabs(norm_signed_deg(cfg->site.runway_heading-course))>65.0):
        (!feasible||fabs(h.radial_error)>fmax(750.0,fmin(5000.0,radius*.25))||fabs(h.course_error)>60.0);
    g->hac_capture_lost_duration=lost?g->hac_capture_lost_duration+dt:fmax(0,g->hac_capture_lost_duration-dt*2);
    bool entry_recoverable=terminal_entry_recovery_available(g,t,v,s);
    if(g->hac_capture_lost_duration>=5.0&&entry_recoverable)
        return terminal_return_to_entry(g,t,state,course,p,aero,cfg,dt);
    /* A committed join may cross the exit plane before looping back to its
       tangent. Judge station passage only after that join has completed. */
    bool missed_station=!runway_line_path&&!transition_block&&g->hac_progress_valid&&
        t->runway_along_track>station+exit_window&&
        fabs(norm_signed_deg(course-cfg->site.runway_heading))<45&&
        g->hac_remaining<=exit_window;
    if(!g->hac_completed&&!g->final_approach_captured&&!approach&&missed_station){
        if(entry_recoverable)return terminal_return_to_entry(g,t,state,course,p,aero,cfg,dt);
        return terminal_abort(g,"HAC missed: no feasible high-pass circuit or final capture remains.");
    }
    if (!g->hac_completed && g->hac_progress_valid && g->hac_remaining <= exit_window) {
        if(g->hac_remaining>=-exit_window&&g->hac_captured&&approach&&t->runway_along_track<0){g->hac_completed=true;g->hac_remaining=0;g->terminal_test_spiral_active=false;g->terminal_test_revolution_remaining=0;}
        else if(g->hac_remaining < -exit_window)
            return terminal_abort(g,"HAC missed: exit crossed without a valid final capture.");
    }
    TaemTerminalContract delivery_contract=terminal_delivery_contract(g,t,course,p,cfg,&approach_plan);
    if(getenv("KSP_LANDER_HAC_DIAGNOSTICS")&&
       strcmp(getenv("KSP_LANDER_HAC_DIAGNOSTICS"),"1")==0&&
       !g->diagnostic_shadow){
        static double last_contract_diag_ut=-INFINITY;
        if(t->ut-last_contract_diag_ut>=1.0){
            fprintf(stderr,
                "HAC_TERMINAL_CONTRACT UT %.3f valid=%d planFeasible=%d planReject=%s heightValid=%d heightMargin %.3f "
                "triggerAlt %.3f minSpeed %.3f refSpeed %.3f response %.3f "
                "range %.3f rangeMargin %.3f altitude %.3f altitudeMargin %.3f fpa %.3f fpaMargin %.3f q %.3f qMargin %.3f "
                "energy %.3f energyMargin %.3f responseAvail %.3f responseReq %.3f committed=%d hacCompleted=%d approach=%d\n",
                t->ut,delivery_contract.valid?1:0,approach_plan.feasible?1:0,
                approach_plan.reject_reason?approach_plan.reject_reason:"null",
                approach_plan.height.valid?1:0,approach_plan.height.margin,
                approach_plan.trigger_altitude,approach_plan.minimum_speed,
                approach_plan.reference_speed,approach_plan.response_time,
                delivery_contract.range_to_go,delivery_contract.range_margin,
                delivery_contract.altitude,delivery_contract.altitude_margin,
                delivery_contract.flight_path_angle,delivery_contract.flight_path_angle_margin,
                delivery_contract.dynamic_pressure,delivery_contract.dynamic_pressure_margin,
                delivery_contract.specific_energy,delivery_contract.specific_energy_margin,
                delivery_contract.response_time_available,
                delivery_contract.response_time_required,
                g->terminal_path_committed?1:0,g->hac_completed?1:0,approach?1:0);
            last_contract_diag_ut=t->ut;
        }
    }
    /* Publish the live numeric contract before any public Final latch.  The
       executive must first admit FINAL_INTERCEPT, then explicitly deliver Final
       on the same qualified contract; coarse HAC-complete/approach booleans are
       no longer sufficient to bypass those checks. */
    (void)taem_exec_sync_with_contract(g,t,s,p,aero,cfg,&delivery_contract,false,false);
    if(g->hac_completed && !g->final_approach_captured && !approach){
        /* A completed analytic HAC may reach the runway-aligned outer glide while
           an immediate pull-up projection is still speed-limited.  Final itself
           already treats that state as recoverable above the preflare trigger,
           and final-only admission uses this same physics-derived conditioning
           proof.  Keep TAEM ownership on the runway line only when that proof
           says the unchanged Final contract remains reachable; otherwise abort. */
        EnergyPathEnvelope conditioning_energy={0};
        double conditioning_end_speed=NAN,conditioning_slope=NAN;
        bool conditioning=terminal_final_conditioning_reachable(g,t,course,p,aero,cfg,
            &conditioning_energy,&conditioning_end_speed,&conditioning_slope);
        if(getenv("KSP_LANDER_HAC_DIAGNOSTICS")&&
           strcmp(getenv("KSP_LANDER_HAC_DIAGNOSTICS"),"1")==0&&!g->diagnostic_shadow)
            fprintf(stderr,
                "HAC_FINAL_CONDITIONING UT %.3f reachable=%d slope=%.3f endSpeed=%.3f energyMargin=%.3f along=%.1f cross=%.1f alt=%.1f speed=%.3f fpa=%.3f\n",
                t->ut,conditioning?1:0,conditioning_slope,conditioning_end_speed,
                conditioning_energy.valid?conditioning_energy.energy.margin:NAN,
                t->runway_along_track,t->runway_cross_track,t->mean_altitude,
                t->true_air_speed,t->flight_path_angle);
        if(!conditioning)
            return terminal_abort(g,runway_line_path?
                "Terminal runway-line exit rejected: final approach envelope is invalid and no conditioning path remains.":
                "HAC exit rejected: final approach envelope is invalid and no conditioning path remains.");
    }
    if (!g->final_approach_captured && g->hac_completed && approach && t->runway_along_track < 0 &&
        g->taem_exec.phase==TAEM_PHASE_FINAL_INTERCEPT&&g->taem_exec.terminal_evaluation.feasible) {
        (void)taem_exec_sync_with_contract(g,t,s,p,aero,cfg,&delivery_contract,true,true);
        if(g->taem_exec.taem_complete){
            g->final_approach_captured = true;
            terminal_store_preflare_plan(g,&approach_plan);
            terminal_set_stage(g,TERMINAL_TRAJECTORY_CAPTURE,t->ut);
            robust_pid_reset(&g->final_altitude_pid);
            robust_pid_reset(&g->speed_pid);
            robust_pid_reset(&g->flare_sink_pid);
        }
    }
    if(!g->final_approach_captured||!g->taem_exec.taem_complete)g->final_invalid_duration=0;
    Trajectory ref;
    trajectory_init(&ref);
    if(g->fixed_alignment_hac_latched&&!g->final_approach_captured)
        reference_trajectory_fixed_hac(&ref,&cfg->site,hs,p->radius,g);
    else
        reference_trajectory_radius(&ref, &cfg->site, hs, p->radius, g->hac_side, radius);
    GuidanceResult r;
    if (!g->final_approach_captured || !g->taem_exec.taem_complete) {
        if(g->hac_captured)g->hac_progress_valid = true;
        /* MM305 owns path acquisition and runway alignment as internal TAEM
           substates.  Do not surface a peer Heading-Alignment guidance phase;
           public ownership stays PHASE_TAEM until final approach is captured. */
        g->phase = PHASE_TAEM;
        r = taem_guidance(g, t, course, p, aero, cfg, &ref, dt);
    } else {
        r=terminal_approach_sequence(g,t,course,p,aero,cfg,&ref,dt);
    }
    trajectory_clear(&ref);
    return r;
}

#define guidance_update guidance_update_impl
GuidanceResult guidance_update(GuidanceMachine*g,const Telemetry*t,const VehicleState*state,const DeorbitPlan*plan,const PlanetModel*p,AerodynamicModel aero,const LandingConfiguration*cfg){double dt=g->has_previous_ut?clampd(t->ut-g->previous_ut,0,1):.1;g->previous_ut=t->ut;g->has_previous_ut=true;if(g->aborted){GuidanceCommand c;guidance_command_init(&c);return result_make(PHASE_ABORT,c,g->abort_reason[0]?g->abort_reason:"Automation aborted. Manual control restored.",NULL);}if(!g->automation_engaged){g->phase=PHASE_IDLE;reset_limiters(g);GuidanceCommand c;guidance_command_init(&c);if(plan){if(plan->execution_qualified)return result_make(PHASE_IDLE,c,plan->execution_degraded?"Recoverable deorbit plan is ready. Engage guidance to begin execution.":"Robust deorbit plan is ready. Engage guidance to begin execution.",plan->execution_degraded?"The plan misses the preferred strict corridor but passed the guarded recovery envelope.":NULL);return result_make(PHASE_IDLE,c,"The current deorbit plan is preview-only and cannot be executed safely.","Replan for a later orbital opportunity or revise the vehicle/site model before engagement.");}return result_make(PHASE_IDLE,c,"Guidance is not engaged.",NULL);}if(g->paused){g->phase=PHASE_PAUSED;GuidanceCommand c;guidance_command_init(&c);return result_make(PHASE_PAUSED,c,"Guidance paused. Attitude hold released.",NULL);}const VehicleProfile*v=&cfg->vehicle;const GuidanceSettings*s=&cfg->guidance;double course=surface_course(state->position,state->velocity,planet_rotation_vector(p),p->north_axis,t->heading);if(g->terminal_glide_mode)return terminal_guidance(g,t,state,course,p,aero,cfg,dt);if(!plan){g->phase=PHASE_PLANNING;GuidanceCommand c;guidance_command_init(&c);return result_make(PHASE_PLANNING,c,"A deorbit plan is required before engagement.",NULL);}double entry_alt=entry_guidance_start_altitude(p,s);if(!g->deorbit_burn_completed&&t->mean_altitude<=entry_alt&&t->vertical_speed<0)g->deorbit_burn_completed=true;double peri_rem=t->periapsis_altitude-plan->predicted_post_burn_periapsis_altitude;bool has_peri=plan->predicted_post_burn_periapsis_altitude>10000&&plan->predicted_post_burn_periapsis_altitude<p->atmosphere_depth&&isfinite(t->periapsis_altitude)&&t->periapsis_altitude>-p->radius*.5,peri_hit=g->has_burn_command_started&&has_peri&&peri_rem<=300,state_hit=g->has_burn_command_started&&plan->live_cutoff_capture_qualified;double dvtarget=plan->delta_v+(has_peri?8:0);if(g->has_burn_command_started&&(g->delivered_delta_v>=dvtarget||peri_hit||state_hit))g->deorbit_burn_completed=true;bool still=g->has_burn_command_started&&!g->deorbit_burn_completed&&g->delivered_delta_v<dvtarget&&!peri_hit&&!state_hit;double burnstart=plan->burn_ut-plan->estimated_burn_duration*.5,open=t->ut>=burnstart;if(!g->deorbit_burn_completed&&t->mean_altitude>entry_alt-4500&&(t->vertical_speed>=-20||still||open)){GuidanceCommand c;guidance_command_init(&c);c.autopilot_engaged=true;c.use_inertial_direction=true;c.inertial_direction=vnorm(vscale(state->velocity,-1),v3(-1,0,0));c.navball_speed_mode=SPEED_ORBIT;c.control_profile=PROFILE_ORBITAL;if(t->ut<burnstart-12){g->phase=PHASE_COAST;char st[128];int sec=(int)fmax(0,round(burnstart-t->ut));snprintf(st,sizeof(st),"Coasting while acquiring retrograde. T−%02d:%02d.",sec/60,sec%60);return result_make(g->phase,c,st,NULL);}if(t->ut<burnstart){g->phase=PHASE_BURN_SETUP;return result_make(g->phase,c,"Aligning retrograde for the deorbit burn.",NULL);}if(g->delivered_delta_v<dvtarget&&!peri_hit&&!state_hit){g->phase=PHASE_DEORBIT_BURN;double maxthr=s->deorbit_maximum_throttle,avail=fmax(.05,t->available_thrust*maxthr/fmax(t->mass,1)),meas=fmax(0,t->current_thrust/fmax(t->mass,1)),align=fabs(t->autopilot_error);bool aligned=align<=10;if(!aligned&&!g->has_burn_command_started&&t->ut>burnstart+fmax(30,plan->estimated_burn_duration)){guidance_abort(g);GuidanceCommand safe;guidance_command_init(&safe);return result_make(PHASE_ABORT,safe,"Deorbit burn aborted because retrograde alignment missed the burn window.","The vehicle did not achieve the required retrograde alignment in time. Manual control restored; replan from the current orbit.");}if(aligned&&!g->has_burn_command_started){g->burn_command_started_ut=t->ut;g->has_burn_command_started=true;g->burn_active_elapsed=0;g->burn_progress_watch_ut=t->ut;g->burn_progress_watch_delta_v=g->delivered_delta_v;g->has_burn_progress_watch=true;}if(aligned&&g->has_burn_command_started){g->delivered_delta_v+=meas*fmax(0,cos(align*DEG2RAD))*dt;g->burn_active_elapsed+=dt;if(!g->has_burn_progress_watch||g->delivered_delta_v-g->burn_progress_watch_delta_v>=.25){g->burn_progress_watch_ut=t->ut;g->burn_progress_watch_delta_v=g->delivered_delta_v;g->has_burn_progress_watch=true;}}double rem=fmax(0,dvtarget-g->delivered_delta_v);double fraction=burn_fraction(g->burn_active_elapsed,rem,avail,s->deorbit_throttle_ramp_duration),authority=clampd((10-align)/6,0,1),pa=has_peri?clampd((peri_rem-250)/8000,0,1):1;c.target_throttle=rem<.08?0:(aligned?maxthr*fmin(fraction,pa)*authority:0);double expected_accel=fmax(0,t->available_thrust*c.target_throttle/fmax(t->mass,1));bool stalled=aligned&&g->has_burn_command_started&&g->has_burn_progress_watch&&rem>2&&t->ut-g->burn_progress_watch_ut>6&&expected_accel>.02&&meas<fmax(.01,expected_accel*.10);if(stalled){guidance_abort(g);GuidanceCommand safe;guidance_command_init(&safe);return result_make(PHASE_ABORT,safe,"Deorbit burn aborted after sustained loss of thrust/delta-v progress.","The burn stopped making measurable progress despite a meaningful commanded thrust level. Manual control restored; do not continue entry on the stale plan.");}char st[320];snprintf(st,sizeof(st),"Deorbit burn: %.1f / %.1f m/s, throttle %.0f%%.%s%s",g->delivered_delta_v,plan->delta_v,c.target_throttle*100,has_peri?" Periapsis closure active.":"",plan->live_cutoff_capture_qualified?" Cutoff-now trajectory is inside the entry corridor.":"");return result_make(g->phase,c,st,t->available_thrust<1?"No usable thrust is available.":align>10?"Holding throttle until retrograde alignment is stable.":NULL);}}
    if(!g->deorbit_burn_completed){g->phase=PHASE_COAST;GuidanceCommand c;guidance_command_init(&c);c.autopilot_engaged=true;c.use_inertial_direction=true;c.inertial_direction=vnorm(vscale(state->velocity,-1),v3(-1,0,0));c.navball_speed_mode=SPEED_ORBIT;c.control_profile=PROFILE_ORBITAL;return result_make(g->phase,c,"Holding retrograde until the deorbit burn completes.",NULL);}if(!g->atmospheric_interface_crossed){double signed_entry_roll=norm_signed_deg(t->roll),entry_roll=fabs(signed_entry_roll),capture_aoa=entry_low_q_protective_aoa_floor(t->dynamic_pressure,v),entry_pitch_error=fabs(t->angle_of_attack-capture_aoa),entry_heading_error=fabs(norm_signed_deg(t->ground_track_heading-t->heading));bool entry_attitude_ready=entry_roll<=18&&entry_pitch_error<=4&&entry_heading_error<=15&&fabs(t->roll_rate)<=8&&fabs(t->pitch_rate)<=8&&fabs(t->heading_rate)<=8;bool entry_reached=t->mean_altitude<=entry_alt&&t->vertical_speed<0;if(entry_reached&&entry_attitude_ready){g->atmospheric_interface_crossed=true;/* Continue from the bank direction actually reached instead of forcing an unnecessary high-Mach reversal. */if(entry_roll>=5)g->s_turn_sign=signed_entry_roll<0?-1:1;}else{g->phase=PHASE_ENTRY_INTERFACE;const char*st=entry_reached?"Atmospheric interface reached. Holding prograde heading, entry AoA and wings-level attitude before MM304 guidance.":"Burn complete. Using RCS/direct control to capture prograde heading, entry AoA and wings-level attitude.";return result_make(g->phase,entry_capture(t,state,v),st,entry_reached&&!entry_attitude_ready?"MM304 guidance is inhibited until entry heading/AoA/roll attitude is stabilized.":NULL);}}
    return terminal_guidance(g,t,state,course,p,aero,cfg,dt);
}
#undef guidance_update

