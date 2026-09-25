#include "guidance_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "taem_candidate_search.h"
#include "mm305_planning.h"

#include <stdlib.h>
#include <time.h>

/* Stage-neutral energy/physics helpers and the TAEM executive bridge are shared
 * by MM304, native MM305 fixed-HAC planning/tracking, and Final admission. */

#include "taem/state.inc"
#include "taem/energy.inc"
#include "taem/executive_bridge.inc"

double terminal_energy_commit_tolerance = NAN;

double terminal_test_speed_floor(const VehicleProfile *v) {
    return v ? fmax(65.0, v->touchdown_speed * .88) : 65.0;
}

double terminal_projected_lift_accel_at_aoa(const Telemetry *t,
        AerodynamicModel aero, const VehicleProfile *v, double aoa) {
    if (!t || !v || !isfinite(aoa)) return NAN;
    double target = clampd(aoa, 0.0, v->maximum_angle_of_attack);
    double lf = 0.0;
    aerodynamic_force_factors_mach(t->mach, target, v, &lf, NULL);
    double modeled = NAN;
    if (isfinite(t->dynamic_pressure) && t->dynamic_pressure >= 0.0 &&
        isfinite(aero.ballistic_coefficient) &&
        aero.ballistic_coefficient > DBL_MIN &&
        isfinite(aero.lift_to_drag) && aero.lift_to_drag >= 0.0)
        modeled = t->dynamic_pressure / aero.ballistic_coefficient *
            aero.lift_to_drag * fabs(lf);

    double incidence = hypot(t->angle_of_attack, t->sideslip);
    double measured = t->mass > DBL_MIN && isfinite(t->lift_force) &&
        t->lift_force >= 0.0 ? t->lift_force / t->mass : NAN;
    double same_incidence = sqrt(DBL_EPSILON) * fmax(1.0, fabs(incidence));
    if (isfinite(measured) && fabs(target - incidence) <= same_incidence)
        return measured;
    return isfinite(modeled) && modeled >= 0.0 ? modeled : NAN;
}

bool guidance_begin_hac_test(GuidanceMachine *g, const Telemetry *t,
        double course, const PlanetModel *planet, AerodynamicModel aero,
        const LandingConfiguration *cfg, char *message, size_t message_size) {
    if (!g || !t || !planet || !cfg || !isfinite(course) ||
        !(t->radar_altitude > 0.0) || !(t->true_air_speed > 0.0) ||
        !isfinite(t->ut) || !isfinite(t->mean_altitude)) {
        if (message && message_size)
            snprintf(message, message_size,
                "MM305 test requires a valid airborne state and guidance configuration.");
        return false;
    }

    guidance_machine_init(g);
    guidance_set_engaged(g, true);
    terminal_glide_initialize(g, &cfg->vehicle, &cfg->guidance);

    /* This fixture starts at the MM305 boundary, so record the prior Entry
     * stages as checkpoint-complete while keeping HAC commitment and Final
     * delivery unset. The normal terminal controller and its Final contract
     * still decide whether either ownership transition is earned. */
    g->has_burn_command_started = true;
    g->deorbit_burn_completed = true;
    g->atmospheric_interface_crossed = true;
    g->terminal_region_entered = true;
    g->terminal_test_capture_active = true;
    g->hac_radius = cfg->guidance.hac_radius;
    g->hac_side = 0.0;
    g->terminal_final_handoff_latched = false;
    g->terminal_path_committed = false;
    g->hac_completed = false;
    g->final_approach_captured = false;

    if (!taem_exec_enter(g, t, &cfg->guidance, planet, aero, cfg, true)) {
        guidance_machine_init(g);
        if (message && message_size)
            snprintf(message, message_size,
                "MM305 checkpoint could not initialize TAEM ownership.");
        return false;
    }
    g->phase = PHASE_TAEM;
    if (message && message_size)
        snprintf(message, message_size,
            "MM305 checkpoint accepted; HAC selection and Final delivery remain subject to the live guidance gates.");
    return true;
}

static TerminalDynamicState terminal_live_state(const VehicleState *state,
        const Telemetry *telemetry,const TerminalModel *model) {
    AttitudeModel attitude = model->attitude;
    attitude.aoa_rad = telemetry->angle_of_attack * DEG2RAD;
    attitude.bank_rad = telemetry->roll * DEG2RAD;
    /* Controlled-coordinate rates: AoA rate, not body pitch rate. */
    attitude.aoa_rate_rad_s = (telemetry->has_angle_of_attack_rate &&
        isfinite(telemetry->angle_of_attack_rate) ? telemetry->angle_of_attack_rate :
        telemetry->pitch_rate) * DEG2RAD;
    attitude.bank_rate_rad_s = telemetry->roll_rate * DEG2RAD;
    attitude.cmd_aoa_rad = attitude.requested_aoa_rad = attitude.aoa_rad;
    attitude.cmd_bank_rad = attitude.requested_bank_rad = attitude.bank_rad;
    return (TerminalDynamicState){
        .position_i_m = {state->position.x,state->position.y,state->position.z},
        .velocity_i_mps = {state->velocity.x,state->velocity.y,state->velocity.z},
        .attitude = attitude,
        .mass_kg = state->mass,
        .ut_s = state->ut
    };
}

/* The frozen terminal model carries the configured runway end.  MM305 frames
 * its route, tracker and replay in the runway end it commits to, so the
 * reciprocal end uses an otherwise identical model with the reciprocal site. */
static void mm305_reciprocal_model(const TerminalModel *model, TerminalModel *out) {
    *out = *model;
    out->site = runway_reciprocal_site(&model->site, model->world.radius_m);
}

static const TerminalModel *mm305_model_for_end(const TerminalModel *model,
        int runway_end) {
    static _Thread_local TerminalModel reciprocal;
    static _Thread_local uint64_t reciprocal_snapshot_id;
    static _Thread_local bool reciprocal_valid;
    if (runway_end!=1) return model;
    if (!reciprocal_valid || reciprocal_snapshot_id!=model->snapshot_id ||
        reciprocal.site.latitude!=runway_reciprocal_site(&model->site,
            model->world.radius_m).latitude) {
        mm305_reciprocal_model(model,&reciprocal);
        reciprocal_snapshot_id=model->snapshot_id;
        reciprocal_valid=true;
    }
    return &reciprocal;
}

/* Scale the planning model's lift/drag toward the measured vehicle.  Crude
 * (one global factor per force) but it closes the loop on model error: each
 * replan starts from the live state with the live force ratio. */
static void mm305_scale_model(TerminalModel *m, double lift_scale, double drag_scale) {
    if (!(fabs(lift_scale-1.0)>1e-9) && !(fabs(drag_scale-1.0)>1e-9)) return;
    for (size_t i=0;i<m->aero.mach_count;++i)
        for (size_t j=0;j<m->aero.alpha_count;++j) {
            m->aero.cl[i][j]*=lift_scale;
            m->aero.cd[i][j]*=drag_scale;
        }
    for (size_t i=0;i<m->aero.book_count;++i) {
        m->aero.book[i].lift_per_q_m2*=lift_scale;
        m->aero.book[i].drag_per_q_m2*=drag_scale;
    }
}

static double mm305_wall_seconds(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC,&ts)!=0) return 0.0;
    return (double)ts.tv_sec+(double)ts.tv_nsec/1e9;
}

static bool mm305_candidate_ok(const TaemFixedHacCandidate *c) {
    return c->status==TAEM_PLAN_UNQUALIFIED && c->route_built &&
        c->replay.path_constraints_ok;
}

Mm305PlanResult mm305_plan(const TerminalModel *model, const Mm305PlanRequest *request) {
    Mm305PlanResult out;
    memset(&out,0,sizeof(out));
    if (!model || !request || !request->valid || !model->replay_validated) {
        snprintf(out.diagnostic,sizeof(out.diagnostic),"MM305 plan request invalid");
        return out;
    }
    double started=mm305_wall_seconds();
    out.request_ut=request->request_ut;
    out.model_snapshot_id=request->model_snapshot_id;
    TerminalModel *scaled=malloc(sizeof(*scaled));
    TerminalModel *reciprocal=malloc(sizeof(*reciprocal));
    if (!scaled || !reciprocal) {
        free(scaled);free(reciprocal);
        snprintf(out.diagnostic,sizeof(out.diagnostic),"MM305 plan allocation failed");
        return out;
    }
    *scaled=*model;
    mm305_scale_model(scaled,clampd(request->lift_scale,0.5,2.0),
        clampd(request->drag_scale,0.5,2.0));
    mm305_reciprocal_model(scaled,reciprocal);
    /* HAC size is the route's energy lever: a larger HAC flies a longer path
       and sheds more energy, a smaller one less.  The candidate search already
       tightens the radius when the configured one fails; add the larger HAC it
       never tries, for states with energy to spare. */
    const double radius_factors[]={1.0,1.5};
    TaemFixedHacSearch search;
    memset(&search,0,sizeof(search));
    int best=-1;
    for (size_t r=0;r<sizeof(radius_factors)/sizeof(radius_factors[0])&&best<0;++r) {
        double radius=request->hac_radius_m*radius_factors[r];
        if (!(radius>=2000.0)) continue;
        search=request->search_both_ends ?
            taem_fixed_hac_search_runway_ends(scaled,reciprocal,&request->state,
                radius,200.0,0.5,1800.0) :
            taem_fixed_hac_search(request->upstream_end==1?reciprocal:scaled,
                &request->state,radius,200.0,0.5,1800.0);
        if (!request->search_both_ends)
            for (int i=0;i<search.candidate_count;++i)
                search.candidates[i].runway_end=request->upstream_end;
        for (int i=0;i<search.candidate_count;++i) {
            const TaemFixedHacCandidate *c=&search.candidates[i];
            if (!mm305_candidate_ok(c)) continue;
            if (request->restrict_side && c->side*request->side<=0.0) continue;
            if (best<0 || (i==search.selected_candidate) ||
                (search.selected_candidate<0 && c->quality_score<search.candidates[best].quality_score))
                best=i;
            if (i==search.selected_candidate) break;
        }
    }
    out.valid=true;
    if (best>=0) {
        out.found=true;
        out.candidate=search.candidates[best];
    }
    int used=snprintf(out.diagnostic,sizeof(out.diagnostic),"MM305 route %s:",
        out.found?"found":"rejected");
    for (int i=0;i<search.candidate_count && used>0 && (size_t)used<sizeof(out.diagnostic);++i) {
        const TaemFixedHacCandidate *c=&search.candidates[i];
        used+=snprintf(out.diagnostic+used,sizeof(out.diagnostic)-(size_t)used,
            " [%s %c] %s t%.1f;",c->runway_end==1?"RECIP":"CONF",c->side>0.0?'R':'L',
            c->reason?c->reason:"unknown",c->replay.elapsed_s);
    }
    free(scaled);free(reciprocal);
    out.solve_wall_s=mm305_wall_seconds()-started;
    return out;
}

Mm305PlanRequest guidance_mm305_plan_request(const GuidanceMachine *g,
        const Telemetry *t, const VehicleState *state,
        const LandingConfiguration *cfg, const TerminalModel *model) {
    Mm305PlanRequest r;
    memset(&r,0,sizeof(r));
    if (!g || !t || !state || !cfg || !model || !model->replay_validated ||
        !g->mm305_planning_needed)
        return r;
    r.state=terminal_live_state(state,t,model);
    if (!terminal_state_validate(&r.state,NULL,0)) return r;
    r.request_ut=t->ut;
    r.model_snapshot_id=model->snapshot_id;
    r.hac_radius_m=g->taem_interface_target.valid &&
        isfinite(g->taem_interface_target.hac_radius) &&
        g->taem_interface_target.hac_radius>0.0 ?
        g->taem_interface_target.hac_radius : g->hac_radius;
    if (!(r.hac_radius_m>0.0)) r.hac_radius_m=cfg->guidance.hac_radius;
    r.search_both_ends=cfg->site.allow_reciprocal_runway && !g->runway_end_committed;
    r.upstream_end=g->runway_end_preview_valid&&g->runway_end_index==1?1:0;
    r.restrict_side=g->mm305_route_committed||g->mm305_replans>0;
    r.side=g->hac_side;
    r.lift_scale=isfinite(g->mm305_lift_scale)&&g->mm305_lift_scale>0.0?g->mm305_lift_scale:1.0;
    r.drag_scale=isfinite(g->mm305_drag_scale)&&g->mm305_drag_scale>0.0?g->mm305_drag_scale:1.0;
    r.valid=true;
    return r;
}

bool guidance_mm305_accept_plan(GuidanceMachine *g, const Mm305PlanResult *result,
        const LandingConfiguration *cfg) {
    if (!g || !result || !result->valid || !cfg) return false;
    g->mm305_planning_needed=false;
    if (g->phase!=PHASE_TAEM || g->mm305_hac_exit_reached || g->hac_completed ||
        g->final_approach_captured)
        return false;
    if (!result->found) {
        g->mm305_plan_failures++;
        fprintf(stderr,"%s\n",result->diagnostic);
        return false;
    }
    const TaemFixedHacCandidate *candidate=&result->candidate;
    bool replan=g->mm305_route_committed;
    g->mm305_route=candidate->route;
    g->runway_end_index=candidate->runway_end;
    g->runway_end_preview_valid=true;
    g->runway_end_committed=true;
    fprintf(stderr,"MM305_ROUTE {\"runwayEnd\":%d,\"side\":%.17g,"
        "\"radius\":%.17g,\"sweep\":%.17g,\"leadLength\":%.17g,\"arcLength\":%.17g,"
        "\"entry\":[%.17g,%.17g],\"center\":[%.17g,%.17g],\"exit\":[%.17g,%.17g],"
        "\"finalDistance\":%.17g,\"replan\":%s,\"liftScale\":%.4f,\"dragScale\":%.4f,"
        "\"solveWall\":%.3f}\n",
        candidate->runway_end,candidate->side,candidate->route.hac.radius_m,
        candidate->route.hac.arc_sweep_rad,candidate->route.lead_length_m,
        candidate->route.hac.arc_length_m,
        candidate->route.hac.entry.x,candidate->route.hac.entry.y,
        candidate->route.hac.center.x,candidate->route.hac.center.y,
        candidate->route.hac.exit.x,candidate->route.hac.exit.y,
        cfg->guidance.final_approach_distance,replan?"true":"false",
        g->mm305_lift_scale,g->mm305_drag_scale,result->solve_wall_s);
    g->mm305_route_cursor=0;
    g->mm305_route_committed=true;
    g->mm305_hac_exit_reached=false;
    g->mm305_model_snapshot_id=result->model_snapshot_id;
    g->mm305_last_route_ut=result->request_ut;
    if (replan) g->mm305_replans++;
    g->hac_side=candidate->side;
    g->hac_side_selected=true;
    g->hac_radius=candidate->route.hac.radius_m;
    g->hac_remaining=candidate->route.length_m;
    g->hac_progress_valid=true;
    g->terminal_path_kind=TERMINAL_PATH_HAC;
    g->terminal_path_committed=true;
    g->terminal_final_handoff_latched=true;
    g->terminal_final_handoff_distance=cfg->guidance.final_approach_distance;
    g->hac_completed=false;
    g->hac_captured=true;
    return true;
}

/* Low-pass the measured/model force ratio at the current state. */
static void mm305_observe_force_scale(GuidanceMachine *g, const Telemetry *t,
        const TerminalModel *model, const TerminalDynamicState *current, double dt) {
    if (!t->physics_sample_valid || !(t->mass>0.0) || !(t->dynamic_pressure>200.0)) return;
    AeroForces modeled=aero_compute(&model->world,&model->aero,current->position_i_m,
        current->velocity_i_mps,current->ut_s,current->mass_kg,
        current->attitude.aoa_rad,current->attitude.bank_rad);
    if (!(modeled.lift_n>1.0) || !(modeled.drag_n>1.0)) return;
    double lift_ratio=clampd(fabs(t->lift_force)/modeled.lift_n,0.5,2.0);
    double drag_ratio=clampd(fabs(t->drag_force)/modeled.drag_n,0.5,2.0);
    double a=clampd(dt/(8.0+dt),0.0,1.0);
    if (!(g->mm305_lift_scale>0.0)) g->mm305_lift_scale=1.0;
    if (!(g->mm305_drag_scale>0.0)) g->mm305_drag_scale=1.0;
    g->mm305_lift_scale+=a*(lift_ratio-g->mm305_lift_scale);
    g->mm305_drag_scale+=a*(drag_ratio-g->mm305_drag_scale);
}

/* Acquisition law flown while no qualified route is held: head for the
 * upstream HAC region on the extended centreline, descending on a bounded
 * geometric path, using the same model-inverted tracker as the route. */
static bool mm305_acquisition_command(const TerminalModel *model,
        const TerminalDynamicState *current, const TaemGeometryState *geometry,
        double hac_radius, double dt, TaemTrackerOutput *demand,
        TaemPathReference *reference) {
    double target_along=-(model->guidance.final_approach_distance+hac_radius);
    double dx=target_along-geometry->runway_along_m;
    double dy=-geometry->runway_cross_m;
    double distance=hypot(dx,dy);
    double bearing=model->site.runway_heading+atan2(dy,dx)*RAD2DEG;
    double course_error=remainder(bearing-geometry->course_deg,360.0);
    /* Turn toward the target at up to a quarter-turn per ~40 s of flight. */
    double speed=fmax(geometry->ground_speed_mps,1.0);
    double curvature=clampd(course_error*DEG2RAD/(40.0*speed),-1.0/3000.0,1.0/3000.0);
    double target_altitude=model->site.altitude+
        model->guidance.final_approach_distance*tan(model->guidance.final_glide_slope*DEG2RAD)+
        M_PI*hac_radius*tan(fmin(model->guidance.taem_glide_slope,12.0)*DEG2RAD);
    double height=model->site.altitude+geometry->altitude_above_runway_m-target_altitude;
    double fpa=-clampd(atan2(fmax(height,0.0),fmax(distance,2000.0))*RAD2DEG,2.0,18.0);
    *reference=(TaemPathReference){
        .runway_along_m=geometry->runway_along_m,
        .runway_cross_m=geometry->runway_cross_m,
        .course_deg=geometry->course_deg,
        .curvature_right_per_m=curvature,
        .altitude_m=NAN,
        .flight_path_angle_deg=fpa,
        .vertical_curvature_per_m=0.0,
        .station_m=0.0
    };
    *demand=taem_tracker_update(model,current,geometry,reference,dt);
    return demand->valid;
}

GuidanceResult taem_guidance_native(GuidanceMachine *g,const Telemetry *t,
        const VehicleState *vehicle_state,double course,const PlanetModel *planet,
        AerodynamicModel aero,const LandingConfiguration *cfg,
        const TerminalModel *model,double dt) {
    (void)planet;
    (void)aero;
    if (!g || !t || !vehicle_state || !cfg || !model || !model->replay_validated)
        return terminal_abort(g,
            "MM305 native fixed-HAC model is unavailable; no legacy HAC fallback is permitted.");

    if (g->terminal_final_test_mode && g->hac_completed)
        return result_make(PHASE_TAEM, atmospheric(t,course,0.0,
            &cfg->vehicle,0.0,false,PROFILE_TAEM),
            "MM305 native HAC exit reached; awaiting the existing numeric Final contract.",NULL);

    TerminalDynamicState current=terminal_live_state(vehicle_state,t,model);
    if (!terminal_state_validate(&current,NULL,0))
        return terminal_abort(g,
            "MM305 native state could not be projected into the fixed runway frame.");
    if (g->mm305_route_committed && g->mm305_model_snapshot_id!=model->snapshot_id)
        return terminal_abort(g,
            "The native MM305 model snapshot changed after route commitment.");
    mm305_observe_force_scale(g,t,model,&current,dt);

    const double replan_interval_s=20.0;
    const double retry_interval_s=10.0;
    if (!g->mm305_route_committed) {
        /* Back off after failed searches: a failing search is the most
           expensive one (every radius, end and side is tried). */
        double backoff=retry_interval_s*pow(2.0,(double)(g->mm305_plan_failures<3?
            g->mm305_plan_failures:3));
        bool due=!isfinite(g->mm305_last_plan_attempt_ut) ||
            t->ut-g->mm305_last_plan_attempt_ut>=backoff;
        if (due && !g->mm305_planning_needed) {
            g->mm305_planning_needed=true;
            g->mm305_last_plan_attempt_ut=t->ut;
            g->mm305_plan_request_ut=t->ut;
        }
        if (g->mm305_planning_needed && !g->mm305_async_planning) {
            /* No worker (offline tools/tests): plan synchronously, but only at
               the retry cadence so a failing search cannot stall every tick. */
            Mm305PlanRequest request=guidance_mm305_plan_request(g,t,vehicle_state,cfg,model);
            Mm305PlanResult result=mm305_plan(model,&request);
            if (!guidance_mm305_accept_plan(g,&result,cfg))
                g->mm305_planning_needed=false;
        }
    } else if (g->mm305_async_planning && !g->mm305_planning_needed &&
               t->ut-g->mm305_last_plan_attempt_ut>=replan_interval_s) {
        /* Periodic replan from the measured state with the live force scale:
           the only energy lever left once bank and AoA are spent on path
           tracking is the route itself.  Not on the last HAC segment. */
        double remaining=taem_route_remaining_at_index(&g->mm305_route,g->mm305_route_cursor);
        if (isfinite(remaining) && remaining>fmax(8000.0,0.35*g->mm305_route.hac.arc_length_m)) {
            g->mm305_planning_needed=true;
            g->mm305_last_plan_attempt_ut=t->ut;
            g->mm305_plan_request_ut=t->ut;
        }
    }

    int end=g->mm305_route_committed&&g->runway_end_index==1?1:
        (g->runway_end_preview_valid&&g->runway_end_index==1?1:0);
    const TerminalModel *end_model=mm305_model_for_end(model,end);
    TaemGeometryState geometry;
    if (!taem_geometry_state(end_model,&current,&geometry))
        return terminal_abort(g,
            "MM305 native state could not be projected into the fixed runway frame.");
    if (geometry.altitude_above_runway_m<=0.0)
        return terminal_abort(g,
            "MM305 crossed runway elevation before reaching the fixed-HAC exit.");

    TaemPathReference reference;
    TaemTrackerOutput demand;
    const char *status;
    if (g->mm305_route_committed) {
        if (!taem_route_reference(&g->mm305_route,&geometry,
                &g->mm305_route_cursor,&reference,NULL))
            return terminal_abort(g,
                "Committed MM305 route could not produce a tracker reference.");
        double route_remaining=taem_route_remaining_at_index(
            &g->mm305_route,g->mm305_route_cursor);
        if (!isfinite(route_remaining))
            return terminal_abort(g,"Committed MM305 route station became non-finite.");
        g->hac_remaining=route_remaining;
        g->hac_progress_valid=true;
        g->hac_captured=true;
        demand=taem_tracker_update(end_model,&current,&geometry,&reference,dt);
        if (!demand.valid)
            return terminal_abort(g,
                "Live MM305 tracker could not produce a bounded control command.");
        double emergency_cross_track_m=fmax(3000.0,0.25*g->mm305_route.hac.radius_m);
        if (fabs(demand.cross_track_error_m)>emergency_cross_track_m ||
            fabs(demand.course_error_deg)>45.0) {
            /* Leaving the qualified corridor invalidates the route, not the
               mission: release it, replan from the measured state and fly the
               acquisition law meanwhile.  Only a state too low to re-plan aborts. */
            if (geometry.altitude_above_runway_m<1500.0)
                return terminal_abort(g,
                    "Live MM305 tracking diverged below the height needed to re-plan.");
            fprintf(stderr,"MM305 route released: cross %.0f m course %.1f deg; re-planning.\n",
                demand.cross_track_error_m,demand.course_error_deg);
            g->mm305_route_committed=false;
            g->terminal_path_committed=false;
            g->hac_captured=false;
            g->mm305_planning_needed=true;
            g->mm305_last_plan_attempt_ut=t->ut;
            g->mm305_plan_request_ut=t->ut;
        }
        double exit_distance=hypot(geometry.runway_along_m-g->mm305_route.hac.exit.x,
            geometry.runway_cross_m-g->mm305_route.hac.exit.y);
        if (g->mm305_route_committed && g->mm305_route_cursor+2>=g->mm305_route.count &&
            exit_distance<=700.0 && fabs(demand.course_error_deg)<=12.0) {
            g->mm305_hac_exit_reached=true;
            g->hac_completed=true;
            g->hac_captured=true;
            g->hac_remaining=0.0;
        }
        status=g->mm305_hac_exit_reached?
            "MM305 native tracker reached the fixed-HAC exit." :
            "MM305 native tracker is following the committed fixed-HAC route.";
    }
    if (!g->mm305_route_committed) {
        double hac_radius=g->hac_radius>=3000.0?g->hac_radius:cfg->guidance.hac_radius;
        if (!mm305_acquisition_command(end_model,&current,&geometry,hac_radius,dt,
                &demand,&reference))
            return terminal_abort(g,
                "MM305 acquisition law could not produce a bounded control command.");
        if (geometry.altitude_above_runway_m<1000.0 && g->mm305_plan_failures>0)
            return terminal_abort(g,
                "MM305 found no qualified route before the height needed to fly one.");
        status="MM305 acquisition: flying toward the HAC region while a route is planned.";
    }

    const char *diagnostics=getenv("KSP_LANDER_TAEM_DIAGNOSTICS");
    if (diagnostics && strcmp(diagnostics,"2")==0)
        fprintf(stderr,
            "TAEM live trace: ut=%.2f committed=%d idx=%zu along=%.0f cross=%.0f h=%.0f course=%.1f ref=%.1f kappa=%.3g xt=%.0f crsErr=%.1f fpa=%.1f refFpa=%.1f bank=%.1f cmdBank=%.1f aoa=%.1f cmdAoa=%.1f scales=%.3f/%.3f\n",
            current.ut_s,g->mm305_route_committed,g->mm305_route_cursor,geometry.runway_along_m,
            geometry.runway_cross_m,geometry.altitude_above_runway_m,
            geometry.course_deg,reference.course_deg,
            reference.curvature_right_per_m,demand.cross_track_error_m,
            demand.course_error_deg,geometry.flight_path_angle_deg,
            reference.flight_path_angle_deg,current.attitude.bank_rad*RAD2DEG,
            demand.control.bank_rad*RAD2DEG,current.attitude.aoa_rad*RAD2DEG,
            demand.control.angle_of_attack_rad*RAD2DEG,
            g->mm305_lift_scale,g->mm305_drag_scale);

    GuidanceCommand command=atmospheric(t,reference.course_deg,
        demand.control.bank_rad*RAD2DEG,&cfg->vehicle,0.0,false,PROFILE_TAEM);
    command.heading_control_enabled=false;
    command.has_target_aoa=true;
    command.target_aoa=demand.control.angle_of_attack_rad*RAD2DEG;
    command.target_pitch=t->flight_path_angle+command.target_aoa;
    GuidanceResult result=stabilized(g,
        result_make(PHASE_TAEM,command,status,NULL),t,&cfg->vehicle,&cfg->guidance,dt);
    g->phase=PHASE_TAEM;
    return result;
}
