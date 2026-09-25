#include "guidance_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "taem_candidate_search.h"

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
    attitude.aoa_rate_rad_s = telemetry->pitch_rate * DEG2RAD;
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
 * reciprocal end uses an otherwise identical model with the reciprocal site.
 * Cached per snapshot; the plant tables are unchanged. */
static const TerminalModel *mm305_model_for_end(const TerminalModel *model,
        int runway_end) {
    static _Thread_local TerminalModel reciprocal;
    static _Thread_local uint64_t reciprocal_snapshot_id;
    static _Thread_local bool reciprocal_valid;
    if (runway_end!=1) return model;
    if (!reciprocal_valid || reciprocal_snapshot_id!=model->snapshot_id ||
        reciprocal.site.latitude!=runway_reciprocal_site(&model->site,
            model->world.radius_m).latitude) {
        reciprocal=*model;
        reciprocal.site=runway_reciprocal_site(&model->site,model->world.radius_m);
        reciprocal_snapshot_id=model->snapshot_id;
        reciprocal_valid=true;
    }
    return &reciprocal;
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

    if (!g->mm305_route_committed) {
        double frozen_hac_radius = g->taem_interface_target.valid &&
            isfinite(g->taem_interface_target.hac_radius) && g->taem_interface_target.hac_radius > 0.0 ?
            g->taem_interface_target.hac_radius : g->hac_radius;
        /* An end already committed upstream (MM304 handoff) is authoritative.
         * Otherwise both ends compete under the same fixed-HAC contract. */
        bool search_both_ends=cfg->site.allow_reciprocal_runway &&
            !g->runway_end_committed;
        int upstream_end=g->runway_end_preview_valid&&g->runway_end_index==1?1:0;
        TaemFixedHacSearch search=search_both_ends ?
            taem_fixed_hac_search_runway_ends(model,mm305_model_for_end(model,1),
                &current,frozen_hac_radius,200.0,0.5,1800.0) :
            taem_fixed_hac_search(mm305_model_for_end(model,upstream_end),
                &current,frozen_hac_radius,200.0,0.5,1800.0);
        if (!search_both_ends)
            for (int i=0;i<search.candidate_count;++i)
                search.candidates[i].runway_end=upstream_end;
        int selected=search.selected_candidate;
        if (selected<0 || selected>=search.candidate_count ||
            search.candidates[selected].status!=TAEM_PLAN_UNQUALIFIED ||
            !search.candidates[selected].route_built ||
            !search.candidates[selected].replay.path_constraints_ok) {
            char diagnostic[768];
            int used=snprintf(diagnostic,sizeof(diagnostic),"MM305 route rejected:");
            for (int i=0;i<search.candidate_count &&
                    used>0 && (size_t)used<sizeof(diagnostic);++i) {
                const TaemFixedHacCandidate *c=&search.candidates[i];
                used+=snprintf(diagnostic+used,sizeof(diagnostic)-(size_t)used,
                    " [%s %c] %s lat %.1f/%.1f t%.1f i%zu;",
                    c->runway_end==1?"RECIP":"CONF",c->side>0.0?'R':'L',
                    c->reason?c->reason:"unknown",
                    c->replay.failure_required_lateral_accel_mps2,
                    c->replay.failure_available_lateral_accel_mps2,
                    c->replay.elapsed_s,c->replay.failure_route_index);
            }
            return terminal_abort(g,diagnostic);
        }

        const TaemFixedHacCandidate *candidate=&search.candidates[selected];
        /* Replay qualifies the complete MM305 route through the HAC exit only.
         * Final tail qualification remains with the unchanged live contract. */
        g->mm305_route=candidate->route;
        g->runway_end_index=candidate->runway_end;
        g->runway_end_preview_valid=true;
        g->runway_end_committed=true;
        /* One machine-readable commitment record; observers must not infer
         * geometry from human wording or a retired fixed-270-degree log. */
        fprintf(stderr,"MM305_ROUTE {\"runwayEnd\":%d,\"side\":%.17g,"
            "\"radius\":%.17g,\"sweep\":%.17g,\"leadLength\":%.17g,\"arcLength\":%.17g,"
            "\"entry\":[%.17g,%.17g],\"center\":[%.17g,%.17g],\"exit\":[%.17g,%.17g],"
            "\"finalDistance\":%.17g}\n",
            candidate->runway_end,candidate->side,candidate->route.hac.radius_m,
            candidate->route.hac.arc_sweep_rad,candidate->route.lead_length_m,
            candidate->route.hac.arc_length_m,
            candidate->route.hac.entry.x,candidate->route.hac.entry.y,
            candidate->route.hac.center.x,candidate->route.hac.center.y,
            candidate->route.hac.exit.x,candidate->route.hac.exit.y,
            cfg->guidance.final_approach_distance);
        g->mm305_route_cursor=0;
        g->mm305_route_committed=true;
        g->mm305_hac_exit_reached=false;
        g->mm305_model_snapshot_id=model->snapshot_id;
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
    } else if (g->mm305_model_snapshot_id!=model->snapshot_id) {
        return terminal_abort(g,
            "The native MM305 model snapshot changed after route commitment.");
    }

    const TerminalModel *end_model=mm305_model_for_end(model,
        g->runway_end_index==1?1:0);
    TaemGeometryState geometry;
    if (!taem_geometry_state(end_model,&current,&geometry))
        return terminal_abort(g,
            "MM305 native state could not be projected into the fixed runway frame.");
    TaemPathReference reference;
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
    TaemTrackerOutput demand=taem_tracker_update(end_model,&current,&geometry,
        &reference,dt);
    if (!demand.valid)
        return terminal_abort(g,
            "Live MM305 tracker could not produce a bounded control command.");
    const char *diagnostics=getenv("KSP_LANDER_TAEM_DIAGNOSTICS");
    if (diagnostics && strcmp(diagnostics,"2")==0)
        fprintf(stderr,
            "TAEM live trace: ut=%.2f idx=%zu along=%.0f cross=%.0f h=%.0f course=%.1f ref=%.1f kappa=%.3g xt=%.0f crsErr=%.1f fpa=%.1f refFpa=%.1f bank=%.1f cmdBank=%.1f aoa=%.1f cmdAoa=%.1f dt=%.3f\n",
            current.ut_s,g->mm305_route_cursor,geometry.runway_along_m,
            geometry.runway_cross_m,geometry.altitude_above_runway_m,
            geometry.course_deg,reference.course_deg,
            reference.curvature_right_per_m,demand.cross_track_error_m,
            demand.course_error_deg,geometry.flight_path_angle_deg,
            reference.flight_path_angle_deg,current.attitude.bank_rad*RAD2DEG,
            demand.control.bank_rad*RAD2DEG,current.attitude.aoa_rad*RAD2DEG,
            demand.control.angle_of_attack_rad*RAD2DEG,dt);
    /* The committed route was qualified with the same saturated native attitude
     * response.  Do not abort on a one-tick lift-vector shortfall; abort only if
     * the actual vehicle leaves a wider emergency tracking corridor. */
    double emergency_cross_track_m=fmax(3000.0,0.25*g->mm305_route.hac.radius_m);
    if (fabs(demand.cross_track_error_m)>emergency_cross_track_m ||
        fabs(demand.course_error_deg)>45.0)
        return terminal_abort(g,
            "Live MM305 tracking diverged outside the replay-qualified corridor.");
    if (geometry.altitude_above_runway_m<=0.0)
        return terminal_abort(g,
            "MM305 crossed runway elevation before reaching the fixed-HAC exit.");

    double exit_distance=hypot(geometry.runway_along_m-g->mm305_route.hac.exit.x,
        geometry.runway_cross_m-g->mm305_route.hac.exit.y);
    if (g->mm305_route_cursor+2>=g->mm305_route.count &&
        exit_distance<=700.0 && fabs(demand.course_error_deg)<=12.0) {
        g->mm305_hac_exit_reached=true;
        g->hac_completed=true;
        g->hac_captured=true;
        g->hac_remaining=0.0;
    }

    GuidanceCommand command=atmospheric(t,reference.course_deg,
        demand.control.bank_rad*RAD2DEG,&cfg->vehicle,0.0,false,PROFILE_TAEM);
    command.heading_control_enabled=false;
    command.has_target_aoa=true;
    command.target_aoa=demand.control.angle_of_attack_rad*RAD2DEG;
    command.target_pitch=t->flight_path_angle+command.target_aoa;
    GuidanceResult result=stabilized(g,
        result_make(PHASE_TAEM,command,
            g->mm305_hac_exit_reached?
                "MM305 native tracker reached the fixed-HAC exit." :
                "MM305 native tracker is following the committed fixed-HAC route.",
            NULL),t,&cfg->vehicle,&cfg->guidance,dt);
    g->phase=PHASE_TAEM;
    return result;
}
