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
    g->hac_side = terminal_default_hac_side(t, &cfg->site, course);
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
    TaemGeometryState geometry;
    if (!terminal_state_validate(&current,NULL,0) ||
        !taem_geometry_state(model,&current,&geometry))
        return terminal_abort(g,
            "MM305 native state could not be projected into the fixed runway frame.");

    if (!g->mm305_route_committed) {
        TaemFixedHacSearch search=taem_fixed_hac_search(model,&current,
            200.0,0.25,1800.0);
        int selected=search.selected_candidate;
        if (selected<0 || selected>=2 ||
            search.candidates[selected].status!=TAEM_PLAN_UNQUALIFIED ||
            !search.candidates[selected].route_built ||
            !search.candidates[selected].replay.path_constraints_ok) {
            char diagnostic[384];
            snprintf(diagnostic,sizeof(diagnostic),
                "MM305 route rejected: L %s lat %.1f/%.1f; R %s lat %.1f/%.1f vert %.1f (%.1f/%.1f) at %.1fs h%.0f i%zu; bow%+.0f local%+.0f@%.2f^%.2f-%.2f sag%+.0f target h%.0f/FPA%.1f",
                search.candidates[0].reason?search.candidates[0].reason:"unknown",
                search.candidates[0].required_lateral_accel_mps2,
                search.candidates[0].available_lateral_accel_mps2,
                search.candidates[1].reason?search.candidates[1].reason:"unknown",
                search.candidates[1].required_lateral_accel_mps2,
                search.candidates[1].available_lateral_accel_mps2,
                search.candidates[1].replay.maximum_vertical_authority_shortfall_mps2,
                search.candidates[1].replay.failure_required_vertical_lift_mps2,
                search.candidates[1].replay.failure_delivered_vertical_lift_mps2,
                search.candidates[1].replay.elapsed_s,
                search.candidates[1].replay.final_geometry.altitude_above_runway_m,
                search.candidates[1].replay.failure_route_index,
                search.candidates[1].route.profile_midpoint_offset_m,
                search.candidates[1].route.profile_local_offset_m,
                search.candidates[1].route.profile_local_start_fraction,
                search.candidates[1].route.profile_local_peak_fraction,
                search.candidates[1].route.profile_local_end_fraction,
                search.candidates[1].route.profile_initial_sag_m,
                search.candidates[1].replay.failure_target_altitude_m,
                search.candidates[1].replay.failure_target_flight_path_angle_deg);
            return terminal_abort(g,diagnostic);
        }

        const TaemFixedHacCandidate *candidate=&search.candidates[selected];
        /* Replay qualifies the complete MM305 route through the HAC exit only.
         * Final tail qualification remains with the unchanged live contract. */
        g->mm305_route=candidate->route;
        g->mm305_route_cursor=0;
        g->mm305_route_committed=true;
        g->mm305_hac_exit_reached=false;
        g->mm305_model_snapshot_id=model->snapshot_id;
        g->hac_side=candidate->side;
        g->hac_side_selected=true;
        g->hac_radius=candidate->route.hac.radius_m;
        g->hac_remaining=candidate->route.hac.arc_length_m;
        g->terminal_path_kind=TERMINAL_PATH_HAC;
        g->terminal_path_committed=true;
        g->terminal_final_handoff_latched=true;
        g->terminal_final_handoff_distance=cfg->guidance.final_approach_distance;
        g->terminal_candidate.valid=false;
        g->hac_completed=false;
        g->hac_captured=false;
    } else if (g->mm305_model_snapshot_id!=model->snapshot_id) {
        return terminal_abort(g,
            "The native MM305 model snapshot changed after route commitment.");
    }

    TaemPathReference reference;
    if (!taem_route_reference(&g->mm305_route,&geometry,
            &g->mm305_route_cursor,&reference,NULL))
        return terminal_abort(g,
            "Committed MM305 route could not produce a tracker reference.");
    TaemTrackerOutput demand=taem_tracker_update(model,&current,&geometry,
        &reference,dt);
    if (!demand.valid || !demand.lateral_authority_ok ||
        !demand.vertical_authority_ok)
        return terminal_abort(g,
            "Live MM305 tracker demand left its replay-qualified control envelope.");
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
