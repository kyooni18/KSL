#include "guidance_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "taem_candidate_search.h"
#include "taem_alignment.h"
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

static bool guidance_begin_mm305_restart(GuidanceMachine *g, const Telemetry *t,
        double course, const PlanetModel *planet, AerodynamicModel aero,
        const LandingConfiguration *cfg, bool test_mode,
        char *message, size_t message_size) {
    if (!g || !t || !planet || !cfg || !isfinite(course) ||
        !(t->radar_altitude > 0.0) || !(t->true_air_speed > 0.0) ||
        !isfinite(t->ut) || !isfinite(t->mean_altitude)) {
        if (message && message_size)
            snprintf(message, message_size,
                "MM305 restart requires a valid airborne state and guidance configuration.");
        return false;
    }

    guidance_machine_init(g);
    guidance_set_engaged(g, true);
    terminal_glide_initialize(g, &cfg->vehicle, &cfg->guidance);

    /* A late-terminal restart adopts MM305 ownership from the measured state.
       It does not manufacture a route or a Final handoff: native MM305 must still
       plan and replay-qualify its route, then satisfy the existing Final contract. */
    g->has_burn_command_started = true;
    g->deorbit_burn_completed = true;
    g->atmospheric_interface_crossed = true;
    g->terminal_region_entered = true;
    g->terminal_test_capture_active = test_mode;
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
                "MM305 live state could not initialize TAEM ownership.");
        return false;
    }
    g->phase = PHASE_TAEM;
    if (message && message_size)
        snprintf(message, message_size,
            test_mode ?
                "MM305 checkpoint accepted; HAC selection and Final delivery remain subject to the live guidance gates." :
                "MM305 continuation adopted from the live state; route selection and Final delivery remain subject to the normal guidance gates.");
    return true;
}

bool guidance_begin_mm305_continuation(GuidanceMachine *g, const Telemetry *t,
        double course, const PlanetModel *planet, AerodynamicModel aero,
        const LandingConfiguration *cfg, char *message, size_t message_size) {
    return guidance_begin_mm305_restart(g, t, course, planet, aero, cfg, false,
        message, message_size);
}

bool guidance_begin_hac_test(GuidanceMachine *g, const Telemetry *t,
        double course, const PlanetModel *planet, AerodynamicModel aero,
        const LandingConfiguration *cfg, char *message, size_t message_size) {
    return guidance_begin_mm305_restart(g, t, course, planet, aero, cfg, true,
        message, message_size);
}

/* TAEM elevator trim model.  With the body flap removed from pitch the
 * elevators alone balance the static pitching moment; both scale with q, so
 * the trim input is a q-independent line u = a + b*AoA.  The slope is the
 * clean live identification at the STS-N MM305 checkpoint (M2.3-2.5,
 * elevator-only: u 0.00 held AoA -2.4 deg, u 0.44 held 7.0 deg for 10 s).
 * The intercept drifts with Mach and is re-identified only from settled
 * tracking (AoA steady and on its command), where the input is pure trim.
 * The highest AoA the elevators can hold while keeping a quarter of their
 * travel for manoeuvring bounds what TAEM may plan and command. */
static const double mm305_trim_slope_per_deg = 0.047;
static const double mm305_trim_prior_intercept = 0.113;
static const double mm305_trim_intercept_memory_s = 10.0;
static const double mm305_trim_input_budget = 0.75;

static double mm305_trim_intercept(const GuidanceMachine *g) {
    return g && g->mm305_trim_initialized && isfinite(g->mm305_trim_intercept) ?
        g->mm305_trim_intercept : mm305_trim_prior_intercept;
}

static void mm305_trim_observe(GuidanceMachine *g, const Telemetry *t) {
    if (!g || !t) return;
    if (!g->mm305_trim_initialized) {
        g->mm305_trim_intercept = mm305_trim_prior_intercept;
        g->mm305_trim_initialized = true;
        g->mm305_trim_last_ut = t->ut;
        return;
    }
    double dt = isfinite(g->mm305_trim_last_ut) ? t->ut - g->mm305_trim_last_ut : 0.0;
    g->mm305_trim_last_ut = t->ut;
    if (!(dt > 0.0) || dt > 2.0 || !g->mm305_command_valid) return;
    double aoa_rate = t->has_angle_of_attack_rate && isfinite(t->angle_of_attack_rate) ?
        t->angle_of_attack_rate : t->pitch_rate;
    double command_error = g->mm305_command_aoa_rad * RAD2DEG - t->angle_of_attack;
    bool settled = t->has_controls && isfinite(t->control_pitch) &&
        fabs(t->control_pitch) < 0.95 && isfinite(aoa_rate) && fabs(aoa_rate) < 0.3 &&
        fabs(command_error) < 0.5 && t->dynamic_pressure > 2000.0;
    if (!settled) return;
    double observed = t->control_pitch - mm305_trim_slope_per_deg * t->angle_of_attack;
    double alpha = 1.0 - exp(-dt / mm305_trim_intercept_memory_s);
    g->mm305_trim_intercept += alpha * (observed - g->mm305_trim_intercept);
}

static double mm305_trim_input(const GuidanceMachine *g, double aoa_deg) {
    return mm305_trim_intercept(g) + mm305_trim_slope_per_deg * aoa_deg;
}

/* Trim-limited AoA ceiling in degrees. */
static double mm305_trim_aoa_ceiling_deg(const GuidanceMachine *g) {
    return fmax(4.0, (mm305_trim_input_budget - mm305_trim_intercept(g)) /
                     mm305_trim_slope_per_deg);
}

static TerminalDynamicState terminal_live_state(const GuidanceMachine *g,
        const VehicleState *state, const Telemetry *telemetry,
        const TerminalModel *model) {
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
    /* Continue the shaped command only while MM305 has owned every recent
       tick; after any other owner (e.g. attitude recovery) the command restarts
       from the measured attitude instead of a stale target. */
    if (g && g->mm305_command_valid && isfinite(g->mm305_command_ut) &&
        telemetry->ut - g->mm305_command_ut >= 0.0 &&
        telemetry->ut - g->mm305_command_ut <= 1.0) {
        attitude.cmd_bank_rad = attitude.requested_bank_rad =
            g->mm305_command_bank_rad;
        attitude.cmd_aoa_rad = attitude.requested_aoa_rad =
            g->mm305_command_aoa_rad;
    }
    double trim_ceiling = mm305_trim_aoa_ceiling_deg(g);
    return (TerminalDynamicState){
        .trim_aoa_ceiling_rad = isfinite(trim_ceiling) ? trim_ceiling * DEG2RAD : 0.0,
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

/* Apply each measured force ratio only near the Mach where it was observed.
 * Live MM305 drag has a strong Mach dependence, so extrapolating one ratio
 * across the complete table can badly misprice the low-subsonic HAC exit. */
static double mm305_mach_scale_weight(double mach, double observed_mach) {
    const double support_width=0.5;
    if (!isfinite(mach) || !isfinite(observed_mach)) return 0.0;
    return clampd(1.0-fabs(mach-observed_mach)/support_width,0.0,1.0);
}

static void mm305_scale_model(TerminalModel *m, double observed_mach,
        double lift_scale, double drag_scale) {
    if (!(fabs(lift_scale-1.0)>1e-9) && !(fabs(drag_scale-1.0)>1e-9)) return;
    for (size_t i=0;i<m->aero.mach_count;++i) {
        double weight=mm305_mach_scale_weight(m->aero.mach[i],observed_mach);
        double local_lift_scale=1.0+weight*(lift_scale-1.0);
        double local_drag_scale=1.0+weight*(drag_scale-1.0);
        for (size_t j=0;j<m->aero.alpha_count;++j) {
            m->aero.cl[i][j]*=local_lift_scale;
            m->aero.cd[i][j]*=local_drag_scale;
        }
    }
    for (size_t i=0;i<m->aero.book_count;++i) {
        double weight=mm305_mach_scale_weight(m->aero.book[i].mach,observed_mach);
        m->aero.book[i].lift_per_q_m2*=1.0+weight*(lift_scale-1.0);
        m->aero.book[i].drag_per_q_m2*=1.0+weight*(drag_scale-1.0);
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
    out.scale_mach=request->scale_mach;
    out.lift_scale=request->lift_scale;
    out.drag_scale=request->drag_scale;
    TerminalModel *scaled=malloc(sizeof(*scaled));
    TerminalModel *reciprocal=malloc(sizeof(*reciprocal));
    if (!scaled || !reciprocal) {
        free(scaled);free(reciprocal);
        snprintf(out.diagnostic,sizeof(out.diagnostic),"MM305 plan allocation failed");
        return out;
    }
    *scaled=*model;
    mm305_scale_model(scaled,request->scale_mach,
        clampd(request->lift_scale,0.5,2.0),
        clampd(request->drag_scale,0.5,2.0));
    /* Plan only incidences the TAEM elevators can hold in trim. */
    if (request->state.trim_aoa_ceiling_rad > 0.0)
        scaled->vehicle.maximum_angle_of_attack = fmin(
            scaled->vehicle.maximum_angle_of_attack,
            request->state.trim_aoa_ceiling_rad * RAD2DEG);
    mm305_reciprocal_model(scaled,reciprocal);
    /* HAC size is the route's energy lever: a larger HAC flies a longer path
       and sheds more energy, a smaller one less.  The candidate search already
       tightens the radius when the configured one fails; add the larger HAC it
       never tries, for states with energy to spare. */
    const double radius_factors[]={1.0,1.5};
    TaemFixedHacSearch search;
    memset(&search,0,sizeof(search));
    int best=-1;

    /* Prefer the configured runway end.  Searching both runway ends doubles the
       most expensive part of MM305 planning and can consume tens of seconds of
       live TAEM time.  The reciprocal remains a fallback when the configured end
       has no feasible route. */
    const TerminalModel *end_models[2]={
        request->search_both_ends||request->upstream_end==0?scaled:reciprocal,
        reciprocal
    };
    int end_ids[2]={
        request->search_both_ends?0:request->upstream_end,
        1
    };
    int end_count=request->search_both_ends?2:1;
    for (int end_pass=0;end_pass<end_count&&best<0;++end_pass) {
        const TerminalModel *end_model=end_models[end_pass];
        int runway_end=end_ids[end_pass];
        for (size_t r=0;r<sizeof(radius_factors)/sizeof(radius_factors[0])&&best<0;++r) {
            double radius=request->hac_radius_m*radius_factors[r];
            if (!(radius>=2000.0)) continue;
            search=taem_fixed_hac_search(end_model,&request->state,
                radius,200.0,0.5,420.0);
            for (int i=0;i<search.candidate_count;++i)
                search.candidates[i].runway_end=runway_end;
            for (int i=0;i<search.candidate_count;++i) {
                const TaemFixedHacCandidate *c=&search.candidates[i];
                if (!mm305_candidate_ok(c)) continue;
                if (request->restrict_side && c->side*request->side<=0.0) continue;
                if (best<0 || (i==search.selected_candidate) ||
                    (search.selected_candidate<0 &&
                     c->quality_score<search.candidates[best].quality_score))
                    best=i;
                if (i==search.selected_candidate) break;
            }
        }
    }
    out.valid=true;
    if (best>=0) {
        out.found=true;
        out.candidate=search.candidates[best];
    }
    int used=snprintf(out.diagnostic,sizeof(out.diagnostic),
        "MM305 route %s scale=%.3f/%.3f:",
        out.found?"found":"rejected",request->lift_scale,request->drag_scale);
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

static Mm305PlanRequest mm305_request_from_state(const GuidanceMachine *g,
        const Telemetry *t, const VehicleState *state,
        const LandingConfiguration *cfg, const TerminalModel *model) {
    Mm305PlanRequest r;
    memset(&r,0,sizeof(r));
    if (!g || !t || !state || !cfg || !model || !model->replay_validated)
        return r;
    r.state=terminal_live_state(g,state,t,model);
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
    AeroForces observed=aero_compute(&model->world,&model->aero,
        r.state.position_i_m,r.state.velocity_i_mps,r.state.ut_s,r.state.mass_kg,
        r.state.attitude.aoa_rad,r.state.attitude.bank_rad);
    r.scale_mach=observed.mach;
    r.valid=true;
    return r;
}

Mm305PlanRequest guidance_mm305_plan_request(const GuidanceMachine *g,
        const Telemetry *t, const VehicleState *state,
        const LandingConfiguration *cfg, const TerminalModel *model) {
    if (!g || !g->mm305_planning_needed) {
        Mm305PlanRequest r;
        memset(&r,0,sizeof(r));
        return r;
    }
    return mm305_request_from_state(g,t,state,cfg,model);
}

Mm305PlanRequest guidance_mm305_admission_request(const GuidanceMachine *g,
        const Telemetry *t, const VehicleState *state,
        const LandingConfiguration *cfg, const TerminalModel *model) {
    if (!g || !g->mm305_admission_needed) {
        Mm305PlanRequest r;
        memset(&r,0,sizeof(r));
        return r;
    }
    Mm305PlanRequest r=mm305_request_from_state(g,t,state,cfg,model);
    /* Qualification is for the route MM305 will fly from this state: no side
       is committed yet, and the measured force scales belong to MM305. */
    r.restrict_side=false;
    r.lift_scale=r.drag_scale=1.0;
    return r;
}

void guidance_mm305_admission_accept(GuidanceMachine *g, const Mm305PlanResult *result) {
    if (!g || !result || !result->valid) return;
    g->mm305_admission_needed=false;
    g->mm305_admission_valid=true;
    g->mm305_admission_found=result->found;
    g->mm305_admission_result_ut=result->request_ut;
    g->mm305_admission_snapshot_id=result->model_snapshot_id;
    g->mm305_admission_solve_wall_s=result->solve_wall_s;
    if (result->found) {
        g->mm305_admission_route=result->candidate.route;
        g->mm305_admission_runway_end=result->candidate.runway_end;
        g->mm305_admission_side=result->candidate.side;
    } else {
        g->mm305_admission_route=(TaemRoute){0};
        g->mm305_admission_rejections++;
        fprintf(stderr,"MM305_ADMISSION rejected: %s\n",result->diagnostic);
    }
}

bool guidance_mm305_admission_qualified(GuidanceMachine *g, const Telemetry *t,
        bool physically_admissible, uint64_t model_snapshot_id) {
    if (!g || !t || !isfinite(t->ut)) return false;
    if (!physically_admissible) {
        /* Outside the physical set a pending request is moot; a held result
           stays for diagnosis but can never be handed over once stale. */
        g->mm305_admission_needed=false;
        return false;
    }
    double age=t->ut-g->mm305_admission_result_ut;
    if (g->mm305_admission_valid && g->mm305_admission_found &&
        g->mm305_admission_snapshot_id==model_snapshot_id &&
        age>=-1e-6 && age<=MM305_ADMISSION_MAX_AGE_S)
        return true;
    bool rejected_recently=g->mm305_admission_valid && !g->mm305_admission_found &&
        isfinite(g->mm305_admission_last_attempt_ut) &&
        t->ut-g->mm305_admission_last_attempt_ut<MM305_ADMISSION_RETRY_S;
    if (!g->mm305_admission_needed && !rejected_recently) {
        g->mm305_admission_needed=true;
        g->mm305_admission_request_ut=t->ut;
        g->mm305_admission_last_attempt_ut=t->ut;
    }
    return false;
}

bool guidance_mm305_adopt_admission_route(GuidanceMachine *g,
        const LandingConfiguration *cfg) {
    if (!g || !cfg || !g->mm305_admission_valid || !g->mm305_admission_found ||
        !g->mm305_admission_route.valid)
        return false;
    Mm305PlanResult result;
    memset(&result,0,sizeof(result));
    result.valid=result.found=true;
    result.request_ut=g->mm305_admission_result_ut;
    result.model_snapshot_id=g->mm305_admission_snapshot_id;
    result.solve_wall_s=g->mm305_admission_solve_wall_s;
    result.candidate.route=g->mm305_admission_route;
    result.candidate.runway_end=g->mm305_admission_runway_end;
    result.candidate.side=g->mm305_admission_side;
    bool adopted=guidance_mm305_accept_plan(g,&result,cfg);
    if (adopted) g->mm305_last_plan_attempt_ut=g->mm305_admission_result_ut;
    return adopted;
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
        "\"finalDistance\":%.17g,\"replan\":%s,\"scaleMach\":%.3f,"
        "\"liftScale\":%.4f,\"dragScale\":%.4f,"
        "\"solveWall\":%.3f}\n",
        candidate->runway_end,candidate->side,candidate->route.hac.radius_m,
        candidate->route.hac.arc_sweep_rad,candidate->route.lead_length_m,
        candidate->route.hac.arc_length_m,
        candidate->route.hac.entry.x,candidate->route.hac.entry.y,
        candidate->route.hac.center.x,candidate->route.hac.center.y,
        candidate->route.hac.exit.x,candidate->route.hac.exit.y,
        cfg->guidance.final_approach_distance,replan?"true":"false",
        result->scale_mach,result->lift_scale,result->drag_scale,result->solve_wall_s);
    g->mm305_route_cursor=0;
    g->mm305_route_committed=true;
    g->mm305_acquisition_route=(TaemRoute){0};
    g->mm305_acquisition_route_cursor=0;
    g->mm305_acquisition_route_valid=false;
    g->mm305_acquisition_route_ut=-INFINITY;
    g->mm305_acquisition_sweep_rad=NAN;
    g->mm305_hac_exit_reached=false;
    g->mm305_runway_alignment_active=false;
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
    /* A multiplicative whole-model scale is not observable around the
       startup zero/negative-incidence transient.  TAEM's useful lifting regime
       is positive AoA, so seed only after the live vehicle has established at
       least 3 degrees of positive aerodynamic incidence. */
    if (!isfinite(t->angle_of_attack) || t->angle_of_attack<3.0) return;
    if (!t->physics_sample_valid || !(t->mass>0.0) || !(t->dynamic_pressure>200.0)) return;
    AeroForces modeled=aero_compute(&model->world,&model->aero,current->position_i_m,
        current->velocity_i_mps,current->ut_s,current->mass_kg,
        current->attitude.aoa_rad,current->attitude.bank_rad);
    if (!(modeled.lift_n>1.0) || !(modeled.drag_n>1.0)) return;
    double lift_raw=fabs(t->lift_force)/modeled.lift_n;
    double drag_raw=fabs(t->drag_force)/modeled.drag_n;
    double lift_ratio=clampd(lift_raw,0.5,2.0);
    double drag_ratio=clampd(drag_raw,0.5,2.0);
    double a=clampd(dt/(8.0+dt),0.0,1.0);
    bool first_lift=!(g->mm305_lift_scale>0.0)||!isfinite(g->mm305_lift_scale);
    bool first_drag=!(g->mm305_drag_scale>0.0)||!isfinite(g->mm305_drag_scale);
    if (first_lift) g->mm305_lift_scale=lift_ratio;
    else g->mm305_lift_scale+=a*(lift_ratio-g->mm305_lift_scale);
    /* TAEM flies with the split rudder near its nominal setting, so the
       measured drag (brake included) is what the planner should model; the
       brake's departures from nominal are the energy control. */
    if (first_drag) g->mm305_drag_scale=drag_ratio;
    else g->mm305_drag_scale+=a*(drag_ratio-g->mm305_drag_scale);
    const char *diag=getenv("KSP_LANDER_TAEM_DIAGNOSTICS");
    if (diag && (first_lift || first_drag || strcmp(diag,"aero")==0))
        fprintf(stderr,"MM305 aero sample: raw=%.3f/%.3f clamped=%.3f/%.3f filtered=%.3f/%.3f measured=%.0f/%.0f modeled=%.0f/%.0f q=%.0f aoa=%.2f sb=%.3f\n",
            lift_raw,drag_raw,lift_ratio,drag_ratio,g->mm305_lift_scale,g->mm305_drag_scale,
            fabs(t->lift_force),fabs(t->drag_force),modeled.lift_n,modeled.drag_n,
            t->dynamic_pressure,t->angle_of_attack,g->taem_speedbrake_fraction);

}

/* Build the concrete path flown while no replay-qualified route is held.
 * This is still provisional: it may be replaced by the async planner at any
 * time, and it never satisfies the MM305->Final admission contract.  The key
 * invariant is simpler: while it is displayed as acquisition guidance, the
 * tracker follows this exact same route geometry. */
static bool mm305_build_acquisition_route(GuidanceMachine *g,
        const TerminalModel *model, const TerminalDynamicState *current,
        const TaemGeometryState *geometry, double hac_radius, double side,
        double selected_sweep) {
    if (!g || !model || !current || !geometry) return false;

    TaemReachability reachability;
    if (!taem_fixed_hac_turn_reachability(model,current,geometry,hac_radius,
            &reachability) || !reachability.valid ||
        !(reachability.available_lateral_accel_mps2 > 0.0))
        return false;

    double live_curvature=0.95*reachability.available_lateral_accel_mps2/
        fmax(geometry->ground_speed_mps*geometry->ground_speed_mps,1.0);
    double rho_live=world_atmosphere_sample(&model->world,
        model->site.altitude+geometry->altitude_above_runway_m).density_kg_m3;

    TaemFixedHacGeometry sweep_geometry;
    if (!taem_hac_geometry_sweep(model,hac_radius,side,selected_sweep,
            &sweep_geometry))
        return false;
    double terminal_height=sweep_geometry.final_length_m*
        tan(model->guidance.final_glide_slope*DEG2RAD);
    double hac_height=terminal_height+sweep_geometry.arc_length_m*
        tan(fmin(model->guidance.taem_glide_slope,12.0)*DEG2RAD);
    double rho_join=world_atmosphere_sample(&model->world,
        model->site.altitude+fmax(0.0,hac_height)).density_kg_m3;
    double density_gain=rho_live>0.0&&rho_join>rho_live?
        rho_join/rho_live:1.0;
    double peak_curvature_limit=fmax(live_curvature,
        live_curvature*density_gain);
    double spacing=clampd(0.5*geometry->ground_speed_mps,250.0,700.0);

    TaemRoute route={0};
    char reason[160]={0};
    bool built=taem_route_build_hac_shortest(model,geometry,hac_radius,side,
        selected_sweep,spacing,live_curvature,peak_curvature_limit,
        &route,reason,sizeof(reason));
    if (!built)
        built=taem_route_build_hac(model,geometry,hac_radius,side,
            selected_sweep,spacing,live_curvature,&route,reason,sizeof(reason));
    if (!built) {
        const char *diag=getenv("KSP_LANDER_TAEM_DIAGNOSTICS");
        if (diag && strcmp(diag,"2")==0 && !g->diagnostic_shadow)
            fprintf(stderr,"MM305 acquisition route build failed: %s\n",
                reason[0]?reason:"unknown");
        return false;
    }

    g->mm305_acquisition_route=route;
    g->mm305_acquisition_route_cursor=0;
    g->mm305_acquisition_route_valid=true;
    g->mm305_acquisition_route_ut=current->ut_s;
    g->mm305_acquisition_sweep_rad=selected_sweep;
    return true;
}

static void mm305_route_to_reference_trajectory(Trajectory *out,
        const TaemRoute *route, const TerminalModel *model,
        const Telemetry *telemetry) {
    if (!out || !route || !route->valid || route->count<2 || !model ||
        !telemetry) return;
    trajectory_clear(out);
    GeoPoint origin={model->site.latitude,model->site.longitude,model->site.altitude};
    double heading=model->site.runway_heading*DEG2RAD;
    double sh=sin(heading),ch=cos(heading);
    size_t max_points=120;
    size_t step=route->count<=max_points?1:
        (route->count-1)/(max_points-1)+
        (((route->count-1)%(max_points-1))?1:0);
    double end_speed=fmax(model->guidance.final_alignment_speed,
        model->vehicle.minimum_safe_speed);
    for (size_t index=0;index<route->count;index+=step) {
        TaemRoutePoint rp;
        if (!taem_route_point_at_index(route,index,&rp)) continue;
        double east=rp.along_m*sh+rp.cross_m*ch;
        double north=rp.along_m*ch-rp.cross_m*sh;
        double altitude=isfinite(rp.altitude_m)?rp.altitude_m:
            model->site.altitude+telemetry->radar_altitude;
        GeoPoint p=local_point(origin,east,north,model->world.radius_m,altitude);
        double f=route->length_m>0.0?
            clampd(rp.distance_from_start_m/route->length_m,0.0,1.0):0.0;
        double speed=telemetry->true_air_speed+
            f*(end_speed-telemetry->true_air_speed);
        double average_speed=fmax(1.0,0.5*(telemetry->true_air_speed+speed));
        trajectory_append(out,(TrajectoryPoint){
            .ut=telemetry->ut+rp.distance_from_start_m/average_speed,
            .latitude=p.latitude,.longitude=p.longitude,.altitude=p.altitude,
            .speed=speed,.phase=PHASE_TAEM,.kind=TRAJ_REFERENCE
        });
    }
    if (route->count>1 && (route->count-1)%step!=0) {
        TaemRoutePoint rp;
        if (taem_route_point_at_index(route,route->count-1,&rp)) {
            double east=rp.along_m*sh+rp.cross_m*ch;
            double north=rp.along_m*ch-rp.cross_m*sh;
            double altitude=isfinite(rp.altitude_m)?rp.altitude_m:
                model->site.altitude+telemetry->radar_altitude;
            GeoPoint p=local_point(origin,east,north,model->world.radius_m,altitude);
            trajectory_append(out,(TrajectoryPoint){
                .ut=telemetry->ut+route->length_m/
                    fmax(1.0,0.5*(telemetry->true_air_speed+end_speed)),
                .latitude=p.latitude,.longitude=p.longitude,.altitude=p.altitude,
                .speed=end_speed,.phase=PHASE_TAEM,.kind=TRAJ_REFERENCE
            });
        }
    }
}

static bool mm305_acquisition_command(GuidanceMachine *g,
        const TerminalModel *model, const TerminalDynamicState *current,
        const TaemGeometryState *geometry, double hac_radius, double dt,
        TaemTrackerOutput *demand, TaemPathReference *reference,
        double *path_remaining_out, double *required_drag_out,
        double *available_drag_out) {
    if (!g) return false;
    double side=g->mm305_acquisition_route_valid?
        g->mm305_acquisition_route.side:
        (geometry->runway_cross_m>=0.0?1.0:-1.0);
    double alignment_height=model->guidance.final_approach_distance*
        tan(model->guidance.final_glide_slope*DEG2RAD);
    double alignment_speed=fmax(model->guidance.final_alignment_speed,
        model->vehicle.minimum_safe_speed);
    double radius=model->world.radius_m+model->site.altitude+
        fmax(0.0,geometry->altitude_above_runway_m);
    double gravity=model->world.mu_m3_s2/fmax(radius*radius,1.0);
    double energy_excess=0.5*(geometry->airspeed_mps*geometry->airspeed_mps-
        alignment_speed*alignment_speed)+
        gravity*(geometry->altitude_above_runway_m-alignment_height);
    energy_excess=fmax(0.0,energy_excess);

    AeroForces modeled=aero_compute(&model->world,&model->aero,
        current->position_i_m,current->velocity_i_mps,current->ut_s,
        current->mass_kg,current->attitude.aoa_rad,current->attitude.bank_rad);
    double available_drag=current->mass_kg>1.0&&modeled.drag_n>0.0?
        modeled.drag_n/current->mass_kg:0.0;
    double planning_drag=fmax(0.75,0.85*available_drag);
    double required_path=energy_excess/planning_drag;

    double selected_sweep=0.5*M_PI;
    double selected_path=0.0;
    for (double sweep_deg=90.0;sweep_deg<=270.0+1e-6;sweep_deg+=15.0) {
        TaemFixedHacGeometry trial;
        double sweep=sweep_deg*DEG2RAD;
        if (!taem_hac_geometry_sweep(model,hac_radius,side,sweep,&trial))
            continue;
        double dx=trial.entry.x-geometry->runway_along_m;
        double dy=trial.entry.y-geometry->runway_cross_m;
        double rollout=fmax(0.0,
            -model->guidance.final_approach_distance-trial.exit.x);
        double path=hypot(dx,dy)+trial.arc_length_m+rollout;
        selected_sweep=sweep;
        selected_path=path;
        if (path>=1.10*required_path) break;
    }
    if (!(selected_path>0.0)) return false;

    bool refresh=!g->mm305_acquisition_route_valid||
        !g->mm305_acquisition_route.valid||
        current->ut_s-g->mm305_acquisition_route_ut>=6.0||
        !isfinite(g->mm305_acquisition_sweep_rad)||
        fabs(selected_sweep-g->mm305_acquisition_sweep_rad)>15.0*DEG2RAD;
    if (refresh) {
        if (!mm305_build_acquisition_route(g,model,current,geometry,hac_radius,
                side,selected_sweep))
            g->mm305_acquisition_route_valid=false;
    }

    bool have_route=g->mm305_acquisition_route_valid&&
        taem_route_reference(&g->mm305_acquisition_route,geometry,
            &g->mm305_acquisition_route_cursor,reference,NULL);
    if (!have_route) {
        if (!mm305_build_acquisition_route(g,model,current,geometry,hac_radius,
                side,selected_sweep) ||
            !taem_route_reference(&g->mm305_acquisition_route,geometry,
                &g->mm305_acquisition_route_cursor,reference,NULL))
            return false;
    }

    double route_remaining=taem_route_remaining_at_index(
        &g->mm305_acquisition_route,g->mm305_acquisition_route_cursor);
    if (isfinite(route_remaining)&&route_remaining>0.0)
        selected_path=route_remaining;

    double altitude_to_alignment=fmax(0.0,
        geometry->altitude_above_runway_m-alignment_height);
    double required_average_slope=
        atan2(altitude_to_alignment,fmax(selected_path,2000.0))*RAD2DEG;
    /* Keep the provisional route's lateral geometry exact, but preserve altitude
       while the qualified planner works.  The committed route later owns the
       full vertical profile. */
    reference->altitude_m=NAN;
    reference->flight_path_angle_deg=
        -clampd(required_average_slope,2.0,8.0);
    reference->vertical_curvature_per_m=0.0;

    TaemTrackerOutput full=taem_tracker_update(model,current,geometry,reference,dt);
    if (!full.valid) return false;
    double full_lateral=fabs(full.required_lateral_accel_mps2);
    double lateral_limit=0.80*fmax(0.0,full.available_lateral_accel_mps2);
    if (full.lateral_authority_ok&&full_lateral<=lateral_limit+1e-6) {
        *demand=full;
    } else {
        TaemPathReference shaped=*reference;
        TaemTrackerOutput best=full;
        double lo=0.0,hi=1.0;
        for (int i=0;i<7;++i) {
            double scale=0.5*(lo+hi);
            TaemPathReference trial=*reference;
            trial.curvature_right_per_m=reference->curvature_right_per_m*scale;
            TaemTrackerOutput td=taem_tracker_update(model,current,geometry,&trial,dt);
            double req=fabs(td.required_lateral_accel_mps2);
            double cap=0.80*fmax(0.0,td.available_lateral_accel_mps2);
            bool ok=td.valid&&td.lateral_authority_ok&&req<=cap+1e-6;
            if (ok) {lo=scale;shaped=trial;best=td;} else hi=scale;
        }
        *reference=shaped;
        *demand=best;
    }

    if (path_remaining_out) *path_remaining_out=selected_path;
    if (required_drag_out) *required_drag_out=
        energy_excess/fmax(selected_path,1000.0);
    if (available_drag_out) *available_drag_out=available_drag;

    const char *diag=getenv("KSP_LANDER_TAEM_DIAGNOSTICS");
    if (diag&&strcmp(diag,"2")==0&&!g->diagnostic_shadow)
        fprintf(stderr,
            "MM305 acquisition route: sweep=%.0f path=%.0f E=%.0f reqD=%.2f availD=%.2f cursor=%zu/%zu xt=%.0f courseErr=%.1f V=%.1f h=%.0f\n",
            selected_sweep*RAD2DEG,selected_path,energy_excess,
            energy_excess/fmax(selected_path,1000.0),available_drag,
            g->mm305_acquisition_route_cursor,g->mm305_acquisition_route.count,
            demand->cross_track_error_m,demand->course_error_deg,
            geometry->airspeed_mps,geometry->altitude_above_runway_m);
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

    mm305_trim_observe(g,t);
    TerminalDynamicState current=terminal_live_state(g,vehicle_state,t,model);
    if (!terminal_state_validate(&current,NULL,0))
        return terminal_abort(g,
            "MM305 native state could not be projected into the fixed runway frame.");
    if (g->mm305_route_committed && g->mm305_model_snapshot_id!=model->snapshot_id)
        return terminal_abort(g,
            "The native MM305 model snapshot changed after route commitment.");
    mm305_observe_force_scale(g,t,model,&current,dt);

    const double replan_interval_s=20.0;
    const double retry_interval_s=2.0;
    if (!g->mm305_route_committed) {
        /* The vehicle state changes materially during each async solve.  After a
           failed solve, request the next measured-state solve promptly instead
           of exponential 10/20/40 s dead time while acquisition burns geometry. */
        bool aero_calibrated=isfinite(g->mm305_lift_scale)&&g->mm305_lift_scale>0.0&&
            isfinite(g->mm305_drag_scale)&&g->mm305_drag_scale>0.0;
        double backoff=retry_interval_s;
        bool due=aero_calibrated&&(!isfinite(g->mm305_last_plan_attempt_ut) ||
            t->ut-g->mm305_last_plan_attempt_ut>=backoff);
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
    }

    int end=g->mm305_route_committed&&g->runway_end_index==1?1:
        (g->runway_end_preview_valid&&g->runway_end_index==1?1:0);
    const TerminalModel *end_model=mm305_model_for_end(model,end);
    /* Planner and live tracker must fly the same measured aerodynamics.
       The asynchronous planner has always applied these Mach-local force scales
       to its private model copy; the live tracker previously used the unscaled
       nominal model, so it systematically under-commanded AoA whenever live lift
       was below the table.  Scale a per-tick copy only; the frozen certified
       model snapshot remains immutable. */
    TerminalModel live_model=*end_model;
    double live_lift_scale=isfinite(g->mm305_lift_scale)&&g->mm305_lift_scale>0.0?
        clampd(g->mm305_lift_scale,0.5,2.0):1.0;
    double live_drag_scale=isfinite(g->mm305_drag_scale)&&g->mm305_drag_scale>0.0?
        clampd(g->mm305_drag_scale,0.5,2.0):1.0;
    mm305_scale_model(&live_model,t->mach,live_lift_scale,live_drag_scale);
    const TerminalModel *control_model=&live_model;
    TaemGeometryState geometry;
    if (!taem_geometry_state(control_model,&current,&geometry))
        return terminal_abort(g,
            "MM305 native state could not be projected into the fixed runway frame.");
    if (geometry.altitude_above_runway_m<=0.0)
        return terminal_abort(g,
            "MM305 crossed runway elevation before reaching the fixed-HAC exit.");

    TaemPathReference reference;
    TaemTrackerOutput demand;
    const char *status;
    if (g->mm305_route_committed && g->mm305_runway_alignment_active) {
        reference=taem_alignment_reference(control_model,&geometry);
        demand=taem_tracker_update(control_model,&current,&geometry,&reference,dt);
        if (!demand.valid)
            return terminal_abort(g,
                "MM305 runway alignment could not produce a bounded control command.");
        if (taem_alignment_exhausted(control_model,&geometry))
            return terminal_abort(g,
                "MM305 missed the Final alignment point; late runway-line capture is not permitted.");
        if (taem_alignment_ready(control_model,&current,&geometry)) {
            fprintf(stderr,
                "MM305_LIVE_EXIT model=%llu side=%+.0f radius=%.0f sweep=%.1f lead=%.0f cursor=%zu/%zu refErr=%.2f runwayCourseErr=%.2f along=%.1f cross=%.1f h=%.1f airV=%.1f fpa=%.2f bank=%.2f aoa=%.2f cmdBank=%.2f cmdAoa=%.2f reason=runway-aligned\n",
                (unsigned long long)g->mm305_model_snapshot_id,
                g->mm305_route.side,g->mm305_route.hac.radius_m,
                g->mm305_route.hac.arc_sweep_rad*RAD2DEG,
                g->mm305_route.lead_length_m,g->mm305_route_cursor,
                g->mm305_route.count,demand.course_error_deg,
                geometry.heading_error_deg,geometry.runway_along_m,
                geometry.runway_cross_m,geometry.altitude_above_runway_m,
                geometry.airspeed_mps,geometry.flight_path_angle_deg,
                current.attitude.bank_rad*RAD2DEG,
                current.attitude.aoa_rad*RAD2DEG,
                demand.control.bank_rad*RAD2DEG,
                demand.control.angle_of_attack_rad*RAD2DEG);
            g->mm305_hac_exit_reached=true;
            g->hac_completed=true;
            g->hac_remaining=0.0;
        }
        status=g->mm305_hac_exit_reached?
            "MM305 delivered a settled runway-line state to Final." :
            "MM305 is settling bank and course on the runway line after HAC.";
    } else if (g->mm305_route_committed) {
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
        demand=taem_tracker_update(control_model,&current,&geometry,&reference,dt);
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
            if (!g->diagnostic_shadow) fprintf(stderr,"MM305 route released: cross %.0f m course %.1f deg; re-planning.\n",
                demand.cross_track_error_m,demand.course_error_deg);
            g->mm305_route_committed=false;
            g->terminal_path_committed=false;
            g->hac_captured=false;
            g->mm305_acquisition_route_valid=false;
            g->mm305_acquisition_route_cursor=0;
            g->mm305_planning_needed=true;
            g->mm305_last_plan_attempt_ut=t->ut;
            g->mm305_plan_request_ut=t->ut;
        }
        /* Replan only when tracking degrades.  Periodic replanning adopted a
           different route every interval (the energy lever picks new lengths)
           and each switch kicked the tracker, producing bank/incidence spikes
           worse than the error it removed. */
        /* Energy is closed-loop too: the plan's speed profile comes from aero
           scales identified at plan time, and live drag varies with Mach.  When
           the measured energy (altitude surplus counted as speed) departs the
           plan by more than 10%, re-plan from the measured state so the route
           (length, HAC sweep) again delivers the alignment speed. */
        double plan_speed_now=taem_route_planned_speed(&g->mm305_route,reference.station_m);
        double energy_ratio=NAN;
        if (isfinite(plan_speed_now)&&plan_speed_now>1.0&&isfinite(reference.altitude_m)) {
            double dh=control_model->site.altitude+geometry.altitude_above_runway_m-
                reference.altitude_m;
            energy_ratio=sqrt(fmax(0.0,geometry.airspeed_mps*geometry.airspeed_mps+
                2.0*9.81*dh))/plan_speed_now-1.0;
        }
        bool energy_off_plan=isfinite(energy_ratio)&&fabs(energy_ratio)>0.10&&
            isfinite(route_remaining)&&route_remaining>15000.0;
        if (g->mm305_route_committed && g->mm305_async_planning &&
            !g->mm305_planning_needed &&
            t->ut-g->mm305_last_plan_attempt_ut>=replan_interval_s &&
            (energy_off_plan ||
             fabs(demand.cross_track_error_m)>1000.0 ||
             fabs(demand.course_error_deg)>20.0 ||
             (isfinite(reference.flight_path_angle_deg) &&
              fabs(geometry.flight_path_angle_deg-reference.flight_path_angle_deg)>6.0))) {
            double remaining=taem_route_remaining_at_index(&g->mm305_route,g->mm305_route_cursor);
            if (isfinite(remaining) && remaining>fmax(8000.0,0.35*g->mm305_route.hac.arc_length_m)) {
                if (energy_off_plan && !g->diagnostic_shadow)
                    fprintf(stderr,"MM305 energy off plan by %+.0f%% with %.0f m left; re-planning.\n",
                        100.0*energy_ratio,remaining);
                g->mm305_planning_needed=true;
                g->mm305_last_plan_attempt_ut=t->ut;
                g->mm305_plan_request_ut=t->ut;
            }
        }
        double exit_distance=hypot(
            geometry.runway_along_m-g->mm305_route.alignment_along_m,
            geometry.runway_cross_m);
        if (g->mm305_route_committed && g->mm305_route_cursor+2>=g->mm305_route.count &&
            exit_distance<=700.0 && fabs(demand.course_error_deg)<=12.0) {
            fprintf(stderr,
                "MM305_ALIGNMENT_ROUTE_END model=%llu side=%+.0f radius=%.0f sweep=%.1f lead=%.0f cursor=%zu/%zu refErr=%.2f runwayCourseErr=%.2f along=%.1f cross=%.1f h=%.1f airV=%.1f fpa=%.2f bank=%.2f aoa=%.2f cmdBank=%.2f cmdAoa=%.2f\n",
                (unsigned long long)g->mm305_model_snapshot_id,
                g->mm305_route.side, g->mm305_route.hac.radius_m,
                g->mm305_route.hac.arc_sweep_rad * 180.0 / 3.14159265358979323846,
                g->mm305_route.lead_length_m, g->mm305_route_cursor,
                g->mm305_route.count, demand.course_error_deg,
                geometry.heading_error_deg, geometry.runway_along_m,
                geometry.runway_cross_m, geometry.altitude_above_runway_m,
                geometry.airspeed_mps, geometry.flight_path_angle_deg,
                current.attitude.bank_rad*RAD2DEG,
                current.attitude.aoa_rad*RAD2DEG,
                demand.control.bank_rad*RAD2DEG,
                demand.control.angle_of_attack_rad*RAD2DEG);
            g->mm305_runway_alignment_active=true;
            g->hac_captured=true;
            g->hac_remaining=0.0;
        }
        status=g->mm305_runway_alignment_active?
            "MM305 native route reached the Final alignment station; verifying settled handoff state." :
            "MM305 native tracker is following the committed route through HAC and alignment rollout.";
    }
    double acquisition_path_remaining=NAN;
    double acquisition_required_drag=NAN;
    double acquisition_available_drag=NAN;
    if (!g->mm305_route_committed) {
        double hac_radius=g->hac_radius>=3000.0?g->hac_radius:cfg->guidance.hac_radius;
        if (!mm305_acquisition_command(g,control_model,&current,&geometry,
                hac_radius,dt,&demand,&reference,
                &acquisition_path_remaining,&acquisition_required_drag,
                &acquisition_available_drag))
            return terminal_abort(g,
                "MM305 acquisition law could not produce a bounded control command.");
        if (geometry.altitude_above_runway_m<1000.0 && g->mm305_plan_failures>0)
            return terminal_abort(g,
                "MM305 found no qualified route before the height needed to fly one.");
        status="MM305 acquisition: tracking the displayed provisional TAEM route while qualification runs.";
    }

    const char *diagnostics=getenv("KSP_LANDER_TAEM_DIAGNOSTICS");
    if (diagnostics && strcmp(diagnostics,"2")==0 && !g->diagnostic_shadow)
        fprintf(stderr,
            "TAEM live trace: ceil=%.1f latReq=%.2f latAvail=%.2f ut=%.2f committed=%d idx=%zu along=%.0f cross=%.0f h=%.0f course=%.1f ref=%.1f kappa=%.3g xt=%.0f crsErr=%.1f fpa=%.1f refFpa=%.1f bank=%.1f cmdBank=%.1f aoa=%.1f cmdAoa=%.1f scales=%.3f/%.3f\n",
            current.trim_aoa_ceiling_rad*RAD2DEG,
            demand.required_lateral_accel_mps2,demand.available_lateral_accel_mps2,
            current.ut_s,g->mm305_route_committed,g->mm305_route_cursor,
            geometry.runway_along_m,geometry.runway_cross_m,
            geometry.altitude_above_runway_m,geometry.course_deg,reference.course_deg,
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

    /* Split-rudder speedbrake is TAEM's energy control.  A committed route uses
       its tabulated speed/altitude profile.  Acquisition uses endpoint energy and
       the provisional HAC path, so route-search latency cannot silently preserve
       too much energy all the way to KSC. */
    const double sb_cap=0.50, sb_nominal=0.25, sb_slew_per_s=0.05;
    const double sb_stow_remaining_m=500.0, sb_fade_m=2500.0;
    double sb_dt=g->taem_speedbrake_ut>0.0&&t->ut>g->taem_speedbrake_ut?
        fmin(t->ut-g->taem_speedbrake_ut,1.0):0.0;
    g->taem_speedbrake_ut=t->ut;

    double planned_speed=g->mm305_route_committed?
        taem_route_planned_speed(&g->mm305_route,reference.station_m):NAN;
    double sb_remaining=g->mm305_route_committed?
        taem_route_remaining_at_index(&g->mm305_route,g->mm305_route_cursor):
        acquisition_path_remaining;
    double sb_geometric_fade=isfinite(sb_remaining)?
        clampd((sb_remaining-sb_stow_remaining_m)/sb_fade_m,0.0,1.0):0.0;
    double sb_target=0.0;
    double sb_limit=sb_cap*sb_geometric_fade;

    if (g->mm305_route_committed &&
            isfinite(planned_speed)&&planned_speed>1.0&&isfinite(reference.altitude_m)) {
        double surplus_height=control_model->site.altitude+
            geometry.altitude_above_runway_m-reference.altitude_m;
        double equivalent=sqrt(fmax(0.0,geometry.airspeed_mps*geometry.airspeed_mps+
            2.0*9.81*surplus_height));
        double ratio=equivalent/planned_speed-1.0;
        double energy_high=clampd(ratio/0.02,0.0,1.0);
        double authority_fade=fmax(sb_geometric_fade,energy_high);
        sb_limit=sb_cap*authority_fade;
        double sb_base=sb_nominal*authority_fade;
        g->taem_speedbrake_integral=clampd(
            g->taem_speedbrake_integral+0.05*ratio*sb_dt,
            -sb_base,sb_limit-sb_base);
        sb_target=clampd(sb_base+2.5*ratio+g->taem_speedbrake_integral,
            0.0,sb_limit);
        if (diagnostics && strcmp(diagnostics,"2")==0 && !g->diagnostic_shadow)
            fprintf(stderr,
                "TAEM speedbrake: V=%.1f Veq=%.1f plan=%.1f dh=%.0f limit=%.2f target=%.2f\n",
                geometry.airspeed_mps,equivalent,planned_speed,surplus_height,
                sb_limit,sb_target);
    } else if (!g->mm305_route_committed &&
            isfinite(acquisition_required_drag) &&
            isfinite(acquisition_available_drag)) {
        double ratio=acquisition_required_drag/
            fmax(acquisition_available_drag,0.25)-1.0;
        double energy_high=clampd(ratio/0.15,0.0,1.0);
        sb_limit=sb_cap;
        double sb_base=sb_nominal*energy_high;
        g->taem_speedbrake_integral=clampd(
            g->taem_speedbrake_integral+0.04*ratio*sb_dt,
            -sb_base,sb_limit-sb_base);
        sb_target=clampd(sb_base+0.35*ratio+g->taem_speedbrake_integral,
            0.0,sb_limit);
        if (diagnostics && strcmp(diagnostics,"2")==0 && !g->diagnostic_shadow)
            fprintf(stderr,
                "TAEM acquisition speedbrake: reqD=%.2f availD=%.2f path=%.0f ratio=%+.2f target=%.2f\n",
                acquisition_required_drag,acquisition_available_drag,
                acquisition_path_remaining,ratio,sb_target);
    } else {
        g->taem_speedbrake_integral=0.0;
    }

    double sb_step=sb_slew_per_s*sb_dt;
    g->taem_speedbrake_fraction=clampd(sb_target,
        g->taem_speedbrake_fraction-sb_step,
        g->taem_speedbrake_fraction+sb_step);
    command.speedbrake_fraction=g->taem_speedbrake_fraction;
    GuidanceResult result=stabilized(g,
        result_make(PHASE_TAEM,command,status,NULL),t,&cfg->vehicle,&cfg->guidance,dt);
    const TaemRoute *display_route=g->mm305_route_committed?
        &g->mm305_route:
        (g->mm305_acquisition_route_valid?&g->mm305_acquisition_route:NULL);
    if (display_route)
        mm305_route_to_reference_trajectory(&result.reference,display_route,
            control_model,t);
    result.command.has_pitch_trim_feedforward=true;
    result.command.pitch_trim_feedforward=clampd(
        mm305_trim_input(g,result.command.target_aoa),-0.9,0.9);
    g->mm305_command_bank_rad = result.command.target_roll * DEG2RAD;
    g->mm305_command_aoa_rad = result.command.target_aoa * DEG2RAD;
    g->mm305_command_ut = t->ut;
    g->mm305_command_valid = true;
    g->phase=PHASE_TAEM;
    return result;
}
