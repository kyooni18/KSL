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
    g->terminal_final_handoff_distance = NAN;
    g->terminal_final_handoff_slope_deg = NAN;
    g->terminal_final_handoff_speed_mps = NAN;
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

/* The Final alignment station is an energy-management choice, not a fixed
 * runway landmark.  A high/fast MM305 state needs more runway-upstream Final
 * distance so Final does not inherit a steep, late energy dump.  Keep the
 * configured distance as the low-energy minimum and smoothly extend it up to
 * 9 km for high/fast arrivals.  The live replay frontier for this vehicle is
 * between 9.0 and 9.5 km; keep margin instead of sitting on the infeasible edge.
 * Once a route is committed this value is
 * latched, so replanning cannot walk the alignment point toward the runway. */
static double mm305_energy_aware_final_distance(const TerminalModel *model,
        const TerminalDynamicState *state) {
    if (!model) return NAN;
    double base=fmax(500.0,model->guidance.final_approach_distance);
    if (!state) return base;
    TaemGeometryState geometry;
    if (!taem_geometry_state(model,state,&geometry)) return base;

    double slope=model->guidance.final_glide_slope;
    if (!(slope>1.0&&slope<60.0)) return base;
    double nominal_height=base*tan(slope*DEG2RAD);
    double alignment_speed=fmax(model->guidance.final_alignment_speed,
        model->vehicle.minimum_safe_speed);
    double altitude_load=clampd(
        fmax(0.0,geometry.altitude_above_runway_m-nominal_height)/3000.0,0.0,1.0);
    double speed_load=clampd(
        fmax(0.0,geometry.airspeed_mps-alignment_speed)/250.0,0.0,1.0);
    double load=fmax(altitude_load,speed_load);
    double maximum=fmax(base,9000.0);
    return base+load*fmax(0.0,maximum-base);
}

static double mm305_final_alignment_speed_for_distance(const TerminalModel *model,
        double final_distance_m) {
    if (!model) return NAN;
    double base=fmax(model->guidance.final_alignment_speed,
        fmax(model->vehicle.minimum_safe_speed,model->vehicle.touchdown_speed));
    /* A farther alignment station is useful only if MM305 does not throw away
       the extra energy before Final receives it.  Preserve a modest kinetic
       reserve as the station moves from 8 to 9 km; Final then spends that
       reserve over the extra runway-upstream distance instead of forcing TAEM
       to arrive at the same 160 m/s regardless of geometry. */
    double extra_distance=clampd(final_distance_m-8000.0,0.0,1000.0);
    return base+0.005*extra_distance;
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
    if (isfinite(request->final_approach_distance_m) &&
            request->final_approach_distance_m>=500.0) {
        double final_distance=request->final_approach_distance_m;
        double base_slope=clampd(model->guidance.final_glide_slope,3.0,45.0);
        double raw_handoff_height=final_distance*tan(base_slope*DEG2RAD);
        double handoff_height=fmin(raw_handoff_height,4000.0);
        scaled->guidance.final_approach_distance=final_distance;
        scaled->guidance.final_glide_slope=clampd(
            atan2(handoff_height,final_distance)*RAD2DEG,15.0,base_slope);
        scaled->guidance.final_alignment_speed=
            mm305_final_alignment_speed_for_distance(model,final_distance);
    }
    mm305_scale_model(scaled,request->scale_mach,
        clampd(request->lift_scale,0.5,2.0),
        clampd(request->drag_scale,0.5,2.0));
    /* Plan only incidences the TAEM elevators can hold in trim. */
    if (request->state.trim_aoa_ceiling_rad > 0.0)
        scaled->vehicle.maximum_angle_of_attack = fmin(
            scaled->vehicle.maximum_angle_of_attack,
            request->state.trim_aoa_ceiling_rad * RAD2DEG);
    mm305_reciprocal_model(scaled,reciprocal);
    /* First try to qualify the exact provisional route the live tracker is
       already flying.  A successful result is a bumpless promotion from
       acquisition to committed MM305 guidance; only a failed seed falls back to
       the wider HAC search. */
    if (request->seed_route_valid &&
        fabs(request->seed_route.alignment_along_m+
            scaled->guidance.final_approach_distance)<=500.0 &&
        (!request->restrict_side || request->seed_route.side * request->side > 0.0)) {
        const TerminalModel *seed_model =
            request->seed_runway_end == 1 ? reciprocal : scaled;
        TaemFixedHacCandidate seed = taem_fixed_hac_evaluate_route(
            seed_model, &request->state, &request->seed_route,
            request->seed_runway_end, 0.5, 420.0);
        if (mm305_candidate_ok(&seed)) {
            out.valid = true;
            out.found = true;
            out.candidate = seed;
            snprintf(out.diagnostic,sizeof(out.diagnostic),
                "MM305 provisional route promoted after native replay scale=%.3f/%.3f",
                request->lift_scale,request->drag_scale);
            free(scaled); free(reciprocal);
            out.solve_wall_s=mm305_wall_seconds()-started;
            return out;
        }
        const char *seed_diag=getenv("KSP_LANDER_TAEM_DIAGNOSTICS");
        if (seed_diag && strcmp(seed_diag,"2")==0)
            fprintf(stderr,
                "MM305 seed route rejected: side=%+.0f radius=%.0f sweep=%.1f length=%.0f reason=%s elapsed=%.1f\n",
                seed.side,seed.route.hac.radius_m,
                seed.route.hac.arc_sweep_rad*RAD2DEG,
                seed.route.length_m,seed.reason?seed.reason:"unknown",
                seed.replay.elapsed_s);
    }
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
            const char *plan_diag=getenv("KSP_LANDER_TAEM_DIAGNOSTICS");
            if (plan_diag && strcmp(plan_diag,"2")==0) {
                fprintf(stderr,
                    "MM305 search pass: end=%s radius=%.0f selected=%d candidates=%d",
                    runway_end==1?"RECIP":"CONF",radius,
                    search.selected_candidate,search.candidate_count);
                for (int di=0;di<search.candidate_count;++di) {
                    const TaemFixedHacCandidate *dc=&search.candidates[di];
                    fprintf(stderr," [%c status=%d len=%.0f %s]",
                        dc->side>0.0?'R':'L',(int)dc->status,
                        dc->route_built?dc->route.length_m:NAN,
                        dc->reason?dc->reason:"unknown");
                }
                fprintf(stderr,"\n");
            }
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
    r.final_approach_distance_m=g->terminal_final_handoff_latched &&
        isfinite(g->terminal_final_handoff_distance) &&
        g->terminal_final_handoff_distance>=500.0 ?
        g->terminal_final_handoff_distance :
        mm305_energy_aware_final_distance(model,&r.state);
    if (!(r.final_approach_distance_m>=500.0) ||
            !isfinite(r.final_approach_distance_m))
        r.final_approach_distance_m=cfg->guidance.final_approach_distance;
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
    if (g->mm305_acquisition_route_valid &&
            g->mm305_acquisition_route.valid) {
        r.seed_route_valid = true;
        r.seed_route = g->mm305_acquisition_route;
        r.seed_runway_end = r.upstream_end;
    }
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
    double final_distance=-candidate->route.alignment_along_m;
    if (!(final_distance>=500.0) || !isfinite(final_distance))
        final_distance=cfg->guidance.final_approach_distance;
    double final_slope=candidate->route.profile_final_slope_deg;
    if (!(final_slope>=3.0 && final_slope<=45.0) || !isfinite(final_slope))
        final_slope=cfg->guidance.final_glide_slope;
    TerminalModel handoff_model={0};
    handoff_model.guidance=cfg->guidance;
    handoff_model.vehicle=cfg->vehicle;
    double final_speed=mm305_final_alignment_speed_for_distance(
        &handoff_model,final_distance);
    bool replan=g->mm305_route_committed;
    g->mm305_route=candidate->route;
    g->runway_end_index=candidate->runway_end;
    g->runway_end_preview_valid=true;
    g->runway_end_committed=true;
    fprintf(stderr,"MM305_ROUTE {\"runwayEnd\":%d,\"side\":%.17g,"
        "\"radius\":%.17g,\"sweep\":%.17g,\"leadLength\":%.17g,\"arcLength\":%.17g,"
        "\"entry\":[%.17g,%.17g],\"center\":[%.17g,%.17g],\"exit\":[%.17g,%.17g],"
        "\"finalDistance\":%.17g,\"finalSlope\":%.3f,\"finalSpeed\":%.1f,\"replan\":%s,\"scaleMach\":%.3f,"
        "\"liftScale\":%.4f,\"dragScale\":%.4f,"
        "\"solveWall\":%.3f}\n",
        candidate->runway_end,candidate->side,candidate->route.hac.radius_m,
        candidate->route.hac.arc_sweep_rad,candidate->route.lead_length_m,
        candidate->route.hac.arc_length_m,
        candidate->route.hac.entry.x,candidate->route.hac.entry.y,
        candidate->route.hac.center.x,candidate->route.hac.center.y,
        candidate->route.hac.exit.x,candidate->route.hac.exit.y,
        final_distance,final_slope,final_speed,replan?"true":"false",
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
    g->terminal_final_handoff_distance=final_distance;
    g->terminal_final_handoff_slope_deg=final_slope;
    g->terminal_final_handoff_speed_mps=final_speed;
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
        fprintf(stderr,"MM305 aero sample: ut=%.2f mach=%.3f aoa=%.2f raw=%.3f/%.3f clamped=%.3f/%.3f filtered=%.3f/%.3f measured=%.0f/%.0f modeled=%.0f/%.0f q=%.0f sb=%.3f\n",
            current->ut_s,modeled.mach,t->angle_of_attack,lift_raw,drag_raw,
            lift_ratio,drag_ratio,g->mm305_lift_scale,g->mm305_drag_scale,
            fabs(t->lift_force),fabs(t->drag_force),modeled.lift_n,modeled.drag_n,
            t->dynamic_pressure,g->taem_speedbrake_fraction);

}

/* Build one concrete provisional route without mutating guidance state.
 * Acquisition uses this both to compare real route lengths across sweep choices
 * and to install the chosen path. */
static bool mm305_make_acquisition_route(const GuidanceMachine *g,
        const TerminalModel *model, const TerminalDynamicState *current,
        const TaemGeometryState *geometry, double hac_radius, double side,
        double selected_sweep, TaemRoute *out, char *reason, size_t reason_size) {
    if (!model || !current || !geometry || !out) return false;

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
    bool built=taem_route_build_hac_shortest(model,geometry,hac_radius,side,
        selected_sweep,spacing,live_curvature,peak_curvature_limit,
        &route,reason,reason_size);
    if (!built)
        built=taem_route_build_hac(model,geometry,hac_radius,side,
            selected_sweep,spacing,live_curvature,&route,reason,reason_size);
    if (!built) {
        const char *diag=getenv("KSP_LANDER_TAEM_DIAGNOSTICS");
        if (diag && strcmp(diag,"2")==0 && g && !g->diagnostic_shadow)
            fprintf(stderr,"MM305 acquisition route build failed: %s\n",
                reason&&reason[0]?reason:"unknown");
        return false;
    }
    if (!taem_route_limit_initial_vertical_authority(
            model,current,geometry,&route)) {
        const char *diag=getenv("KSP_LANDER_TAEM_DIAGNOSTICS");
        if (diag && strcmp(diag,"2")==0 && g && !g->diagnostic_shadow)
            fprintf(stderr,
                "MM305 acquisition route rejected: initial vertical authority cannot support route curvature\n");
        return false;
    }
    *out=route;
    return true;
}

static bool mm305_install_acquisition_route(GuidanceMachine *g,
        const TerminalModel *model, const TerminalDynamicState *current,
        const TaemGeometryState *geometry, double hac_radius, double side,
        double selected_sweep, const TaemRoute *prebuilt) {
    if (!g) return false;
    TaemRoute route={0};
    char reason[160]={0};
    if (prebuilt && prebuilt->valid) route=*prebuilt;
    else if (!mm305_make_acquisition_route(g,model,current,geometry,hac_radius,
            side,selected_sweep,&route,reason,sizeof(reason)))
        return false;

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

    /* Choose using the route the tracker would actually fly, not a chord+
       arc approximation.  The old approximation could select a 135 deg sweep
       whose built cubic lead made the real route ~140 km even though the energy
       target was only ~40 km.  Re-evaluate at a modest cadence so the displayed
       and tracked path remains stable between updates. */
    bool refresh=!g->mm305_acquisition_route_valid||
        !g->mm305_acquisition_route.valid||
        current->ut_s-g->mm305_acquisition_route_ut>=4.0;
    if (refresh) {
        double selected_sweep=NAN;
        double selected_error=INFINITY;
        double selected_length=INFINITY;
        TaemRoute selected_route={0};

        /* Use the same radius freedom as qualification.  At the handoff save,
           a 12 km HAC can require a 90+ km finite lead while a tight HAC reduces
           that lead by tens of kilometres.  Acquisition must improve geometry,
           not lock itself to the nominal radius and wait for an impossible plan. */
        const double radius_candidates[]={
            hac_radius,
            fmax(3000.0,0.60*hac_radius),
            fmax(3000.0,0.36*hac_radius),
            3000.0
        };
        for (size_t ri=0;ri<sizeof(radius_candidates)/sizeof(radius_candidates[0]);++ri) {
            double candidate_radius=radius_candidates[ri];
            bool duplicate=false;
            for (size_t rj=0;rj<ri;++rj)
                if (fabs(candidate_radius-radius_candidates[rj])<1.0)
                    duplicate=true;
            if (duplicate) continue;

            for (double sweep_deg=15.0;sweep_deg<=270.0+1e-6;sweep_deg+=15.0) {
                double sweep=sweep_deg*DEG2RAD;
                TaemRoute trial={0};
                char reason[160]={0};
                if (!mm305_make_acquisition_route(g,model,current,geometry,
                        candidate_radius,side,sweep,&trial,reason,sizeof(reason)))
                    continue;
                double error=fabs(trial.length_m-required_path);
                bool trial_over=trial.length_m>required_path;
                bool selected_over=selected_route.valid&&selected_length>required_path;

                /* If every build is too long, the shortest route is the only
                   recoverable direction for an unpowered vehicle. Otherwise use
                   the closest energy length, breaking ties shorter. */
                bool prefer=false;
                if (!selected_route.valid) prefer=true;
                else if (trial_over && selected_over)
                    prefer=trial.length_m<selected_length-1.0;
                else if (error<selected_error-1.0)
                    prefer=true;
                else if (fabs(error-selected_error)<=1.0 &&
                        trial.length_m<selected_length)
                    prefer=true;
                if (prefer) {
                    selected_error=error;
                    selected_length=trial.length_m;
                    selected_sweep=sweep;
                    selected_route=trial;
                }
            }
        }
        if (!selected_route.valid || !isfinite(selected_sweep) ||
            !mm305_install_acquisition_route(g,model,current,geometry,
                selected_route.hac.radius_m,side,selected_sweep,&selected_route)) {
            g->mm305_acquisition_route_valid=false;
            return false;
        }
    }

    double selected_sweep=g->mm305_acquisition_sweep_rad;
    double selected_path=g->mm305_acquisition_route.length_m;
    bool have_route=g->mm305_acquisition_route_valid&&
        taem_route_reference(&g->mm305_acquisition_route,geometry,
            &g->mm305_acquisition_route_cursor,reference,NULL);
    if (!have_route)
        return false;

    double route_remaining=taem_route_remaining_at_index(
        &g->mm305_acquisition_route,g->mm305_acquisition_route_cursor);
    if (isfinite(route_remaining)&&route_remaining>0.0)
        selected_path=route_remaining;

    /* Acquisition owns the exact lateral route, but its analytic vertical
       curvature is not replay-qualified yet.  Feeding that curvature forward at
       the live handoff turned a ~1 deg FPA correction into >20 m/s^2 of required
       vertical lift.  Track the route FPA without altitude/curvature feed-forward,
       then back the requested pull-up toward the measured FPA until the native
       tracker reports adequate vertical authority.  A committed route retains its
       full tabulated vertical profile. */
    reference->altitude_m=NAN;
    reference->vertical_curvature_per_m=0.0;
    double route_fpa=reference->flight_path_angle_deg;
    double measured_fpa=geometry->flight_path_angle_deg;
    TaemTrackerOutput full=taem_tracker_update(model,current,geometry,reference,dt);
    if (!full.valid) return false;
    if (route_fpa>measured_fpa &&
            (!full.vertical_authority_ok ||
             full.required_vertical_lift_mps2>
                full.delivered_vertical_lift_mps2+0.25)) {
        TaemPathReference best=*reference;
        best.flight_path_angle_deg=measured_fpa;
        TaemTrackerOutput best_demand=taem_tracker_update(
            model,current,geometry,&best,dt);
        if (!best_demand.valid) return false;
        double lo=measured_fpa,hi=route_fpa;
        for (int i=0;i<8;++i) {
            double candidate=0.5*(lo+hi);
            TaemPathReference trial=*reference;
            trial.flight_path_angle_deg=candidate;
            TaemTrackerOutput td=taem_tracker_update(
                model,current,geometry,&trial,dt);
            bool ok=td.valid&&td.vertical_authority_ok&&
                td.required_vertical_lift_mps2<=
                    td.delivered_vertical_lift_mps2+0.25;
            if (ok) {
                lo=candidate;
                best=trial;
                best_demand=td;
            } else hi=candidate;
        }
        *reference=best;
        full=best_demand;
    }
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
    if (g->terminal_final_handoff_latched &&
            isfinite(g->terminal_final_handoff_distance) &&
            g->terminal_final_handoff_distance>=500.0) {
        live_model.guidance.final_approach_distance=g->terminal_final_handoff_distance;
        if (isfinite(g->terminal_final_handoff_slope_deg) &&
                g->terminal_final_handoff_slope_deg>=3.0 &&
                g->terminal_final_handoff_slope_deg<=45.0)
            live_model.guidance.final_glide_slope=g->terminal_final_handoff_slope_deg;
        if (isfinite(g->terminal_final_handoff_speed_mps) &&
                g->terminal_final_handoff_speed_mps>0.0)
            live_model.guidance.final_alignment_speed=g->terminal_final_handoff_speed_mps;
    }
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
            /* Geometric alignment is necessary but not sufficient for ownership
               transfer.  Keep MM305 on the runway line until Final's own priced
               delivery contract accepts the measured state; otherwise the old
               +/-250 m, +/-10 m/s readiness window could hand Final a state that
               it immediately had to abort. */
            TerminalPreflarePlan final_plan={0};
            bool final_approach=terminal_outer_capture_admissible(g,t,course,planet,aero,cfg,
                &final_plan);
            TaemTerminalContract final_contract=terminal_delivery_contract(g,t,course,planet,cfg,
                &final_plan);
            TaemTerminalEvaluation final_eval=
                taem_exec_evaluate_terminal_contract(&final_contract);
            if (final_approach && final_eval.valid && final_eval.feasible) {
                fprintf(stderr,
                    "MM305_LIVE_EXIT model=%llu side=%+.0f radius=%.0f sweep=%.1f lead=%.0f cursor=%zu/%zu refErr=%.2f runwayCourseErr=%.2f along=%.1f cross=%.1f h=%.1f airV=%.1f fpa=%.2f bank=%.2f aoa=%.2f cmdBank=%.2f cmdAoa=%.2f reason=final-contract-ready\n",
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

        /* An unpowered vehicle cannot correct an energy deficit with the
           speedbrake: it must exchange altitude for airspeed.  The certified
           route remains the geometric target, but while there is ample path
           remaining, bias the tracker toward a steeper local glide and a lower
           temporary altitude target.  Fade the bias out before the alignment
           station so the exact Final delivery geometry is still recovered. */
        TaemPathReference tracker_reference=reference;
        double tracker_plan_speed=taem_route_planned_speed(
            &g->mm305_route,reference.station_m);
        if (isfinite(tracker_plan_speed)&&tracker_plan_speed>1.0&&
                isfinite(reference.altitude_m)&&route_remaining>3500.0) {
            double dh=control_model->site.altitude+geometry.altitude_above_runway_m-
                reference.altitude_m;
            double equivalent=sqrt(fmax(0.0,
                geometry.airspeed_mps*geometry.airspeed_mps+2.0*9.81*dh));
            double ratio=equivalent/tracker_plan_speed-1.0;
            if (ratio<-0.03) {
                double strength=clampd((-ratio-0.03)/0.15,0.0,1.0);
                double fade=clampd((route_remaining-3500.0)/5000.0,0.0,1.0);
                double deficit_height=fmax(0.0,
                    (tracker_plan_speed*tracker_plan_speed-equivalent*equivalent)/(2.0*9.81));
                double altitude_relief=fmin(700.0,0.70*deficit_height)*fade;
                tracker_reference.altitude_m-=altitude_relief;
                tracker_reference.flight_path_angle_deg=clampd(
                    reference.flight_path_angle_deg-8.0*strength*fade,-35.0,-2.0);
                if (!g->diagnostic_shadow) {
                    const char *diag=getenv("KSP_LANDER_TAEM_DIAGNOSTICS");
                    if (diag&&strcmp(diag,"2")==0)
                        fprintf(stderr,
                            "MM305 energy recovery: ratio=%+.1f%% remaining=%.0f fpa %.1f->%.1f altRelief=%.0f\n",
                            100.0*ratio,route_remaining,reference.flight_path_angle_deg,
                            tracker_reference.flight_path_angle_deg,altitude_relief);
                }
            }
        }
        demand=taem_tracker_update(control_model,&current,&geometry,&tracker_reference,dt);
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
        bool energy_high=isfinite(energy_ratio)&&energy_ratio>0.10;
        bool energy_low=isfinite(energy_ratio)&&energy_ratio<-0.06;
        bool energy_off_plan=(energy_high||energy_low)&&isfinite(route_remaining);
        double energy_replan_interval=energy_low?10.0:replan_interval_s;
        if (g->mm305_route_committed && g->mm305_async_planning &&
            !g->mm305_planning_needed &&
            t->ut-g->mm305_last_plan_attempt_ut>=energy_replan_interval &&
            (energy_off_plan ||
             fabs(demand.cross_track_error_m)>1000.0 ||
             fabs(demand.course_error_deg)>20.0 ||
             (isfinite(reference.flight_path_angle_deg) &&
              fabs(geometry.flight_path_angle_deg-reference.flight_path_angle_deg)>6.0))) {
            double remaining=taem_route_remaining_at_index(
                &g->mm305_route,g->mm305_route_cursor);
            /* Excess energy can wait for generous route authority.  An energy
               deficit cannot: with the brake already shut, replanning is the only
               way to trade altitude for speed and shorten the remaining path. */
            double minimum_remaining=energy_low?
                fmax(4500.0,0.20*g->mm305_route.hac.arc_length_m):
                fmax(8000.0,0.35*g->mm305_route.hac.arc_length_m);
            if (isfinite(remaining) && remaining>minimum_remaining) {
                if (energy_off_plan && !g->diagnostic_shadow)
                    fprintf(stderr,
                        "MM305 energy off plan by %+.0f%% with %.0f m left; re-planning.\n",
                        100.0*energy_ratio,remaining);
                g->mm305_planning_needed=true;
                g->mm305_last_plan_attempt_ut=t->ut;
                g->mm305_plan_request_ut=t->ut;
            }
        }

        double exit_distance=hypot(
            geometry.runway_along_m-g->mm305_route.alignment_along_m,
            geometry.runway_cross_m);
        double alignment_capture=taem_alignment_capture_distance(
            control_model,&geometry);
        if (g->mm305_route_committed &&
            isfinite(route_remaining) && route_remaining<=alignment_capture &&
            exit_distance<=alignment_capture+500.0 &&
            fabs(demand.course_error_deg)<=12.0) {
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
    /* The replayed MM305 speed profile is a clean-glider profile: split-rudder
       drag is not part of terminal_solver propagation.  Therefore the brake is
       corrective authority only, never a nominal 25% bias.  Opening stays
       deliberately slow, while stowing is faster so a falling energy error does
       not turn into an unrecoverable underspeed several seconds later. */
    const double sb_cap=0.50, sb_acquisition_nominal=0.25;
    const double sb_deploy_slew_per_s=0.05, sb_stow_slew_per_s=0.20;
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
        /* A farther Final station already buys clean-glider dissipation distance.
           Do not spend the same split-rudder authority used by the short 8 km
           corridor or natural drag will compound it and create a late underspeed. */
        double extended_final=clampd(
            (control_model->guidance.final_approach_distance-8000.0)/2000.0,
            0.0,1.0);
        double committed_cap=sb_cap*(1.0-0.40*extended_final);
        double energy_deadband=0.01+0.03*extended_final;
        double brake_error=fmax(0.0,ratio-energy_deadband);
        double proportional_gain=2.5-0.8*extended_final;
        double integral_gain=0.04*(1.0-0.50*extended_final);
        sb_limit=committed_cap*sb_geometric_fade;
        if (brake_error>0.0)
            g->taem_speedbrake_integral=clampd(
                g->taem_speedbrake_integral+integral_gain*brake_error*sb_dt,
                0.0,sb_limit);
        else
            g->taem_speedbrake_integral=fmax(0.0,
                g->taem_speedbrake_integral-0.14*sb_dt);
        sb_target=clampd(proportional_gain*brake_error+
            g->taem_speedbrake_integral,0.0,sb_limit);
        if (diagnostics && strcmp(diagnostics,"2")==0 && !g->diagnostic_shadow)
            fprintf(stderr,
                "TAEM speedbrake: V=%.1f Veq=%.1f plan=%.1f dh=%.0f limit=%.2f target=%.2f actual=%.2f\n",
                geometry.airspeed_mps,equivalent,planned_speed,surplus_height,
                sb_limit,sb_target,g->taem_speedbrake_fraction);
    } else if (!g->mm305_route_committed &&
            isfinite(acquisition_required_drag) &&
            isfinite(acquisition_available_drag)) {
        double ratio=acquisition_required_drag/
            fmax(acquisition_available_drag,0.25)-1.0;
        double energy_high=clampd(ratio/0.15,0.0,1.0);
        sb_limit=sb_cap;
        double sb_base=sb_acquisition_nominal*energy_high;
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

    double sb_open_step=sb_deploy_slew_per_s*sb_dt;
    double sb_close_step=sb_stow_slew_per_s*sb_dt;
    g->taem_speedbrake_fraction=clampd(sb_target,
        g->taem_speedbrake_fraction-sb_close_step,
        g->taem_speedbrake_fraction+sb_open_step);
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
