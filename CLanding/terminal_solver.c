#include "terminal_solver.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "taem_frames_energy.h"
#include "taem_tracker.h"
#include "shuttlesim/aero.h"
#include "shuttlesim/math3.h"

static bool state_energy(const TerminalModel *m, const TerminalDynamicState *s,
        AeroForces forces, TaemEnergyDiagnostic *energy) {
    TaemFrameWorld world;
    if (!taem_geometry_world(m, &world)) return false;
    TaemVec3 pi = {s->position_i_m.x, s->position_i_m.y, s->position_i_m.z};
    TaemVec3 vi = {s->velocity_i_mps.x, s->velocity_i_mps.y, s->velocity_i_mps.z};
    TaemVec3 pb, vb;
    if (!taem_inertial_to_fixed(&world, pi, vi, s->ut_s, &pb, &vb)) return false;
    Vec3 force_fixed_native = world_inertial_to_fixed(&m->world, forces.force_i, s->ut_s);
    TaemVec3 fi = {forces.force_i.x, forces.force_i.y, forces.force_i.z};
    TaemVec3 fb = {force_fixed_native.x, force_fixed_native.y, force_fixed_native.z};
    TaemVec3 air = vb; /* Current ShuttleSim atmosphere is co-rotating and windless. */
    return taem_energy_diagnostic(&world, pi, vi, vb, air, fi, fb,
        s->mass_kg, forces.drag_n, energy);
}
/* Steepest sustained descent MM305 may plan when the vehicle starts above the
 * exit glide line, as an excess over the Final glide slope.  Steeper
 * descents are not flown: the route is rejected so a longer one is chosen. */
#define TERMINAL_PROFILE_MAX_DESCENT_EXCESS_DEG 10.0
/* Incidence of the late energy-building dive: low enough to accelerate,
 * with dynamic pressure and load still checked by the full replay. */
#define TERMINAL_PROFILE_DIVE_AOA_DEG 3.0
#define TERMINAL_PROFILE_PULLOUT_M 1000.0
#define TERMINAL_PROFILE_TURN_MARGIN 1.3

typedef struct {
    double station_m, altitude_m, fpa_deg;
} ProfileSample;

typedef struct {
    double end_altitude_m;   /* MSL at route end; -INFINITY if runway reached first */
    double capture_station_m; /* INFINITY if the exit glide line was never met */
    double exit_speed_mps, exit_fpa_deg;
    double exit_specific_energy_j_kg;
    bool valid;
} ProfileFlight;

static double max_profile_aoa(const TerminalModel *m) {
    double aoa = fmin(m->vehicle.maximum_angle_of_attack,
                      m->aero.alpha_deg[m->aero.alpha_count - 1]);
    if (isfinite(m->vehicle.terminal_maximum_lift_angle_of_attack) &&
        m->vehicle.terminal_maximum_lift_angle_of_attack > 0.0)
        aoa = fmin(aoa, m->vehicle.terminal_maximum_lift_angle_of_attack);
    return aoa;
}

/* Fly the lateral route at constant AoA (lateral tracker supplies bank),
 * switching to dive_aoa_deg beyond dive_station_m.  A dive that steepens to
 * descent_limit_deg holds that flight-path angle instead.  With
 * capture_line set, once the vehicle descends onto the exit glide line (the
 * straight line through the HAC exit at the Final slope) it tracks that line
 * to the exit.  Optionally records (station, altitude, FPA). */
static ProfileFlight fly_profile(const TerminalModel *m,
        const TerminalDynamicState *initial, const TaemRoute *route,
        double aoa_deg, double dive_station_m, double dive_aoa_deg,
        double descent_limit_deg, bool capture_line, double dt, double max_elapsed,
        ProfileSample *samples, size_t sample_capacity, size_t *sample_count) {
    const double pi = 3.14159265358979323846;
    ProfileFlight out = {.end_altitude_m = NAN, .capture_station_m = INFINITY,
                         .exit_speed_mps = NAN, .exit_fpa_deg = NAN,
                         .exit_specific_energy_j_kg = NAN};
    TerminalDynamicState state = *initial;
    TaemGeometryState geometry;
    if (!taem_geometry_state(m, &state, &geometry)) return out;
    size_t cursor = 0, count = 0;
    double station = 0.0;
    double bank_max = fmin(m->vehicle.maximum_bank_angle, 80.0) * pi / 180.0;
    double line_slope = tan(route->profile_final_slope_deg * pi / 180.0);
    bool on_line = false, limited = false, start_above_line = true;
    size_t max_steps = (size_t)ceil(max_elapsed / dt);
    for (size_t step = 0; step < max_steps; ++step) {
        TaemPathReference reference;
        size_t index = cursor;
        if (!taem_route_reference(route, &geometry, &cursor, &reference, &index))
            return out;
        station = reference.station_m;
        double altitude = m->site.altitude + geometry.altitude_above_runway_m;
        if (samples && count < sample_capacity)
            samples[count++] = (ProfileSample){station, altitude,
                geometry.flight_path_angle_deg};
        if (station >= route->length_m) {
            AeroForces forces = aero_compute(&m->world,&m->aero,
                state.position_i_m,state.velocity_i_mps,state.ut_s,state.mass_kg,
                state.attitude.aoa_rad,state.attitude.bank_rad);
            TaemEnergyDiagnostic energy;
            if (!state_energy(m,&state,forces,&energy)) return out;
            out.exit_specific_energy_j_kg = energy.effective_specific_energy_j_kg;
            if (sample_count) *sample_count = count;
            out.end_altitude_m = altitude;
            out.exit_speed_mps = geometry.airspeed_mps;
            out.exit_fpa_deg = geometry.flight_path_angle_deg;
            out.valid = true;
            return out;
        }
        double line_altitude = route->profile_final_altitude_m +
            fmax(0.0, route->length_m - station) * line_slope;
        if (step == 0) start_above_line = altitude > line_altitude;
        if (capture_line && !on_line && step > 0 &&
            (start_above_line ? altitude <= line_altitude : altitude >= line_altitude)) {
            on_line = true;
            out.capture_station_m = station;
        }
        if (!on_line && station >= dive_station_m && isfinite(descent_limit_deg) &&
            geometry.flight_path_angle_deg <= -descent_limit_deg)
            limited = true;
        TerminalControl control;
        if (on_line || limited) {
            reference.altitude_m = on_line ? line_altitude : NAN;
            reference.flight_path_angle_deg = on_line ?
                -route->profile_final_slope_deg : -descent_limit_deg;
            reference.vertical_curvature_per_m = 0.0;
            TaemTrackerOutput demand = taem_tracker_update(m, &state, &geometry,
                                                           &reference, dt);
            if (!demand.valid) return out;
            control = demand.control;
        } else {
            double required_lateral = taem_tracker_lateral_demand(m, &geometry,
                                                                  &reference);
            if (!isfinite(required_lateral)) return out;
            double aoa_cmd = station < dive_station_m ? aoa_deg : dive_aoa_deg;
            double aoa = aoa_cmd * pi / 180.0;
            AeroForces forces = aero_compute(&m->world, &m->aero,
                state.position_i_m, state.velocity_i_mps, state.ut_s,
                state.mass_kg, aoa, 0.0);
            double lift = fabs(forces.lift_n) / state.mass_kg;
            /* Never pick an incidence whose lift cannot produce the lateral
             * acceleration this station of the route requires within the
             * bank limit (with margin): the turn is not optional. */
            double needed = TERMINAL_PROFILE_TURN_MARGIN *
                fabs(required_lateral) / sin(bank_max);
            while (lift < needed && aoa_cmd < max_profile_aoa(m)) {
                aoa_cmd = fmin(max_profile_aoa(m), aoa_cmd + 0.5);
                aoa = aoa_cmd * pi / 180.0;
                forces = aero_compute(&m->world, &m->aero, state.position_i_m,
                    state.velocity_i_mps, state.ut_s, state.mass_kg, aoa, 0.0);
                lift = fabs(forces.lift_n) / state.mass_kg;
            }
            double bank = lift > 1e-6 ? asin(fmax(-1.0, fmin(1.0,
                required_lateral / lift))) : 0.0;
            control = (TerminalControl){.angle_of_attack_rad = aoa,
                .bank_rad = fmax(-bank_max, fmin(bank_max, bank)), .dt_s = dt};
        }
        if (terminal_propagator_step(m, &state, &control) != TERMINAL_STEP_OK)
            return out;
        if (!taem_geometry_state(m, &state, &geometry)) return out;
        if (geometry.altitude_above_runway_m <= 0.0) {
            if (sample_count) *sample_count = count;
            out.end_altitude_m = -INFINITY;
            out.valid = true;
            return out;
        }
    }
    return out;
}


TerminalProfileResult terminal_solver_generate_profile(const TerminalModel *m,
        const TerminalDynamicState *initial, TaemRoute *route, double dt,
        double max_elapsed) {
    TerminalProfileResult out = {0};
    out.reason = "invalid profile input";
    if (!m || !m->replay_validated || !terminal_model_validate(m,NULL,0) ||
        !terminal_state_validate(initial,NULL,0) || !route || !route->valid ||
        !(dt > 0.0) || !(max_elapsed > 0.0) || !isfinite(dt) || !isfinite(max_elapsed) ||
        !isfinite(max_elapsed/dt) || max_elapsed/dt >= (double)SIZE_MAX/sizeof(ProfileSample)-1.0)
        return out;
    TaemGeometryState start;
    if (!taem_geometry_state(m, initial, &start)) return out;
    const double pi = 3.14159265358979323846;
    double target = route->profile_final_altitude_m;
    double start_altitude = m->site.altitude + start.altitude_above_runway_m;
    bool above_line = start_altitude > target +
        route->length_m * tan(route->profile_final_slope_deg * pi / 180.0);

    /* End altitude is not monotonic in AoA: range peaks near the best-L/D
     * incidence and falls again toward maximum lift.  Locate the
     * maximum-range incidence; solutions are taken on the low-AoA (faster)
     * branch below it. */
    double best_aoa = NAN, high_end = -INFINITY, best_energy = -INFINITY;
    for (double aoa = 2.0; aoa <= max_profile_aoa(m) + 1e-9; aoa += 2.0) {
        ProfileFlight f = fly_profile(m, initial, route, aoa, INFINITY, aoa, NAN, false, dt,
                                      max_elapsed, NULL, 0, NULL);
        if (!f.valid) continue;
        if (!isfinite(f.end_altitude_m)) { if (!isfinite(high_end)) high_end = -INFINITY; continue; }
        high_end = fmax(high_end, f.end_altitude_m);
        double energy = f.exit_specific_energy_j_kg;
        if (energy > best_energy) { best_energy = energy; best_aoa = aoa; }
    }
    if (!isfinite(best_aoa)) {
        out.reason = high_end == -INFINITY ?
            "route is too long for the available energy" :
            "profile propagation failed";
        return out;
    }
    if (high_end < target) {
        out.reason = "route is too long for the available energy";
        return out;
    }
    double aoa_low = 0.5, aoa_high = best_aoa;
    bool capture = false;
    double dive_station = INFINITY, dive_aoa = aoa_low, descent_limit = NAN;
    if (above_line) {
        /* Energy-optimal shape for an unpowered vehicle that must reach the
         * exit glide line fast: glide at the maximum-range incidence (energy
         * kept as altitude), dive at low incidence late, then capture and hold
         * the exit glide line for the pull-out.  The dive may not steepen past
         * the descent limit: it then holds that slope, and a route that cannot
         * reach the line within it is rejected rather than plunged.  Bisect
         * the dive start so the capture happens one pull-out before the exit. */
        double capture_target = route->length_m - TERMINAL_PROFILE_PULLOUT_M;
        dive_aoa = fmin(best_aoa, TERMINAL_PROFILE_DIVE_AOA_DEG);
        descent_limit = route->profile_final_slope_deg +
            TERMINAL_PROFILE_MAX_DESCENT_EXCESS_DEG;
        ProfileFlight steep = fly_profile(m, initial, route, best_aoa, 0.0,
            dive_aoa, descent_limit, true, dt, max_elapsed, NULL, 0, NULL);
        if (!steep.valid || !(steep.capture_station_m <= capture_target)) {
            out.reason = "route is too short to descend within the slope limit";
            return out;
        }
        double early = 0.0, late = route->length_m;
        for (int iteration = 0; iteration < 12; ++iteration) {
            double s = 0.5 * (early + late);
            ProfileFlight f = fly_profile(m, initial, route, best_aoa, s,
                dive_aoa, descent_limit, true, dt, max_elapsed, NULL, 0, NULL);
            if (!f.valid) { out.reason = "profile propagation failed"; return out; }
            if (f.capture_station_m <= capture_target) early = s; else late = s;
        }
        dive_station = early;
        aoa_high = best_aoa;
        capture = true;
    } else {
        /* Below the exit glide line an energetic vehicle flies a shallower
         * path until the line descends onto it.  The lowest incidence that
         * still meets the line one pull-out before the exit keeps the most
         * speed; states that never meet it fall back to meeting the exit
         * altitude directly. */
        double capture_target = route->length_m - TERMINAL_PROFILE_PULLOUT_M;
        ProfileFlight meet = fly_profile(m, initial, route, best_aoa, INFINITY,
            best_aoa, NAN, true, dt, max_elapsed, NULL, 0, NULL);
        if (meet.valid && meet.capture_station_m <= capture_target) {
            double shallow = best_aoa;
            for (int iteration = 0; iteration < 10; ++iteration) {
                double aoa = 0.5 * (aoa_low + shallow);
                ProfileFlight f = fly_profile(m, initial, route, aoa, INFINITY,
                    aoa, NAN, true, dt, max_elapsed, NULL, 0, NULL);
                if (!f.valid) { out.reason = "profile propagation failed"; return out; }
                if (f.capture_station_m <= capture_target) shallow = aoa; else aoa_low = aoa;
            }
            aoa_high = shallow;
            dive_aoa = shallow;
            capture = true;
        }
    }
    if (!above_line && !capture) {
        ProfileFlight low = fly_profile(m, initial, route, aoa_low, INFINITY, aoa_low, NAN, false, dt,
                                        max_elapsed, NULL, 0, NULL);
        if (!low.valid) { out.reason = "profile propagation failed"; return out; }
        if (low.end_altitude_m > target) {
            out.reason = "route is too short to dissipate the available energy";
            return out;
        }
        for (int iteration = 0; iteration < 10; ++iteration) {
            double aoa = 0.5 * (aoa_low + aoa_high);
            ProfileFlight f = fly_profile(m, initial, route, aoa, INFINITY, aoa, NAN, false, dt,
                                          max_elapsed, NULL, 0, NULL);
            if (!f.valid) { out.reason = "profile propagation failed"; return out; }
            if (f.end_altitude_m < target) aoa_low = aoa; else aoa_high = aoa;
        }
    }
    size_t capacity = (size_t)ceil(max_elapsed / dt) + 1;
    ProfileSample *samples = calloc(capacity, sizeof(*samples));
    if (!samples) { out.reason = "profile allocation failed"; return out; }
    size_t count = 0;
    ProfileFlight flight = fly_profile(m, initial, route, aoa_high,
        dive_station, dive_aoa, descent_limit, capture, dt,
        max_elapsed, samples, capacity, &count);
    if (!flight.valid || !isfinite(flight.end_altitude_m) || count < 2) {
        free(samples);
        out.reason = "profile propagation failed";
        return out;
    }
    /* Resample the actual projected route stations. Flight distance includes
     * cross-track corrections and must never stretch an early endpoint onto
     * the runway-anchored route exit. */
    const size_t intervals = TAEM_ROUTE_PROFILE_LUT_POINTS - 1;
    size_t k = 0;
    for (size_t j = 0; j <= intervals; ++j) {
        double s = route->length_m * (double)j / (double)intervals;
        while (k + 2 < count && samples[k + 1].station_m < s) ++k;
        double s0 = samples[k].station_m, s1 = samples[k + 1].station_m;
        double f = s1 > s0 ? fmax(0.0, fmin(1.0, (s - s0) / (s1 - s0))) : 0.0;
        route->profile_altitude_lut[j] = (float)((1.0 - f) * samples[k].altitude_m +
                                                 f * samples[k + 1].altitude_m);
        route->profile_fpa_lut[j] = (float)((1.0 - f) * samples[k].fpa_deg +
                                            f * samples[k + 1].fpa_deg);
    }
    free(samples);
    route->profile_total_length_m = route->length_m;
    route->profile_generation_aoa_deg = aoa_high;
    route->profile_tabulated = true;
    out.valid = true;
    out.aoa_deg = aoa_high;
    out.exit_speed_mps = flight.exit_speed_mps;
    out.exit_fpa_deg = flight.exit_fpa_deg;
    out.reason = capture ?
        "native slope-limited dive captured the exit glide line" :
        "native constant-incidence profile meets the exit altitude";
    return out;
}

TerminalSolverResult terminal_solver_replay(const TerminalModel *m,
        const TerminalDynamicState *initial, const TaemRoute *route,
        double dt, double max_elapsed) {
    TerminalSolverResult out;
    memset(&out, 0, sizeof(out));
    out.status = TERMINAL_SOLVER_INVALID_INPUT;
    out.reason = "invalid solver input";
    if (!m || !m->replay_validated || !terminal_model_validate(m, NULL, 0) ||
        !terminal_state_validate(initial, NULL, 0) || !route || !route->valid ||
        route->count < 3 || !(dt > 0.0) || !(max_elapsed > 0.0) ||
        !isfinite(dt) || !isfinite(max_elapsed) || !isfinite(max_elapsed/dt) ||
        max_elapsed/dt >= (double)SIZE_MAX/sizeof(ProfileSample)-1.0) return out;

    TerminalDynamicState state = *initial;
    TaemGeometryState geometry;
    if (!taem_geometry_state(m, &state, &geometry)) {
        out.reason = "initial runway-frame projection failed";
        return out;
    }
    size_t cursor = 0;
    double initial_speed = geometry.airspeed_mps;
    out.minimum_speed_mps = initial_speed;
    out.energy_start_j_kg = NAN;
    out.energy_end_j_kg = NAN;
    AeroForces initial_forces = aero_compute(&m->world, &m->aero,
        state.position_i_m, state.velocity_i_mps, state.ut_s, state.mass_kg,
        state.attitude.aoa_rad, state.attitude.bank_rad);
    TaemEnergyDiagnostic energy;
    if (state_energy(m, &state, initial_forces, &energy))
        out.energy_start_j_kg = energy.effective_specific_energy_j_kg;

    bool reached_gate = false;
    double elapsed = 0.0;
    size_t max_steps = (size_t)ceil(max_elapsed / dt);
    const char *diagnostics = getenv("KSP_LANDER_TAEM_DIAGNOSTICS");
    for (size_t step = 0; step < max_steps; ++step) {
        TaemPathReference reference;
        size_t reference_index = cursor;
        if (!taem_route_reference(route, &geometry, &cursor, &reference,
                                  &reference_index)) {
            out.status = TERMINAL_SOLVER_INVALID_INPUT;
            out.reason = "route reference lookup failed";
            break;
        }
        if (diagnostics && strcmp(diagnostics, "1") == 0 && step < 8)
            fprintf(stderr,
                "TAEM replay step: bow=%+.0f k=%zu idx=%zu along=%.0f refAlong=%.0f cross=%.0f refCross=%.0f course=%.1f refCourse=%.1f kappa=%g alt=%.0f refAlt=%.0f gamma=%.2f refGamma=%.2f\n",
                route->profile_midpoint_offset_m, step, reference_index,
                geometry.runway_along_m, reference.runway_along_m,
                geometry.runway_cross_m, reference.runway_cross_m,
                geometry.course_deg, reference.course_deg,
                reference.curvature_right_per_m,
                geometry.altitude_above_runway_m,
                reference.altitude_m, geometry.flight_path_angle_deg,
                reference.flight_path_angle_deg);
        TaemTrackerOutput demand = taem_tracker_update(m, &state, &geometry,
                                                       &reference, dt);
        if (diagnostics && strcmp(diagnostics, "2") == 0 &&
            step % (size_t)fmax(1.0, round(2.5 / dt)) == 0)
            fprintf(stderr,
                "TAEM replay trace: t=%.1f idx=%zu along=%.0f cross=%.0f h=%.0f course=%.1f ref=%.1f kappa=%.3g xt=%.0f crsErr=%.1f fpa=%.1f refFpa=%.1f bank=%.1f cmdBank=%.1f aoa=%.1f cmdAoa=%.1f\n",
                elapsed, reference_index, geometry.runway_along_m,
                geometry.runway_cross_m, geometry.altitude_above_runway_m,
                geometry.course_deg, reference.course_deg,
                reference.curvature_right_per_m, demand.cross_track_error_m,
                demand.course_error_deg, geometry.flight_path_angle_deg,
                reference.flight_path_angle_deg,
                state.attitude.bank_rad * 57.29577951308232,
                demand.control.bank_rad * 57.29577951308232,
                state.attitude.aoa_rad * 57.29577951308232,
                demand.control.angle_of_attack_rad * 57.29577951308232);
        if (!demand.valid) {
            out.status = TERMINAL_SOLVER_INFEASIBLE;
            out.reason = "tracker could not produce a bounded command";
            break;
        }
        out.failure_target_altitude_m = reference.altitude_m;
        out.failure_target_flight_path_angle_deg =
            reference.flight_path_angle_deg;
        out.failure_route_index = reference_index;
        out.failure_cross_track_m = demand.cross_track_error_m;
        out.failure_course_error_deg = demand.course_error_deg;
        out.failure_required_lateral_accel_mps2 = demand.required_lateral_accel_mps2;
        out.failure_available_lateral_accel_mps2 = demand.available_lateral_accel_mps2;
        out.failure_required_vertical_lift_mps2 = demand.required_vertical_lift_mps2;
        out.failure_delivered_vertical_lift_mps2 = demand.delivered_vertical_lift_mps2;
        out.maximum_lateral_authority_shortfall_mps2 = fmax(
            out.maximum_lateral_authority_shortfall_mps2,
            fabs(demand.required_lateral_accel_mps2) -
                demand.available_lateral_accel_mps2);
        out.maximum_vertical_authority_shortfall_mps2 = fmax(
            out.maximum_vertical_authority_shortfall_mps2,
            fabs(demand.required_vertical_lift_mps2 -
                 demand.delivered_vertical_lift_mps2));
        out.maximum_cross_track_m = fmax(out.maximum_cross_track_m,
                                         fabs(demand.cross_track_error_m));
        out.maximum_course_error_deg = fmax(out.maximum_course_error_deg,
                                            fabs(demand.course_error_deg));
        /* Both axes are rate/authority limited by the native attitude model.
         * The tracker returns the best physically achievable AoA/bank command, so
         * transient saturation must be propagated rather than treated as an
         * instantaneous proof of infeasibility.  Replay rejects the route from the
         * resulting cross-track/course/altitude/FPA/speed/energy envelope instead. */

        AeroForces forces = aero_compute(&m->world, &m->aero,
            state.position_i_m, state.velocity_i_mps, state.ut_s, state.mass_kg,
            state.attitude.aoa_rad, state.attitude.bank_rad);
        double altitude = v3_norm(state.position_i_m) - m->world.radius_m;
        double local_radius = m->world.radius_m + altitude;
        double gravity = m->world.mu_m3_s2 / (local_radius * local_radius);
        double load = v3_norm(forces.force_i) / state.mass_kg / fmax(gravity, 1e-9);
        out.maximum_load_g = fmax(out.maximum_load_g, load);
        out.maximum_dynamic_pressure_pa = fmax(out.maximum_dynamic_pressure_pa,
                                               forces.dynamic_pressure_pa);
        out.minimum_speed_mps = fmin(out.minimum_speed_mps, forces.airspeed_mps);
        if (forces.dynamic_pressure_pa > m->vehicle.maximum_dynamic_pressure * (1.0 + 1e-9) ||
            load > m->vehicle.maximum_g_load * (1.0 + 1e-6)) {
            out.status = TERMINAL_SOLVER_INFEASIBLE;
            out.reason = "candidate exceeded dynamic-pressure or load limit";
            break;
        }
        double drag_work_rate = -forces.drag_n * forces.airspeed_mps / state.mass_kg;
        out.drag_work_j_kg += drag_work_rate * dt;
        TerminalStepStatus status = terminal_propagator_step(m, &state,
                                                              &demand.control);
        if (status != TERMINAL_STEP_OK) {
            out.status = TERMINAL_SOLVER_INFEASIBLE;
            out.reason = "native airborne propagation failed";
            break;
        }
        elapsed += dt;
        if (!taem_geometry_state(m, &state, &geometry)) {
            out.status = TERMINAL_SOLVER_INFEASIBLE;
            out.reason = "runway-frame state became invalid";
            break;
        }
        if (geometry.altitude_above_runway_m <= 0.0) {
            out.status = TERMINAL_SOLVER_INFEASIBLE;
            out.reason = "candidate crossed runway elevation before the HAC exit";
            break;
        }
        double emergency_cross_track_m = fmax(3000.0, 0.25 * route->hac.radius_m);
        if (fabs(demand.cross_track_error_m) > emergency_cross_track_m ||
            fabs(demand.course_error_deg) > 45.0) {
            out.status = TERMINAL_SOLVER_INFEASIBLE;
            out.reason = "candidate diverged from the MM305 tracking corridor";
            break;
        }
        double current_altitude = m->site.altitude +
                                  geometry.altitude_above_runway_m;
        out.maximum_altitude_error_m = fmax(out.maximum_altitude_error_m,
            fabs(current_altitude - reference.altitude_m));
        AeroForces end_forces = aero_compute(&m->world, &m->aero,
            state.position_i_m, state.velocity_i_mps, state.ut_s, state.mass_kg,
            state.attitude.aoa_rad, state.attitude.bank_rad);
        if (state_energy(m, &state, end_forces, &energy))
            out.energy_end_j_kg = energy.effective_specific_energy_j_kg;

        double end_distance = hypot(geometry.runway_along_m - route->hac.exit.x,
                                    geometry.runway_cross_m - route->hac.exit.y);
        bool at_final_route_point = reference_index + 2 >= route->count;
        if (at_final_route_point && end_distance <= 700.0 &&
            fabs(demand.course_error_deg) <= 12.0) {
            reached_gate = true;
            out.final_route_index = reference_index;
            out.final_target_altitude_m = reference.altitude_m;
            out.final_target_flight_path_angle_deg = reference.flight_path_angle_deg;
            break;
        }
        out.final_route_index = reference_index;
    }

    out.final_state = state;
    out.final_geometry = geometry;
    out.elapsed_s = elapsed;
    if (isfinite(out.energy_start_j_kg) && isfinite(out.energy_end_j_kg))
        out.energy_closure_residual_j_kg = out.energy_end_j_kg -
            out.energy_start_j_kg - out.drag_work_j_kg;
    if (reached_gate) {
        double energy_scale = fmax(fabs(out.drag_work_j_kg),
                                   0.5 * initial_speed * initial_speed);
        double energy_tolerance = fmax(250.0, 0.02 * energy_scale);
        bool energy_closure_ok = isfinite(out.energy_closure_residual_j_kg) &&
            fabs(out.energy_closure_residual_j_kg) <= energy_tolerance;
        bool path_geometry_ok = out.maximum_cross_track_m <=
            fmax(1200.0, 0.1 * route->hac.radius_m) &&
            out.maximum_course_error_deg <= 20.0 &&
            out.maximum_altitude_error_m <= 2000.0 &&
            fabs(out.final_geometry.altitude_above_runway_m + m->site.altitude -
                 out.final_target_altitude_m) <= 1200.0 &&
            fabs(out.final_geometry.flight_path_angle_deg -
                 out.final_target_flight_path_angle_deg) <= 6.0;
        bool speed_ok = out.minimum_speed_mps >= m->vehicle.minimum_safe_speed;
        out.path_constraints_ok = path_geometry_ok && speed_ok && energy_closure_ok;
        if (out.path_constraints_ok) {
            out.status = TERMINAL_SOLVER_UNQUALIFIED;
            out.reason = "HAC exit reached; downstream Final tail is not qualified";
        } else {
            out.status = TERMINAL_SOLVER_INFEASIBLE;
            out.reason = !path_geometry_ok ? "candidate exceeded path tracking envelope" :
                (!speed_ok ? "candidate fell below the vehicle minimum safe speed" :
                 "native energy and drag-work closure exceeded tolerance");
        }
    } else if (out.status == TERMINAL_SOLVER_INVALID_INPUT && elapsed > 0.0) {
        out.status = TERMINAL_SOLVER_SEARCH_EXHAUSTED;
        out.reason = "native replay reached its wall-simulation horizon before HAC exit";
    }
    if (diagnostics && strcmp(diagnostics, "1") == 0)
        fprintf(stderr,
            "TAEM replay result: status=%d elapsed=%.2f index=%zu reason=%s lat=%.2f/%.2f vert=%.2f/%.2f\n",
            (int)out.status, out.elapsed_s, out.failure_route_index,
            out.reason ? out.reason : "none",
            out.failure_required_lateral_accel_mps2,
            out.failure_available_lateral_accel_mps2,
            out.failure_required_vertical_lift_mps2,
            out.failure_delivered_vertical_lift_mps2);
    return out;
}
