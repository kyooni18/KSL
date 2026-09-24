#include "terminal_solver.h"

#include <math.h>
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
        !isfinite(dt) || !isfinite(max_elapsed)) return out;

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
    for (size_t step = 0; step < max_steps; ++step) {
        TaemPathReference reference;
        size_t reference_index = cursor;
        if (!taem_route_reference(route, &geometry, &cursor, &reference,
                                  &reference_index)) {
            out.status = TERMINAL_SOLVER_INVALID_INPUT;
            out.reason = "route reference lookup failed";
            break;
        }
        TaemTrackerOutput demand = taem_tracker_update(m, &state, &geometry,
                                                       &reference, dt);
        if (!demand.valid) {
            out.status = TERMINAL_SOLVER_INFEASIBLE;
            out.reason = "tracker could not produce a bounded command";
            break;
        }
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
            demand.required_vertical_lift_mps2 - demand.delivered_vertical_lift_mps2);
        out.maximum_cross_track_m = fmax(out.maximum_cross_track_m,
                                         fabs(demand.cross_track_error_m));
        out.maximum_course_error_deg = fmax(out.maximum_course_error_deg,
                                            fabs(demand.course_error_deg));

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
    return out;
}
