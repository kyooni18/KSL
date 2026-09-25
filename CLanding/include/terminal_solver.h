#ifndef CLANDING_TERMINAL_SOLVER_H
#define CLANDING_TERMINAL_SOLVER_H

#include "taem_planner.h"

typedef enum {
    TERMINAL_SOLVER_QUALIFIED = 0,
    TERMINAL_SOLVER_INFEASIBLE,
    TERMINAL_SOLVER_UNQUALIFIED,
    TERMINAL_SOLVER_SEARCH_EXHAUSTED,
    TERMINAL_SOLVER_INVALID_INPUT
} TerminalSolverStatus;

typedef struct {
    TerminalSolverStatus status;
    bool path_constraints_ok;
    bool final_interface_qualified;
    TerminalDynamicState final_state;
    TaemGeometryState final_geometry;
    double elapsed_s;
    double minimum_speed_mps;
    double maximum_dynamic_pressure_pa;
    double maximum_load_g;
    double maximum_cross_track_m;
    double maximum_course_error_deg;
    double maximum_altitude_error_m;
    double maximum_lateral_authority_shortfall_mps2;
    double maximum_vertical_authority_shortfall_mps2;
    double failure_cross_track_m;
    double failure_course_error_deg;
    double failure_required_lateral_accel_mps2;
    double failure_available_lateral_accel_mps2;
    double failure_required_vertical_lift_mps2;
    double failure_delivered_vertical_lift_mps2;
    double failure_target_altitude_m;
    double failure_target_flight_path_angle_deg;
    size_t failure_route_index;
    double energy_start_j_kg;
    double energy_end_j_kg;
    double drag_work_j_kg;
    double energy_closure_residual_j_kg;
    size_t final_route_index;
    double final_target_altitude_m;
    double final_target_flight_path_angle_deg;
    const char *reason;
} TerminalSolverResult;

/* Full-horizon deterministic replay against the shared native airborne tick.
 * A successful route solve ends at the HAC exit / Final gate. It does not by
 * itself certify the downstream Final tail, so status remains UNQUALIFIED. */
typedef struct {
    bool valid;            /* profile written into the route */
    double aoa_deg;        /* constant incidence that meets the exit altitude */
    double exit_speed_mps;
    double exit_fpa_deg;
    const char *reason;
} TerminalProfileResult;

/* Generates a dynamically feasible vertical reference for a lateral route:
 * the vehicle is propagated natively along the route (lateral tracker bank)
 * at constant AoA, bisected so it arrives at the route's exit altitude.  On
 * success the altitude/FPA table is written into route (profile_tabulated).
 * Fails when no AoA brackets the exit altitude: the route is too long for the
 * available energy, or too short to dissipate it. */
TerminalProfileResult terminal_solver_generate_profile(const TerminalModel *model,
        const TerminalDynamicState *initial, TaemRoute *route, double dt_s,
        double maximum_elapsed_s);

TerminalSolverResult terminal_solver_replay(const TerminalModel *model,
        const TerminalDynamicState *initial, const TaemRoute *route,
        double dt_s, double maximum_elapsed_s);

#endif
