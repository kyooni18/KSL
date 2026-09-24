#include "terminal_propagator.h"

#include <math.h>

#include "shuttlesim/flight_physics.h"

static bool finite_vec3(Vec3 v) {
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

TerminalStepStatus terminal_propagator_step(const TerminalModel *m,
        TerminalDynamicState *s, const TerminalControl *u) {
    if (!m || !s || !u || !(u->dt_s > 0.0) || !isfinite(u->dt_s) ||
        !isfinite(u->angle_of_attack_rad) || !isfinite(u->bank_rad) ||
        !(s->mass_kg > 0.0) || !isfinite(s->mass_kg) || !isfinite(s->ut_s) ||
        !finite_vec3(s->position_i_m) || !finite_vec3(s->velocity_i_mps))
        return TERMINAL_STEP_INVALID_INPUT;
    if (!m->replay_validated || !terminal_state_validate(s, NULL, 0))
        return TERMINAL_STEP_INVALID_INPUT;

    FlightAirborneState plant = {
        .position_i_m = s->position_i_m,
        .velocity_i_mps = s->velocity_i_mps,
        .attitude = s->attitude,
        .body_q_i = {1.0, 0.0, 0.0, 0.0},
        .aero = {0}
    };
    attitude_set_command(&plant.attitude, u->angle_of_attack_rad, u->bank_rad);
    FlightAirborneStepInput input = {
        .mass_kg = s->mass_kg,
        .ut = s->ut_s,
        .dt_s = u->dt_s,
        .burn_accel_mps2 = 0.0,
        .engine_thrust_n = 0.0
    };
    flight_physics_airborne_step(&m->world, &m->aero, &input, &plant);
    if (!finite_vec3(plant.position_i_m) || !finite_vec3(plant.velocity_i_mps) ||
        !isfinite(plant.attitude.aoa_rad) || !isfinite(plant.attitude.bank_rad))
        return TERMINAL_STEP_INVALID_RESULT;
    s->position_i_m = plant.position_i_m;
    s->velocity_i_mps = plant.velocity_i_mps;
    s->attitude = plant.attitude;
    s->ut_s += u->dt_s;
    return isfinite(s->ut_s) ? TERMINAL_STEP_OK : TERMINAL_STEP_INVALID_RESULT;
}
