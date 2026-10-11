#include <math.h>
#include "shuttlesim/flight_physics.h"
#include "shuttlesim/math3.h"

void flight_physics_airborne_step(const KerbinWorld *world,
                                  const AeroTable *aero,
                                  const FlightAirborneStepInput *input,
                                  FlightAirborneState *state) {
    if (!world || !aero || !input || !state) return;

    Vec3 p = state->position_i_m;
    Vec3 v = state->velocity_i_mps;
    double ut = input->ut;
    double dt = input->dt_s;
    AtmosphereSample attitude_atm = world_atmosphere_sample_state(world, p, ut);
    Vec3 attitude_vair = v3_sub(v, world_atmosphere_velocity_i(world, p));
    double attitude_speed = v3_norm(attitude_vair);
    double attitude_q = 0.5 * attitude_atm.density_kg_m3 * attitude_speed * attitude_speed;
    if(state->attitude.direct&&state->attitude.direct_body_coordinates&&
       !state->attitude.body_rates_initialized&&attitude_q>=state->attitude.direct_min_q_pa){
        state->body_q_i=attitude_body_quat_wind(p,attitude_vair,
                                               state->attitude.aoa_rad,
                                               state->attitude.bank_rad);
    }

    AeroForces applied_aero;
    applied_aero = aero_compute(world, aero, p, v, ut, input->mass_kg,
                                state->attitude.aoa_rad, state->attitude.bank_rad);
    Vec3 accel = v3_add(world_gravity_accel(world, p),
                        v3_scale(applied_aero.force_i, 1.0 / input->mass_kg));
    if (input->burn_accel_mps2 > 0.0) {
        Vec3 retro = v3_scale(v3_normalized(v), -input->burn_accel_mps2);
        accel = v3_add(accel, retro);
    }
    if (input->engine_thrust_n > 0.0 && input->mass_kg > 0.0) {
        Vec3 axis = input->engine_thrust_dir_i;
        Vec3 dir = v3_norm(axis) > 0.5 ? v3_normalized(axis) : v3_scale(v3_normalized(v), -1.0);
        accel = v3_add(accel, v3_scale(dir, input->engine_thrust_n / input->mass_kg));
    }
    state->velocity_i_mps = v3_add(v, v3_scale(accel, dt));
    state->position_i_m = v3_add(p, v3_scale(state->velocity_i_mps, dt));

    Vec3 vair = v3_sub(state->velocity_i_mps,
                       world_atmosphere_velocity_i(world, state->position_i_m));
    bool body_step=attitude_step_body(&state->attitude,&state->body_q_i,
                                      state->position_i_m,vair,attitude_q,dt);
    if(!body_step){
        state->attitude.body_rates_initialized=false;
        attitude_step(&state->attitude, attitude_q, dt);
        state->body_q_i = attitude_body_quat(state->position_i_m, vair,
                                             state->attitude.aoa_rad,
                                             state->attitude.bank_rad);
    }

    state->aero = aero_compute(world, aero, state->position_i_m,
                               state->velocity_i_mps, ut + dt, input->mass_kg,
                               state->attitude.aoa_rad, state->attitude.bank_rad);
}
