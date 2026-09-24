#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "shuttlesim/flight_physics.h"
#include "shuttlesim/math3.h"

/* Frozen reference for the airborne portion of sim_step before extraction.
   Keep this intentionally local to the parity fixture: it represents the
   source ordering at the extraction boundary, not a second production API. */
static Vec3 reference_accel(const KerbinWorld *world, const AeroTable *aero,
                            const FlightAirborneStepInput *input,
                            const FlightAirborneState *state, Vec3 p, Vec3 v,
                            double ut) {
    AeroForces applied = aero_compute(world, aero, p, v, ut, input->mass_kg,
                                      state->attitude.aoa_rad,
                                      state->attitude.bank_rad);
    Vec3 accel = v3_add(world_gravity_accel(world, p),
                        v3_scale(applied.force_i, 1.0 / input->mass_kg));
    if (input->burn_accel_mps2 > 0.0) {
        Vec3 retro = v3_scale(v3_normalized(v), -input->burn_accel_mps2);
        accel = v3_add(accel, retro);
    }
    if (input->engine_thrust_n > 0.0 && input->mass_kg > 0.0) {
        Vec3 retro = v3_scale(v3_normalized(v),
                              -input->engine_thrust_n / input->mass_kg);
        accel = v3_add(accel, retro);
    }
    return accel;
}

static void reference_step(const KerbinWorld *world, const AeroTable *aero,
                           const FlightAirborneStepInput *input,
                           FlightAirborneState *state) {
    Vec3 p = state->position_i_m;
    Vec3 v = state->velocity_i_mps;
    double ut = input->ut;
    double dt = input->dt_s;
    AtmosphereSample atm = world_atmosphere_sample_state(world, p, ut);
    Vec3 vair = v3_sub(v, world_atmosphere_velocity_i(world, p));
    double q = 0.5 * atm.density_kg_m3 * v3_dot(vair, vair);
    Vec3 accel = reference_accel(world, aero, input, state, p, v, ut);

    state->velocity_i_mps = v3_add(v, v3_scale(accel, dt));
    state->position_i_m = v3_add(p, v3_scale(state->velocity_i_mps, dt));
    attitude_step(&state->attitude, q, dt);
    state->aero = aero_compute(world, aero, state->position_i_m,
                               state->velocity_i_mps, ut + dt, input->mass_kg,
                               state->attitude.aoa_rad,
                               state->attitude.bank_rad);
    vair = v3_sub(state->velocity_i_mps,
                  world_atmosphere_velocity_i(world, state->position_i_m));
    state->body_q_i = attitude_body_quat(state->position_i_m, vair,
                                         state->attitude.aoa_rad,
                                         state->attitude.bank_rad);
}

static int close_scalar(double a, double b) {
    return fabs(a - b) <= 1e-12 * fmax(1.0, fmax(fabs(a), fabs(b)));
}

static int same_vec(Vec3 a, Vec3 b) {
    return close_scalar(a.x, b.x) && close_scalar(a.y, b.y) &&
           close_scalar(a.z, b.z);
}

static int same_quat(Quat a, Quat b) {
    return close_scalar(a.w, b.w) && close_scalar(a.x, b.x) &&
           close_scalar(a.y, b.y) && close_scalar(a.z, b.z);
}

static int same_attitude(const AttitudeModel *a, const AttitudeModel *b) {
    return close_scalar(a->aoa_rad, b->aoa_rad) &&
           close_scalar(a->bank_rad, b->bank_rad) &&
           close_scalar(a->aoa_rate_rad_s, b->aoa_rate_rad_s) &&
           close_scalar(a->bank_rate_rad_s, b->bank_rate_rad_s) &&
           close_scalar(a->cmd_aoa_rad, b->cmd_aoa_rad) &&
           close_scalar(a->cmd_bank_rad, b->cmd_bank_rad) &&
           close_scalar(a->requested_aoa_rad, b->requested_aoa_rad) &&
           close_scalar(a->requested_bank_rad, b->requested_bank_rad) &&
           close_scalar(a->pitch_wn, b->pitch_wn) &&
           close_scalar(a->pitch_zeta, b->pitch_zeta) &&
           close_scalar(a->roll_wn, b->roll_wn) &&
           close_scalar(a->roll_zeta, b->roll_zeta) &&
           close_scalar(a->max_pitch_rate_rad_s, b->max_pitch_rate_rad_s) &&
           close_scalar(a->max_roll_rate_rad_s, b->max_roll_rate_rad_s) &&
           close_scalar(a->max_pitch_accel_rad_s2, b->max_pitch_accel_rad_s2) &&
           close_scalar(a->max_roll_accel_rad_s2, b->max_roll_accel_rad_s2) &&
           close_scalar(a->pitch_full_authority_q_pa,
                        b->pitch_full_authority_q_pa) &&
           close_scalar(a->roll_full_authority_q_pa,
                        b->roll_full_authority_q_pa);
}

static int same_aero(const AeroForces *a, const AeroForces *b) {
    return close_scalar(a->lift_n, b->lift_n) &&
           close_scalar(a->drag_n, b->drag_n) &&
           close_scalar(a->side_n, b->side_n) &&
           same_vec(a->force_i, b->force_i) &&
           close_scalar(a->mach, b->mach) &&
           close_scalar(a->dynamic_pressure_pa, b->dynamic_pressure_pa) &&
           close_scalar(a->airspeed_mps, b->airspeed_mps);
}

static unsigned next(unsigned *state) {
    *state = *state * 1664525u + 1013904223u;
    return *state;
}

int main(void) {
    KerbinWorld world;
    AeroTable aero;
    world_seed_kerbin(&world);
    aero_seed_stsn(&aero);

    unsigned seed = 305u;
    for (unsigned i = 0; i < 512; ++i) {
        double u0 = (double)(next(&seed) & 0xffffffu) / 16777215.0;
        double u1 = (double)(next(&seed) & 0xffffffu) / 16777215.0;
        double u2 = (double)(next(&seed) & 0xffffffu) / 16777215.0;
        double u3 = (double)(next(&seed) & 0xffffffu) / 16777215.0;
        double lat = deg2rad(-75.0 + 150.0 * u0);
        double lon = deg2rad(-180.0 + 360.0 * u1);
        double alt = 5000.0 + 65000.0 * u2;
        double ut = 1000.0 + 200000.0 * u3;
        FlightAirborneState extracted = {0};
        extracted.position_i_m = world_lla_to_inertial(&world, lat, lon,
                                                        alt, ut);
        LocalFrame frame = world_local_frame_i(&world,
                                               extracted.position_i_m, ut);
        double heading = deg2rad(-180.0 + 360.0 *
            ((double)(next(&seed) & 0xffffffu) / 16777215.0));
        double speed = 350.0 + 1500.0 *
            ((double)(next(&seed) & 0xffffffu) / 16777215.0);
        Vec3 air = v3_scale(v3_add(v3_scale(frame.north, cos(heading)),
                                   v3_scale(frame.east, sin(heading))), speed);
        extracted.velocity_i_mps = v3_add(air, world_atmosphere_velocity_i(
                                                  &world, extracted.position_i_m));
        attitude_seed(&extracted.attitude);
        extracted.attitude.aoa_rad = deg2rad(-5.0 + 45.0 *
            ((double)(next(&seed) & 0xffffffu) / 16777215.0));
        extracted.attitude.bank_rad = deg2rad(-70.0 + 140.0 *
            ((double)(next(&seed) & 0xffffffu) / 16777215.0));
        attitude_set_command(&extracted.attitude,
                             deg2rad(-5.0 + 45.0 *
                               ((double)(next(&seed) & 0xffffffu) / 16777215.0)),
                             deg2rad(-70.0 + 140.0 *
                               ((double)(next(&seed) & 0xffffffu) / 16777215.0)));
        extracted.body_q_i = attitude_body_quat(extracted.position_i_m, air,
                                                  extracted.attitude.aoa_rad,
                                                  extracted.attitude.bank_rad);
        extracted.aero = aero_compute(&world, &aero, extracted.position_i_m,
                                      extracted.velocity_i_mps, ut, 100000.0,
                                      extracted.attitude.aoa_rad,
                                      extracted.attitude.bank_rad);
        FlightAirborneState reference = extracted;
        FlightAirborneStepInput input = {
            .mass_kg = 80000.0 + 40000.0 * u0,
            .ut = ut,
            .dt_s = 0.005 + 0.04 * u1,
            .burn_accel_mps2 = (i % 3u == 0u) ? 0.15 * u2 : 0.0,
            .engine_thrust_n = (i % 4u == 0u) ? 400000.0 * u3 : 0.0
        };
        reference_step(&world, &aero, &input, &reference);
        flight_physics_airborne_step(&world, &aero, &input, &extracted);

        if (!same_vec(reference.position_i_m, extracted.position_i_m) ||
            !same_vec(reference.velocity_i_mps, extracted.velocity_i_mps) ||
            !same_attitude(&reference.attitude, &extracted.attitude) ||
            !same_quat(reference.body_q_i, extracted.body_q_i) ||
            !same_aero(&reference.aero, &extracted.aero)) {
            fprintf(stderr, "airborne parity failed at deterministic case %u\n", i);
            return EXIT_FAILURE;
        }
    }

    puts("flight_physics_parity: PASS (512 deterministic airborne ticks)");
    return EXIT_SUCCESS;
}
