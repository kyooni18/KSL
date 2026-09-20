#ifndef SHUTTLESIM_SIM_H
#define SHUTTLESIM_SIM_H
#include <stdbool.h>
#include "types.h"
#include "world.h"
#include "aero.h"
#include "attitude.h"
#include "scenario.h"
#include "protocol.h"

typedef struct {
    double ut;
    double sim_elapsed_s;
    Vec3 position_i_m;
    Vec3 velocity_i_mps;
    double mass_kg;
    AttitudeModel attitude;
    Quat body_q_i;
    bool gear_down;
    bool brakes;
    bool on_ground;
    bool touchdown_seen;
    bool on_runway_at_touchdown;
    double touchdown_sink_mps;
    double touchdown_speed_mps;
    double touchdown_along_m;
    double touchdown_cross_m;
    AeroForces aero;
} SimState;

typedef struct {
    KerbinWorld world;
    Runway runway;
    AeroTable aero;
    Scenario scenario;
    SimState state;
    double physics_dt_s;
    double rolling_mu;
    double brake_mu;
    bool paused;
    double deorbit_burn_delay_remaining_s;
    double deorbit_burn_remaining_s;
    double deorbit_burn_pending_duration_s;
    double deorbit_impulse_pending_mps;
    double deorbit_burn_accel_mps2;
} Simulation;

void sim_init(Simulation *sim, const Scenario *scenario);
void sim_apply_deorbit_initial(Simulation *sim);
void sim_step(Simulation *sim, double dt);
void sim_apply_command(Simulation *sim, const SimCommand *cmd);
void sim_set_attitude(Simulation *sim, double aoa_deg, double bank_deg);
void sim_build_telemetry_json(const Simulation *sim, double observed_rate, char *out, size_t out_size);
void sim_build_summary_json(const Simulation *sim, char *out, size_t out_size);
#endif
