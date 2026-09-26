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
    bool airbrakes;
    double wheel_steering;
    double ground_lateral_accel_mps2;
    double ground_airbrake_decel_mps2;
    bool on_ground;
    bool touchdown_seen;
    bool on_runway_at_touchdown;
    double touchdown_sink_mps;
    double touchdown_speed_mps;
    double touchdown_along_m;
    double touchdown_cross_m;
    bool touchdown_gear;
    bool tail_strike;
    double touchdown_pitch_deg;
    AeroForces aero;
    /* Landing-gear contact model (sim.c).  Point order: main, nose, tail. */
    double contact_prev_height_m[3];
    bool contact_prev_valid;
    double contact_load_n[3];
    bool main_contact;
    bool nose_contact;
    /* Post-mains contract: after the first main-wheel contact pitch is free
       (stick zero unless the direct-control FCS says otherwise) and evolves as
       a rigid body under identified aerodynamic and gear moments. */
    bool pitch_released;
    double released_pitch_rad;
    double released_pitch_rate_rad_s;
    double released_prev_fpa_rad;
    Vec3 released_heading_i;   /* horizontal nose direction, kept when slow */
    /* Touchdown/rollout classification. */
    bool main_touchdown_seen;
    double main_touchdown_ut;
    bool nose_touchdown_seen;
    double nose_touchdown_sink_mps;
    double nose_touchdown_delay_s;
    int bounce_count;
    double airborne_after_contact_s;
    double max_gear_load_g;
    bool runway_departure;
    double departure_along_m, departure_cross_m, departure_speed_mps;
    bool stopped;
    double stop_along_m, stop_cross_m;
    /* Non-survivable impact (structure contact far beyond gear capability):
       the vehicle is destroyed and the state freezes at the impact point. */
    bool crashed;
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
    double ground_airbrake_cda_m2;
    double ground_max_lateral_accel_g;
    double ground_max_yaw_rate_rad_s;
    /* Extended-gear drag area.  Live KSP identification finds no measurable
       gear increment (dCD_gear = -0.003, ShuttleSim/reference-model/
       stsn_aero_identified.report.json), so the default is zero. */
    double gear_cda_m2;
    /* Pitch inertia for gear moments: SURROGATE (radius of gyration 5.5 m at
       40 t), not identified.  Aerodynamic pitch moments come from the
       identified direct-control coefficients, which are already per inertia. */
    double pitch_inertia_kg_m2;
    bool paused;
    double deorbit_burn_delay_remaining_s;
    double deorbit_burn_remaining_s;
    double deorbit_burn_pending_duration_s;
    double deorbit_impulse_pending_mps;
    double deorbit_burn_accel_mps2;
    double commanded_throttle;
} Simulation;

void sim_init(Simulation *sim, const Scenario *scenario);
void sim_apply_deorbit_initial(Simulation *sim);
void sim_step(Simulation *sim, double dt);
void sim_apply_command(Simulation *sim, const SimCommand *cmd);
void sim_set_attitude(Simulation *sim, double aoa_deg, double bank_deg);
void sim_build_telemetry_json(const Simulation *sim, double observed_rate, char *out, size_t out_size);
void sim_build_summary_json(const Simulation *sim, char *out, size_t out_size);
#endif
