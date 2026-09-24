#ifndef SHUTTLESIM_FLIGHT_PHYSICS_H
#define SHUTTLESIM_FLIGHT_PHYSICS_H

#include "aero.h"
#include "attitude.h"
#include "quat.h"
#include "world.h"

/* Minimal state carried by the shared, airborne native-tick plant. Ground
   handling, guidance and simulator protocol remain outside this interface. */
typedef struct {
    Vec3 position_i_m;
    Vec3 velocity_i_mps;
    AttitudeModel attitude;
    Quat body_q_i;
    AeroForces aero;
} FlightAirborneState;

typedef struct {
    double mass_kg;
    double ut;
    double dt_s;
    double burn_accel_mps2;
    double engine_thrust_n;
} FlightAirborneStepInput;

/* One discrete KSP-like native airborne tick. Force and attitude authority
   use the pre-step state; velocity, position, and then attitude are advanced
   in the same order as ShuttleSim's original sim_step implementation. */
void flight_physics_airborne_step(const KerbinWorld *world,
                                  const AeroTable *aero,
                                  const FlightAirborneStepInput *input,
                                  FlightAirborneState *state);

#endif
