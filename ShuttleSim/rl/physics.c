/* Offline-only ABI. No sockets, actuator control, or changes to sim physics. */
#include "shuttlesim/sim.h"
#include "shuttlesim/math3.h"
#include <math.h>
#include <stdlib.h>


typedef struct {
  Simulation sim;
  char telemetry[8192];
  char summary[4096];
  bool initial_conditions_set;
} OfflineSim;

void *offline_create(const char *scenario, const char *atmosphere,
                     const char *aero, const char *attitude) {
  Scenario initial;
  if (!scenario_load(scenario, &initial))
    return NULL;
  OfflineSim *env = calloc(1, sizeof(*env));
  if (!env)
    return NULL;
  sim_init(&env->sim, &initial);
  if (!world_load_atmosphere_csv(&env->sim.world, atmosphere) ||
      !aero_load_csv(&env->sim.aero, aero) ||
      !attitude_load_ini(&env->sim.state.attitude, attitude)) {
    free(env);
    return NULL;
  }
  return env;
}

void offline_destroy(void *handle) { free(handle); }

/* Advance one guidance interval.  The caller supplies elapsed simulator time;
 * the simulator derives exact physics substeps from its configured timestep,
 * including a partial final step.  Replay speed is independent of a wall clock
 * and the ABI has no fixed tick-count cap. */
int offline_step(void *handle, double aoa, double bank, int gear, int brakes,
                 double duration_s) {
  if (!handle || !isfinite(aoa) || !isfinite(bank) ||
      !isfinite(duration_s) || !(duration_s > 0.0))
    return 0;
  OfflineSim *env = handle;
  const double physics_dt_s = env->sim.physics_dt_s;
  if (!isfinite(physics_dt_s) || !(physics_dt_s > 0.0))
    return 0;
  SimCommand command = {.has_attitude = true,
                        .aoa_deg = aoa,
                        .bank_deg = bank,
                        .has_gear = true,
                        .gear_down = gear != 0,
                        .has_brakes = true,
                        .brakes = brakes != 0};
  sim_apply_command(&env->sim, &command);
  double elapsed_s = 0.0;
  while (elapsed_s < duration_s) {
    const double remaining_s = duration_s - elapsed_s;
    const double step_s = fmin(physics_dt_s, remaining_s);
    if (!isfinite(step_s) || !(step_s > 0.0))
      return 0;
    sim_step(&env->sim, step_s);
    const double next_elapsed_s = elapsed_s + step_s;
    if (!(next_elapsed_s > elapsed_s))
      return 0;
    elapsed_s = next_elapsed_s;
  }
  return 1;
}

const char *offline_telemetry(void *handle) {
  OfflineSim *env = handle;
  sim_build_telemetry_json(&env->sim, 0, env->telemetry,
                           sizeof(env->telemetry));
  return env->telemetry;
}

const char *offline_summary(void *handle) {
  OfflineSim *env = handle;
  sim_build_summary_json(&env->sim, env->summary, sizeof(env->summary));
  return env->summary;
}

int offline_runway_contains(void *handle, double along_m, double cross_m) {
  if (!handle || !isfinite(along_m) || !isfinite(cross_m))
    return 0;
  OfflineSim *env = handle;
  return runway_contains(&env->sim.runway, along_m, cross_m) ? 1 : 0;
}

/* Re-seed the atmospheric entry state for one simulator-only episode.  The
 * state is expressed in local flight variables so the experiment can vary
 * entry energy and orbital-plane geometry without writing scenario files or
 * changing the live UDP path.  speed_mps is surface-relative speed and
 * heading_deg is local north-clockwise heading.  This ABI is deliberately
 * simulator-only; the expanded altitude/speed envelope below supports
 * curriculum rollouts near the MM304/TAEM interface and never touches live UDP
 * control. */
int offline_set_initial_conditions(void *handle, double altitude_m,
                                   double speed_mps, double flight_path_angle_deg,
                                   double heading_deg, double latitude_deg,
                                   double longitude_deg) {
  if (!handle || !isfinite(altitude_m) || !isfinite(speed_mps) ||
      !isfinite(flight_path_angle_deg) || !isfinite(heading_deg) ||
      !isfinite(latitude_deg) || !isfinite(longitude_deg))
    return 0;
  OfflineSim *e = handle;
  double radius=e->sim.world.radius_m;
  if (e->initial_conditions_set || e->sim.state.sim_elapsed_s != 0 ||
      !(radius > 0.0) || altitude_m <= -radius ||
      !(speed_mps > 0.0) ||
      flight_path_angle_deg <= -90.0 || flight_path_angle_deg >= 90.0 ||
      fabs(latitude_deg) > 90.0)
    return 0; /* decision-literal-ok: spherical-coordinate and positive-speed domains */

  double ut = e->sim.state.ut;
  Vec3 position = world_lla_to_inertial(
      &e->sim.world, deg2rad(latitude_deg), deg2rad(longitude_deg),
      altitude_m, ut);
  LocalFrame frame = world_local_frame_i(&e->sim.world, position, ut);
  double heading = deg2rad(heading_deg);
  double fpa = deg2rad(flight_path_angle_deg);
  Vec3 horizontal = v3_normalized(v3_add(
      v3_scale(frame.north, cos(heading)),
      v3_scale(frame.east, sin(heading))));
  Vec3 surface = v3_add(v3_scale(horizontal, speed_mps * cos(fpa)),
                        v3_scale(frame.up, speed_mps * sin(fpa)));
  e->sim.state.position_i_m = position;
  e->sim.state.velocity_i_mps = v3_add(
      surface, world_atmosphere_velocity_i(&e->sim.world, position));
  e->sim.state.body_q_i = attitude_body_quat(
      position, surface, e->sim.state.attitude.aoa_rad,
      e->sim.state.attitude.bank_rad);
  e->sim.state.aero = aero_compute(
      &e->sim.world, &e->sim.aero, position, e->sim.state.velocity_i_mps,
      ut, e->sim.state.mass_kg, e->sim.state.attitude.aoa_rad,
      e->sim.state.attitude.bank_rad);
  e->initial_conditions_set = true;
  return 1;
}

/* Build a pre-deorbit Keplerian orbit, perform an instantaneous retrograde
 * deorbit burn at apoapsis, and leave the simulator at the post-burn state.
 * The orbit parameters are intentionally restricted to the simulator-only
 * deorbit domain; atmospheric guidance still starts from the resulting
 * telemetry and does not receive orbital-element labels. */
int offline_set_deorbit_orbit(void *handle, double apoapsis_altitude_m,
                              double periapsis_altitude_m, double inclination_deg,
                              double raan_deg, double argument_of_periapsis_deg,
                              double target_periapsis_altitude_m) {
  if (!handle || !isfinite(apoapsis_altitude_m) ||
      !isfinite(periapsis_altitude_m) || !isfinite(inclination_deg) ||
      !isfinite(raan_deg) || !isfinite(argument_of_periapsis_deg) ||
      !isfinite(target_periapsis_altitude_m))
    return 0;
  OfflineSim *e = handle;
  const double radius = e->sim.world.radius_m;
  const double mu = e->sim.world.mu_m3_s2;
  const double atmosphere_top = e->sim.world.atmosphere_top_m;
  /* Input validity is orbital/planetary, not a hand-tuned test corridor.  A
   * pre-deorbit orbit must stay outside the atmosphere; the post-burn
   * periapsis must intersect it; inclination uses the standard [0, pi] orbital
   * element domain.  Sampling coverage is owned by the experiment, not here. */
  if (e->initial_conditions_set || e->sim.state.sim_elapsed_s != 0 ||
      !(radius > 0.0) || !(mu > 0.0) || !(atmosphere_top >= 0.0) ||
      apoapsis_altitude_m < atmosphere_top ||
      periapsis_altitude_m < atmosphere_top ||
      periapsis_altitude_m > apoapsis_altitude_m ||
      inclination_deg < 0.0 || inclination_deg > 180.0 ||
      target_periapsis_altitude_m <= -radius ||
      target_periapsis_altitude_m >= atmosphere_top ||
      target_periapsis_altitude_m >= periapsis_altitude_m)
    return 0;

  const double ra = radius + apoapsis_altitude_m;
  const double rp = radius + periapsis_altitude_m;
  const double target_rp = radius + target_periapsis_altitude_m;
  const double a = 0.5 * (ra + rp);
  const double target_a = 0.5 * (ra + target_rp);
  const double pre_speed = sqrt(mu * (2.0 / ra - 1.0 / a));
  const double post_speed = sqrt(mu * (2.0 / ra - 1.0 / target_a));
  const double delta_v = pre_speed - post_speed;
  if (!isfinite(pre_speed) || !isfinite(post_speed) ||
      !isfinite(delta_v) || !(delta_v > 0.0))
    return 0;

  double ut = e->sim.state.ut;
  double phase = e->sim.world.rotation_phase_rad_at_ut0 +
                 e->sim.world.rotation_rate_rad_s * ut;
  double raan = deg2rad(raan_deg) + phase;
  double argument = deg2rad(argument_of_periapsis_deg);
  double ci = cos(deg2rad(inclination_deg));
  double si = sin(deg2rad(inclination_deg));
  double co = cos(raan), so = sin(raan), cw = cos(argument), sw = sin(argument);
  Vec3 periapsis_axis = ss_v3(co * cw - so * sw * ci,
                           so * cw + co * sw * ci, sw * si);
  Vec3 transverse_axis = ss_v3(-co * sw - so * cw * ci,
                            -so * sw + co * cw * ci, cw * si);
  /* The burn is at apoapsis: true anomaly pi, so position and velocity are
   * opposite the perifocal axes.  The argument of periapsis is allowed to
   * move the atmospheric crossing onto a realistic MM304/TAEM ground track. */
  Vec3 position = v3_scale(periapsis_axis, -ra);
  Vec3 pre_velocity = v3_scale(transverse_axis, -pre_speed);
  Vec3 post_velocity = v3_add(
      pre_velocity, v3_scale(v3_normalized(pre_velocity), -delta_v));
  e->sim.state.position_i_m = position;
  e->sim.state.velocity_i_mps = post_velocity;
  e->sim.state.body_q_i = attitude_body_quat(
      position, post_velocity, e->sim.state.attitude.aoa_rad,
      e->sim.state.attitude.bank_rad);
  e->sim.state.aero = aero_compute(
      &e->sim.world, &e->sim.aero, position, post_velocity, ut,
      e->sim.state.mass_kg, e->sim.state.attitude.aoa_rad,
      e->sim.state.attitude.bank_rad);
  e->initial_conditions_set = true;
  return 1;
}

/* Episode-local uncertainty, never writes calibration files. Call only at
 * reset. */
int offline_randomize(void *handle, double mass, double density, double lift,
                      double drag, double lag, double velocity) {
  if (!handle || !isfinite(mass) || !isfinite(density) || !isfinite(lift) ||
      !isfinite(drag) || !isfinite(lag) || !isfinite(velocity) || mass < .95 ||
      mass > 1.05 || density < .9 || density > 1.1 || lift < .9 || lift > 1.1 ||
      drag < .9 || drag > 1.1 || lag < .9 || lag > 1.1 || fabs(velocity) > 2)
    return 0;
  OfflineSim *e = handle;
  if (e->sim.state.sim_elapsed_s != 0)
    return 0;
  e->sim.state.mass_kg *= mass;
  e->sim.state.velocity_i_mps.x += velocity;
  /* The simulator has no low-level servo lag model.  Preserve the stress
     dimension by varying the configured target-following rate instead. */
  e->sim.state.attitude.max_pitch_rate_rad_s *= lag;
  e->sim.state.attitude.max_roll_rate_rad_s *= lag;
  for (size_t i = 0; i < e->sim.world.atmosphere.count; ++i)
    e->sim.world.atmosphere.p[i].sample.density_kg_m3 *= density;
  for (size_t i = 0; i < e->sim.aero.mach_count; ++i)
    for (size_t j = 0; j < e->sim.aero.alpha_count; ++j) {
      e->sim.aero.cl[i][j] *= lift;
      e->sim.aero.cd[i][j] *= drag;
    }
  for (size_t i = 0; i < e->sim.aero.book_count; ++i) {
    e->sim.aero.book[i].lift_per_q_m2 *= lift;
    e->sim.aero.book[i].drag_per_q_m2 *= drag;
  }
  return 1;
}
int offline_load_book(void *handle, const char *path) {
  if (!handle)
    return 0;
  return aero_load_book_csv(&((OfflineSim *)handle)->sim.aero, path);
}
