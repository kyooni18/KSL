/* Simulator-only deterministic teacher. Mapping copied from guidance_runner.c.
 */
#define _POSIX_C_SOURCE 200809L
#include "json.h"
#include "landing.h"
#include "decision_envelope.h"
#include "sim_telemetry.h"
#include "terminal_model.h"

#include <arpa/inet.h>
#include <errno.h>
#include <float.h>
#include <math.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

static const char *phase_name(GuidancePhase p) {
  static const char *n[] = {"IDLE",
                            "PLANNING",
                            "CALIBRATION",
                            "COAST",
                            "BURN_SETUP",
                            "DEORBIT_BURN",
                            "ENTRY_INTERFACE",
                            "ENTRY_ENERGY",
                            "TAEM",
                            "HEADING_ALIGNMENT",
                            "FINAL",
                            "FLARE",
                            "TOUCHDOWN",
                            "ROLLOUT",
                            "COMPLETE",
                            "PAUSED",
                            "ABORT",
                            "FAULT",
                            "ATTITUDE_RECOVERY"};
  return p >= 0 && p < (int)(sizeof(n) / sizeof(n[0])) ? n[p] : "?";
}

static bool load_config(const char *path, LandingConfiguration *c) {
  *c = landing_configuration_default();
  FILE *f = fopen(path, "rb");
  if (!f) {
    landing_configuration_normalize(c);
    return false;
  }
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  rewind(f);
  char *buf = calloc((size_t)n + 1, 1);
  if (!buf) {
    fclose(f);
    return false;
  }
  fread(buf, 1, (size_t)n, f);
  fclose(f);
  JsonToken *toks = calloc(16384, sizeof(*toks));
  JsonDoc d;
  bool ok = toks && json_parse(buf, toks, 16384, &d) > 0 &&
            d.tokens[0].type == JSON_OBJECT &&
            landing_configuration_from_json(c, &d, 0);
  free(toks);
  free(buf);
  landing_configuration_normalize(c);
  return ok;
}


static bool parse_packet(const char *buf, const LandingConfiguration *cfg,
                         const PlanetModel *p, const Telemetry *previous,
                         Telemetry *t, VehicleState *s) {
  char error[256];
  if (!shuttle_sim_decode_telemetry(buf, p, previous, t, s, error,
                                    sizeof(error)))
    return false;
  shuttle_sim_prepare_guidance_telemetry(t, cfg, p);
  return true;
}

/* Offline API follows below. */
typedef struct {
  LandingConfiguration cfg;
  PlanetModel planet;
  AerodynamicModel aero;
  AerodynamicEnvelope envelope;
  TrajectoryCalibrationModel calibration;
  DeorbitPlan plan;
  GuidanceMachine g;
  TerminalModel terminal_model;
  bool initialized, have_prev;
  Telemetry previous;
  char result[8192];
} OfflineExpert;

static const char *terminal_source_path(const char *environment_name,
        const char *atmosphere_path,const char *filename,char *buffer,
        size_t buffer_size) {
  const char *configured=getenv(environment_name);
  if(configured&&configured[0])return configured;
  if(!atmosphere_path||!atmosphere_path[0])return filename;
  const char *slash=strrchr(atmosphere_path,'/');
  size_t directory_length=slash?(size_t)(slash-atmosphere_path):0;
  if(directory_length+1+strlen(filename)+1>buffer_size)return filename;
  if(directory_length)memcpy(buffer,atmosphere_path,directory_length);
  buffer[directory_length]='/';
  strcpy(buffer+directory_length+1,filename);
  return buffer;
}

void *expert_create(const char *config, const char *atmosphere) {
  OfflineExpert *e = calloc(1, sizeof(*e));
  if (!e)
    return NULL;
  if (!load_config(config, &e->cfg)) {
    free(e);
    return NULL;
  }
  char error[256];
  if (!shuttle_sim_load_planet(atmosphere, &e->planet, error, sizeof(error))) {
    fprintf(stderr, "expert: %s\n", error);
    free(e);
    return NULL;
  }
  char aero_path[1024],book_path[1024],attitude_path[1024],reason[256];
  TerminalModelSourceFiles model_files={
    .atmosphere_csv=atmosphere,
    .aero_csv=terminal_source_path("KSP_LANDER_TERMINAL_AERO",atmosphere,
        "stsn_aero_ksp_robust.csv",aero_path,sizeof(aero_path)),
    .aero_book_csv=terminal_source_path("KSP_LANDER_TERMINAL_AERO_BOOK",atmosphere,
        "stsn_force_book.csv",book_path,sizeof(book_path)),
    .attitude_ini=terminal_source_path("KSP_LANDER_TERMINAL_ATTITUDE",atmosphere,
        "stsn_attitude_ksp.ini",attitude_path,sizeof(attitude_path))
  };
  if(!terminal_model_capture(&e->terminal_model,&model_files,&e->cfg,1,
      reason,sizeof(reason))){
    fprintf(stderr,"expert: native MM305 model unavailable: %s\n",reason);
    free(e);
    return NULL;
  }
  e->aero =
      (AerodynamicModel){e->cfg.vehicle.estimated_lift_to_drag,
                         e->cfg.vehicle.estimated_ballistic_coefficient, .9};
  for (int i = 0; i < 4; ++i)
    e->envelope.regimes[i] = e->aero;
  e->calibration.density_scale = e->calibration.drag_scale =
      e->calibration.lift_scale = 1;
  e->calibration.bank_effectiveness = e->calibration.speed_of_sound_scale = 1;
  e->calibration.speed_of_sound = 300;
  e->calibration.stress_drag_scale = e->calibration.stress_lift_scale = 1;
  e->calibration.confidence = .9;
  e->plan.execution_qualified = e->plan.target_capture_achieved = true;
  e->plan.predicted_post_burn_periapsis_altitude = 50000;
  guidance_machine_init(&e->g);
  return e;
}
void expert_destroy(void *handle) { free(handle); }

/* Called once per control interval; never drives actuators or opens sockets.
 * Phase/reversal/HAC/gear ownership deliberately remains with the teacher.
 */
const char *expert_command(void *handle, const char *packet) {
  OfflineExpert *e = handle;
  Telemetry t;
  VehicleState s = {0};
  if (!e || !parse_packet(packet, &e->cfg, &e->planet,
                          e->have_prev ? &e->previous : NULL, &t, &s))
    return NULL;
  e->previous = t;
  e->have_prev = true;
  if (!e->initialized) {
    guidance_initialize_reentry_continuation(&e->g, &t, &e->planet, &e->cfg, 1,
                                             false, &e->envelope,
                                             &e->calibration);
    e->initialized = true;
  }
  double ref = e->g.entry_reference_speed > 0 ? e->g.entry_reference_speed
                                              : t.true_air_speed;
  EntryTerminalDemand demand = entry_terminal_demand(
      t.latitude, t.mean_altitude, t.range_to_site, t.horizontal_speed,
      t.course_to_site_error, t.vertical_speed, t.true_air_speed, ref,
      &e->planet, &e->cfg.vehicle, &e->cfg.site, &e->cfg.guidance);
  t.energy_excess_range = -demand.projected_taem_range_error;
  GuidanceResult r =
      guidance_update_with_terminal_model(&e->g,&t,&s,&e->plan,&e->planet,
          e->aero,&e->cfg,&e->terminal_model);
  double aoa = r.command.has_target_aoa
                   ? r.command.target_aoa
                   : r.command.target_pitch - t.flight_path_angle;
  double bank = r.command.target_roll;
  double stall = vessel_physics_conservative_stall_fraction(
      t.true_air_speed, t.angle_of_attack, t.dynamic_pressure, &e->cfg.vehicle);
  double bank_limit = entry_bank_authority_limit(
      t.true_air_speed, t.dynamic_pressure, t.g_force, &e->cfg.vehicle,
      e->cfg.vehicle.maximum_bank_angle);
  TaemInterfaceCapture capture = entry_taem_interface_capture(
      &e->g.taem_interface_target, &t, t.ground_track_heading, &e->planet,
      &e->cfg);
  capture.ready = capture.valid && capture.veto == 0;
  double floor = fmax(
      entry_low_q_protective_aoa_floor(t.dynamic_pressure, &e->cfg.vehicle),
      entry_terminal_turn_aoa_floor(t.true_air_speed, t.dynamic_pressure,
                                    &e->cfg.vehicle));
  bool valid = isfinite(aoa) && isfinite(bank) && isfinite(stall);
  /* Restrict learning to a small entry residual.  A GuidanceResult warning is
   * not itself a physical abort: the simulator starts from a deliberately
   * unqualified post-burn checkpoint, so the deterministic MM304 supervisor
   * commonly publishes an advisory warning while the vehicle remains inside
   * the aerodynamic envelope.  Treating every advisory as a hard gate made
   * the learner receive zero action-dependent steps forever.
   *
   * Hard physical protection remains here and in Environment.step(): finite
   * commands, the active phase, speed/altitude, stall margin, q/G limits and
   * the existing AoA/bank projection still bound every learned command.  The
   * learner cannot commit a reversal, change phase, select HAC geometry, or
   * bypass an abort/fault.
   */
  Telemetry authority_state=t;
  authority_state.stall_fraction=stall;
  authority_state.stall_fraction_is_measured=isfinite(stall);
  ControlAuthorityEnvelope authority=decision_control_authority_envelope(
      &e->g,&authority_state,&e->planet,&e->cfg);
  bool hard_physical_gate =
      !valid || r.phase == PHASE_ABORT || r.phase == PHASE_FAULT ||
      !authority.valid || !authority.survivable;
  bool residual_phase = r.phase == PHASE_ENTRY_ENERGY || r.phase == PHASE_TAEM;
  double command_resolution=sqrt(DBL_EPSILON)*
      fmax(1.0,fmax(fabs(bank),fabs(bank_limit)));
  bool enabled =
      !hard_physical_gate && residual_phase && authority.controllable &&
      fabs(bank) <= bank_limit + command_resolution &&
      fabs(aoa) <= e->cfg.vehicle.maximum_angle_of_attack;
  int result_length = snprintf(
      e->result, sizeof(e->result),
      "{\"phase\":\"%s\",\"phase_id\":%d,\"phase_domain_max\":%d,\"aoa\":%.9g,\"bank\":%.9g,"
      "\"gear\":%s,\"brakes\":%s,\"abort\":%s,\"enabled\":%s,"
      "\"aoa_min\":%.9g,\"aoa_max\":%.9g,\"bank_max\":%.9g,"
      "\"q_max\":%.9g,\"g_max\":%.9g,\"stall\":%.9g,"
      "\"touchdown_speed_mps\":%.9g,\"touchdown_sink_rate_mps\":%.9g,"
      "\"authority_valid\":%s,\"authority_survivable\":%s,"
      "\"authority_controllable\":%s,\"authority_response_time_s\":%.9g,"
      "\"authority_speed_margin\":%.9g,\"authority_q_margin\":%.9g,"
      "\"authority_load_margin\":%.9g,\"authority_stall_margin\":%.9g,"
      "\"authority_speed_normalized_margin\":%.9g,"
      "\"authority_q_normalized_margin\":%.9g,"
      "\"authority_load_normalized_margin\":%.9g,"
      "\"authority_stall_normalized_margin\":%.9g,"
      "\"energy_error\":%.9g,\"course_error\":%.9g,\"range\":%.9g,"
      "\"bearing\":%.9g,\"heading_error\":%.9g,\"warning\":%s,\"course_rate\":%"
      ".9g,\"hac_radius\":%.9g,\"hac_remaining\":%.9g,\"hac_captured\":%s,"
      "\"reversal_committed\":%s,\"hac_violation\":%.9g,"
      "\"taem_capture_valid\":%s,\"taem_capture_ready\":%s,"
      "\"taem_capture_veto\":%u,\"taem_capture_along_m\":%.9g,"
      "\"taem_capture_cross_m\":%.9g,\"taem_capture_course_error_deg\":%.9g,"
     "\"taem_capture_altitude_error_m\":%.9g,"
     "\"taem_capture_energy_margin\":%.9g,\"taem_capture_turn_margin\":%.9g,"
     "\"taem_capture_path_length_m\":%.9g,\"taem_capture_required_time_s\":%.9g,\"taem_capture_available_time_s\":%.9g,\"taem_capture_minimum_turn_radius_m\":%.9g,"
     "\"taem_energy_available\":%.9g,\"taem_energy_drag_work\":%.9g,"
      "\"taem_energy_uncertainty\":%.9g}",
      phase_name(r.phase), r.phase, PHASE_ATTITUDE_RECOVERY,
      valid ? aoa : t.angle_of_attack,
      valid ? bank : t.roll, r.command.gear ? "true" : "false",
      r.command.brakes ? "true" : "false",
      (!valid || r.phase == PHASE_ABORT || r.phase == PHASE_FAULT) ? "true"
                                                                   : "false",
      enabled ? "true" : "false", floor, e->cfg.vehicle.maximum_angle_of_attack,
      bank_limit, e->cfg.vehicle.maximum_dynamic_pressure,
      e->cfg.vehicle.maximum_g_load, stall,
      e->cfg.vehicle.touchdown_speed, e->cfg.guidance.touchdown_sink_rate,
      authority.valid ? "true" : "false",
      authority.survivable ? "true" : "false",
      authority.controllable ? "true" : "false",
      authority.control_response_time_s,
      authority.speed.margin, authority.dynamic_pressure.margin,
      authority.load.margin, authority.stall.margin,
      authority.speed.normalized_margin,
      authority.dynamic_pressure.normalized_margin,
      authority.load.normalized_margin, authority.stall.normalized_margin,
      demand.projected_taem_range_error,
      t.course_to_site_error, t.range_to_site, t.bearing_to_site,
      t.heading_error, r.has_warning ? "true" : "false", t.course_rate,
      e->g.hac_radius, e->g.hac_remaining, e->g.hac_captured ? "true" : "false",
      e->g.entry_reversal_scheduled ? "true" : "false",
      e->g.hac_plan_violation_score,
      capture.valid ? "true" : "false", capture.ready ? "true" : "false",
      capture.veto, capture.along, capture.cross, capture.course_error,
      capture.altitude_error, capture.energy_margin, capture.turn_margin,
      capture.path_length_m, capture.required_time_s, capture.available_time_s,
      capture.minimum_turn_radius_m,
      capture.energy_available, capture.energy_drag_work, capture.energy_uncertainty);
  JsonWriter diagnostic;
  jw_init(&diagnostic);
  jw_raw(&diagnostic, ",\"diagnostics\":{\"status\":");
  jw_string(&diagnostic, r.status);
  jw_raw(&diagnostic, ",\"warning\":");jw_string(&diagnostic, r.warning);
#define DIAG_NUMBER(key, value) do { jw_raw(&diagnostic, ",\"" key "\":"); jw_number(&diagnostic, value); } while (0)
#define DIAG_BOOL(key, value) do { jw_raw(&diagnostic, ",\"" key "\":"); jw_bool(&diagnostic, value); } while (0)
  DIAG_BOOL("taem_owned", e->g.taem_exec.ownership_latched);
  DIAG_NUMBER("taem_phase", e->g.taem_exec.phase);
  DIAG_NUMBER("transition_reason", e->g.taem_exec.last_transition_reason);
  DIAG_BOOL("terminal_region_entered", e->g.terminal_region_entered);
  DIAG_BOOL("terminal_glide_mode", e->g.terminal_glide_mode);
  DIAG_BOOL("terminal_test_capture_active", e->g.terminal_test_capture_active);
  DIAG_BOOL("taem_interface_captured", e->g.taem_interface_captured);
  DIAG_BOOL("final_approach_captured", e->g.final_approach_captured);
  DIAG_BOOL("entry_exec_initialized", e->g.entry_exec.initialized);
  DIAG_BOOL("entry_exec_complete", e->g.entry_exec.entry_complete);
  DIAG_NUMBER("entry_exec_phase", e->g.entry_exec.phase);
  DIAG_BOOL("entry_target_side_latched", e->g.entry_target_side_latched);
  DIAG_NUMBER("entry_target_side", e->g.entry_target_side);
  DIAG_BOOL("target_valid", e->g.taem_interface_target.valid);
  DIAG_BOOL("path_committed", e->g.terminal_path_committed);
  DIAG_BOOL("candidate_valid", e->g.terminal_candidate.valid);
  DIAG_NUMBER("candidate_kind", e->g.terminal_candidate.kind);
  DIAG_BOOL("candidate_degraded", e->g.terminal_candidate.degraded);
  DIAG_BOOL("candidate_geometry_degraded", e->g.terminal_candidate.geometry_degraded);
  DIAG_BOOL("candidate_energy_degraded", e->g.terminal_candidate.energy_degraded);
  DIAG_NUMBER("candidate_radius_m", e->g.terminal_candidate.radius);
  DIAG_NUMBER("candidate_slope_deg", e->g.terminal_candidate.slope);
  DIAG_NUMBER("candidate_altitude_m", e->g.terminal_candidate.altitude);
  DIAG_NUMBER("candidate_speed_mps", e->g.terminal_candidate.speed);
  DIAG_NUMBER("candidate_course_deg", e->g.terminal_candidate.course);
  DIAG_NUMBER("candidate_response_s", e->g.terminal_candidate.response);
  DIAG_NUMBER("candidate_arrival_ut", e->g.terminal_candidate.arrival_ut);
  DIAG_NUMBER("candidate_lead_length_m", e->g.terminal_candidate.join.lead_length);
  DIAG_NUMBER("candidate_join_length_m", e->g.terminal_candidate.join.length);
  DIAG_NUMBER("candidate_arc_remaining_m", e->g.terminal_candidate.join.arc_remaining);
  DIAG_NUMBER("candidate_final_distance_m", e->g.terminal_candidate.final_distance);
  DIAG_NUMBER("candidate_violation", e->g.terminal_candidate.join.violation_score);
  DIAG_NUMBER("candidate_peak_lateral_mps2", e->g.terminal_candidate.join.peak_lateral);
  DIAG_NUMBER("candidate_peak_rate_ratio", e->g.terminal_candidate.join.peak_course_rate_ratio);
  DIAG_NUMBER("path_kind", e->g.terminal_path_kind);
  DIAG_NUMBER("reference_heading_deg", e->g.terminal_reference_heading);
  DIAG_NUMBER("reference_bank_deg", e->g.terminal_reference_bank);
  DIAG_NUMBER("reference_aoa_deg", e->g.terminal_reference_aoa);
  DIAG_NUMBER("reference_path_lateral_accel_mps2", e->g.terminal_reference_path_lateral_acceleration);
  DIAG_NUMBER("reference_path_bank_deg", e->g.terminal_reference_path_bank);
  DIAG_NUMBER("reference_path_course_error_deg", e->g.terminal_reference_path_course_error);
  DIAG_NUMBER("reference_path_arc_remaining_m", e->g.terminal_reference_path_arc_remaining);
  DIAG_BOOL("reference_path_transition_active", e->g.terminal_reference_path_transition_active);
  DIAG_NUMBER("candidate_live_energy_margin_j_kg", e->g.terminal_candidate_live_energy_margin);
  DIAG_BOOL("candidate_live_energy_valid", e->g.terminal_candidate_live_energy_valid);
  DIAG_BOOL("hac_transition_active", e->g.hac_transition_active);
  DIAG_BOOL("hac_captured", e->g.hac_captured);
  DIAG_NUMBER("hac_side", e->g.hac_side);
  DIAG_NUMBER("terminal_mix", e->g.terminal_mix);
  DIAG_NUMBER("target_along_m", e->g.taem_interface_target.along_track);
  DIAG_NUMBER("target_cross_m", e->g.taem_interface_target.cross_track);
  DIAG_NUMBER("target_course_deg", e->g.taem_interface_target.course);
  DIAG_NUMBER("target_altitude_m", e->g.taem_interface_target.altitude);
  DIAG_NUMBER("target_speed_mps", e->g.taem_interface_target.speed);
  DIAG_NUMBER("target_fpa_deg", e->g.taem_interface_target.flight_path_angle);
  DIAG_NUMBER("target_selected_ut", e->g.taem_interface_target.selected_ut);
  DIAG_NUMBER("target_arrival_ut", e->g.taem_interface_target.arrival_ut);
  DIAG_NUMBER("target_acquisition_lead_m", e->g.taem_interface_target.acquisition_lead);
  DIAG_NUMBER("target_remaining_path_m", e->g.taem_interface_target.remaining_path);
  DIAG_NUMBER("target_response_s", e->g.taem_interface_target.response_time);
  DIAG_NUMBER("capture_course_error_deg", capture.course_error);
  DIAG_BOOL("delivery_valid", e->g.taem_exec.terminal_evaluation.valid);
  DIAG_NUMBER("delivery_block", e->g.taem_exec.terminal_evaluation.block_reason);
  DIAG_NUMBER("delivery_altitude_margin_m", e->g.taem_exec.terminal_contract.altitude_margin);
  DIAG_NUMBER("delivery_energy_margin_j_kg", e->g.taem_exec.terminal_contract.specific_energy_margin);
  DIAG_BOOL("preflare_valid", e->g.terminal_preflare_plan_valid);
  DIAG_NUMBER("preflare_height_loss_m", e->g.preflare_predicted_height_loss);
  DIAG_NUMBER("vertical_stage", e->g.terminal_vertical_stage);
  DIAG_NUMBER("atmosphere_samples", e->planet.atmosphere_sample_count);
  /* Entry departure/reversal state is diagnostic evidence for the offline
     teacher.  It does not grant the adapter any additional control authority
     or change the production decision. */
  DIAG_BOOL("attitude_recovery", e->g.attitude_recovery);
  DIAG_NUMBER("control_bad_duration_s", e->g.control_bad_duration);
  DIAG_NUMBER("control_good_duration_s", e->g.control_good_duration);
  DIAG_NUMBER("recovery_duration_s", e->g.recovery_duration);
  DIAG_NUMBER("roll_oscillation_score", e->g.roll_oscillation_score);
  DIAG_NUMBER("roll_rate_excess_duration_s", e->g.roll_rate_excess_duration);
  DIAG_NUMBER("previous_relative_roll_rate_deg_s", e->g.previous_relative_roll_rate);
  DIAG_BOOL("entry_reversal_scheduled", e->g.entry_reversal_scheduled);
  DIAG_BOOL("entry_reversal_is_final", e->g.entry_reversal_is_final);
  DIAG_NUMBER("entry_reversal_ut", e->g.entry_reversal_ut);
  DIAG_NUMBER("entry_reversal_sign", e->g.entry_reversal_sign);
  DIAG_NUMBER("entry_control_reversals", e->g.entry_control_reversals);
  DIAG_NUMBER("entry_command_bank_deg", e->g.entry_control_bank);
  DIAG_NUMBER("entry_plan_bank_deg", e->g.entry_s_turn_plan.target_bank);
  DIAG_NUMBER("entry_plan_aoa_deg", e->g.entry_s_turn_plan.target_aoa);
  DIAG_NUMBER("entry_lateral_bank_magnitude_deg", e->g.entry_lateral_bank_magnitude);
  DIAG_NUMBER("entry_geometry_bank_deg", e->g.entry_geometry_bank);
  DIAG_NUMBER("entry_vertical_bank_magnitude_deg", e->g.entry_vertical_bank_magnitude);
  DIAG_NUMBER("entry_demand_bank_deg", e->g.entry_demand_bank);
  DIAG_NUMBER("entry_lateral_required_bank_deg", e->g.entry_lateral_required_bank);
#undef DIAG_NUMBER
#undef DIAG_BOOL
  jw_raw(&diagnostic, "}}");
  bool fits = result_length > 0 && (size_t)result_length < sizeof(e->result) &&
      !diagnostic.failed && (size_t)result_length + diagnostic.length < sizeof(e->result);
  if (fits) memcpy(e->result + result_length - 1, diagnostic.data, diagnostic.length + 1);
  jw_free(&diagnostic);
  guidance_result_clear(&r);
  if (!fits) return NULL;
  return e->result;
}
