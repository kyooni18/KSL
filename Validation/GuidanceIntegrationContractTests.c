#include "../CLanding/guidance.c"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static PlanetModel verifier_kerbin(void) {
    PlanetModel p;
    memset(&p, 0, sizeof(p));
    snprintf(p.name, sizeof(p.name), "Kerbin");
    p.radius = 600000.0;
    p.gravitational_parameter = 3.5316e12;
    p.rotational_speed = 2.0 * LANDER_PI / 21549.425;
    p.atmosphere_depth = 70000.0;
    p.surface_density = 1.225;
    p.atmosphere_adiabatic_index = 1.4;
    p.north_axis = v3(0, 0, 1);
    p.prime_meridian_at_epoch = v3(1, 0, 0);
    return p;
}

static Telemetry entry_telemetry(const LandingConfiguration *cfg) {
    Telemetry t;
    telemetry_init(&t);
    t.ut = 1000.0;
    t.true_air_speed = 4000.0;
    t.surface_speed = 4000.0;
    t.horizontal_speed = 3990.0;
    t.mean_altitude = 65000.0;
    t.radar_altitude = 65000.0;
    t.latitude = -1.0;
    t.longitude = cfg->site.longitude - 25.0;
    t.range_to_site = 900000.0;
    t.bearing_to_site = cfg->site.runway_heading;
    t.heading = cfg->site.runway_heading;
    t.ground_track_heading = cfg->site.runway_heading;
    t.mass = 40000.0;
    t.lift_force = 180000.0;
    t.drag_force = 20000.0;
    t.dynamic_pressure = 100.0;
    t.g_force = 0.05;
    t.mach = 12.0;
    t.angle_of_attack = 18.0;
    t.roll = 0.0;
    t.bank_effectiveness = 1.0;
    t.aerodynamic_confidence = 0.8;
    t.trajectory_density_scale = 1.0;
    t.trajectory_drag_scale = 1.0;
    t.trajectory_lift_scale = 1.0;
    t.trajectory_calibration_confidence = 0.8;
    t.speed_of_sound = 340.0;
    return t;
}

static void seed_entry_vertical_capture_fixture(GuidanceMachine *g, Telemetry *t,
        const LandingConfiguration *cfg) {
    g->taem_interface_target.valid = true;
    g->taem_interface_target.along_track = -cfg->guidance.final_approach_distance;
    g->taem_interface_target.cross_track = 0.0;
    g->taem_interface_target.altitude = cfg->guidance.taem_interface_altitude;
    g->taem_interface_target.speed = cfg->guidance.taem_force_handoff_speed;
    g->taem_interface_target.course = cfg->site.runway_heading;
    g->taem_interface_target.flight_path_angle = -fabs(cfg->guidance.taem_glide_slope);
    g->taem_interface_target.acquisition_lead = 5000.0;
    g->taem_interface_target.response_time = 3.0;
    t->runway_along_track = -fmax(cfg->guidance.final_approach_distance, t->range_to_site);
    t->runway_cross_track = 0.0;
    t->attitude_response.pitch_valid = true;
    t->attitude_response.maximum_pitch_rate_deg_s = 8.0;
    t->attitude_response.maximum_pitch_accel_deg_s2 = 5.0;
    t->attitude_response.roll_valid = true;
    t->attitude_response.maximum_roll_rate_deg_s = 18.0;
    t->attitude_response.maximum_roll_accel_deg_s2 = 15.0;
}

static void test_reentry_continuation_initializer_is_shared_policy_seed(void) {
    LandingConfiguration cfg = landing_configuration_default();
    PlanetModel p = verifier_kerbin();
    AerodynamicEnvelope env; memset(&env, 0, sizeof(env));
    TrajectoryCalibrationModel cal; memset(&cal, 0, sizeof(cal));
    Telemetry t = entry_telemetry(&cfg);
    double entry_alt = entry_guidance_start_altitude(&p, &cfg.guidance);

    t.mean_altitude = entry_alt + 5000.0;
    t.vertical_speed = -25.0;
    GuidanceMachine coast;
    guidance_initialize_reentry_continuation(&coast, &t, &p, &cfg, -1.0, false, &env, &cal);
    assert(coast.automation_engaged);
    assert(coast.has_burn_command_started && coast.deorbit_burn_completed);
    assert(!coast.atmospheric_interface_crossed);
    assert(coast.phase == PHASE_ENTRY_INTERFACE);
    assert(coast.s_turn_sign < 0.0);
    assert(coast.entry_predictor_models_valid);

    t.mean_altitude = entry_alt - 1000.0;
    GuidanceMachine loaded;
    guidance_initialize_reentry_continuation(&loaded, &t, &p, &cfg, 1.0, false, &env, &cal);
    assert(loaded.atmospheric_interface_crossed);
    assert(loaded.phase == PHASE_ENTRY_ENERGY);
    assert(loaded.s_turn_sign > 0.0);

    GuidanceMachine late;
    guidance_initialize_reentry_continuation(&late, &t, &p, &cfg, -1.0, true, &env, &cal);
    assert(late.atmospheric_interface_crossed);
    assert(late.phase == PHASE_TAEM);
    assert(late.s_turn_sign < 0.0);
}
static void test_preentry_capture_uses_inertial_native_capture_until_airload(void) {
    LandingConfiguration cfg = landing_configuration_default();
    Telemetry t = entry_telemetry(&cfg);
    VehicleState state = {.ut = t.ut, .position = v3(600000.0, 0.0, 0.0), .velocity = v3(0.0, 2000.0, 0.0), .mass = t.mass};
    t.flight_path_angle = -2.0;
    t.ground_track_heading = 90.0;

    t.dynamic_pressure = 0.0;
    GuidanceCommand vacuum = entry_capture(&t, &state, &cfg.vehicle);
    assert(vacuum.autopilot_engaged);
    assert(vacuum.use_inertial_direction);
    assert(vacuum.control_profile == PROFILE_ENTRY);
    assert(vdot(vacuum.inertial_direction, state.velocity) > 0.0);
    assert(vacuum.navball_speed_mode == SPEED_ORBIT);

    t.dynamic_pressure = 0.49;
    assert(entry_capture(&t, &state, &cfg.vehicle).use_inertial_direction);

    t.dynamic_pressure = 0.5;
    GuidanceCommand aero = entry_capture(&t, &state, &cfg.vehicle);
    assert(!aero.use_inertial_direction);
    assert(aero.control_profile == PROFILE_ENTRY);
    assert(aero.navball_speed_mode == SPEED_SURFACE);
    assert(fabs(aero.target_heading - 90.0) < 1e-9);
    assert(fabs(aero.target_aoa - entry_low_q_protective_aoa_floor(t.dynamic_pressure, &cfg.vehicle)) < 1e-9);

    t.dynamic_pressure = 525.0;
    aero = entry_capture(&t, &state, &cfg.vehicle);
    assert(fabs(aero.target_aoa - entry_low_q_protective_aoa_floor(t.dynamic_pressure, &cfg.vehicle)) < 1e-9);
    assert(aero.target_aoa < cfg.vehicle.maximum_angle_of_attack);
    assert(aero.target_aoa >= cfg.vehicle.entry_angle_of_attack);

    t.dynamic_pressure = 700.0;
    aero = entry_capture(&t, &state, &cfg.vehicle);
    assert(fabs(aero.target_aoa - cfg.vehicle.entry_angle_of_attack) < 1e-9);
}

static void test_beta_runaway_gate_allows_corrective_yaw_braking_transient(void) {
    LandingConfiguration cfg = landing_configuration_default();
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.phase = PHASE_ENTRY_ENERGY;
    g.has_previous_sideslip = true;
    g.previous_sideslip = -10.75;

    Telemetry t = entry_telemetry(&cfg);
    t.ut = 67273.4566012118;
    t.dynamic_pressure = 422.16;
    t.lift_force = 12306.14;
    t.drag_force = 32426.54;
    t.mass = 40231.64;
    t.roll = 5.80;
    t.pitch_rate = 0.83;
    t.roll_rate = 1.29;
    t.heading_rate = 3.46;
    t.sideslip = -10.93;
    t.has_controls = true;
    t.control_pitch = 0.15;
    t.control_roll = -0.18;
    t.control_yaw = -0.79;
    t.has_body_pitch_rate = true;
    t.has_body_roll_rate = true;
    t.has_body_yaw_rate = true;
    t.body_pitch_rate = 0.88;
    t.body_roll_rate = 0.27;
    t.body_yaw_rate = 3.19;

    /* beta is still moving farther negative, but negative rudder is opposing
       the remaining positive yaw rate and has the correct sign to recover a
       negative beta. This is braking, not a departure. */
    assert(!control_recovery_needed(&g, &t, &cfg.guidance, &cfg.vehicle, .06));
    assert(!g.attitude_recovery);

    /* finalaoa11 carried +25..31 deg beta with +0.8 rudder while the body still
       had about -10 deg/s of yaw inertia. The rudder had the correct beta sign
       and opposed the measured yaw rate; the old generic saturated-yaw >10 dps
       gate ignored that braking state and entered recovery after ~0.35 s. */
    guidance_machine_init(&g);
    g.phase = PHASE_ENTRY_ENERGY;
    g.has_previous_sideslip = true;
    g.previous_sideslip = 24.5;
    t.dynamic_pressure = 8400.0;
    t.lift_force = t.mass * 12.0;
    t.drag_force = t.mass * 20.0;
    t.roll = -71.0;
    t.roll_rate = 0.8;
    t.body_roll_rate = 0.8;
    t.pitch_rate = 1.1;
    t.body_pitch_rate = 1.1;
    t.heading_rate = -5.0;
    t.sideslip = 25.0;
    t.body_yaw_rate = -10.2;
    t.control_pitch = 0.2;
    t.control_roll = -0.03;
    t.control_yaw = 0.8;
    for (int i = 0; i < 8; ++i) {
        t.sideslip += 0.75;
        assert(!control_recovery_needed(&g, &t, &cfg.guidance, &cfg.vehicle, .06));
    }
    assert(!g.attitude_recovery);

    /* If the exact same growing-beta state is not being yaw-rate-braked, the
       existing runaway protection must still accumulate and trip. */
    guidance_machine_init(&g);
    g.phase = PHASE_ENTRY_ENERGY;
    g.has_previous_sideslip = true;
    g.previous_sideslip = -10.75;
    t.body_yaw_rate = 10.2;
    t.heading_rate = 5.0;
    t.control_yaw = 0.79;
    for (int i = 0; i < 8 && !g.attitude_recovery; ++i) {
        t.sideslip -= 0.18;
        (void)control_recovery_needed(&g, &t, &cfg.guidance, &cfg.vehicle, .06);
    }
    assert(g.attitude_recovery);
}

static void test_normal_mm304_initialization_honors_preentry_load_gate(void) {
    LandingConfiguration cfg = landing_configuration_default();
    PlanetModel p = verifier_kerbin();
    AerodynamicModel aero = {
        .lift_to_drag = 0.6,
        .ballistic_coefficient = 700.0,
        .confidence = 0.8,
    };
    GuidanceMachine g;
    guidance_machine_init(&g);
    guidance_set_engaged(&g, true);
    Telemetry t = entry_telemetry(&cfg);

    double gravity = planet_surface_gravity(&p);
    assert(t.g_force * gravity < 1.5);

    GuidanceResult r = entry_program_guidance(&g, &t, NULL,
        cfg.site.runway_heading, &p, aero, &cfg, 0.1);

    assert(g.entry_exec.initialized);
    assert(g.entry_exec.phase == ENTRY_PHASE_PREENTRY);
    assert(!g.entry_exec.entry_complete);
    assert(strstr(r.status, "MM304 Pre-entry") != NULL);

    double expected_low_q_floor=fmax(cfg.vehicle.entry_angle_of_attack,
        cfg.vehicle.maximum_angle_of_attack*.72);
    assert(g.entry_control_aoa>=expected_low_q_floor-1e-9);
    guidance_result_clear(&r);
}


static void test_mm304_high_for_range_restores_vertical_capture_before_taem(void) {
    LandingConfiguration cfg = landing_configuration_default();
    cfg.guidance.taem_force_handoff_speed = 1300.0;
    cfg.guidance.taem_interface_altitude = 16500.0;
    cfg.guidance.taem_interface_range = 35000.0;
    cfg.guidance.target_entry_range = 1030000.0;
    cfg.guidance.taem_glide_slope = 12.0;
    cfg.vehicle.entry_angle_of_attack = 18.0;
    cfg.vehicle.maximum_angle_of_attack = 28.0;
    cfg.vehicle.maximum_dynamic_pressure = 45000.0;
    cfg.vehicle.maximum_g_load = 3.5;
    cfg.vehicle.maximum_bank_angle = 70.0;
    cfg.vehicle.minimum_safe_speed = 85.0;
    PlanetModel p = verifier_kerbin();
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.entry_exec.initialized = true;
    g.entry_exec.phase = ENTRY_PHASE_CONSTANT_DRAG;
    g.s_turn_sign = 1.0;

    Telemetry t = entry_telemetry(&cfg);
    /* Frozen from the 10:57 two-fix flight while the global predictor already
       saw a recoverable TAEM solution but the local MM304 allocator still held
       almost wings-level: enough q/lift and stall margin existed to spend path. */
    t.true_air_speed = 1796.3;
    t.mean_altitude = 35773.0;
    t.range_to_site = 92718.0;
    t.flight_path_angle = -3.11;
    t.dynamic_pressure = 3000.0;
    t.g_force = 0.60;
    t.stall_fraction = 0.046;
    t.angle_of_attack = 21.3;

    AerodynamicModel aero = {.lift_to_drag = .4, .ballistic_coefficient = 700.0, .confidence = .6};
    seed_entry_vertical_capture_fixture(&g, &t, &cfg);
    EntryControlPlan plan = {.valid = true, .target_bank = 0.0, .target_aoa = 23.5};
    assert(entry_program_shape_vertical_capture_plan(&g, &t, &p, aero, &cfg, &plan));
    assert(fabs(plan.target_bank) > 15.0);
    assert(plan.target_aoa >= entry_thermal_protection_aoa_floor(&cfg.vehicle) - 1e-9);
    assert(isfinite(plan.target_turn_radius));

}


static void test_mm304_drag_reference_reserves_terminal_geometry_not_raw_interface(void) {
    LandingConfiguration cfg = landing_configuration_default();
    PlanetModel p = verifier_kerbin();
    AerodynamicModel aero = {.lift_to_drag = .4, .ballistic_coefficient = 700.0, .confidence = 0.0};
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.entry_exec.initialized = true;
    g.entry_exec.phase = ENTRY_PHASE_CONSTANT_DRAG;
    g.s_turn_sign = 1.0;

    Telemetry t = entry_telemetry(&cfg);
    t.true_air_speed = 1800.0;
    t.surface_speed = 1800.0;
    t.horizontal_speed = 1798.0;
    t.mean_altitude = 30000.0;
    t.range_to_site = 160000.0;
    t.flight_path_angle = -5.0;
    t.vertical_speed = -157.0;
    t.dynamic_pressure = 6000.0;
    t.g_force = .70;
    t.angle_of_attack = 18.0;
    t.aerodynamic_confidence = 0.0;
    seed_entry_vertical_capture_fixture(&g, &t, &cfg);

    GuidanceResult r = entry_program_guidance(&g, &t, NULL,
        cfg.site.runway_heading, &p, aero, &cfg, .1);
    const char *dref_text = strstr(r.status, "Dref ");
    double reported_dref = NAN;
    assert(dref_text && sscanf(dref_text, "Dref %lf", &reported_dref) == 1);

    const TaemInterfaceTarget *target = &g.taem_interface_target;
    assert(target->valid);
    assert(isfinite(t.runway_along_track) && isfinite(t.runway_cross_track));
    double da = target->along_track - t.runway_along_track;
    double dc = target->cross_track - t.runway_cross_track;
    double dynamic_target_range = hypot(da, dc);
    assert(dynamic_target_range > 1000.0);

    double rh = cfg.site.runway_heading * DEG2RAD;
    double east = target->along_track * sin(rh) + target->cross_track * cos(rh);
    double north = target->along_track * cos(rh) - target->cross_track * sin(rh);
    GeoPoint origin = {cfg.site.latitude, cfg.site.longitude, cfg.site.altitude};
    GeoPoint gate = local_point(origin, east, north, p.radius, target->altitude);
    assert(isfinite(gate.latitude));

    EntryDragReferenceConfig drag_cfg = entry_drag_reference_default_config();
    drag_cfg.entry_velocity_ratio = g.entry_drag_velocity_ratio;
    double measured_drag = t.mass > 1.0 && t.drag_force > 0.0 ? t.drag_force / t.mass : NAN;
    double modeled_drag = live_drag_accel(&t, aero, &cfg.vehicle);
    EntryDragReferenceInput expected_input = {
        .phase = g.entry_exec.phase, .relative_velocity = t.true_air_speed,
        .latitude = t.latitude, .altitude = t.mean_altitude,
        .measured_drag_accel = measured_drag, .modeled_drag_accel = modeled_drag,
        .aero_confidence = 0.0, .range_to_site = dynamic_target_range,
        .taem_range = 0.0, .taem_latitude = gate.latitude,
        .taem_altitude = target->altitude, .taem_velocity = target->speed,
        .planet = &p, .vehicle = &cfg.vehicle
    };
    EntryDragReferenceOutput expected = entry_drag_reference_compute(&expected_input, &drag_cfg);
    assert(expected.valid);
    assert(fabs(reported_dref - expected.reference_drag_accel) <= .011);

    guidance_result_clear(&r);
}

static void test_mm304_unmeasured_stall_proxy_does_not_unload_thermal_alpha_at_recorded_1739_state(void) {
    LandingConfiguration cfg = landing_configuration_default();
    PlanetModel p = verifier_kerbin();
    AerodynamicModel aero = {.lift_to_drag = .4, .ballistic_coefficient = 700.0, .confidence = .8};
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.phase = PHASE_ENTRY_ENERGY;
    g.automation_engaged = true;
    g.entry_exec.initialized = true;
    g.entry_exec.phase = ENTRY_PHASE_CONSTANT_DRAG;
    g.s_turn_sign = 1.0;

    /* Exact 17:39 diagnostic state near UT 67511.77. The vehicle was ~13 km
       above its range-indexed TAEM corridor while holding ~26 deg AoA. The
       AoA-derived fallback stall proxy had risen to 0.69 at 1531 m/s, but later
       live evidence proved that proxy is not independent stall evidence. It must
       not authorize a below-thermal-floor unload; measured stall or airspeed
       margin evidence still owns that emergency escape. */
    Telemetry t = entry_telemetry(&cfg);
    t.ut = 67511.7683298516;
    t.latitude = -0.588406631833426;
    t.longitude = -86.4257773870075;
    t.mean_altitude = 32268.1937940629;
    t.radar_altitude = 31922.0092924295;
    t.true_air_speed = 1530.95178222656;
    t.horizontal_speed = 1527.49587686024;
    t.vertical_speed = -102.809564684111;
    t.flight_path_angle = -3.85053958057131;
    t.ground_track_heading = 103.58935546875;
    t.heading = t.ground_track_heading;
    t.bearing_to_site = 87.3996440584944;
    t.range_to_site = 122623.354732596;
    t.runway_along_track = -122495.303595525;
    t.runway_cross_track = 5652.72547458935;
    t.angle_of_attack = 25.8538513183594;
    t.roll = 23.6016845703125;
    t.has_body_roll_rate = true;
    t.body_roll_rate = -0.123892043911254;
    t.has_body_pitch_rate = true;
    t.body_pitch_rate = -0.00459218387017195;
    t.dynamic_pressure = 4389.90478515625;
    t.g_force = 0.996807396411896;
    t.stall_fraction = 0.693407331194196;
    t.mass = 40215.3359375;
    t.lift_force = 175426.549417494;
    t.drag_force = 352934.503470184;
    t.aerodynamic_confidence = .8;
    t.trajectory_density_scale = 1.0;
    t.trajectory_drag_scale = 1.0;
    t.trajectory_lift_scale = 1.0;
    t.bank_effectiveness = 1.0;
    t.trajectory_calibration_confidence = .8;

    VehicleState state = {
        .ut = t.ut,
        .position = {394342.617666289, 494180.947731261, -6493.04821625515},
        .velocity = {-1400.33730039011, 984.620569342239, -96.8316805639286},
        .mass = t.mass
    };

    /* Seed the exact stale segment shape seen in the diagnostic run. It is still
       temporally valid and supervision is marked fresh, so conservative proxy
       recovery may override execution without replacing the persistent topology. */
    g.entry_s_turn_plan.valid = true;
    g.entry_s_turn_plan.plan_id = 41;
    g.entry_s_turn_plan.plan_version = 1;
    g.entry_s_turn_plan.planned_ut = t.ut;
    g.entry_s_turn_plan.segment_duration = 75.0;
    g.entry_s_turn_plan.target_bank = 46.0;
    g.entry_s_turn_plan.target_aoa = 26.0;
    g.entry_s_turn_plan.target_heading = t.ground_track_heading;
    g.entry_control_plan_valid = true;
    g.entry_supervision_valid = true;
    g.entry_supervision_mode = ENTRY_SUPERVISION_PASS_THROUGH;
    g.entry_supervision_ut = t.ut;
    GuidanceResult r = entry_program_guidance(&g, &t, &state,
        t.ground_track_heading, &p, aero, &cfg, .1);
    double thermal_floor = entry_thermal_protection_aoa_floor(&cfg.vehicle);
    assert(!t.stall_fraction_is_measured&&t.stall_fraction>.12);
    assert(g.entry_s_turn_plan.valid);
    assert(g.entry_s_turn_plan.target_aoa >= thermal_floor - 1e-9);
    assert(g.entry_control_aoa >= thermal_floor - 1e-9);
    /* The synthetic C-Nano/AoA-derived stall proxy is useful telemetry, but is
       not independent evidence that may unload execution or rewrite S-turn
       topology. The live command may slew from measured incidence toward the
       nominal plan, but must remain on the thermal side of the floor. */
    assert(fabs(r.command.target_roll) <= 24.0 + 1e-9);
    assert(r.command.has_target_aoa);
    assert(r.command.target_aoa >= thermal_floor - 1e-9);
    if(r.has_warning)
        assert(strstr(r.warning, "thermal AoA floor temporarily overridden") == NULL);

    /* An unmeasured proxy excursion must neither trigger an emergency unload nor
       churn the persistent plan every 100 ms. Hold supervision fresh so this
       regression isolates the proxy semantics from predictor refresh. */
    unsigned long long stable_plan_id = g.entry_s_turn_plan.plan_id;
    unsigned long long stable_plan_version = g.entry_s_turn_plan.plan_version;
    double stable_plan_ut = g.entry_s_turn_plan.planned_ut;
    assert(stable_plan_id > 0);
    guidance_result_clear(&r);
    for(int i=0;i<20;i++){
        t.ut += .1;
        state.ut = t.ut;
        g.entry_supervision_valid = true;
        g.entry_supervision_mode = ENTRY_SUPERVISION_PASS_THROUGH;
        g.entry_supervision_boundary_missed = false;
        g.entry_supervision_ut = t.ut;
        g.entry_supervision_bank_correction = 0.0;
        g.entry_supervision_aoa_correction = 0.0;
        r = entry_program_guidance(&g, &t, &state,
            t.ground_track_heading, &p, aero, &cfg, .1);
        assert(g.entry_s_turn_plan.plan_id == stable_plan_id);
        assert(g.entry_s_turn_plan.plan_version == stable_plan_version);
        assert(fabs(g.entry_s_turn_plan.planned_ut - stable_plan_ut) < 1e-9);
        assert(g.entry_control_aoa >= thermal_floor - 1e-9);
        assert(g.entry_s_turn_plan.target_aoa >= thermal_floor - 1e-9);
        assert(r.command.target_aoa >= thermal_floor - 1e-9);
        guidance_result_clear(&r);
    }
}


static __attribute__((unused)) void test_mm304_stabilizer_reacquires_thermal_floor_after_stall_clears(void) {
    LandingConfiguration cfg = landing_configuration_default();
    GuidanceMachine g;
    guidance_machine_init(&g);
    Telemetry t = entry_telemetry(&cfg);
    t.flight_path_angle = -1.70;
    t.dynamic_pressure = 68.7;
    t.true_air_speed = 2119.0;
    t.mach = 7.14;
    t.angle_of_attack = 21.5;
    t.roll = 0.0;
    t.heading = 90.0;
    t.throttle = 0.0;

    double thermal_floor = entry_thermal_protection_aoa_floor(&cfg.vehicle);
    double low_q_target = entry_low_q_protective_aoa_floor(t.dynamic_pressure, &cfg.vehicle);
    assert(fabs(thermal_floor - 18.0) < 1e-9);

    g.has_last_stabilized_phase = true;
    g.last_stabilized_phase = PHASE_ENTRY_ENERGY;
    g.pitch_limiter.has_value = true;
    g.pitch_limiter.value = 11.26;
    g.pitch_limiter.rate = -3.0; /* live failure class: stale emergency recovery momentum */
    g.roll_limiter.has_value = true;
    g.roll_limiter.value = t.roll;
    g.heading_limiter.has_value = true;
    g.heading_limiter.value = t.heading;
    g.throttle_limiter.has_value = true;
    g.throttle_limiter.value = t.throttle;

    GuidanceCommand c = atmospheric(&t, t.heading, 0.0, &cfg.vehicle, 0.0, false, PROFILE_ENTRY);
    c.has_target_aoa = true;
    c.target_aoa = low_q_target;
    c.target_pitch = t.flight_path_angle + low_q_target;
    GuidanceResult r;
    memset(&r, 0, sizeof(r));
    r.phase = PHASE_ENTRY_ENERGY;
    r.command = c;
    r = stabilized(&g, r, &t, &cfg.vehicle, &cfg.guidance, 0.1);
    assert(r.command.target_aoa >= low_q_target - 1e-9);
    assert(fabs(g.pitch_limiter.value - r.command.target_aoa) < 1e-9);
    assert(g.pitch_limiter.rate >= -1e-9);
    double first_reacquired = r.command.target_aoa;

    /* With the q-aware fallback proxy, the nominal low-q envelope is no longer
       a stall trigger. Subsequent nominal frames stay on the same protective
       reference rather than passing through a stale hard-18 intermediate target. */
    memset(&r, 0, sizeof(r));
    r.phase = PHASE_ENTRY_ENERGY;
    r.command = c;
    r = stabilized(&g, r, &t, &cfg.vehicle, &cfg.guidance, 0.1);
    assert(r.command.target_aoa >= first_reacquired - 1e-9);
    assert(r.command.target_aoa >= low_q_target - 1e-9);

    /* A genuinely active emergency target below the floor still owns execution. */
    g.pitch_limiter.value = 11.26;
    g.pitch_limiter.rate = -1.0;
    c.target_aoa = 11.26;
    c.target_pitch = t.flight_path_angle + c.target_aoa;
    memset(&r, 0, sizeof(r));
    r.phase = PHASE_ENTRY_ENERGY;
    r.command = c;
    r = stabilized(&g, r, &t, &cfg.vehicle, &cfg.guidance, 0.1);
    assert(r.command.target_aoa < thermal_floor - 0.1);
}

static void test_mm304_reversal_is_plan_milestone_not_crossrange_chase(void) {
    LandingConfiguration cfg = landing_configuration_default();
    cfg.guidance.s_turn_minimum_leg_duration = 24.0;
    PlanetModel p = verifier_kerbin();
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.phase = PHASE_ENTRY_ENERGY;
    g.entry_continuation_bootstrap = true;
    g.s_turn_sign = 1.0;
    g.has_s_turn_leg_started = true;


    g.s_turn_leg_started_ut = 90.0;
    g.entry_control_plan_valid = true;
    g.entry_control_bank = 45.0;
    g.entry_s_turn_plan.valid = true;
    g.entry_s_turn_plan.planned_ut = 90.0;
    g.entry_s_turn_plan.segment_duration = 80.0;
    g.entry_reversal_scheduled = true;
    g.entry_reversal_ut = 120.0;
    g.entry_reversal_range = 50000.0;
    g.entry_reversal_sign = -1.0;
    g.entry_reversal_bank = 45.0;

    Telemetry t = entry_telemetry(&cfg);
    t.ut = 115.0;
    t.range_to_site = 60000.0;
    /* Reproduce the v34 50 km continuation context. Bootstrap mode may lack
       serialized actuator history, but it must not reinterpret the first genuine
       non-final committed S-turn reversal as a final-turn endpoint event. */
    t.runway_along_track = -300000.0;
    /* Deliberately absurd lateral errors: they are diagnostics/replan evidence only
       and must not directly execute a bank reversal before the planned milestone. */
    t.runway_cross_track = 50000.0;
    t.course_to_site_error = -75.0;
    assert(!entry_program_planned_reversal_due(&g, &t, &p, &cfg.vehicle, &cfg.guidance));
    assert(!entry_program_execute_planned_reversal(&g, &t, &p, &cfg));
    assert(g.s_turn_sign > 0.0);
    assert(g.entry_reversal_scheduled);

    t.ut = 120.0;
    g.s_turn_leg_started_ut = 90.0;

    /* An outstanding roll-capture latch still blocks a second reversal. */
    g.has_s_turn_reversal_requested = true;
    assert(!entry_program_planned_reversal_due(&g, &t, &p, &cfg.vehicle, &cfg.guidance));
    g.has_s_turn_reversal_requested = false;
    assert(entry_program_planned_reversal_due(&g, &t, &p, &cfg.vehicle, &cfg.guidance));
    assert(entry_program_execute_planned_reversal(&g, &t, &p, &cfg));
    assert(g.s_turn_sign < 0.0);
    assert(!g.entry_reversal_scheduled);
    assert(!g.entry_s_turn_plan.valid);
    assert(g.entry_control_reversals == 1u);
}

static void test_mm304_fallback_admits_due_nonfinal_deadline_reversal(void) {
    LandingConfiguration cfg = landing_configuration_default();
    cfg.guidance.s_turn_minimum_leg_duration = 24.0;
    PlanetModel p = verifier_kerbin();
    AerodynamicModel aero = {.lift_to_drag = 0.55, .ballistic_coefficient = 750.0, .confidence = 0.8};
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.phase = PHASE_ENTRY_ENERGY;
    g.entry_exec.initialized = true;
    g.entry_exec.phase = ENTRY_PHASE_TRANSITION;
    g.s_turn_sign = 1.0;
    g.has_s_turn_leg_started = true;
    g.s_turn_leg_started_ut = 100.0;
    g.entry_control_plan_valid = true;
    g.entry_control_plan_ut = 100.0;
    g.entry_control_segment_until_ut = 300.0;
    g.entry_control_bank = 40.0;
    g.entry_control_aoa = 18.0;
    g.entry_control_heading = cfg.site.runway_heading;
    g.entry_reversal_scheduled = true;
    g.entry_reversal_ut = 130.0;
    g.entry_reversal_range = 40000.0;
    g.entry_reversal_sign = -1.0;
    g.entry_reversal_bank = 40.0;

    Telemetry t = entry_telemetry(&cfg);
    t.ut = 130.0;
    t.true_air_speed = 1800.0;
    t.surface_speed = 1800.0;
    t.horizontal_speed = 1780.0;
    t.mean_altitude = 40000.0;
    t.radar_altitude = 39930.0;
    t.range_to_site = 40000.0;
    t.course_to_site_error = 0.0;
    t.flight_path_angle = -5.0;
    t.vertical_speed = -155.0;
    t.dynamic_pressure = 5000.0;
    t.g_force = 1.3;
    t.roll = 38.0;

    /* A final-exit event remains gated while high/off-corridor. */
    g.entry_reversal_is_final = true;
    GuidanceResult r = entry_guidance(&g, &t, t.ground_track_heading, &p, aero, &cfg, 0.1);
    assert(g.s_turn_sign > 0.0);
    assert(g.entry_reversal_scheduled);
    guidance_result_clear(&r);

    /* The same due event, when explicitly non-final, is the deadline maneuver
       itself and must execute instead of waiting until runway range is consumed. */
    g.entry_reversal_is_final = false;
    r = entry_guidance(&g, &t, t.ground_track_heading, &p, aero, &cfg, 0.1);
    assert(g.s_turn_sign < 0.0);
    assert(!g.entry_reversal_scheduled);
    assert(g.entry_control_bank < 0.0);
    assert(g.has_s_turn_reversal_requested);
    guidance_result_clear(&r);

    /* Even if stale bookkeeping tries to arm another due event before the new
       bank is physically captured, the reversal-request latch prevents a backflip. */
    g.has_s_turn_leg_started = true;
    g.s_turn_leg_started_ut = 100.0;
    g.entry_reversal_scheduled = true;
    g.entry_reversal_ut = t.ut;
    g.entry_reversal_sign = 1.0;
    g.entry_reversal_bank = 40.0;
    r = entry_guidance(&g, &t, t.ground_track_heading, &p, aero, &cfg, 0.1);
    assert(g.s_turn_sign < 0.0);
    assert(g.entry_reversal_scheduled);
    assert(g.has_s_turn_reversal_requested);
    guidance_result_clear(&r);
}
static void test_mm304_planned_range_is_diagnostic_not_direct_reversal_trigger(void) {
    LandingConfiguration cfg = landing_configuration_default();
    cfg.guidance.s_turn_minimum_leg_duration = 24.0;
    PlanetModel p = verifier_kerbin();
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.phase = PHASE_ENTRY_ENERGY;
    g.s_turn_sign = 1.0;
    g.has_s_turn_leg_started = true;
    g.s_turn_leg_started_ut = 90.0;
    g.entry_control_plan_valid = true;
    g.entry_control_bank = 40.0;
    g.entry_s_turn_plan.valid = true;
    g.entry_s_turn_plan.planned_ut = 90.0;
    g.entry_s_turn_plan.segment_duration = 80.0;
    g.entry_reversal_scheduled = true;
    g.entry_reversal_ut = 140.0;
    /* A propagated path may cross the site and later have a much larger spherical
       range. Such a range remains useful evidence, but cannot be an execution edge. */
    g.entry_reversal_range = 948724.0;
    g.entry_reversal_sign = -1.0;
    g.entry_reversal_bank = 40.0;

    Telemetry t = entry_telemetry(&cfg);
    t.ut = 116.0;
    t.range_to_site = 49999.0;
    assert(!entry_program_planned_reversal_due(&g, &t, &p, &cfg.vehicle, &cfg.guidance));
    assert(!entry_program_execute_planned_reversal(&g, &t, &p, &cfg));
    assert(g.s_turn_sign > 0.0);
}


static void test_controlled_coordinate_rate_helpers_reject_body_axis_aliases(void) {
    Telemetry t;
    telemetry_init(&t);
    /* Reproduce the 17:57 discriminator: controlled-coordinate derivatives can
       oppose body p/q in high-AoA coupled motion. The helpers must follow the
       coordinates they actually control, never the body axes. */
    t.roll_rate = 3.25;
    t.has_body_roll_rate = true;
    t.body_roll_rate = -18.0;
    assert(fabs(controlled_roll_rate(&t) - 3.25) < 1e-12);

    t.pitch_rate = 9.0;
    t.has_body_pitch_rate = true;
    t.body_pitch_rate = 17.0;
    t.has_angle_of_attack_rate = true;
    t.angle_of_attack_rate = -4.5;
    assert(fabs(controlled_aoa_rate(&t) + 4.5) < 1e-12);

    /* Cold-start/missing derivatives are neutral, not silently replaced by a
       different coordinate that may have the opposite physical meaning. */
    t.roll_rate = NAN;
    assert(fabs(controlled_roll_rate(&t)) < 1e-12);
    t.has_angle_of_attack_rate = false;
    t.angle_of_attack_rate = NAN;
    assert(fabs(controlled_aoa_rate(&t)) < 1e-12);
}

static void test_mm304_first_program_side_follows_taem_tangent_gate(void) {
    LandingConfiguration cfg = landing_configuration_default();
    cfg.site.runway_heading = 90.0;
    Telemetry t;
    telemetry_init(&t);
    t.ground_track_heading = 90.0;
    t.heading = 90.0;
    t.runway_along_track = -400000.0;
    t.runway_cross_track = 0.0;

    GuidanceMachine g;
    guidance_machine_init(&g);
    assert(entry_program_first_segment(&g));
    g.s_turn_sign = -1.0;
    g.has_s_turn_reversal_requested = true;
    g.taem_interface_target = (TaemInterfaceTarget){
        .valid = true, .along_track = 0.0, .cross_track = 72000.0, .course = 90.0
    };
    entry_program_seed_initial_side(&g, &t, &cfg);
    assert(g.s_turn_sign > 0.0);
    assert(!g.has_s_turn_reversal_requested);

    /* Mirror the tangent gate across the runway-local trajectory.  The initial
       S-turn must mirror with it rather than retaining a KSC-specific handedness. */
    GuidanceMachine mirrored;
    guidance_machine_init(&mirrored);
    mirrored.taem_interface_target = g.taem_interface_target;
    mirrored.taem_interface_target.cross_track = -72000.0;
    entry_program_seed_initial_side(&mirrored, &t, &cfg);
    assert(mirrored.s_turn_sign < 0.0);

    /* Once a real program exists, reseeding is forbidden: later side is plan-owned. */
    g.entry_s_turn_plan.valid = true;
    g.s_turn_sign = -1.0;
    entry_program_seed_initial_side(&g, &t, &cfg);
    assert(g.s_turn_sign < 0.0);
    /* A late/restarted MM304 state with no synthetic plan object is not a first
       segment and must preserve its already-flown side. */
    g.entry_s_turn_plan.valid = false;
    g.entry_exec.initialized = true;
    g.entry_exec.phase = ENTRY_PHASE_TRANSITION;
    g.s_turn_sign = -1.0;
    assert(!entry_program_first_segment(&g));
    entry_program_seed_initial_side(&g, &t, &cfg);
    assert(g.s_turn_sign < 0.0);
}

static void test_mm304_leg_dwell_requires_continuous_loaded_bank_capture(void) {
    LandingConfiguration cfg = landing_configuration_default();
    cfg.guidance.s_turn_minimum_leg_duration = 24.0;
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.phase = PHASE_ENTRY_ENERGY;
    g.s_turn_sign = -1.0;
    g.entry_exec.initialized = true;
    g.entry_exec.phase = ENTRY_PHASE_PREENTRY;

    Telemetry t = entry_telemetry(&cfg);
    /* Frozen 14:39 regression: a tiny -2.9 deg near-vacuum target overshot to
       -4.09 deg at q=5.2 Pa and used to start a leg timer 300+ s too early. */
    t.ut = 67009.5680953744;
    t.dynamic_pressure = 5.20254278182983;
    t.g_force = 0.00281273480504751;
    t.roll = -4.08584594726562;
    entry_update_s_turn_leg_capture(&g, &t, -2.9, &cfg.vehicle);
    assert(!g.has_s_turn_leg_started);
    assert(g.s_turn_leg_started_ut == 0.0);
    assert(g.s_turn_sign < 0.0);

    /* A tiny opposite-side pre-entry target is not a plan-owned side choice.
       It may overshoot in near vacuum but cannot leak into the first MM304 plan. */
    g.s_turn_sign = 1.0;
    g.has_s_turn_reversal_requested = false;
    t.ut = 101.0;
    t.roll = -4.1;
    entry_update_s_turn_leg_capture(&g, &t, -2.9, &cfg.vehicle);
    assert(g.s_turn_sign > 0.0);
    assert(!g.has_s_turn_reversal_requested);

    /* Recorded 04-50 positive fixture: PREENTRY may already be doing authoritative
       bank/path work before the 1.5 m/s2 executive load transition. The same useful-aero
       gate that admits that work must let a meaningful same-side measured capture start
       the physical S-turn dwell clock. */
    g.s_turn_sign = 1.0;
    g.has_s_turn_reversal_requested = false;
    t.ut = 67407.4739548304;
    t.true_air_speed = 2033.062;
    t.dynamic_pressure = 950.7496;
    t.g_force = 0.07736;
    t.stall_fraction = 0.0;
    t.roll = 21.1758;
    assert(entry_s_turn_bank_authority_available(t.dynamic_pressure, t.true_air_speed,
        t.stall_fraction, t.g_force, &cfg.vehicle));
    entry_update_s_turn_leg_capture(&g, &t, 23.5797, &cfg.vehicle);
    assert(g.entry_exec.phase == ENTRY_PHASE_PREENTRY);
    assert(g.has_s_turn_leg_started);
    assert(fabs(g.s_turn_leg_started_ut - t.ut) < 1e-9);

    /* Loss of the meaningful capture still resets the continuous dwell. */
    t.ut += 5.0;
    t.roll = 1.0;
    entry_update_s_turn_leg_capture(&g, &t, 23.5797, &cfg.vehicle);
    assert(!g.has_s_turn_leg_started);
    assert(g.s_turn_leg_started_ut == 0.0);

    /* Crossing the executive load phase is still sufficient, but the commanded bank
       itself must be meaningful: measured overshoot cannot promote a -2.9 deg target. */
    g.entry_exec.phase = ENTRY_PHASE_TEMPERATURE_CONTROL;
    t.ut = 110.0;
    t.roll = -8.0;
    entry_update_s_turn_leg_capture(&g, &t, -2.9, &cfg.vehicle);
    assert(!g.has_s_turn_leg_started);
    assert(g.s_turn_sign > 0.0);

    /* Measured opposite-side roll is evidence of actuator motion, not permission to
       mutate durable S-turn ownership. v3 proved that allowing physical capture to
       self-authorize a new side bypasses the terminal-arc geometry gate. */
    t.ut = 119.0;
    t.roll = -8.0;
    entry_update_s_turn_leg_capture(&g, &t, -10.0, &cfg.vehicle);
    assert(g.s_turn_sign > 0.0);
    assert(!g.has_s_turn_leg_started);
    assert(g.s_turn_leg_started_ut == 0.0);

    /* Once the plan/executive explicitly owns the opposite side, the same real
       loaded bank capture establishes that leg and starts its dwell clock. */
    request_side(&g, -1.0, 119.5);
    assert(g.s_turn_sign < 0.0);
    assert(g.has_s_turn_reversal_requested);
    t.ut = 120.0;
    t.roll = -8.0;
    entry_update_s_turn_leg_capture(&g, &t, -10.0, &cfg.vehicle);
    assert(g.s_turn_sign < 0.0);
    assert(g.has_s_turn_leg_started);
    assert(fabs(g.s_turn_leg_started_ut - 120.0) < 1e-9);
    assert(!g.has_s_turn_reversal_requested);

    /* Wings-level planning interrupts the physical leg and clears accumulated dwell. */
    t.ut = 130.0;
    t.roll = -2.0;
    entry_update_s_turn_leg_capture(&g, &t, 0.0, &cfg.vehicle);
    assert(!g.has_s_turn_leg_started);
    assert(g.s_turn_leg_started_ut == 0.0);

}


static void test_mm304_routine_replan_may_pull_but_not_postpone_reversal(void) {
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.entry_reversal_scheduled = true;
    g.entry_reversal_ut = 200.0;
    g.entry_reversal_range = 42000.0;
    g.entry_reversal_sign = -1.0;
    g.entry_reversal_bank = 30.0;

    EntryControlPlan plan = {0};
    plan.valid = true;
    plan.has_planned_reversal = true;
    plan.planned_reversal_ut = 220.0;
    plan.planned_reversal_range = 36000.0;
    plan.planned_reversal_sign = -1.0;
    plan.target_bank = 45.0;

    /* A normal model checkpoint may resize the segment, but cannot procrastinate
       an already accepted reversal just because a new solve puts it later. */
    entry_program_commit_planned_reversal(&g, &plan, 150.0, false);
    assert(fabs(g.entry_reversal_ut - 200.0) < 1e-9);
    assert(fabs(g.entry_reversal_range - 42000.0) < 1e-9);
    assert(fabs(g.entry_reversal_bank - 30.0) < 1e-9);
    assert(plan.has_planned_reversal);
    assert(fabs(plan.planned_reversal_ut - 200.0) < 1e-9);
    assert(fabs(plan.planned_reversal_range - 42000.0) < 1e-9);
    assert(plan.planned_reversal_sign < 0.0);

    /* Better current physics may prove an earlier reversal is needed. Pulling the
       milestone earlier is safe and is the intended response to growing authority. */
    plan.planned_reversal_ut = 180.0;
    plan.planned_reversal_range = 47000.0;
    plan.target_bank = 42.0;
    entry_program_commit_planned_reversal(&g, &plan, 150.0, false);
    assert(fabs(g.entry_reversal_ut - 180.0) < 1e-9);
    assert(fabs(g.entry_reversal_range - 47000.0) < 1e-9);
    assert(fabs(g.entry_reversal_bank - 42.0) < 1e-9);
    assert(fabs(plan.planned_reversal_ut - 180.0) < 1e-9);
    assert(fabs(plan.planned_reversal_range - 47000.0) < 1e-9);

    /* Routine replanning cannot invent an opposite-side milestone either. */
    plan.planned_reversal_ut = 170.0;
    plan.planned_reversal_sign = 1.0;
    entry_program_commit_planned_reversal(&g, &plan, 150.0, false);
    assert(fabs(g.entry_reversal_ut - 180.0) < 1e-9);
    assert(g.entry_reversal_sign < 0.0);
    assert(fabs(plan.planned_reversal_ut - 180.0) < 1e-9);
    assert(plan.planned_reversal_sign < 0.0);

    /* Only a materially infeasible-plan replacement may discard the committed
       milestone and install a later one. */
    plan.planned_reversal_ut = 230.0;
    plan.planned_reversal_range = 30000.0;
    plan.planned_reversal_sign = -1.0;
    plan.target_bank = 35.0;
    entry_program_commit_planned_reversal(&g, &plan, 150.0, true);
    assert(fabs(g.entry_reversal_ut - 230.0) < 1e-9);
    assert(fabs(g.entry_reversal_range - 30000.0) < 1e-9);
    assert(fabs(g.entry_reversal_bank - 35.0) < 1e-9);
    assert(fabs(plan.planned_reversal_ut - 230.0) < 1e-9);
    assert(fabs(plan.planned_reversal_range - 30000.0) < 1e-9);
    assert(plan.planned_reversal_sign < 0.0);
}

static void test_mm304_safety_replan_preserves_committed_reversal(void) {
    /* The 04-15 live regression alternated alpha safety replans every few
       seconds and repeatedly postponed a previously accepted upstream
       reversal. Safety is allowed to resize the command, not procrastinate
       the durable milestone. */
    assert(!entry_program_replan_may_discard_reversal(true, false, false, false));
    assert(!entry_program_replan_may_discard_reversal(true, true, true, false));
    assert(!entry_program_replan_may_discard_reversal(false, true, true, false));

    /* A genuinely invalid supervisory trajectory may replace an ordinary milestone,
       but not an overdue terminal event that is being held solely for live geometry. */
    assert(entry_program_replan_may_discard_reversal(false, true, false, false));
    assert(entry_program_replan_may_discard_reversal(true, true, false, false));
    assert(!entry_program_replan_may_discard_reversal(false, true, false, true));
    assert(!entry_program_replan_may_discard_reversal(true, true, false, true));
}

static void test_mm304_persistent_plan_is_cleared_on_guidance_reset(void) {
    GuidanceMachine g;
    guidance_machine_init(&g);
    assert(isinf(g.entry_supervision_ut) && g.entry_supervision_ut < 0.0);
    g.entry_s_turn_plan.valid = true;
    g.entry_s_turn_plan.planned_ut = 123.0;
    g.entry_supervision_ut = 124.0;
    g.entry_control_plan_valid = true;
    g.entry_reversal_scheduled = true;

    guidance_reset_plan(&g);
    assert(!g.entry_s_turn_plan.valid);
    assert(!g.entry_control_plan_valid);
    assert(!g.entry_reversal_scheduled);
    assert(isinf(g.entry_supervision_ut) && g.entry_supervision_ut < 0.0);
}

static void test_executable_control_plan_lineage_is_guidance_owned(void) {
    GuidanceMachine g;
    guidance_machine_init(&g);
    EntryControlPlan first = {.valid = true};
    control_plan_assign_lineage(&g, &first, NULL);
    assert(first.plan_id == 1 && first.plan_version == 1);
    assert(first.parent_plan_id == 0 && first.parent_plan_version == 0);

    EntryControlPlan second = {.valid = true};
    control_plan_assign_lineage(&g, &second, &first);
    assert(second.plan_id == 2 && second.plan_version == 1);
    assert(second.parent_plan_id == first.plan_id);
    assert(second.parent_plan_version == first.plan_version);

    /* Reusing/copying one executable plan must preserve its join key. */
    uint64_t stable_id = second.plan_id;
    control_plan_assign_lineage(&g, &second, &first);
    assert(second.plan_id == stable_id);
    assert(g.control_plan_sequence == 2);

    /* Reset clears active plans but does not recycle ids inside one runtime/log session. */
    guidance_reset_plan(&g);
    EntryControlPlan after_reset = {.valid = true};
    control_plan_assign_lineage(&g, &after_reset, NULL);
    assert(after_reset.plan_id == 3);
}



static void test_mm304_nominal_plan_and_execution_preserve_thermal_aoa(void) {
    LandingConfiguration cfg = landing_configuration_default();
    PlanetModel p = verifier_kerbin();
    AerodynamicModel aero = {.lift_to_drag = .4, .ballistic_coefficient = 700.0, .confidence = .8};
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.phase = PHASE_ENTRY_ENERGY;
    g.automation_engaged = true;
    g.entry_exec.initialized = true;
    g.entry_exec.phase = ENTRY_PHASE_CONSTANT_DRAG;
    g.s_turn_sign = 1.0;

    Telemetry t = entry_telemetry(&cfg);
    t.true_air_speed = 1800.0;
    t.surface_speed = 1800.0;
    t.horizontal_speed = 1798.0;
    t.mean_altitude = 40000.0;
    t.radar_altitude = 39500.0;
    t.range_to_site = 300000.0;
    t.flight_path_angle = -3.0;
    t.vertical_speed = -94.0;
    t.dynamic_pressure = 2500.0; /* Above the legacy low-q-only guard. */
    t.g_force = .25;
    t.stall_fraction = .01;
    t.angle_of_attack = cfg.vehicle.entry_angle_of_attack;
    t.pitch = t.flight_path_angle + t.angle_of_attack;
    t.aerodynamic_confidence = .8;

    /* Recovered/stale metadata may contain an old sub-thermal alpha. The runtime must
       normalize the persistent contract and the live nominal command before replay. */
    g.entry_s_turn_plan.valid = true;
    g.entry_s_turn_plan.planned_ut = t.ut;
    g.entry_s_turn_plan.segment_duration = 75.0;
    g.entry_s_turn_plan.target_bank = 20.0;
    g.entry_s_turn_plan.target_aoa = 10.0;
    g.entry_s_turn_plan.target_heading = cfg.site.runway_heading;
    g.entry_control_plan_valid = true;
    g.entry_supervision_valid = false;
    g.entry_supervision_ut = t.ut;

    GuidanceResult r = entry_program_guidance(&g, &t, NULL,
        cfg.site.runway_heading, &p, aero, &cfg, .1);
    double floor = entry_thermal_protection_aoa_floor(&cfg.vehicle);
    assert(g.entry_s_turn_plan.target_aoa >= floor - 1e-9);
    assert(g.entry_control_aoa >= floor - 1e-9);
    assert(r.command.has_target_aoa);
    assert(r.command.target_aoa >= floor - 1e-9);
    guidance_result_clear(&r);
}

static void test_mm304_late_vertical_debt_preserves_entry_thermal_aoa(void) {
    LandingConfiguration cfg = landing_configuration_default();
    cfg.guidance.taem_interface_altitude = 16500.0;
    cfg.guidance.taem_interface_range = 35000.0;
    cfg.vehicle.entry_angle_of_attack = 18.0;
    cfg.vehicle.maximum_angle_of_attack = 28.0;

    /* The configured Entry incidence is the hard heatshield floor. Thin-air entry
       retains a stronger belly-first drag cushion (90% of max AoA) while control
       authority is weak, then fades smoothly to the floor by q=700 Pa. Vertical
       debt remains a bank/path-geometry problem once bank authority is available. */
    double floor = entry_thermal_protection_aoa_floor(&cfg.vehicle);
    assert(fabs(floor - 18.0) < 1e-9);
    double low0 = entry_low_q_protective_aoa_floor(0.0,&cfg.vehicle);
    double low350 = entry_low_q_protective_aoa_floor(350.0,&cfg.vehicle);
    double low700 = entry_low_q_protective_aoa_floor(700.0,&cfg.vehicle);
    double high = entry_low_q_protective_aoa_floor(2500.0,&cfg.vehicle);
    assert(low0 >= floor - 1e-9 && low0 <= cfg.vehicle.maximum_angle_of_attack + 1e-9);
    assert(low350 >= floor - 1e-9 && low350 <= low0 + 1e-9);
    assert(low700 >= floor - 1e-9 && low700 <= low350 + 1e-9);
    assert(fabs(high - floor) < 1e-9);
}

static void test_mm304_preentry_loaded_vertical_debt_activates_capture(void) {
    LandingConfiguration cfg = landing_configuration_default();
    cfg.guidance.taem_interface_altitude = 16500.0;
    cfg.guidance.taem_interface_range = 72000.0;

    GuidanceMachine g;
    guidance_machine_init(&g);
    g.entry_exec.initialized = true;
    g.entry_exec.phase = ENTRY_PHASE_PREENTRY;

    PlanetModel p = verifier_kerbin();
    AerodynamicModel aero={.lift_to_drag=1.0,.ballistic_coefficient=700.0,.confidence=.9};
    Telemetry t = entry_telemetry(&cfg);
    /* 04-27 live discriminator: executive was still PREENTRY because normal
       acceleration was only ~1.24 m/s2, but q was already high enough for a
       bounded aerodynamic bank and the vehicle was severely high-for-range. */
    t.true_air_speed = 2048.96;
    t.surface_speed = 2048.96;
    t.horizontal_speed = 2048.28;
    t.mean_altitude = 47633.9;
    t.radar_altitude = 47300.7;
    t.range_to_site = 363660.0;
    t.flight_path_angle = -1.483;
    t.dynamic_pressure = 672.52;
    t.g_force = 0.1264;
    t.stall_fraction = 0.01;

    double bank = 0.0;
    double aoa = 20.2;
    seed_entry_vertical_capture_fixture(&g, &t, &cfg);
    assert(entry_program_altitude_capture(&g, &t, &p, aero, &cfg, &bank, &aoa));
    double bank_limit = dynamic_bank_limit(&t, &cfg.vehicle);
    /* The recorded q=672 Pa state is well past the shared aerodynamic-authority
       gate. Keeping the old 2500 Pa near-vacuum ramp here limited bank to ~21 deg
       and let vertical debt grow through 300 km even though live roll tracking was
       stable. Open enough authority for bank/path geometry to become the primary
       vertical actuator while retaining the independent stall/q/g caps. */
    assert(bank_limit >= 44.0);
    assert(fabs(bank) >= 0.75 * bank_limit - 1e-9);
    assert(fabs(bank) <= bank_limit + 1e-9);
    Telemetry thin = t;
    thin.dynamic_pressure = 250.0;
    double thin_limit = dynamic_bank_limit(&thin, &cfg.vehicle);
    Telemetry gate = t;
    gate.dynamic_pressure = fmax(250.0,cfg.vehicle.maximum_dynamic_pressure*.008);
    double gate_limit = dynamic_bank_limit(&gate, &cfg.vehicle);
    Telemetry established = t;
    established.dynamic_pressure = 900.0;
    double established_limit = dynamic_bank_limit(&established, &cfg.vehicle);
    assert(thin_limit >= 0.0 && thin_limit <= gate_limit + 1e-9);
    assert(gate_limit <= established_limit + 1e-9);
    assert(established_limit <= cfg.vehicle.maximum_bank_angle + 1e-9);
    /* Reproduces the consecutive-live reversal chatter: the conservative AoA
       proxy briefly reached ~0.102 while q/speed still allowed high bank. A proxy
       must not masquerade as an independent measured stall and cliff bank to24deg. */
    established.stall_fraction = 0.102206;
    established.stall_fraction_is_measured = false;
    double proxy_limit = dynamic_bank_limit(&established, &cfg.vehicle);
    established.stall_fraction_is_measured = true;
    double measured_limit = dynamic_bank_limit(&established, &cfg.vehicle);
    assert(proxy_limit >= measured_limit - 1e-9);
    established.stall_fraction = 0.20;
    assert(dynamic_bank_limit(&established, &cfg.vehicle) <= measured_limit + 1e-9);
    assert(aoa >= cfg.vehicle.entry_angle_of_attack - 1e-9);
}

static void test_mm304_bank_authority_acquisition_replans_once(void) {
    LandingConfiguration cfg = landing_configuration_default();
    PlanetModel p = verifier_kerbin();
    AerodynamicModel aero={.lift_to_drag=1.0,.ballistic_coefficient=700.0,.confidence=.9};
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.phase = PHASE_ENTRY_ENERGY;
    g.automation_engaged = true;
    g.entry_exec.initialized = true;
    g.entry_exec.phase = ENTRY_PHASE_PREENTRY;
    g.entry_s_turn_plan.valid = true;
    g.entry_s_turn_plan.planned_ut = 67227.7116403517;
    g.entry_s_turn_plan.segment_duration = 75.0;
    g.entry_s_turn_plan.target_bank = 0.0;
    g.entry_s_turn_plan.target_aoa = 18.0;
    g.entry_control_plan_valid = true;

    Telemetry t = entry_telemetry(&cfg);
    t.ut = 67227.7116403517;
    t.true_air_speed = 2090.0;
    t.surface_speed = 2090.0;
    t.horizontal_speed = 2089.0;
    t.vertical_speed = -51.0;
    t.mean_altitude = 54698.0;
    t.radar_altitude = 54000.0;
    t.range_to_site = 637700.0;
    t.flight_path_angle = -1.40;
    t.angle_of_attack = 23.50;
    t.dynamic_pressure = 230.48;
    t.g_force = 0.080;
    t.stall_fraction = 0.0;


    const double parent_end = g.entry_s_turn_plan.planned_ut + g.entry_s_turn_plan.segment_duration;
    /* Old d296 telemetry under the repaired proxy first crosses the shared gate at
       q~=360.084 Pa, about 21.76 s before the parent's scheduled refresh. */
    t.ut = 67280.9516403517;
    t.mean_altitude = 51786.0;
    t.range_to_site = 536309.0;
    t.flight_path_angle = -1.406;
    t.angle_of_attack = 23.277;
    t.dynamic_pressure = 360.084;
    t.g_force = 0.0808;
    t.stall_fraction = 0.0;
    assert(entry_s_turn_bank_authority_available(t.dynamic_pressure,t.true_air_speed,
        t.stall_fraction,t.g_force,&cfg.vehicle));
    assert(entry_bank_authority_replan_due(&g,&t,&cfg.vehicle));
    assert(g.entry_bank_authority_acquired);
    assert(!entry_bank_authority_replan_due(&g,&t,&cfg.vehicle));

    double child_duration=entry_bank_authority_child_duration(&g.entry_s_turn_plan,t.ut,75.0);
    assert(child_duration > 20.0 && child_duration < 23.0);
    assert(fabs((t.ut + child_duration) - parent_end) < 1e-9);

    double bank = 0.0;
    double aoa = entry_low_q_protective_aoa_floor(t.dynamic_pressure,&cfg.vehicle);
    seed_entry_vertical_capture_fixture(&g, &t, &cfg);
    assert(entry_program_altitude_capture(&g,&t,&p,aero,&cfg,&bank,&aoa));
    assert(fabs(bank) > 8.0);

    /* Later stall chatter cannot re-arm the edge and recreate per-tick topology. */
    t.stall_fraction = 0.20;
    assert(!entry_bank_authority_replan_due(&g,&t,&cfg.vehicle));
    t.stall_fraction = 0.0;
    assert(!entry_bank_authority_replan_due(&g,&t,&cfg.vehicle));

    reset_entry_s_turn_program(&g);
    assert(!g.entry_bank_authority_acquired);
}

static void test_mm304_vertical_capture_uses_soft_low_q_authority_and_measured_stall_only(void) {
    LandingConfiguration cfg = landing_configuration_default();
    PlanetModel p = verifier_kerbin();
    AerodynamicModel aero={.lift_to_drag=1.0,.ballistic_coefficient=700.0,.confidence=.9};
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.phase = PHASE_ENTRY_ENERGY;
    g.automation_engaged = true;
    g.s_turn_sign = 1.0;
    g.entry_reference_speed = 2102.7;
    g.entry_reference_altitude = 68343.9;
    g.entry_reference_range = 1068868.3;

    Telemetry t = entry_telemetry(&cfg);
    t.ut = 67258.091259488;
    t.true_air_speed = 2092.324;
    t.surface_speed = t.true_air_speed;
    t.horizontal_speed = 2091.0;
    t.vertical_speed = -52.7;
    t.mean_altitude = 52791.943;
    t.radar_altitude = 52000.0;
    t.range_to_site = 578700.0;
    t.flight_path_angle = -1.441775;
    t.angle_of_attack = 26.080973;
    t.dynamic_pressure = 304.9;
    t.g_force = 0.081;
    t.stall_fraction = 0.314633;
    t.stall_fraction_is_measured = false;

    double bank = 0.0;
    double aoa = t.angle_of_attack;
    seed_entry_vertical_capture_fixture(&g, &t, &cfg);
    assert(entry_program_altitude_capture(&g,&t,&p,aero,&cfg,&bank,&aoa));
    assert(fabs(bank) > 5.0);
    assert(fabs(bank) <= dynamic_bank_limit(&t,&cfg.vehicle) + 1e-9);
    assert(aoa >= entry_thermal_protection_aoa_floor(&cfg.vehicle) - 1e-9);


    /* A stale wings-level segment at the recorded high-debt state can now refresh
       before the stricter leg-authority edge, so the live controller actually gets
       a chance to spend vertical lift instead of waiting for q~=360 Pa. */
    t.dynamic_pressure = 304.9;
    g.entry_s_turn_plan.valid = true;
    g.entry_s_turn_plan.planned_ut = t.ut - 14.0;
    g.entry_s_turn_plan.segment_duration = 75.0;
    g.entry_s_turn_plan.target_bank = 0.0;
    g.entry_s_turn_plan.target_aoa = t.angle_of_attack;
    g.entry_control_plan_valid = true;
    assert(entry_vertical_capture_growth_replan_due(&g,&t,&p,aero,&cfg));
}

static __attribute__((unused)) void test_mm304_vertical_capture_growth_replans_before_stale_plan_expiry(void) {
    LandingConfiguration cfg = landing_configuration_default();
    PlanetModel p = verifier_kerbin();
    AerodynamicModel aero={.lift_to_drag=1.0,.ballistic_coefficient=700.0,.confidence=.9};
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.phase = PHASE_ENTRY_ENERGY;
    g.automation_engaged = true;
    g.s_turn_sign = 1.0;
    g.entry_bank_authority_acquired = true;
    g.entry_control_plan_valid = true;
    g.entry_reference_speed = 2102.7;
    g.entry_reference_altitude = 68343.9;
    g.entry_reference_range = 1068868.3;
    g.entry_s_turn_plan.valid = true;
    g.entry_s_turn_plan.planned_ut = 67302.7319040283;
    g.entry_s_turn_plan.segment_duration = 75.0;
    g.entry_s_turn_plan.target_bank = 23.4260689038583;
    g.entry_s_turn_plan.target_aoa = 20.3986313257462;

    Telemetry t = entry_telemetry(&cfg);
    t.ut = 67303.7319040283;
    t.true_air_speed = 2036.2;
    t.surface_speed = 2036.2;
    t.horizontal_speed = 2035.0;
    t.vertical_speed = -61.0;
    t.mean_altitude = 48087.2113640443;
    t.radar_altitude = 48000.0;
    t.range_to_site = 399814.42836581;
    t.flight_path_angle = -1.71065944145349;
    t.angle_of_attack = 20.3245258331299;
    t.roll = 24.2644653320312;
    t.g_force = 0.121;
    t.stall_fraction = 0.0;

    /* Demand probing must be observational: an unseeded copy may derive anchors for
       the calculation, but the live GuidanceMachine cannot be changed by a due check. */
    GuidanceMachine unseeded = g;
    unseeded.entry_reference_speed = NAN;
    unseeded.entry_reference_altitude = NAN;
    unseeded.entry_reference_range = NAN;
    t.dynamic_pressure = 470.0;
    seed_entry_vertical_capture_fixture(&g, &t, &cfg);
    (void)entry_vertical_capture_growth_replan_due(&unseeded,&t,&p,aero,&cfg);
    assert(isnan(unseeded.entry_reference_speed));
    assert(isnan(unseeded.entry_reference_altitude));
    assert(isnan(unseeded.entry_reference_range));

    /* A fresh child cannot immediately re-solve even if more q is available. */
    assert(!entry_vertical_capture_growth_replan_due(&g,&t,&p,aero,&cfg));

    /* At the recorded high-debt geometry, q=440 growth is intentionally below the
       material hysteresis, while q=470 has opened enough executable bank to matter. */
    t.ut = 67317.7319040283;
    t.dynamic_pressure = 440.0;
    assert(!entry_vertical_capture_growth_replan_due(&g,&t,&p,aero,&cfg));
    t.dynamic_pressure = 470.0;
    double first_bank = g.entry_s_turn_plan.target_bank;
    double first_aoa = g.entry_s_turn_plan.target_aoa;
    assert(entry_program_altitude_capture(&g,&t,&p,aero,&cfg,&first_bank,&first_aoa));
    assert(first_bank > g.entry_s_turn_plan.target_bank + 8.0);
    assert(first_bank <= dynamic_bank_limit(&t,&cfg.vehicle) + 1e-9);
    assert(entry_vertical_capture_growth_replan_due(&g,&t,&p,aero,&cfg));

    const double original_end = g.entry_s_turn_plan.planned_ut + g.entry_s_turn_plan.segment_duration;
    double first_duration = entry_authority_refresh_child_duration(&g,&g.entry_s_turn_plan,
        t.ut,75.0,&cfg.guidance);
    assert(fabs((t.ut + first_duration) - original_end) < 1e-9);
    g.entry_s_turn_plan.target_bank = first_bank;
    g.entry_s_turn_plan.target_aoa = first_aoa;
    g.entry_s_turn_plan.planned_ut = t.ut;
    g.entry_s_turn_plan.segment_duration = first_duration;

    /* Same state, tiny q jitter, and stall chatter cannot immediately re-arm. */
    t.ut += 5.1;
    t.dynamic_pressure = 472.0;
    assert(!entry_vertical_capture_growth_replan_due(&g,&t,&p,aero,&cfg));
    t.stall_fraction = 0.20;
    assert(!entry_vertical_capture_growth_replan_due(&g,&t,&p,aero,&cfg));
    t.stall_fraction = 0.0;

    /* A later new material opening may refresh again, but only after the age gate. */
    t.ut += 10.0;
    t.dynamic_pressure = 530.0;
    double second_bank = g.entry_s_turn_plan.target_bank;
    double second_aoa = g.entry_s_turn_plan.target_aoa;
    assert(entry_program_altitude_capture(&g,&t,&p,aero,&cfg,&second_bank,&second_aoa));
    assert(second_bank > first_bank + 8.0);
    assert(second_bank <= dynamic_bank_limit(&t,&cfg.vehicle) + 1e-9);
    assert(entry_vertical_capture_growth_replan_due(&g,&t,&p,aero,&cfg));
    double second_duration = entry_authority_refresh_child_duration(&g,&g.entry_s_turn_plan,
        t.ut,75.0,&cfg.guidance);
    assert(fabs((t.ut + second_duration) - original_end) < 1e-9);

    g.entry_s_turn_plan.target_bank = second_bank;
    g.entry_s_turn_plan.target_aoa = second_aoa;
    g.entry_s_turn_plan.planned_ut = t.ut;
    g.entry_s_turn_plan.segment_duration = second_duration;
    t.ut += 5.1;
    t.dynamic_pressure = 560.0;
    assert(!entry_vertical_capture_growth_replan_due(&g,&t,&p,aero,&cfg));

    /* An already committed, physically executable reversal is an even earlier
       anti-procrastination deadline than the replaced segment end. */
    g.entry_reversal_scheduled = true;
    g.has_s_turn_leg_started = true;
    g.has_s_turn_reversal_requested = false;
    g.s_turn_leg_started_ut = t.ut - cfg.guidance.s_turn_minimum_leg_duration - 5.0;
    g.entry_reversal_ut = t.ut + 7.0;
    double reversal_limited = entry_authority_refresh_child_duration(&g,&g.entry_s_turn_plan,
        t.ut,75.0,&cfg.guidance);
    assert(fabs(reversal_limited - 7.0) < 1e-9);
}

static void test_mm304_live_bank_limit_matches_shared_predictor_authority_core(void) {
    LandingConfiguration cfg = landing_configuration_default();
    Telemetry t = entry_telemetry(&cfg);
    t.true_air_speed = 2050.0;
    t.g_force = 0.13;
    t.stall_fraction = 0.0;

    const double authority_q = fmax(250.0, cfg.vehicle.maximum_dynamic_pressure * .008);
    const double pressures[] = {250.0, authority_q, 400.0, 600.0, 672.52, 900.0};
    for (size_t i = 0; i < sizeof(pressures) / sizeof(pressures[0]); ++i) {
        t.dynamic_pressure = pressures[i];
        double shared = entry_bank_authority_limit(t.true_air_speed, t.dynamic_pressure,
            t.g_force, &cfg.vehicle, cfg.vehicle.maximum_bank_angle);
        double live = dynamic_bank_limit(&t, &cfg.vehicle);
        assert(fabs(live - shared) < 1e-12);
    }

    /* Speed and g protections are part of the same shared predictor/live core. */
    t.dynamic_pressure = 900.0;
    t.true_air_speed = cfg.vehicle.minimum_safe_speed * 1.05;
    double slow = entry_bank_authority_limit(t.true_air_speed, t.dynamic_pressure,
        t.g_force, &cfg.vehicle, cfg.vehicle.maximum_bank_angle);
    assert(fabs(dynamic_bank_limit(&t, &cfg.vehicle) - slow) < 1e-12);
    assert(slow >= 0.0 && slow <= cfg.vehicle.maximum_bank_angle + 1e-12);

    t.true_air_speed = 2050.0;
    t.g_force = cfg.vehicle.maximum_g_load * .95;
    double high_g = entry_bank_authority_limit(t.true_air_speed, t.dynamic_pressure,
        t.g_force, &cfg.vehicle, cfg.vehicle.maximum_bank_angle);
    assert(fabs(dynamic_bank_limit(&t, &cfg.vehicle) - high_g) < 1e-12);
    assert(high_g >= 0.0 && high_g <= cfg.vehicle.maximum_bank_angle + 1e-12);

    /* A conservative proxy remains useful to @ALPHA and authority guards, but it
       must not masquerade as an independent measured stall and cliff bank to24deg. */
    t.g_force = .13;
    t.stall_fraction = .20;
    t.stall_fraction_is_measured = false;
    double shared_without_measured_stall = entry_bank_authority_limit(t.true_air_speed,
        t.dynamic_pressure, t.g_force, &cfg.vehicle, cfg.vehicle.maximum_bank_angle);
    assert(shared_without_measured_stall >= 0.0);
    assert(shared_without_measured_stall <= cfg.vehicle.maximum_bank_angle + 1e-12);
    assert(fabs(dynamic_bank_limit(&t, &cfg.vehicle) - shared_without_measured_stall) < 1e-12);

    /* A genuine independent measured-stall observable must not increase the
       available bank envelope relative to the conservative proxy. */
    t.stall_fraction_is_measured = true;
    assert(dynamic_bank_limit(&t, &cfg.vehicle) <= shared_without_measured_stall + 1e-12);
}
static void test_mm304_aoa_reference_does_not_overshoot_reversed_target(void) {
    LandingConfiguration cfg = landing_configuration_default();
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.has_last_stabilized_phase = true;
    g.last_stabilized_phase = PHASE_ENTRY_ENERGY;
    g.pitch_limiter.has_value = true;
    g.pitch_limiter.value = 19.1;
    g.pitch_limiter.rate = -2.0;

    Telemetry t = entry_telemetry(&cfg);
    t.dynamic_pressure = 20.0;
    t.angle_of_attack = 19.1;
    t.flight_path_angle = -2.0;

    GuidanceCommand c;
    guidance_command_init(&c);
    c.autopilot_engaged = true;
    c.control_profile = PROFILE_ENTRY;
    c.has_target_aoa = true;
    c.target_aoa = 19.0;
    c.target_pitch = 17.0;
    c.target_roll = 0.0;
    c.target_heading = cfg.site.runway_heading;
    GuidanceResult in = result_make(PHASE_ENTRY_ENERGY, c, "test", NULL);
    GuidanceResult out = stabilized(&g, in, &t, &cfg.vehicle, &cfg.guidance, 0.1);

    assert(isfinite(out.command.target_aoa));
    assert(out.command.target_aoa >= 0.0);
    assert(out.command.target_aoa <= cfg.vehicle.maximum_angle_of_attack + 1e-9);
    assert(out.command.target_aoa <= 19.1 + 1e-9);
    guidance_result_clear(&out);
}


static void test_mm304_roll_reference_stops_when_bank_target_reverses(void) {
    LandingConfiguration cfg = landing_configuration_default();
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.has_last_stabilized_phase = true;
    g.last_stabilized_phase = PHASE_ENTRY_ENERGY;
    g.roll_limiter.has_value = true;
    g.roll_limiter.value = 44.0;
    g.roll_limiter.rate = 4.5;
    g.pitch_limiter.has_value = true;
    g.pitch_limiter.value = 20.0;
    g.pitch_limiter.rate = 0.0;

    Telemetry t = entry_telemetry(&cfg);
    t.dynamic_pressure = 4000.0;
    t.roll = 44.0;
    t.angle_of_attack = 20.0;
    t.flight_path_angle = -5.0;

    GuidanceCommand c;
    guidance_command_init(&c);
    c.autopilot_engaged = true;
    c.control_profile = PROFILE_ENTRY;
    c.has_target_aoa = true;
    c.target_aoa = 20.0;
    c.target_pitch = 15.0;
    c.target_roll = 24.0;
    c.target_heading = cfg.site.runway_heading;
    GuidanceResult in = result_make(PHASE_ENTRY_ENERGY, c, "test", NULL);
    GuidanceResult out = stabilized(&g, in, &t, &cfg.vehicle, &cfg.guidance, 0.1);

    assert(isfinite(out.command.target_roll));
    assert(fabs(out.command.target_roll) <= cfg.vehicle.maximum_bank_angle + 1e-9);
    guidance_result_clear(&out);
}


static __attribute__((unused)) void test_taem_handoff_requires_nominal_mm304_capture(void) {
    LandingConfiguration cfg = landing_configuration_default();
    cfg.guidance.taem_force_handoff_speed = 1300.0;
    cfg.guidance.taem_interface_altitude = 16500.0;
    cfg.guidance.taem_interface_range = 35000.0;
    cfg.guidance.final_approach_distance = 8000.0;
    cfg.guidance.hac_radius = 12000.0;
    PlanetModel p = verifier_kerbin();
    AerodynamicModel aero = {
        .lift_to_drag = 0.56,
        .ballistic_coefficient = 751.0,
        .confidence = 0.78,
    };
    GuidanceMachine g;
    guidance_machine_init(&g);
    guidance_set_engaged(&g, true);
    g.phase = PHASE_ENTRY_ENERGY;
    g.entry_exec.initialized = true;
    g.entry_exec.phase = ENTRY_PHASE_TRANSITION;
    g.entry_exec.entry_complete = false;
    g.s_turn_sign = -1.0;

    /* Recorded immediately before the 2026-09-11T11-24-50Z abort. The nominal
       fixed point/course contract was still missed when TAS crossed the historical
       1300 m/s threshold. That threshold is no longer an ownership event: without
       explicit interface capture MM304 must retain authority. */
    Telemetry t = entry_telemetry(&cfg);
    t.ut = 67569.4297361134;
    t.latitude = -1.00778189087511;
    t.longitude = -75.5217859485948;
    t.true_air_speed = 1299.72436523438;
    t.surface_speed = 1299.72436523438;
    t.horizontal_speed = 1292.3;
    t.mean_altitude = 24315.0368195267;
    t.radar_altitude = 24035.0;
    t.vertical_speed = -138.586165888105;
    t.range_to_site = 12990.8112506345;
    t.runway_along_track = -8212.04715610793;
    t.runway_cross_track = 10066.3173960251;
    t.bearing_to_site = 39.60;
    t.heading = 112.68;
    t.ground_track_heading = 102.72;
    t.flight_path_angle = -6.12093350981417;
    t.dynamic_pressure = 14552.7;
    t.g_force = 2.0269;
    t.mach = 4.20;
    t.roll = 39.02;
    t.angle_of_attack = 15.83;
    t.sideslip = 0.10;
    t.mass = 40251.27;
    t.lift_force = 388711.0;
    t.drag_force = 701077.0;
    t.aerodynamic_confidence = 0.775;
    t.bank_effectiveness = 0.996;
    t.has_body_pitch_rate = true;
    t.has_body_roll_rate = true;
    t.has_body_yaw_rate = true;
    t.body_pitch_rate = 0.13;
    t.body_roll_rate = -5.41;
    t.body_yaw_rate = -1.57;
    t.energy_excess_range = 91083.26;
    t.predicted_taem_distance = 12600.16;
    t.predicted_taem_speed = 1291.50;
    t.predicted_taem_energy_error = 539287.0;

    GuidanceResult r = terminal_guidance(&g, &t, NULL, t.ground_track_heading,
        &p, aero, &cfg, 0.1);
    assert(!g.entry_exec.entry_complete);
    assert(!g.terminal_region_entered);
    assert(!taem_exec_owns_vehicle(&g.taem_exec));
    assert(!g.terminal_path_committed);
    assert(r.phase == PHASE_ENTRY_ENERGY);
    guidance_result_clear(&r);

    /* Passing the fixed rear-alignment station without satisfying the full MM304
       contract cannot manufacture MM305 ownership. The correct response is to keep
       Entry ownership while a physically valid capture remains possible, or abort
       if it does not; there is no degraded TAEM ownership channel. */
    t.ut += 0.1;
    t.runway_along_track = -cfg.guidance.final_approach_distance;
    t.range_to_site = hypot(t.runway_along_track, t.runway_cross_track);
    t.runway_along_track = -cfg.guidance.final_approach_distance + 1001.0;
    t.range_to_site = hypot(t.runway_along_track, t.runway_cross_track);
    r = terminal_guidance(&g, &t, NULL, t.ground_track_heading, &p, aero, &cfg, 0.1);
    assert(!g.entry_exec.entry_complete);
    assert(!g.terminal_region_entered);
    assert(!taem_exec_owns_vehicle(&g.taem_exec));
    assert(!g.terminal_path_committed);
    guidance_result_clear(&r);

    /* Descending below a historical altitude floor without the fixed-point/course
       contract also cannot create TAEM ownership. Altitude is evidence about the
       remaining MM304 feasibility, not an alternate state-machine transition. */
    guidance_machine_init(&g);
    guidance_set_engaged(&g, true);
    g.phase = PHASE_ENTRY_ENERGY;
    g.entry_exec.initialized = true;
    g.entry_exec.phase = ENTRY_PHASE_TRANSITION;
    g.entry_exec.entry_complete = false;
    g.s_turn_sign = -1.0;
    t = entry_telemetry(&cfg);
    t.ut = 67571.0;
    t.true_air_speed = 1350.0;
    t.surface_speed = 1350.0;
    t.horizontal_speed = 1335.0;
    t.mean_altitude = cfg.site.altitude + 14950.0;
    t.radar_altitude = 14950.0;
    t.vertical_speed = -120.0;
    t.flight_path_angle = -5.1;
    t.runway_along_track = -20000.0;
    t.runway_cross_track = 6000.0;
    t.range_to_site = hypot(t.runway_along_track, t.runway_cross_track);
    t.ground_track_heading = 90.0;
    t.heading = 90.0;
    t.roll = 0.0;
    t.body_pitch_rate = 0.0;
    t.body_roll_rate = 0.0;
    t.body_yaw_rate = 0.0;
    r = terminal_guidance(&g, &t, NULL, t.ground_track_heading, &p, aero, &cfg, 0.1);
    assert(!g.entry_exec.entry_complete);
    assert(!g.terminal_region_entered);
    assert(!taem_exec_owns_vehicle(&g.taem_exec));
    guidance_result_clear(&r);

    /* A successful fixed-interface capture is different. Publish the normal MM304
       target while safely upstream, then place the vehicle exactly on that target
       and mark the executive capture facts. Only the qualified contract may latch
       MM305 ownership. */
    guidance_machine_init(&g);
    guidance_set_engaged(&g, true);
    g.phase = PHASE_ENTRY_ENERGY;
    g.entry_exec.initialized = true;
    g.entry_exec.phase = ENTRY_PHASE_TRANSITION;
    g.entry_exec.entry_complete = false;
    g.s_turn_sign = -1.0;

    GeoPoint threshold = {cfg.site.latitude, cfg.site.longitude, cfg.site.altitude};
    GeoPoint staged = local_point(threshold, -70000.0, 0.0, p.radius,
                                  cfg.guidance.taem_interface_altitude + 6000.0);
    t = entry_telemetry(&cfg);
    t.ut = 67570.5;
    t.latitude = staged.latitude;
    t.longitude = staged.longitude;
    t.mean_altitude = cfg.guidance.taem_interface_altitude + 6000.0;
    t.radar_altitude = t.mean_altitude - cfg.site.altitude;
    t.true_air_speed = 1400.0;
    t.surface_speed = t.true_air_speed;
    t.horizontal_speed = 1395.0;
    t.vertical_speed = -110.0;
    t.flight_path_angle = -4.5;
    t.heading = 90.0;
    t.ground_track_heading = 90.0;
    t.roll = 0.0;
    t.angle_of_attack = 16.0;
    t.body_pitch_rate = 0.0;
    t.body_roll_rate = 0.0;
    t.body_yaw_rate = 0.0;
    t.attitude_response.pitch_valid = true;
    t.attitude_response.maximum_pitch_rate_deg_s = 8.0;
    t.attitude_response.maximum_pitch_accel_deg_s2 = 5.0;
    t.attitude_response.roll_valid = true;
    t.attitude_response.maximum_roll_rate_deg_s = 18.0;
    t.attitude_response.maximum_roll_accel_deg_s2 = 15.0;
    GeoPoint current = {t.latitude, t.longitude, t.mean_altitude};
    double along = 0.0, cross = 0.0;
    runway_coordinates(current, threshold, cfg.site.runway_heading, p.radius, &along, &cross);
    t.runway_along_track = along;
    t.runway_cross_track = cross;
    t.range_to_site = hypot(along, cross);
    assert(t.runway_along_track < -cfg.guidance.final_approach_distance - 50000.0);

    g.taem_interface_target = (TaemInterfaceTarget){
        .valid = true, .energy_qualified = true,
        .along_track = -cfg.guidance.final_approach_distance,
        .cross_track = 0.0,
        .course = norm_deg(cfg.site.runway_heading + 90.0),
        .altitude = cfg.guidance.taem_interface_altitude,
        .speed = cfg.guidance.taem_force_handoff_speed,
        .flight_path_angle = -fabs(cfg.guidance.taem_glide_slope),
        .acquisition_lead = 5000.0, .response_time = 3.0
    };
    TaemInterfaceTarget handoff_target = g.taem_interface_target;
    assert(fabs(handoff_target.along_track + cfg.guidance.final_approach_distance) < 1e-6);
    assert(fabs(handoff_target.cross_track) < 1e-6);
    double runway_offset = fabs(norm_signed_deg(handoff_target.course - cfg.site.runway_heading));
    assert(runway_offset >= 60.0 - 1e-6 && runway_offset <= 120.0 + 1e-6);
    assert(isfinite(handoff_target.acquisition_lead) && handoff_target.acquisition_lead > 0.0);
    assert(isfinite(handoff_target.response_time) && handoff_target.response_time > 0.0);

    double rh = cfg.site.runway_heading * DEG2RAD;
    double target_e = handoff_target.along_track * sin(rh) + handoff_target.cross_track * cos(rh);
    double target_n = handoff_target.along_track * cos(rh) - handoff_target.cross_track * sin(rh);
    staged = local_point(threshold, target_e, target_n, p.radius, handoff_target.altitude);
    t.ut += 0.1;
    t.latitude = staged.latitude;
    t.longitude = staged.longitude;
    t.mean_altitude = handoff_target.altitude;
    t.radar_altitude = t.mean_altitude - cfg.site.altitude;
    t.true_air_speed = fmin(handoff_target.speed, cfg.guidance.taem_force_handoff_speed);
    t.surface_speed = t.true_air_speed;
    t.flight_path_angle = handoff_target.flight_path_angle;
    t.horizontal_speed = t.true_air_speed * cos(t.flight_path_angle * DEG2RAD);
    t.vertical_speed = t.true_air_speed * sin(t.flight_path_angle * DEG2RAD);
    t.ground_track_heading = handoff_target.course;
    t.heading = handoff_target.course;
    t.roll = 0.0;
    t.body_pitch_rate = 0.0;
    t.body_roll_rate = 0.0;
    t.body_yaw_rate = 0.0;
    current = (GeoPoint){t.latitude, t.longitude, t.mean_altitude};
    runway_coordinates(current, threshold, cfg.site.runway_heading, p.radius,
                       &t.runway_along_track, &t.runway_cross_track);
    t.range_to_site = hypot(t.runway_along_track, t.runway_cross_track);
    assert(hypot(t.runway_along_track - handoff_target.along_track,
                 t.runway_cross_track - handoff_target.cross_track) < 1.0);

    g.entry_exec.entry_complete = true;
    g.taem_interface_captured = true;
    r = terminal_guidance(&g, &t, NULL, t.ground_track_heading, &p, aero, &cfg, 0.1);
    assert(g.terminal_region_entered);
    assert(taem_exec_owns_vehicle(&g.taem_exec));
    assert(g.taem_exec.last_transition_reason == TAEM_TRANSITION_MM304_HANDOFF);
    assert(r.phase == PHASE_TAEM);
    guidance_result_clear(&r);
}

static void test_latched_taem_path_recovery_never_returns_to_entry(void) {
    LandingConfiguration cfg = landing_configuration_default();
    PlanetModel p = verifier_kerbin();
    AerodynamicModel aero = {
        .lift_to_drag = 0.55,
        .ballistic_coefficient = 750.0,
        .confidence = 0.8,
    };
    GuidanceMachine g;
    guidance_machine_init(&g);
    guidance_set_engaged(&g, true);
    g.phase = PHASE_TAEM;
    g.terminal_region_entered = true;
    g.terminal_glide_mode = true;
    g.terminal_path_committed = true;
    g.hac_side_selected = true;
    g.hac_captured = true;
    g.taem_exec.initialized = true;
    g.taem_exec.ownership_latched = true;
    g.taem_exec.phase = TAEM_PHASE_RUNWAY_ALIGNMENT;

    Telemetry t = entry_telemetry(&cfg);
    t.ut = 7000.0;
    t.true_air_speed = 900.0;
    t.surface_speed = 900.0;
    t.horizontal_speed = 880.0;
    t.mean_altitude = 19000.0;
    t.radar_altitude = 19000.0;
    t.vertical_speed = -90.0;
    t.flight_path_angle = -8.0;
    t.dynamic_pressure = 9000.0;
    t.g_force = 1.2;
    t.angle_of_attack = 12.0;
    t.roll = -25.0;
    t.range_to_site = 60000.0;
    t.runway_along_track = -55000.0;
    t.runway_cross_track = 6000.0;
    t.energy_excess_range = 20000.0;

    GuidanceResult r = terminal_return_to_entry(&g, &t, t.ground_track_heading,
        &p, aero, &cfg, 0.1);

    assert(taem_exec_owns_vehicle(&g.taem_exec));
    assert(g.terminal_region_entered);
    assert(g.terminal_glide_mode);
    assert(!g.terminal_path_committed);
    assert(g.phase == PHASE_TAEM);
    assert(r.phase == PHASE_TAEM);
    assert(g.taem_exec.phase == TAEM_PHASE_PATH_ACQUISITION);
    assert(strstr(r.status, "TAEM") != NULL);
    guidance_result_clear(&r);
}

static __attribute__((unused)) void test_stale_regular_hac_preview_may_refresh_without_relaxing_geometry(void) {
    GuidanceMachine g;
    guidance_machine_init(&g);
    TerminalCandidate retained = {
        .valid = true,
        .geometry_degraded = false,
        .energy_degraded = false,
        .speed = 600.0,
        .altitude = 18000.0,
        .selected_ut = 1000.0,
    };
    TerminalCandidate fresh = retained;
    fresh.energy_degraded = true;
    fresh.altitude = 19050.0;
    fresh.selected_ut = 1002.0;

    /* Energy hysteresis alone would retain the old candidate. Once its propagated
       arrival has drifted outside the shared forecast envelope, a fresh regular
       preview must be allowed to replace it. */
    assert(!terminal_candidate_refinement_ok(&retained, &fresh));
    g.terminal_prediction_altitude = retained.altitude + 1200.0;
    g.terminal_prediction_speed = retained.speed;
    assert(terminal_candidate_refresh_allowed(&g, &retained, &fresh));

    fresh.geometry_degraded = true;
    assert(!terminal_candidate_refresh_allowed(&g, &retained, &fresh));
}


static void test_geometry_degraded_preview_cannot_replace_recent_clean_entry_inlet(void) {
    LandingConfiguration cfg = landing_configuration_default();
    PlanetModel p = verifier_kerbin();
    AerodynamicModel aero = {.lift_to_drag = .4, .ballistic_coefficient = 700.0, .confidence = .12};
    GuidanceMachine g;
    guidance_machine_init(&g);
    Telemetry t;
    telemetry_init(&t);

    /* v18 immediately before the degraded terminal replacement. The last clean
       dynamic inlet was upstream of the live orbiter but still within the capture
       gate's intentional downstream grace, so its vertical-delivery contract was
       still meaningful. */
    t.ut = 67557.28;
    t.runway_along_track = -57765.3209237946;
    t.runway_cross_track = 1022.89520181561;
    t.horizontal_speed = 1075.0;
    g.taem_interface_target = (TaemInterfaceTarget){
        .valid = true,
        .along_track = -73881.0,
        .cross_track = 0.0,
        .course = cfg.site.runway_heading,
        .altitude = 27248.0,
        .speed = 1224.5,
        .flight_path_angle = -16.0,
        .specific_energy = 1.0,
        .acquisition_lead = 65881.0,
        .remaining_path = 100000.0,
        .response_time = 8.0,
    };
    assert(terminal_interface_target_spatially_relevant(&g.taem_interface_target, &t, &cfg));

    g.terminal_candidate.valid = true;
    g.terminal_candidate.geometry_degraded = true;
    terminal_publish_interface_target(&g, &t, &p, aero, &cfg);
    assert(g.taem_interface_target.valid);
    assert(fabs(g.taem_interface_target.along_track + 73881.0) < 1e-9);
    assert(fabs(g.taem_interface_target.flight_path_angle + 16.0) < 1e-9);

    /* Past the same capture grace the clean demand must expire rather than becoming
       a permanent stale target while degraded previews are all that remain. */
    t.runway_along_track = -45000.0;
    t.runway_cross_track = 1000.0;
    assert(!terminal_interface_target_spatially_relevant(&g.taem_interface_target, &t, &cfg));
    terminal_publish_interface_target(&g, &t, &p, aero, &cfg);
    assert(!g.taem_interface_target.valid);
}

static void test_latched_taem_target_is_immutable_after_handoff(void) {
    LandingConfiguration cfg = landing_configuration_default();
    PlanetModel p = verifier_kerbin();
    AerodynamicModel aero = {.lift_to_drag = .4, .ballistic_coefficient = 700.0, .confidence = .12};
    GuidanceMachine g;
    guidance_machine_init(&g);
    Telemetry t = entry_telemetry(&cfg);

    g.taem_interface_captured = true;
    g.taem_interface_target = (TaemInterfaceTarget){
        .valid = true,
        .along_track = -8000.0,
        .cross_track = 0.0,
        .course = cfg.site.runway_heading,
        .altitude = 16500.0,
        .speed = 520.0,
        .flight_path_angle = -12.0,
        .acquisition_lead = 9000.0,
        .remaining_path = 35000.0,
        .response_time = 8.0,
    };
    g.terminal_candidate.valid = true;
    g.terminal_candidate.geometry_degraded = false;
    g.terminal_candidate.join.valid = true;
    g.terminal_candidate.join.p0 = (HACPoint2){-40000.0, 0.0};
    g.terminal_candidate.join.p1 = (HACPoint2){-30000.0, 0.0};
    g.terminal_candidate.join.p2 = (HACPoint2){-20000.0, 0.0};
    g.terminal_candidate.join.p3 = (HACPoint2){-8000.0, 0.0};

    TaemInterfaceTarget fixed = g.taem_interface_target;
    terminal_publish_interface_target(&g, &t, &p, aero, &cfg);
    assert(g.taem_interface_target.valid == fixed.valid);
    assert(fabs(g.taem_interface_target.along_track - fixed.along_track) < 1e-9);
    assert(fabs(g.taem_interface_target.cross_track - fixed.cross_track) < 1e-9);
    assert(fabs(g.taem_interface_target.course - fixed.course) < 1e-9);
    assert(fabs(g.taem_interface_target.altitude - fixed.altitude) < 1e-9);
    assert(fabs(g.taem_interface_target.speed - fixed.speed) < 1e-9);
    assert(fabs(g.taem_interface_target.flight_path_angle - fixed.flight_path_angle) < 1e-9);
    assert(fabs(g.taem_interface_target.acquisition_lead - fixed.acquisition_lead) < 1e-9);
    assert(fabs(g.taem_interface_target.remaining_path - fixed.remaining_path) < 1e-9);
    assert(fabs(g.taem_interface_target.response_time - fixed.response_time) < 1e-9);
}

static void test_v12_committed_spline_detects_vertical_closure_loss(void) {
    LandingConfiguration cfg = landing_configuration_default();
    GuidanceMachine g;
    guidance_machine_init(&g);
    terminal_glide_initialize(&g, &cfg.vehicle, &cfg.guidance);

    assert(fabs(g.terminal_test_preflare_altitude - 500.0) < 1e-9);
    assert(fabs(g.terminal_test_glide_slope - 22.0) < 1e-9);
    double gate_altitude = cfg.site.altitude + g.terminal_test_preflare_altitude;
    double gate_ground = g.terminal_test_preflare_altitude /
        tan(g.terminal_test_glide_slope * DEG2RAD);


    /* By UT 67584.7 the frozen spline had only 21.1 km left while the vehicle
       was still at 21.3 km altitude. Even an instantaneous -35 deg descent
       was about 1.7 km of ground path short of the preflare gate. */
    double failed_height = 21300.0 - gate_altitude;
    double failed_path = 21100.0 + cfg.guidance.final_approach_distance - gate_ground;
    assert(terminal_vertical_path_unrecoverable(failed_height, failed_path, 35.0));
    double required_extra = failed_height / tan(35.0 * DEG2RAD) - failed_path;
    assert(required_extra > 1600.0);

    /* A longer replacement geometry is the correct causal remedy; adding 3 km
       restores the hard vertical-closure condition without weakening Final. */
    assert(!terminal_vertical_path_unrecoverable(failed_height, failed_path + 3000.0, 35.0));

    /* Response-limited vertical drop is now represented by the shared vertical
       closure/unrecoverable-path predicate above; the removed helper no longer
       has a separate public contract to assert here. */
}

static __attribute__((unused)) void test_recorded_late_taem_state_rejects_control_degraded_geometry_and_energy_commit(void) {
    LandingConfiguration cfg = landing_configuration_default();
    PlanetModel p = verifier_kerbin();
    AerodynamicModel aero = {.lift_to_drag = .4, .ballistic_coefficient = 700.0, .confidence = .12};
    GuidanceMachine g;
    guidance_machine_init(&g);
    guidance_set_engaged(&g, true);
    terminal_glide_initialize(&g, &cfg.vehicle, &cfg.guidance);
    g.terminal_region_entered = true;

    /* 2026-09-10T01-51-41Z seq 863: the historical controller had already
       chased cross-track to about 9.1 km and was descending near -28 deg.
       Preserve a finite terminal preview for MM304/TAEM shaping, but do not
       freeze it when propagated lateral/control authority marks the HAC geometry
       dynamically degraded. Measured unpowered energy must independently reject
       the same candidate even if geometry is hypothetically made usable. */
    Telemetry t;
    telemetry_init(&t);
    t.ut = 67646.0322752074;
    t.latitude = .818966203614688;
    t.longitude = -77.0518186373342;
    t.mean_altitude = 10103.7269299066;
    t.radar_altitude = 9467.99738015211;
    t.vertical_speed = -136.810928618465;
    t.horizontal_speed = 255.070730512862;
    t.surface_speed = 289.444826787096;
    t.true_air_speed = 289.44482421875;
    t.atmospheric_density = .275110512971878;
    t.speed_of_sound = 298.949951171875;
    t.mach = .970705568790436;
    t.heading = 61.4724273681641;
    t.ground_track_heading = 66.2248121189724;
    t.pitch = -21.2563343048096;
    t.roll = -32.6778869628906;
    t.angle_of_attack = 8.17587947845459;
    t.sideslip = -.25676617026329;
    t.dynamic_pressure = 11524.146484375;
    t.g_force = 1.38116419315338;
    t.lift_force = 298243.64226661;
    t.drag_force = 455651.387877783;
    t.mass = 40730.4140625;
    t.runway_along_track = -24331.4724490779;
    t.runway_cross_track = -9085.24839442941;
    t.range_to_site = hypot(t.runway_along_track, t.runway_cross_track);
    t.flight_path_angle = -28.2075369441143;
    t.estimated_lift_to_drag = .4;
    t.estimated_ballistic_coefficient = 700.0;
    t.aerodynamic_confidence = .12;
    t.trajectory_density_scale = .996080015593;
    t.trajectory_drag_scale = 1.0;
    t.trajectory_lift_scale = 1.0;
    t.bank_effectiveness = 1.0;
    t.trajectory_calibration_confidence = .05;

    g.terminal_energy_loss_accel_ema = t.drag_force / t.mass;
    g.terminal_speed_loss_accel_ema = g.terminal_energy_loss_accel_ema;
    terminal_predict(&g, &t, t.ground_track_heading, &p, aero, &cfg, .1);

    assert(g.terminal_candidate.valid);
    assert(g.terminal_candidate.kind == TERMINAL_PATH_HAC);
    assert(g.terminal_candidate.join.valid && g.terminal_candidate.join.degraded);
    assert(g.terminal_candidate.join.violation_score > 1.0);
    assert(g.terminal_candidate.geometry_degraded);
    assert(!terminal_candidate_operationally_usable(&g, &g.terminal_candidate, &t, t.ground_track_heading, &p, &cfg));

    /* Preferred altitude-shell misses are ranking degradation, not a reason to
       throw away otherwise executable geometry. Isolate that policy from the
       recorded control violation with a copy. */
    TerminalCandidate policy = g.terminal_candidate;
    policy.geometry_degraded = false;
    policy.shell_degraded = true;
    assert(terminal_candidate_operationally_usable(&g, &policy, &t, t.ground_track_heading, &p, &cfg));
    policy.geometry_degraded = true;
    assert(!terminal_candidate_operationally_usable(&g, &policy, &t, t.ground_track_heading, &p, &cfg));

    /* Prove energy is an independent blocker rather than relying on the geometry
       veto to make commit fail. */
    TerminalCandidate recorded = g.terminal_candidate;
    g.terminal_candidate.geometry_degraded = false;
    assert(!terminal_candidate_live_energy_ready(&g, &t, &p, aero, &cfg, &g.terminal_candidate));
    assert(!terminal_candidate_commit_ready(&g, &t, &p, aero, &cfg));
    g.terminal_candidate = recorded;
    assert(g.terminal_reference_fpa > t.flight_path_angle);
}

static void test_proven_taem_boundary_miss_replans_without_infeasible_dwell(void){
    LandingConfiguration cfg=landing_configuration_default();
    GuidanceMachine g;guidance_machine_init(&g);
    Telemetry t;telemetry_init(&t);
    g.entry_s_turn_plan.valid=true;g.entry_s_turn_plan.planned_ut=100.0;
    g.entry_supervision_valid=false;g.entry_supervision_mode=ENTRY_SUPERVISION_INFEASIBLE;
    g.entry_supervision_ut=101.0;t.ut=101.0;
    assert(!entry_supervision_replan_due(&g,&t,&cfg.guidance));

    /* A proven miss may invalidate an established plan promptly, but a freshly issued
       child must observe the ordinary infeasible-supervision dwell. Otherwise each
       replacement can be rejected in the same tick and create a per-frame lineage storm. */
    g.entry_supervision_boundary_missed=true;
    assert(!entry_supervision_replan_due(&g,&t,&cfg.guidance));
    double invalid_dwell=fmax(5.0,cfg.guidance.prediction_interval*2.0);
    t.ut=g.entry_s_turn_plan.planned_ut+invalid_dwell+.01;
    assert(entry_supervision_replan_due(&g,&t,&cfg.guidance));

    /* A valid farther-out miss must not churn every supervisor refresh, but it must
       become replannable after one physical S-turn leg. Simulate a refresh just
       before the deadline: resetting entry_supervision_ut may not reset this clock. */
    g.entry_supervision_valid=true;g.entry_supervision_mode=ENTRY_SUPERVISION_PASS_THROUGH;
    double future_dwell=fmax(cfg.guidance.s_turn_minimum_leg_duration,cfg.guidance.prediction_interval);
    t.ut=g.entry_s_turn_plan.planned_ut+future_dwell-.10;
    g.entry_supervision_ut=t.ut;
    assert(!entry_supervision_replan_due(&g,&t,&cfg.guidance));
    t.ut=g.entry_s_turn_plan.planned_ut+future_dwell+.01;
    assert(entry_supervision_replan_due(&g,&t,&cfg.guidance));

    /* Ordinary invalid supervision without a proven boundary miss retains its
       anti-churn dwell. */
    g.entry_supervision_boundary_missed=false;g.entry_supervision_valid=false;
    g.entry_supervision_mode=ENTRY_SUPERVISION_INFEASIBLE;g.entry_supervision_ut=t.ut;
    t.ut=100.0+fmax(5.0,cfg.guidance.prediction_interval*2.0)+.01;
    assert(entry_supervision_replan_due(&g,&t,&cfg.guidance));
}

static void test_mm304_dynamic_pressure_overrides_persistent_plan(void) {
    LandingConfiguration cfg = landing_configuration_default();
    PlanetModel p = verifier_kerbin();
    AerodynamicModel aero = {.lift_to_drag = .6, .ballistic_coefficient = 700, .confidence = 1};
    Telemetry t = entry_telemetry(&cfg);
    t.true_air_speed = cfg.guidance.taem_force_handoff_speed;
    t.dynamic_pressure = cfg.vehicle.maximum_dynamic_pressure;
    t.g_force = 1;
    t.angle_of_attack = cfg.vehicle.entry_angle_of_attack;
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.entry_exec.initialized = true;
    g.entry_exec.phase = ENTRY_PHASE_CONSTANT_DRAG;
    g.entry_s_turn_plan = (EntryControlPlan){.valid = true, .planned_ut = t.ut,
        .segment_duration = 60, .target_bank = 5,
        .target_aoa = cfg.vehicle.entry_angle_of_attack,
        .target_heading = t.heading};
    g.entry_control_plan_valid = true;
    g.entry_supervision_valid = true;
    g.entry_supervision_ut = t.ut;
    /* No propagated state is available to replace the plan. The live safety
       envelope must still take effect, including against a cached correction. */
    g.entry_supervision_aoa_correction = -2;
    GuidanceResult r = entry_program_guidance(&g, &t, NULL, t.heading,
        &p, aero, &cfg, .1);
    assert(g.entry_control_aoa > cfg.vehicle.entry_angle_of_attack);
    assert(r.command.target_aoa > t.angle_of_attack);
    guidance_result_clear(&r);
}

static void test_stabilized_commands_obey_live_hard_bounds(void) {
    LandingConfiguration cfg = landing_configuration_default();
    Telemetry t = entry_telemetry(&cfg);
    t.dynamic_pressure = 5000;
    t.mach = 3;
    GuidanceMachine g;
    guidance_machine_init(&g);
    g.has_last_stabilized_phase = true;
    g.last_stabilized_phase = PHASE_ENTRY_ENERGY;
    g.pitch_limiter.has_value = g.roll_limiter.has_value = true;
    g.pitch_limiter.value = cfg.vehicle.maximum_angle_of_attack;
    g.pitch_limiter.rate = 2;
    g.roll_limiter.value = cfg.vehicle.maximum_bank_angle + 10;
    g.roll_limiter.rate = 1;
    GuidanceCommand c = atmospheric(&t, t.heading, 0, &cfg.vehicle, 0, false, PROFILE_ENTRY);
    c.has_target_aoa = true;
    c.target_aoa = cfg.vehicle.maximum_angle_of_attack - 1;
    GuidanceResult r = stabilized(&g, result_make(PHASE_ENTRY_ENERGY, c, "test", NULL),
        &t, &cfg.vehicle, &cfg.guidance, .1);
    assert(r.command.target_aoa <= cfg.vehicle.maximum_angle_of_attack);
    assert(fabs(r.command.target_roll) <= dynamic_bank_limit(&t, &cfg.vehicle));
    assert(g.pitch_limiter.value == r.command.target_aoa);
    assert(norm_signed_deg(g.roll_limiter.value) == r.command.target_roll);
    guidance_result_clear(&r);
}

static void test_taem_ownership_rebases_entry_preview_timing(void) {
    LandingConfiguration cfg = landing_configuration_default();
    PlanetModel p = verifier_kerbin();
    GuidanceMachine g;
    guidance_machine_init(&g);
    Telemetry t = entry_telemetry(&cfg);
    t.ut = 67540.8;
    t.true_air_speed = 1299.6;
    t.horizontal_speed = 1294.7;
    t.mean_altitude = 27198.0;
    t.radar_altitude = 25787.0;
    t.flight_path_angle = -5.0;
    t.roll = 38.4;
    t.runway_cross_track = 4220.0;

    g.terminal_candidate.valid = true;
    g.terminal_candidate.selected_ut = t.ut - 34.2;
    g.terminal_candidate.arrival_ut = t.ut + 9.8;
    g.terminal_prediction_valid = true;
    g.terminal_prediction_ut = t.ut;
    g.terminal_prediction_altitude = 26770.0;
    g.terminal_prediction_speed = 1287.0;
    g.terminal_prediction_time = 9.8;
    g.taem_interface_target.valid = true;
    g.terminal_energy_loss_accel_ema = 14.75;
    g.terminal_speed_loss_accel_ema = 12.0;

    terminal_force_acquisition(&g, &t, t.ground_track_heading, &p, &cfg);

    assert(g.terminal_region_entered);
    assert(g.terminal_test_capture_active);
    assert(!g.terminal_candidate.valid);
    assert(!g.terminal_prediction_valid);
    assert(isinf(g.terminal_prediction_ut) && g.terminal_prediction_ut < 0.0);
    assert(isnan(g.terminal_prediction_altitude));
    assert(isnan(g.terminal_prediction_speed));
    assert(g.terminal_prediction_time == 0.0);
    assert(g.taem_interface_target.valid);
    assert(fabs(g.terminal_energy_loss_accel_ema - 14.75) < 1e-9);
    assert(fabs(g.terminal_speed_loss_accel_ema - 12.0) < 1e-9);
}

int main(void) {
    test_mm304_dynamic_pressure_overrides_persistent_plan();
    test_stabilized_commands_obey_live_hard_bounds();
    test_reentry_continuation_initializer_is_shared_policy_seed();
    test_preentry_capture_uses_inertial_native_capture_until_airload();
    test_beta_runaway_gate_allows_corrective_yaw_braking_transient();
    test_normal_mm304_initialization_honors_preentry_load_gate();
    test_mm304_high_for_range_restores_vertical_capture_before_taem();
    test_mm304_drag_reference_reserves_terminal_geometry_not_raw_interface();
    test_mm304_unmeasured_stall_proxy_does_not_unload_thermal_alpha_at_recorded_1739_state();
    test_mm304_nominal_plan_and_execution_preserve_thermal_aoa();

    test_mm304_reversal_is_plan_milestone_not_crossrange_chase();
    test_mm304_fallback_admits_due_nonfinal_deadline_reversal();
    test_mm304_planned_range_is_diagnostic_not_direct_reversal_trigger();
    test_mm304_first_program_side_follows_taem_tangent_gate();
    test_controlled_coordinate_rate_helpers_reject_body_axis_aliases();
    test_mm304_leg_dwell_requires_continuous_loaded_bank_capture();

    test_mm304_routine_replan_may_pull_but_not_postpone_reversal();
    test_mm304_safety_replan_preserves_committed_reversal();
    test_mm304_persistent_plan_is_cleared_on_guidance_reset();
    test_executable_control_plan_lineage_is_guidance_owned();
    test_mm304_late_vertical_debt_preserves_entry_thermal_aoa();
    test_mm304_preentry_loaded_vertical_debt_activates_capture();
    test_mm304_bank_authority_acquisition_replans_once();
    test_mm304_vertical_capture_uses_soft_low_q_authority_and_measured_stall_only();
    test_mm304_live_bank_limit_matches_shared_predictor_authority_core();

    test_mm304_aoa_reference_does_not_overshoot_reversed_target();
    test_mm304_roll_reference_stops_when_bank_target_reverses();
    test_taem_ownership_rebases_entry_preview_timing();
    test_latched_taem_path_recovery_never_returns_to_entry();
    test_geometry_degraded_preview_cannot_replace_recent_clean_entry_inlet();
    test_latched_taem_target_is_immutable_after_handoff();

    test_v12_committed_spline_detects_vertical_closure_loss();
    test_proven_taem_boundary_miss_replans_without_infeasible_dwell();
    puts("Guidance integration contract tests passed.");
    return 0;
}
