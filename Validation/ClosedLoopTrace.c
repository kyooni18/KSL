/* Synchronous closed-loop guidance trace.
 *
 * Flies one ShuttleSim scenario with guidance_update_with_terminal_model() and
 * the direct flight-control law in a single thread (no wall-clock coupling),
 * printing every guidance command and the simulator state with exact hex
 * floating point.  Two builds of the landing library that behave identically
 * produce byte-identical output:
 *
 *   ClosedLoopTrace SCENARIO.ini final|hac|reentry [steps] [dt]
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "landing_api.h"
#include "flight_control.h"
#include "sim_telemetry.h"
#include "terminal_model.h"
#include "shuttlesim/sim.h"
#include "shuttlesim/scenario.h"
#include "shuttlesim/aero.h"
#include "shuttlesim/attitude.h"
#include "shuttlesim/world.h"

static AerodynamicModel envelope_at_mach(const AerodynamicEnvelope *e, double mach) {
    const double anchor[] = {.35, 1.05, 2.6, 6};
    double x = fmax(0, mach);
    if (x <= anchor[0]) return e->regimes[0];
    if (x >= anchor[3]) return e->regimes[3];
    for (int i = 0; i < 3; i++) {
        if (x <= anchor[i + 1]) {
            double f = (x - anchor[i]) / fmax(anchor[i + 1] - anchor[i], 1e-9);
            return (AerodynamicModel){
                e->regimes[i].lift_to_drag + (e->regimes[i + 1].lift_to_drag - e->regimes[i].lift_to_drag) * f,
                e->regimes[i].ballistic_coefficient + (e->regimes[i + 1].ballistic_coefficient - e->regimes[i].ballistic_coefficient) * f,
                e->regimes[i].confidence + (e->regimes[i + 1].confidence - e->regimes[i].confidence) * f};
        }
    }
    return e->regimes[3];
}

static void load_force_book(const char *path, VesselPhysicsModel *model) {
    FILE *f = fopen(path, "r");
    if (!f) return;
    char line[512];
    while (fgets(line, sizeof(line), f)) {
        if (strstr(line, "q_pa")) continue;
        double q, mach, aoa, lift_per_q, drag_per_q, support = 1;
        int got = sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf", &q, &mach, &aoa, &lift_per_q, &drag_per_q, &support);
        if (got < 5 || !isfinite(q) || q <= 1 || !isfinite(mach) || !isfinite(aoa) || !isfinite(lift_per_q) || !isfinite(drag_per_q)) continue;
        unsigned obs = (unsigned)fmax(1.0, round(got >= 6 ? support : 1.0));
        VesselAeroSample s = {
            .q = q, .mach = mach, .aoa = aoa, .beta = 0, .mass = 40252.91796875,
            .force_per_q = v3(fmax(0, drag_per_q), fmax(0, lift_per_q), 0),
            .gear = false, .brakes = false, .airbrakes = 0, .observations = obs,
            .trust = clampd(.72 + .035 * (double)obs, .72, 1.0), .last_observation_ut = 0};
        vessel_physics_import_sample(model, &s);
    }
    fclose(f);
}

static void print_command(int step, const GuidanceResult *r, const FlightControlOutput *o, const SimState *s) {
    const GuidanceCommand *c = &r->command;
    printf("%d phase=%d ut=%a pitch=%a hdg=%a roll=%a thr=%a aoa=%a/%d trim=%a sb=%a gear=%d brk=%d ab=%d prof=%d | "
           "fcs p=%a r=%a y=%a ws=%a | sim pos=%a,%a,%a vel=%a,%a,%a mass=%a aoa=%a bank=%a\n",
           step, r->phase, s->ut, c->target_pitch, c->target_heading, c->target_roll, c->target_throttle,
           c->target_aoa, c->has_target_aoa, c->pitch_trim_feedforward, c->speedbrake_fraction,
           c->gear, c->brakes, c->airbrakes, c->control_profile, o->pitch, o->roll, o->yaw, o->wheel_steering,
           s->position_i_m.x, s->position_i_m.y, s->position_i_m.z, s->velocity_i_mps.x, s->velocity_i_mps.y,
           s->velocity_i_mps.z, s->mass_kg, s->attitude.aoa_rad, s->attitude.bank_rad);
}

int main(int argc, char **argv) {
    if (argc < 3) return 2;
    setvbuf(stdout, NULL, _IOLBF, 0);
    const char *scenario_path = argv[1], *mode = argv[2];
    int steps = argc > 3 ? atoi(argv[3]) : 500;
    double dt = argc > 4 ? atof(argv[4]) : 0.1;
    Scenario sc;
    if (!scenario_load(scenario_path, &sc)) { fprintf(stderr, "scenario\n"); return 1; }
    Simulation sim;
    sim_init(&sim, &sc);
    char atm[512], aero_path[512], book[512], att[512], prior[512];
    shuttle_sim_model_path(NULL, SHUTTLE_SIM_MODEL_ATMOSPHERE, atm, sizeof(atm));
    world_load_atmosphere_csv(&sim.world, atm);
    shuttle_sim_model_path(NULL, SHUTTLE_SIM_MODEL_AERO, aero_path, sizeof(aero_path));
    aero_load_csv(&sim.aero, aero_path);
    shuttle_sim_model_path(NULL, SHUTTLE_SIM_MODEL_FORCE_BOOK, book, sizeof(book));
    aero_load_book_csv(&sim.aero, book);
    shuttle_sim_model_path(NULL, SHUTTLE_SIM_MODEL_ATTITUDE, att, sizeof(att));
    attitude_load_ini(&sim.state.attitude, att);
    sim.state.aero = aero_compute(&sim.world, &sim.aero, sim.state.position_i_m, sim.state.velocity_i_mps,
                                  sim.state.ut, sim.state.mass_kg, sim.state.attitude.aoa_rad, sim.state.attitude.bank_rad);

    LandingConfiguration cfg = landing_configuration_default();
    landing_configuration_normalize(&cfg);
    cfg.site.latitude = sc.runway_latitude_deg;
    cfg.site.longitude = sc.runway_longitude_deg;
    cfg.site.altitude = sc.runway_elevation_m;
    cfg.site.runway_heading = sc.runway_heading_deg;
    cfg.site.runway_length = sc.runway_length_m;
    cfg.site.runway_width = sc.runway_width_m;

    PlanetModel planet;
    char err[256];
    if (!shuttle_sim_load_planet(atm, &planet, err, sizeof(err))) { fprintf(stderr, "planet %s\n", err); return 1; }
    VesselPhysicsModel *physics = calloc(1, sizeof(*physics));
    vessel_physics_init(physics);
    shuttle_sim_model_path(NULL, SHUTTLE_SIM_MODEL_CERTIFIED_PRIOR, prior, sizeof(prior));
    load_force_book(prior, physics);
    AerodynamicEnvelope env;
    AerodynamicModel plan_aero;
    if (!vessel_physics_derive_envelope(physics, &cfg.vehicle, &env, &plan_aero)) return 1;

    TerminalModel *model = calloc(1, sizeof(*model));
    TerminalModelSourceFiles files = {.atmosphere_csv = atm, .aero_csv = aero_path, .aero_book_csv = book, .attitude_ini = att};
    char reason[256] = {0};
    bool have_model = terminal_model_capture(model, &files, &cfg, 1, reason, sizeof(reason));
    printf("terminal_model=%d\n", have_model);

    char json[8192];
    Telemetry prev = {0};
    bool has_prev = false;
    sim_build_telemetry_json(&sim, 50.0, json, sizeof(json));
    Telemetry t0;
    VehicleState v0;
    if (!shuttle_sim_decode_telemetry(json, &planet, NULL, &t0, &v0, err, sizeof(err))) return 1;
    shuttle_sim_prepare_guidance_telemetry(&t0, &cfg, &planet);
    AerodynamicModel aero0 = envelope_at_mach(&env, t0.mach);
    t0.estimated_lift_to_drag = aero0.lift_to_drag;
    t0.estimated_ballistic_coefficient = aero0.ballistic_coefficient;
    t0.aerodynamic_confidence = aero0.confidence;

    GuidanceMachine *g = calloc(1, sizeof(*g));
    guidance_machine_init(g);
    guidance_set_engaged(g, true);
    double course0 = surface_course(v0.position, v0.velocity, planet_rotation_vector(&planet), planet.north_axis, t0.heading);
    char msg[256] = {0};
    TrajectoryCalibrationModel cal = {.density_scale = 1, .drag_scale = 1, .lift_scale = 1, .bank_effectiveness = 1,
        .speed_of_sound = 340, .speed_of_sound_scale = 1, .stress_drag_scale = 1, .stress_lift_scale = 1, .confidence = .05};
    bool ok = true;
    DeorbitPlan plan;
    bool have_plan = false;
    if (!strcmp(mode, "final")) ok = guidance_begin_final_test(g, &t0, course0, &planet, aero0, &cfg, msg, sizeof(msg));
    else if (!strcmp(mode, "hac")) ok = guidance_begin_hac_test(g, &t0, course0, &planet, aero0, &cfg, msg, sizeof(msg));
    else {
        /* Offline downstream-controller diagnostic, as the controller builds it
           for a simulator reentry continuation. */
        memset(&plan, 0, sizeof(plan));
        trajectory_init(&plan.trajectory);
        plan.created_ut = t0.ut;
        plan.burn_ut = t0.ut;
        plan.nominal_capture_achieved = plan.robustness_qualified = plan.target_capture_achieved = true;
        plan.execution_qualified = true;
        plan.live_cutoff_capture_qualified = plan.achieved_state_verified = plan.achieved_state_capture_qualified = true;
        plan.confidence = 1;
        have_plan = true;
        guidance_initialize_reentry_continuation(g, &t0, &planet, &cfg, 0, false, &env, &cal);
    }
    printf("begin=%d %s\n", ok, msg);
    if (!ok) return 0;

    FlightControlState fcs;
    flight_control_init(&fcs, dt);
    for (int step = 0; step < steps && !sim.state.on_ground; ++step) {
        sim_build_telemetry_json(&sim, 50.0, json, sizeof(json));
        Telemetry t;
        VehicleState vs;
        if (!shuttle_sim_decode_telemetry(json, &planet, has_prev ? &prev : NULL, &t, &vs, err, sizeof(err))) return 1;
        shuttle_sim_prepare_guidance_telemetry(&t, &cfg, &planet);
        AerodynamicModel aero = envelope_at_mach(&env, t.mach);
        t.estimated_lift_to_drag = aero.lift_to_drag;
        t.estimated_ballistic_coefficient = aero.ballistic_coefficient;
        t.aerodynamic_confidence = aero.confidence;
        GuidanceResult res = guidance_update_with_terminal_model(g, &t, &vs, have_plan ? &plan : NULL, &planet, aero, &cfg, have_model ? model : NULL);
        FlightControlOutput fco;
        memset(&fco, 0, sizeof(fco));
        bool direct = !strcmp(mode, "final");
        if (direct && !flight_control_step(&fcs, &t, &res.command, dt, &fco)) { printf("fcs-fail %d\n", step); break; }
        print_command(step, &res, &fco, &sim.state);
        SimCommand cmd;
        memset(&cmd, 0, sizeof(cmd));
        if (!direct) {
            /* Ideal AoA/bank servo, as the simulator transport applies guidance. */
            const GuidanceCommand *gc = &res.command;
            cmd.has_attitude = true;
            cmd.aoa_deg = gc->has_target_aoa ? gc->target_aoa : gc->target_pitch;
            cmd.bank_deg = gc->target_roll;
            cmd.has_speedbrake = true; cmd.speedbrake = gc->speedbrake_fraction;
            cmd.has_gear = true; cmd.gear_down = gc->gear;
            cmd.has_brakes = true; cmd.brakes = gc->brakes;
            cmd.has_airbrakes = true; cmd.airbrakes = gc->airbrakes;
            cmd.has_wheel_steering = true; cmd.wheel_steering = gc->wheel_steering;
            sim_apply_command(&sim, &cmd);
            sim_step(&sim, dt);
            prev = t; has_prev = true;
            guidance_result_clear(&res);
            continue;
        }
        cmd.has_gear = true; cmd.gear_down = fco.gear;
        cmd.has_brakes = true; cmd.brakes = fco.brakes;
        cmd.has_airbrakes = true; cmd.airbrakes = fco.airbrakes;
        cmd.has_wheel_steering = true; cmd.wheel_steering = fco.wheel_steering;
        cmd.has_inputs = true; cmd.pitch_input = fco.pitch; cmd.roll_input = fco.roll; cmd.yaw_input = fco.yaw;
        sim_apply_command(&sim, &cmd);
        sim_step(&sim, dt);
        prev = t;
        has_prev = true;
        guidance_result_clear(&res);
    }
    printf("done on_ground=%d touchdown=%d ut=%a\n", sim.state.on_ground, sim.state.touchdown_seen, sim.state.ut);
    return 0;
}
