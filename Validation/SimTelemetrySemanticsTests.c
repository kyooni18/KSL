#include "../CLanding/sim_telemetry.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifdef SIM_TELEMETRY_TEST_STANDALONE
/* The adapter depends only on the zero/default contract of telemetry_init.
   This stand-alone test definition avoids pulling the entire flight executive
   into a transport-boundary unit test. */
void telemetry_init(Telemetry *t) {
    memset(t, 0, sizeof(*t));
    t->speed_of_sound = 340.0;
    t->navball_speed_mode = SPEED_UNCHANGED;
    t->trajectory_density_scale = 1.0;
    t->trajectory_drag_scale = 1.0;
    t->trajectory_lift_scale = 1.0;
    t->bank_effectiveness = 1.0;
}
#endif

static const char *packet(double ut, double fpa, double aoa, double bank,
                          double heading, double qw, double qx,
                          double qy, double qz, bool publish_response) {
    static char json[4096];
    const char *response = publish_response
        ? "\"aoa_rate_deg_s\":2.25,\"bank_rate_deg_s\":-3.5,"
          "\"pitch_wn_s_inv\":1.4,\"pitch_zeta\":0.9,"
          "\"roll_wn_s_inv\":1.8,\"roll_zeta\":0.85,"
          "\"max_pitch_rate_deg_s\":8,\"max_roll_rate_deg_s\":18,"
          "\"max_pitch_accel_deg_s2\":5,\"max_roll_accel_deg_s2\":15,"
        : "";
    snprintf(json, sizeof(json),
        "{\"schema\":1,\"type\":\"telemetry\",\"source\":\"sim\","
        "\"ut\":%.9g,"
        "\"position\":{\"x\":700000,\"y\":0,\"z\":0,\"lat_deg\":0,\"lon_deg\":0,\"altitude_m\":100000},"
        "\"velocity\":{\"x_mps\":0,\"y_mps\":2200,\"z_mps\":0,\"surface_mps\":2200,\"air_mps\":2200,\"horizontal_mps\":2200,\"vertical_mps\":0,\"flight_path_angle_deg\":%.9g},"
        "\"attitude\":{\"aoa_deg\":%.9g,\"bank_deg\":%.9g,\"heading_deg\":%.9g,%s\"q_w\":%.9g,\"q_x\":%.9g,\"q_y\":%.9g,\"q_z\":%.9g},"
        "\"atmosphere\":{\"density_kg_m3\":0.01,\"pressure_pa\":1000,\"speed_of_sound_mps\":300},"
        "\"aero\":{\"mach\":7.3,\"q_pa\":20000,\"lift_n\":100000,\"drag_n\":25000},"
        "\"vehicle\":{\"mass_kg\":40000},"
        "\"runway\":{\"along_m\":-100000,\"cross_m\":5000},"
        "\"ground\":{\"gear_down\":false,\"brakes\":false,\"on_ground\":false}}",
        ut, fpa, aoa, bank, heading, response, qw, qx, qy, qz);
    return json;
}

static PlanetModel kerbin(void) {
    PlanetModel p;
    memset(&p, 0, sizeof(p));
    p.radius = 600000.0;
    p.gravitational_parameter = 3.5316e12;
    return p;
}

static void assert_close(double actual, double expected, double tolerance) {
    assert(isfinite(actual));
    if (fabs(actual - expected) > tolerance)
        fprintf(stderr, "assert_close: actual=%.17g expected=%.17g tolerance=%.17g\n",
                actual, expected, tolerance);
    assert(fabs(actual - expected) <= tolerance);
}

int main(void) {
    PlanetModel planet = kerbin();
    LandingConfiguration configuration = {0};
    configuration.site.latitude = 0.0;
    configuration.site.longitude = 1.0;
    configuration.site.altitude = 0.0;
    configuration.site.runway_heading = 90.0;
    configuration.vehicle.estimated_lift_to_drag = 0.4;
    configuration.vehicle.estimated_ballistic_coefficient = 700.0;
    Telemetry first, second, duplicate, fallback;
    VehicleState first_state, second_state, duplicate_state, fallback_state;
    char error[256];

    assert(shuttle_sim_decode_telemetry(
        packet(100.0, -5.0, 10.0, 179.0, 359.0,
               0.5, 0.1, 0.2, 0.3, true),
        &planet, NULL, &first, &first_state, error, sizeof(error)));

    /* Simulator JSON is w,x,y,z; the production ABI is x,y,z,w. */
    assert(first.has_attitude_quaternion);
    assert_close(first.attitude_quaternion[0], 0.1, 1e-12);
    assert_close(first.attitude_quaternion[1], 0.2, 1e-12);
    assert_close(first.attitude_quaternion[2], 0.3, 1e-12);
    assert_close(first.attitude_quaternion[3], 0.5, 1e-12);

    /* A complete simulator frame is certified for physics use.  This is
       distinct from the attitude-response model below: both contracts must
       be explicit before downstream guidance treats the sample as live-like
       physics input. */
    assert(first.physics_sample_valid);
    assert_close(first.physics_confidence, 1.0, 1e-12);
    assert_close(first.aerodynamic_confidence, 1.0, 1e-12);

    /* Simulator state rates are valid on the first packet without differencing. */
    assert(first.has_angle_of_attack_rate);
    assert_close(first.angle_of_attack_rate, 2.25, 1e-12);
    assert_close(first.roll_rate, -3.5, 1e-12);
    assert(!first.has_course_rate);

    /* Response limits belong to the response model, not physics-authority. */
    assert(first.attitude_response.pitch_valid);
    assert(first.attitude_response.roll_valid);
    assert_close(first.attitude_response.pitch_natural_frequency_s_inv, 1.4, 1e-12);
    assert_close(first.attitude_response.roll_natural_frequency_s_inv, 1.8, 1e-12);
    assert_close(first.attitude_response.maximum_pitch_rate_deg_s, 8.0, 1e-12);
    assert_close(first.attitude_response.maximum_roll_rate_deg_s, 18.0, 1e-12);
    assert_close(first.attitude_response.maximum_pitch_accel_deg_s2, 5.0, 1e-12);
    assert_close(first.attitude_response.maximum_roll_accel_deg_s2, 15.0, 1e-12);
    assert_close(first.physics_authority_confidence[0], 0.0, 0.0);
    assert_close(first.physics_authority_confidence[1], 0.0, 0.0);
    assert_close(first.physics_authority_confidence[2], 0.0, 0.0);

    assert(!first.has_body_pitch_rate);
    assert(!first.has_body_roll_rate);
    assert(!first.has_body_yaw_rate);

    /* Configuration/site-dependent guidance metadata is prepared by the same
       adapter used by both the offline expert and UDP runner. */
    shuttle_sim_prepare_guidance_telemetry(&first, &configuration, &planet);
    assert_close(first.estimated_lift_to_drag, 4.0, 1e-12);
    assert_close(first.estimated_ballistic_coefficient, 700.0, 1e-12);
    assert_close(first.physics_authority[0], 5.0, 1e-12);
    assert_close(first.physics_authority[1], 15.0, 1e-12);
    assert_close(first.physics_authority_confidence[0], 1.0, 1e-12);
    assert_close(first.physics_authority_confidence[1], 1.0, 1e-12);
    assert(isfinite(first.range_to_site));
    assert(first.vessel_name[0] != '\0');

    assert(shuttle_sim_decode_telemetry(
        packet(102.0, -4.0, 12.0, -179.0, 1.0,
               0.7, 0.4, 0.5, 0.6, true),
        &planet, &first, &second, &second_state, error, sizeof(error)));

    /* Direct simulator response rates win; course/pitch still use geometry. */
    assert_close(second.pitch_rate, 1.5, 1e-12);
    assert_close(second.roll_rate, -3.5, 1e-12);
    assert_close(second.heading_rate, 1.0, 1e-12);
    assert_close(second.angle_of_attack_rate, 2.25, 1e-12);
    assert(second.has_course_rate);
    assert_close(second.course_rate, 1.0, 1e-12);
    assert(!second.has_body_pitch_rate);
    assert(!second.has_body_roll_rate);
    assert(!second.has_body_yaw_rate);

    /* Missing response fields fail closed.  A duplicate timestamp cannot be
       turned into synthetic rate evidence. */
    assert(shuttle_sim_decode_telemetry(
        packet(100.0, -3.0, 20.0, -170.0, 10.0,
               1.0, 0.0, 0.0, 0.0, false),
        &planet, &first, &duplicate, &duplicate_state, error, sizeof(error)));
    assert(!duplicate.has_angle_of_attack_rate);
    assert(!duplicate.has_course_rate);
    assert(!duplicate.attitude_response.pitch_valid);
    assert(!duplicate.attitude_response.roll_valid);
    assert(!duplicate.has_body_pitch_rate);
    assert(!duplicate.has_body_roll_rate);
    assert(!duplicate.has_body_yaw_rate);

    /* Legacy packets without explicit coordinate rates can still be
       differentiated when simulator time advances; circular angles wrap. */
    assert(shuttle_sim_decode_telemetry(
        packet(104.0, -3.0, 14.0, -177.0, 3.0,
               1.0, 0.0, 0.0, 0.0, false),
        &planet, &second, &fallback, &fallback_state, error, sizeof(error)));
    assert(fallback.has_angle_of_attack_rate);
    assert_close(fallback.angle_of_attack_rate, 1.0, 1e-12);
    assert_close(fallback.roll_rate, 1.0, 1e-12);
    assert_close(fallback.heading_rate, 1.0, 1e-12);
    assert_close(fallback.course_rate, 1.0, 1e-12);
    assert(!fallback.has_body_pitch_rate);
    assert(!fallback.has_body_roll_rate);
    assert(!fallback.has_body_yaw_rate);

    puts("SimTelemetrySemanticsTests: PASS");
    return 0;
}
