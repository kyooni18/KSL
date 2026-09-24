#include "sim_telemetry.h"
#include "json.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void set_error(char *out, size_t size, const char *message) {
    if (!out || size == 0) return;
    snprintf(out, size, "%s", message ? message : "");
}

bool shuttle_sim_load_planet(const char *path, PlanetModel *planet,
                             char *error, size_t error_size) {
    if (!path || !planet) {
        set_error(error, error_size, "Simulator atmosphere path and destination are required");
        return false;
    }
    FILE *file = fopen(path, "r");
    if (!file) {
        set_error(error, error_size, "Could not open simulator atmosphere profile");
        return false;
    }
    PlanetModel model = {0};
    snprintf(model.name, sizeof(model.name), "Kerbin");
    /* decision-literal: configuration-vehicle-parameter | Kerbin world constants match ShuttleSim world_seed_kerbin; these describe the simulated body, not flight policy. */
    model.radius = 600000.0;
    /* decision-literal: configuration-vehicle-parameter | Kerbin gravitational parameter in m^3/s^2 from the simulator world definition. */
    model.gravitational_parameter = 3.5316e12;
    /* decision-literal: configuration-vehicle-parameter | Kerbin sidereal rotation period in seconds from the simulator world definition. */
    model.rotational_speed = 2.0 * LANDER_PI / 21549.425;
    /* decision-literal: physical-law-constant | Specific heat ratio for the simulator's ideal diatomic atmosphere. */
    model.atmosphere_adiabatic_index = 1.4;
    model.north_axis = (Vector3){0, 0, 1};
    model.prime_meridian_at_epoch = (Vector3){0, 1, 0};
    char line[512];
    bool valid = true, found_top = false;
    while (fgets(line, sizeof(line), file)) {
        if (line[0] == '#' || strstr(line, "altitude_m")) continue;
        double h, density, pressure, temperature, sound;
        char trailing;
        /* decision-literal: protocol-domain-requirement | The fitted atmosphere CSV has exactly five numeric columns. */
        if (sscanf(line, "%lf,%lf,%lf,%lf,%lf %c", &h, &density, &pressure,
                   &temperature, &sound, &trailing) != 5 ||
            !isfinite(h) || h < 0.0 || !isfinite(density) || density < 0.0 ||
            !isfinite(pressure) || pressure < 0.0 ||
            !isfinite(temperature) || temperature < 0.0 ||
            !isfinite(sound) || sound < 0.0 ||
            (density > 0.0 && (pressure <= 0.0 || temperature <= 0.0 || sound <= 0.0)) ||
            (density == 0.0 && pressure != 0.0) ||
            model.atmosphere_sample_count == LANDER_ATMOSPHERE_SAMPLE_MAX) {
            valid = false;
            break;
        }
        size_t i = model.atmosphere_sample_count;
        if ((i && h <= model.atmosphere_altitude[i-1]) ||
            (found_top && (density > 0.0 || pressure > 0.0))) {
            valid = false;
            break;
        }
        if (i && model.atmosphere_density[i-1] > 0.0 && density == 0.0) {
            model.atmosphere_depth = h;
            found_top = true;
        }
        model.atmosphere_altitude[i] = h;
        model.atmosphere_density[i] = density;
        model.atmosphere_pressure[i] = pressure;
        model.atmosphere_sample_count++;
    }
    valid = valid && !ferror(file);
    fclose(file);
    /* decision-literal: mathematical-numerical-requirement | Interpolation requires at least two strictly increasing altitude samples and an explicit vacuum boundary. */
    if (!valid || model.atmosphere_sample_count < 2 || !found_top) {
        set_error(error, error_size, "Invalid, incomplete, or oversized simulator atmosphere profile");
        return false;
    }
    model.surface_density = model.atmosphere_density[0];
    *planet = model;
    set_error(error, error_size, "");
    return true;
}

static int object_value(const JsonDoc *doc, int parent, const char *key) {
    return parent >= 0 ? json_object_get(doc, parent, key) : -1;
}

static double number_value(const JsonDoc *doc, int parent, const char *key,
                           double fallback) {
    int token = object_value(doc, parent, key);
    return token >= 0 ? json_number(doc, token, fallback) : fallback;
}

static bool bool_value(const JsonDoc *doc, int parent, const char *key,
                       bool fallback) {
    int token = object_value(doc, parent, key);
    return token >= 0 ? json_boolean(doc, token, fallback) : fallback;
}

static bool object_present(const JsonDoc *doc, int index) {
    return doc && index >= 0 && index < doc->count &&
        doc->tokens[index].type == JSON_OBJECT;
}

static bool required_number(const JsonDoc *doc, int parent, const char *key) {
    int token = object_value(doc, parent, key);
    if (token < 0 || token >= doc->count ||
        doc->tokens[token].type != JSON_PRIMITIVE)
        return false;
    return isfinite(json_number(doc, token, NAN));
}

static double signed_degrees(double angle) {
    if (!isfinite(angle)) return angle;
    double wrapped = fmod(angle + 180.0, 360.0);
    if (wrapped < 0.0) wrapped += 360.0;
    return wrapped - 180.0;
}

void shuttle_sim_prepare_guidance_telemetry(
    Telemetry *t, const LandingConfiguration *cfg, const PlanetModel *planet) {
    if (!t || !cfg || !planet) return;

    bool aero_finite = isfinite(t->dynamic_pressure) &&
        t->dynamic_pressure >= 0.0 && isfinite(t->lift_force) &&
        isfinite(t->drag_force);
    t->physics_confidence = t->physics_sample_valid ? 1.0 : 0.0;
    t->aerodynamic_confidence = aero_finite ? 1.0 : 0.0;
    t->estimated_lift_to_drag = t->drag_force > 1.0
        ? t->lift_force / t->drag_force
        : cfg->vehicle.estimated_lift_to_drag;
    t->estimated_ballistic_coefficient =
        cfg->vehicle.estimated_ballistic_coefficient;
    t->physics_authority[0] = t->attitude_response.pitch_valid
        ? t->attitude_response.maximum_pitch_accel_deg_s2 : 0.0;
    t->physics_authority[1] = t->attitude_response.roll_valid
        ? t->attitude_response.maximum_roll_accel_deg_s2 : 0.0;
    t->physics_authority[2] = 0.0;
    t->physics_authority_confidence[0] =
        t->attitude_response.pitch_valid ? 1.0 : 0.0;
    t->physics_authority_confidence[1] =
        t->attitude_response.roll_valid ? 1.0 : 0.0;
    t->physics_authority_confidence[2] = 0.0;

    GeoPoint here = {t->latitude, t->longitude, t->mean_altitude};
    GeoPoint site = {cfg->site.latitude, cfg->site.longitude,
                     cfg->site.altitude};
    t->range_to_site = great_circle_distance(here, site, planet->radius);
    t->bearing_to_site = initial_bearing(here, site);
    t->heading_error = signed_degrees(cfg->site.runway_heading - t->heading);
    t->course_to_site_error =
        signed_degrees(t->bearing_to_site - t->ground_track_heading);
    snprintf(t->vessel_name, sizeof(t->vessel_name), "STS-N Sim");
}

static bool attitude_axis_response_valid(double natural_frequency,
                                         double damping_ratio,
                                         double maximum_rate,
                                         double maximum_acceleration) {
    return isfinite(natural_frequency) && natural_frequency > 0.0 &&
        isfinite(damping_ratio) && damping_ratio >= 0.0 &&
        isfinite(maximum_rate) && maximum_rate > 0.0 &&
        isfinite(maximum_acceleration) && maximum_acceleration > 0.0;
}

static double vector_dot(Vector3 a, Vector3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static double vector_magnitude(Vector3 v) {
    return sqrt(vector_dot(v, v));
}

static Vector3 vector_cross(Vector3 a, Vector3 b) {
    return (Vector3){a.y * b.z - a.z * b.y,
                     a.z * b.x - a.x * b.z,
                     a.x * b.y - a.y * b.x};
}

static void fill_orbit_fields(const PlanetModel *planet,
                              const VehicleState *state,
                              Telemetry *telemetry) {
    if (!planet || !state || !telemetry) return;
    Vector3 r = state->position;
    Vector3 v = state->velocity;
    double radius = vector_magnitude(r);
    double speed_squared = vector_dot(v, v);
    double mu = planet->gravitational_parameter;
    if (!isfinite(radius) || radius <= 0.0 ||
        !isfinite(speed_squared) || speed_squared < 0.0 ||
        !isfinite(mu) || mu <= 0.0) return;

    double kinetic_specific_energy = 0.5 * speed_squared;
    double potential_scale = mu / radius;
    double energy = kinetic_specific_energy - potential_scale;
    /* Semimajor axis is singular for a parabolic orbit.  Compare the residual
       specific energy against the scale of the two terms that formed it, so
       the numerical boundary is dimensionally meaningful rather than an
       absolute epsilon in m^2/s^2. */
    double energy_scale = kinetic_specific_energy + potential_scale;
    if (!isfinite(energy) ||
        fabs(energy) <= DBL_EPSILON * fmax(energy_scale, DBL_MIN)) return;
    double semimajor = -mu / (2.0 * energy);
    Vector3 h = vector_cross(r, v);
    double h_squared = vector_dot(h, h);
    double eccentricity_squared = fmax(
        0.0, 1.0 + 2.0 * energy * h_squared / (mu * mu));
    double eccentricity = sqrt(eccentricity_squared);
    telemetry->periapsis_altitude =
        semimajor * (1.0 - eccentricity) - planet->radius;
    telemetry->apoapsis_altitude =
        semimajor * (1.0 + eccentricity) - planet->radius;
    if (semimajor > 0.0)
        telemetry->orbit_period = 2.0 * LANDER_PI *
            sqrt(semimajor * semimajor * semimajor / mu);
}

bool shuttle_sim_decode_telemetry(const char *packet,
                                  const PlanetModel *planet,
                                  const Telemetry *previous,
                                  Telemetry *t,
                                  VehicleState *state,
                                  char *error,
                                  size_t error_size) {
    if (!packet || !planet || !t || !state) {
        set_error(error, error_size, "Invalid ShuttleSim telemetry decode arguments");
        return false;
    }

    JsonToken tokens[512];
    JsonDoc doc;
    if (json_parse(packet, tokens, 512, &doc) < 1 ||
        doc.tokens[0].type != JSON_OBJECT) {
        set_error(error, error_size, "Invalid ShuttleSim telemetry JSON");
        return false;
    }

    char source[32] = {0};
    int source_token = object_value(&doc, 0, "source");
    if (source_token >= 0)
        json_string(&doc, source_token, source, sizeof(source));
    if (strcmp(source, "sim") != 0) {
        if (error && error_size)
            snprintf(error, error_size,
                     "Unexpected simulator telemetry source '%s'", source);
        return false;
    }

    telemetry_init(t);
    memset(state, 0, sizeof(*state));

    /*
     * ShuttleSim publishes the instantaneous whole-vehicle force state, but it
     * does not publish passive-calibration products.  telemetry_init() uses
     * zero-filled storage for generic transport compatibility; zero is a real
     * AoA, though, and terminal guidance treats a finite calibrated best-glide
     * value as authoritative.  Preserve the distinction between "0 deg" and
     * "not measured" here instead of silently commanding wings-level flight.
     */
    t->calibrated_best_glide_angle_of_attack = NAN;
    t->calibrated_stall_speed = NAN;

    int position = object_value(&doc, 0, "position");
    int velocity = object_value(&doc, 0, "velocity");
    int attitude = object_value(&doc, 0, "attitude");
    int atmosphere = object_value(&doc, 0, "atmosphere");
    int aero = object_value(&doc, 0, "aero");
    int vehicle = object_value(&doc, 0, "vehicle");
    int runway = object_value(&doc, 0, "runway");
    int ground = object_value(&doc, 0, "ground");

    bool required_objects = object_present(&doc, position) &&
        object_present(&doc, velocity) && object_present(&doc, attitude) &&
        object_present(&doc, atmosphere) && object_present(&doc, aero) &&
        object_present(&doc, vehicle) && object_present(&doc, runway) &&
        object_present(&doc, ground);
    bool required_numeric =
        required_number(&doc, 0, "ut") &&
        required_number(&doc, position, "x") &&
        required_number(&doc, position, "y") &&
        required_number(&doc, position, "z") &&
        required_number(&doc, position, "lat_deg") &&
        required_number(&doc, position, "lon_deg") &&
        required_number(&doc, position, "altitude_m") &&
        required_number(&doc, velocity, "x_mps") &&
        required_number(&doc, velocity, "y_mps") &&
        required_number(&doc, velocity, "z_mps") &&
        required_number(&doc, velocity, "surface_mps") &&
        required_number(&doc, velocity, "air_mps") &&
        required_number(&doc, velocity, "horizontal_mps") &&
        required_number(&doc, velocity, "vertical_mps") &&
        required_number(&doc, velocity, "flight_path_angle_deg") &&
        required_number(&doc, attitude, "aoa_deg") &&
        required_number(&doc, attitude, "bank_deg") &&
        required_number(&doc, attitude, "heading_deg") &&
        required_number(&doc, attitude, "q_w") &&
        required_number(&doc, attitude, "q_x") &&
        required_number(&doc, attitude, "q_y") &&
        required_number(&doc, attitude, "q_z") &&
        required_number(&doc, atmosphere, "density_kg_m3") &&
        required_number(&doc, atmosphere, "pressure_pa") &&
        required_number(&doc, atmosphere, "speed_of_sound_mps") &&
        required_number(&doc, aero, "mach") &&
        required_number(&doc, aero, "q_pa") &&
        required_number(&doc, aero, "lift_n") &&
        required_number(&doc, aero, "drag_n") &&
        required_number(&doc, vehicle, "mass_kg") &&
        required_number(&doc, runway, "along_m") &&
        required_number(&doc, runway, "cross_m");

    t->ut = number_value(&doc, 0, "ut", 0.0);
    state->ut = t->ut;
    t->latitude = number_value(&doc, position, "lat_deg", 0.0);
    t->longitude = number_value(&doc, position, "lon_deg", 0.0);
    t->mean_altitude = number_value(&doc, position, "altitude_m", 0.0);
    /* ShuttleSim publishes height above the runway reference plane explicitly.
       Use it for radar-altitude-dependent flare/gear/contact gates; mean altitude
       is only a compatibility fallback for legacy simulator packets. */
    t->radar_altitude = number_value(&doc, runway, "vertical_m", t->mean_altitude);
    state->position = (Vector3){
        number_value(&doc, position, "x", 0.0),
        number_value(&doc, position, "y", 0.0),
        number_value(&doc, position, "z", 0.0)};
    state->velocity = (Vector3){
        number_value(&doc, velocity, "x_mps", 0.0),
        number_value(&doc, velocity, "y_mps", 0.0),
        number_value(&doc, velocity, "z_mps", 0.0)};

    t->surface_speed = number_value(&doc, velocity, "surface_mps", 0.0);
    t->true_air_speed = number_value(&doc, velocity, "air_mps", t->surface_speed);
    t->horizontal_speed = number_value(&doc, velocity, "horizontal_mps", 0.0);
    t->vertical_speed = number_value(&doc, velocity, "vertical_mps", 0.0);
    t->flight_path_angle =
        number_value(&doc, velocity, "flight_path_angle_deg", 0.0);

    t->angle_of_attack = number_value(&doc, attitude, "aoa_deg", 0.0);
    t->roll = number_value(&doc, attitude, "bank_deg", 0.0);
    /* ShuttleSim's heading is the horizontal air/surface velocity azimuth. */
    t->heading = number_value(&doc, attitude, "heading_deg", 0.0);
    t->ground_track_heading = t->heading;
    t->pitch = t->flight_path_angle + t->angle_of_attack;
    t->sideslip = 0.0;

    double published_aoa_rate =
        number_value(&doc, attitude, "aoa_rate_deg_s", NAN);
    double published_bank_rate =
        number_value(&doc, attitude, "bank_rate_deg_s", NAN);
    bool has_published_aoa_rate = isfinite(published_aoa_rate);
    bool has_published_bank_rate = isfinite(published_bank_rate);
    if (has_published_aoa_rate) {
        t->angle_of_attack_rate = published_aoa_rate;
        t->has_angle_of_attack_rate = true;
    }
    if (has_published_bank_rate)
        t->roll_rate = published_bank_rate;

    AttitudeResponseModel *response = &t->attitude_response;
    response->pitch_natural_frequency_s_inv =
        number_value(&doc, attitude, "pitch_wn_s_inv", NAN);
    response->pitch_damping_ratio =
        number_value(&doc, attitude, "pitch_zeta", NAN);
    response->roll_natural_frequency_s_inv =
        number_value(&doc, attitude, "roll_wn_s_inv", NAN);
    response->roll_damping_ratio =
        number_value(&doc, attitude, "roll_zeta", NAN);
    response->maximum_pitch_rate_deg_s =
        number_value(&doc, attitude, "max_pitch_rate_deg_s", NAN);
    response->maximum_roll_rate_deg_s =
        number_value(&doc, attitude, "max_roll_rate_deg_s", NAN);
    response->maximum_pitch_accel_deg_s2 =
        number_value(&doc, attitude, "max_pitch_accel_deg_s2", NAN);
    response->maximum_roll_accel_deg_s2 =
        number_value(&doc, attitude, "max_roll_accel_deg_s2", NAN);
    response->pitch_valid = attitude_axis_response_valid(
        response->pitch_natural_frequency_s_inv,
        response->pitch_damping_ratio,
        response->maximum_pitch_rate_deg_s,
        response->maximum_pitch_accel_deg_s2);
    response->roll_valid = attitude_axis_response_valid(
        response->roll_natural_frequency_s_inv,
        response->roll_damping_ratio,
        response->maximum_roll_rate_deg_s,
        response->maximum_roll_accel_deg_s2);

    /* Production Telemetry stores quaternion components as x,y,z,w.  The
       simulator's JSON is w,x,y,z, so normalize the representation here. */
    t->has_attitude_quaternion = true;
    t->attitude_quaternion[0] = number_value(&doc, attitude, "q_x", 0.0);
    t->attitude_quaternion[1] = number_value(&doc, attitude, "q_y", 0.0);
    t->attitude_quaternion[2] = number_value(&doc, attitude, "q_z", 0.0);
    t->attitude_quaternion[3] = number_value(&doc, attitude, "q_w", 1.0);
    snprintf(t->attitude_reference_frame,
             sizeof(t->attitude_reference_frame),
             "body-non-rotating-canonical");

    t->atmospheric_density =
        number_value(&doc, atmosphere, "density_kg_m3", 0.0);
    t->static_pressure = number_value(&doc, atmosphere, "pressure_pa", 0.0);
    t->speed_of_sound =
        number_value(&doc, atmosphere, "speed_of_sound_mps", 0.0);
    t->mach = number_value(&doc, aero, "mach", 0.0);
    t->dynamic_pressure = number_value(&doc, aero, "q_pa", 0.0);
    t->lift_force = number_value(&doc, aero, "lift_n", 0.0);
    t->drag_force = number_value(&doc, aero, "drag_n", 0.0);

    t->mass = number_value(&doc, vehicle, "mass_kg", 0.0);
    state->mass = t->mass;
    t->available_thrust = number_value(&doc, vehicle, "available_thrust_n", 0.0);
    t->current_thrust = number_value(&doc, vehicle, "current_thrust_n", 0.0);
    t->throttle = number_value(&doc, vehicle, "throttle", 0.0);
    t->gear = bool_value(&doc, ground, "gear_down", false);
    t->brakes = bool_value(&doc, ground, "brakes", false);
    t->has_airbrakes = true;
    t->airbrakes = false;
    bool on_ground = bool_value(&doc, ground, "on_ground", false);
    snprintf(t->vessel_situation, sizeof(t->vessel_situation), "%s",
             on_ground ? "landed" : "flying");

    t->runway_along_track = number_value(&doc, runway, "along_m", 0.0);
    t->runway_cross_track = number_value(&doc, runway, "cross_m", 0.0);
    t->has_center_of_mass = true;
    t->center_of_mass = state->position;
    bool state_finite = isfinite(state->position.x) &&
        isfinite(state->position.y) && isfinite(state->position.z) &&
        isfinite(state->velocity.x) && isfinite(state->velocity.y) &&
        isfinite(state->velocity.z) && isfinite(state->mass) &&
        state->mass > 0.0;
    bool telemetry_finite = isfinite(t->ut) && isfinite(t->latitude) &&
        isfinite(t->longitude) && isfinite(t->mean_altitude) &&
        isfinite(t->surface_speed) && isfinite(t->true_air_speed) &&
        isfinite(t->horizontal_speed) && isfinite(t->vertical_speed) &&
        isfinite(t->flight_path_angle) && isfinite(t->angle_of_attack) &&
        isfinite(t->roll) && isfinite(t->heading) &&
        isfinite(t->atmospheric_density) && t->atmospheric_density >= 0.0 &&
        isfinite(t->static_pressure) && t->static_pressure >= 0.0 &&
        isfinite(t->speed_of_sound) && t->speed_of_sound >= 0.0 &&
        isfinite(t->mach) && t->mach >= 0.0 &&
        isfinite(t->dynamic_pressure) && t->dynamic_pressure >= 0.0 &&
        isfinite(t->lift_force) && isfinite(t->drag_force) &&
        isfinite(t->mass) && t->mass > 0.0 &&
        isfinite(t->runway_along_track) && isfinite(t->runway_cross_track);
    bool position_domain = t->latitude >= -90.0 && t->latitude <= 90.0 &&
        t->longitude >= -360.0 && t->longitude <= 360.0;
    bool quaternion_finite = isfinite(t->attitude_quaternion[0]) &&
        isfinite(t->attitude_quaternion[1]) &&
        isfinite(t->attitude_quaternion[2]) &&
        isfinite(t->attitude_quaternion[3]);
    t->physics_sample_valid = required_objects && required_numeric &&
        state_finite && telemetry_finite && position_domain && quaternion_finite;
    t->physics_confidence = t->physics_sample_valid ? 1.0 : 0.0;
    t->aerodynamic_confidence =
        isfinite(t->dynamic_pressure) && t->dynamic_pressure >= 0.0 &&
        isfinite(t->lift_force) && isfinite(t->drag_force) ? 1.0 : 0.0;
    t->estimated_lift_to_drag = t->drag_force > 0.0
        ? t->lift_force / t->drag_force : 0.0;
    t->g_force = t->mass > 0.0
        ? hypot(t->lift_force, t->drag_force) / (t->mass * 9.80665) : 0.0;
    t->stall_fraction = 0.0;
    t->stall_fraction_is_measured = false;

    if (previous) {
        double dt = t->ut - previous->ut;
        /* ShuttleSim lockstep packets are sequential physics samples.  Session
           creation resets previous, so a finite positive simulator-time delta
           is the derivative domain requirement; wall-time freshness thresholds
           do not describe this transport. */
        if (isfinite(dt) && dt > 0.0) {
            t->pitch_rate = (t->pitch - previous->pitch) / dt;
            if (!has_published_bank_rate)
                t->roll_rate = signed_degrees(t->roll - previous->roll) / dt;
            t->heading_rate =
                signed_degrees(t->heading - previous->heading) / dt;
            if (!has_published_aoa_rate) {
                t->angle_of_attack_rate =
                    signed_degrees(t->angle_of_attack - previous->angle_of_attack) / dt;
                t->has_angle_of_attack_rate = true;
            }
            /* In ShuttleSim heading is already ground-track azimuth. */
            t->course_rate = t->heading_rate;
            t->has_course_rate = true;
        }
    }

    /* ShuttleSim currently exposes controlled AoA/bank response, but not a
       rigid-body angular-velocity vector.  Do not manufacture body p/q/r from
       coordinate derivatives; telemetry_init deliberately leaves these false. */
    t->has_body_pitch_rate = false;
    t->has_body_roll_rate = false;
    t->has_body_yaw_rate = false;

    fill_orbit_fields(planet, state, t);
    if (error && error_size) error[0] = '\0';
    return true;
}
