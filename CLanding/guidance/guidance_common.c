#include "guidance_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

double hac_course_rate_cap_for_speed(double true_air_speed) {
    return 1.50 + 1.50 * clampd((220.0 - true_air_speed) / 70.0, 0.0, 1.0);
}

double hac_course_rate_cap(const Telemetry *t) {
    return hac_course_rate_cap_for_speed(t ? t->true_air_speed : 0.0);
}

double guidance_lateral_speed(const Telemetry *t) {
    if (!t) return 0.0;
    if (isfinite(t->horizontal_speed) && t->horizontal_speed > 1.0) return t->horizontal_speed;
    if (isfinite(t->surface_speed) && t->surface_speed > 1.0) return t->surface_speed;
    if (isfinite(t->true_air_speed) && t->true_air_speed > 1.0) {
        double c = isfinite(t->flight_path_angle) ? fabs(cos(t->flight_path_angle * DEG2RAD)) : 1.0;
        return fmax(1.0, t->true_air_speed * c);
    }
    return 1.0;
}

static void append_veto_reason(char *buf, size_t n, bool *first, const char *text) {
    if (!buf || n == 0 || !first || !text) return;
    size_t used = strlen(buf);
    if (used >= n - 1) return;
    int written = snprintf(buf + used, n - used, "%s%s", *first ? "" : "|", text);
    if (written > 0) *first = false;
}

void taem_capture_veto_reasons(unsigned veto, char *buf, size_t n) {
    if (!buf || n == 0) return;
    buf[0] = '\0';
    if (veto == 0u) { snprintf(buf, n, "none"); return; }
    bool first = true;
    if (veto & 1u) append_veto_reason(buf, n, &first, "speed");
    if (veto & 2u) append_veto_reason(buf, n, &first, "spatial");
    if (veto & 4u) append_veto_reason(buf, n, &first, "course");
    if (veto & 8u) append_veto_reason(buf, n, &first, "altitude");
    if (veto & 16u) append_veto_reason(buf, n, &first, "maneuver-energy");
    if (veto & 32u) append_veto_reason(buf, n, &first, "fpa");
    if (veto & 64u) append_veto_reason(buf, n, &first, "structural");
    if (veto & 128u) append_veto_reason(buf, n, &first, "hac-radius");
    if (veto & 256u) append_veto_reason(buf, n, &first, "minimum-altitude");
    if (buf[0] == '\0') snprintf(buf, n, "unknown");
}

double controlled_roll_rate(const Telemetry *t) {
    return t && isfinite(t->roll_rate) ? t->roll_rate : 0.0;
}

double controlled_aoa_rate(const Telemetry *t) {
    return t && t->has_angle_of_attack_rate && isfinite(t->angle_of_attack_rate) ?
        t->angle_of_attack_rate : 0.0;
}

GuidanceAttitudeLimits stabilized_attitude_limits(const Telemetry *t,
        const GuidanceSettings *s, GuidancePhase phase, bool pitch_axis) {
    return decision_guidance_attitude_limits(t, s, phase, pitch_axis);
}

void seed_stabilized_limiter(JerkLimiter *limiter, double value, double rate, bool angle) {
    if (!limiter || limiter->has_value || !isfinite(value)) return;
    limiter->has_value = true;
    limiter->value = angle ? norm_deg(value) : value;
    limiter->rate = isfinite(rate) ? rate : 0.0;
}

double roll_capture_time(const Telemetry *t, double target_bank,
        double configured_rate_limit, double configured_accel_limit) {
    if (!t || !isfinite(t->roll) || !isfinite(target_bank)) return INFINITY;
    double rate_limit = fabs(configured_rate_limit);
    double accel_limit = fabs(configured_accel_limit);
    if (t->attitude_response.roll_valid) {
        rate_limit = t->attitude_response.maximum_roll_rate_deg_s;
        accel_limit = t->attitude_response.maximum_roll_accel_deg_s2;
    }
    if (!(rate_limit > DBL_MIN) || !(accel_limit > DBL_MIN) ||
        !isfinite(rate_limit) || !isfinite(accel_limit)) return INFINITY;
    double error = norm_signed_deg(target_bank - norm_signed_deg(t->roll));
    return decision_axis_capture_time(error, controlled_roll_rate(t), accel_limit, rate_limit);
}

double hac_response_lead_time(const Telemetry *t, const GuidanceSettings *s,
        double nominal_bank) {
    if (!s) return INFINITY;
    return roll_capture_time(t, nominal_bank, s->taem_roll_rate, s->entry_roll_acceleration);
}

void hac_projected_local_state(const Telemetry *t, const LandingSite *site,
        double planet_radius, double course, double seconds, double target_bank,
        double roll_rate_limit, double roll_accel_limit, double lift_accel,
        double bank_effectiveness, double drag_accel, double *out_e, double *out_n,
        double *out_course) {
    GeoPoint origin = {site->latitude, site->longitude, site->altitude};
    GeoPoint current = {t->latitude, t->longitude, t->mean_altitude};
    double e = 0.0, n = 0.0;
    local_offsets(origin, current, planet_radius, &e, &n);
    seconds = fmax(0.0, seconds);
    target_bank = clampd(target_bank, -70.0, 70.0);
    roll_rate_limit = fmax(1.0, fabs(roll_rate_limit));
    roll_accel_limit = fmax(.5, fabs(roll_accel_limit));
    bank_effectiveness = clampd(bank_effectiveness, .35, 1.8);
    lift_accel = fmax(0.0, lift_accel);
    drag_accel = clampd(drag_accel, -40.0, 40.0);
    double bank = norm_signed_deg(t->roll);
    double bank_rate = clampd(controlled_roll_rate(t), -roll_rate_limit * 1.4, roll_rate_limit * 1.4);
    double course_rate = isfinite(t->course_rate) ? clampd(t->course_rate, -3.5, 3.5) : 0.0;
    double tas0 = fmax(1.0, t->true_air_speed), tas = tas0;
    double horizontal0 = fmax(1.0, t->horizontal_speed), horizontal = horizontal0;
    int steps = (int)ceil(seconds / .12);
    if (steps < 1) steps = 1;
    if (steps > 240) steps = 240;
    double dt = seconds / (double)steps;
    for (int i = 0; i < steps; ++i) {
        double bank_error = norm_signed_deg(target_bank - bank);
        double desired_bank_rate = clampd(bank_error / 1.05, -roll_rate_limit, roll_rate_limit);
        bank_rate += clampd(desired_bank_rate - bank_rate,
                            -roll_accel_limit * dt, roll_accel_limit * dt);
        bank = clampd(bank + bank_rate * dt, -70.0, 70.0);
        double speed_scale = clampd((tas / tas0) * (tas / tas0), .45, 1.15);
        double lateral = lift_accel * speed_scale *
            sin(clampd(bank * bank_effectiveness, -89.0, 89.0) * DEG2RAD);
        double kinematic_rate = lateral / fmax(horizontal, 20.0) * RAD2DEG;
        double rate_cap = hac_course_rate_cap_for_speed(tas) * 1.60 / 1.50;
        kinematic_rate = clampd(kinematic_rate, -rate_cap, rate_cap);
        double rate_alpha = dt / fmax(.55 + dt, 1e-6);
        course_rate += (kinematic_rate - course_rate) * rate_alpha;
        double mid_course = course + course_rate * dt * .5;
        double projected_altitude = t->mean_altitude + t->vertical_speed * (i + .5) * dt;
        double surface_horizontal = horizontal * planet_radius /
            fmax(1.0, planet_radius + projected_altitude);
        e += surface_horizontal * sin(mid_course * DEG2RAD) * dt;
        n += surface_horizontal * cos(mid_course * DEG2RAD) * dt;
        course = norm_deg(course + course_rate * dt);
        tas = fmax(1.0, tas - drag_accel * dt);
        horizontal = fmax(1.0, horizontal0 * (tas / tas0));
    }
    if (out_e) *out_e = e;
    if (out_n) *out_n = n;
    if (out_course) *out_course = norm_deg(course);
}

bool hac_fixed_alignment_geometry(HACPoint2 *entry_out, HACPoint2 *center_out,
        HACPoint2 *exit_out, const LandingSite *site, const GuidanceSettings *s,
        double hac_radius, double capture_course, double side) {
    if (!entry_out || !center_out || !exit_out || !site || !s ||
        !(hac_radius > 0.0) || fabs(side) != 1.0 || !isfinite(capture_course)) return false;
    double runway = site->runway_heading * DEG2RAD;
    double capture = capture_course * DEG2RAD;
    double vh_e = sin(runway), vh_n = cos(runway);
    double rh_e = cos(runway), rh_n = -sin(runway);
    double rf_e = cos(capture), rf_n = -sin(capture);
    HACPoint2 exit = {-vh_e * s->final_approach_distance,
                      -vh_n * s->final_approach_distance};
    HACPoint2 center = {exit.e + side * hac_radius * rh_e,
                        exit.n + side * hac_radius * rh_n};
    HACPoint2 entry = {center.e - side * hac_radius * rf_e,
                       center.n - side * hac_radius * rf_n};
    *entry_out = entry;
    *center_out = center;
    *exit_out = exit;
    return true;
}

double terminal_default_hac_side(const Telemetry *t, const LandingSite *site,
        double course) {
    if (!site) return 1.0;
    double reference = isfinite(course) ? course :
        (t && isfinite(t->ground_track_heading) ? t->ground_track_heading :
         (t && isfinite(t->heading) ? t->heading : site->runway_heading));
    double turn_delta = norm_signed_deg(site->runway_heading - reference);
    double resolution = sqrt(DBL_EPSILON) * RAD2DEG;
    if (turn_delta < -resolution) return 1.0;
    if (turn_delta > resolution) return -1.0;
    if (t && isfinite(t->runway_cross_track) && fabs(t->runway_cross_track) > sqrt(DBL_EPSILON))
        return t->runway_cross_track >= 0.0 ? 1.0 : -1.0;
    return 1.0;
}

HACGuidance terminal_runway_line_path_guidance(const Telemetry *t,
        const LandingSite *site, const GuidanceSettings *s, double gravity,
        double course) {
    HACGuidance out;
    memset(&out, 0, sizeof(out));
    if (!t || !site || !s) return out;
    double speed = fmax(1.0, guidance_lateral_speed(t));
    double station = -s->final_approach_distance;
    double station_gap = fmax(0.0, station - t->runway_along_track);
    double look = clampd(fmax(speed * 5.5, station_gap * .45), 700.0, 5200.0);
    double intercept = clampd(atan2(-t->runway_cross_track, look) * RAD2DEG, -24.0, 24.0);
    out.heading = norm_deg(site->runway_heading + intercept);
    out.course_error = norm_signed_deg(out.heading - course);
    double lateral = 2.0 * speed * speed / fmax(look, 1.0) * sin(out.course_error * DEG2RAD);
    double rate_cap = fmin(1.35, hac_course_rate_cap(t)) * DEG2RAD;
    lateral = clampd(lateral, -speed * rate_cap, speed * rate_cap);
    out.lateral_acceleration = lateral;
    out.bank = clampd(atan2(lateral, fmax(gravity, .01)) * RAD2DEG, -45.0, 45.0);
    out.radial_error = t->runway_cross_track;
    out.arc_remaining = station_gap;
    out.distance_final = fmax(0.0, -t->runway_along_track);
    out.desired_altitude = site->altitude + out.distance_final * tan(s->taem_glide_slope * DEG2RAD);
    return out;
}

void reference_trajectory(Trajectory *out, const LandingSite *site,
        const GuidanceSettings *settings, double radius, double hac_side) {
    (void)site;
    (void)settings;
    (void)radius;
    (void)hac_side;
    if (out) trajectory_clear(out);
}
