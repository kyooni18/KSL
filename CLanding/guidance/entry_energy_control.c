#include "entry_energy_control.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define STANDARD_GRAVITY 9.80665

EntryEnergyConfig entry_energy_default_config(void) {
    EntryEnergyConfig c;
    memset(&c, 0, sizeof(c));
    /* A ~90 s drag-tracking period: slow compared with the attitude loop,
       fast compared with the energy trajectory (Shuttle drag tracking had a
       comparable period relative to its much longer entry). */
    c.natural_frequency_rad_s = 0.07;
    c.damping_ratio = 0.8;
    c.integral_ratio = 0.1;
    c.integral_limit_m_s = 60000.0;
    c.altitude_error_limit_m = 12000.0;
    c.minimum_drag_accel = 0.02;
    c.minimum_lift_accel = 0.05;
    c.protection_fraction = 0.9;
    c.prediction_steps = 64;
    return c;
}

void entry_energy_reset(EntryEnergyState *state) {
    if (!state) return;
    memset(state, 0, sizeof(*state));
    state->last_ut = NAN;
}

double entry_energy_scale_height(const PlanetModel *planet, double altitude) {
    const double fallback = 5600.0; /* Kerbin-like default when the table is empty */
    if (!planet || !isfinite(altitude)) return fallback;
    double step = 250.0;
    double low = fmax(0.0, altitude - step), high = low + 2.0 * step;
    double rho_low = planet_atmospheric_density(planet, low);
    double rho_high = planet_atmospheric_density(planet, high);
    if (!(rho_low > 0.0) || !(rho_high > 0.0) || !(rho_low > rho_high))
        return fallback;
    double hs = (high - low) / log(rho_low / rho_high);
    return isfinite(hs) ? fmin(fmax(hs, 2000.0), 12000.0) : fallback;
}

/* Linear transition from the TAEM drag at V_T to the constant-drag level at
   the transition velocity; constant above it.  When the vehicle is already
   inside the transition band the segment is anchored at the current speed so
   the one-parameter family always spans [V_T, V]. */
static double profile_drag(double v, double v_taem, double v_trans, double v_now,
                           double d0, double d_taem) {
    double v1 = v_now < v_trans ? v_now : v_trans;
    if (v >= v1) return d0;
    double span = fmax(v1 - v_taem, 1.0);
    double x = fmin(fmax((v - v_taem) / span, 0.0), 1.0);
    return d_taem + (d0 - d_taem) * x;
}

static double predicted_range(const EntryEnergyInput *in, const EntryEnergyConfig *c,
        double d0, double d_taem, double remaining_energy) {
    double v = in->relative_velocity, vt = in->taem_velocity;
    double range = 0.0;
    double dmin = c->minimum_drag_accel;
    if (v > vt) {
        double dv = (v - vt) / (double)c->prediction_steps;
        for (unsigned n = 0; n < c->prediction_steps; ++n) {
            double vv = vt + ((double)n + 0.5) * dv;
            double d = profile_drag(vv, vt, in->transition_velocity, v, d0, d_taem);
            range += vv / fmax(dmin, d) * dv;
        }
    }
    /* dE/ds = -D: the kinetic part is the velocity integral above; potential/
       rotational energy still to be shed (or supplied) is charged at the
       geometric-mean drag of the remaining profile. */
    double kinetic = 0.5 * (v * v - vt * vt);
    double other = remaining_energy - fmax(0.0, kinetic);
    if (other != 0.0) {
        double d_now = profile_drag(fmax(v, vt), vt, in->transition_velocity, v, d0, d_taem);
        double mean = sqrt(fmax(dmin, d_now) * fmax(dmin, d_taem));
        range += other / fmax(dmin, mean);
    }
    return range;
}

EntryEnergyOutput entry_energy_update(EntryEnergyState *state,
        const EntryEnergyInput *in, const EntryEnergyConfig *c) {
    EntryEnergyOutput out;
    memset(&out, 0, sizeof(out));
    if (!state || !in || !c || !in->planet || !in->vehicle ||
        !isfinite(in->relative_velocity) || !(in->relative_velocity > 0.0) ||
        !isfinite(in->altitude) || !isfinite(in->latitude) ||
        !isfinite(in->flight_path_angle_deg) || !isfinite(in->vertical_speed) ||
        !isfinite(in->range_to_go) || !(in->taem_velocity > 0.0) ||
        !isfinite(in->taem_altitude) || !isfinite(in->transition_velocity) ||
        !isfinite(in->maximum_bank_deg) || !isfinite(in->ut) ||
        c->prediction_steps < 8)
        return out;
    if (!state->initialized) {
        entry_energy_reset(state);
        state->initialized = true;
    }
    double dt = isfinite(state->last_ut) ? in->ut - state->last_ut : 0.0;
    if (!(dt > 0.0) || dt > 5.0) dt = 0.0;
    state->last_ut = in->ut;

    const PlanetModel *p = in->planet;
    const VehicleProfile *veh = in->vehicle;
    double v = in->relative_velocity;
    double gamma = in->flight_path_angle_deg * DEG2RAD;
    double r = p->radius + in->altitude;
    double g = p->gravitational_parameter / (r * r);
    double drag = isfinite(in->measured_drag_accel) ? fmax(0.0, in->measured_drag_accel) : 0.0;
    double lift = isfinite(in->measured_lift_accel) ? fmax(0.0, in->measured_lift_accel) : 0.0;
    bool drag_valid = drag >= c->minimum_drag_accel;

    out.scale_height_m = entry_energy_scale_height(p, in->altitude);
    out.lift_to_drag = drag_valid ? lift / drag :
        fmax(0.05, veh->estimated_lift_to_drag);

    /* Ballistic coefficient observed now (q/D) scales TAEM and structural drag. */
    double beta = drag_valid && in->dynamic_pressure > 0.0 ?
        in->dynamic_pressure / drag : veh->estimated_ballistic_coefficient;
    if (!(beta > 1.0) || !isfinite(beta)) beta = fmax(1.0, veh->estimated_ballistic_coefficient);
    double rho_t = planet_atmospheric_density(p, in->taem_altitude);
    out.taem_drag_accel = fmax(c->minimum_drag_accel,
        0.5 * fmax(0.0, rho_t) * in->taem_velocity * in->taem_velocity / beta);
    double by_q = veh->maximum_dynamic_pressure / beta;
    double by_load = veh->maximum_g_load * STANDARD_GRAVITY / fmax(0.05, out.lift_to_drag);
    out.maximum_drag_accel = fmax(out.taem_drag_accel,
        c->protection_fraction * fmin(by_q, by_load));

    double remaining = entry_remaining_specific_energy(in->latitude, in->altitude, v,
        in->taem_latitude, in->taem_altitude, in->taem_velocity, p);
    out.required_range_m = in->range_to_go;

    /* Solve the one-parameter drag level so predicted range == required range.
       Predicted range decreases monotonically with drag level. */
    double lo = log(c->minimum_drag_accel), hi = log(out.maximum_drag_accel);
    double r_lo = predicted_range(in, c, exp(lo), out.taem_drag_accel, remaining);
    double r_hi = predicted_range(in, c, exp(hi), out.taem_drag_accel, remaining);
    double d0;
    if (!(in->range_to_go < r_lo)) {
        d0 = exp(lo);
        out.profile_at_minimum = true;
    } else if (!(in->range_to_go > r_hi)) {
        d0 = exp(hi);
        out.profile_at_maximum = true;
    } else {
        for (int i = 0; i < 48; ++i) {
            double mid = 0.5 * (lo + hi);
            double rm = predicted_range(in, c, exp(mid), out.taem_drag_accel, remaining);
            if (rm > in->range_to_go) lo = mid; else hi = mid;
        }
        d0 = exp(0.5 * (lo + hi));
    }
    out.profile_drag_accel = d0;
    out.predicted_range_m = predicted_range(in, c, d0, out.taem_drag_accel, remaining);
    out.range_error_m = out.predicted_range_m - out.required_range_m;
    out.reference_drag_accel = profile_drag(v, in->taem_velocity, in->transition_velocity,
        v, d0, out.taem_drag_accel);
    double dv = fmax(1.0, 0.002 * v);
    double d_hi = profile_drag(v, in->taem_velocity, in->transition_velocity, v + dv, d0, out.taem_drag_accel);
    double d_lo = profile_drag(v - dv, in->taem_velocity, in->transition_velocity, v, d0, out.taem_drag_accel);
    out.reference_drag_dv = (d_hi - d_lo) / (2.0 * dv);

    /* Reference altitude rate for an exponential atmosphere:
       D = rho V^2 / (2 beta)  =>  hdot = -Hs (Ddot/D - 2 Vdot/V). */
    double hs = out.scale_height_m;
    double vdot = -drag - g * sin(gamma);
    double dref = fmax(c->minimum_drag_accel, out.reference_drag_accel);
    out.hdot_reference = -hs * (out.reference_drag_dv * vdot / dref - 2.0 * vdot / v);

    /* Drag error as an altitude error: >0 means the vehicle is too high. */
    double eh = drag_valid ? hs * log(dref / drag) : c->altitude_error_limit_m;
    eh = fmin(fmax(eh, -c->altitude_error_limit_m), c->altitude_error_limit_m);
    out.altitude_error_m = eh;

    double w = c->natural_frequency_rad_s, z = c->damping_ratio;
    double ki = c->integral_ratio * w * w * w;
    double hddot = -w * w * eh - 2.0 * z * w * (in->vertical_speed - out.hdot_reference) -
        ki * state->integral_m_s;

    /* Lift perpendicular to the velocity: hddot = L cos(bank) cos(gamma)
       - D sin(gamma) - (g - Vh^2 / r). */
    double vh = v * cos(gamma);
    double cg = fmax(0.2, cos(gamma));
    double vertical = (hddot + g - vh * vh / r + drag * sin(gamma)) / cg;

    double bank_limit = fmin(fmax(0.0, in->maximum_bank_deg), 89.0);
    double load = lift / STANDARD_GRAVITY;
    bool protect = in->dynamic_pressure > c->protection_fraction * veh->maximum_dynamic_pressure ||
        load > c->protection_fraction * veh->maximum_g_load;
    out.protection_active = protect;
    double bank;
    if (!(lift >= c->minimum_lift_accel) || bank_limit <= 0.0) {
        out.degraded = !(lift >= c->minimum_lift_accel);
        bank = 0.0;
        out.saturated_lift_up = true;
    } else if (protect) {
        /* Near the q/structural limit stop diving deeper: lift up. */
        bank = 0.0;
        out.saturated_lift_up = true;
    } else {
        double ratio = vertical / lift;
        double cmin = cos(bank_limit * DEG2RAD);
        if (ratio >= 1.0) { bank = 0.0; out.saturated_lift_up = true; }
        else if (ratio <= cmin) { bank = bank_limit; out.saturated_bank_limit = true; }
        else bank = acos(ratio) * RAD2DEG;
    }
    out.vertical_lift_command = vertical;
    out.bank_magnitude_deg = bank;

    /* Bias integrator with anti-windup: hold while saturated, protecting,
       unmeasured, or during an externally requested hold (reversal). */
    bool hold = in->integrator_hold || out.saturated_lift_up ||
        out.saturated_bank_limit || !drag_valid || protect;
    if (!hold && dt > 0.0) {
        state->integral_m_s += eh * dt;
        state->integral_m_s = fmin(fmax(state->integral_m_s, -c->integral_limit_m_s),
                                   c->integral_limit_m_s);
    }
    out.integral_m_s = state->integral_m_s;
    out.degraded = out.degraded || !drag_valid;
    out.valid = isfinite(out.bank_magnitude_deg) && isfinite(out.reference_drag_accel);
    return out;
}
