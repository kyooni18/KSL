#include "terminal_model.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static uint64_t digest_file(const char *path, bool optional, bool *ok) {
    *ok = false;
    if (!path || !*path || (optional && !strcmp(path, "none"))) {
        *ok = optional;
        return 0;
    }
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    uint64_t h = UINT64_C(1469598103934665603);
    unsigned char block[4096];
    size_t n;
    do {
        n = fread(block, 1, sizeof(block), f);
        for (size_t i = 0; i < n; ++i) {
            h ^= block[i];
            h *= UINT64_C(1099511628211);
        }
    } while (n == sizeof(block));
    bool read_ok = !ferror(f);
    if (fclose(f) != 0) read_ok = false;
    if (read_ok) *ok = true;
    return read_ok ? h : 0;
}

static bool finite_vec3(Vec3 v) {
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static bool reject(char *reason, size_t n, const char *message) {
    if (reason && n) snprintf(reason, n, "%s", message);
    return false;
}

bool terminal_model_capture(TerminalModel *m,
        const TerminalModelSourceFiles *files,
        const LandingConfiguration *cfg, uint64_t snapshot_id,
        char *reason, size_t reason_size) {
    if (reason && reason_size) reason[0] = '\0';
    if (!m || !files || !cfg || !snapshot_id)
        return reject(reason, reason_size, "missing terminal model capture input");
    memset(m, 0, sizeof(*m));
    m->schema_version = 1;
    m->snapshot_id = snapshot_id;

    world_seed_kerbin(&m->world);
    if (!world_load_atmosphere_csv(&m->world, files->atmosphere_csv))
        return reject(reason, reason_size, "failed to load native atmosphere source");
    aero_seed_stsn(&m->aero);
    if (!aero_load_csv(&m->aero, files->aero_csv))
        return reject(reason, reason_size, "failed to load native aerodynamic source");
    bool book_ok = true;
    if (files->aero_book_csv && strcmp(files->aero_book_csv, "none") != 0) {
        if (!aero_load_book_csv(&m->aero, files->aero_book_csv))
            return reject(reason, reason_size, "failed to load native force-book source");
    }
    attitude_seed(&m->attitude);
    if (!attitude_load_ini(&m->attitude, files->attitude_ini))
        return reject(reason, reason_size, "failed to load native attitude source");

    m->vehicle = cfg->vehicle;
    m->site = cfg->site;
    m->guidance = cfg->guidance;
    m->atmosphere_source_digest = digest_file(files->atmosphere_csv, false, &book_ok);
    if (!book_ok) return reject(reason, reason_size, "failed to hash native atmosphere source");
    m->aero_source_digest = digest_file(files->aero_csv, false, &book_ok);
    if (!book_ok) return reject(reason, reason_size, "failed to hash native aerodynamic source");
    m->aero_book_source_digest = digest_file(files->aero_book_csv, true, &book_ok);
    if (!book_ok) return reject(reason, reason_size, "failed to hash native force-book source");
    m->attitude_source_digest = digest_file(files->attitude_ini, false, &book_ok);
    if (!book_ok) return reject(reason, reason_size, "failed to hash native attitude source");
    if (!terminal_model_validate(m, reason, reason_size)) return false;
    m->replay_validated = true;
    return true;
}

static bool positive_finite(double value) {
    return isfinite(value) && value > 0.0;
}
bool terminal_model_validate(const TerminalModel *m, char *reason,
                             size_t n) {
    if (reason && n) reason[0] = '\0';
    if (!m) return reject(reason, n, "missing terminal model");
    if (m->schema_version != 1 || !m->snapshot_id)
        return reject(reason, n, "terminal model snapshot identity is invalid");
    if (!positive_finite(m->world.radius_m) || !positive_finite(m->world.mu_m3_s2) ||
        !positive_finite(m->world.atmosphere_top_m) || m->world.atmosphere.count < 2 ||
        m->world.atmosphere.count > ATM_TABLE_MAX)
        return reject(reason, n, "terminal world snapshot is incomplete");
    if (!isfinite(m->world.rotation_rate_rad_s) ||
        !isfinite(m->world.rotation_phase_rad_at_ut0))
        return reject(reason, n, "terminal world rotation is invalid");
    for (size_t i = 0; i < m->world.atmosphere.count; ++i) {
        const AtmospherePoint *p = &m->world.atmosphere.p[i];
        if (!isfinite(p->altitude_m) || p->altitude_m < 0.0 ||
            !isfinite(p->sample.density_kg_m3) || p->sample.density_kg_m3 < 0.0 ||
            !isfinite(p->sample.pressure_pa) || p->sample.pressure_pa < 0.0 ||
            !isfinite(p->sample.temperature_k) || p->sample.temperature_k <= 0.0 ||
            !isfinite(p->sample.speed_of_sound_mps) || p->sample.speed_of_sound_mps <= 0.0 ||
            (i && p->altitude_m <= m->world.atmosphere.p[i - 1].altitude_m))
            return reject(reason, n, "terminal atmosphere table is invalid or unordered");
    }
    if (m->aero.mach_count < 2 || m->aero.alpha_count < 2 ||
        m->aero.mach_count > AERO_MACH_MAX ||
        m->aero.alpha_count > AERO_ALPHA_MAX ||
        !positive_finite(m->aero.reference_area_m2))
        return reject(reason, n, "terminal aerodynamic snapshot is incomplete");
    for (size_t i = 0; i < m->aero.mach_count; ++i) {
        if (!isfinite(m->aero.mach[i]) || (i && m->aero.mach[i] <= m->aero.mach[i - 1]))
            return reject(reason, n, "terminal Mach grid is invalid or unordered");
    }
    for (size_t i = 0; i < m->aero.alpha_count; ++i) {
        if (!isfinite(m->aero.alpha_deg[i]) || (i && m->aero.alpha_deg[i] <= m->aero.alpha_deg[i - 1]))
            return reject(reason, n, "terminal incidence grid is invalid or unordered");
    }
    for (size_t i = 0; i < m->aero.mach_count; ++i) {
        for (size_t j = 0; j < m->aero.alpha_count; ++j) {
            if (!isfinite(m->aero.cl[i][j]) || !isfinite(m->aero.cd[i][j]) ||
                m->aero.cd[i][j] < 0.0)
                return reject(reason, n, "terminal aerodynamic coefficient table is invalid");
        }
    }
    if (!positive_finite(m->attitude.pitch_wn) || !positive_finite(m->attitude.roll_wn) ||
        !positive_finite(m->attitude.pitch_zeta) || !positive_finite(m->attitude.roll_zeta) ||
        !positive_finite(m->attitude.max_pitch_rate_rad_s) ||
        !positive_finite(m->attitude.max_roll_rate_rad_s) ||
        !positive_finite(m->attitude.max_pitch_accel_rad_s2) ||
        !positive_finite(m->attitude.max_roll_accel_rad_s2))
        return reject(reason, n, "terminal attitude response snapshot is incomplete");
    if (!positive_finite(m->vehicle.touchdown_speed) ||
        !positive_finite(m->vehicle.maximum_angle_of_attack) ||
        !positive_finite(m->vehicle.maximum_bank_angle) ||
        !positive_finite(m->guidance.hac_radius) ||
        !positive_finite(m->guidance.final_approach_distance))
        return reject(reason, n, "terminal vehicle or runway policy is invalid");
    if (!isfinite(m->site.latitude) || !isfinite(m->site.longitude) ||
        !isfinite(m->site.altitude) || !isfinite(m->site.runway_heading) ||
        !positive_finite(m->site.runway_length) || !positive_finite(m->site.runway_width))
        return reject(reason, n, "terminal runway snapshot is invalid");
    return true;
}

bool terminal_state_validate(const TerminalDynamicState *s,
                             char *reason, size_t n) {
    if (reason && n) reason[0] = '\0';
    if (!s) return reject(reason, n, "missing terminal dynamic state");
    if (!finite_vec3(s->position_i_m) || !finite_vec3(s->velocity_i_mps) ||
        !(s->mass_kg > 0.0) || !isfinite(s->mass_kg) || !isfinite(s->ut_s))
        return reject(reason, n, "terminal dynamic state contains invalid position, velocity, mass, or time");
    if (!isfinite(s->attitude.aoa_rad) || !isfinite(s->attitude.bank_rad) ||
        !isfinite(s->attitude.aoa_rate_rad_s) ||
        !isfinite(s->attitude.bank_rate_rad_s) ||
        !isfinite(s->attitude.cmd_aoa_rad) || !isfinite(s->attitude.cmd_bank_rad) ||
        !isfinite(s->attitude.requested_aoa_rad) || !isfinite(s->attitude.requested_bank_rad))
        return reject(reason, n, "terminal attitude state is invalid");
    return true;
}
