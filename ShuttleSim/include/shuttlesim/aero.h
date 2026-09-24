#ifndef SHUTTLESIM_AERO_H
#define SHUTTLESIM_AERO_H
#include "types.h"
#include "world.h"

#define AERO_MACH_MAX 40
#define AERO_ALPHA_MAX 40
#define AERO_BOOK_MAX 2048

typedef struct {
    double q_pa;
    double mach;
    double alpha_deg;
    double lift_per_q_m2;
    double drag_per_q_m2;
    double support;
} AeroBookPoint;

typedef struct {
    double mach[AERO_MACH_MAX];
    double alpha_deg[AERO_ALPHA_MAX];
    double cl[AERO_MACH_MAX][AERO_ALPHA_MAX];
    double cd[AERO_MACH_MAX][AERO_ALPHA_MAX];
    size_t mach_count;
    size_t alpha_count;
    double reference_area_m2;
    AeroBookPoint book[AERO_BOOK_MAX];
    size_t book_count;
    bool book_enabled;
} AeroTable;

void aero_seed_stsn(AeroTable *a);
bool aero_load_csv(AeroTable *a, const char *path);
bool aero_load_book_csv(AeroTable *a, const char *path);
void aero_coefficients(const AeroTable *a, double mach, double alpha_deg, double *cl, double *cd);
AeroForces aero_compute(const KerbinWorld *w, const AeroTable *a, Vec3 position_i, Vec3 velocity_i,
                        double ut, double mass_kg, double aoa_rad, double bank_rad);
#endif
