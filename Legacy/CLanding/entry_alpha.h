#ifndef KSP_LANDER_ENTRY_ALPHA_H
#define KSP_LANDER_ENTRY_ALPHA_H

#include "landing.h"

#include <stdbool.h>
#include <stddef.h>

#define ENTRY_ALPHA_MAX_SCHEDULE_POINTS 8

typedef enum {
    ENTRY_ALPHA_PRE_ENTRY = 0,
    ENTRY_ALPHA_TEMPERATURE_CONTROL,
    ENTRY_ALPHA_EQUILIBRIUM_GLIDE,
    ENTRY_ALPHA_CONSTANT_DRAG,
    ENTRY_ALPHA_TRANSITION,
    ENTRY_ALPHA_PHASE_COUNT
} EntryAlphaPhase;

typedef enum {
    ENTRY_ALPHA_LIMIT_NONE = 0,
    ENTRY_ALPHA_LIMIT_MAX_AOA,
    ENTRY_ALPHA_LIMIT_STALL_MARGIN,
    ENTRY_ALPHA_LIMIT_DYNAMIC_PRESSURE,
    ENTRY_ALPHA_LIMIT_G_LOAD,
    ENTRY_ALPHA_LIMIT_THERMAL_PROTECTION,
    ENTRY_ALPHA_LIMIT_AERO_CONFIDENCE,
    ENTRY_ALPHA_LIMIT_COMMAND_RATE,
    ENTRY_ALPHA_LIMIT_INVALID_INPUT
} EntryAlphaLimitReason;

typedef struct {
    size_t point_count;
    double relative_velocity[ENTRY_ALPHA_MAX_SCHEDULE_POINTS];
    double alpha[ENTRY_ALPHA_MAX_SCHEDULE_POINTS];
    double phase_modulation_scale[ENTRY_ALPHA_PHASE_COUNT];
    double drag_error_gain;
    double maximum_modulation;
    double modulation_rate_limit;
    double target_rate_limit;
    /* Minimum nominal Entry incidence required to keep the heatshield presented
       to the flow. Stall/g-load emergency protection may temporarily override it. */
    double minimum_protective_aoa;
    double minimum_aero_confidence;
    double dynamic_pressure_protect_ratio;
    double g_load_protect_ratio;
    double stall_fraction_limit;
    double calibrated_stall_speed_margin;
} EntryAlphaSchedule;

typedef struct {
    double relative_velocity;
    EntryAlphaPhase phase;

    /* Positive drag error means measured drag is below the reference. */
    double reference_drag_accel;
    double measured_drag_accel;

    double current_aoa;
    double current_aoa_rate;
    bool has_previous_target;
    double previous_target_aoa;
    bool has_previous_modulation;
    double previous_modulation;
    double dt;

    double maximum_aoa;
    /* Learned/calibrated maximum incidence that preserves the requested stall margin.
       Pass NAN when no learned incidence limit is available. */
    double stall_margin_aoa_limit;
    double stall_fraction;
    /* True only for an independent measured stall observable. Conservative
       q/speed/AoA fallback risk must not authorize a low-q emergency thermal unload. */
    bool stall_fraction_is_measured;
    double calibrated_stall_speed;
    double minimum_safe_speed;

    double dynamic_pressure;
    double maximum_dynamic_pressure;
    double g_load;
    double maximum_g_load;
    double aero_confidence;
} EntryAlphaInput;

typedef struct {
    bool valid;
    bool degraded;
    double nominal_aoa;
    double requested_modulation;
    double modulation;
    double unconstrained_target_aoa;
    double target_aoa;
    EntryAlphaLimitReason limiting_reason;
} EntryAlphaResult;

void entry_alpha_schedule_default(EntryAlphaSchedule *schedule,
                                  const VehicleProfile *vehicle,
                                  double transition_velocity);
bool entry_alpha_schedule_valid(const EntryAlphaSchedule *schedule);
double entry_alpha_schedule_interpolate(const EntryAlphaSchedule *schedule,
                                        double relative_velocity);
/* Lowest steady non-emergency AoA the production drag-tracking law can command.
   This mirrors phase/confidence/asymmetric negative modulation and q protection,
   but intentionally excludes stall/g-load emergency unloads and finite-rate lag. */
double entry_alpha_minimum_drag_tracking_aoa(const EntryAlphaSchedule *schedule,
                                             EntryAlphaPhase phase,
                                             double relative_velocity,
                                             double dynamic_pressure,
                                             double maximum_dynamic_pressure,
                                             double aero_confidence);
EntryAlphaResult entry_alpha_command(const EntryAlphaSchedule *schedule,
                                     const EntryAlphaInput *input);
const char *entry_alpha_limit_reason_string(EntryAlphaLimitReason reason);

#endif
