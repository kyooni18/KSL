#ifndef KSP_LANDER_ENTRY_DRAG_REFERENCE_H
#define KSP_LANDER_ENTRY_DRAG_REFERENCE_H

#include "landing.h"

#include <stdbool.h>

typedef struct {
    /* Velocity anchors are expressed relative to the supplied TAEM target so the
       profile scales with the active STS-N/Kerbin terminal condition rather than
       embedding an Earth Shuttle I-load. */
    double entry_velocity_ratio;
    double temperature_end_progress;
    double equilibrium_end_progress;
    double constant_end_progress;

    double temperature_start_drag_ratio;
    double equilibrium_blend;
    double equilibrium_anchor_weight;

    /* Remaining-range feedback multiplies the drag-v profile by a smooth positive
       scale. Authority grows continuously as Entry progresses. */
    double range_gain;
    double minimum_range_authority;
    double minimum_drag_scale;
    double maximum_drag_scale;
    double range_error_scale_fraction;
    double minimum_range_error_scale;

    /* Convert longitudinal drag/range error into required vertical lift. The
       lateral worker turns this demand into bank magnitude using measured lift. */
    double vertical_drag_gain;
    double vertical_range_gain;
    double maximum_vertical_lift_correction;

    double maximum_drag_g_fraction;
    /* Local profile feasibility: calibrate q/beta against measured/modelled drag
       and allow bounded control authority above the current achieved value. */
    double local_drag_authority_multiplier;
    double minimum_local_drag_confidence;
    double minimum_drag_accel;
    unsigned prediction_steps;
} EntryDragReferenceConfig;

typedef struct {
    EntryPhase phase;
    double relative_velocity;
    double latitude;
    double altitude;

    /* Acceleration magnitudes in m/s^2. Either drag source may be NAN. */
    double measured_drag_accel;
    double modeled_drag_accel;
    double aero_confidence;
    /* Current flow incidence lets the reduced drag-v model respect the same
       thermal-protection AoA floor that Entry execution must actually fly. */
    bool has_incidence;
    double angle_of_attack;
    double sideslip;

    /* Great-circle range to the landing site and desired TAEM shell radius. */
    double range_to_site;
    double taem_range;
    double taem_latitude;
    double taem_altitude;
    double taem_velocity;

    const PlanetModel *planet;
    const VehicleProfile *vehicle;
} EntryDragReferenceInput;

typedef struct {
    bool valid;
    bool active;
    bool degraded;
    double confidence;

    double entry_anchor_velocity;
    double profile_progress;
    double range_authority;

    double required_average_drag_accel;
    double equilibrium_drag_accel;
    double constant_drag_accel;
    double taem_drag_accel;
    double nominal_drag_accel;
    double reference_drag_accel;
    double minimum_achievable_drag_accel;
    bool drag_floor_active;
    double observed_drag_accel;
    /* Positive means actual/measured drag is below the reference. */
    double drag_error_accel;
    double drag_error_fraction;

    double available_range_to_taem;
    double unshaped_predicted_range;
    double predicted_range_to_taem;
    /* Positive means the drag-v reference predicts an overshoot. */
    double unshaped_range_error;
    double predicted_range_error;
    double range_error_scale;
    double drag_range_scale;

    /* Useful local trends for supervisory prediction/control allocation. */
    double reference_drag_dv;
    double predicted_range_d_drag;
    double required_vertical_lift_accel;
} EntryDragReferenceOutput;

EntryDragReferenceConfig entry_drag_reference_default_config(void);
bool entry_drag_reference_config_valid(const EntryDragReferenceConfig *config);
EntryDragReferenceOutput entry_drag_reference_compute(
    const EntryDragReferenceInput *input,
    const EntryDragReferenceConfig *config);

/* Populate the phase gates owned by the Entry executive without mutating it.
   Pre-entry load sensing remains an executive/integration input, not DRAGREF. */
EntryExecProfile entry_drag_reference_exec_profile(
    const EntryDragReferenceInput *input,
    const EntryDragReferenceConfig *config,
    const EntryDragReferenceOutput *reference);

#endif
