#ifndef KSP_LANDER_ENTRY_ENERGY_CONTROL_H
#define KSP_LANDER_ENTRY_ENERGY_CONTROL_H

#include "landing.h"
#include "entry_energy_state.h"

#include <stdbool.h>

/*
 * MM304 longitudinal (energy/range) control.
 *
 * Structure follows the Shuttle entry-guidance principle rather than its Earth
 * I-loads: a drag-acceleration reference D_ref(V) is re-solved every cycle so
 * that its analytically predicted range equals the range still required; the
 * measured drag is driven to that reference with a damped second-order law
 * (altitude-equivalent drag error, altitude-rate damping, bounded integral) and
 * the resulting vertical-lift demand is converted to bank magnitude against the
 * *measured* lift, cos(bank) = a_vertical / L_measured.  Nothing here scales a
 * measured force by a model confidence.
 */
typedef struct {
    double natural_frequency_rad_s;   /* drag-tracking loop bandwidth */
    double damping_ratio;
    double integral_ratio;            /* Ki = ratio * omega^3 */
    double integral_limit_m_s;        /* bound on integrated altitude-equivalent error */
    double altitude_error_limit_m;    /* bound on Hs*ln(D_ref/D) */
    double minimum_drag_accel;        /* below this drag is not a usable measurement */
    double minimum_lift_accel;        /* below this lift cannot steer the trajectory */
    double protection_fraction;       /* q / normal-load fraction that forces lift-up */
    unsigned prediction_steps;
} EntryEnergyConfig;

typedef struct {
    double ut;
    double relative_velocity;      /* air-relative speed, m/s */
    double altitude;               /* m above datum */
    double latitude;               /* deg */
    double flight_path_angle_deg;  /* air-relative */
    double vertical_speed;         /* m/s */
    double measured_drag_accel;    /* m/s^2 */
    double measured_lift_accel;    /* m/s^2 */
    double dynamic_pressure;       /* Pa */
    double range_to_go;            /* m still to fly to the TAEM interface (may be <= 0) */
    double taem_velocity;
    double taem_altitude;
    double taem_latitude;
    double transition_velocity;    /* start of the linear transition-to-TAEM drag segment */
    double maximum_bank_deg;
    bool integrator_hold;          /* e.g. during a bank reversal */
    const PlanetModel *planet;
    const VehicleProfile *vehicle;
} EntryEnergyInput;

typedef struct {
    bool valid;
    bool degraded;                 /* no usable drag/lift measurement yet */
    double scale_height_m;
    double reference_drag_accel;
    double reference_drag_dv;      /* dD_ref/dV */
    double profile_drag_accel;     /* solved constant-drag level */
    double taem_drag_accel;
    double maximum_drag_accel;
    double predicted_range_m;      /* range predicted by the solved (bounded) profile */
    double required_range_m;
    double range_error_m;          /* predicted - required; >0 overshoot even at max drag */
    bool profile_at_maximum;       /* needs more drag than the vehicle may fly */
    bool profile_at_minimum;       /* cannot stretch far enough */
    double hdot_reference;
    double altitude_error_m;       /* >0: vehicle is above the drag-reference altitude */
    double integral_m_s;
    double vertical_lift_command;  /* m/s^2 of lift along local vertical demanded */
    double lift_to_drag;
    double bank_magnitude_deg;
    bool saturated_lift_up;
    bool saturated_bank_limit;
    bool protection_active;
} EntryEnergyOutput;

EntryEnergyConfig entry_energy_default_config(void);
void entry_energy_reset(EntryEnergyState *state);
EntryEnergyOutput entry_energy_update(EntryEnergyState *state,
    const EntryEnergyInput *input, const EntryEnergyConfig *config);

/* Local density scale height from the planet atmosphere table. */
double entry_energy_scale_height(const PlanetModel *planet, double altitude);

#endif
