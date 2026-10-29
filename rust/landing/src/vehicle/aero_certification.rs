use crate::sys::*;
extern "C" {
    fn exp(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn log(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn sqrt(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn hypot(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn fabs(__x: ::core::ffi::c_double) -> ::core::ffi::c_double;
    fn fmax(
        __x: ::core::ffi::c_double,
        __y: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn clampd(
        v: ::core::ffi::c_double,
        lo: ::core::ffi::c_double,
        hi: ::core::ffi::c_double,
    ) -> ::core::ffi::c_double;
    fn aerodynamic_force_factors_mach(
        mach: ::core::ffi::c_double,
        angle_of_attack: ::core::ffi::c_double,
        vehicle: *const VehicleProfile,
        lift_factor: *mut ::core::ffi::c_double,
        drag_factor: *mut ::core::ffi::c_double,
    );
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Vector3 {
    pub x: ::core::ffi::c_double,
    pub y: ::core::ffi::c_double,
    pub z: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VehicleProfile {
    pub model_id: [::core::ffi::c_char; 96],
    pub estimated_lift_to_drag: ::core::ffi::c_double,
    pub estimated_ballistic_coefficient: ::core::ffi::c_double,
    pub entry_angle_of_attack: ::core::ffi::c_double,
    pub maximum_angle_of_attack: ::core::ffi::c_double,
    pub maximum_bank_angle: ::core::ffi::c_double,
    pub maximum_dynamic_pressure: ::core::ffi::c_double,
    pub maximum_g_load: ::core::ffi::c_double,
    pub minimum_safe_speed: ::core::ffi::c_double,
    pub final_approach_speed: ::core::ffi::c_double,
    pub touchdown_speed: ::core::ffi::c_double,
    pub terminal_maximum_lift_angle_of_attack: ::core::ffi::c_double,
    pub allow_powered_approach: bool,
    pub maximum_approach_throttle: ::core::ffi::c_double,
    pub airbrake_action_group: ::core::ffi::c_uint,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VesselPhysicsModel {
    pub state: VehicleState,
    pub center_of_mass: Vector3,
    pub lift_vector: Vector3,
    pub drag_vector: Vector3,
    pub center_of_mass_root: Vector3,
    pub has_center_of_mass_root: bool,
    pub has_state: bool,
    pub has_center_of_mass: bool,
    pub gear: bool,
    pub brakes: bool,
    pub airbrakes: ::core::ffi::c_int,
    pub thrust: ::core::ffi::c_double,
    pub available_thrust: ::core::ffi::c_double,
    pub throttle: ::core::ffi::c_double,
    pub mass_flow: ::core::ffi::c_double,
    pub propellant_per_impulse: ::core::ffi::c_double,
    pub authority: [::core::ffi::c_double; 3],
    pub authority_confidence: [::core::ffi::c_double; 3],
    pub previous_rate: [::core::ffi::c_double; 3],
    pub previous_control: [::core::ffi::c_double; 3],
    pub torque: [::core::ffi::c_double; 3],
    pub inertia: [::core::ffi::c_double; 3],
    pub previous_rate_valid: [bool; 3],
    pub authority_q: ::core::ffi::c_double,
    pub sideslip: ::core::ffi::c_double,
    pub confidence: ::core::ffi::c_double,
    pub model_residual: ::core::ffi::c_double,
    pub model_residual_confidence: ::core::ffi::c_double,
    pub certified_uncertainty: ::core::ffi::c_double,
    pub certified_confidence: ::core::ffi::c_double,
    pub last_residual_ut: ::core::ffi::c_double,
    pub has_last_residual_ut: bool,
    pub force_residual_per_q: Vector3,
    pub force_residual_variance_per_q: Vector3,
    pub force_residual_confidence: ::core::ffi::c_double,
    pub dry_mass: ::core::ffi::c_double,
    pub count: ::core::ffi::c_uint,
    pub next: ::core::ffi::c_uint,
    pub accepted: ::core::ffi::c_uint,
    pub rejected: ::core::ffi::c_uint,
    pub live_observations: ::core::ffi::c_uint,
    pub samples: [VesselAeroSample; 192],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VesselAeroSample {
    pub q: ::core::ffi::c_double,
    pub mach: ::core::ffi::c_double,
    pub aoa: ::core::ffi::c_double,
    pub beta: ::core::ffi::c_double,
    pub mass: ::core::ffi::c_double,
    pub force_per_q: Vector3,
    pub gear: bool,
    pub brakes: bool,
    pub airbrakes: ::core::ffi::c_int,
    pub observations: ::core::ffi::c_uint,
    pub trust: ::core::ffi::c_double,
    pub last_observation_ut: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct VehicleState {
    pub ut: ::core::ffi::c_double,
    pub position: Vector3,
    pub velocity: Vector3,
    pub mass: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AerodynamicModel {
    pub lift_to_drag: ::core::ffi::c_double,
    pub ballistic_coefficient: ::core::ffi::c_double,
    pub confidence: ::core::ffi::c_double,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AerodynamicEnvelope {
    pub regimes: [AerodynamicModel; 4],
}
unsafe extern "C" fn certification_finite_vector(mut v: Vector3) -> bool {
    return v.x.is_finite() as i32 != 0 && v.y.is_finite() as i32 != 0
        && v.z.is_finite() as i32 != 0;
}
unsafe extern "C" fn certified_regime(
    mut mach: ::core::ffi::c_double,
) -> ::core::ffi::c_int {
    static mut anchors: [::core::ffi::c_double; 4] = [0.35f64, 1.05f64, 2.6f64, 6.0f64];
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < 3 as ::core::ffi::c_int {
        if mach
            < 0.5f64
                * (anchors[i as usize] + anchors[(i + 1 as ::core::ffi::c_int) as usize])
        {
            return i;
        }
        i += 1;
    }
    return 3 as ::core::ffi::c_int;
}
unsafe extern "C" fn certified_sample_parameters(
    mut sample: *const VesselAeroSample,
    mut baseline: *const VehicleProfile,
    mut lift_to_drag: *mut ::core::ffi::c_double,
    mut ballistic_coefficient: *mut ::core::ffi::c_double,
    mut precision_weight: *mut ::core::ffi::c_double,
    mut independent_weight: *mut ::core::ffi::c_double,
) -> bool {
    if sample.is_null() || baseline.is_null()
        || (*sample).gear as ::core::ffi::c_int != 0
        || (*sample).brakes as ::core::ffi::c_int != 0
        || (*sample).airbrakes != 0 as ::core::ffi::c_int
        || !((*sample).force_per_q.x > 0.0f64) || !((*sample).mass > 0.0f64)
        || !certification_finite_vector((*sample).force_per_q)
        || (*sample).mach.is_finite() as i32 == 0
        || (*sample).aoa.is_finite() as i32 == 0
        || (*sample).beta.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut incidence: ::core::ffi::c_double = hypot((*sample).aoa, (*sample).beta);
    if !(incidence <= (*baseline).maximum_angle_of_attack) {
        return false_0 != 0;
    }
    let mut lift_factor: ::core::ffi::c_double = 0.0f64;
    let mut drag_factor: ::core::ffi::c_double = 1.0f64;
    aerodynamic_force_factors_mach(
        (*sample).mach,
        (*sample).aoa,
        baseline,
        &raw mut lift_factor,
        &raw mut drag_factor,
    );
    if !(drag_factor > DBL_MIN) || !(fabs(lift_factor) > DBL_MIN) {
        return false_0 != 0;
    }
    let mut observed_ld: ::core::ffi::c_double = fabs((*sample).force_per_q.y)
        / (*sample).force_per_q.x;
    let mut observed_beta: ::core::ffi::c_double = (*sample).mass
        / (*sample).force_per_q.x;
    let mut ld: ::core::ffi::c_double = observed_ld * drag_factor / fabs(lift_factor);
    let mut beta: ::core::ffi::c_double = observed_beta * drag_factor;
    if !(ld > 0.0f64) || !(beta > 0.0f64) || ld.is_finite() as i32 == 0
        || beta.is_finite() as i32 == 0
    {
        return false_0 != 0;
    }
    let mut trust: ::core::ffi::c_double = clampd(
        if (*sample).trust.is_finite() as i32 != 0 { (*sample).trust } else { 0.0f64 },
        0.0f64,
        1.0f64,
    );
    let mut independent: ::core::ffi::c_double = (if (*sample).observations != 0 {
        (*sample).observations
    } else {
        1 as ::core::ffi::c_uint
    }) as ::core::ffi::c_double;
    let mut weight: ::core::ffi::c_double = trust * independent;
    if !(weight > 0.0f64) {
        return false_0 != 0;
    }
    *lift_to_drag = ld;
    *ballistic_coefficient = beta;
    *precision_weight = weight;
    *independent_weight = independent;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn vessel_physics_derive_envelope(
    mut model: *mut VesselPhysicsModel,
    mut baseline: *const VehicleProfile,
    mut envelope: *mut AerodynamicEnvelope,
    mut planning: *mut AerodynamicModel,
) -> bool {
    if model.is_null() || baseline.is_null() || envelope.is_null() || planning.is_null()
    {
        return false_0 != 0;
    }
    let mut precision: [::core::ffi::c_double; 4] = [
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0.,
        0.,
        0.,
    ];
    let mut independent: [::core::ffi::c_double; 4] = [
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0.,
        0.,
        0.,
    ];
    let mut ld_sum: [::core::ffi::c_double; 4] = [
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0.,
        0.,
        0.,
    ];
    let mut log_beta_sum: [::core::ffi::c_double; 4] = [
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0.,
        0.,
        0.,
    ];
    let mut regime: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while regime < 4 as ::core::ffi::c_int {
        (*envelope).regimes[regime as usize] = AerodynamicModel {
            lift_to_drag: (*baseline).estimated_lift_to_drag,
            ballistic_coefficient: (*baseline).estimated_ballistic_coefficient,
            confidence: 0.0f64,
        };
        regime += 1;
    }
    let mut i: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while i < (*model).count {
        let mut ld: ::core::ffi::c_double = 0.;
        let mut beta: ::core::ffi::c_double = 0.;
        let mut weight: ::core::ffi::c_double = 0.;
        let mut sample_independent: ::core::ffi::c_double = 0.;
        if certified_sample_parameters(
            (&raw mut (*model).samples as *mut VesselAeroSample).offset(i as isize)
                as *mut VesselAeroSample,
            baseline,
            &raw mut ld,
            &raw mut beta,
            &raw mut weight,
            &raw mut sample_independent,
        ) {
            let mut regime_0: ::core::ffi::c_int = certified_regime(
                (*model).samples[i as usize].mach,
            );
            precision[regime_0 as usize] += weight;
            independent[regime_0 as usize] += sample_independent;
            ld_sum[regime_0 as usize] += weight * ld;
            log_beta_sum[regime_0 as usize] += weight * log(beta);
        }
        i = i.wrapping_add(1);
    }
    let mut ld_mean: [::core::ffi::c_double; 4] = [
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0.,
        0.,
        0.,
    ];
    let mut beta_mean: [::core::ffi::c_double; 4] = [
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0.,
        0.,
        0.,
    ];
    let mut regime_1: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while regime_1 < 4 as ::core::ffi::c_int {
        if precision[regime_1 as usize] > 0.0f64 {
            ld_mean[regime_1 as usize] = ld_sum[regime_1 as usize]
                / precision[regime_1 as usize];
            beta_mean[regime_1 as usize] = exp(
                log_beta_sum[regime_1 as usize] / precision[regime_1 as usize],
            );
            (*envelope).regimes[regime_1 as usize].lift_to_drag = ld_mean[regime_1
                as usize];
            (*envelope).regimes[regime_1 as usize].ballistic_coefficient = beta_mean[regime_1
                as usize];
        }
        regime_1 += 1;
    }
    let mut scatter_sum: [::core::ffi::c_double; 4] = [
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0.,
        0.,
        0.,
    ];
    let mut scatter_weight: [::core::ffi::c_double; 4] = [
        0 as ::core::ffi::c_int as ::core::ffi::c_double,
        0.,
        0.,
        0.,
    ];
    let mut global_scatter_sum: ::core::ffi::c_double = 0.0f64;
    let mut global_scatter_weight: ::core::ffi::c_double = 0.0f64;
    let mut i_0: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while i_0 < (*model).count {
        let mut ld_0: ::core::ffi::c_double = 0.;
        let mut beta_0: ::core::ffi::c_double = 0.;
        let mut weight_0: ::core::ffi::c_double = 0.;
        let mut sample_independent_0: ::core::ffi::c_double = 0.;
        if certified_sample_parameters(
            (&raw mut (*model).samples as *mut VesselAeroSample).offset(i_0 as isize)
                as *mut VesselAeroSample,
            baseline,
            &raw mut ld_0,
            &raw mut beta_0,
            &raw mut weight_0,
            &raw mut sample_independent_0,
        ) {
            let mut regime_2: ::core::ffi::c_int = certified_regime(
                (*model).samples[i_0 as usize].mach,
            );
            if !(!(precision[regime_2 as usize] > 0.0f64)
                || !(ld_mean[regime_2 as usize] > 0.0f64)
                || !(beta_mean[regime_2 as usize] > 0.0f64))
            {
                let mut ld_relative: ::core::ffi::c_double = (ld_0
                    - ld_mean[regime_2 as usize]) / ld_mean[regime_2 as usize];
                let mut beta_log: ::core::ffi::c_double = log(
                    beta_0 / beta_mean[regime_2 as usize],
                );
                let mut error_squared: ::core::ffi::c_double = 0.5f64
                    * (ld_relative * ld_relative + beta_log * beta_log);
                scatter_sum[regime_2 as usize] += weight_0 * error_squared;
                scatter_weight[regime_2 as usize] += weight_0;
                global_scatter_sum += weight_0 * error_squared;
                global_scatter_weight += weight_0;
            }
        }
        i_0 = i_0.wrapping_add(1);
    }
    let mut supported_regimes: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    let mut confidence_sum: ::core::ffi::c_double = 0.0f64;
    let mut regime_3: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while regime_3 < 4 as ::core::ffi::c_int {
        if !(!(precision[regime_3 as usize] > 0.0f64)
            || !(independent[regime_3 as usize] > 0.0f64))
        {
            supported_regimes = supported_regimes.wrapping_add(1);
            let mut rms_scatter: ::core::ffi::c_double = if scatter_weight[regime_3
                as usize] > 0.0f64
            {
                sqrt(scatter_sum[regime_3 as usize] / scatter_weight[regime_3 as usize])
            } else {
                0.0f64
            };
            let mut standard_error: ::core::ffi::c_double = rms_scatter
                / sqrt(independent[regime_3 as usize]);
            let mut mean_trust: ::core::ffi::c_double = precision[regime_3 as usize]
                / independent[regime_3 as usize];
            let mut confidence: ::core::ffi::c_double = clampd(
                mean_trust / (1.0f64 + standard_error),
                0.0f64,
                1.0f64,
            );
            (*envelope).regimes[regime_3 as usize].confidence = confidence;
            confidence_sum += confidence;
        }
        regime_3 += 1;
    }
    (*model).certified_confidence = confidence_sum / 4.0f64;
    let mut empirical_scatter: ::core::ffi::c_double = if global_scatter_weight > 0.0f64
    {
        sqrt(global_scatter_sum / global_scatter_weight)
    } else {
        0.0f64
    };
    (*model).certified_uncertainty = fmax(
        empirical_scatter,
        1.0f64 - (*model).certified_confidence,
    );
    let mut first_regime: ::core::ffi::c_int = if precision[2 as ::core::ffi::c_int
        as usize] > 0.0f64 || precision[3 as ::core::ffi::c_int as usize] > 0.0f64
    {
        2 as ::core::ffi::c_int
    } else {
        0 as ::core::ffi::c_int
    };
    let mut last_regime: ::core::ffi::c_int = 3 as ::core::ffi::c_int;
    let mut target_regimes: ::core::ffi::c_uint = (last_regime - first_regime
        + 1 as ::core::ffi::c_int) as ::core::ffi::c_uint;
    let mut planning_supported: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    let mut planning_weight: ::core::ffi::c_double = 0.0f64;
    let mut planning_ld: ::core::ffi::c_double = 0.0f64;
    let mut planning_log_beta: ::core::ffi::c_double = 0.0f64;
    let mut regime_4: ::core::ffi::c_int = first_regime;
    while regime_4 <= last_regime {
        if precision[regime_4 as usize] > 0.0f64 {
            planning_supported = planning_supported.wrapping_add(1);
            let mut confidence_0: ::core::ffi::c_double = (*envelope)
                .regimes[regime_4 as usize]
                .confidence;
            let mut weight_1: ::core::ffi::c_double = precision[regime_4 as usize]
                * confidence_0;
            if weight_1 > 0.0f64 {
                planning_weight += weight_1;
                planning_ld
                    += weight_1 * (*envelope).regimes[regime_4 as usize].lift_to_drag;
                planning_log_beta
                    += weight_1
                        * log(
                            (*envelope).regimes[regime_4 as usize].ballistic_coefficient,
                        );
            }
        }
        regime_4 += 1;
    }
    if planning_weight > 0.0f64 {
        let mut coverage: ::core::ffi::c_double = planning_supported
            as ::core::ffi::c_double / target_regimes as ::core::ffi::c_double;
        let mut reliability_numerator: ::core::ffi::c_double = 0.0f64;
        let mut reliability_denominator: ::core::ffi::c_double = 0.0f64;
        let mut regime_5: ::core::ffi::c_int = first_regime;
        while regime_5 <= last_regime {
            if precision[regime_5 as usize] > 0.0f64 {
                reliability_numerator
                    += precision[regime_5 as usize]
                        * (*envelope).regimes[regime_5 as usize].confidence;
                reliability_denominator += precision[regime_5 as usize];
            }
            regime_5 += 1;
        }
        let mut reliability: ::core::ffi::c_double = if reliability_denominator > 0.0f64
        {
            reliability_numerator / reliability_denominator
        } else {
            0.0f64
        };
        *planning = AerodynamicModel {
            lift_to_drag: planning_ld / planning_weight,
            ballistic_coefficient: exp(planning_log_beta / planning_weight),
            confidence: clampd(coverage * reliability, 0.0f64, 1.0f64),
        };
    } else {
        *planning = AerodynamicModel {
            lift_to_drag: (*baseline).estimated_lift_to_drag,
            ballistic_coefficient: (*baseline).estimated_ballistic_coefficient,
            confidence: 0.0f64,
        };
    }
    return supported_regimes > 0 as ::core::ffi::c_uint;
}
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const DBL_MIN: ::core::ffi::c_double = __DBL_MIN__;
pub const __DBL_MIN__: ::core::ffi::c_double = 2.2250738585072014e-308f64;
