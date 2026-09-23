#include "landing.h"

#include <float.h>
#include <math.h>

static bool certification_finite_vector(Vector3 v) {
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

/*
 * Regime boundaries are the midpoints of the predictor's interpolation anchors.
 * They are a discretization identity, not a separate flight-behavior threshold.
 */
static int certified_regime(double mach) {
    static const double anchors[] = {0.35, 1.05, 2.6, 6.0};
    for (int i = 0; i < 3; ++i)
        if (mach < 0.5 * (anchors[i] + anchors[i + 1])) return i;
    return 3;
}

static bool certified_sample_parameters(
        const VesselAeroSample *sample,
        const VehicleProfile *baseline,
        double *lift_to_drag,
        double *ballistic_coefficient,
        double *precision_weight,
        double *independent_weight) {
    if (!sample || !baseline ||
        sample->gear || sample->brakes || sample->airbrakes != 0 ||
        !(sample->force_per_q.x > 0.0) ||
        !(sample->mass > 0.0) ||
        !certification_finite_vector(sample->force_per_q) ||
        !isfinite(sample->mach) ||
        !isfinite(sample->aoa) ||
        !isfinite(sample->beta))
        return false;

    /*
     * The clean-airframe reference-factor model is identified only inside the
     * vehicle's configured incidence envelope. Outside observations remain
     * available to local interpolation; they do not redefine global factors.
     */
    double incidence = hypot(sample->aoa, sample->beta);
    if (!(incidence <= baseline->maximum_angle_of_attack))
        return false;

    double lift_factor = 0.0;
    double drag_factor = 1.0;
    aerodynamic_force_factors_mach(
        sample->mach, sample->aoa, baseline,
        &lift_factor, &drag_factor);

    /*
     * A reference L/D is unobservable where the model lift factor is zero, and
     * ballistic coefficient is unobservable where the drag factor is zero.
     * These are mathematical identifiability boundaries.
     */
    if (!(drag_factor > DBL_MIN) ||
        !(fabs(lift_factor) > DBL_MIN))
        return false;

    double observed_ld =
        fabs(sample->force_per_q.y) / sample->force_per_q.x;
    double observed_beta =
        sample->mass / sample->force_per_q.x;
    double ld =
        observed_ld * drag_factor / fabs(lift_factor);
    double beta =
        observed_beta * drag_factor;
    if (!(ld > 0.0) || !(beta > 0.0) ||
        !isfinite(ld) || !isfinite(beta))
        return false;

    double trust =
        clampd(isfinite(sample->trust) ? sample->trust : 0.0, 0.0, 1.0);
    double independent =
        (double)(sample->observations ? sample->observations : 1U);
    double weight = trust * independent;
    if (!(weight > 0.0)) return false;

    *lift_to_drag = ld;
    *ballistic_coefficient = beta;
    *precision_weight = weight;
    *independent_weight = independent;
    return true;
}

bool vessel_physics_derive_envelope(
        VesselPhysicsModel *model,
        const VehicleProfile *baseline,
        AerodynamicEnvelope *envelope,
        AerodynamicModel *planning) {
    if (!model || !baseline || !envelope || !planning)
        return false;

    double precision[4] = {0};
    double independent[4] = {0};
    double ld_sum[4] = {0};
    double log_beta_sum[4] = {0};

    for (int regime = 0; regime < 4; ++regime) {
        envelope->regimes[regime] = (AerodynamicModel){
            baseline->estimated_lift_to_drag,
            baseline->estimated_ballistic_coefficient,
            0.0
        };
    }

    for (unsigned i = 0; i < model->count; ++i) {
        double ld, beta, weight, sample_independent;
        if (!certified_sample_parameters(
                &model->samples[i], baseline,
                &ld, &beta, &weight, &sample_independent))
            continue;

        int regime = certified_regime(model->samples[i].mach);
        precision[regime] += weight;
        independent[regime] += sample_independent;
        ld_sum[regime] += weight * ld;
        log_beta_sum[regime] += weight * log(beta);
    }

    double ld_mean[4] = {0};
    double beta_mean[4] = {0};
    for (int regime = 0; regime < 4; ++regime) {
        if (!(precision[regime] > 0.0)) continue;
        ld_mean[regime] = ld_sum[regime] / precision[regime];
        beta_mean[regime] =
            exp(log_beta_sum[regime] / precision[regime]);
        envelope->regimes[regime].lift_to_drag = ld_mean[regime];
        envelope->regimes[regime].ballistic_coefficient =
            beta_mean[regime];
    }

    double scatter_sum[4] = {0};
    double scatter_weight[4] = {0};
    double global_scatter_sum = 0.0;
    double global_scatter_weight = 0.0;

    for (unsigned i = 0; i < model->count; ++i) {
        double ld, beta, weight, sample_independent;
        if (!certified_sample_parameters(
                &model->samples[i], baseline,
                &ld, &beta, &weight, &sample_independent))
            continue;
        (void)sample_independent;

        int regime = certified_regime(model->samples[i].mach);
        if (!(precision[regime] > 0.0) ||
            !(ld_mean[regime] > 0.0) ||
            !(beta_mean[regime] > 0.0))
            continue;

        double ld_relative =
            (ld - ld_mean[regime]) / ld_mean[regime];
        double beta_log =
            log(beta / beta_mean[regime]);
        /*
         * These are dimensionless multiplicative parameter residuals. Their
         * RMS is measured model scatter; no guessed percentage floor/ceiling
         * is inserted.
         */
        double error_squared =
            0.5 * (ld_relative * ld_relative +
                   beta_log * beta_log);
        scatter_sum[regime] += weight * error_squared;
        scatter_weight[regime] += weight;
        global_scatter_sum += weight * error_squared;
        global_scatter_weight += weight;
    }

    unsigned supported_regimes = 0;
    double confidence_sum = 0.0;
    for (int regime = 0; regime < 4; ++regime) {
        if (!(precision[regime] > 0.0) ||
            !(independent[regime] > 0.0))
            continue;

        supported_regimes++;
        double rms_scatter =
            scatter_weight[regime] > 0.0
                ? sqrt(scatter_sum[regime] /
                       scatter_weight[regime])
                : 0.0;
        double standard_error =
            rms_scatter / sqrt(independent[regime]);
        double mean_trust =
            precision[regime] / independent[regime];

        /*
         * Confidence is evidence reliability divided by one plus measured
         * relative standard error. Missing regimes contribute zero to the
         * global coverage average below.
         */
        double confidence =
            clampd(mean_trust / (1.0 + standard_error), 0.0, 1.0);
        envelope->regimes[regime].confidence = confidence;
        confidence_sum += confidence;
    }

    /*
     * Global confidence explicitly includes Mach-envelope coverage: each
     * unsupported regime contributes zero. Certified uncertainty is the larger
     * of measured model scatter and missing/reliability coverage, so neither
     * can be hidden by a tuned blend.
     */
    model->certified_confidence = confidence_sum / 4.0;
    double empirical_scatter =
        global_scatter_weight > 0.0
            ? sqrt(global_scatter_sum / global_scatter_weight)
            : 0.0;
    model->certified_uncertainty =
        fmax(empirical_scatter, 1.0 - model->certified_confidence);

    /*
     * Entry planning prefers the two high-Mach interpolation regimes when
     * either is represented; otherwise it falls back to all represented
     * regimes. Planning confidence is reliability times the literal fraction
     * of target regimes actually covered.
     */
    int first_regime =
        (precision[2] > 0.0 || precision[3] > 0.0) ? 2 : 0;
    int last_regime = 3;
    unsigned target_regimes =
        (unsigned)(last_regime - first_regime + 1);
    unsigned planning_supported = 0;
    double planning_weight = 0.0;
    double planning_ld = 0.0;
    double planning_log_beta = 0.0;

    for (int regime = first_regime;
         regime <= last_regime; ++regime) {
        if (!(precision[regime] > 0.0)) continue;
        planning_supported++;
        double confidence = envelope->regimes[regime].confidence;
        double weight = precision[regime] * confidence;
        if (!(weight > 0.0)) continue;
        planning_weight += weight;
        planning_ld +=
            weight * envelope->regimes[regime].lift_to_drag;
        planning_log_beta +=
            weight * log(
                envelope->regimes[regime].ballistic_coefficient);
    }

    if (planning_weight > 0.0) {
        double coverage =
            (double)planning_supported / (double)target_regimes;
        double reliability_numerator = 0.0;
        double reliability_denominator = 0.0;
        for (int regime = first_regime;
             regime <= last_regime; ++regime) {
            if (!(precision[regime] > 0.0)) continue;
            reliability_numerator +=
                precision[regime] *
                envelope->regimes[regime].confidence;
            reliability_denominator += precision[regime];
        }
        double reliability =
            reliability_denominator > 0.0
                ? reliability_numerator / reliability_denominator
                : 0.0;

        *planning = (AerodynamicModel){
            planning_ld / planning_weight,
            exp(planning_log_beta / planning_weight),
            clampd(coverage * reliability, 0.0, 1.0)
        };
    } else {
        *planning = (AerodynamicModel){
            baseline->estimated_lift_to_drag,
            baseline->estimated_ballistic_coefficient,
            0.0
        };
    }

    return supported_regimes > 0;
}
