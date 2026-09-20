#include "flybrain.h"
#include "simulator.h"

#include <stdint.h>
#include <stdio.h>

static void print_aggregate(const char *name, const FlyBrainAggregate *a) {
    printf("%s\n", name);
    printf("  stable fraction:       %.4f\n", a->stable_fraction);
    printf("  mean RMS error:        %.3f deg\n", a->mean_rms_error_deg);
    printf("  mean final error:      %.3f deg\n", a->mean_final_error_deg);
    printf("  worst final error:     %.3f deg\n", a->worst_final_error_deg);
    printf("  worst body rate:       %.3f deg/s\n", a->worst_rate_deg_s);
    printf("  mean control activity: %.3f /s\n", a->mean_control_activity);
    printf("  saturation fraction:   %.5f\n", a->saturation_fraction);
    printf("  mean cost:             %.3f\n", a->mean_cost);
}

int main(int argc, char **argv) {
    const char *path = argc > 1 ? argv[1] : "trained_params.txt";
    FlyBrainParams params;
    if (!flybrain_params_load(path, &params)) {
        fprintf(stderr, "could not load %s; run `make train` first\n", path);
        return 2;
    }

    const FlyBrainParams baseline_params = flybrain_default_params();
    const FlyBrainAggregate baseline_normal = flybrain_evaluate(&baseline_params,
                                                                 UINT64_C(0x3c6ef372fe94f82b),
                                                                 2000,
                                                                 false);
    const FlyBrainAggregate baseline_hard = flybrain_evaluate(&baseline_params,
                                                               UINT64_C(0xa54ff53a5f1d36f1),
                                                               2000,
                                                               true);

    const FlyBrainAggregate normal = flybrain_evaluate(&params,
                                                        UINT64_C(0x3c6ef372fe94f82b),
                                                        2000,
                                                        false);
    const FlyBrainAggregate hard = flybrain_evaluate(&params,
                                                      UINT64_C(0xa54ff53a5f1d36f1),
                                                      2000,
                                                      true);
    print_aggregate("untuned baseline / normal", &baseline_normal);
    print_aggregate("untuned baseline / hard", &baseline_hard);
    print_aggregate("normal envelope (2000 unseen episodes)", &normal);
    print_aggregate("hard envelope (2000 unseen episodes)", &hard);

    const int normal_ok = normal.stable_fraction >= 0.995 &&
                          normal.mean_final_error_deg <= 3.0 &&
                          normal.saturation_fraction <= 0.12;
    const int hard_ok = hard.stable_fraction >= 0.98 &&
                        hard.mean_final_error_deg <= 5.0 &&
                        hard.saturation_fraction <= 0.20;
    const int improvement_ok = normal.mean_cost < baseline_normal.mean_cost &&
                               hard.mean_cost < baseline_hard.mean_cost &&
                               normal.stable_fraction >= baseline_normal.stable_fraction &&
                               hard.stable_fraction >= baseline_hard.stable_fraction;

    if (!normal_ok || !hard_ok || !improvement_ok) {
        fprintf(stderr,
                "FlyBrain validation FAILED (normal=%d hard=%d improvement=%d)\n",
                normal_ok, hard_ok, improvement_ok);
        return 1;
    }
    printf("FlyBrain validation PASSED\n");
    return 0;
}
