/* Isolated terminal regressions; the complete entry suite remains separate. */
#define main full_guidance_suite_main
#include "GuidanceIntegrationContractTests.c"
#undef main

int main(void) {
    test_stale_regular_hac_preview_may_refresh_without_relaxing_geometry();
    test_recorded_late_taem_state_keeps_geometry_but_rejects_energy_commit();
    test_latched_taem_path_recovery_never_returns_to_entry();
    test_v_taem_handoff_geometry_and_mm305_s_turn_continuity();
    puts("PASS: terminal capture contracts and measured-energy rejection.");
    return 0;
}
