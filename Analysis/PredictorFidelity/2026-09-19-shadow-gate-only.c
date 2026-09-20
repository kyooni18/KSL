#define main entry_predictor_supervision_full_main
#include "../../Validation/EntryPredictorSupervisionTests.c"
#undef main
int main(void){
    test_entry_guidance_shadow_records_mm304_50km_checkpoint();
    puts("Shadow 50km checkpoint test passed.");
    return 0;
}
