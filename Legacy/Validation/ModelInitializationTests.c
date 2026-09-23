#include "landing.h"

#include <assert.h>
#include <math.h>

/* models.o serializes executive telemetry, but this test exercises only model
   initialization. Keep it independent from executive implementations so an
   unrelated state-machine edit cannot block this low-level contract. */
const char *entry_phase_string(EntryPhase phase){(void)phase;return "test";}
const char *entry_transition_reason_string(EntryTransitionReason reason){(void)reason;return "test";}
const char *taem_phase_string(TaemPhase phase){(void)phase;return "test";}
const char *taem_transition_reason_string(TaemTransitionReason reason){(void)reason;return "test";}
const char *taem_recovery_reason_string(TaemRecoveryReason reason){(void)reason;return "test";}
const char *taem_terminal_block_reason_string(TaemTerminalBlockReason reason){(void)reason;return "test";}

int main(void){
    Telemetry t;
    telemetry_init(&t);

    /* Initializers describe absence of observations. They may initialize
       dimensionless correction factors to the identity, but must not invent a
       physical atmosphere measurement. */
    assert(t.speed_of_sound==0.0);
    assert(t.atmospheric_density==0.0);
    assert(t.dynamic_pressure==0.0);
    assert(t.mach==0.0);
    assert(t.trajectory_density_scale==1.0);
    assert(t.trajectory_drag_scale==1.0);
    assert(t.trajectory_lift_scale==1.0);
    assert(t.bank_effectiveness==1.0);

    LandingSnapshot snapshot;
    landing_snapshot_init(&snapshot,NULL);
    assert(snapshot.telemetry.speed_of_sound==0.0);
    assert(snapshot.connection_status==CONN_DISCONNECTED);
    assert(snapshot.phase==PHASE_IDLE);
    trajectory_clear(&snapshot.actual_trajectory);
    trajectory_clear(&snapshot.reference_trajectory);
    trajectory_clear(&snapshot.predicted_trajectory);
    return 0;
}
