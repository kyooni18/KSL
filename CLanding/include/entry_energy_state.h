#ifndef KSP_LANDER_ENTRY_ENERGY_STATE_H
#define KSP_LANDER_ENTRY_ENERGY_STATE_H

#include <stdbool.h>

/* Persistent state of the MM304 energy/drag-tracking law
   (entry_energy_control.h).  Kept dependency-free so GuidanceMachine can embed it. */
typedef struct {
    bool initialized;
    double integral_m_s;
    double last_ut;
} EntryEnergyState;

#endif
