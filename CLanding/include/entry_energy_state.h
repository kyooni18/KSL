#ifndef KSP_LANDER_ENTRY_ENERGY_STATE_H
#define KSP_LANDER_ENTRY_ENERGY_STATE_H

#include <stdbool.h>

/* State of the retired MM304 energy/drag-tracking law; kept only because
   GuidanceMachine embeds it (layout shared with the Rust backend). */
typedef struct {
    bool initialized;
    double integral_m_s;
    double last_ut;
} EntryEnergyState;

#endif
