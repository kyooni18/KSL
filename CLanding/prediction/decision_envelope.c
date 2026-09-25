#include "decision_envelope.h"
#include "taem_route.h"

#include <float.h>
#include <math.h>


/* Decision envelopes remain one translation unit to preserve shared static math,
 * while the source is organized by physical decision responsibility. */
#include "decision/basic_envelopes.inc"
#include "decision/response.inc"
#include "decision/authority.inc"
#include "decision/geometry.inc"
#include "decision/path_energy.inc"
#include "decision/taem_capture.inc"
