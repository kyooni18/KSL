#ifndef KSP_LANDER_KRPC_CNANO_BATCH_H
#define KSP_LANDER_KRPC_CNANO_BATCH_H

#include "krpc_cnano_transport.h"

#include <krpc_cnano/decoder.h>
#include <krpc_cnano/krpc.pb.h>

#include <stdbool.h>
#include <stddef.h>

#define KRPC_CNANO_BATCH_MAX 32u

/*
 * Execute several ordinary kRPC procedure calls in one serial-protocol
 * Request/Response pair. The wire schema is standard kRPC; the vendored
 * nanopb Request/Response fixed arrays are enlarged to KRPC_CNANO_BATCH_MAX.
 * Each result must have been initialized with krpc_init_result().
 *
 * A top-level Response.error still fails the whole transaction. Per-procedure
 * errors are reported through result_failed[] so callers can mix required and
 * optional telemetry without throwing away unrelated successful results.
 */
krpc_error_t krpc_cnano_invoke_batch(krpc_connection_t connection,
                                     const krpc_schema_ProcedureCall *calls,
                                     krpc_result_t *results,
                                     bool *result_failed,
                                     size_t count);

#endif
