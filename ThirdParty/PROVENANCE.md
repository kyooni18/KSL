# Vendored native dependencies

## kRPC C-Nano 0.6.0

Path: `ThirdParty/krpc-cnano-0.6.0`

Source: official kRPC C-Nano 0.6.0 source archive supplied for this migration.
The original archive SHA-256 is:

`4edd712b13680e8797ec5e61d01ce0e719631304e3a0100c78c9316e0aa57fb3`

The vendored tree reports version `0.6.0` in `VERSION.txt`. Its license is
LGPL-3.0-or-later; retain the upstream `LICENSE` and notices with redistributed
source or binaries as required by that license.

The application does not compile upstream `src/communication.c` into a POSIX
TCP client. It builds C-Nano with `KRPC_COMMUNICATION_CUSTOM` and supplies the
project-owned serial-protocol byte transport in
`CLanding/krpc_cnano_transport.c`.

## nanopb 0.4.9.1

Path: `ThirdParty/nanopb-0.4.9.1`

Source: upstream nanopb Git tag `0.4.9.1`, resolved at commit
`cad3c18ef15a663e30e3e43e3a752b66378adec1` when vendored. The `.git` directory
is intentionally omitted. The upstream `LICENSE.txt` is retained unchanged.

Only `pb.h`, `pb_common.*`, `pb_encode.*`, and `pb_decode.*` are linked into the
landing backend; the rest of the vendored source is kept for license,
reproducibility, and upstream build context.

### Host-side C-Nano batch-capacity patch

The upstream C-Nano nanopb options intentionally cap `Request.calls` and
`Response.results` at one for small embedded targets. This host-side landing
backend needs synchronous real-time telemetry without Streams, so the vendored
`krpc.options` and generated `krpc.pb.h` arrays are locally raised from 1 to 32.
The protobuf wire schema is unchanged. Ordinary generated C-Nano calls still
use one result; `CLanding/krpc_cnano_batch.c` uses the additional fixed capacity
to group landing telemetry into bounded multi-call requests. The original
archive SHA-256 above remains the provenance hash of the unmodified upstream
source.
