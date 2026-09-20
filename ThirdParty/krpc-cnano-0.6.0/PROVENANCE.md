# Vendored kRPC C-Nano provenance

This directory contains the exact kRPC C-Nano 0.6.0 source tree supplied by the user in `krpc-cnano-0.6.0.zip`, plus the pinned nanopb dependency required to compile the pre-generated kRPC protobuf sources.

- kRPC C-Nano version: 0.6.0 (`VERSION.txt`)
- User archive SHA-256: `4edd712b13680e8797ec5e61d01ce0e719631304e3a0100c78c9316e0aa57fb3`
- kRPC license files preserved verbatim: `LICENSE`, `COPYING`, `COPYING.LESSER`
- nanopb version: 0.4.9.1, matching the tag pinned by upstream C-Nano 0.6.0 `CMakeLists.txt`
- nanopb source release: `https://github.com/nanopb/nanopb/releases/download/nanopb-0.4.9.1/nanopb-0.4.9.1.tar.gz`
- nanopb release SHA-256: `882cd8473ad932b24787e676a808e4fb29c12e086d20bcbfbacc66c183094b5c`
- nanopb license preserved at `vendor/nanopb-0.4.9.1/LICENSE.txt`

The nanopb tarball was fetched once for vendoring and its checksum was verified before extraction. Builds and tests use only these vendored files; CMake FetchContent is not required.

C-Nano 0.6.0 expects nanopb headers at paths such as `<krpc_cnano/pb.h>`. `vendor/nanopb-compat/krpc_cnano/` therefore contains copies of the root nanopb public headers, mirroring the compatibility copy performed by upstream C-Nano's own CMake build.

For the Shuttle project, production semantics remain C-Nano's serial-port protocol. `CLanding/krpc_cnano_transport.h` defines `KRPC_COMMUNICATION_CUSTOM` and the custom connection types. Compile C-Nano core translation units with that header pre-included and omit upstream `src/communication.c`; link `CLanding/krpc_cnano_transport.c` plus the desired byte backend. The offline test backend is `Validation/CNanoFakeTransport.c`. It never opens a device or network connection.

No pre-existing owned transport/vendor files were present when this integration was created, so there was no production file to back up for this job.
