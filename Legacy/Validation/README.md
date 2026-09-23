# Archived validation probes

This directory contains historical narrow tests and architecture probes retired from the normal `Validation/` surface during the CLanding modularization.

They are preserved for archaeology and regression reconstruction only. Some encode assumptions about the former flat `CLanding/*.c` layout, private helper visibility, or build targets that no longer exist. They do not define the current production architecture and are not part of `make -C CLanding test`.

If an old failure must be reconstructed, port the behavior being protected to a small public-interface regression instead of restoring obsolete source layout or exporting implementation-only symbols.
