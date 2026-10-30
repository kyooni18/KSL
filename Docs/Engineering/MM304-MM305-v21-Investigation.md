# MM304 → MM305: v21 runway-27 investigation

Investigation of `runs/live-swift-v21-rwy27-crossrange-checkpoint72-20261031/`.
Status: **not fixed; no landing demonstrated**. This is a forensic checkpoint,
not a request to change flight guidance based on a successful build.

## Evidence and reproduction

- Baseline `36c4c99` **plus** the run's `source.patch`. At investigation start,
  `git diff` was byte-identical to that patch. All 13 pre-existing modified
  source/test files are preserved, not included in this investigation commit.
- Configuration: `live.jsonl.configuration.json`, runway 27, spherical
  reciprocal threshold, `captureTargetRange=5000`, `maximumBank=70°`, RCS
  enabled by the vehicle configuration's default. No release rebuild or live
  operation was performed. Debug-only offline diagnostics are used.
- Foundation Memory was searched for **KSL TAEM capture**, then original
  sources on reciprocal geometry, the energy investigation, and HAC continuity
  were read. The old remote endpoint is stale; the existing local MCP client
  credential works with the local Foundation endpoint. No credentials belong
  in this report.
- Reconstruct the logged spherical frame and decoded proposals:

  ```sh
  python3 swift/KSL-Landing/Tools/investigate_transition.py \
    runs/live-swift-v21-rwy27-crossrange-checkpoint72-20261031 \
    --output runs/mm304-mm305-v21-investigation
  ```

  Outputs include exact event states, per-sample geometry/energy/acceleration,
  decoded proposals, and input SHA-256 hashes. Run artifacts remain local
  under ignored `runs/`; this report and analysis tools are tracked.

## Established first findings

Elapsed time is from UT **196014.14879776392**. Crossrange is positive right
of selected runway 27, i.e. approximately **north**, not south.

| Event | Elapsed s | ASL m | Mach | Along m | Cross m | Threshold distance m |
|---|---:|---:|---:|---:|---:|---:|
| First MM305 request | 463.92 | 23959.05 | 2.850 | 143.37 | 24892.37 | 24892.78 |
| First result / acquisition owns controls | 464.02 | 23950.10 | 2.846 | 65.70 | 24865.50 | 24865.59 |
| First `headingAlignment` label | 470.00 | 23446.44 | 2.609 | -4330.55 | 23223.73 | 23624.04 |
| First crossing of objective along=-5000 | 471.00 | 23358.37 | 2.589 | -5022.67 | 22939.11 | 23482.55 |
| Closest threshold | 477.18 | 22689.09 | 2.544 | -9205.12 | 21159.02 | 23074.63 |
| Closest objective | 483.94 | 21686.54 | 2.524 | -13645.55 | 19188.07 | 23545.34 |

Thus MM304 itself did **not** fly past the along=-5000 plane before MM305
owned control. This is not v18's missing-search failure. The closest objective
distance is **21045.64 m**, not the distance to KSC, the threshold, or an along
coordinate. Passing was not by itself disallowed or evidence of failure.

The first terminal result examined 54 candidates, with **zero completed,
zero feasible** candidates. It nevertheless supplied the advisory route, as
required by the non-veto policy. Selected reference: final radius 3000 m,
remaining winding 3.52628 rad, first spiral station 8944.14 m along the path.
Its full advisory pricing reports peak load **31.222 g**, peak bank **89.981°**,
and minimum lift margin **-292.111 m/s²**. Those are properties of the
reference, not loads or bank actually achieved by the vehicle. Acceptance
and the `headingAlignment` phase label are not capture certificates.

Further model/command reconstruction and bounded recovery conclusions follow
in the next investigation checkpoint. No guidance patch is justified solely
by these first observations.
