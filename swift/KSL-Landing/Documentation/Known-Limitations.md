# Unfinished and unverified functionality

This file defines the limits of the delivered implementation. The source is functional and built/tested; the following items prevent describing it as a fully qualified live KSP landing system or a robust solution over an arbitrary entry envelope.

| Area | Implemented behavior | Unfinished or unverified portion |
| --- | --- | --- |
| Live KSP landing | Native discovery, snapshots, control batches, shutdown, and reconnection API | No real KSP connection, touchdown, or rollout was performed. There is no verified live landing success. |
| Entry robustness | Receding-horizon search, both bank signs, modeled handoff evaluation, correction after rejected handoff | Some perturbations in the supplied matrix still fail. In particular, the 1,950 m/s synthetic entry is not recovered. A rejected finite search is not global proof of physical impossibility. |
| Actual attitude and degraded control | Bounded target bank/alpha, rate feedback, surface saturation diagnostics | The 70° requested bank bound does not prevent low-q transient overshoot; saved samples reach 89.35°. Reducing actuator authority by 35% produces a 6.072 m/s impact and modeled crash. Online moment/actuator identification is not implemented. |
| Terminal robustness | Complete nominal path pricing, trim/lift/q/load/rate constraints, two force stress corners | Stress-corner feasibility is reported but not required for plan commitment. Some nominally successful routes fail one or both corners. This is not robust optimization. |
| Physical identification | Latest supplied STS polar, direct-input pitch parameters, archival inertia, provisional roll/yaw | Missing raw live calibration sessions and a malformed observations database prevent a fresh held-out force/moment accuracy estimate. Roll/yaw, sideslip, inertia, and wheel parameters need independent live identification. |
| Contact and rollout | Gear-height crossing, touchdown quality, centerline/containment checks, binary brakes, ground steering | Flat runway and simplified nose lowering; no suspension, tire slip, gear extension transient, brake fade, structural impact damage, or terrain mesh. Successful modeled rollout is not live wheel qualification. |
| Airbrake detection | The configured action group is forced off and observed | The client does not enumerate every airbrake part/module or infer craft-specific action-group assignments. Correct group mapping in the actual craft is unverified. |
| KSP environment | Kerbin identity/radius/mu checks; stock atmosphere resources | FAR/custom atmosphere, other bodies, modded aerodynamic environments, wind inference from the live API, and craft variants are not qualified. |
| Native protocol | Bounded kRPC 0.6 request/response subset and independent TCP fault fixture | No general generated SDK, streams/events, automatic arbitrary-service support, or installed-server flight verification. |
| Timing | Single ordered runtime, bounded socket deadlines, stale-state rejection, per-step computation timing | General-purpose OS scheduling and occasional planning spikes; no hard real-time guarantee. Performance on the user's host and under KSP load is unmeasured. |
| Platforms | Swift 6.0.3 Linux x86_64 build/test; Darwin branches and macOS 13 target | No macOS build/run evidence, no Windows socket implementation, no mobile target. |
| Failure recovery | Classified aborts, explicit cleanup, no command replay after lost acknowledgement, read-only reconnect | No autonomous diversion, emergency landing controller, state transfer across reconnect, or guaranteed control recovery after a broken socket. |
| Memory qualification | Acyclic ARC ownership, bounded histories, weak-release test, lifecycle/cancellation tests | No Instruments session, long-duration heap profiling, or allocator-stress certification on a production host. |
| Historical replay | Exact full-rate native replay; conservative schema-5 import and scenario reconstruction | Damaged records cannot be recreated. Reconstructed attitude/missing rates and mixed simulator provenance prevent claiming command-identical legacy live replay. |
| Whole-flight hazards | q/load/trim and runway constraints | Aerothermal heating, damage progression, detailed terrain collision avoidance, navigation uncertainty, and hardware failures are outside the current model. |

## Meaning of a reported failure

The matrix preserves every requested scenario and all modeled aborts. A low-energy state 120 km from the runway is intentionally rejected. The heavy STS-N final fixture is a vehicle/model mismatch, not a nominal STS qualification case. Other failed entries expose the remaining search/controller envelope. They are not removed from the denominator or relabeled as successful protective landings.

An abort can be the correct classification after a state becomes unrecoverable, while the preceding guidance may still have made the state unrecoverable. The report therefore separates correct rejection from successful mission completion. Neutralizing commands does not mean the aircraft landed safely.

## Evidence needed for stronger claims

The next qualification steps are concrete: build/run on the intended host; verify actual kRPC procedures, coordinate signs, airbrake group, and timing using read-only observations and controlled surface-response data; identify moment/gear parameters; compare held-out force and trajectory measurements; extend the feasible entry envelope; require or redesign around appropriate uncertainty margins; and demonstrate repeated actual touchdown and stop under the same restrictions. No result in this delivery substitutes for those measurements.

All current numerical outcomes, including failed cases, are in [Validation.md](Validation.md), [matrix.json](../Validation/matrix.json), and the per-case reports. A successful synthetic case is evidence about this code/model combination, not a certification of the real craft.
