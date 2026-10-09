# Measured roll authority

The v5 reversal exposed a mismatch between reported torque/inertia authority and observed braking response. Previously, any positive reported authority excluded learned roll authority from the control law.

Roll control now blends inverse reported and observed authority using confidence earned by current-flight response. Offline seeds cannot displace reported authority. Confidence grows with input magnitude and elapsed time relative to configured roll lag; simultaneous rudder reduces observation weight. This lets sustained weak response increase braking input without installing gains fitted to this STS run.

Validation: backend build and FlightControlQuaternionTests, FlightControlYawTests, and EntryLateralFeedbackTests pass. One regression verifies gradual adoption of sustained weak roll response; the existing stale-seed test still passes.

The observer measures net response, including aerodynamic disturbances and cross-axis coupling, rather than isolated actuator gain. Negative inferred gains remain rejected. This change has not yet been validated in a live flight. The active v5 process uses its pinned earlier backend and remains running under the authorized warning-continuation policy.
