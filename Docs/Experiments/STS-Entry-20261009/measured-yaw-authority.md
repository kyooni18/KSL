# Yaw authority follows measured response

V6 improved roll braking but still developed divergent sideslip and saturated rudder after reversal. Inspection found yaw retained the same unconditional preference for positive reported torque/inertia authority that roll previously used. In normal atmospheric guidance, the controlled yaw coordinate is negative measured sideslip, rather than body yaw during a coordinated turn.

Yaw now uses the same gradual inverse-authority blend as roll. Live observation confidence grows with input magnitude and elapsed time relative to configured yaw lag, discounted by simultaneous roll input. Offline seeds do not count as live observations. Existing sideslip signs, physical admission gates, and recovery ownership remain unchanged.

Backend build and FlightControlYawTests, FlightControlQuaternionTests, and EntryLateralFeedbackTests pass. One regression verifies gradual reduction under sustained weak beta-coordinate response. The existing test verifies cross-axis excitation reduces identification confidence.

This is a candidate correction, not a demonstrated cure for coupled departure. Net-response identification includes aerodynamic disturbances and cannot isolate rudder effectiveness. The active v6 still runs its pinned earlier backend. Candidate binary: /tmp/sts-measured-yaw-authority-backend.

Follow-up: yaw identification now resets its learned gain, live confidence, disturbance estimate, and derivative sample when switching among beta, heading, and body-yaw coordinates. Reported physical authority remains available. The weak-response regression also verifies beta observations cannot survive a switch to heading hold. All three focused suites and the backend build pass after this change.
