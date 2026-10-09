# Yaw response and bank continuity

The human requested abort after recovery-v3 developed yaw/roll oscillations. Its runner 73594 and backend 73962 were absent on inspection; the last console record was UT 2124.1, 31.1 km, Attitude Recovery. No landing was validated.

Atmospheric entry yaw now closes measured sideslip rather than quaternion yaw error from the changing bank reference. Recovery retains quaternion yaw and body-rate arrest; runway heading holds retain heading control. Removed the empirical bank-dependent rudder bias. Yaw bandwidth is capped by available angular acceleration divided by error and actuator response lag, so large errors reduce acquisition bandwidth instead of increasing it.

A shrinking bank envelope previously clamped the reference twice: inside the lateral reference generator and again in the entry contract. Both clamps bypassed the reference rate/acceleration limits. The envelope now limits the destination, while the command unloads through the bounded reference generator. Bank reversals additionally wait for physical rest-to-rest bank-exchange time (and any executive minimum dwell), even if the previous leg was momentarily captured.

Validation: native build /tmp/sts-yaw-response-backend succeeded. FlightControlYawTests, FlightControlQuaternionTests, and EntryLateralFeedbackTests passed. Focused regression covers a collapsing bank envelope and a premature repeat reversal; the quaternion sign check now verifies that entry yaw ignores bank-frame yaw error at zero beta. These tests verify controller invariants, not live stability or landing. The next live attempt must validate yaw damping and bank continuity through reversal from the verified new 72 km checkpoint.
