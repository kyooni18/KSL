# Correct the recorded center-of-mass coordinate frame

2026-11-03, Asia/Seoul. Source inspected: `abbe21c4` native kRPC adapter.

The adapter requested both Part_Rotation and Vessel_Rotation relative to the
rotating celestial-body frame. It rotated the anchor-relative CoM vector into
that external frame, then permuted axes and labelled the result aircraft FRD.
It had not removed the vessel attitude. The recorded vector therefore rotated
with the aircraft even though its label claimed body coordinates. The raw
`centerOfMassFromAnchor` measurement was separate and is preserved.

`KRPCFrameConverter.anchorOffsetFRD` now applies inverse vessel rotation after
part rotation, then converts the polar vector from kRPC right/forward/down to
FRD. Both input rotations refer to the same external frame. No new RPC is needed;
the measured vessel rotation already exists in the same bounded snapshot batch.
The body-frame rotation receives the existing quaternion norm check. Invalid
optional anchor rotation yields unavailable body CoM instead of silently
normalizing to an identity rotation.

New corrected records carry `centerOfMassBodyFrame=aircraftFRD`. Historical
records leave this tag nil: their property name is not sufficient evidence of
correct body coordinates. Do not silently repair or use old values as body-axis
moment arms. CoM remains an anchor-relative displacement expressed in body axes,
not an identified aerodynamic center of pressure.

One narrow geometry regression passed: applying an arbitrary common external
rotation to both part and vessel poses leaves the FRD offset unchanged and
preserves its norm. An identity pose verifies the polar-axis permutation.
Evidence: `Validation/TAEM-CoM-Frame-20261103/frame-invariance-test.txt`.
The later combined moment-support checks also include this regression.

No production consumer currently uses `centerOfMassBodyFRD` outside telemetry
construction/validation, as verified by source search. Dynamics invalidation
continues to use the original anchor-relative measurement. This change corrects
future model-identification evidence; it does not alter actuator authority or
claim that the native attitude model now closes. No live kRPC connection, flight,
save operation or release rebuild occurred.
