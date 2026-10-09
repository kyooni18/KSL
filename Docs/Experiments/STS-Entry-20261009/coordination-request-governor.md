# Slow attitude requests when coordination falls behind

The human identified aggressive attitude requests as the trigger for recent recoveries. V6 evidence supports that sequence: bank reversal, overshoot, growing beta and saturated rudder, then recovery.

The existing bank-reference coordination governor incorrectly required sideslip rate and roll rate to have the same sign. During v6 stopping, beta grew negative while roll remained positive; this condition bypassed the slowdown. The governor now reserves beta margin whenever sideslip grows outward, regardless of coupling polarity. Its existing acceleration-limited slowdown and configured sideslip envelope remain in effect. Improving beta releases the restriction. No vessel-specific rate or angle was introduced.

The existing coordination regression now checks both roll-rate signs with the same outward beta response. Backend build and EntryLateralFeedbackTests, FlightControlYawTests, and FlightControlQuaternionTests pass. Live validation remains pending; active v6 uses its pinned earlier controller.
