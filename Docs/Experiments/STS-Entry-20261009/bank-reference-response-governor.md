# Bank reference response governor

Live yaw-response-v4 reversal still departed. At UT 2081.95 the reference was +16.47 degrees while measured bank was +1.84; at UT 2098.19 the vessel reached +58.21 degrees against +26.48 command, beta -2.71 and yaw input -1. The configured time trajectory alone did not account for measured acquisition lag. This is observed behavior; cross-axis plant causality remains unresolved.

The bank reference now limits its requested rate using remaining braking reserve: reserve = maximum roll rate squared / (2 * maximum roll acceleration); lead is reference minus measured bank in the direction of travel. Desired rate is bounded by sqrt(2 * acceleration * remaining reserve), then the existing acceleration limiter applies. It waits when the airframe falls behind and resumes as measured bank catches up. No vessel-specific gain or angular threshold is introduced. This change is built separately; the active live controller remains pinned and unchanged.

Native build /tmp/sts-bank-response-governor-backend succeeds. EntryLateralFeedbackTests passes, including a focused nonresponding-airframe check followed by response capture/resumption. This establishes reference behavior, not live stability. MM305 gates remain unchanged.
