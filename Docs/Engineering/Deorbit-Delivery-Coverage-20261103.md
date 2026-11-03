# Deorbit delivery coverage — 2026-11-03

## Observed failure

Authorized live run `runs/20261103T043851Z-advisory-follow-live` flew source `12f1c7af67d0fc7ef3159fe8214998d8e6ec7c5d`. Its selected Dessert 36 plan predicted atmospheric entry only 120,615.853 m from the runway, against the configured 1,030,000 m aim (909,593.416 m orbital-screen miss). The actual first entry sample was UT 18555.384419, altitude 69,497.42 m ASL, speed 2,101.55 m/s and range 110,866.648 m. Burn execution did not explain the roughly 910 km aim error.

The retained entry predictor had stopped at 69,494.8685 m ASL (68,673.9185 m above the runway), Mach 7.2085. Missing pitch-moment support prevents a priced continuation. The ranker nevertheless treated the nearest recorded point to its delivery altitude as a delivery state, rewarding the small horizontal distance of an incomplete atmospheric prefix. This was a selection defect upstream of advisory tracking.

## Change

Entry screens now explicitly retain delivery coverage. A track must contain the selected delivery point inside the existing configured delivery altitude band; track heights are converted from runway-relative to ASL before comparison. Missing coverage cannot purchase a cheap TAEM arrival. When both candidates lack coverage, inner window ordering and final runway ordering use the existing configured orbital entry objective, then impulse and time. Existing full-route qualification, physical safety ordering, propulsion limits and execution gates remain in place. Historical records without explicit coverage are treated as unknown.

## Decisive check

One regression uses the exact recorded plan fixture and existing orbital-only search from its measured source and propulsion. Of 12 orbital proposals, an available candidate had entry range 1,029,860.723 m, orbital-screen miss 14,978.170 m, delta-v 32.144886 m/s and burn start UT 17591.685329. Both ordering stages prefer it over the selected short-range plan when descent coverage is absent. These are orbital-screen results, not a simulated or live landing certificate.

Five focused checks passed: recorded coverage and final ordering, existing physical MM305 ranking, and three existing OMS budget/waiting checks. Evidence: `Validation/Deorbit-Delivery-Coverage-20261103/focused-tests.txt`. No capability perturbations or full-flight campaign were run. The existing live process retains its immutable original executable; this fix has not flown. Landing readiness remains unqualified.
