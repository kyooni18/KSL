# Dessert Airfield landing target

Select **Dessert Airfield · provisional** in the Landing tab (or the Console
landing settings). The selected site is passed to the native Swift backend and
the operation review identifies it. The configured site option retains the
site from the selected landing configuration. The backend may choose either
runway direction; this is not automatic selection between airfields.

The provisional runway 36 reference is the STS center position recorded at
UT 216874.4738: latitude -6.60031229594°, longitude -144.04084177757°,
surface elevation 820.95 m above Kerbin sea level. The heading 0.94276554°
comes from that point to the second measured position (-6.44721964676°,
-144.03830647736°). Their separation is 1,603.4 m. The user reported a
1,750 m runway length and runway 18/36 from the wiki; the wiki blocked direct
access. These measurements do not establish the actual pavement thresholds:
the reported length exceeds the surveyed separation by 146.6 m. The configured
36 threshold is therefore provisional, and the reciprocal threshold is derived
1,750 m from it, not independently surveyed.

The width of 42.6 m is a rough estimate: the user judged the runway about three
shuttle wingspans wide, and the active STS wing-part bounds measured 14.2 m.
It is not a direct pavement-edge measurement. The new site has not been
validated in an end-to-end simulation or live landing. Simulation scenarios
carry their own runway and cannot use this selector; direct Swift and legacy C
engines likewise do not support this selection. Existing scenario and flight
safety gates remain in force.
