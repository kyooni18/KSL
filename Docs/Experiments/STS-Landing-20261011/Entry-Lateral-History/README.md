# Original entry lateral chronology

`analyze.py` decodes original live L5/L7 native state and reconstructs surface
course from inertial velocity minus planetary rotation. It records every
centreline crossing and sustained bank-command sign changes beyond ±5°;
that threshold labels chronology, not controller reversal ownership. CSVs
retain 10 s samples; `result.json` retains source hashes and event frames.
Cached groups have different update rates, so these are sampled events.

L5 enters MM304 at −6.93 km cross-track and hands off at −2.59 km, with no
centreline crossing. L7 enters at +101.44 km, crosses through zero at UT
6388.830, and hands off at −32.53 km. At L7's crossing it is 46.77 km high,
1948.22 m/s, course 78.82°, bank command +12.86°, actual body roll +13.46°.
The bank command had already exceeded +5° at UT 6386.570. This is not a
wrong-sign bank persisting after crossing: it is turning toward the runway,
with substantial lateral velocity still to arrest.

Course reaches about 90° only near UT 6538, when cross-track is about
−34.51 km; the later handoff recovers some crossrange but still leaves a large
lateral MM305 problem. At high entry speed and low lift, changing bank does
not instantly redirect velocity. The current allocator projects crossrange
through roll-exchange time, but omits the additional heading-arrest distance.
This motivates the paired [experimental projection](../Entry-Lateral-Arrest/README.md).
It does not prove that the new formula qualifies, or that deorbit targeting,
aero/history error and spherical geometry make no contribution.
