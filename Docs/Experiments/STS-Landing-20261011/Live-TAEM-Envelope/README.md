# Measured TAEM envelope and planner assumptions

Controlled L5/L6 landings followed TAEM paths outside the native smooth
planner's 24-degree reference-slope family. They spent about 130 seconds
steeper than −24 degrees and reached roughly −35 degrees before recovering
for Final. Original L7 also descended steeply through the transonic band,
but reached the alignment vicinity with substantially less speed and altitude.
This supports investigating a different vertical profile family under load,
q, incidence and response constraints. It does not justify relaxing gates or
qualify either flight against all current touchdown acceptance requirements.

| Measured TAEM | L5 | L6 | Original L7 |
|---|---:|---:|---:|
| Duration before first Final, s | 249.5 | 249.8 | 437.9; no Final |
| Time with actual FPA below −24°, s | 129.84 | 130.08 | 30.36 |
| Steepest actual FPA, degrees | −35.75 | −34.65 | −34.43 |
| Approximate speed near aligned 8 km station, m/s | 141.56 | 142.29 | 101.32 |
| Altitude at that sample, m MSL | 2989 | 2975 | 2585 |
| Actual FPA at that sample, degrees | −25.76 | −25.69 | −17.57 |
| Speed at first Final handoff, m/s | 129.02 | 130.96 | no Final |
| Along-track at first Final, m | −6244 | −6260 | no Final |

The aligned station sample is the nearest along-track sample to −8000 m among
TAEM samples below 5 km altitude and within 300 m cross-track. It is not an
interpolated crossing, complete capture test or qualification result. Exact
positions and times are retained in result.json. L5/L6 speed near this station
is within the planner's existing ±12% window around 160 m/s (lower edge
140.8 m/s), so the later 129–131 m/s handoff must not be misrepresented as a
same-station comparison. Their FPA is still steeper than the planned −20°
Final endpoint, and neither sample proves full nominal capture.

The original L7 inlet was 87 km from the site and 32.5 km cross-track, versus
about 60 km and 2.6 km in L5. Steep descent alone does not fix that lateral
energy burden. The measured L5/L6 paths have a sustained steep segment and
later recovery, rather than simply a steeper endpoint-to-endpoint Hermite
curve. A future candidate must propagate that shape with lateral geometry,
aerodynamics and actuator response, then pass the unchanged Final/landing
qualification. The old successful completion flags do not validate current
nominal touchdown-speed acceptance; those live landings were slow.

[analyze.py](analyze.py) passively decodes keyframes/deltas from original native
L5/L6/L7 logs, excludes repeated/backward UT, and integrates duration only for
gaps no larger than one second. The first TAEM packet has zero duration so
prior-phase time is not counted. Logged actual FPA is distinct from the
planner's reference slope; allowing a different reference family would be a
behavioral change requiring justification and full verification.

Run `python3 Docs/Experiments/STS-Landing-20261011/Live-TAEM-Envelope/analyze.py`
from the repository root. [result.json](result.json) records source/config
hashes, exact inlet/alignment/Final states, per-Mach-band coverage, maxima
and steepest-descent joint observations. CSVs preserve a 5-second history
plus key events. No source, test, gate or production configuration changed,
and no new flight was performed. This is evidence for profile design, not
simulator or plant qualification.
