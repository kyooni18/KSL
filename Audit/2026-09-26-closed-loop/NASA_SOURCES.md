# NASA source register and mechanism notes

Retrieved 2026-09-26. PDF hashes are in `provenance.json`; working PDFs/text are
under `/tmp/ksp-nasa`. No Earth-specific numerical loads are adopted.

* [Harpold/Graves, NTRS 19790037248](https://ntrs.nasa.gov/citations/19790037248):
  record/abstract located; NTRS supplies no full-text download. The abstract
  describes analytic drag-reference generation with damped tracking. Full-paper
  study remains outstanding; the memorandum below supplies detailed equations.
* [TM-81093/JSC-11746](https://ntrs.nasa.gov/api/citations/19800018907/downloads/19800018907.pdf):
  full text retrieved. Section 3.4 and Appendix A specify entry phases, range
  prediction, transition, alpha selection, gain scheduling and bank smoothing.
  This is an MCC formulation, not a claim of exact late flight-software identity.
* [TM-81094/JSC-11786 rev. 2](https://ntrs.nasa.gov/api/citations/19800018908/downloads/19800018908.pdf):
  sections 4.1.5–4.1.8 provide range-based altitude, energy and qbar references,
  altitude-rate feedback, transition logic and bank commands. The altitude
  reference joins the autoland slope tangentially. S-turn range prediction
  evaluates the path if the turn ended now; it is not an indefinite orbit.
* [TM-80796/JSC-16522](https://ntrs.nasa.gov/api/citations/19800016874/downloads/19800016874.pdf):
  sections 3–4 establish rotating local/runway frames, excess-energy S-turns,
  minimum-entry-point low-energy operation, optional overhead/straight-in
  targeting, and latched overhead-side selection with low-energy downmode.
* [TM-104744](https://ntrs.nasa.gov/api/citations/19920010688/downloads/19920010688.pdf):
  sections 2–2.1 explain Hybrid TAEM and why rigid total-energy control gave
  poor altitude/qbar allocation during wind reversals. Altitude and altitude-rate
  control with normal acceleration replaced it; energy/qbar bounds remained
  supervisory constraints. KSP adaptation must replace speedbrake authority
  with physically qualified path choices, not silently enable the effector.
* [CR-150994](https://ntrs.nasa.gov/api/citations/19760026174/downloads/19760026174.pdf):
  summary and feedback discussion explain integral correction for persistent
  drag-reference bias (including atmosphere and roll-deadband effects), with
  conditions where integration is disabled. Supports retaining bounded bias
  feedback, not increasing unverified lift authority.
* [CR-151480](https://ntrs.nasa.gov/api/citations/19770022264/downloads/19770022264.pdf):
  summary and sections 2–3 tie reversals to delta azimuth, current bank direction,
  and lateral deadband. A minimum-bank constraint prioritizes crossrange when
  needed; the numerical schedules were evaluated with L/D dispersions.
* [CR-151362 GEMASS](https://ntrs.nasa.gov/api/citations/19770017858/downloads/19770017858.pdf):
  introduction, DAP interfaces and dispersion appendix separate guidance,
  attitude response, trim/aero and integration. Comparison to a more detailed
  simulator is part of validation; a direct attitude command is not sufficient
  evidence of achievable FCS behavior.
* [Entry GN&C verification, 19820030348](https://ntrs.nasa.gov/citations/19820030348):
  record/abstract only; no NTRS PDF. Test-matrix and flight comparison are
  documented at abstract level, including remaining sideslip/bank/preflare
  mismatches. Full text remains outstanding.
* [Autoland design, 19820056897](https://ntrs.nasa.gov/citations/19820056897)
  and [automatic landing, 19820062072](https://ntrs.nasa.gov/citations/19820062072):
  records/abstracts located, no NTRS PDFs. Do not claim full-paper review.
  The FSSR below supplies the detailed longitudinal/lateral implementation.
* [GN&C development history, 20110014833](https://ntrs.nasa.gov/api/citations/20110014833/downloads/20110014833.pdf):
  full text retrieved; verification and lessons sections emphasize flight
  instrumentation, uncertainty models, transitions and integrated testing.

## Surviving flight-software specification

[STS 83-0001-34, Entry Through Landing Guidance, December 14, 2007](https://www.ibiblio.org/apollo/Shuttle/sts83-0001-34%20-%20Guidance%20Entry%20Through%20Landing.pdf)
is a Boeing/NASA-contract FSSR preserved by the ibiblio Shuttle archive.
Title/date verified in the downloaded document. Sections 4.6.2.6–4.6.2.9 cover
TAEM references, transitions, normal acceleration and lateral control;
4.7.2.1.1–4.7.2.1.6 cover trajectory generation/capture, steep glide, preflare,
shallow glide, final flare and speedbrake. Detailed equation review will be
recorded alongside each subsequent implementation decision; locating this
document alone is not architecture verification.
