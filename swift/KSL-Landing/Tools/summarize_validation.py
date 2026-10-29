#!/usr/bin/env python3
"""Generate the human-readable report and audit from saved validation evidence."""
import gzip
import hashlib
import json
import math
from pathlib import Path


def main():
    root = Path("Validation")
    matrix = json.loads((root/"matrix.json").read_text())
    replay = json.loads((root/"replay.json").read_text())
    wire = json.loads((root/"krpc-mock.json").read_text())
    expected_failures = {"entry-crossrange15km": "aborted", "entry-speed-plus50": "aborted",
                         "entry-actuator-minus35pct": "crashed", "low-energy-unreachable": "aborted",
                         "archival-heavy-final": "aborted"}
    actual_failures = {r["case"]: r["outcome"] for r in matrix["cases"] if r["outcome"] != "complete"}
    if actual_failures != expected_failures:
        raise SystemExit("Case outcomes changed. Update the engineering interpretations in this report template before regenerating it; raw JSON/CSV results remain valid.")
    for name, expected in matrix["sourceSHA256"].items():
        assert hashlib.sha256(Path(name).read_bytes()).hexdigest() == expected, f"Source changed after validation: {name}"
    audits = []
    for case in matrix["cases"]:
        maximum_bank, maximum_alpha, samples = 0.0, -math.inf, 0
        forbidden, airborne_brake_commands, nonfinite = 0, 0, 0
        with gzip.open(root/"logs"/(case["case"]+".jsonl.gz"), "rt") as stream:
            for line in stream:
                record = json.loads(line)
                state, command = record["state"], record["command"]
                systems, aero = state["systems"], state["aerodynamics"]
                samples += 1
                maximum_bank = max(maximum_bank, abs(math.degrees(aero["bank"])))
                maximum_alpha = max(maximum_alpha, math.degrees(aero["angleOfAttack"]))
                forbidden += int(systems["throttle"] > .0001 or systems["thrust"] > 1 or systems["sas"] or systems["rcs"] or systems["airbrakes"])
                airborne_brake_commands += int(state["contact"] == "airborne" and command["wheelBrake"] > 0)
                for value in [*state["position"].values(), *state["velocity"].values(), *state["attitude"].values(), *command.values()]:
                    nonfinite += int(not math.isfinite(value))
        audits.append({"case": case["case"], "recordedSamples": samples,
                       "sampledMaximumAbsoluteBankDegrees": maximum_bank,
                       "sampledMaximumAlphaDegrees": maximum_alpha,
                       "forbiddenSystemSamples": forbidden,
                       "airborneBrakeCommands": airborne_brake_commands,
                       "nonfiniteKinematicsAttitudeOrCommands": nonfinite})
    audit = {"scope": "Saved telemetry samples; 10 Hz nominal-final and 1 Hz other cases, with final records. Sample maxima are not continuous-time bounds.",
             "cases": audits, "allRecordedControlsRespectUnpoweredPolicy": all(r["forbiddenSystemSamples"] == r["airborneBrakeCommands"] == r["nonfiniteKinematicsAttitudeOrCommands"] == 0 for r in audits)}
    (root/"telemetry-audit.json").write_text(json.dumps(audit,indent=2)+"\n")
    assert audit["allRecordedControlsRespectUnpoweredPolicy"]
    def fmt(value, digits=1): return "—" if value is None else f"{value:.{digits}f}"
    lines = ["# Quantitative validation — final delivered source", "",
             "## Evidence and overall result", "",
             f"The final Swift 6.0.3 Linux x86_64 build completed successfully. All **37 Swift tests** (26 XCTest and 11 Swift Testing cases) and **{sum(x['passed'] for x in wire['cases'])} independent TCP protocol/failure cases** passed. **{matrix['passedLandingCases']} of {matrix['totalCases']} closed-loop scenario runs completed an acceptable touchdown and confirmed runway stop.** All six explicit regression gates passed.", "",
             "The 19 cases are a deterministic engineering matrix, not random samples from a probability distribution. The completion fraction is not a statistical reliability estimate. The intentionally unreachable case and the archived heavier-vehicle mismatch remain in the table. No real KSP connection or live landing was performed.", "",
             "Source SHA-256 values and the executable hash are in [matrix.json](../Validation/matrix.json). Test output is in [swift-tests.log](../Validation/swift-tests.log), toolchain identification in [toolchain.txt](../Validation/toolchain.txt), and independent wire results in [krpc-mock.json](../Validation/krpc-mock.json). These refer to the delivered source, not an earlier development experiment.", "",
             "## Complete case matrix", "",
             "Positions are metres from the Runway 09 threshold along the runway. Positive sink means downward velocity. Touchdown speed is horizontal ground speed. `complete` requires acceptable contact and a stop, not simply entering an approach phase. A `crashed` row can have contact measurements without an acceptable touchdown.", "",
             "| Case | Outcome | Spiral flown | Touchdown x (m) | Horizontal speed (m/s) | Sink (m/s) | Stop x (m) |",
             "| --- | --- | :---: | ---: | ---: | ---: | ---: |"]
    for r in matrix["cases"]:
        lines.append(f"| {r['case']} | **{r['outcome']}** | {'yes' if r['spiralFlown'] else 'no'} | {fmt(r['touchdownAlongMeters'])} | {fmt(r['touchdownHorizontalMps'],2)} | {fmt(r['touchdownSinkMps'],3)} | {fmt(r['stopAlongMeters'])} |")
    lines += ["", "Exact inputs are saved under `Validation/scenarios/`; reports, final plans, and telemetry are under `cases/`, `plans/`, and `logs/`. The CSV includes peak q/load, saturation, computation timing, and source report hashes. All dimensions and disturbances are also recorded in each row's arguments.", "",
              "The nominal synthetic entry starts 450 km before the runway at 72 km height, 1,900 m/s, and −3° flight-path angle. The heading case adds 8°. The downrange case starts 100 km farther away. The cross-range case adds 15 km right offset and 3° heading error. The low/high speed cases change entry speed by −/+50 m/s. The final-only baseline starts 10 km before the threshold at 4,200 m height, 150 m/s, and −24° flight-path angle.", "",
              "`required-spiral` starts 24 km before and 4 km right of the runway at 12 km height, 220 m/s, −20° heading error, and −24° flight-path angle, with direct terminal paths disabled. The two historical entry cases use the supplied recorded initial-condition INI files. Their new results remain offline simulations.", "",
              "## Failure interpretation", "",
              "- **Cross-range 15 km / heading 3°:** no complete terminal route was captured before the modeled optimistic range bound was exceeded. This exposes a remaining entry/search-envelope failure; it is not proof that the original state is physically unreachable.",
              "- **Entry speed 1,950 m/s:** entry energy/geometry did not converge to a complete terminal path; the mission aborted at low altitude. The bounded candidate policy is not qualified for this case.",
              "- **Actuator authority reduced by 35%:** contact occurred on the runway at 1,190.3 m and 83.07 m/s, but sink was 6.072 m/s, exceeding the 3 m/s contact limit. It is correctly classified as a crash, not a successful landing. Force-effectiveness correction does not identify or compensate all moment/actuator degradation.",
              "- **Low-energy unreachable:** the initial state is 120 km away, 1 km high, and 70 m/s. The modeled optimistic range is only about 6.33 km. Immediate rejection is the expected regression outcome; no landing is counted.",
              "- **Archived heavy final:** the 40,252.9 kg STS-N starting condition is not qualified by the 25 t STS model. No complete terminal capture was found; the run aborted. It is retained to make that model boundary visible.", "",
              "Classified failures and the last terminal rejection summary are present in each report. A correct abort classification does not establish that preceding guidance was optimal or that neutralization provides a safe emergency landing.", "",
              "## Spiral evidence", "",
              "The following successful runs entered and completed the heading-alignment segment. The radial parameter is `Rf + k*theta²`; local curvature radius is a different quantity. These particular flown spirals do not establish that every multi-revolution candidate can be flown. Geometry unit tests separately include 450° curves.", "",
              "| Case | Turn parameter (deg) | Entry radial parameter (m) | Final radial parameter (m) | Growth k (m/rad²) |",
              "| --- | ---: | ---: | ---: | ---: |"]
    for r in matrix["cases"]:
        if r["outcome"] == "complete" and r["spiralFlown"]:
            p=json.loads((root/"plans"/(r["case"]+".json")).read_text())["spiral"]
            lines.append(f"| {r['case']} | {math.degrees(p['turnAngle']):.0f} | {p['finalRadius']+p['growth']*p['turnAngle']**2:.1f} | {p['finalRadius']:.1f} | {p['growth']:.1f} |")
    lines += ["", "![Entry through variable-radius HAC and runway stop](../Validation/figures/entry-and-landing.png)", "",
              "## Numerical and resource checks", "",
              "The plant integrates translation with RK4 and midpoint attitude/rate at a nominal 20 ms substep; guidance/control runs at 100 ms. The vacuum orbital-energy relative-error test passes at 1e−9 tolerance over 1,000 s. The rotating-frame angular-momentum relative-error test passes at 1e−5 tolerance over 100 s. The 10 ms/20 ms final-approach refinement test completes both landings with touchdown-position difference below 15 m, horizontal-speed difference below 1 m/s, and sink difference below 0.2 m/s. These are test bounds, not asserted measured equalities to the limits.", "",
              f"Full-rate nominal-final replay processed **{replay['records']} records with {replay['commandOrGuidanceMismatches']} command/guidance mismatches**. The exact same `FlightComputer.step` generated the original and replayed outputs. Same-build determinism is demonstrated; bitwise identity across different architectures/compilers is not asserted. Wall-clock timing fields are intentionally not deterministic.", "",
              "Unit and lifecycle checks also cover frame round trips, handedness, drag removing energy, no artificial vacuum control, analytic spiral curvature, C2 joins/capture, impossible paths, initial curvature measured against integrated plant acceleration, normal pullout acceleration, force-estimator convergence, ARC release, malformed/oversized data, read/write failure, frozen telemetry, command sequence, cancellation, and explicit close.", "",
              f"The largest observed flight-computer step in the matrix was **{max(r['maximumComputationMilliseconds'] for r in matrix['cases']):.1f} ms**. Per-case mean compute times ranged from {min(r['meanComputationMilliseconds'] for r in matrix['cases']):.4f} to {max(r['meanComputationMilliseconds'] for r in matrix['cases']):.4f} ms. Planning spikes exceed the 100 ms nominal control period. The live adapter rejects commands older than 500 ms, but these host timings do not prove hard real-time operation under actual KSP load.", "",
              "## Envelope, model uncertainty, and interpretation", "",
              f"Across this matrix, maximum sampled controller-step q was **{max(r['maximumQPa'] for r in matrix['cases'])/1000:.2f} kPa**, and maximum normal load was **{max(r['maximumNormalG'] for r in matrix['cases']):.3f} g**. The configured q/load limits are 45 kPa and 3.5 g. These peaks do not include unmodeled heating or structural constraints.", "",
              f"All {sum(r['recordedSamples'] for r in audits):,} saved telemetry records complied with the unpowered-system policy and contained no airborne wheel-brake commands or nonfinite kinematics/attitude/commands. Full control validation also runs inside the Swift loop. Sampled absolute bank reached **{max(r['sampledMaximumAbsoluteBankDegrees'] for r in audits):.2f}°** despite a 70° target limit; low-q attitude transients can overshoot requested bank. The target bound is not an observed-attitude guarantee. Details are in [telemetry-audit.json](../Validation/telemetry-audit.json).", "",
              "Every selected terminal plan in this matrix fails at least one of its two configured opposing ±20% lift/drag corner assessments. Those failures are preserved as `modelUncertainty` warnings. Nominal completion and success in individual disturbance runs therefore do not constitute a robust-feasibility certificate. The corner tests are not measured confidence intervals.", "",
              "Reduced-lift and increased-drag final-only cases touch down more slowly than the nominal 85 m/s design speed. They remain acceptable under the implemented contact limits, but this does not establish stall, gear, or structural margin in KSP. The low-drag final stops at 2,231.1 m on a 2,500 m runway, leaving approximately 268.9 m; the reduced-lift final reaches 13.32° pitch, close to the 14° contact limit. Those margins matter more than a bare success count.", "",
              "The millimetre-scale cross-track values in symmetric runs reflect an idealized deterministic model, precise state, and smooth runway. They are not a prediction of live navigation accuracy. Ground suspension, wheel slip, airbrake part mapping, live moments, and real timing remain unverified.", "",
              "![Nominal approach, flare, touchdown, and rollout](../Validation/figures/approach-and-rollout.png)", "",
              "## Reproduction", "", "```sh", "bash Tools/run-validation.sh", "python3 Tools/plot_validation.py", "python3 Tools/summarize_validation.py", "```", "",
              "The scripts use only the built native Swift executable, Python's standard library, and Matplotlib for plots. The loopback fixture binds its own temporary port and does not contact an existing KSP server. The full-rate replay log can be decompressed from `Validation/logs/nominal-final.jsonl.gz`. All other matrix logs are sampled evidence and are not full-rate command-replay inputs.", "",
              "The precise outstanding functionality is listed in [Known-Limitations.md](Known-Limitations.md). Original-data limitations and raw log audit counts are in [Reference-Audit.md](Reference-Audit.md)."]
    Path("Documentation/Validation.md").write_text("\n".join(lines)+"\n")
    print(json.dumps({"sourceHashesMatch":True,"landingCases":matrix['passedLandingCases'],"totalCases":matrix['totalCases'],"telemetryRecords":sum(r['recordedSamples'] for r in audits),"report":"Documentation/Validation.md"}))


if __name__ == "__main__":
    main()
