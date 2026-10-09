#!/usr/bin/env python3
"""Build a separate offline STS fixture with a local vertical-profile shape."""
import argparse
import hashlib
import json
import os
import pathlib
import shlex
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[1]
NATIVE = ROOT / "CLanding"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--fraction", type=float, required=True,
                    help="local altitude offset as fraction of start-to-exit drop")
    ap.add_argument("--output-dir", type=pathlib.Path, required=True)
    ap.add_argument("--base-shape", choices=("hermite", "ramp"), default="hermite")
    ap.add_argument("--match-search-shape", action="store_true",
                    help="size and rank route leads with the ramp base's actual peak slope")
    args = ap.parse_args()
    if not -0.1 <= args.fraction <= 0.1:
        ap.error("use a finite local offset within +/-10%% of altitude drop")
    if args.match_search_shape and args.base_shape != "ramp":
        ap.error("matching experimental search requires the ramp base shape")
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    original = NATIVE / "terminal_solver.c"
    source = original.read_text()
    marker = "    trial.profile_local_offset_m = 0.0;"
    assert source.count(marker) == 1
    source = source.replace(marker, f"""    trial.profile_local_offset_m = {args.fraction:.17g} * drop;
    trial.profile_local_start_fraction = 0.0;
    trial.profile_local_peak_fraction = 0.20;
    trial.profile_local_end_fraction = 0.55;""")
    marker = "    route->profile_local_offset_m=0.0;"
    assert source.count(marker) == 1
    source = source.replace(marker, """    route->profile_local_offset_m=trial.profile_local_offset_m;
    route->profile_local_start_fraction=trial.profile_local_start_fraction;
    route->profile_local_peak_fraction=trial.profile_local_peak_fraction;
    route->profile_local_end_fraction=trial.profile_local_end_fraction;""")
    candidate = out / "terminal_solver.c"
    candidate.write_text(source)
    env = dict(os.environ, KSP_LANDER_MODEL_PROFILE="sts-provisional",
               KSP_LANDER_TAEM_DIAGNOSTICS="1",
               KSP_LANDER_TAEM_PROFILE_DIAGNOSTICS="1")
    # Read actual repository compiler/link flags. Only the solver object is
    # substituted; no baseline objects or production source are overwritten.
    make = ["make", "-f", "Makefile", "-f", "../Validation/STSRepairTests.mk"]
    subprocess.run(make + ["build/mm305_configured_tests"], cwd=NATIVE,
                   env=env, check=True, stdout=subprocess.DEVNULL)
    dry = subprocess.check_output(make + ["-n", "-B", "build/terminal_solver.o",
             "build/mm305_configured_tests"], cwd=NATIVE, text=True)
    commands = [shlex.split(line) for line in dry.replace("\\\n", " ").splitlines()
                if line.startswith("clang ")]
    compile_commands = [cmd for cmd in commands if "terminal_solver.c" in cmd and "-c" in cmd]
    link_commands = [cmd for cmd in commands if "../Validation/MM305FeasibilityPropertyTests.c" in cmd]
    assert compile_commands and len(link_commands) == 1
    obj = out / "terminal_solver.o"
    binary = out / "mm305_local_profile"
    compile_cmd = [str(candidate) if item == "terminal_solver.c" else
                   str(obj) if item == "build/terminal_solver.o" else item
                   for item in compile_commands[0]]
    link_cmd = [str(obj) if item == "build/terminal_solver.o" else
                str(binary) if item == "build/mm305_configured_tests" else item
                for item in link_commands[0]]
    compilation = [compile_cmd]
    if args.base_shape == "ramp":
        planner = (NATIVE / "taem_planner.c").read_text()
        marker = "    double slope = derivative_x / total_length + initial_slope + local_slope;"
        assert planner.count(marker) == 1
        # Integrate a continuous slope: 10% smooth entry ramp, constant middle,
        # 10% smooth exit ramp. Its integral fixes the exact endpoint altitude.
        replacement = """    const double ramp = 0.10;
    double middle_slope = ((h1-h0)/total_length -
        0.5*ramp*(slope0+slope1))/(1.0-ramp);
    double base_height, base_slope, base_second;
    if (x < ramp) {
        double u=x/ramp;
        double integral=u*u*u-0.5*u*u*u*u;
        base_height=h0+total_length*ramp*(slope0*u+
            (middle_slope-slope0)*integral);
        base_slope=slope0+(middle_slope-slope0)*u*u*(3.0-2.0*u);
        base_second=(middle_slope-slope0)*6.0*u*(1.0-u)/ramp;
    } else if (x <= 1.0-ramp) {
        base_height=h0+total_length*(0.5*ramp*(slope0+middle_slope)+
            middle_slope*(x-ramp));
        base_slope=middle_slope;
        base_second=0.0;
    } else {
        double u=(x-(1.0-ramp))/ramp;
        double integral=u*u*u-0.5*u*u*u*u;
        base_height=h0+total_length*(0.5*ramp*(slope0+middle_slope)+
            middle_slope*(1.0-2.0*ramp)+ramp*(middle_slope*u+
            (slope1-middle_slope)*integral));
        base_slope=middle_slope+(slope1-middle_slope)*u*u*(3.0-2.0*u);
        base_second=(slope1-middle_slope)*6.0*u*(1.0-u)/ramp;
    }
    height=base_height+midpoint_offset*bow+local_offset*local_bow+
        initial_sag*initial_bow;
    derivative_x=total_length*base_slope+midpoint_offset*bow_derivative_x;
    second_derivative_x=total_length*base_second+
        midpoint_offset*bow_second_derivative_x;
""" + marker
        planner_source = out / "taem_planner.c"
        planner_source.write_text(planner.replace(marker, replacement))
        planner_obj = out / "taem_planner.o"
        planner_commands = [cmd for cmd in commands if "taem_planner.c" in cmd and "-c" in cmd]
        assert planner_commands
        compilation.append([str(planner_source) if item == "taem_planner.c" else
            str(planner_obj) if item == "build/taem_planner.o" else item
            for item in planner_commands[0]])
        link_cmd = [str(planner_obj) if item == "build/taem_planner.o" else item for item in link_cmd]
    if args.match_search_shape:
        search = (NATIVE / "taem_candidate_search.c").read_text()
        start = search.index("static double hermite_peak_descent_deg(")
        end = search.index("static double minimum_vertical_profile_length(", start)
        search = search[:start] + """static double hermite_peak_descent_deg(const TaemRoute *route,
        double length_m) {
    if (!route || !(length_m > 0.0)) return INFINITY;
    const double rad=3.14159265358979323846/180.0;
    const double ramp=0.10;
    double m0=tan(route->profile_start_fpa_deg*rad);
    double m1=-tan(route->profile_final_slope_deg*rad);
    double middle=((route->profile_final_altitude_m-
        route->profile_start_altitude_m)/length_m-
        0.5*ramp*(m0+m1))/(1.0-ramp);
    return fmax(0.0,-atan(fmin(middle,fmin(m0,m1)))/rad);
}

""" + search[end:]
        search_source = out / "taem_candidate_search.c"
        search_source.write_text(search)
        search_obj = out / "taem_candidate_search.o"
        search_commands = [cmd for cmd in commands if "taem_candidate_search.c" in cmd and "-c" in cmd]
        assert search_commands
        compilation.append([str(search_source) if item == "taem_candidate_search.c" else
            str(search_obj) if item == "build/taem_candidate_search.o" else item
            for item in search_commands[0]])
        link_cmd = [str(search_obj) if item == "build/taem_candidate_search.o" else item for item in link_cmd]
    for cmd in compilation + [link_cmd]:
        subprocess.run(cmd, cwd=NATIVE, env=env, check=True)
    command = [str(binary), "--configuration", "Configuration/sts.json",
               "--mass-kg", "25816.072266", "--aero",
               "ShuttleSim/reference-model/experiments/sts_api_incidence.csv"]
    report = {"classification": "offline local profile shape experiment; not flight qualification",
              "fraction": args.fraction, "localFractions": [0.0, 0.20, 0.55],
              "baseShape": args.base_shape,
              "searchShapeMatched": args.match_search_shape,
              "candidateSearchSHA256": digest(out / "taem_candidate_search.c")
                  if args.match_search_shape else digest(NATIVE / "taem_candidate_search.c"),
              "candidatePlannerSHA256": digest(out / "taem_planner.c")
                  if args.base_shape == "ramp" else digest(NATIVE / "taem_planner.c"),
              "originalSourceSHA256": digest(original),
              "candidateSourceSHA256": digest(candidate), "binarySHA256": digest(binary),
              "baselineFixtureSHA256": digest(NATIVE / "build/mm305_configured_tests"),
              "nativeObjectSHA256": {item: digest((NATIVE / item).resolve())
                  for item in link_cmd if item.endswith(".o")},
              "compileCommands": compilation, "linkCommand": link_cmd,
              "command": command, "configurationSHA256": digest(ROOT / command[2]),
              "aeroSHA256": digest(ROOT / command[-1]), "status": "running"}
    manifest = out / "experiment.json"
    manifest.write_text(json.dumps(report, indent=2) + "\n")
    log = out / "fixture.log"
    with log.open("w") as stream:
        result = subprocess.run(command, cwd=ROOT, env=env, stdout=stream,
                                stderr=subprocess.STDOUT)
    report.update(status="completed", exitCode=result.returncode, logSHA256=digest(log))
    manifest.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
