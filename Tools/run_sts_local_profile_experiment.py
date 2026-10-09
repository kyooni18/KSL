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
    args = ap.parse_args()
    if not -0.1 <= args.fraction <= 0.1:
        ap.error("use a finite local offset within +/-10%% of altitude drop")
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
    for cmd in (compile_cmd, link_cmd):
        subprocess.run(cmd, cwd=NATIVE, env=env, check=True)
    command = [str(binary), "--configuration", "Configuration/sts.json",
               "--mass-kg", "25816.072266", "--aero",
               "ShuttleSim/reference-model/experiments/sts_api_incidence.csv"]
    report = {"classification": "offline local profile shape experiment; not flight qualification",
              "fraction": args.fraction, "localFractions": [0.0, 0.20, 0.55],
              "originalSourceSHA256": digest(original),
              "candidateSourceSHA256": digest(candidate), "binarySHA256": digest(binary),
              "baselineFixtureSHA256": digest(NATIVE / "build/mm305_configured_tests"),
              "nativeObjectSHA256": {item: digest((NATIVE / item).resolve())
                  for item in link_cmd if item.endswith(".o")},
              "compileCommand": compile_cmd, "linkCommand": link_cmd,
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
