#!/usr/bin/env python3
"""Derive a four-ET-clamp STS pad save without modifying its source.

This narrowly targets the 41-part STS stack in the supplied quicksave: two
clamps attached to the orbiter's aft fuselage are moved onto the ET. Moving
the PART records after their new parent is essential for KSP to load the
whole vessel. Unexpected craft topology is rejected rather than guessed.
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path


PART = re.compile(r"(?ms)^\t\t\tPART\n\t\t\t\{\n.*?^\t\t\t\}\n")
VESSEL = re.compile(r"(?m)^\t\tVESSEL\n\t\t\{\n\t\t\tpid = [^\n]+\n"
                    r"\t\t\tpersistentId = [^\n]+\n\t\t\tname = STS\n")


def repair(source: str) -> str:
    matches = list(VESSEL.finditer(source))
    if len(matches) != 1:
        raise ValueError(f"Expected one STS vessel, found {len(matches)}")
    start = matches[0].start()
    end = source.index("\n\t\t}\n", start) + len("\n\t\t}\n")
    vessel = source[start:end]
    spans = [(match.start(), match.end()) for match in PART.finditer(vessel)]
    if len(spans) != 41:
        raise ValueError(f"Expected 41 STS parts, found {len(spans)}")
    parts = [vessel[a:b] for a, b in spans]
    if any(not re.search(r"(?m)^\t{4}name = launchClamp1$", parts[i]) or
           not re.search(r"(?m)^\t{4}parent = 4$", parts[i]) for i in (13, 14)):
        raise ValueError("Expected two aft-fuselage clamps at part indices 13 and 14")
    if not re.search(r"(?m)^\t{4}name = Size4.Tank.04$", parts[24]):
        raise ValueError("Expected external tank at part index 24")
    if sum(bool(re.search(r"(?m)^\t{4}name = benjee10.shuttle.deltaWing$", part))
           for part in parts) != 2:
        raise ValueError("Expected both orbiter wings")

    order = [*range(13), *range(15, 41), 13, 14]
    index = {old: new for new, old in enumerate(order)}

    def map_reference(match: re.Match[str]) -> str:
        old = int(match.group(2))
        return match.group(1) + str(index[old] if old >= 0 else old)

    rebuilt: list[str] = []
    for old in order:
        part = parts[old]
        part = re.sub(r"(?m)^(\t{4}(?:parent|sym) = )(-?\d+)", map_reference, part)
        part = re.sub(r"(?m)^(\t{4}(?:attN|srfN) = [^\n]*?, )(-?\d+)",
                      map_reference, part)
        if old in (13, 14):
            positive = old == 13
            position = (("2.2781293392181396,-10.0,6.8542776107788086") if positive else
                        ("-2.2781293392181396,-10.0,6.8542757034301758"))
            rotation = (("0.00138499821,0.923873663,-0.00334378611,-0.382680923") if positive else
                        ("-0.00138499832,0.923873544,-0.00334378728,0.382680923"))
            for key, value in (("parent", str(index[24])), ("position", position),
                               ("rotation", rotation), ("srfN", f"srfAttach, {index[24]},COL")):
                part, count = re.subn(rf"(?m)^\t{{4}}{key} = [^\n]+$",
                                      f"\t\t\t\t{key} = {value}", part, count=1)
                if count != 1:
                    raise ValueError(f"Moved clamp is missing {key}")
        rebuilt.append(part)
    repaired_vessel = vessel[:spans[0][0]] + "".join(rebuilt) + vessel[spans[-1][1]:]
    repaired_parts = [match.group() for match in PART.finditer(repaired_vessel)]
    if len(repaired_parts) != 41:
        raise ValueError("Repaired vessel lost a part")
    for number, part in enumerate(repaired_parts):
        parent = re.search(r"(?m)^\t{4}parent = (\d+)$", part)
        if parent is None or int(parent.group(1)) > number:
            raise ValueError(f"Invalid part parent at index {number}")
    if sum(bool(re.search(rf"(?m)^\t{{4}}parent = {index[24]}$", part)) and
           bool(re.search(r"(?m)^\t{4}name = launchClamp1$", part))
           for part in repaired_parts) != 4:
        raise ValueError("Repaired vessel must have four ET clamps")
    result = source[:start] + repaired_vessel + source[end:]
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="Original quicksave.sfs")
    parser.add_argument("output", type=Path, help="New derived .sfs path")
    args = parser.parse_args()
    text = args.source.read_text(encoding="utf-8-sig")
    result = repair(text)
    with args.output.open("x", encoding="utf-8") as handle:
        handle.write(result)
    print(f"Created {args.output}; source preserved")


if __name__ == "__main__":
    main()
