#!/usr/bin/env python3
"""Render scientific validation figures from recorded Swift simulation output.

Requires Matplotlib. This script does not run or implement flight guidance.
"""
import argparse
import gzip
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from validate import runway_coordinates


COLORS = {"entry": "#4567be", "acquisition": "#9473b5", "headingAlignment": "#009b99",
          "approach": "#559a48", "preflare": "#df9d29", "flare": "#e26441",
          "rollout": "#565d6c", "complete": "#565d6c", "abort": "#bb3030"}


def records(root, name):
    with gzip.open(root/"logs"/(name+".jsonl.gz"), "rt") as source:
        data = [json.loads(line) for line in source]
    scenario = json.loads((root/"scenarios"/(name+".json")).read_text())
    config = scenario["configuration"]
    origin = data[0]["state"]["time"]
    rows = []
    for record in data:
        state, guidance = record["state"], record["guidance"]
        x, y, h = runway_coordinates(state["position"], config["runway"], config["planet"]["radius"])
        p, v = state["position"], state["velocity"]
        radius = math.sqrt(sum(a*a for a in p.values()))
        vz = sum(p[k]*v[k] for k in "xyz")/radius
        rows.append({"t": state["time"]-origin, "x": x, "y": y, "h": h,
                     "speed": state["aerodynamics"]["airspeed"], "sink": -vz,
                     "alpha": math.degrees(state["aerodynamics"]["angleOfAttack"]),
                     "bank": math.degrees(state["aerodynamics"]["bank"]),
                     "targetBank": math.degrees(guidance["target"]["bank"]),
                     "targetHeight": guidance["target"]["desiredHeight"], "phase": guidance["phase"]})
    return rows, config


def series(rows, key, scale=1):
    return [r[key]/scale for r in rows]


def style(ax, title, xlabel, ylabel):
    ax.set_title(title, loc="left", fontsize=12, weight="bold", pad=10)
    ax.set_xlabel(xlabel); ax.set_ylabel(ylabel)
    ax.grid(color="#e1e5eb", linewidth=.65)
    ax.spines[["top", "right"]].set_visible(False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--validation", default="Validation")
    args = parser.parse_args()
    root = Path(args.validation); dest = root/"figures"; dest.mkdir(exist_ok=True)
    matrix = json.loads((root/"matrix.json").read_text())
    choices = [r for r in matrix["cases"] if r["outcome"] == "complete" and r["spiralFlown"] and "--entry" in r["arguments"]]
    if not choices:
        raise SystemExit("No complete entry-through-spiral result exists; refusing to draw a success figure")
    chosen = choices[0]; name = chosen["case"]
    data, config = records(root, name)
    plan = json.loads((root/"plans"/(name+".json")).read_text())
    plt.rcParams.update({"font.size": 10, "font.family": "DejaVu Sans", "axes.labelcolor": "#303c4d",
                         "text.color": "#263447", "figure.facecolor": "white"})
    fig, axs = plt.subplots(2, 2, figsize=(13.6, 9.2), layout="constrained")
    fig.suptitle("Entry to runway stop | native Swift closed-loop simulation", fontsize=19, weight="bold")
    ax = axs[0, 0]
    for phase in dict.fromkeys(r["phase"] for r in data):
        selected = [r for r in data if r["phase"] == phase]
        ax.scatter(series(selected, "x", 1000), series(selected, "y", 1000), s=5,
                   color=COLORS.get(phase, "gray"), label=phase, rasterized=True)
    ax.scatter([0], [0], color="#202b3a", marker="|", s=100)
    style(ax, "A  Ground track", "Along runway chart (km)", "Right of runway chart (km)")
    ax.legend(fontsize=8, ncol=2, frameon=False); ax.set_aspect("equal", adjustable="datalim")
    ax = axs[0, 1]
    ax.plot(series(data,"t"), series(data,"h",1000), color="#4567be", label="height")
    style(ax, "B  Entry energy descent", "Elapsed simulation time (s)", "Height above runway (km)")
    other = ax.twinx(); other.plot(series(data,"t"), series(data,"speed"), color="#dd8947", linewidth=1.2)
    other.set_ylabel("Airspeed (m/s)", color="#b66a2e"); other.spines["top"].set_visible(False)
    ax = axs[1, 0]
    terminal = [r for r in data if r["phase"] != "entry"]
    stations = plan["stations"]
    ax.plot([s["position"]["x"]/1000 for s in stations], [s["position"]["y"]/1000 for s in stations],
            "--", color="#939aa7", linewidth=1.3, label="committed reference")
    for phase in dict.fromkeys(r["phase"] for r in terminal):
        selected = [r for r in terminal if r["phase"] == phase]
        ax.scatter(series(selected,"x",1000), series(selected,"y",1000), s=7, color=COLORS.get(phase,"gray"))
    ax.plot([0,config["runway"]["length"]/1000],[0,0], color="#202b3a",linewidth=3,label="runway")
    style(ax,"C  Variable-radius HAC and final", "Along runway chart (km)", "Right of runway chart (km)")
    ax.set_aspect("equal",adjustable="datalim");ax.legend(fontsize=8,frameon=False)
    ax = axs[1, 1]
    ax.plot(series(data,"t"),series(data,"bank"),color="#009b99",linewidth=1.2,label="observed bank")
    ax.plot(series(data,"t"),series(data,"targetBank"),color="#009b99",alpha=.4,linewidth=.8,label="target bank")
    ax.plot(series(data,"t"),series(data,"alpha"),color="#9473b5",linewidth=1,label="observed alpha")
    style(ax,"D  Attitude response", "Elapsed simulation time (s)", "Angle (degrees)")
    ax.legend(fontsize=8,frameon=False)
    fig.supxlabel(f"Case: {name} | offline model only; no live KSP qualification | touchdown {chosen['touchdownAlongMeters']:.1f} m, {chosen['touchdownHorizontalMps']:.1f} m/s, sink {chosen['touchdownSinkMps']:.2f} m/s",fontsize=10)
    fig.savefig(dest/"entry-and-landing.png",dpi=160);plt.close(fig)

    data, config = records(root,"nominal-final")
    fig, axs = plt.subplots(3, 1, figsize=(12,9.5),sharex=True,layout="constrained")
    fig.suptitle("Approach, flare, touchdown, rollout | offline STS model",fontsize=18,weight="bold")
    data = [r for r in data if r["x"] > -3000]
    phase_starts = {}
    for r in data: phase_starts.setdefault(r["phase"],r["x"])
    axs[0].plot(series(data,"x"),series(data,"h"),color="#4567be",label="observed height")
    flight = [r for r in data if r["phase"] not in {"rollout","complete"}]
    axs[0].plot(series(flight,"x"),series(flight,"targetHeight"),"--",color="#8895ac",label="reference height")
    style(axs[0],"A  Approach and flare profile", "", "Height (m)");axs[0].legend(frameon=False)
    axs[1].plot(series(data,"x"),series(data,"speed"),color="#009b99")
    style(axs[1],"B  Speed and runway stop", "", "Airspeed (m/s)")
    axs[2].plot(series(data,"x"),series(data,"sink"),color="#e26441")
    axs[2].axhline(3,color="#999",linestyle=":",linewidth=1,label="touchdown sink limit")
    style(axs[2],"C  Descent and contact", "Along runway (m); threshold = 0", "Sink rate (m/s)")
    axs[2].legend(frameon=False)
    for ax in axs:
        ax.axvline(0,color="#404a5a",linewidth=.8)
        for phase,x in phase_starts.items():
            if phase in {"preflare","flare","rollout"}:
                ax.axvline(x,color=COLORS[phase],linewidth=.8,alpha=.6)
    for phase,x in phase_starts.items():
        if phase in {"preflare","flare","rollout"}:
            axs[0].text(x,axs[0].get_ylim()[1]*.85,phase,rotation=90,color=COLORS[phase],fontsize=9,va="top")
    final = next(r for r in matrix["cases"] if r["case"] == "nominal-final")
    fig.supxlabel(f"Nominal final: touchdown {final['touchdownAlongMeters']:.1f} m, cross-track {final['touchdownCrossMeters']:.3f} m, sink {final['touchdownSinkMps']:.2f} m/s | stop {final['stopAlongMeters']:.1f} m\nMillimetre-scale cross-track reflects a symmetric idealized simulation, not expected live accuracy.",fontsize=10)
    fig.savefig(dest/"approach-and-rollout.png",dpi=160);plt.close(fig)
    print(json.dumps({"entryCase":name,"figures":[str(p) for p in sorted(dest.glob("*.png"))]}))


if __name__ == "__main__":
    main()
