# Landing videos

`Tools/landing_video.py` turns a macOS screen recording of a KSL landing into a video with a telemetry HUD burned in.
It also writes a cropped copy with no HUD.

```bash
pip install -e '.[video]'          # Pillow, numpy; ffmpeg/ffprobe from Homebrew
python3 Tools/landing_video.py "~/Desktop/화면 기록 2026-10-22 오후 1.34.58.mov" --preview 60 800 1290 1555
python3 Tools/landing_video.py "~/Desktop/화면 기록 2026-10-22 오후 1.34.58.mov" --name STS-landing
```

`--preview` writes SDR JPG stills and exits; check them before the full render. A full render runs at about real time (26 min for a 26 min flight).

## What it does automatically

- **Sync:** reads the recording start from the macOS file name (Korean `화면 기록 … 오전/오후 h.mm.ss` or English
  `Screen Recording … at h.mm.ss AM/PM`), else from the file's birth time. Override with `--start-epoch`, and nudge with `--offset`.
- **Data:** uses the `runs/<id>` whose telemetry overlaps the recording (`--run`), and the `FlightLogs/*-vehicle.jsonl`
  session started within 2 min of it (`--vehicle-log`).
- **Crop:** finds the window on the recording's black background and cuts the title bar at its dark separator line.
  It also insets 10 px past the rounded corners. Use `--crop W:H:X:Y` to set the box yourself, or `--crop none` for an input that is already cropped.
- **HDR:** an HDR10 recording (PQ/BT.2020) stays 10-bit PQ through to HEVC Main10. The HUD is converted from sRGB into PQ at
  the recording's MaxCLL, else 203 nits (`--white-nits`; KSP game white measured ~100). SDR recordings are composited as BT.709. HLG is refused.

## HUD

- **Top left:** backend phase and mission clock (T+ from the first telemetry sample).
- **Bottom left:** a navball (attitude indicator) with heading, plus ALT / SPD / MACH / RNG.
  - Pitch, heading and roll are the vehicle log's surface-frame attitude.
  - The marker is the surface-relative velocity vector: green prograde, or amber retrograde when the velocity is behind the nose (deorbit burn).
  - Without a vehicle log it falls back to FPA+AoA. That fallback is wrong whenever the nose points away from the velocity.
- **Top right:** north-up map.
  - Beyond 45 km it shows the ground track (latitude ×40) to KSC.
  - Inside 45 km it switches to the runway 09 frame: along-track to the right, and positive crosstrack (south) drawn down.
- **Bottom right:** the last three decisions from `events.jsonl`: phase changes, plus the planner's degraded/uncertified/cannot-reach verdicts in amber.
