import json
import time


v = vessel
f = v.flight()
last = None

while True:
    try:
        active = space_center.active_vessel
        if active is not None and active != v:
            v = active
            f = v.flight()

        control = v.control
        ut = float(space_center.ut)
        pitch = float(f.pitch)
        roll = float(f.roll)
        heading = float(f.heading)

        if last is None:
            pitch_rate = 0.0
            roll_rate = 0.0
            heading_rate = 0.0
        else:
            dt = max(ut - last[0], 1e-3)
            pitch_rate = (pitch - last[1]) / dt
            roll_rate = ((roll - last[2] + 180.0) % 360.0 - 180.0) / dt
            heading_rate = ((heading - last[3] + 180.0) % 360.0 - 180.0) / dt

        last = (ut, pitch, roll, heading)
        print(
            json.dumps(
                {
                    "ut": round(ut, 2),
                    "situation": str(v.situation).split(".")[-1],
                    "alt": round(float(f.mean_altitude), 1),
                    "radar": round(float(f.surface_altitude), 1),
                    "verticalSpeed": round(float(f.vertical_speed), 2),
                    "trueAirSpeed": round(float(f.true_air_speed), 2),
                    "dynamicPressure": round(float(f.dynamic_pressure), 1),
                    "pitch": round(pitch, 2),
                    "roll": round(roll, 2),
                    "heading": round(heading, 2),
                    "pitchRate": round(pitch_rate, 2),
                    "rollRate": round(roll_rate, 2),
                    "headingRate": round(heading_rate, 2),
                    "angleOfAttack": round(float(f.angle_of_attack), 2),
                    "sideslip": round(float(f.sideslip_angle), 2),
                    "controlPitch": round(float(control.pitch), 3),
                    "controlRoll": round(float(control.roll), 3),
                    "controlYaw": round(float(control.yaw), 3),
                    "sas": bool(control.sas),
                    "sasMode": str(control.sas_mode).split(".")[-1],
                },
                separators=(",", ":"),
            ),
            flush=True,
        )
    except Exception as exc:
        print(
            json.dumps(
                {"observerError": f"{type(exc).__name__}: {str(exc)[:160]}"},
                separators=(",", ":"),
            ),
            flush=True,
        )

    time.sleep(0.5)
