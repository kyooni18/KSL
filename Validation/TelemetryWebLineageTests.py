from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import tempfile
import types
import unittest

# These tests exercise pure planner-log lineage helpers only. The web observer
# imports kRPC eagerly even though no connection is used here; stub only that
# dependency and leave the real HUD module namespace untouched.
sys.modules.setdefault("krpc", types.SimpleNamespace())

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Tools"))

from telemetry_web import (
    PlannerFollower,
    RLTrainingMonitor,
    apply_exact_trajectory_geometry,
    actual_trajectory_can_resume,
    bearing_degrees,
    build_orbital_trajectory,
    load_actual_trajectory_history,
    load_actual_trajectory_from_vehicle_log,
    load_simulation_replay,
    projected_taem_trajectory,
    prune_actual_trajectory,
    save_actual_trajectory_history,
    taem_capture_veto_reasons,
)
from attach_live_hud import _load_latest_plan


class TelemetryWebLineageTests(unittest.TestCase):

    def test_rl_training_monitor_reads_filtered_condition_and_progress(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            output = root / "ShuttleSim" / "rl" / "results" / "monitor-test"
            output.mkdir(parents=True)
            reset = {
                "type": "reset",
                "seed": 47,
                "policy": "policy-a",
                "initial_condition": {
                    "apoapsis_altitude_m": 228000.0,
                    "periapsis_altitude_m": 156000.0,
                    "inclination_deg": 84.8,
                    "target_post_deorbit_periapsis_altitude_m": 67000.0,
                    "mm304_taem_screen": {
                        "entry_fpa_deg": -1.7,
                        "entry_speed_mps": 2410.0,
                        "entry_along_m": -160000.0,
                        "entry_cross_m": 8000.0,
                    },
                },
            }
            episode = {
                "type": "episode",
                "policy": "policy-a",
                "seed": 47,
                "phase": "ENTRY_ENERGY",
                "elapsed": 1200,
                "outcome": "timeout",
                "return": -250.0,
                "eligible_steps": 500,
                "active_steps": 400,
                "fallbacks": 2,
                "success": False,
                "rollout_valid": False,
                "metrics": {"touchdown": False},
            }
            train = output / "train.jsonl"
            train.write_text(
                json.dumps(reset) + "\n" + json.dumps(episode) + "\n",
                encoding="utf-8",
            )
            monitor = RLTrainingMonitor(root, Path("ShuttleSim/rl/results/monitor-test"))
            status = monitor.poll()
            self.assertTrue(status["available"])
            self.assertEqual(status["state"], "running")
            self.assertEqual(status["latestCondition"]["inclination_deg"], 84.8)
            self.assertEqual(status["activeSteps"], 400)
            self.assertEqual(status["latestEpisode"]["outcome"], "timeout")

            report = {"status": "smoke_only_unproven"}
            (output / "training-report.json").write_text(json.dumps(report), encoding="utf-8")
            finished = monitor.poll()
            self.assertEqual(finished["state"], "finished")
            self.assertEqual(finished["status"], "smoke_only_unproven")

    def test_simulation_replay_exposes_monotonic_elapsed_time_across_placeholder_ut(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            run_dir = root / "ShuttleSim" / "runs" / "timing-test"
            run_dir.mkdir(parents=True)
            replay_path = run_dir / "guidance-snapshots.jsonl"
            frames = [
                {"phase": "Idle", "telemetry": {"ut": 0.0}},
                {"phase": "Entry Interface", "telemetry": {"ut": 66550.512509}},
                {"phase": "Entry Interface", "telemetry": {"ut": 66559.312509}},
            ]
            replay_path.write_text(
                "".join(json.dumps(frame) + "\n" for frame in frames),
                encoding="utf-8",
            )
            manifest = {
                "runId": "timing-test",
                "state": "finished",
                "mode": "closed-loop",
                "prerollFinal": {"ut": 66550.412509},
                "guidanceSnapshots": str(replay_path),
            }
            (run_dir / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

            replay = load_simulation_replay(root, "timing-test", max_frames=20)
            self.assertIsNotNone(replay)
            compact = replay["frames"]
            elapsed = [frame["simulation"]["simElapsedSeconds"] for frame in compact]
            self.assertEqual(len(elapsed), 3)
            self.assertEqual(elapsed[0], 0.0)
            self.assertAlmostEqual(elapsed[1], 0.1, places=5)
            self.assertAlmostEqual(elapsed[2], 8.9, places=5)
            self.assertLessEqual(elapsed[0], elapsed[1])
            self.assertLessEqual(elapsed[1], elapsed[2])


    def test_simulation_replay_normalizes_exact_controller_frames_for_web_ui(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            config_dir = root / "Configuration"
            config_dir.mkdir(parents=True)
            config_path = config_dir / "replay.json"
            config_path.write_text(json.dumps({
                "site": {
                    "latitude": -0.0486,
                    "longitude": -74.7247,
                    "altitude": 70.0,
                    "runwayHeading": 90.0,
                },
                "guidance": {
                    "hacRadius": 12000.0,
                    "finalApproachDistance": 8000.0,
                    "taemGlideSlope": 12.0,
                    "finalGlideSlope": 20.0,
                },
            }), encoding="utf-8")

            run_dir = root / "ShuttleSim" / "runs" / "ui-schema-test"
            run_dir.mkdir(parents=True)
            replay_path = run_dir / "guidance-snapshots.jsonl"
            predicted = [
                {
                    "ut": 100.0,
                    "latitude": -0.2,
                    "longitude": -75.2,
                    "altitude": 30000.0,
                    "speed": 1200.0,
                    "phase": "ENTRY",
                },
                {
                    "ut": 110.0,
                    "latitude": -0.08,
                    "longitude": -74.9,
                    "altitude": 18000.0,
                    "speed": 500.0,
                    "phase": "TAEM",
                },
            ]
            reference = [
                {
                    "ut": 110.0,
                    "latitude": -0.08,
                    "longitude": -74.9,
                    "altitude": 18000.0,
                },
                {
                    "ut": 140.0,
                    "latitude": -0.05,
                    "longitude": -74.75,
                    "altitude": 5000.0,
                },
            ]
            frames = [
                {
                    "connectionStatus": "connected",
                    "tickSequence": 42,
                    "phase": "MM304 Entry",
                    "telemetry": {"ut": 100.0, "latitude": -0.2, "longitude": -75.2},
                    "plan": {"burnUT": 95.0, "trajectory": []},
                    "guidanceState": {
                        "entryPlanValid": True,
                        "entryPlanTerminalReady": True,
                        "terminalCandidateValid": False,
                    },
                    "predictedTrajectory": predicted,
                    "referenceTrajectory": reference,
                },
                {
                    "connectionStatus": "connected",
                    "tickSequence": 43,
                    "phase": "TAEM",
                    "telemetry": {"ut": 110.0, "latitude": -0.08, "longitude": -74.9},
                    "plan": {"burnUT": 95.0, "trajectory": []},
                    "guidanceState": {
                        "entryPlanValid": True,
                        "entryPlanTerminalReady": True,
                        "terminalPathCommitted": True,
                    },
                    "predictedTrajectory": predicted,
                    "referenceTrajectory": reference,
                },
            ]
            replay_path.write_text(
                "".join(json.dumps(frame) + "\n" for frame in frames),
                encoding="utf-8",
            )
            manifest = {
                "runId": "ui-schema-test",
                "state": "finished",
                "mode": "closed-loop",
                "config": str(config_path),
                "prerollFinal": {"ut": 100.0},
                "guidanceSnapshots": str(replay_path),
            }
            (run_dir / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

            replay = load_simulation_replay(root, "ui-schema-test", max_frames=20)
            self.assertIsNotNone(replay)
            entry, terminal = replay["frames"]

            self.assertEqual(entry["deorbitPlan"]["burnUT"], 95.0)
            self.assertEqual(entry["site"]["runwayHeading"], 90.0)
            self.assertEqual(entry["telemetry"]["vesselName"], "STS-N · ShuttleSim")
            self.assertEqual(entry["trajectoryRevision"], 42)
            self.assertEqual(len(entry["predictedTrajectory"]), 2)
            self.assertGreater(len(entry["projectedTAEMTrajectory"]), 1)
            self.assertEqual(entry["plannedTrajectory"], [])
            self.assertEqual(entry["referenceTrajectory"], [])

            self.assertEqual(terminal["trajectoryRevision"], 43)
            self.assertEqual(terminal["projectedTAEMTrajectory"], [])
            self.assertEqual(terminal["referenceTrajectory"], reference)
            self.assertEqual(terminal["plannedTrajectory"], reference)

    def _records(self) -> tuple[dict, dict, list[dict], list[dict]]:
        predicted = [{"ut": 100.0, "latitude": 1.0, "longitude": 2.0, "altitude": 30000.0}]
        planned = [{"ut": 100.0, "latitude": 3.0, "longitude": 4.0, "altitude": 31000.0}]
        populated = {
            "recordType": "plannerSample",
            "ut": 100.0,
            "trajectoryIncluded": True,
            "publishedPrediction": predicted,
            "currentPlan": {"valid": True, "terminalReady": True, "targetBank": 20.0, "targetAoA": 18.0, "targetHeading": 90.0},
            "plannerTrace": {"candidates": [{"selected": True, "candidatePath": planned}]},
        }
        # Mirrors the real 12:24 abort/disconnect record: executable lineage is
        # explicitly gone, but plannerTrace still contains a diagnostic candidate.
        residual = [{"ut": 102.0, "latitude": 7.0, "longitude": 8.0, "altitude": 18000.0}]
        cleared = {
            "recordType": "plannerSample",
            "ut": 101.0,
            "trajectoryIncluded": True,
            "publishedPrediction": [],
            "currentPlan": None,
            "lineage": {"planId": 0, "version": 0, "lineageSource": "none"},
            "plannerTrace": {"candidates": [{"selected": True, "candidatePath": residual}]},
        }
        return populated, cleared, predicted, planned

    def test_planner_follower_explicit_trajectory_clear_clears_prediction_and_plan(self) -> None:
        populated, cleared, predicted, planned = self._records()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            logs = root / "FlightLogs"
            logs.mkdir()
            path = logs / "test-planner.jsonl"
            path.write_text(json.dumps(populated) + "\n", encoding="utf-8")
            follower = PlannerFollower(root)
            follower.poll()
            self.assertEqual(follower.prediction, predicted)
            self.assertEqual(follower.planned, planned)

            with path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(cleared) + "\n")
            follower.poll()
            self.assertEqual(follower.prediction, [])
            self.assertEqual(follower.planned, [])
            self.assertEqual(follower.trajectory_revision, 2)

    def test_planner_follower_revision_changes_only_on_authoritative_trajectory_snapshots(self) -> None:
        populated, cleared, predicted, planned = self._records()
        ordinary = {
            "recordType": "plannerSample",
            "ut": 100.5,
            "trajectoryIncluded": False,
            "currentPlan": populated["currentPlan"],
        }
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            logs = root / "FlightLogs"
            logs.mkdir()
            path = logs / "test-planner.jsonl"
            path.write_text(json.dumps(populated) + "\n", encoding="utf-8")
            follower = PlannerFollower(root)
            follower.poll()
            self.assertEqual(follower.trajectory_revision, 1)
            self.assertEqual(follower.prediction, predicted)
            self.assertEqual(follower.planned, planned)

            with path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(ordinary) + "\n")
            follower.poll()
            self.assertEqual(follower.trajectory_revision, 1)
            self.assertEqual(follower.prediction, predicted)
            self.assertEqual(follower.planned, planned)

            with path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(cleared) + "\n")
            follower.poll()
            self.assertEqual(follower.trajectory_revision, 2)
            self.assertEqual(follower.prediction, [])
            self.assertEqual(follower.planned, [])


    def test_exact_controller_geometry_overrides_forensic_copies(self) -> None:
        old_prediction = [{"latitude": 0.0, "longitude": 0.0, "altitude": 30000.0}]
        old_plan = [{"latitude": 1.0, "longitude": 1.0, "altitude": 20000.0}]
        prediction = [
            {"latitude": 2.0, "longitude": 3.0, "altitude": 25000.0},
            {"latitude": 2.1, "longitude": 3.2, "altitude": 24000.0},
        ]
        reference = [
            {"latitude": 4.0, "longitude": 5.0, "altitude": 12000.0},
            {"latitude": 4.1, "longitude": 5.2, "altitude": 10000.0},
        ]
        snapshot = {
            "predictedTrajectory": old_prediction,
            "plannedTrajectory": old_plan,
            "referenceTrajectory": [],
            "trajectoryRevision": 3,
        }
        exact = {
            "tickSequence": 77,
            "phase": "TAEM",
            "guidanceState": {"terminalPathCommitted": True},
            "predictedTrajectory": prediction,
            "referenceTrajectory": reference,
        }
        self.assertTrue(apply_exact_trajectory_geometry(snapshot, exact, {}, {"site": {}, "guidance": {}}))
        self.assertEqual(snapshot["predictedTrajectory"], prediction)
        self.assertEqual(snapshot["referenceTrajectory"], reference)
        self.assertEqual(snapshot["plannedTrajectory"], reference)
        self.assertEqual(snapshot["trajectoryRevision"], 77)

    def test_exact_empty_prediction_clears_stale_forecast_but_keeps_plan_fallback(self) -> None:
        fallback_plan = [
            {"latitude": 1.0, "longitude": 1.0, "altitude": 12000.0},
            {"latitude": 1.1, "longitude": 1.2, "altitude": 10000.0},
        ]
        snapshot = {
            "predictedTrajectory": [{"latitude": 9.0, "longitude": 9.0, "altitude": 30000.0}],
            "plannedTrajectory": fallback_plan,
            "referenceTrajectory": fallback_plan,
            "trajectoryRevision": 4,
        }
        exact = {"tickSequence": 78, "predictedTrajectory": [], "referenceTrajectory": []}
        self.assertTrue(apply_exact_trajectory_geometry(snapshot, exact, {}, {"site": {}, "guidance": {}}))
        self.assertEqual(snapshot["predictedTrajectory"], [])
        self.assertEqual(snapshot["referenceTrajectory"], [])
        self.assertEqual(snapshot["plannedTrajectory"], fallback_plan)
        self.assertEqual(snapshot["trajectoryRevision"], 78)

    def test_unproven_entry_plan_never_publishes_plan_geometry(self) -> None:
        populated, _cleared, predicted, _planned = self._records()
        populated["currentPlan"]["terminalReady"] = False
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            logs = root / "FlightLogs"
            logs.mkdir()
            path = logs / "test-planner.jsonl"
            path.write_text(json.dumps(populated) + "\n", encoding="utf-8")
            follower = PlannerFollower(root)
            follower.poll()
            self.assertEqual(follower.prediction, predicted)
            self.assertEqual(follower.planned, [])

    def test_capture_veto_hides_entry_candidate_path(self) -> None:
        populated, _cleared, _predicted, _planned = self._records()
        populated["currentPlan"]["terminalReady"] = True
        populated["currentPlan"]["taemCaptureReady"] = False
        populated["currentPlan"]["taemCaptureVeto"] = 167
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            logs = root / "FlightLogs"
            logs.mkdir()
            path = logs / "test-planner.jsonl"
            path.write_text(json.dumps(populated) + "\n", encoding="utf-8")
            follower = PlannerFollower(root)
            follower.poll()
            self.assertEqual(follower.planned, [])

    def test_unproven_entry_prediction_does_not_publish_projected_terminal_plan(self) -> None:
        prediction = [
            {"latitude": -1.2, "longitude": -82.0, "altitude": 30000.0, "speed": 1200.0, "ut": 100.0, "phase": "ENTRY"},
            {"latitude": -0.8, "longitude": -78.0, "altitude": 22000.0, "speed": 900.0, "ut": 120.0, "phase": "TAEM HANDOFF"},
        ]
        snapshot = {"predictedTrajectory": [], "projectedTAEMTrajectory": [{"latitude": 9.0, "longitude": 9.0}], "trajectoryRevision": 0}
        exact = {
            "tickSequence": 80,
            "phase": "MM304 Entry",
            "telemetry": {"ut": 100.0},
            "guidanceState": {
                "entryPlanValid": True,
                "entryPlanTerminalReady": False,
                "entryPlanSegmentRemaining": 10.0,
            },
            "predictedTrajectory": prediction,
        }
        configuration = {
            "site": {"latitude": -0.0486111111, "longitude": -74.7283333333, "altitude": 70.0, "runwayHeading": 90.0},
            "guidance": {"hacRadius": 12000.0, "finalApproachDistance": 8000.0, "taemGlideSlope": 12.0, "finalGlideSlope": 20.0},
        }
        self.assertTrue(apply_exact_trajectory_geometry(snapshot, exact, {}, configuration))
        self.assertEqual(snapshot["predictedTrajectory"], [prediction[0]])
        self.assertEqual(snapshot["projectedTAEMTrajectory"], [])

        exact["guidanceState"]["entryPlanTerminalReady"] = True
        exact["guidanceState"]["entryPlanTAEMCaptureReady"] = False
        exact["guidanceState"]["entryPlanTAEMCaptureVeto"] = 167
        self.assertTrue(apply_exact_trajectory_geometry(snapshot, exact, {}, configuration))
        self.assertEqual(snapshot["projectedTAEMTrajectory"], [])
        self.assertIn("speed", taem_capture_veto_reasons(167))
        self.assertIn("hac-radius", taem_capture_veto_reasons(167))

        exact["guidanceState"]["entryPlanTAEMCaptureReady"] = True
        exact["guidanceState"]["entryPlanTAEMCaptureVeto"] = 0
        self.assertTrue(apply_exact_trajectory_geometry(snapshot, exact, {}, configuration))
        self.assertGreaterEqual(len(snapshot["projectedTAEMTrajectory"]), 2)

        exact["phase"] = "TAEM"
        self.assertTrue(apply_exact_trajectory_geometry(snapshot, exact, {}, configuration))
        self.assertEqual(snapshot["projectedTAEMTrajectory"], [])

    def test_uncommitted_terminal_reference_is_hidden_and_clears_plan(self) -> None:
        old_plan = [
            {"latitude": 1.0, "longitude": 1.0, "altitude": 12000.0},
            {"latitude": 1.1, "longitude": 1.2, "altitude": 10000.0},
        ]
        candidate = [
            {"latitude": 4.0, "longitude": 5.0, "altitude": 12000.0},
            {"latitude": 4.1, "longitude": 5.2, "altitude": 10000.0},
        ]
        snapshot = {
            "predictedTrajectory": [],
            "plannedTrajectory": old_plan,
            "referenceTrajectory": old_plan,
            "trajectoryRevision": 4,
        }
        exact = {
            "tickSequence": 79,
            "phase": "TAEM",
            "guidanceState": {
                "terminalCandidateValid": True,
                "terminalPathSelected": True,
                "terminalPathCommitted": False,
            },
            "predictedTrajectory": [],
            "referenceTrajectory": candidate,
        }
        self.assertTrue(apply_exact_trajectory_geometry(snapshot, exact, {}, {"site": {}, "guidance": {}}))
        self.assertEqual(snapshot["referenceTrajectory"], [])
        self.assertEqual(snapshot["plannedTrajectory"], [])
        self.assertEqual(snapshot["trajectoryRevision"], 79)

    def test_projected_taem_trajectory_extends_handoff_to_runway(self) -> None:
        configuration = {
            "site": {
                "latitude": -0.0486111111,
                "longitude": -74.7283333333,
                "altitude": 70.0,
                "runwayHeading": 90.0,
            },
            "guidance": {
                "hacRadius": 12000.0,
                "finalApproachDistance": 8000.0,
                "taemGlideSlope": 12.0,
                "finalGlideSlope": 20.0,
            },
        }
        prediction = [
            {"ut": 100.0, "latitude": -0.40, "longitude": -79.0, "altitude": 22000.0, "speed": 900.0, "phase": "MM304 Entry"},
            {"ut": 180.0, "latitude": -0.55, "longitude": -77.4, "altitude": 18000.0, "speed": 610.0, "phase": "TAEM"},
        ]
        terminal = {"candidateKind": 2, "candidateRadius": 12000.0, "candidateSide": -1.0}
        projected = projected_taem_trajectory(
            prediction, terminal, configuration, {"sTurnSign": 1.0}
        )
        self.assertGreater(len(projected), 30)
        self.assertAlmostEqual(projected[0]["latitude"], prediction[-1]["latitude"], places=6)
        self.assertAlmostEqual(projected[0]["longitude"], prediction[-1]["longitude"], places=6)
        initial_course = bearing_degrees(projected[0], projected[1])
        initial_error = abs((initial_course + 180.0) % 360.0 - 180.0)
        self.assertLess(initial_error, 30.0)
        self.assertTrue(all(point.get("kind") == "projected" for point in projected))
        self.assertAlmostEqual(projected[-1]["latitude"], configuration["site"]["latitude"], places=6)
        self.assertAlmostEqual(projected[-1]["longitude"], configuration["site"]["longitude"], places=6)
        self.assertAlmostEqual(projected[-1]["altitude"], configuration["site"]["altitude"], places=6)

    def test_projected_taem_trajectory_waits_for_taem_handoff(self) -> None:
        configuration = {
            "site": {"latitude": 0.0, "longitude": 0.0, "altitude": 0.0, "runwayHeading": 90.0},
            "guidance": {},
        }
        prediction = [{"latitude": 0.0, "longitude": -5.0, "altitude": 25000.0, "phase": "MM304 Entry"}]
        self.assertEqual(projected_taem_trajectory(prediction, {}, configuration), [])

    def test_actual_trajectory_history_persists_and_keeps_long_rear_trail(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            points = [
                {"ut": float(index), "latitude": 0.0, "longitude": index * 0.001, "altitude": 50000.0 - index}
                for index in range(600)
            ]
            retained = prune_actual_trajectory(points, 599.0)
            self.assertEqual(len(retained), 600)
            save_actual_trajectory_history(root, retained)
            self.assertEqual(load_actual_trajectory_history(root), retained)

    def test_actual_trajectory_resume_rejects_quickload_or_teleport(self) -> None:
        history = [{"ut": 100.0, "latitude": 0.0, "longitude": 0.0, "altitude": 50000.0}]
        nearby = {"ut": 102.0, "latitude": 0.0, "longitude": 0.05, "altitude": 49800.0}
        quickload = {"ut": 90.0, "latitude": 0.0, "longitude": 0.05, "altitude": 50000.0}
        teleport = {"ut": 102.0, "latitude": 0.0, "longitude": 80.0, "altitude": 50000.0}
        self.assertTrue(actual_trajectory_can_resume(history, nearby))
        self.assertFalse(actual_trajectory_can_resume(history, quickload))
        self.assertFalse(actual_trajectory_can_resume(history, teleport))

    def test_actual_trajectory_backfills_from_vehicle_log(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            logs = root / "FlightLogs"
            logs.mkdir()
            path = logs / "test-vehicle.jsonl"
            records = [
                {"recordType": "vehicleKeyframe", "ut": 100.0, "fields": {"position": {"latitude": 0.0, "longitude": 0.0, "altitude": 50000.0}}},
                {"recordType": "vehicleDelta", "ut": 101.0, "fields": {"position": {"longitude": 0.1, "altitude": 49900.0}}},
                {"recordType": "vehicleDelta", "ut": 102.0, "fields": {"position": {"longitude": 0.2, "altitude": 49800.0}}},
            ]
            path.write_text("".join(json.dumps(record) + "\n" for record in records), encoding="utf-8")
            current = {"ut": 102.1, "latitude": 0.0, "longitude": 0.21, "altitude": 49790.0}
            points = load_actual_trajectory_from_vehicle_log(root, current)
            self.assertEqual(len(points), 3)
            self.assertAlmostEqual(points[0]["longitude"], 0.0)
            self.assertAlmostEqual(points[-1]["longitude"], 0.2)


    def test_osculating_orbit_separates_3d_orbit_from_rotating_ground_track(self) -> None:
        class Body:
            equatorial_radius = 600000.0
            atmosphere_depth = 0.0
            rotational_period = 4000.0
            reference_frame = object()

        class Orbit:
            period = 1000.0

            def position_at(self, ut: float, _frame: object) -> tuple[float, float, float]:
                angle = 2.0 * 3.141592653589793 * ut / self.period
                radius = 700000.0
                return radius * math.cos(angle), 0.0, radius * math.sin(angle)

        points, meta = build_orbital_trajectory(
            Orbit(), Body(), 0.0, {"latitude": 0.0, "longitude": 0.0, "altitude": 100000.0}
        )
        self.assertGreaterEqual(len(points), 70)
        self.assertEqual(meta["kind"], "osculating-orbit")
        self.assertAlmostEqual(points[-1]["inertialLongitude"], 0.0, places=6)
        self.assertAlmostEqual(points[-1]["longitude"], -90.0, places=6)
        self.assertEqual(points[-1]["phase"], "1 ORBIT")

    def test_osculating_orbit_stops_at_atmosphere_interface(self) -> None:
        class Body:
            equatorial_radius = 600000.0
            atmosphere_depth = 70000.0
            rotational_period = 4000.0
            reference_frame = object()

        class Orbit:
            period = 1000.0

            def position_at(self, ut: float, _frame: object) -> tuple[float, float, float]:
                radius = 700000.0 - 100.0 * ut
                angle = 2.0 * 3.141592653589793 * ut / self.period
                return radius * math.cos(angle), 0.0, radius * math.sin(angle)

        points, meta = build_orbital_trajectory(
            Orbit(), Body(), 0.0, {"latitude": 0.0, "longitude": 0.0, "altitude": 100000.0}
        )
        self.assertGreater(len(points), 2)
        self.assertLess(len(points), 73)
        self.assertEqual(meta["kind"], "vacuum-arc")
        self.assertEqual(meta["endReason"], "atmosphere-interface")
        self.assertAlmostEqual(points[-1]["altitude"], 70000.0, places=6)
        self.assertEqual(points[-1]["phase"], "ENTRY INTERFACE")

    def test_attached_hud_explicit_trajectory_clear_does_not_resurrect_old_plan(self) -> None:
        populated, cleared, predicted, _planned = self._records()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "test-planner.jsonl"
            path.write_text(json.dumps(populated) + "\n" + json.dumps(cleared) + "\n", encoding="utf-8")
            latest, prediction, planned = _load_latest_plan(path)
            self.assertEqual(latest, cleared)
            self.assertEqual(prediction, [])
            self.assertEqual(planned, [])


if __name__ == "__main__":
    unittest.main()
