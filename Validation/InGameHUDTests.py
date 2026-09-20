from __future__ import annotations

import math
import unittest

from PyQtApp.ingame_hud import (
    _active_trajectory_samples,
    _body_attitude_director_segments,
    _target_attitude_basis_body,
    _attitude_director_distance,
    _decision_label,
    _downsample,
    _geo_position,
    _geo_points,
    _guidance_corridor_geometry,
    _back_extrapolation_positions,
    _planned_back_trajectory_points,
    _guidance_gate_samples,
    _mode_label,
    _reference_samples,
    _runway_segments,
    _select_guidance_trajectory,
    _status_label,
    _trajectory_segment_layers,
)


class InGameHUDTests(unittest.TestCase):
    def test_geo_points_accepts_snapshot_objects_and_rejects_invalid(self) -> None:
        points = _geo_points(
            [
                {"latitude": -0.1, "longitude": -74.7, "altitude": 15000},
                {"latitude": "bad", "longitude": 0, "altitude": 0},
                [1, 2, 3],
            ]
        )
        self.assertEqual(points, [(-0.1, -74.7, 15000.0), (1.0, 2.0, 3.0)])

    def test_downsample_preserves_path_endpoints(self) -> None:
        points = [(float(i), float(i * 2), float(i * 3)) for i in range(100)]
        reduced = _downsample(points, 12)
        self.assertEqual(len(reduced), 12)
        self.assertEqual(reduced[0], points[0])
        self.assertEqual(reduced[-1], points[-1])

    def test_trajectory_layers_keep_near_path_bright_and_far_path_quiet(self) -> None:
        points = [(float(index) * 1000.0, 0.0, 0.0) for index in range(8)]
        near, far = _trajectory_segment_layers(points, 2500.0)
        self.assertEqual(len(near), 3)
        self.assertEqual(len(far), 4)
        self.assertEqual(near[0][0], points[0])
        self.assertEqual(far[-1][1], points[-1])

    def test_mode_label_surfaces_terminal_executive_without_telemetry_wall(self) -> None:
        snapshot = {
            "phase": "TAEM",
            "automationEngaged": True,
            "guidanceState": {"taemExecutive": {"phase": "HAC capture"}},
        }
        self.assertEqual(_mode_label(snapshot), "TAEM · HAC CAPTURE · AUTO")

    def test_trajectory_corridor_is_guidance_native_world_space_geometry(self) -> None:
        trajectory = [
            {
                "ut": 100.0 + index * 6.0,
                "latitude": 0.02 * math.sin(index * 0.35),
                "longitude": -75.0 + index * 0.025,
                "altitude": 26000.0 - index * 900.0,
                "speed": 1350.0 - index * 45.0,
                "phase": "TAEM",
            }
            for index in range(14)
        ]
        self.assertEqual(len(_reference_samples(trajectory)), 14)
        gates = _guidance_gate_samples(trajectory, 100.0, "TAEM", maximum=6)
        self.assertGreaterEqual(len(gates), 3)
        self.assertTrue(all(gate[0] > 100.0 for gate in gates))
        corridor, active = _guidance_corridor_geometry(
            gates,
            600000.0,
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
            "TAEM",
        )
        self.assertEqual(len(corridor), max(0, len(gates) - 1))
        self.assertGreaterEqual(len(active), 8)
        geometry = corridor + active
        self.assertTrue(all(math.isfinite(value) for segment in geometry for point in segment for value in point))

    def test_entry_uses_predicted_trajectory_and_terminal_uses_reference(self) -> None:
        predicted = [{"ut": 100.0, "latitude": 1.0, "longitude": 2.0, "altitude": 30000.0}]
        reference = [{"ut": 0.0, "latitude": 3.0, "longitude": 4.0, "altitude": 10000.0}]
        planned = [{"ut": 200.0, "latitude": 5.0, "longitude": 6.0, "altitude": 16000.0}]
        path, tag = _select_guidance_trajectory(
            {
                "phase": "entryEnergy",
                "predictedTrajectory": predicted * 2,
                "referenceTrajectory": reference * 2,
                "plannedTrajectory": planned * 2,
            }
        )
        self.assertEqual(tag, "PRED")
        self.assertEqual(path, predicted * 2)
        path, tag = _select_guidance_trajectory(
            {
                "phase": "TAEM",
                "predictedTrajectory": predicted * 2,
                "referenceTrajectory": reference * 2,
                "plannedTrajectory": planned * 2,
            }
        )
        self.assertEqual(tag, "REF")
        self.assertEqual(path, reference * 2)

        # Explicit planner geometry is an honest terminal fallback when an observer cannot
        # reconstruct the backend reference trajectory, but it remains labelled PLAN.
        path, tag = _select_guidance_trajectory(
            {"phase": "TAEM", "referenceTrajectory": [], "plannedTrajectory": planned * 2}
        )
        self.assertEqual(tag, "PLAN")
        self.assertEqual(path, planned * 2)

        # During entry, a missing published prediction is a lineage gap. Never substitute a
        # planner candidate or terminal reference and make it look like the active forecast.
        path, tag = _select_guidance_trajectory(
            {
                "phase": "entryEnergy",
                "predictedTrajectory": [],
                "referenceTrajectory": reference * 2,
                "plannedTrajectory": planned * 2,
            }
        )
        self.assertEqual(tag, "PRED WAIT")
        self.assertEqual(path, [])

    def test_timed_predicted_trajectory_starts_at_vehicle_then_joins_smoothly(self) -> None:
        trajectory = [
            {"ut": 100.0 + i * 5.0, "latitude": 0.0, "longitude": i * 0.01, "altitude": 30000.0 - i * 500.0, "phase": "entryEnergy"}
            for i in range(8)
        ]
        active = _active_trajectory_samples(trajectory, 116.0, 0.0, 0.03, 28500.0, "entryEnergy", 600000.0)
        self.assertEqual(active[0][0], 116.0)
        self.assertEqual(active[0][1], (0.0, 0.03, 28500.0))
        self.assertAlmostEqual(active[1][0], 116.5, places=6)
        self.assertAlmostEqual(active[1][1][1], 0.033, places=6)
        self.assertAlmostEqual(active[1][1][2], 28350.0, places=6)
        self.assertEqual(active[2][0], 120.0)
        self.assertTrue(all(sample[0] >= active[0][0] for sample in active))

    def test_static_trajectory_starts_at_vehicle_before_next_reference_sample(self) -> None:
        trajectory = [
            {"ut": 0.0, "latitude": 0.0, "longitude": i * 0.01, "altitude": 20000.0 - i * 100.0, "phase": "TAEM"}
            for i in range(8)
        ]
        active = _active_trajectory_samples(trajectory, float("nan"), 0.0, 0.03, 19700.0, "TAEM", 600000.0)
        self.assertEqual(active[0][1], (0.0, 0.03, 19700.0))
        self.assertEqual(active[1][1], (0.0, 0.04, 19600.0))

    def test_static_capture_gate_starts_ahead_of_vehicle(self) -> None:
        samples = [
            (0.0, (0.0, -75.0 + i * 0.01, 20000.0 - i * 100.0), 900.0, "TAEM")
            for i in range(8)
        ]
        gates = _guidance_gate_samples([], float("nan"), "TAEM", maximum=4, samples_override=samples)
        self.assertEqual(gates[0], samples[2])
        self.assertNotIn(samples[0], gates)

    def test_back_trajectory_uses_selected_plan_before_vehicle(self) -> None:
        plan = [
            {
                "ut": 100.0 + i * 5.0,
                "latitude": 0.0,
                "longitude": i * 0.01,
                "altitude": 20000.0 - i * 100.0,
                "phase": "entryEnergy",
            }
            for i in range(8)
        ]
        back = _planned_back_trajectory_points(
            plan, 116.0, 0.0, 0.033, 19670.0, "entryEnergy", 600000.0,
            maximum=8, max_distance=900.0,
        )
        self.assertGreaterEqual(len(back), 1)
        self.assertLessEqual(len(back), 8)
        self.assertEqual(back[-1], (0.0, 0.03, 19700.0))
        self.assertTrue(all(point[1] < 0.033 for point in back))

    def test_future_only_plan_extrapolates_same_tangent_behind_vehicle(self) -> None:
        back = _back_extrapolation_positions(
            (0.0, 0.0, 0.0),
            [(100.0, 0.0, 0.0), (200.0, 0.0, 0.0)],
            distance=400.0,
            count=4,
        )
        self.assertEqual(back, [
            (-400.0, 0.0, 0.0),
            (-300.0, 0.0, 0.0),
            (-200.0, 0.0, 0.0),
            (-100.0, 0.0, 0.0),
        ])

    def test_attitude_director_uses_full_surface_to_body_attitude_transform(self) -> None:
        current = {"heading": 90.0, "pitch": 8.0, "roll": -15.0}
        identity = _target_attitude_basis_body(
            current,
            {"targetHeading": 90.0, "targetPitch": 8.0, "targetRoll": -15.0},
        )
        self.assertIsNotNone(identity)
        assert identity is not None
        forward, right, up = identity
        for actual, expected in zip(forward, (0.0, 1.0, 0.0)):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(right, (1.0, 0.0, 0.0)):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(up, (0.0, 0.0, -1.0)):
            self.assertAlmostEqual(actual, expected, places=6)

        # With wings level, a target heading to the right is a body-right displacement.
        level = _target_attitude_basis_body(
            {"heading": 90.0, "pitch": 0.0, "roll": 0.0},
            {"targetHeading": 102.0, "targetPitch": 0.0, "targetRoll": 0.0},
        )
        # At +90 deg roll the same surface heading correction appears mainly body-up;
        # applying heading error directly to body-right would incorrectly make these equal.
        rolled = _target_attitude_basis_body(
            {"heading": 90.0, "pitch": 0.0, "roll": 90.0},
            {"targetHeading": 102.0, "targetPitch": 0.0, "targetRoll": 90.0},
        )
        assert level is not None and rolled is not None
        self.assertGreater(level[0][0], 0.15)
        self.assertAlmostEqual(level[0][2], 0.0, places=6)
        self.assertAlmostEqual(rolled[0][0], 0.0, places=6)
        self.assertLess(rolled[0][2], -0.15)

        # Pitch-up at +90 deg roll cross-couples toward body-left, not body-up.
        rolled_pitch = _target_attitude_basis_body(
            {"heading": 90.0, "pitch": 0.0, "roll": 90.0},
            {"targetHeading": 90.0, "targetPitch": 10.0, "targetRoll": 90.0},
        )
        assert rolled_pitch is not None
        self.assertLess(rolled_pitch[0][0], -0.15)
        self.assertAlmostEqual(rolled_pitch[0][2], 0.0, places=6)

        # Positive target roll is a standard right roll (right wing down): the target
        # right axis therefore gains positive body-Z/down while the nose stays centered.
        roll_target = _target_attitude_basis_body(
            {"heading": 90.0, "pitch": 0.0, "roll": 0.0},
            {"targetHeading": 90.0, "targetPitch": 0.0, "targetRoll": 20.0},
        )
        assert roll_target is not None
        self.assertAlmostEqual(roll_target[0][0], 0.0, places=6)
        self.assertAlmostEqual(roll_target[0][2], 0.0, places=6)
        self.assertGreater(roll_target[1][2], 0.30)

        datum_level, director_level = _body_attitude_director_segments(
            {"heading": 90.0, "pitch": 0.0, "roll": 0.0},
            {"targetHeading": 102.0, "targetPitch": 0.0, "targetRoll": 0.0},
            300.0,
        )
        datum_rolled, director_rolled = _body_attitude_director_segments(
            {"heading": 90.0, "pitch": 0.0, "roll": 90.0},
            {"targetHeading": 102.0, "targetPitch": 0.0, "targetRoll": 90.0},
            300.0,
        )
        self.assertEqual(datum_level, datum_rolled)
        self.assertNotEqual(director_level, director_rolled)
        self.assertTrue(all(math.isfinite(v) for seg in director_level + director_rolled for point in seg for v in point))


    def test_aerodynamic_attitude_director_uses_aoa_and_bank_error_not_surface_euler_pose(self) -> None:
        # At high bank, surface pitch is not flight-path angle + AoA. This sample is already
        # essentially on its aerodynamic pitch target even though the synthetic targetPitch is
        # more than ten degrees away from measured surface pitch.
        telemetry = {
            "heading": 108.247367858887,
            "pitch": 5.05841064453125,
            "roll": 64.1355895996094,
            "angleOfAttack": 19.827615737915,
            "commandPitchError": 0.0971166312766,
            "commandRollError": 0.3231051727323,
        }
        command = {
            "targetHeading": 91.3674458428387,
            "targetPitch": 15.9801190388001,
            "targetRoll": 64.4586947723417,
            "targetAoA": 19.9247323691916,
            "headingControlEnabled": True,
        }
        basis = _target_attitude_basis_body(telemetry, command)
        self.assertIsNotNone(basis)
        assert basis is not None
        forward, right, _ = basis
        self.assertAlmostEqual(forward[0], 0.0, places=8)
        self.assertAlmostEqual(
            forward[2],
            -math.sin(math.radians(telemetry["commandPitchError"])),
            places=8,
        )
        self.assertGreater(right[2], 0.0)
        self.assertLess(abs(forward[2]), 0.002)

    def test_aerodynamic_attitude_director_pitch_is_body_native_even_at_large_bank(self) -> None:
        basis = _target_attitude_basis_body(
            {"heading": 90.0, "pitch": 5.0, "roll": 70.0, "angleOfAttack": 10.0},
            {"targetHeading": 30.0, "targetPitch": 25.0, "targetRoll": 70.0, "targetAoA": 20.0},
        )
        self.assertIsNotNone(basis)
        assert basis is not None
        forward, right, _ = basis
        self.assertAlmostEqual(forward[0], 0.0, places=8)
        self.assertAlmostEqual(forward[1], math.cos(math.radians(10.0)), places=8)
        self.assertAlmostEqual(forward[2], -math.sin(math.radians(10.0)), places=8)
        self.assertAlmostEqual(right[0], 1.0, places=8)
        self.assertAlmostEqual(right[2], 0.0, places=8)

    def test_attitude_director_depth_stays_close_to_vehicle_without_extreme_parallax(self) -> None:
        # The cue remains body-fixed rather than screen-fixed. Keep it close enough to read
        # as shuttle-attached while retaining a modest camera-distance guard for chase view.
        self.assertEqual(_attitude_director_distance(0.0, float("nan")), 500.0)
        self.assertEqual(_attitude_director_distance(12000.0, 80.0), 800.0)
        self.assertEqual(_attitude_director_distance(80000.0, 500.0), 5000.0)
        distance = _attitude_director_distance(20000.0, 100.0)
        self.assertLess(math.degrees(math.atan2(100.0, distance)), 6.0)
        datum, director = _body_attitude_director_segments(
            {"heading": 90.0, "pitch": 5.0, "roll": 10.0},
            {"targetHeading": 90.0, "targetPitch": 5.0, "targetRoll": 10.0},
            distance,
        )
        self.assertTrue(all(point[1] == distance for segment in datum + director for point in segment))


    def test_entry_decision_label_surfaces_plan_and_final_reversal(self) -> None:
        label = _decision_label(
            {
                "phase": "entryEnergy",
                "command": {"targetRoll": 25.0, "targetAoA": 10.0},
                "guidanceState": {
                    "entryPlanValid": True,
                    "entryPlanTargetBank": 24.0,
                    "entryPlanTargetAoA": 9.0,
                    "entryReversalScheduled": True,
                    "entryReversalIsFinal": True,
                    "entryReversalTimeRemaining": 18.0,
                    "entryPlanTAEMRangeError": -12500.0,
                },
            }
        )
        self.assertEqual(label, "REV FINAL T−18s")
        self.assertNotIn("BANK", label)
        self.assertNotIn("α", label)
        self.assertNotIn("ΔR", label)
        quiet = _decision_label(
            {
                "phase": "entryEnergy",
                "guidanceState": {"entryPlanTAEMEnergyError": -88110533.0},
            }
        )
        self.assertEqual(quiet, "")

    def test_runway_overlay_is_outline_plus_extended_centerline(self) -> None:
        site = {
            "latitude": -0.0486111111,
            "longitude": -74.7283333333,
            "altitude": 70,
            "runwayHeading": 90,
            "runwayLength": 2500,
            "runwayWidth": 70,
        }
        prime = (1.0, 0.0, 0.0)
        quarter = (0.0, 1.0, 0.0)
        pole = (0.0, 0.0, 1.0)
        segments = _runway_segments(site, 600000.0, prime, quarter, pole)
        self.assertEqual(len(segments), 21)
        self.assertTrue(all(math.isfinite(value) for segment in segments for point in segment for value in point))

        threshold = _geo_position(
            (site["latitude"], site["longitude"], site["altitude"]), 600000.0, prime, quarter, pole, lift=10.0
        )
        start_edge = segments[2]
        start_midpoint = tuple((start_edge[0][axis] + start_edge[1][axis]) * 0.5 for axis in range(3))
        self.assertLess(math.dist(start_midpoint, threshold), 1e-6)

    def test_status_label_stays_compact(self) -> None:
        snapshot = {
            "telemetry": {
                "meanAltitude": 12000.0,
                "trueAirSpeed": 850.0,
                "latitude": 0.0,
                "longitude": 0.0,
            }
        }
        configuration = {"site": {"latitude": 0.0, "longitude": 0.0}}
        self.assertEqual(
            _status_label(snapshot, configuration, 600000.0),
            "ALT 12.0 km  ·  SPD 850 m/s  ·  RWY 0 m",
        )


if __name__ == "__main__":
    unittest.main()
