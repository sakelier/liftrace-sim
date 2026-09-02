import math
import random
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from search_sim import (  # noqa: E402
    CameraModel,
    CueModel,
    M0Parameters,
    Point3,
    SearchArea,
    Target,
    TargetGenerationConfig,
    cue_probability,
    evaluate_m0,
    evaluate_target_visibility,
    generate_boustrophedon,
    generate_random_targets,
    load_parameters,
    observe_target_on_segment,
    run_monte_carlo,
    segment_start_times,
    simulate_cues,
)
from vision_performance import (  # noqa: E402
    EmpiricalVisionModel,
    VisionCondition,
    load_empirical_vision_model,
)


class BoustrophedonM0Test(unittest.TestCase):
    def test_hand_calculable_six_by_eight_case(self):
        params = M0Parameters(
            area=SearchArea(0.0, 6.0, 0.0, 8.0),
            lane_direction="x",
            lane_spacing=1.2,
            altitude=2.0,
            speed=1.0,
            turn_penalty=1.0,
        )

        waypoints = generate_boustrophedon(params)
        metrics = evaluate_m0(params, waypoints)

        self.assertEqual(metrics.lane_count, 8)
        self.assertEqual(metrics.waypoint_count, 16)
        self.assertEqual(metrics.connector_segment_count, 7)
        self.assertAlmostEqual(metrics.lane_distance_m, 48.0)
        self.assertAlmostEqual(metrics.connector_distance_m, 8.0)
        self.assertAlmostEqual(metrics.route_distance_m, 56.0)
        self.assertEqual(metrics.turn_count, 14)
        self.assertAlmostEqual(metrics.total_time_s, 70.0)

    def test_baseline_matches_current_uav_mission_geometry(self):
        params, _ = load_parameters(PROJECT_ROOT / "config" / "baseline.yaml")
        waypoints = generate_boustrophedon(params)
        metrics = evaluate_m0(params, waypoints)

        self.assertEqual(metrics.lane_count, 8)
        self.assertEqual(metrics.waypoint_count, 16)
        self.assertAlmostEqual(metrics.lane_distance_m, 49.6)
        self.assertAlmostEqual(metrics.connector_distance_m, 8.0)
        self.assertAlmostEqual(metrics.route_distance_m, 57.6)
        self.assertEqual(metrics.turn_count, 14)
        self.assertAlmostEqual(metrics.total_time_s, 71.6)

        self.assertEqual((waypoints[0].x, waypoints[0].y), (-3.6, -2.0))
        self.assertEqual((waypoints[1].x, waypoints[1].y), (2.6, -2.0))
        self.assertAlmostEqual(waypoints[-1].y, 6.0)

    def test_y_direction_transposes_the_pattern(self):
        params = M0Parameters(
            area=SearchArea(0.0, 6.0, 0.0, 8.0),
            lane_direction="y",
            lane_spacing=2.0,
            altitude=2.0,
            speed=2.0,
            turn_penalty=0.5,
        )

        waypoints = generate_boustrophedon(params)
        metrics = evaluate_m0(params, waypoints)

        self.assertEqual(metrics.lane_count, 4)
        self.assertAlmostEqual(metrics.lane_distance_m, 32.0)
        self.assertAlmostEqual(metrics.connector_distance_m, 6.0)
        self.assertEqual(metrics.turn_count, 6)
        self.assertAlmostEqual(metrics.total_time_s, 22.0)

    def test_invalid_parameters_fail_closed(self):
        with self.assertRaises(ValueError):
            SearchArea(1.0, 1.0, 0.0, 2.0)
        with self.assertRaises(ValueError):
            M0Parameters(
                area=SearchArea(0.0, 1.0, 0.0, 1.0),
                lane_direction="diagonal",
                lane_spacing=1.0,
                altitude=1.0,
                speed=1.0,
                turn_penalty=0.0,
            )
        with self.assertRaises(ValueError):
            M0Parameters(
                area=SearchArea(0.0, 1.0, 0.0, 1.0),
                lane_direction="x",
                lane_spacing=math.nan,
                altitude=1.0,
                speed=1.0,
                turn_penalty=0.0,
            )


class VisibilityM1aTest(unittest.TestCase):
    def setUp(self):
        self.camera = CameraModel(
            horizontal_fov=1.047,
            image_width=640,
            image_height=480,
            update_rate=30.0,
        )
        self.start = Point3(0.0, 0.0, 2.2)
        self.end = Point3(6.0, 0.0, 2.2)

    def test_camera_fov_and_footprint_follow_aspect_ratio(self):
        expected_vertical = 2.0 * math.atan(
            math.tan(1.047 / 2.0) * 480.0 / 640.0)
        self.assertAlmostEqual(self.camera.vertical_fov, expected_vertical)

        cross, along = self.camera.footprint(2.2)
        self.assertAlmostEqual(cross, 2.0 * 2.2 * math.tan(1.047 / 2.0))
        self.assertAlmostEqual(along, 2.0 * 2.2 * math.tan(expected_vertical / 2.0))

    def test_center_target_has_symmetric_visible_interval(self):
        target = Target("center", "tent", Point3(3.0, 0.0, 0.0))
        observation = observe_target_on_segment(
            self.start, self.end, target, self.camera, speed=1.0,
            require_fully_in_frame=False,
        )

        self.assertIsNotNone(observation)
        expected_half_along = self.camera.footprint(2.2)[1] / 2.0
        self.assertAlmostEqual(observation.entry_distance_m, 3.0 - expected_half_along)
        self.assertAlmostEqual(observation.exit_distance_m, 3.0 + expected_half_along)
        self.assertAlmostEqual(observation.dwell_time_s, 2.0 * expected_half_along)
        self.assertAlmostEqual(observation.cross_track_distance_m, 0.0)

    def test_cross_track_edge_is_visible_and_just_outside_is_not(self):
        cross_half = self.camera.footprint(2.2)[0] / 2.0
        edge = Target("edge", "tent", Point3(3.0, cross_half, 0.0))
        outside = Target("outside", "tent", Point3(3.0, cross_half + 0.001, 0.0))

        self.assertIsNotNone(observe_target_on_segment(
            self.start, self.end, edge, self.camera, speed=1.0,
            require_fully_in_frame=False,
        ))
        self.assertIsNone(observe_target_on_segment(
            self.start, self.end, outside, self.camera, speed=1.0,
            require_fully_in_frame=False,
        ))

    def test_full_target_requirement_reduces_dwell_and_cross_track_width(self):
        target = Target(
            "standard", "tent", Point3(3.0, 0.0, 0.0),
            size_x=1.0, size_y=1.0,
        )
        center = observe_target_on_segment(
            self.start, self.end, target, self.camera, speed=1.0,
            require_fully_in_frame=False,
        )
        full = observe_target_on_segment(
            self.start, self.end, target, self.camera, speed=1.0,
            require_fully_in_frame=True,
        )

        self.assertIsNotNone(center)
        self.assertIsNotNone(full)
        self.assertAlmostEqual(
            center.visible_path_length_m - full.visible_path_length_m,
            1.0,
        )

        cross_half = self.camera.footprint(2.2)[0] / 2.0
        center_only_target = Target(
            "center-only", "tent", Point3(3.0, cross_half - 0.25, 0.0),
            size_x=1.0, size_y=1.0,
        )
        self.assertIsNotNone(observe_target_on_segment(
            self.start, self.end, center_only_target, self.camera, speed=1.0,
            require_fully_in_frame=False,
        ))
        self.assertIsNone(observe_target_on_segment(
            self.start, self.end, center_only_target, self.camera, speed=1.0,
            require_fully_in_frame=True,
        ))

    def test_target_near_segment_start_is_clipped_to_segment(self):
        target = Target("start", "red_cross", Point3(0.0, 0.0, 0.0))
        observation = observe_target_on_segment(
            self.start, self.end, target, self.camera, speed=2.0,
            require_fully_in_frame=False,
        )

        expected_half_along = self.camera.footprint(2.2)[1] / 2.0
        self.assertAlmostEqual(observation.entry_distance_m, 0.0)
        self.assertAlmostEqual(observation.exit_distance_m, expected_half_along)
        self.assertAlmostEqual(observation.dwell_time_s, expected_half_along / 2.0)


class CueM1bTest(unittest.TestCase):
    def setUp(self):
        self.params = M0Parameters(
            area=SearchArea(0.0, 6.0, 0.0, 6.0),
            lane_direction="x", lane_spacing=1.0, altitude=2.2,
            speed=1.0, turn_penalty=1.0,
        )
        self.camera = CameraModel(1.047, 640, 480, 30.0)
        self.waypoints = generate_boustrophedon(self.params)
        self.generation = TargetGenerationConfig(
            class_counts={"tent": 2, "red_cross": 1},
            class_sizes={"tent": (1.0, 1.0), "red_cross": (0.35, 0.35)},
            minimum_separation=1.0,
            maximum_attempts=1000,
        )

    def test_random_generation_is_seed_reproducible_and_constrained(self):
        first = generate_random_targets(self.params.area, self.generation, random.Random(7))
        second = generate_random_targets(self.params.area, self.generation, random.Random(7))
        self.assertEqual(first, second)
        for target in first:
            self.assertGreaterEqual(target.position.x - target.size_x / 2.0, 0.0)
            self.assertLessEqual(target.position.x + target.size_x / 2.0, 6.0)
            self.assertGreaterEqual(target.position.y - target.size_y / 2.0, 0.0)
            self.assertLessEqual(target.position.y + target.size_y / 2.0, 6.0)
        for index, left in enumerate(first):
            for right in first[index + 1:]:
                self.assertGreaterEqual(
                    math.hypot(left.position.x - right.position.x,
                               left.position.y - right.position.y), 1.0)

    def test_cue_probability_rewards_dwell_and_penalizes_cross_track(self):
        short_center = observe_target_on_segment(
            Point3(0.0, 0.0, 2.2), Point3(0.3, 0.0, 2.2),
            Target("short", "tent", Point3(0.15, 0.0, 0.0)), self.camera, 1.0,
            require_fully_in_frame=False)
        long_center = observe_target_on_segment(
            Point3(0.0, 0.0, 2.2), Point3(6.0, 0.0, 2.2),
            Target("long", "tent", Point3(3.0, 0.0, 0.0)), self.camera, 1.0,
            require_fully_in_frame=False)
        long_edge = observe_target_on_segment(
            Point3(0.0, 0.0, 2.2), Point3(6.0, 0.0, 2.2),
            Target("edge", "tent", Point3(3.0, 1.0, 0.0)), self.camera, 1.0,
            require_fully_in_frame=False)
        model = CueModel({"tent": 0.8}, 0.5, 1.0)
        self.assertLess(cue_probability(short_center, model), cue_probability(long_center, model))
        self.assertLess(cue_probability(long_edge, model), cue_probability(long_center, model))

    def test_probability_override_extremes_are_exact(self):
        target = Target("target", "tent", Point3(3.0, 0.0, 0.0))
        observation = observe_target_on_segment(
            Point3(0.0, 0.0, 2.2), Point3(6.0, 0.0, 2.2),
            target, self.camera, 1.0, require_fully_in_frame=False)
        self.assertEqual(cue_probability(observation, CueModel({}, 1.0, 0.0, 0.0)), 0.0)
        self.assertEqual(cue_probability(observation, CueModel({}, 1.0, 0.0, 1.0)), 1.0)

    def test_empirical_exact_condition_overrides_legacy_formula(self):
        observation = observe_target_on_segment(
            Point3(0.0, 0.0, 1.2), Point3(6.0, 0.0, 1.2),
            Target("bridge", "bridge", Point3(3.0, 0.0, 0.0)),
            self.camera, 2.0, require_fully_in_frame=False,
        )
        empirical = EmpiricalVisionModel((VisionCondition("bridge", 1.2, 2.0, 0, 4),))
        model = CueModel({"bridge": 1.0}, 0.01, 0.0, empirical_model=empirical)
        self.assertEqual(cue_probability(observation, model), 0.0)

    def test_unmeasured_empirical_condition_uses_explicit_legacy_fallback(self):
        observation = observe_target_on_segment(
            Point3(0.0, 0.0, 2.2), Point3(6.0, 0.0, 2.2),
            Target("tent", "tent", Point3(3.0, 0.0, 0.0)),
            self.camera, 1.0, require_fully_in_frame=False,
        )
        empirical = EmpiricalVisionModel((VisionCondition("tent", 1.2, 0.5, 1, 1),))
        legacy = CueModel({"tent": 0.6}, 0.5, 0.0)
        combined = CueModel({"tent": 0.6}, 0.5, 0.0, empirical_model=empirical)
        self.assertEqual(cue_probability(observation, combined), cue_probability(observation, legacy))

    def test_checked_in_empirical_table_contains_expected_boundary(self):
        table = PROJECT_ROOT / "data" / "vision" / "vsim04_20260902_v2" / "condition_success_rates.csv"
        model = load_empirical_vision_model(table)
        self.assertEqual(len(model.conditions), 100)
        self.assertEqual(model.probability("bridge", 2.4, 1.0), 1.0)
        self.assertEqual(model.probability("pillbox", 3.6, 2.0), 0.0)
        self.assertIsNone(model.probability("bridge", 2.2, 1.0))

    def test_segment_timing_includes_turn_penalty(self):
        points = [Point3(0, 0, 1), Point3(2, 0, 1), Point3(2, 1, 1)]
        self.assertEqual(segment_start_times(points, speed=2.0, turn_penalty=0.5), [0.0, 1.5])

    def test_zero_and_one_probability_control_cue_results(self):
        target = Target("target", "tent", Point3(3.0, 0.0, 0.0))
        points = [Point3(0.0, 0.0, 2.2), Point3(6.0, 0.0, 2.2)]
        observations = evaluate_target_visibility(
            points, [target], self.camera, 1.0, require_fully_in_frame=False)
        miss = simulate_cues(points, [target], observations,
                             CueModel({}, 1.0, 0.0, 0.0), 1.0, 0.0, random.Random(1))
        hit = simulate_cues(points, [target], observations,
                            CueModel({}, 1.0, 0.0, 1.0), 1.0, 0.0, random.Random(1))
        self.assertFalse(miss[0].cued)
        self.assertTrue(hit[0].cued)
        self.assertIsNotNone(hit[0].first_cue_time_s)

    def test_monte_carlo_is_reproducible(self):
        model = CueModel({"tent": 0.7, "red_cross": 0.8}, 0.5, 1.0)
        first = run_monte_carlo(
            self.params, self.waypoints, self.camera, self.generation,
            model, trials=30, seed=42)
        second = run_monte_carlo(
            self.params, self.waypoints, self.camera, self.generation,
            model, trials=30, seed=42)
        self.assertEqual(first, second)
        self.assertGreaterEqual(first.mean_cue_rate, 0.0)
        self.assertLessEqual(first.mean_cue_rate, 1.0)


if __name__ == "__main__":
    unittest.main()
