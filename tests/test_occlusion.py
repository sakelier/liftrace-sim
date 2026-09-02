import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from obstacles import Obstacle  # noqa: E402
from occlusion import OcclusionConfig, evaluate_observation_occlusion  # noqa: E402
from search_sim import (  # noqa: E402
    CameraModel,
    CueModel,
    Point3,
    Target,
    cue_probability,
    observe_target_on_segment,
)


class OcclusionVisibilityTest(unittest.TestCase):
    def setUp(self):
        self.start = Point3(0.0, 0.0, 3.0)
        self.end = Point3(6.0, 0.0, 3.0)
        self.target = Target("t", "tent", Point3(3.0, 0.0, 0.0), 0.2, 0.2)
        self.camera = CameraModel(1.047, 640, 480, 30.0)
        self.observation = observe_target_on_segment(
            self.start, self.end, self.target, self.camera, 1.0,
            require_fully_in_frame=True,
        )
        self.config = OcclusionConfig(15.0, 0.6, 0.2)

    def test_unobstructed_pass_preserves_full_visibility(self):
        result = evaluate_observation_occlusion(
            self.observation, self.start, self.end, self.target, [], self.config)
        self.assertTrue(result.occlusion_passed)
        self.assertEqual(result.mean_visible_fraction, 1.0)
        self.assertEqual(result.visible_frame_count, result.sampled_frame_count)
        self.assertAlmostEqual(result.effective_visible_time_s, result.dwell_time_s)

    def test_fully_blocked_pass_is_gated_before_cue_model(self):
        blocker = Obstacle("cover", 3.0, 0.0, 0.8, 0.8, 2.8)
        result = evaluate_observation_occlusion(
            self.observation, self.start, self.end, self.target, [blocker], self.config)
        self.assertFalse(result.occlusion_passed)
        self.assertEqual(result.visible_frame_count, 0)
        self.assertIn("cover", result.occluding_obstacle_ids)
        self.assertEqual(cue_probability(result, CueModel({"tent": 1.0}, 0.01, 0.0)), 0.0)


if __name__ == "__main__":
    unittest.main()
