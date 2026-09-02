import random
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from mission_sim import MissionConfig, simulate_mission_trial  # noqa: E402
from search_sim import (  # noqa: E402
    CameraModel,
    CueModel,
    M0Parameters,
    Point3,
    SearchArea,
    Target,
)


class MissionStateMachineTest(unittest.TestCase):
    def setUp(self):
        self.params = M0Parameters(
            SearchArea(0.0, 4.0, -1.0, 1.0), "x", 1.0, 2.0, 1.0, 0.0
        )
        self.route = [Point3(0, 0, 2), Point3(4, 0, 2)]
        self.camera = CameraModel(1.047, 640, 480, 30.0)
        self.target = Target("tent_1", "tent", Point3(2, 0, 0), 0.2, 0.2)

    def config(self, deadline=100.0):
        return MissionConfig(
            deadline, 3.0, 1.0, 1.0, 1.0, 0.5, 0.5,
            {"tent": 1.0}, {"tent": 1.0},
        )

    def test_successful_interrupt_delivers_and_resumes_search(self):
        result = simulate_mission_trial(
            self.params, self.route, self.camera, [self.target],
            CueModel({}, 1.0, 0.0, probability_override=1.0),
            self.config(), random.Random(1), random.Random(2),
        )
        states = [event.state for event in result.events]
        self.assertEqual(result.cue_count, 1)
        self.assertEqual(result.confirmed_count, 1)
        self.assertEqual(result.delivered_count, 1)
        self.assertTrue(result.search_completed)
        self.assertFalse(result.timed_out)
        for state in ("CUE", "APPROACH", "VERIFY", "DELIVER", "RETURN", "COMPLETE"):
            self.assertIn(state, states)

    def test_deadline_stops_state_machine(self):
        result = simulate_mission_trial(
            self.params, self.route, self.camera, [self.target],
            CueModel({}, 1.0, 0.0, probability_override=1.0),
            self.config(deadline=1.0), random.Random(1), random.Random(2),
        )
        self.assertTrue(result.timed_out)
        self.assertFalse(result.search_completed)
        self.assertEqual(result.elapsed_time_s, 1.0)
        self.assertEqual(result.events[-1].state, "TIMEOUT")

    def test_delivery_attempts_are_limited_by_cargo_capacity(self):
        targets = [
            Target(f"tent_{index}", "tent", Point3(x, 0, 0), 0.2, 0.2)
            for index, x in enumerate((0.5, 1.5, 2.5), start=1)
        ]
        mission = MissionConfig(
            100.0, 3.0, 1.0, 1.0, 1.0, 0.0, 0.0,
            {"tent": 1.0}, {"tent": 1.0}, cargo_capacity=2,
        )
        result = simulate_mission_trial(
            self.params, self.route, self.camera, targets,
            CueModel({}, 1.0, 0.0, probability_override=1.0),
            mission, random.Random(1), random.Random(2),
        )
        self.assertEqual(result.confirmed_count, 2)
        self.assertEqual(result.delivery_attempt_count, 2)
        self.assertEqual(result.delivered_count, 2)
        self.assertIn("NO_CARGO", [event.state for event in result.events])


if __name__ == "__main__":
    unittest.main()
