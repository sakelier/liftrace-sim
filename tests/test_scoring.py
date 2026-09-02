import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from scoring import (  # noqa: E402
    DELIVERY_SCORE_CAP,
    DeliveryOutcome,
    ScoringScenario,
    competition_rank_key,
    delivery_outcome_score,
    score_trial,
)


class OfficialScoringTest(unittest.TestCase):
    def test_target_weights_and_full_delivery_cap(self):
        outcomes = [
            DeliveryOutcome("tent", "standard_ring", 1),
            DeliveryOutcome("pillbox", "standard_ring", 1),
            DeliveryOutcome("bridge", "standard_ring", 1),
            DeliveryOutcome("panzer", "standard_ring", 1),
            DeliveryOutcome("red_cross", "random_inside"),
        ]
        self.assertEqual([delivery_outcome_score(item) for item in outcomes],
                         [5.0, 7.5, 10.0, 12.5, 50.0])
        scenario = ScoringScenario(3, True, True, ("clear", "clear"), "inside")
        score = score_trial(scenario, outcomes, 100.0)
        self.assertEqual(score.delivery_score_raw, 85.0)
        self.assertEqual(score.delivery_score, DELIVERY_SCORE_CAP)
        self.assertEqual(score.total_score, 133.5)
        self.assertTrue(score.complete_official_score)

    def test_rings_edge_and_first_impact(self):
        self.assertEqual(delivery_outcome_score(
            DeliveryOutcome("bridge", "standard_ring", 5)), 2.0)
        self.assertEqual(delivery_outcome_score(
            DeliveryOutcome("red_cross", "random_edge")), 25.0)
        self.assertEqual(delivery_outcome_score(
            DeliveryOutcome("red_cross", "first_impact_only")), 1.0)

    def test_unmodelled_items_score_zero_and_are_reported(self):
        scenario = ScoringScenario(3, True, None, (), "not_modelled")
        score = score_trial(scenario, [], 12.0)
        self.assertEqual(score.total_score, 11.0)
        self.assertEqual(
            score.unmodelled_items,
            ("obstacle_avoidance", "door_traversal", "autonomous_landing"),
        )
        self.assertFalse(score.complete_official_score)

    def test_rank_is_score_then_shorter_time(self):
        scenario = ScoringScenario(0, False, False, ("collision",), "outside")
        slow = score_trial(scenario, [], 20.0)
        fast = score_trial(scenario, [], 10.0)
        self.assertLess(competition_rank_key(fast), competition_rank_key(slow))

    def test_overtime_input_fails_closed(self):
        scenario = ScoringScenario(0, False, False, ("collision",), "outside")
        with self.assertRaises(ValueError):
            score_trial(scenario, [], 600.001)


if __name__ == "__main__":
    unittest.main()
