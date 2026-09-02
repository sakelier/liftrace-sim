import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from compare_delivery_policies import paired_difference  # noqa: E402
from mission_sim import MissionTrialResult  # noqa: E402
from scoring import ScoreBreakdown  # noqa: E402


def result(policy, score, elapsed):
    breakdown = ScoreBreakdown(
        0, 0, 0, score, score, 0, 0, score, elapsed, (), True
    )
    return MissionTrialResult(
        policy, 0, 0, 0, 0, 0, True, False, elapsed, elapsed, 0, (), breakdown, ()
    )


class PairedComparisonTest(unittest.TestCase):
    def test_paired_delta_preserves_scenario_alignment(self):
        a = [result("a", 10, 20), result("a", 30, 40)]
        b = [result("b", 8, 25), result("b", 26, 35)]
        comparison = paired_difference(a, b)
        self.assertEqual(comparison.mean_score_delta_a_minus_b, 3.0)
        self.assertEqual(comparison.probability_a_scores_higher, 1.0)
        self.assertEqual(comparison.mean_elapsed_time_delta_a_minus_b_s, 0.0)


if __name__ == "__main__":
    unittest.main()
