import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from sweep_delivery_threshold import (  # noqa: E402
    ThresholdSweepResult,
    parameter_grid,
    select_candidates,
)


class ThresholdSweepTest(unittest.TestCase):
    def result(self, name, score, elapsed):
        return ThresholdSweepResult(
            name, 0.5, 1.0, score, score - 1, score + 1,
            score - 2, 0.5, 0.5, elapsed, elapsed / 2, 0.0,
        )

    def test_grid_is_cartesian_product(self):
        grid = parameter_grid([0.25, 0.5], [0.5, 1.0, 2.0])
        self.assertEqual(len(grid), 6)
        self.assertEqual(grid[0].uniform_future_cue_prior, 0.25)
        self.assertEqual(grid[-1].remaining_progress_exponent, 2.0)

    def test_selection_uses_score_then_time(self):
        selected = select_candidates([
            self.result("slow", 10.0, 20.0),
            self.result("fast", 10.0, 15.0),
            self.result("lower", 9.0, 1.0),
        ], 2)
        self.assertEqual([item.policy_name for item in selected], ["fast", "slow"])

    def test_invalid_axes_fail_closed(self):
        with self.assertRaises(ValueError):
            parameter_grid([], [1.0])
        with self.assertRaises(ValueError):
            parameter_grid([1.1], [1.0])
        with self.assertRaises(ValueError):
            parameter_grid([0.5], [0.0])


if __name__ == "__main__":
    unittest.main()
