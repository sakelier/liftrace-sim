import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from search_sim import load_parameters  # noqa: E402
from sweep_sim import SweepResult, pareto_flags, run_sweep  # noqa: E402


class ParameterSweepTest(unittest.TestCase):
    def test_pareto_front_rejects_dominated_point(self):
        def point(time, cue):
            return SweepResult(2.0, 1.0, 1.0, 2, 10.0, time, cue, cue, cue, 1.0)

        results = [point(10.0, 0.5), point(12.0, 0.4), point(15.0, 0.8)]
        self.assertEqual(pareto_flags(results), [True, False, True])

    def test_small_sweep_is_complete_and_reproducible(self):
        baseline, config = load_parameters(PROJECT_ROOT / "config" / "baseline.yaml")
        sweep = {
            "grid": {
                "altitude_m": [1.6, 2.4],
                "speed_mps": [0.5, 1.0],
                "lane_spacing_m": [0.9, 1.5],
            },
            "monte_carlo": {"trials": 10, "seed": 99},
        }
        first = run_sweep(baseline, config, sweep)
        second = run_sweep(baseline, config, sweep)
        self.assertEqual(len(first), 8)
        self.assertEqual(first, second)
        self.assertTrue(any(item.pareto_optimal for item in first))


if __name__ == "__main__":
    unittest.main()
