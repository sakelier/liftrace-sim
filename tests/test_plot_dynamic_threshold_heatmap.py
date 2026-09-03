import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from plot_dynamic_threshold_heatmap import (  # noqa: E402
    SweepCell,
    render_heatmap,
)


class DynamicThresholdHeatmapTest(unittest.TestCase):
    def test_fixture_sweep_renders_valid_svg(self):
        priors = (0.25, 0.40, 0.55, 0.70, 0.85, 1.00)
        exponents = (0.50, 0.75, 1.00, 1.25, 1.50, 2.00)
        cells = [
            SweepCell(
                f"dynamic_p{prior:.3f}_g{exponent:.3f}",
                prior,
                exponent,
                10.0 + prior - exponent,
                300.0 + 10.0 * exponent - prior,
            )
            for prior in priors
            for exponent in exponents
        ]
        candidate = "dynamic_p0.700_g0.500"
        selected = {
            candidate,
            "dynamic_p0.550_g0.500",
            "dynamic_p0.700_g0.750",
            "dynamic_p0.850_g0.500",
            "dynamic_p0.850_g0.750",
        }
        self.assertEqual(len(cells), 36)
        self.assertEqual(len(selected), 5)
        svg = render_heatmap(cells, selected, candidate)
        root = ET.fromstring(svg)
        self.assertTrue(root.tag.endswith("svg"))
        self.assertIn("validated candidate: p=0.70", svg)
        self.assertIn("Mean computable score", svg)


if __name__ == "__main__":
    unittest.main()
