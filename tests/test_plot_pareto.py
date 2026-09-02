import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from plot_pareto import ParetoPoint, altitude_color, render_pareto  # noqa: E402


class ParetoPlotTest(unittest.TestCase):
    def test_svg_contains_frontier_and_parameters(self):
        points = [
            ParetoPoint(10.0, 0.5, 1.2, 1.0, 1.0, True),
            ParetoPoint(20.0, 0.8, 2.4, 0.5, 0.8, True),
            ParetoPoint(25.0, 0.4, 3.2, 0.5, 0.6, False),
        ]
        svg = render_pareto(points)
        root = ET.fromstring(svg)
        self.assertTrue(root.tag.endswith("svg"))
        self.assertIn("Pareto optimal", svg)
        self.assertIn("h1.2/v1/d1", svg)
        self.assertIn("polyline", svg)

    def test_altitude_color_clamps(self):
        self.assertEqual(altitude_color(0.0, 1.0, 3.0), altitude_color(1.0, 1.0, 3.0))
        self.assertEqual(altitude_color(4.0, 1.0, 3.0), altitude_color(3.0, 1.0, 3.0))


if __name__ == "__main__":
    unittest.main()
