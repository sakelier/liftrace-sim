import json
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from plot_pareto import ParetoPoint, altitude_color, load_points, render_pareto  # noqa: E402


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

    def test_loads_sweep_json_written_by_sweep_sim(self):
        payload = {
            "results": [{
                "total_time_s": 47.1,
                "mean_cue_rate": 0.88,
                "altitude_m": 2.4,
                "speed_mps": 1.5,
                "lane_spacing_m": 2.0,
                "pareto_optimal": True,
            }],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sweep.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            points = load_points(path)
        self.assertEqual(len(points), 1)
        self.assertTrue(points[0].pareto_optimal)
        self.assertEqual(points[0].altitude_m, 2.4)


if __name__ == "__main__":
    unittest.main()
