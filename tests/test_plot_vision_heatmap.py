import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from plot_vision_heatmap import (  # noqa: E402
    load_dynamic_cells,
    probability_color,
    render_heatmap,
)


class VisionHeatmapTest(unittest.TestCase):
    def test_checked_in_table_renders_valid_full_grid_svg(self):
        path = PROJECT_ROOT / "data" / "vision" / "vsim04_20260902_v2" / "condition_success_rates.csv"
        cells = load_dynamic_cells(path)
        self.assertEqual(len(cells), 100)
        svg = render_heatmap(cells)
        root = ET.fromstring(svg)
        self.assertTrue(root.tag.endswith("svg"))
        self.assertIn("pillbox", svg)
        self.assertIn("blank = unmeasured", svg)
        self.assertIn("1/1", svg)

    def test_color_scale_has_distinct_endpoints(self):
        self.assertNotEqual(probability_color(0.0), probability_color(1.0))
        self.assertEqual(probability_color(-1.0), probability_color(0.0))
        self.assertEqual(probability_color(2.0), probability_color(1.0))


if __name__ == "__main__":
    unittest.main()
