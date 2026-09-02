import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from plot_dynamic_threshold_heatmap import (  # noqa: E402
    load_selection,
    load_sweep,
    render_heatmap,
)


class DynamicThresholdHeatmapTest(unittest.TestCase):
    def test_checked_in_sweep_renders_valid_svg(self):
        cells = load_sweep(PROJECT_ROOT / "results" / "dynamic_threshold_sweep.csv")
        selected, candidate = load_selection(
            PROJECT_ROOT / "results" / "dynamic_threshold_sweep.json"
        )
        self.assertEqual(len(cells), 36)
        self.assertEqual(len(selected), 5)
        svg = render_heatmap(cells, selected, candidate)
        root = ET.fromstring(svg)
        self.assertTrue(root.tag.endswith("svg"))
        self.assertIn("validated candidate: p=0.40", svg)
        self.assertIn("Mean computable score", svg)


if __name__ == "__main__":
    unittest.main()
