import random
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from plot_route import render_svg  # noqa: E402
from search_sim import (  # noqa: E402
    generate_boustrophedon,
    generate_random_targets,
    load_m1b,
    load_parameters,
)


class RoutePlotTest(unittest.TestCase):
    def test_svg_is_valid_and_contains_route_and_targets(self):
        params, config = load_parameters(PROJECT_ROOT / "config" / "baseline.yaml")
        generation, _, _, _ = load_m1b(config)
        waypoints = generate_boustrophedon(params)
        targets = generate_random_targets(params.area, generation, random.Random(7))
        svg = render_svg(
            waypoints, targets,
            params.area.min_x, params.area.max_x, params.area.min_y, params.area.max_y,
        )
        root = ET.fromstring(svg)
        self.assertTrue(root.tag.endswith("svg"))
        self.assertIn("search lane", svg)
        for target in targets:
            self.assertIn(target.target_id, svg)


if __name__ == "__main__":
    unittest.main()
