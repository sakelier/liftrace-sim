import sys,unittest,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from r2026_scene import scene_layout,NOMINAL_TREES,PATTERNS,rectangle_vertices,polygons_overlap

class SceneTest(unittest.TestCase):
    def test_wall_intersection_without_corners_inside(self):
        plate=rectangle_vertices(0,0,1,1)
        wall=rectangle_vertices(0,0,2,.1)
        self.assertTrue(polygons_overlap(plate,wall))
        self.assertFalse(polygons_overlap(plate,rectangle_vertices(2,0,1,1)))
    def test_nominal_fixture_is_preserved(self):
        s=scene_layout(0);self.assertEqual(s['door_pattern'],'LR');self.assertEqual([(t['x'],t['y']) for t in s['trees']],NOMINAL_TREES)
    def test_geometry_and_reproducibility(self):
        patterns=set()
        for seed in range(1,501):
            s=scene_layout(seed);self.assertEqual(s,scene_layout(seed));patterns.add(s['door_pattern'])
            for d in s['doors']:self.assertAlmostEqual(d['gap_max_y']-d['gap_min_y'],.8)
            for i,t in enumerate(s['trees']):
                self.assertGreater(math.hypot(t['x'],t['y']),1.15)
                for u in s['trees'][:i]:self.assertGreaterEqual(math.hypot(t['x']-u['x'],t['y']-u['y']),1.2)
        self.assertEqual(patterns,set(PATTERNS))
    def test_independent_random_factors(self):
        a=scene_layout(1,door_seed=2,obstacle_seed=3);b=scene_layout(1,door_seed=99,obstacle_seed=3)
        self.assertEqual(a['trees'],b['trees'])
        self.assertEqual(a['doors'],scene_layout(1,door_seed=2,obstacle_seed=7)['doors'])
if __name__=='__main__':unittest.main()
