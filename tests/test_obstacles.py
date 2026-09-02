import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from obstacles import (  # noqa: E402
    Obstacle,
    line_of_sight_blockers,
    plan_route_astar,
    route_collisions,
    segment_rectangle_interval,
)
from search_sim import Point3  # noqa: E402


class ObstacleGeometryTest(unittest.TestCase):
    def setUp(self):
        self.block = Obstacle("block", 2.0, 0.0, 1.0, 1.0, 2.0)

    def test_segment_rectangle_intersection_and_miss(self):
        interval = segment_rectangle_interval(
            Point3(0, 0, 1), Point3(4, 0, 1), self.block.bounds())
        self.assertEqual(interval, (0.375, 0.625))
        self.assertIsNone(segment_rectangle_interval(
            Point3(0, 2, 1), Point3(4, 2, 1), self.block.bounds()))

    def test_route_collision_respects_altitude_and_clearance(self):
        route = [Point3(0, 0.6, 2.1), Point3(4, 0.6, 2.1)]
        self.assertEqual(route_collisions(route, [self.block], 0.0), [])
        self.assertEqual(len(route_collisions(route, [self.block], 0.2, 0.2)), 1)
        high = [Point3(0, 0, 2.3), Point3(4, 0, 2.3)]
        self.assertEqual(route_collisions(high, [self.block], 0.2, 0.2), [])

    def test_line_of_sight_uses_obstacle_height(self):
        camera, target = Point3(0, 0, 3), Point3(4, 0, 0)
        self.assertEqual(line_of_sight_blockers(camera, target, [self.block]), ["block"])
        low = Obstacle("low", 2.0, 0.0, 1.0, 1.0, 0.2)
        self.assertEqual(line_of_sight_blockers(camera, target, [low]), [])

    def test_astar_detour_is_collision_free_and_deterministic(self):
        route = [Point3(0, 0, 1), Point3(4, 0, 1)]
        kwargs = dict(
            waypoints=route,
            obstacles=[self.block],
            area_bounds=(0.0, 4.0, -2.0, 2.0),
            resolution=0.25,
            horizontal_clearance=0.25,
        )
        first = plan_route_astar(**kwargs)
        second = plan_route_astar(**kwargs)
        self.assertEqual(first, second)
        self.assertEqual(first.replanned_segment_count, 1)
        self.assertGreater(len(first.waypoints), 2)
        self.assertEqual(route_collisions(first.waypoints, [self.block], 0.25), [])


if __name__ == "__main__":
    unittest.main()
