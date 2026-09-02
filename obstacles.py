"""Axis-aligned obstacle geometry for task-level 2-D route analysis."""

from __future__ import annotations

import math
import heapq
from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Sequence, Tuple

from search_sim import Point3


EPSILON = 1.0e-9


@dataclass(frozen=True)
class Obstacle:
    obstacle_id: str
    center_x: float
    center_y: float
    size_x: float
    size_y: float
    height: float

    def __post_init__(self) -> None:
        values = (self.center_x, self.center_y, self.size_x, self.size_y, self.height)
        if not self.obstacle_id:
            raise ValueError("obstacle_id must not be empty")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("obstacle geometry must be finite")
        if self.size_x <= 0.0 or self.size_y <= 0.0 or self.height < 0.0:
            raise ValueError("obstacle sizes must be positive and height non-negative")

    def bounds(self, inflation: float = 0.0) -> Tuple[float, float, float, float]:
        if not math.isfinite(inflation) or inflation < 0.0:
            raise ValueError("inflation must be finite and non-negative")
        return (
            self.center_x - self.size_x / 2.0 - inflation,
            self.center_x + self.size_x / 2.0 + inflation,
            self.center_y - self.size_y / 2.0 - inflation,
            self.center_y + self.size_y / 2.0 + inflation,
        )


@dataclass(frozen=True)
class Collision:
    segment_index: int
    obstacle_id: str
    entry_fraction: float
    exit_fraction: float


@dataclass(frozen=True)
class AStarPlan:
    waypoints: tuple[Point3, ...]
    original_segment_count: int
    replanned_segment_count: int
    expanded_node_count: int


def segment_rectangle_interval(
    start: Point3,
    end: Point3,
    bounds: Tuple[float, float, float, float],
) -> Optional[Tuple[float, float]]:
    """Return the closed [entry, exit] fraction where a segment crosses a rectangle."""
    min_x, max_x, min_y, max_y = bounds
    dx, dy = end.x - start.x, end.y - start.y
    enter, exit_ = 0.0, 1.0
    for origin, delta, lower, upper in (
        (start.x, dx, min_x, max_x),
        (start.y, dy, min_y, max_y),
    ):
        if abs(delta) <= EPSILON:
            if origin < lower - EPSILON or origin > upper + EPSILON:
                return None
            continue
        low_t, high_t = (lower - origin) / delta, (upper - origin) / delta
        if low_t > high_t:
            low_t, high_t = high_t, low_t
        enter, exit_ = max(enter, low_t), min(exit_, high_t)
        if enter > exit_ + EPSILON:
            return None
    return max(0.0, enter), min(1.0, exit_)


def route_collisions(
    waypoints: Sequence[Point3],
    obstacles: Iterable[Obstacle],
    horizontal_clearance: float,
    vertical_clearance: float = 0.0,
) -> list[Collision]:
    if horizontal_clearance < 0.0 or vertical_clearance < 0.0:
        raise ValueError("clearances must be non-negative")
    obstacle_list = list(obstacles)
    collisions = []
    for index, (start, end) in enumerate(zip(waypoints, waypoints[1:])):
        minimum_altitude = min(start.z, end.z)
        for obstacle in obstacle_list:
            if minimum_altitude > obstacle.height + vertical_clearance + EPSILON:
                continue
            interval = segment_rectangle_interval(start, end, obstacle.bounds(horizontal_clearance))
            if interval is not None:
                collisions.append(Collision(index, obstacle.obstacle_id, *interval))
    return collisions


def line_of_sight_blockers(
    camera: Point3,
    target: Point3,
    obstacles: Iterable[Obstacle],
) -> list[str]:
    """Return obstacles whose tops intersect the camera-to-target sight ray."""
    blockers = []
    for obstacle in obstacles:
        interval = segment_rectangle_interval(camera, target, obstacle.bounds())
        if interval is None:
            continue
        entry, exit_ = interval
        # Ignore contact exactly at camera/target endpoints; those are placement errors.
        probe_entry = max(entry, EPSILON)
        probe_exit = min(exit_, 1.0 - EPSILON)
        if probe_entry > probe_exit:
            continue
        entry_height = camera.z + probe_entry * (target.z - camera.z)
        exit_height = camera.z + probe_exit * (target.z - camera.z)
        if obstacle.height >= min(entry_height, exit_height) - EPSILON:
            blockers.append(obstacle.obstacle_id)
    return blockers


def load_obstacles(config: dict) -> tuple[list[Obstacle], float, float]:
    section = config.get("obstacles", {})
    items = [
        Obstacle(
            obstacle_id=str(item["id"]),
            center_x=float(item["center_x_m"]),
            center_y=float(item["center_y_m"]),
            size_x=float(item["size_x_m"]),
            size_y=float(item["size_y_m"]),
            height=float(item["height_m"]),
        )
        for item in section.get("fixtures", [])
    ]
    return (
        items,
        float(section.get("horizontal_clearance_m", 0.0)),
        float(section.get("vertical_clearance_m", 0.0)),
    )


def _edge_is_clear(
    start: Point3,
    end: Point3,
    obstacles: Sequence[Obstacle],
    horizontal_clearance: float,
    vertical_clearance: float,
) -> bool:
    return not route_collisions(
        (start, end), obstacles, horizontal_clearance, vertical_clearance
    )


def _simplify_path(
    points: Sequence[Point3],
    obstacles: Sequence[Obstacle],
    horizontal_clearance: float,
    vertical_clearance: float,
) -> list[Point3]:
    if len(points) <= 2:
        return list(points)
    simplified = [points[0]]
    current = 0
    while current < len(points) - 1:
        candidate = len(points) - 1
        while candidate > current + 1 and not _edge_is_clear(
            points[current], points[candidate], obstacles,
            horizontal_clearance, vertical_clearance,
        ):
            candidate -= 1
        simplified.append(points[candidate])
        current = candidate
    return simplified


def astar_segment(
    start: Point3,
    goal: Point3,
    obstacles: Sequence[Obstacle],
    area_bounds: Tuple[float, float, float, float],
    resolution: float,
    horizontal_clearance: float,
    vertical_clearance: float,
    allow_diagonal: bool = True,
) -> tuple[list[Point3], int]:
    """Plan one constant-altitude segment on a bounded occupancy grid."""
    if not math.isclose(start.z, goal.z, abs_tol=EPSILON):
        raise ValueError("A* currently requires constant-altitude endpoints")
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("A* resolution must be finite and positive")
    min_x, max_x, min_y, max_y = area_bounds
    nx = round((max_x - min_x) / resolution)
    ny = round((max_y - min_y) / resolution)

    def node(point: Point3) -> tuple[int, int]:
        ix = round((point.x - min_x) / resolution)
        iy = round((point.y - min_y) / resolution)
        snapped_x, snapped_y = min_x + ix * resolution, min_y + iy * resolution
        if not math.isclose(snapped_x, point.x, abs_tol=1.0e-6) or not math.isclose(
            snapped_y, point.y, abs_tol=1.0e-6
        ):
            raise ValueError("A* endpoints must align with the configured grid resolution")
        if not (0 <= ix <= nx and 0 <= iy <= ny):
            raise ValueError("A* endpoint lies outside the search area")
        return ix, iy

    def point(item: tuple[int, int]) -> Point3:
        return Point3(min_x + item[0] * resolution, min_y + item[1] * resolution, start.z)

    start_node, goal_node = node(start), node(goal)
    if not _edge_is_clear(start, start, obstacles, horizontal_clearance, vertical_clearance):
        raise RuntimeError("A* start is inside an inflated obstacle")
    if not _edge_is_clear(goal, goal, obstacles, horizontal_clearance, vertical_clearance):
        raise RuntimeError("A* goal is inside an inflated obstacle")

    offsets = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    if allow_diagonal:
        offsets += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
    open_heap: list[tuple[float, float, int, int]] = []
    heapq.heappush(open_heap, (0.0, 0.0, *start_node))
    costs: Dict[tuple[int, int], float] = {start_node: 0.0}
    parents: Dict[tuple[int, int], tuple[int, int]] = {}
    closed: set[tuple[int, int]] = set()
    expanded = 0

    while open_heap:
        _, cost, ix, iy = heapq.heappop(open_heap)
        current = (ix, iy)
        if current in closed:
            continue
        closed.add(current)
        expanded += 1
        if current == goal_node:
            break
        current_point = point(current)
        for dx, dy in offsets:
            neighbor = (ix + dx, iy + dy)
            if not (0 <= neighbor[0] <= nx and 0 <= neighbor[1] <= ny):
                continue
            neighbor_point = point(neighbor)
            if not _edge_is_clear(
                current_point, neighbor_point, obstacles,
                horizontal_clearance, vertical_clearance,
            ):
                continue
            step = resolution * math.hypot(dx, dy)
            new_cost = cost + step
            if new_cost + EPSILON >= costs.get(neighbor, math.inf):
                continue
            costs[neighbor] = new_cost
            parents[neighbor] = current
            heuristic = resolution * math.hypot(
                goal_node[0] - neighbor[0], goal_node[1] - neighbor[1]
            )
            heapq.heappush(open_heap, (new_cost + heuristic, new_cost, *neighbor))
    else:
        raise RuntimeError(f"A* found no route from {start} to {goal}")

    path_nodes = [goal_node]
    while path_nodes[-1] != start_node:
        path_nodes.append(parents[path_nodes[-1]])
    path_nodes.reverse()
    raw_path = [point(item) for item in path_nodes]
    return _simplify_path(
        raw_path, obstacles, horizontal_clearance, vertical_clearance
    ), expanded


def plan_route_astar(
    waypoints: Sequence[Point3],
    obstacles: Sequence[Obstacle],
    area_bounds: Tuple[float, float, float, float],
    resolution: float,
    horizontal_clearance: float,
    vertical_clearance: float = 0.0,
    allow_diagonal: bool = True,
) -> AStarPlan:
    """Replace each colliding route segment with a simplified A* detour."""
    if len(waypoints) < 2:
        raise ValueError("at least two waypoints are required")
    planned = [waypoints[0]]
    replanned = 0
    expanded = 0
    for start, goal in zip(waypoints, waypoints[1:]):
        if _edge_is_clear(start, goal, obstacles, horizontal_clearance, vertical_clearance):
            segment_path = [start, goal]
        else:
            segment_path, count = astar_segment(
                start, goal, obstacles, area_bounds, resolution,
                horizontal_clearance, vertical_clearance, allow_diagonal,
            )
            replanned += 1
            expanded += count
        planned.extend(segment_path[1:])
    return AStarPlan(tuple(planned), len(waypoints) - 1, replanned, expanded)
