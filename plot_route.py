#!/usr/bin/env python3
"""Render the configured 2-D search route and generated targets as SVG."""

from __future__ import annotations

import argparse
import html
import random
from pathlib import Path
from typing import Optional, Sequence

from search_sim import (
    Point3,
    Target,
    generate_boustrophedon,
    generate_random_targets,
    load_m1b,
    load_parameters,
)
from obstacles import (
    Obstacle,
    load_obstacles,
    plan_route_astar,
    route_collisions,
)


COLORS = {
    "tent": "#2ca02c",
    "pillbox": "#9467bd",
    "bridge": "#8c564b",
    "panzer": "#d62728",
    "red_cross": "#e41a1c",
}


def render_svg(
    waypoints: Sequence[Point3],
    targets: Sequence[Target],
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    obstacles: Sequence[Obstacle] = (),
    horizontal_clearance: float = 0.0,
    vertical_clearance: float = 0.0,
    planned_waypoints: Sequence[Point3] = (),
    width: int = 1000,
    height: int = 760,
) -> str:
    if len(waypoints) < 2:
        raise ValueError("at least two waypoints are required")
    margin_left, margin_right, margin_top, margin_bottom = 90, 220, 55, 75
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom
    scale = min(plot_w / (max_x - min_x), plot_h / (max_y - min_y))
    used_w, used_h = (max_x - min_x) * scale, (max_y - min_y) * scale
    ox = margin_left + (plot_w - used_w) / 2
    oy = margin_top + (plot_h - used_h) / 2

    def xy(x: float, y: float) -> tuple[float, float]:
        return ox + (x - min_x) * scale, oy + (max_y - y) * scale

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto" markerUnits="strokeWidth"><path d="M0,0 L8,4 L0,8 z" fill="#1565c0"/></marker></defs>',
        '<rect width="100%" height="100%" fill="#fafafa"/>',
        '<text x="40" y="32" font-family="sans-serif" font-size="22" font-weight="bold">Liftrace 2-D Search Route</text>',
    ]
    x0, y0 = xy(min_x, max_y)
    lines.append(f'<rect x="{x0:.2f}" y="{y0:.2f}" width="{used_w:.2f}" height="{used_h:.2f}" fill="#ffffff" stroke="#263238" stroke-width="2"/>')

    for obstacle in obstacles:
        min_ox, max_ox, min_oy, max_oy = obstacle.bounds(horizontal_clearance)
        ix, iy = xy(min_ox, max_oy)
        lines.append(f'<rect x="{ix:.2f}" y="{iy:.2f}" width="{(max_ox-min_ox)*scale:.2f}" height="{(max_oy-min_oy)*scale:.2f}" fill="#ef5350" fill-opacity="0.12" stroke="#ef5350" stroke-width="1.5" stroke-dasharray="5 4"/>')
        min_ox, max_ox, min_oy, max_oy = obstacle.bounds()
        rx, ry = xy(min_ox, max_oy)
        lines.append(f'<rect x="{rx:.2f}" y="{ry:.2f}" width="{(max_ox-min_ox)*scale:.2f}" height="{(max_oy-min_oy)*scale:.2f}" fill="#b71c1c" fill-opacity="0.72" stroke="#7f0000" stroke-width="2"/>')
        lines.append(f'<text x="{rx + 4:.2f}" y="{ry + 14:.2f}" font-family="sans-serif" font-size="10" fill="#ffffff">{html.escape(obstacle.obstacle_id)} ({obstacle.height:g}m)</text>')

    collisions = route_collisions(
        waypoints, obstacles, horizontal_clearance, vertical_clearance
    )
    collision_segments = {item.segment_index for item in collisions}

    for index in range(len(waypoints) - 1):
        a, b = waypoints[index], waypoints[index + 1]
        ax, ay = xy(a.x, a.y)
        bx, by = xy(b.x, b.y)
        lane = index % 2 == 0
        color = "#c62828" if index in collision_segments else ("#1565c0" if lane else "#78909c")
        dash = "" if lane else ' stroke-dasharray="7 5"'
        marker = ' marker-end="url(#arrow)"' if lane else ""
        lines.append(
            f'<line x1="{ax:.2f}" y1="{ay:.2f}" x2="{bx:.2f}" y2="{by:.2f}" '
            f'stroke="{color}" stroke-width="{4 if lane else 3}"{dash}{marker}/>'
        )

    if planned_waypoints:
        for a, b in zip(planned_waypoints, planned_waypoints[1:]):
            ax, ay = xy(a.x, a.y)
            bx, by = xy(b.x, b.y)
            lines.append(f'<line x1="{ax:.2f}" y1="{ay:.2f}" x2="{bx:.2f}" y2="{by:.2f}" stroke="#00897b" stroke-width="3.2"/>')
        for point in planned_waypoints[1:-1]:
            px, py = xy(point.x, point.y)
            lines.append(f'<circle cx="{px:.2f}" cy="{py:.2f}" r="2.6" fill="#00695c"/>')

    for index, point in enumerate(waypoints):
        px, py = xy(point.x, point.y)
        lines.append(f'<circle cx="{px:.2f}" cy="{py:.2f}" r="3.5" fill="#0d47a1"/>')
        lines.append(f'<text x="{px + 6:.2f}" y="{py - 6:.2f}" font-family="sans-serif" font-size="10" fill="#37474f">{index}</text>')

    for target in targets:
        tx, ty = xy(target.position.x, target.position.y)
        color = COLORS.get(target.class_name, "#f57c00")
        radius = max(5.0, min(target.size_x, target.size_y) * scale / 4.0)
        lines.append(f'<circle cx="{tx:.2f}" cy="{ty:.2f}" r="{radius:.2f}" fill="{color}" fill-opacity="0.78" stroke="#212121"/>')
        lines.append(f'<text x="{tx + radius + 4:.2f}" y="{ty + 4:.2f}" font-family="sans-serif" font-size="11">{html.escape(target.target_id)}</text>')

    sx, sy = xy(waypoints[0].x, waypoints[0].y)
    ex, ey = xy(waypoints[-1].x, waypoints[-1].y)
    lines.extend([
        f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="8" fill="#00a152" stroke="white" stroke-width="2"/>',
        f'<circle cx="{ex:.2f}" cy="{ey:.2f}" r="8" fill="#ff6f00" stroke="white" stroke-width="2"/>',
    ])

    legend_x = width - margin_right + 30
    lines.extend([
        f'<text x="{legend_x}" y="90" font-family="sans-serif" font-size="16" font-weight="bold">Legend</text>',
        f'<line x1="{legend_x}" y1="118" x2="{legend_x + 38}" y2="118" stroke="#1565c0" stroke-width="4"/><text x="{legend_x + 48}" y="123" font-family="sans-serif" font-size="13">search lane</text>',
        f'<line x1="{legend_x}" y1="148" x2="{legend_x + 38}" y2="148" stroke="#78909c" stroke-width="3" stroke-dasharray="7 5"/><text x="{legend_x + 48}" y="153" font-family="sans-serif" font-size="13">connector</text>',
        f'<circle cx="{legend_x + 8}" cy="180" r="7" fill="#00a152"/><text x="{legend_x + 48}" y="185" font-family="sans-serif" font-size="13">start</text>',
        f'<circle cx="{legend_x + 8}" cy="210" r="7" fill="#ff6f00"/><text x="{legend_x + 48}" y="215" font-family="sans-serif" font-size="13">finish</text>',
        f'<rect x="{legend_x + 1}" y="235" width="15" height="15" fill="#b71c1c"/><text x="{legend_x + 48}" y="248" font-family="sans-serif" font-size="13">obstacle</text>',
        f'<line x1="{legend_x}" y1="275" x2="{legend_x + 38}" y2="275" stroke="#c62828" stroke-width="4"/><text x="{legend_x + 48}" y="280" font-family="sans-serif" font-size="13">collision segment</text>',
        f'<text x="{legend_x}" y="315" font-family="sans-serif" font-size="12">collisions: {len(collisions)}</text>',
        f'<line x1="{legend_x}" y1="340" x2="{legend_x + 38}" y2="340" stroke="#00897b" stroke-width="3.2"/><text x="{legend_x + 48}" y="345" font-family="sans-serif" font-size="13">A* route</text>',
        f'<text x="{ox:.2f}" y="{height - 24}" font-family="sans-serif" font-size="13">X (m)</text>',
        f'<text x="20" y="{oy + used_h / 2:.2f}" font-family="sans-serif" font-size="13" transform="rotate(-90 20 {oy + used_h / 2:.2f})">Y (m)</text>',
    ])
    lines.append('</svg>')
    return "\n".join(lines) + "\n"


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=root / "config" / "baseline.yaml")
    parser.add_argument("--output", type=Path, default=root / "results" / "route_2d.svg")
    parser.add_argument("--seed", type=int, default=20260829)
    parser.add_argument("--no-targets", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    params, config = load_parameters(args.config)
    waypoints = generate_boustrophedon(params)
    targets: list[Target] = []
    if not args.no_targets:
        generation, _, _, _ = load_m1b(config)
        targets = generate_random_targets(params.area, generation, random.Random(args.seed))
    obstacles, horizontal_clearance, vertical_clearance = load_obstacles(config)
    obstacle_config = config.get("obstacles", {})
    plan = plan_route_astar(
        waypoints,
        obstacles,
        (params.area.min_x, params.area.max_x, params.area.min_y, params.area.max_y),
        float(obstacle_config.get("planning_resolution_m", 0.1)),
        horizontal_clearance,
        vertical_clearance,
        bool(obstacle_config.get("allow_diagonal", True)),
    )
    svg = render_svg(
        waypoints, targets,
        params.area.min_x, params.area.max_x, params.area.min_y, params.area.max_y,
        obstacles, horizontal_clearance, vertical_clearance,
        plan.waypoints,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(svg, encoding="utf-8")
    collisions = route_collisions(waypoints, obstacles, horizontal_clearance, vertical_clearance)
    print(f"SVG: {args.output} (waypoints={len(waypoints)}, targets={len(targets)}, "
          f"obstacles={len(obstacles)}, collisions={len(collisions)}, "
          f"A*_waypoints={len(plan.waypoints)}, replanned={plan.replanned_segment_count}, "
          f"seed={args.seed})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
