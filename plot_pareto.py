#!/usr/bin/env python3
"""Render search time versus Cue rate and the Pareto frontier as SVG."""

from __future__ import annotations

import argparse
import csv
import html
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence


@dataclass(frozen=True)
class ParetoPoint:
    total_time_s: float
    mean_cue_rate: float
    altitude_m: float
    speed_mps: float
    lane_spacing_m: float
    pareto_optimal: bool


def load_points(path: Path) -> list[ParetoPoint]:
    if path.suffix.lower() == ".json":
        payload = json.loads(path.read_text(encoding="utf-8"))
        rows = payload["results"]
    else:
        with path.open(encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))

    points = []
    for row in rows:
        raw_pareto = row["pareto_optimal"]
        is_pareto = (raw_pareto if isinstance(raw_pareto, bool)
                     else raw_pareto.strip().lower() == "true")
        points.append(ParetoPoint(
            total_time_s=float(row["total_time_s"]),
            mean_cue_rate=float(row["mean_cue_rate"]),
            altitude_m=float(row["altitude_m"]),
            speed_mps=float(row["speed_mps"]),
            lane_spacing_m=float(row["lane_spacing_m"]),
            pareto_optimal=is_pareto,
        ))
    if not points:
        raise ValueError("Pareto input contains no points")
    return points


def altitude_color(altitude: float, minimum: float, maximum: float) -> str:
    fraction = 0.5 if maximum == minimum else (altitude - minimum) / (maximum - minimum)
    fraction = min(1.0, max(0.0, fraction))
    left, right = (49, 130, 189), (222, 45, 38)
    rgb = tuple(round(a + (b - a) * fraction) for a, b in zip(left, right))
    return "#%02x%02x%02x" % rgb


def render_pareto(points: Sequence[ParetoPoint], width: int = 1120, height: int = 760) -> str:
    margin_left, margin_right, margin_top, margin_bottom = 105, 235, 75, 90
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom
    min_time, max_time = min(p.total_time_s for p in points), max(p.total_time_s for p in points)
    min_rate, max_rate = min(p.mean_cue_rate for p in points), max(p.mean_cue_rate for p in points)
    time_pad = max((max_time - min_time) * 0.05, 1.0)
    rate_pad = max((max_rate - min_rate) * 0.08, 0.02)
    x_min, x_max = max(0.0, min_time - time_pad), max_time + time_pad
    y_min, y_max = max(0.0, min_rate - rate_pad), min(1.0, max_rate + rate_pad)
    if y_max <= y_min:
        y_max = min(1.0, y_min + 0.1)
    min_alt, max_alt = min(p.altitude_m for p in points), max(p.altitude_m for p in points)

    def xy(point: ParetoPoint) -> tuple[float, float]:
        x = margin_left + (point.total_time_s - x_min) / (x_max - x_min) * plot_w
        y = margin_top + (y_max - point.mean_cue_rate) / (y_max - y_min) * plot_h
        return x, y

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#fafafa"/>',
        '<text x="40" y="35" font-family="sans-serif" font-size="22" font-weight="bold">Search Time vs Cue Rate</text>',
        '<text x="40" y="59" font-family="sans-serif" font-size="13" fill="#455a64">Lower time and higher Cue rate are better; outlined points form the sampled Pareto frontier</text>',
        f'<rect x="{margin_left}" y="{margin_top}" width="{plot_w}" height="{plot_h}" fill="#ffffff" stroke="#263238" stroke-width="1.5"/>',
    ]

    ticks = 5
    for index in range(ticks + 1):
        value = x_min + (x_max - x_min) * index / ticks
        x = margin_left + plot_w * index / ticks
        lines.extend([
            f'<line x1="{x:.1f}" y1="{margin_top}" x2="{x:.1f}" y2="{margin_top + plot_h}" stroke="#eceff1"/>',
            f'<text x="{x:.1f}" y="{margin_top + plot_h + 25}" text-anchor="middle" font-family="sans-serif" font-size="12">{value:.0f}</text>',
        ])
        rate = y_min + (y_max - y_min) * index / ticks
        y = margin_top + plot_h - plot_h * index / ticks
        lines.extend([
            f'<line x1="{margin_left}" y1="{y:.1f}" x2="{margin_left + plot_w}" y2="{y:.1f}" stroke="#eceff1"/>',
            f'<text x="{margin_left - 12}" y="{y + 4:.1f}" text-anchor="end" font-family="sans-serif" font-size="12">{rate:.2f}</text>',
        ])

    frontier = sorted((p for p in points if p.pareto_optimal), key=lambda p: p.total_time_s)
    if frontier:
        coords = " ".join(f"{x:.1f},{y:.1f}" for x, y in map(xy, frontier))
        lines.append(f'<polyline points="{coords}" fill="none" stroke="#111111" stroke-width="2.5" stroke-dasharray="6 4"/>')

    for point in sorted(points, key=lambda p: p.pareto_optimal):
        x, y = xy(point)
        color = altitude_color(point.altitude_m, min_alt, max_alt)
        radius = 7 if point.pareto_optimal else 4
        stroke = "#111111" if point.pareto_optimal else "#ffffff"
        stroke_w = 2.5 if point.pareto_optimal else 0.8
        tooltip = html.escape(
            f"time={point.total_time_s:.2f}s, cue={point.mean_cue_rate:.4f}, "
            f"h={point.altitude_m:g}m, v={point.speed_mps:g}m/s, d={point.lane_spacing_m:g}m"
        )
        lines.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{radius}" fill="{color}" fill-opacity="0.82" stroke="{stroke}" stroke-width="{stroke_w}"><title>{tooltip}</title></circle>')

    for index, point in enumerate(frontier):
        x, y = xy(point)
        dy = -10 if index % 2 == 0 else 18
        label = f"h{point.altitude_m:g}/v{point.speed_mps:g}/d{point.lane_spacing_m:g}"
        lines.append(f'<text x="{x + 8:.1f}" y="{y + dy:.1f}" font-family="sans-serif" font-size="10" fill="#212121">{label}</text>')

    lines.extend([
        f'<text x="{margin_left + plot_w / 2:.1f}" y="{height - 28}" text-anchor="middle" font-family="sans-serif" font-size="14">Total search time (s)</text>',
        f'<text x="28" y="{margin_top + plot_h / 2:.1f}" text-anchor="middle" font-family="sans-serif" font-size="14" transform="rotate(-90 28 {margin_top + plot_h / 2:.1f})">Mean Cue rate</text>',
    ])
    legend_x = width - margin_right + 42
    lines.append(f'<text x="{legend_x}" y="110" font-family="sans-serif" font-size="16" font-weight="bold">Altitude (m)</text>')
    for index in range(7):
        fraction = index / 6
        altitude = min_alt + (max_alt - min_alt) * fraction
        y = 135 + index * 28
        color = altitude_color(altitude, min_alt, max_alt)
        lines.append(f'<circle cx="{legend_x + 9}" cy="{y}" r="7" fill="{color}"/><text x="{legend_x + 26}" y="{y + 4}" font-family="sans-serif" font-size="12">{altitude:.1f}</text>')
    lines.extend([
        f'<circle cx="{legend_x + 9}" cy="355" r="7" fill="#ffffff" stroke="#111111" stroke-width="2.5"/><text x="{legend_x + 26}" y="359" font-family="sans-serif" font-size="12">Pareto optimal</text>',
        f'<text x="{legend_x}" y="405" font-family="sans-serif" font-size="12">points: {len(points)}</text>',
        f'<text x="{legend_x}" y="425" font-family="sans-serif" font-size="12">Pareto: {len(frontier)}</text>',
        '</svg>',
    ])
    return "\n".join(lines) + "\n"


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=root / "results" / "m1b_sweep.csv")
    parser.add_argument("--output", type=Path, default=root / "results" / "pareto_time_cue.svg")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    points = load_points(args.input)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(render_pareto(points), encoding="utf-8")
    print(f"SVG: {args.output} (points={len(points)}, Pareto={sum(p.pareto_optimal for p in points)})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
