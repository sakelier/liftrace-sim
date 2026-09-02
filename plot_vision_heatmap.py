#!/usr/bin/env python3
"""Render the measured V-SIM-04 class-height-speed matrix as an SVG heatmap."""

from __future__ import annotations

import argparse
import csv
import html
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence


@dataclass(frozen=True)
class Cell:
    class_name: str
    height_m: float
    speed_mps: float
    successes: int
    trials: int

    @property
    def probability(self) -> float:
        return self.successes / self.trials


def load_dynamic_cells(path: Path) -> list[Cell]:
    cells = []
    with path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            if row["kind"] != "dynamic" or not row["speed_mps"]:
                continue
            cells.append(Cell(
                class_name=row["class_name"],
                height_m=float(row["height_m"]),
                speed_mps=float(row["speed_mps"]),
                successes=int(row["successes"]),
                trials=int(row["trials"]),
            ))
    if not cells:
        raise ValueError("no dynamic vision conditions found")
    return cells


def probability_color(probability: float) -> str:
    """White-yellow-red scale: 0=red, 0.5=yellow, 1=green."""
    probability = min(1.0, max(0.0, probability))
    if probability <= 0.5:
        t = probability / 0.5
        left, right = (215, 48, 39), (255, 224, 130)
    else:
        t = (probability - 0.5) / 0.5
        left, right = (255, 224, 130), (30, 136, 80)
    rgb = tuple(round(a + (b - a) * t) for a, b in zip(left, right))
    return "#%02x%02x%02x" % rgb


def render_heatmap(cells: Sequence[Cell], width: int = 1160, height: int = 760) -> str:
    classes = sorted({cell.class_name for cell in cells})
    heights = sorted({cell.height_m for cell in cells}, reverse=True)
    speeds = sorted({cell.speed_mps for cell in cells})
    lookup = {(c.class_name, c.height_m, c.speed_mps): c for c in cells}

    columns = 3
    rows = (len(classes) + columns - 1) // columns
    panel_w, panel_h = 350, 260
    gap_x, gap_y = 25, 28
    origin_x, origin_y = 65, 92
    cell_w = 62
    cell_h = 48
    grid_x_offset, grid_y_offset = 72, 46

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#fafafa"/>',
        '<text x="40" y="35" font-family="sans-serif" font-size="22" font-weight="bold">V-SIM-04 Dynamic Vision Performance</text>',
        '<text x="40" y="60" font-family="sans-serif" font-size="13" fill="#455a64">Cell = p_selected (successes/trials); blank = unmeasured; fixed seed 11, NOT a continuous probability surface</text>',
    ]

    for class_index, class_name in enumerate(classes):
        panel_col, panel_row = class_index % columns, class_index // columns
        px = origin_x + panel_col * (panel_w + gap_x)
        py = origin_y + panel_row * (panel_h + gap_y)
        gx, gy = px + grid_x_offset, py + grid_y_offset
        lines.append(f'<text x="{px}" y="{py + 17}" font-family="sans-serif" font-size="17" font-weight="bold">{html.escape(class_name)}</text>')
        for col, speed in enumerate(speeds):
            x = gx + col * cell_w
            lines.append(f'<text x="{x + cell_w / 2:.1f}" y="{gy - 9}" text-anchor="middle" font-family="sans-serif" font-size="12">{speed:g}</text>')
        for row, altitude in enumerate(heights):
            y = gy + row * cell_h
            lines.append(f'<text x="{gx - 10}" y="{y + cell_h / 2 + 4:.1f}" text-anchor="end" font-family="sans-serif" font-size="12">{altitude:g}</text>')
            for col, speed in enumerate(speeds):
                x = gx + col * cell_w
                cell = lookup.get((class_name, altitude, speed))
                fill = "#eeeeee" if cell is None else probability_color(cell.probability)
                lines.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{cell_w}" height="{cell_h}" fill="{fill}" stroke="#ffffff" stroke-width="2"/>')
                if cell is None:
                    label, detail, color = "—", "", "#9e9e9e"
                else:
                    label = f"{cell.probability:.2f}"
                    detail = f"{cell.successes}/{cell.trials}"
                    color = "#ffffff" if cell.probability <= 0.15 or cell.probability >= 0.8 else "#263238"
                lines.append(f'<text x="{x + cell_w / 2:.1f}" y="{y + 21:.1f}" text-anchor="middle" font-family="sans-serif" font-size="13" font-weight="bold" fill="{color}">{label}</text>')
                if detail:
                    lines.append(f'<text x="{x + cell_w / 2:.1f}" y="{y + 38:.1f}" text-anchor="middle" font-family="sans-serif" font-size="10" fill="{color}">{detail}</text>')
        lines.append(f'<text x="{gx + len(speeds) * cell_w / 2:.1f}" y="{gy + len(heights) * cell_h + 22:.1f}" text-anchor="middle" font-family="sans-serif" font-size="12">speed (m/s)</text>')
        lines.append(f'<text x="{px + 13}" y="{gy + len(heights) * cell_h / 2:.1f}" text-anchor="middle" font-family="sans-serif" font-size="12" transform="rotate(-90 {px + 13} {gy + len(heights) * cell_h / 2:.1f})">height (m)</text>')

    legend_y = origin_y + rows * (panel_h + gap_y) - 5
    legend_x = 380
    steps = 10
    lines.append(f'<text x="{legend_x - 115}" y="{legend_y + 16}" font-family="sans-serif" font-size="13">p_selected</text>')
    for index in range(steps + 1):
        p = index / steps
        x = legend_x + index * 34
        lines.append(f'<rect x="{x}" y="{legend_y}" width="34" height="18" fill="{probability_color(p)}"/>')
    lines.extend([
        f'<text x="{legend_x}" y="{legend_y + 35}" font-family="sans-serif" font-size="11">0.0</text>',
        f'<text x="{legend_x + 5 * 34}" y="{legend_y + 35}" text-anchor="middle" font-family="sans-serif" font-size="11">0.5</text>',
        f'<text x="{legend_x + 11 * 34}" y="{legend_y + 35}" text-anchor="end" font-family="sans-serif" font-size="11">1.0</text>',
        '</svg>',
    ])
    return "\n".join(lines) + "\n"


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=root / "data" / "vision" / "vsim04_20260902_v2" / "condition_success_rates.csv")
    parser.add_argument("--output", type=Path, default=root / "results" / "vision_performance_heatmap.svg")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    cells = load_dynamic_cells(args.data)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(render_heatmap(cells), encoding="utf-8")
    print(f"SVG: {args.output} ({len(cells)} measured dynamic conditions)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
