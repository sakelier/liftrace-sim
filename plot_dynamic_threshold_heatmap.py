#!/usr/bin/env python3
"""Render dynamic-threshold score and time sweeps as an SVG heatmap."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence


@dataclass(frozen=True)
class SweepCell:
    policy_name: str
    prior: float
    exponent: float
    score: float
    elapsed_s: float


def load_sweep(csv_path: Path) -> list[SweepCell]:
    with csv_path.open(encoding="utf-8", newline="") as stream:
        cells = [
            SweepCell(
                row["policy_name"],
                float(row["uniform_future_cue_prior"]),
                float(row["remaining_progress_exponent"]),
                float(row["mean_official_score"]),
                float(row["mean_elapsed_time_s"]),
            )
            for row in csv.DictReader(stream)
        ]
    if not cells:
        raise ValueError("threshold sweep CSV is empty")
    return cells


def load_selection(json_path: Path) -> tuple[set[str], str]:
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    selected = set(payload["selected_for_validation"])
    ranking = payload["validation_ranking"]
    candidate = next(name for name in ranking if name.startswith("dynamic_p"))
    return selected, candidate


def _interpolate_color(value: float, minimum: float, maximum: float, low, high) -> str:
    fraction = 0.5 if maximum == minimum else (value - minimum) / (maximum - minimum)
    fraction = min(1.0, max(0.0, fraction))
    rgb = tuple(round(a + (b - a) * fraction) for a, b in zip(low, high))
    return "#%02x%02x%02x" % rgb


def render_heatmap(
    cells: Sequence[SweepCell],
    selected: set[str],
    candidate: str,
    width: int = 1260,
    height: int = 650,
) -> str:
    priors = sorted({cell.prior for cell in cells})
    exponents = sorted({cell.exponent for cell in cells}, reverse=True)
    lookup = {(cell.prior, cell.exponent): cell for cell in cells}
    scores = [cell.score for cell in cells]
    times = [cell.elapsed_s for cell in cells]
    cell_w, cell_h = 78, 58
    panel_origins = (105, 685)
    grid_y = 145

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#fafafa"/>',
        '<text x="40" y="38" font-family="sans-serif" font-size="23" font-weight="bold">Dynamic Threshold Parameter Sweep</text>',
        '<text x="40" y="65" font-family="sans-serif" font-size="13" fill="#455a64">36 tuning points × 500 scenarios; outlined cells entered independent validation; gold outline = selected candidate</text>',
        '<text x="40" y="88" font-family="sans-serif" font-size="12" fill="#b71c1c">p and γ are policy hyperparameters tuned on an assumed model, not measured physical Cue probabilities</text>',
    ]

    panels = (
        ("Mean computable score (higher is better)", "score"),
        ("Mean mission time / s (lower is better)", "time"),
    )
    for origin_x, (title, metric) in zip(panel_origins, panels):
        lines.append(
            f'<text x="{origin_x}" y="118" font-family="sans-serif" font-size="17" '
            f'font-weight="bold">{title}</text>'
        )
        for column, prior in enumerate(priors):
            x = origin_x + column * cell_w
            lines.append(
                f'<text x="{x + cell_w / 2:.1f}" y="{grid_y - 10}" text-anchor="middle" '
                f'font-family="sans-serif" font-size="12">{prior:.2f}</text>'
            )
        for row, exponent in enumerate(exponents):
            y = grid_y + row * cell_h
            lines.append(
                f'<text x="{origin_x - 12}" y="{y + cell_h / 2 + 4:.1f}" text-anchor="end" '
                f'font-family="sans-serif" font-size="12">{exponent:.2f}</text>'
            )
            for column, prior in enumerate(priors):
                x = origin_x + column * cell_w
                cell = lookup[(prior, exponent)]
                if metric == "score":
                    value = cell.score
                    fill = _interpolate_color(
                        value, min(scores), max(scores), (255, 243, 224), (21, 101, 192)
                    )
                    label = f"{value:.2f}"
                else:
                    value = cell.elapsed_s
                    # Reverse the palette so shorter times are greener.
                    fill = _interpolate_color(
                        value, min(times), max(times), (46, 125, 50), (198, 40, 40)
                    )
                    label = f"{value:.1f}"
                if cell.policy_name == candidate:
                    stroke, stroke_width = "#ffb300", 5
                elif cell.policy_name in selected:
                    stroke, stroke_width = "#263238", 3
                else:
                    stroke, stroke_width = "#ffffff", 2
                lines.append(
                    f'<rect x="{x}" y="{y}" width="{cell_w}" height="{cell_h}" '
                    f'fill="{fill}" stroke="{stroke}" stroke-width="{stroke_width}"/>'
                )
                lines.append(
                    f'<text x="{x + cell_w / 2:.1f}" y="{y + cell_h / 2 + 5:.1f}" '
                    f'text-anchor="middle" font-family="sans-serif" font-size="13" '
                    f'font-weight="bold" fill="#102027">{label}</text>'
                )
        center_x = origin_x + len(priors) * cell_w / 2
        center_y = grid_y + len(exponents) * cell_h / 2
        lines.extend([
            f'<text x="{center_x:.1f}" y="{grid_y + len(exponents) * cell_h + 30}" '
            'text-anchor="middle" font-family="sans-serif" font-size="13">uniform future-Cue prior p</text>',
            f'<text x="{origin_x - 65}" y="{center_y:.1f}" text-anchor="middle" '
            f'font-family="sans-serif" font-size="13" transform="rotate(-90 {origin_x - 65} {center_y:.1f})">progress exponent γ</text>',
        ])

    candidate_cell = next(cell for cell in cells if cell.policy_name == candidate)
    lines.extend([
        '<rect x="390" y="555" width="22" height="16" fill="#eeeeee" stroke="#263238" stroke-width="3"/>',
        '<text x="420" y="568" font-family="sans-serif" font-size="12">top-5 tuning candidate</text>',
        '<rect x="610" y="555" width="22" height="16" fill="#eeeeee" stroke="#ffb300" stroke-width="5"/>',
        f'<text x="642" y="568" font-family="sans-serif" font-size="12">validated candidate: p={candidate_cell.prior:.2f}, γ={candidate_cell.exponent:.2f}</text>',
        '<text x="40" y="620" font-family="sans-serif" font-size="12" fill="#455a64">Color ranges are normalized independently per panel; use printed cell values for exact comparison.</text>',
        '</svg>',
    ])
    return "\n".join(lines) + "\n"


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--csv", type=Path, default=root / "results" / "dynamic_threshold_sweep.csv"
    )
    parser.add_argument(
        "--json", type=Path, default=root / "results" / "dynamic_threshold_sweep.json"
    )
    parser.add_argument(
        "--output", type=Path,
        default=root / "results" / "dynamic_threshold_heatmap.svg",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    cells = load_sweep(args.csv)
    selected, candidate = load_selection(args.json)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(render_heatmap(cells, selected, candidate), encoding="utf-8")
    print(f"SVG: {args.output} ({len(cells)} parameter combinations)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

