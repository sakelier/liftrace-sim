#!/usr/bin/env python3
"""Grid-sweep M1b altitude, speed and lane spacing with common scenarios."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass, replace
from itertools import product
from pathlib import Path
from typing import List, Mapping, Optional, Sequence

import yaml

from search_sim import (
    M0Parameters,
    evaluate_m0,
    generate_boustrophedon,
    load_camera,
    load_m1b,
    load_parameters,
    run_monte_carlo,
)
from obstacles import load_obstacles, plan_route_astar
from occlusion import load_occlusion_config


@dataclass(frozen=True)
class SweepResult:
    altitude_m: float
    speed_mps: float
    lane_spacing_m: float
    lane_count: int
    route_distance_m: float
    total_time_s: float
    mean_cue_rate: float
    mean_targets_cued: float
    probability_all_targets_cued: float
    mean_first_cue_time_s: Optional[float]
    pareto_optimal: bool = False


def load_sweep_grid(path: Path) -> Mapping[str, object]:
    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    grid = config["grid"]
    for key in ("altitude_m", "speed_mps", "lane_spacing_m"):
        values = grid[key]
        if not values or any(float(value) <= 0.0 for value in values):
            raise ValueError(f"grid {key} must contain positive values")
    return config


def pareto_flags(results: Sequence[SweepResult]) -> List[bool]:
    """Mark points not dominated on lower time and higher mean Cue rate."""
    flags: List[bool] = []
    for candidate in results:
        dominated = any(
            other.total_time_s <= candidate.total_time_s
            and other.mean_cue_rate >= candidate.mean_cue_rate
            and (other.total_time_s < candidate.total_time_s
                 or other.mean_cue_rate > candidate.mean_cue_rate)
            for other in results
        )
        flags.append(not dominated)
    return flags


def run_sweep(
        baseline: M0Parameters,
        baseline_config: Mapping[str, object],
        sweep_config: Mapping[str, object],
) -> List[SweepResult]:
    camera = load_camera(baseline_config)
    generation, cue_model, _, _ = load_m1b(baseline_config)
    trials = int(sweep_config["monte_carlo"]["trials"])
    seed = int(sweep_config["monte_carlo"]["seed"])
    grid = sweep_config["grid"]
    obstacles, horizontal_clearance, vertical_clearance = load_obstacles(
        dict(baseline_config)
    )
    obstacle_config = baseline_config.get("obstacles", {})
    occlusion_config = load_occlusion_config(baseline_config)
    area_bounds = (
        baseline.area.min_x, baseline.area.max_x,
        baseline.area.min_y, baseline.area.max_y,
    )
    results: List[SweepResult] = []
    route_cache = {}

    for altitude, speed, spacing in product(
            grid["altitude_m"], grid["speed_mps"], grid["lane_spacing_m"]):
        params = replace(
            baseline,
            altitude=float(altitude),
            speed=float(speed),
            lane_spacing=float(spacing),
        )
        route_key = (params.altitude, params.lane_spacing)
        if route_key not in route_cache:
            nominal_waypoints = generate_boustrophedon(params)
            geometry = evaluate_m0(params, nominal_waypoints)
            plan = plan_route_astar(
                nominal_waypoints,
                obstacles,
                area_bounds,
                float(obstacle_config.get("planning_resolution_m", 0.1)),
                horizontal_clearance,
                vertical_clearance,
                bool(obstacle_config.get("allow_diagonal", True)),
            )
            planned_waypoints = plan.waypoints
            route_distance = sum(
                math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
                for a, b in zip(planned_waypoints, planned_waypoints[1:])
            )
            turn_count = 0
            for before, current, after in zip(
                planned_waypoints, planned_waypoints[1:], planned_waypoints[2:]
            ):
                first = (current.x - before.x, current.y - before.y)
                second = (after.x - current.x, after.y - current.y)
                cross = first[0] * second[1] - first[1] * second[0]
                dot = first[0] * second[0] + first[1] * second[1]
                if abs(cross) > 1.0e-8 or dot < 0.0:
                    turn_count += 1
            route_cache[route_key] = (
                planned_waypoints, geometry.lane_count, route_distance, turn_count
            )
        waypoints, lane_count, route_distance, turn_count = route_cache[route_key]
        total_time = route_distance / params.speed + turn_count * params.turn_penalty
        simulation = run_monte_carlo(
            params, waypoints, camera, generation, cue_model, trials, seed,
            obstacles=obstacles,
            occlusion_config=occlusion_config,
        )
        results.append(SweepResult(
            altitude_m=params.altitude,
            speed_mps=params.speed,
            lane_spacing_m=params.lane_spacing,
            lane_count=lane_count,
            route_distance_m=route_distance,
            total_time_s=total_time,
            mean_cue_rate=simulation.mean_cue_rate,
            mean_targets_cued=simulation.mean_targets_cued,
            probability_all_targets_cued=simulation.probability_all_targets_cued,
            mean_first_cue_time_s=simulation.mean_first_cue_time_s,
        ))
    flags = pareto_flags(results)
    return [replace(item, pareto_optimal=flag) for item, flag in zip(results, flags)]


def write_csv(path: Path, results: Sequence[SweepResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(asdict(results[0]).keys())
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(asdict(item) for item in results)


def write_json(path: Path, results: Sequence[SweepResult], sweep: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pareto = [item for item in results if item.pareto_optimal]
    maximum_altitude = max(float(value) for value in sweep["grid"]["altitude_m"])
    payload = {
        "experiment": "M1b_parameter_sweep",
        "monte_carlo": sweep["monte_carlo"],
        "grid": sweep["grid"],
        "result_count": len(results),
        "pareto_count": sum(item.pareto_optimal for item in results),
        "diagnostics": {
            "pareto_all_at_maximum_altitude": all(
                item.altitude_m == maximum_altitude for item in pareto),
            "maximum_grid_altitude_m": maximum_altitude,
            "interpretation": (
                "Boundary saturation indicates missing or insufficient altitude penalty; "
                "it is not evidence that the maximum altitude is physically optimal."
            ),
        },
        "results": [asdict(item) for item in results],
    }
    with path.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=root / "config" / "baseline.yaml")
    parser.add_argument("--sweep", type=Path, default=root / "config" / "sweep.yaml")
    parser.add_argument("--output-csv", type=Path, default=root / "results" / "m1b_sweep.csv")
    parser.add_argument("--output-json", type=Path, default=root / "results" / "m1b_sweep.json")
    parser.add_argument("--trials", type=int, help="override trials per grid point")
    parser.add_argument("--seed", type=int, help="override common scenario seed")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    baseline, baseline_config = load_parameters(args.baseline)
    sweep = dict(load_sweep_grid(args.sweep))
    sweep["monte_carlo"] = dict(sweep["monte_carlo"])
    if args.trials is not None:
        sweep["monte_carlo"]["trials"] = args.trials
    if args.seed is not None:
        sweep["monte_carlo"]["seed"] = args.seed
    results = run_sweep(baseline, baseline_config, sweep)
    write_csv(args.output_csv, results)
    write_json(args.output_json, results, sweep)

    pareto = sorted(
        (item for item in results if item.pareto_optimal),
        key=lambda item: item.total_time_s,
    )
    print(f"grid points: {len(results)}, Pareto points: {len(pareto)}")
    print("Pareto frontier (time_s, cue_rate, altitude_m, speed_mps, spacing_m):")
    for item in pareto:
        print(f"  {item.total_time_s:8.3f}  {item.mean_cue_rate:.4f}  "
              f"h={item.altitude_m:.2f} v={item.speed_mps:.2f} d={item.lane_spacing_m:.2f}")
    maximum_altitude = max(float(value) for value in sweep["grid"]["altitude_m"])
    if pareto and all(item.altitude_m == maximum_altitude for item in pareto):
        print("WARNING: every Pareto point is on the maximum-altitude boundary.")
        print("This placeholder Cue model has no image-resolution/altitude penalty; "
              "do not interpret the boundary as a physical optimum.")
    print(f"CSV: {args.output_csv}")
    print(f"JSON: {args.output_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
