#!/usr/bin/env python3
"""Compare online delivery policies on common random mission scenarios."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path

from delivery_policy import load_delivery_policies
from mission_sim import load_m2, run_mission_monte_carlo
from obstacles import load_obstacles, plan_route_astar
from occlusion import load_occlusion_config
from search_sim import (
    generate_boustrophedon,
    load_camera,
    load_m1b,
    load_parameters,
)


@dataclass(frozen=True)
class PolicyMetrics:
    policy_name: str
    trials: int
    mean_official_score: float
    score_std: float
    score_ci95_low: float
    score_ci95_high: float
    worst_10_percent_mean_score: float
    mean_delivery_score: float
    red_cross_delivery_rate: float
    mean_cargo_attempt_utilization: float
    mean_cargo_success_utilization: float
    search_completion_rate: float
    timeout_rate: float
    mean_elapsed_time_s: float
    mean_interruption_time_s: float


@dataclass(frozen=True)
class PairedPolicyDifference:
    policy_a: str
    policy_b: str
    mean_score_delta_a_minus_b: float
    score_delta_ci95_low: float
    score_delta_ci95_high: float
    probability_a_scores_higher: float
    probability_equal_score: float
    mean_elapsed_time_delta_a_minus_b_s: float
    clear_mean_score_difference_at_95pct: bool


def summarize_policy(results, cargo_capacity: int) -> PolicyMetrics:
    if not results:
        raise ValueError("cannot summarize an empty result set")
    scores = [item.official_score.total_score for item in results]
    score_std = statistics.stdev(scores) if len(scores) > 1 else 0.0
    half_width = 1.96 * score_std / math.sqrt(len(scores))
    worst_count = max(1, math.ceil(0.10 * len(scores)))
    worst_scores = sorted(scores)[:worst_count]
    capacity_denominator = max(1, cargo_capacity)
    return PolicyMetrics(
        policy_name=results[0].policy_name,
        trials=len(results),
        mean_official_score=statistics.fmean(scores),
        score_std=score_std,
        score_ci95_low=statistics.fmean(scores) - half_width,
        score_ci95_high=statistics.fmean(scores) + half_width,
        worst_10_percent_mean_score=statistics.fmean(worst_scores),
        mean_delivery_score=statistics.fmean(
            item.official_score.delivery_score for item in results
        ),
        red_cross_delivery_rate=(
            sum(
                any(
                    outcome.target_class == "red_cross" and outcome.placement != "outside"
                    for outcome in item.delivery_outcomes
                )
                for item in results
            ) / len(results)
        ),
        mean_cargo_attempt_utilization=statistics.fmean(
            item.delivery_attempt_count / capacity_denominator for item in results
        ),
        mean_cargo_success_utilization=statistics.fmean(
            item.delivered_count / capacity_denominator for item in results
        ),
        search_completion_rate=sum(item.search_completed for item in results) / len(results),
        timeout_rate=sum(item.timed_out for item in results) / len(results),
        mean_elapsed_time_s=statistics.fmean(item.elapsed_time_s for item in results),
        mean_interruption_time_s=statistics.fmean(
            item.interruption_time_s for item in results
        ),
    )


def paired_difference(results_a, results_b) -> PairedPolicyDifference:
    if len(results_a) != len(results_b) or not results_a:
        raise ValueError("paired result sets must be non-empty and equal length")
    score_deltas = [
        a.official_score.total_score - b.official_score.total_score
        for a, b in zip(results_a, results_b)
    ]
    mean_delta = statistics.fmean(score_deltas)
    delta_std = statistics.stdev(score_deltas) if len(score_deltas) > 1 else 0.0
    half_width = 1.96 * delta_std / math.sqrt(len(score_deltas))
    low, high = mean_delta - half_width, mean_delta + half_width
    return PairedPolicyDifference(
        policy_a=results_a[0].policy_name,
        policy_b=results_b[0].policy_name,
        mean_score_delta_a_minus_b=mean_delta,
        score_delta_ci95_low=low,
        score_delta_ci95_high=high,
        probability_a_scores_higher=(
            sum(delta > 0.0 for delta in score_deltas) / len(score_deltas)
        ),
        probability_equal_score=(
            sum(delta == 0.0 for delta in score_deltas) / len(score_deltas)
        ),
        mean_elapsed_time_delta_a_minus_b_s=statistics.fmean(
            a.elapsed_time_s - b.elapsed_time_s
            for a, b in zip(results_a, results_b)
        ),
        clear_mean_score_difference_at_95pct=(low > 0.0 or high < 0.0),
    )


def main() -> int:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=root / "config" / "baseline.yaml")
    parser.add_argument("--trials", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=20260831)
    parser.add_argument(
        "--output-json", type=Path,
        default=root / "results" / "delivery_policy_comparison.json",
    )
    parser.add_argument(
        "--output-csv", type=Path,
        default=root / "results" / "delivery_policy_comparison.csv",
    )
    args = parser.parse_args()
    if args.trials <= 0:
        raise ValueError("trials must be positive")

    params, config = load_parameters(args.config)
    camera = load_camera(config)
    generation, cue_model, _, _ = load_m1b(config)
    mission = load_m2(config)
    policies = load_delivery_policies(config)
    obstacles, clearance, vertical_clearance = load_obstacles(config)
    nominal = generate_boustrophedon(params)
    obstacle_config = config["obstacles"]
    plan = plan_route_astar(
        nominal,
        obstacles,
        (params.area.min_x, params.area.max_x, params.area.min_y, params.area.max_y),
        float(obstacle_config["planning_resolution_m"]),
        clearance,
        vertical_clearance,
        bool(obstacle_config["allow_diagonal"]),
    )
    occlusion = load_occlusion_config(config)

    metrics = []
    result_sets = {}
    for policy in policies.values():
        _, results = run_mission_monte_carlo(
            params,
            plan.waypoints,
            camera,
            generation,
            cue_model,
            mission,
            args.trials,
            args.seed,
            obstacles,
            occlusion,
            policy,
        )
        result_sets[policy.name] = results
        metrics.append(summarize_policy(results, mission.cargo_capacity))

    paired = []
    policy_names = list(result_sets)
    for index, policy_a in enumerate(policy_names):
        for policy_b in policy_names[index + 1:]:
            paired.append(paired_difference(result_sets[policy_a], result_sets[policy_b]))

    ranking = sorted(
        metrics,
        key=lambda item: (-item.mean_official_score, item.mean_elapsed_time_s),
    )
    payload = {
        "model_stage": "M2-D",
        "trials_per_policy": args.trials,
        "master_seed": args.seed,
        "common_random_numbers": {
            "target_layouts": True,
            "cue_draws": True,
            "verification_and_delivery_draws_bound_to_target_id": True,
        },
        "ranking_rule": "higher mean score, then shorter mean elapsed time",
        "ranking": [item.policy_name for item in ranking],
        "policies": [asdict(item) for item in metrics],
        "paired_policy_differences": [asdict(item) for item in paired],
        "evidence_warning": (
            "Policy comparison uses official point constants, but future Cue forecasts, "
            "service probabilities and successful-delivery placement remain assumptions."
        ),
    }

    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(asdict(metrics[0]).keys()))
        writer.writeheader()
        writer.writerows(asdict(item) for item in metrics)

    print(json.dumps(payload, ensure_ascii=False, indent=2))
    print(f"JSON: {args.output_json}")
    print(f"CSV:  {args.output_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
