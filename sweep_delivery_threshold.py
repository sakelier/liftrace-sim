#!/usr/bin/env python3
"""Tune dynamic delivery thresholds, then validate candidates on a fresh seed."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Mapping, Sequence

from compare_delivery_policies import paired_difference, summarize_policy
from delivery_policy import DynamicThresholdPolicy, load_delivery_policies
from mission_sim import load_m2, run_mission_monte_carlo
from obstacles import load_obstacles, plan_route_astar
from occlusion import load_occlusion_config
from search_sim import generate_boustrophedon, load_camera, load_m1b, load_parameters


@dataclass(frozen=True)
class ThresholdParameters:
    uniform_future_cue_prior: float
    remaining_progress_exponent: float

    @property
    def policy_name(self) -> str:
        return (
            f"dynamic_p{self.uniform_future_cue_prior:.3f}"
            f"_g{self.remaining_progress_exponent:.3f}"
        )


@dataclass(frozen=True)
class ThresholdSweepResult:
    policy_name: str
    uniform_future_cue_prior: float
    remaining_progress_exponent: float
    mean_official_score: float
    score_ci95_low: float
    score_ci95_high: float
    worst_10_percent_mean_score: float
    red_cross_delivery_rate: float
    mean_cargo_attempt_utilization: float
    mean_elapsed_time_s: float
    mean_interruption_time_s: float
    timeout_rate: float


def parameter_grid(
    prior_values: Sequence[float],
    exponent_values: Sequence[float],
) -> list[ThresholdParameters]:
    if not prior_values or not exponent_values:
        raise ValueError("threshold sweep axes must not be empty")
    if any(not 0.0 <= value <= 1.0 for value in prior_values):
        raise ValueError("future Cue prior values must be in [0, 1]")
    if any(value <= 0.0 for value in exponent_values):
        raise ValueError("progress exponents must be positive")
    return [
        ThresholdParameters(float(prior), float(exponent))
        for prior in prior_values
        for exponent in exponent_values
    ]


def make_dynamic_policy(
    parameters: ThresholdParameters,
    class_names: Sequence[str],
) -> DynamicThresholdPolicy:
    return DynamicThresholdPolicy(
        {name: parameters.uniform_future_cue_prior for name in class_names},
        parameters.remaining_progress_exponent,
        name=parameters.policy_name,
    )


def compact_result(parameters: ThresholdParameters, metrics) -> ThresholdSweepResult:
    return ThresholdSweepResult(
        policy_name=parameters.policy_name,
        uniform_future_cue_prior=parameters.uniform_future_cue_prior,
        remaining_progress_exponent=parameters.remaining_progress_exponent,
        mean_official_score=metrics.mean_official_score,
        score_ci95_low=metrics.score_ci95_low,
        score_ci95_high=metrics.score_ci95_high,
        worst_10_percent_mean_score=metrics.worst_10_percent_mean_score,
        red_cross_delivery_rate=metrics.red_cross_delivery_rate,
        mean_cargo_attempt_utilization=metrics.mean_cargo_attempt_utilization,
        mean_elapsed_time_s=metrics.mean_elapsed_time_s,
        mean_interruption_time_s=metrics.mean_interruption_time_s,
        timeout_rate=metrics.timeout_rate,
    )


def select_candidates(
    results: Sequence[ThresholdSweepResult],
    top_k: int,
) -> list[ThresholdSweepResult]:
    if top_k <= 0:
        raise ValueError("validation_top_k must be positive")
    return sorted(
        results,
        key=lambda item: (-item.mean_official_score, item.mean_elapsed_time_s),
    )[:min(top_k, len(results))]


def main() -> int:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=root / "config" / "baseline.yaml")
    parser.add_argument("--tuning-trials", type=int)
    parser.add_argument("--validation-trials", type=int)
    parser.add_argument(
        "--output-json", type=Path,
        default=root / "results" / "dynamic_threshold_sweep.json",
    )
    parser.add_argument(
        "--output-csv", type=Path,
        default=root / "results" / "dynamic_threshold_sweep.csv",
    )
    args = parser.parse_args()

    params, config = load_parameters(args.config)
    sweep_config: Mapping[str, object] = config["delivery_decision"]["dynamic_threshold_sweep"]
    tuning_trials = args.tuning_trials or int(sweep_config["tuning_trials"])
    validation_trials = args.validation_trials or int(sweep_config["validation_trials"])
    if tuning_trials <= 0 or validation_trials <= 0:
        raise ValueError("tuning and validation trials must be positive")
    tuning_seed = int(sweep_config["tuning_seed"])
    validation_seed = int(sweep_config["validation_seed"])
    if tuning_seed == validation_seed:
        raise ValueError("tuning and validation seeds must differ")

    grid = parameter_grid(
        [float(value) for value in sweep_config["uniform_future_cue_prior_values"]],
        [float(value) for value in sweep_config["remaining_progress_exponents"]],
    )
    camera = load_camera(config)
    generation, cue_model, _, _ = load_m1b(config)
    mission = load_m2(config)
    class_names = tuple(generation.class_counts)
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

    tuning_results = []
    for index, parameters in enumerate(grid, start=1):
        policy = make_dynamic_policy(parameters, class_names)
        _, trials = run_mission_monte_carlo(
            params, plan.waypoints, camera, generation, cue_model, mission,
            tuning_trials, tuning_seed, obstacles, occlusion, policy,
        )
        metrics = summarize_policy(trials, mission.cargo_capacity)
        tuning_results.append(compact_result(parameters, metrics))
        print(
            f"tuning {index:02d}/{len(grid)} {policy.name}: "
            f"score={metrics.mean_official_score:.4f}, "
            f"time={metrics.mean_elapsed_time_s:.2f}s",
            flush=True,
        )

    selected = select_candidates(tuning_results, int(sweep_config["validation_top_k"]))
    configured_policies = load_delivery_policies(config)
    reference_names = tuple(sweep_config["reference_policies"])
    unknown_references = set(reference_names) - set(configured_policies)
    if unknown_references:
        raise ValueError(f"unknown reference policies: {sorted(unknown_references)}")

    validation_policies = {
        name: configured_policies[name] for name in reference_names
    }
    parameter_by_name = {
        item.policy_name: ThresholdParameters(
            item.uniform_future_cue_prior,
            item.remaining_progress_exponent,
        )
        for item in selected
    }
    validation_policies.update({
        name: make_dynamic_policy(parameters, class_names)
        for name, parameters in parameter_by_name.items()
    })

    validation_metrics = []
    validation_trials_by_policy = {}
    for policy in validation_policies.values():
        _, trials = run_mission_monte_carlo(
            params, plan.waypoints, camera, generation, cue_model, mission,
            validation_trials, validation_seed, obstacles, occlusion, policy,
        )
        validation_trials_by_policy[policy.name] = trials
        validation_metrics.append(summarize_policy(trials, mission.cargo_capacity))

    validation_ranking = sorted(
        validation_metrics,
        key=lambda item: (-item.mean_official_score, item.mean_elapsed_time_s),
    )
    paired_against_references = []
    for candidate_name in parameter_by_name:
        for reference_name in reference_names:
            paired_against_references.append(paired_difference(
                validation_trials_by_policy[candidate_name],
                validation_trials_by_policy[reference_name],
            ))

    payload = {
        "model_stage": "M2-D-TUNE",
        "protocol": {
            "tuning_trials_per_point": tuning_trials,
            "tuning_seed": tuning_seed,
            "validation_trials_per_policy": validation_trials,
            "validation_seed": validation_seed,
            "validation_top_k": len(selected),
            "common_random_numbers_within_each_phase": True,
            "independent_tuning_and_validation_seeds": True,
        },
        "grid_size": len(grid),
        "tuning_ranking": [asdict(item) for item in select_candidates(
            tuning_results, len(tuning_results)
        )],
        "selected_for_validation": [item.policy_name for item in selected],
        "validation_ranking": [item.policy_name for item in validation_ranking],
        "validation_metrics": [asdict(item) for item in validation_metrics],
        "validation_paired_against_references": [
            asdict(item) for item in paired_against_references
        ],
        "evidence_warning": (
            "The sweep tunes an assumed uniform future-Cue prior and progress exponent. "
            "Validation reduces seed overfitting but does not turn these inputs into measured facts."
        ),
    }

    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(asdict(tuning_results[0]).keys()))
        writer.writeheader()
        writer.writerows(asdict(item) for item in tuning_results)

    print("validation ranking: " + " > ".join(item.policy_name for item in validation_ranking))
    print(f"JSON: {args.output_json}")
    print(f"CSV:  {args.output_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

