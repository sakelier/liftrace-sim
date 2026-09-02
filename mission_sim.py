#!/usr/bin/env python3
"""M2 interrupted search, verification, delivery and resume simulation."""

from __future__ import annotations

import argparse
import json
import math
import random
import statistics
from collections import Counter
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Mapping, Optional, Sequence

from obstacles import Obstacle, load_obstacles, plan_route_astar
from occlusion import OcclusionConfig, evaluate_occlusions, load_occlusion_config
from delivery_policy import (
    DeliveryDecisionContext,
    DeliveryPolicy,
    FirstSeenPolicy,
    load_delivery_policies,
)
from search_sim import (
    CameraModel,
    CueModel,
    M0Parameters,
    Point3,
    Target,
    TargetGenerationConfig,
    cue_probability,
    evaluate_target_visibility,
    generate_boustrophedon,
    generate_random_targets,
    load_camera,
    load_m1b,
    load_parameters,
    segment_start_times,
)
from scoring import (
    DeliveryOutcome,
    FLIGHT_CEILING_M,
    MISSION_TIME_LIMIT_S,
    ScoreBreakdown,
    ScoringScenario,
    TARGET_WEIGHTS,
    score_trial,
)


@dataclass(frozen=True)
class MissionConfig:
    deadline_s: float
    transit_altitude_m: float
    approach_altitude_m: float
    horizontal_speed_mps: float
    vertical_speed_mps: float
    verification_hold_s: float
    delivery_service_s: float
    verification_probability_by_class: Mapping[str, float]
    delivery_probability_by_class: Mapping[str, float]
    cargo_capacity: int = 3
    flight_ceiling_m: float = FLIGHT_CEILING_M
    successful_delivery_placement: str = "full_score"
    scoring_scenario: ScoringScenario = field(default_factory=lambda: ScoringScenario(
        carried_parcels=0,
        autonomous_takeoff_qualified=None,
        obstacle_avoidance_qualified=None,
        door_pass_results=(),
        landing_result="not_modelled",
    ))

    def __post_init__(self) -> None:
        positive = (
            self.deadline_s, self.transit_altitude_m, self.approach_altitude_m,
            self.horizontal_speed_mps, self.vertical_speed_mps,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("M2 deadline, altitudes and speeds must be positive")
        if self.verification_hold_s < 0.0 or self.delivery_service_s < 0.0:
            raise ValueError("M2 service durations must be non-negative")
        if self.cargo_capacity < 0:
            raise ValueError("M2 cargo capacity must be non-negative")
        if self.successful_delivery_placement != "full_score":
            raise ValueError("only the current full_score delivery placement model is supported")
        if max(self.transit_altitude_m, self.approach_altitude_m) > self.flight_ceiling_m:
            raise ValueError("M2 altitude exceeds the competition flight ceiling")
        probabilities = (
            list(self.verification_probability_by_class.values())
            + list(self.delivery_probability_by_class.values())
        )
        if any(not 0.0 <= value <= 1.0 for value in probabilities):
            raise ValueError("M2 probabilities must be in [0, 1]")


@dataclass(frozen=True)
class CueCandidate:
    target_id: str
    class_name: str
    search_time_s: float
    position: Point3


@dataclass(frozen=True)
class MissionEvent:
    time_s: float
    state: str
    target_id: Optional[str]
    detail: str


@dataclass(frozen=True)
class MissionTrialResult:
    policy_name: str
    target_count: int
    cue_count: int
    confirmed_count: int
    delivery_attempt_count: int
    delivered_count: int
    search_completed: bool
    timed_out: bool
    elapsed_time_s: float
    search_time_completed_s: float
    interruption_time_s: float
    delivery_outcomes: tuple[DeliveryOutcome, ...]
    official_score: ScoreBreakdown
    events: tuple[MissionEvent, ...]


@dataclass(frozen=True)
class MissionSummary:
    policy_name: str
    trials: int
    seed: int
    mean_cue_count: float
    mean_confirmed_count: float
    mean_delivered_count: float
    probability_search_completed: float
    probability_all_targets_delivered: float
    probability_all_cargo_delivered: float
    timeout_rate: float
    mean_elapsed_time_s: float
    mean_interruption_time_s: float
    mean_official_score: float
    maximum_official_score: float
    minimum_official_score: float
    complete_official_score_rate: float


def load_m2(config: Mapping[str, object]) -> MissionConfig:
    item = config["m2"]
    rules = config["rules"]
    scoring_item = config["scoring"]
    deadline = float(item["mission_deadline_s"])
    rule_deadline = float(rules["mission_time_limit_s"])
    rule_ceiling = float(rules["field_outer_size_m"][2])
    if not math.isclose(deadline, rule_deadline, rel_tol=0.0, abs_tol=1.0e-9):
        raise ValueError("M2 deadline must equal the official rule time limit")
    if not math.isclose(rule_deadline, MISSION_TIME_LIMIT_S, abs_tol=1.0e-9):
        raise ValueError("configured rule time limit disagrees with the official scoring model")
    if not math.isclose(rule_ceiling, FLIGHT_CEILING_M, abs_tol=1.0e-9):
        raise ValueError("configured field ceiling disagrees with the official scoring model")
    configured_weights = {name: float(value) for name, value in rules["targets"].items()}
    if configured_weights != dict(TARGET_WEIGHTS):
        raise ValueError("configured target weights disagree with the official scoring model")
    return MissionConfig(
        deadline_s=deadline,
        transit_altitude_m=float(item["transit_altitude_m"]),
        approach_altitude_m=float(item["approach_altitude_m"]),
        horizontal_speed_mps=float(item["horizontal_service_speed_mps"]),
        vertical_speed_mps=float(item["vertical_service_speed_mps"]),
        verification_hold_s=float(item["verification_hold_s"]),
        delivery_service_s=float(item["delivery_service_s"]),
        verification_probability_by_class={
            name: float(value)
            for name, value in item["verification_probability_by_class"].items()
        },
        delivery_probability_by_class={
            name: float(value)
            for name, value in item["delivery_probability_by_class"].items()
        },
        cargo_capacity=int(rules["cargo_count"]),
        flight_ceiling_m=rule_ceiling,
        successful_delivery_placement=str(scoring_item["successful_delivery_placement"]),
        scoring_scenario=ScoringScenario(
            carried_parcels=int(rules["cargo_count"]),
            autonomous_takeoff_qualified=scoring_item["autonomous_takeoff_qualified"],
            obstacle_avoidance_qualified=scoring_item["obstacle_avoidance_qualified"],
            door_pass_results=tuple(scoring_item["door_pass_results"]),
            landing_result=str(scoring_item["landing_result"]),
        ),
    )


def _route_duration(waypoints: Sequence[Point3], speed: float, turn_penalty: float) -> float:
    starts = segment_start_times(waypoints, speed, turn_penalty)
    return starts[-1] + math.dist(
        (waypoints[-2].x, waypoints[-2].y, waypoints[-2].z),
        (waypoints[-1].x, waypoints[-1].y, waypoints[-1].z),
    ) / speed


def _cue_candidates(
    waypoints: Sequence[Point3],
    targets: Sequence[Target],
    observations,
    cue_model: CueModel,
    speed: float,
    turn_penalty: float,
    rng: random.Random,
) -> list[CueCandidate]:
    starts = segment_start_times(waypoints, speed, turn_penalty)
    candidates = []
    for target in targets:
        passes = sorted(
            (item for item in observations if item.target_id == target.target_id),
            key=lambda item: (item.segment_index, item.entry_distance_m),
        )
        for item in passes:
            if rng.random() >= cue_probability(item, cue_model):
                continue
            offset = item.entry_distance_m / speed + (item.first_effective_offset_s or 0.0)
            distance = min(
                item.exit_distance_m,
                item.entry_distance_m + speed * (item.first_effective_offset_s or 0.0),
            )
            start, end = waypoints[item.segment_index], waypoints[item.segment_index + 1]
            length = math.dist((start.x, start.y, start.z), (end.x, end.y, end.z))
            fraction = 0.0 if length == 0.0 else distance / length
            candidates.append(CueCandidate(
                target.target_id,
                target.class_name,
                starts[item.segment_index] + offset,
                Point3(
                    start.x + fraction * (end.x - start.x),
                    start.y + fraction * (end.y - start.y),
                    start.z + fraction * (end.z - start.z),
                ),
            ))
            break
    return sorted(candidates, key=lambda item: (item.search_time_s, item.target_id))


def _one_way_service_time(cue: CueCandidate, target: Target, config: MissionConfig) -> float:
    horizontal = math.hypot(
        cue.position.x - target.position.x,
        cue.position.y - target.position.y,
    )
    vertical = (
        abs(config.transit_altitude_m - cue.position.z)
        + abs(config.transit_altitude_m - config.approach_altitude_m)
    )
    return (
        horizontal / config.horizontal_speed_mps
        + vertical / config.vertical_speed_mps
    )


def simulate_mission_trial(
    params: M0Parameters,
    waypoints: Sequence[Point3],
    camera: CameraModel,
    targets: Sequence[Target],
    cue_model: CueModel,
    mission: MissionConfig,
    cue_rng: random.Random,
    service_rng: random.Random,
    obstacles: Sequence[Obstacle] = (),
    occlusion_config: Optional[OcclusionConfig] = None,
    delivery_policy: Optional[DeliveryPolicy] = None,
) -> MissionTrialResult:
    policy = delivery_policy or FirstSeenPolicy()
    observations = evaluate_target_visibility(
        waypoints, targets, camera, params.speed, require_fully_in_frame=True
    )
    if obstacles and occlusion_config is not None:
        observations = evaluate_occlusions(
            waypoints, targets, observations, obstacles, occlusion_config
        )
    candidates = _cue_candidates(
        waypoints, targets, observations, cue_model,
        params.speed, params.turn_penalty, cue_rng,
    )
    target_by_id = {target.target_id: target for target in targets}
    class_counts = Counter(target.class_name for target in targets)
    seen_counts: Counter[str] = Counter()
    service_draws = {
        target_id: (service_rng.random(), service_rng.random())
        for target_id in sorted(target_by_id)
    }
    total_search_time = _route_duration(waypoints, params.speed, params.turn_penalty)
    elapsed = search_progress = interruption_time = 0.0
    cues = confirmed = attempts = delivered = 0
    timed_out = False
    events = [MissionEvent(0.0, "SEARCH", None, "mission_start")]
    delivery_outcomes: list[DeliveryOutcome] = []

    def consume(duration: float, state: str, target_id: str, detail: str) -> bool:
        nonlocal elapsed, timed_out
        if elapsed + duration > mission.deadline_s + 1.0e-9:
            elapsed = mission.deadline_s
            timed_out = True
            events.append(MissionEvent(elapsed, "TIMEOUT", target_id, f"during_{state.lower()}"))
            return False
        elapsed += duration
        events.append(MissionEvent(elapsed, state, target_id, detail))
        return True

    for cue in candidates:
        search_delta = cue.search_time_s - search_progress
        if not consume(search_delta, "CUE", cue.target_id, "stable_visual_cue"):
            break
        search_progress = cue.search_time_s
        cues += 1
        target = target_by_id[cue.target_id]
        seen_counts[cue.class_name] += 1
        one_way = _one_way_service_time(cue, target, mission)
        decision = policy.decide(DeliveryDecisionContext(
            target_class=cue.class_name,
            elapsed_time_s=elapsed,
            deadline_s=mission.deadline_s,
            search_progress_fraction=(
                1.0 if total_search_time == 0.0 else search_progress / total_search_time
            ),
            cargo_remaining=mission.cargo_capacity - attempts,
            estimated_service_time_s=(
                2.0 * one_way
                + mission.verification_hold_s
                + mission.delivery_service_s
            ),
            class_counts=class_counts,
            seen_counts=seen_counts,
            verification_probability_by_class=mission.verification_probability_by_class,
            delivery_probability_by_class=mission.delivery_probability_by_class,
        ))
        if not decision.deliver:
            state = "NO_CARGO" if decision.reason == "no_cargo" else "SKIP_DELIVERY"
            events.append(MissionEvent(
                elapsed,
                state,
                cue.target_id,
                (
                    f"{decision.policy_name}:{decision.reason};"
                    f"expected={decision.expected_score:.3f};"
                    f"threshold={decision.opportunity_threshold:.3f}"
                ),
            ))
            continue
        service_start = elapsed
        if not consume(one_way, "APPROACH", cue.target_id, "arrived_at_verification_pose"):
            break
        if not consume(mission.verification_hold_s, "VERIFY", cue.target_id, "verification_complete"):
            break
        verification_draw, delivery_draw = service_draws[cue.target_id]
        verified = verification_draw < mission.verification_probability_by_class[cue.class_name]
        if verified:
            confirmed += 1
            attempts += 1
            if not consume(mission.delivery_service_s, "DELIVER", cue.target_id, "delivery_attempted"):
                break
            if delivery_draw < mission.delivery_probability_by_class[cue.class_name]:
                delivered += 1
                placement = (
                    "random_inside" if cue.class_name == "red_cross" else "standard_ring"
                )
                delivery_outcomes.append(DeliveryOutcome(
                    cue.class_name,
                    placement,
                    1 if placement == "standard_ring" else None,
                ))
                events.append(MissionEvent(elapsed, "DELIVERED", cue.target_id, "delivery_success"))
            else:
                delivery_outcomes.append(DeliveryOutcome(cue.class_name, "outside"))
                events.append(MissionEvent(elapsed, "DELIVERY_FAILED", cue.target_id, "delivery_failure"))
        else:
            events.append(MissionEvent(elapsed, "VERIFY_FAILED", cue.target_id, "verification_failure"))
        if not consume(one_way, "RETURN", cue.target_id, "resumed_at_interruption_point"):
            break
        interruption_time += elapsed - service_start
        events.append(MissionEvent(elapsed, "SEARCH", None, "search_resumed"))

    if not timed_out:
        remaining = total_search_time - search_progress
        if elapsed + remaining <= mission.deadline_s + 1.0e-9:
            elapsed += remaining
            search_progress = total_search_time
            events.append(MissionEvent(elapsed, "COMPLETE", None, "search_route_complete"))
        else:
            search_progress += mission.deadline_s - elapsed
            elapsed = mission.deadline_s
            timed_out = True
            events.append(MissionEvent(elapsed, "TIMEOUT", None, "during_search"))

    official_score = score_trial(mission.scoring_scenario, delivery_outcomes, elapsed)
    return MissionTrialResult(
        policy_name=policy.name,
        target_count=len(targets),
        cue_count=cues,
        confirmed_count=confirmed,
        delivery_attempt_count=attempts,
        delivered_count=delivered,
        search_completed=search_progress >= total_search_time - 1.0e-9,
        timed_out=timed_out,
        elapsed_time_s=elapsed,
        search_time_completed_s=search_progress,
        interruption_time_s=interruption_time,
        delivery_outcomes=tuple(delivery_outcomes),
        official_score=official_score,
        events=tuple(events),
    )


def run_mission_monte_carlo(
    params: M0Parameters,
    waypoints: Sequence[Point3],
    camera: CameraModel,
    generation: TargetGenerationConfig,
    cue_model: CueModel,
    mission: MissionConfig,
    trials: int,
    seed: int,
    obstacles: Sequence[Obstacle] = (),
    occlusion_config: Optional[OcclusionConfig] = None,
    delivery_policy: Optional[DeliveryPolicy] = None,
) -> tuple[MissionSummary, list[MissionTrialResult]]:
    policy = delivery_policy or FirstSeenPolicy()
    results = []
    forbidden = tuple(obstacle.bounds() for obstacle in obstacles)
    for index in range(trials):
        targets = generate_random_targets(
            params.area, generation, random.Random(seed + 3 * index), forbidden
        )
        results.append(simulate_mission_trial(
            params, waypoints, camera, targets, cue_model, mission,
            random.Random(seed + 3 * index + 1),
            random.Random(seed + 3 * index + 2),
            obstacles, occlusion_config, policy,
        ))
    total_targets = sum(generation.class_counts.values())
    summary = MissionSummary(
        policy_name=policy.name,
        trials=trials,
        seed=seed,
        mean_cue_count=statistics.fmean(item.cue_count for item in results),
        mean_confirmed_count=statistics.fmean(item.confirmed_count for item in results),
        mean_delivered_count=statistics.fmean(item.delivered_count for item in results),
        probability_search_completed=sum(item.search_completed for item in results) / trials,
        probability_all_targets_delivered=(
            sum(item.delivered_count == total_targets for item in results) / trials
        ),
        probability_all_cargo_delivered=(
            sum(
                item.delivered_count == min(mission.cargo_capacity, total_targets)
                for item in results
            ) / trials
        ),
        timeout_rate=sum(item.timed_out for item in results) / trials,
        mean_elapsed_time_s=statistics.fmean(item.elapsed_time_s for item in results),
        mean_interruption_time_s=statistics.fmean(item.interruption_time_s for item in results),
        mean_official_score=statistics.fmean(item.official_score.total_score for item in results),
        maximum_official_score=max(item.official_score.total_score for item in results),
        minimum_official_score=min(item.official_score.total_score for item in results),
        complete_official_score_rate=(
            sum(item.official_score.complete_official_score for item in results) / trials
        ),
    )
    return summary, results


def main() -> int:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=root / "config" / "baseline.yaml")
    parser.add_argument("--trials", type=int, default=200)
    parser.add_argument("--seed", type=int, default=20260831)
    parser.add_argument("--output", type=Path, default=root / "results" / "m2_mission.json")
    args = parser.parse_args()

    params, config = load_parameters(args.config)
    camera = load_camera(config)
    generation, cue_model, _, _ = load_m1b(config)
    mission = load_m2(config)
    policies = load_delivery_policies(config)
    active_policy_name = str(config["delivery_decision"]["active_policy"])
    if active_policy_name not in policies:
        raise ValueError("active delivery policy must be included in compare_policies")
    active_policy = policies[active_policy_name]
    obstacles, clearance, vertical_clearance = load_obstacles(config)
    nominal = generate_boustrophedon(params)
    obstacle_config = config["obstacles"]
    plan = plan_route_astar(
        nominal, obstacles,
        (params.area.min_x, params.area.max_x, params.area.min_y, params.area.max_y),
        float(obstacle_config["planning_resolution_m"]), clearance, vertical_clearance,
        bool(obstacle_config["allow_diagonal"]),
    )
    summary, results = run_mission_monte_carlo(
        params, plan.waypoints, camera, generation, cue_model, mission,
        args.trials, args.seed, obstacles, load_occlusion_config(config), active_policy,
    )
    payload = {
        "model_stage": "M2-S",
        "rule_source": {
            "document": config["rules"]["source_document"],
            "scoring_pages": config["rules"]["scoring_pages"],
        },
        "mission_config": asdict(mission),
        "delivery_policy": active_policy_name,
        "summary": asdict(summary),
        "example_trial": asdict(results[0]),
        "evidence_warning": (
            "Official point constants are RULE evidence; M2 service probabilities and "
            "successful-delivery innermost/inside placement are ASSUMED. Unmodelled "
            "official items receive zero and are listed explicitly."
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(asdict(summary), ensure_ascii=False, indent=2))
    print(f"JSON: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
