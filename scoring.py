#!/usr/bin/env python3
"""Official competition scoring primitives for task-level simulation.

The constants in this module come from the supplied competition rulebook.
Whether a simulated trial actually satisfies a scoring item is supplied as
scenario evidence; an unmodelled item is never awarded automatically.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Optional, Sequence


MISSION_TIME_LIMIT_S = 600.0
FLIGHT_CEILING_M = 4.0
DELIVERY_SCORE_CAP = 72.5
THEORETICAL_MAX_SCORE = 133.5

TARGET_WEIGHTS: Mapping[str, float] = {
    "tent": 1.0,
    "pillbox": 1.5,
    "bridge": 2.0,
    "panzer": 2.5,
    "red_cross": 10.0,
}


@dataclass(frozen=True)
class DeliveryOutcome:
    """One parcel's judged result.

    ``placement`` is one of ``standard_ring``, ``random_inside``,
    ``random_edge``, ``first_impact_only`` or ``outside``. Standard targets
    additionally use a 1-based ring index: 1 is the innermost (5 base points),
    and rings 2..5 receive 4..1 base points.
    """

    target_class: str
    placement: str
    standard_ring_index: Optional[int] = None


@dataclass(frozen=True)
class ScoringScenario:
    carried_parcels: int
    autonomous_takeoff_qualified: Optional[bool]
    obstacle_avoidance_qualified: Optional[bool]
    door_pass_results: tuple[str, ...]
    landing_result: str

    def __post_init__(self) -> None:
        if self.carried_parcels < 0:
            raise ValueError("carried_parcels must be non-negative")
        if any(item not in {"clear", "collision"} for item in self.door_pass_results):
            raise ValueError("door results must be 'clear' or 'collision'")
        if self.landing_result not in {"inside", "edge", "outside", "not_modelled"}:
            raise ValueError("invalid landing_result")


@dataclass(frozen=True)
class ScoreBreakdown:
    carried_score: float
    takeoff_score: float
    obstacle_avoidance_score: float
    delivery_score_raw: float
    delivery_score: float
    door_score: float
    landing_score: float
    total_score: float
    elapsed_time_s: float
    unmodelled_items: tuple[str, ...]
    complete_official_score: bool


def delivery_outcome_score(outcome: DeliveryOutcome) -> float:
    """Return the official points for one judged parcel outcome."""

    if outcome.target_class not in TARGET_WEIGHTS:
        raise ValueError(f"unknown target class: {outcome.target_class}")
    if outcome.placement == "outside":
        return 0.0
    if outcome.placement == "first_impact_only":
        # The rule explicitly says this one point is not multiplied by weight.
        return 1.0
    if outcome.placement == "standard_ring":
        ring = outcome.standard_ring_index
        if ring is None or not 1 <= ring <= 5:
            raise ValueError("standard_ring requires ring index in [1, 5]")
        base_score = float(6 - ring)
    elif outcome.placement == "random_inside":
        base_score = 5.0
    elif outcome.placement == "random_edge":
        base_score = 2.5
    else:
        raise ValueError(f"unknown delivery placement: {outcome.placement}")
    return base_score * TARGET_WEIGHTS[outcome.target_class]


def score_trial(
    scenario: ScoringScenario,
    deliveries: Sequence[DeliveryOutcome],
    elapsed_time_s: float,
) -> ScoreBreakdown:
    """Score a trial while leaving unavailable evidence explicitly unmodelled."""

    if not math.isfinite(elapsed_time_s) or not 0.0 <= elapsed_time_s <= MISSION_TIME_LIMIT_S:
        raise ValueError("elapsed_time_s must be within the official 600 s limit")

    carried = 1.0 if scenario.carried_parcels > 0 else 0.0
    takeoff = 10.0 if scenario.autonomous_takeoff_qualified else 0.0
    avoidance = 10.0 if scenario.obstacle_avoidance_qualified else 0.0
    delivery_raw = sum(delivery_outcome_score(item) for item in deliveries)
    delivery = min(delivery_raw, DELIVERY_SCORE_CAP)
    door = float(min(
        30.0,
        sum(15.0 if item == "clear" else 10.0 for item in scenario.door_pass_results),
    ))
    landing = {"inside": 10.0, "edge": 5.0}.get(scenario.landing_result, 0.0)

    unmodelled = []
    if scenario.autonomous_takeoff_qualified is None:
        unmodelled.append("autonomous_takeoff")
    if scenario.obstacle_avoidance_qualified is None:
        unmodelled.append("obstacle_avoidance")
    if not scenario.door_pass_results:
        unmodelled.append("door_traversal")
    if scenario.landing_result == "not_modelled":
        unmodelled.append("autonomous_landing")

    total = carried + takeoff + avoidance + delivery + door + landing
    return ScoreBreakdown(
        carried_score=carried,
        takeoff_score=takeoff,
        obstacle_avoidance_score=avoidance,
        delivery_score_raw=delivery_raw,
        delivery_score=delivery,
        door_score=door,
        landing_score=landing,
        total_score=total,
        elapsed_time_s=elapsed_time_s,
        unmodelled_items=tuple(unmodelled),
        complete_official_score=not unmodelled,
    )


def competition_rank_key(score: ScoreBreakdown) -> tuple[float, float]:
    """Sort key implementing: higher score first, then shorter time."""

    return (-score.total_score, score.elapsed_time_s)
