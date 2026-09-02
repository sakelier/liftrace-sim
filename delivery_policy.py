#!/usr/bin/env python3
"""Online delivery decisions for a finite-cargo search mission."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Protocol

from scoring import DeliveryOutcome, delivery_outcome_score


@dataclass(frozen=True)
class DeliveryDecisionContext:
    target_class: str
    elapsed_time_s: float
    deadline_s: float
    search_progress_fraction: float
    cargo_remaining: int
    estimated_service_time_s: float
    class_counts: Mapping[str, int]
    seen_counts: Mapping[str, int]
    verification_probability_by_class: Mapping[str, float]
    delivery_probability_by_class: Mapping[str, float]

    def __post_init__(self) -> None:
        if not 0.0 <= self.search_progress_fraction <= 1.0:
            raise ValueError("search_progress_fraction must be in [0, 1]")
        if self.cargo_remaining < 0:
            raise ValueError("cargo_remaining must be non-negative")


@dataclass(frozen=True)
class DeliveryDecision:
    deliver: bool
    policy_name: str
    expected_score: float
    opportunity_threshold: float
    reason: str


class DeliveryPolicy(Protocol):
    name: str

    def decide(self, context: DeliveryDecisionContext) -> DeliveryDecision:
        ...


def full_delivery_score(target_class: str) -> float:
    outcome = (
        DeliveryOutcome(target_class, "random_inside")
        if target_class == "red_cross"
        else DeliveryOutcome(target_class, "standard_ring", 1)
    )
    return delivery_outcome_score(outcome)


def expected_delivery_score(context: DeliveryDecisionContext, target_class: str) -> float:
    return (
        full_delivery_score(target_class)
        * context.verification_probability_by_class[target_class]
        * context.delivery_probability_by_class[target_class]
    )


def _terminal_decision(
    context: DeliveryDecisionContext,
    policy_name: str,
) -> DeliveryDecision | None:
    expected = expected_delivery_score(context, context.target_class)
    if context.cargo_remaining == 0:
        return DeliveryDecision(False, policy_name, expected, math.inf, "no_cargo")
    if context.elapsed_time_s + context.estimated_service_time_s > context.deadline_s:
        return DeliveryDecision(False, policy_name, expected, math.inf, "insufficient_time")
    return None


@dataclass(frozen=True)
class FirstSeenPolicy:
    name: str = "first_seen"

    def decide(self, context: DeliveryDecisionContext) -> DeliveryDecision:
        terminal = _terminal_decision(context, self.name)
        if terminal is not None:
            return terminal
        expected = expected_delivery_score(context, context.target_class)
        return DeliveryDecision(True, self.name, expected, 0.0, "first_eligible_cue")


@dataclass(frozen=True)
class FixedPriorityPolicy:
    eligible_classes: frozenset[str]
    name: str = "fixed_priority"

    def decide(self, context: DeliveryDecisionContext) -> DeliveryDecision:
        terminal = _terminal_decision(context, self.name)
        if terminal is not None:
            return terminal
        expected = expected_delivery_score(context, context.target_class)
        deliver = context.target_class in self.eligible_classes
        return DeliveryDecision(
            deliver,
            self.name,
            expected,
            0.0,
            "priority_class" if deliver else "below_fixed_priority",
        )


@dataclass(frozen=True)
class ReserveRedCrossPolicy:
    reserved_slots: int = 1
    name: str = "reserve_red_cross"

    def decide(self, context: DeliveryDecisionContext) -> DeliveryDecision:
        terminal = _terminal_decision(context, self.name)
        if terminal is not None:
            return terminal
        expected = expected_delivery_score(context, context.target_class)
        if context.target_class == "red_cross":
            return DeliveryDecision(True, self.name, expected, 0.0, "red_cross_priority")
        red_remaining = max(
            0,
            context.class_counts.get("red_cross", 0)
            - context.seen_counts.get("red_cross", 0),
        )
        reserve = min(self.reserved_slots, red_remaining)
        deliver = context.cargo_remaining > reserve
        return DeliveryDecision(
            deliver,
            self.name,
            expected,
            0.0,
            "cargo_above_reserve" if deliver else "reserved_for_red_cross",
        )


@dataclass(frozen=True)
class DynamicThresholdPolicy:
    future_cue_probability_by_class: Mapping[str, float]
    remaining_progress_exponent: float = 1.0
    name: str = "dynamic_threshold"

    def __post_init__(self) -> None:
        if self.remaining_progress_exponent <= 0.0:
            raise ValueError("remaining_progress_exponent must be positive")
        if any(not 0.0 <= value <= 1.0 for value in self.future_cue_probability_by_class.values()):
            raise ValueError("future cue probabilities must be in [0, 1]")

    def decide(self, context: DeliveryDecisionContext) -> DeliveryDecision:
        terminal = _terminal_decision(context, self.name)
        if terminal is not None:
            return terminal

        remaining_fraction = (1.0 - context.search_progress_fraction) ** self.remaining_progress_exponent
        future_values: list[float] = []
        for class_name, total_count in context.class_counts.items():
            unseen_count = max(0, total_count - context.seen_counts.get(class_name, 0))
            future_probability = (
                self.future_cue_probability_by_class.get(class_name, 0.0)
                * remaining_fraction
            )
            future_values.extend(
                expected_delivery_score(context, class_name) * future_probability
                for _ in range(unseen_count)
            )

        future_values.sort(reverse=True)
        threshold = (
            future_values[context.cargo_remaining - 1]
            if len(future_values) >= context.cargo_remaining
            else 0.0
        )
        current = expected_delivery_score(context, context.target_class)
        deliver = current + 1.0e-12 >= threshold
        return DeliveryDecision(
            deliver,
            self.name,
            current,
            threshold,
            "meets_dynamic_threshold" if deliver else "reserved_for_higher_expected_value",
        )


def load_delivery_policies(config: Mapping[str, object]) -> dict[str, DeliveryPolicy]:
    item = config["delivery_decision"]
    fixed = item["fixed_priority"]
    reserve = item["reserve_red_cross"]
    dynamic = item["dynamic_threshold"]
    policies: dict[str, DeliveryPolicy] = {
        "first_seen": FirstSeenPolicy(),
        "fixed_priority": FixedPriorityPolicy(frozenset(fixed["eligible_classes"])),
        "reserve_red_cross": ReserveRedCrossPolicy(int(reserve["reserved_slots"])),
        "dynamic_threshold": DynamicThresholdPolicy(
            {
                name: float(value)
                for name, value in dynamic["future_cue_probability_by_class"].items()
            },
            float(dynamic["remaining_progress_exponent"]),
        ),
    }
    requested = tuple(item["compare_policies"])
    if not requested:
        raise ValueError("compare_policies must not be empty")
    unknown = set(requested) - set(policies)
    if unknown:
        raise ValueError(f"unknown delivery policies: {sorted(unknown)}")
    return {name: policies[name] for name in requested}
