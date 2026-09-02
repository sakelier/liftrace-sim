import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from delivery_policy import (  # noqa: E402
    DeliveryDecisionContext,
    DynamicThresholdPolicy,
    FixedPriorityPolicy,
    ReserveRedCrossPolicy,
)


class DeliveryPolicyTest(unittest.TestCase):
    def context(self, target_class="tent", progress=0.1, cargo=1, seen=None):
        classes = {name: 1 for name in ("tent", "pillbox", "bridge", "panzer", "red_cross")}
        seen_counts = {name: 0 for name in classes}
        seen_counts[target_class] = 1
        if seen:
            seen_counts.update(seen)
        probabilities = {name: 0.9 for name in classes}
        return DeliveryDecisionContext(
            target_class=target_class,
            elapsed_time_s=50.0,
            deadline_s=600.0,
            search_progress_fraction=progress,
            cargo_remaining=cargo,
            estimated_service_time_s=20.0,
            class_counts=classes,
            seen_counts=seen_counts,
            verification_probability_by_class=probabilities,
            delivery_probability_by_class=probabilities,
        )

    def test_fixed_priority_uses_only_configured_classes(self):
        policy = FixedPriorityPolicy(frozenset({"red_cross", "panzer", "bridge"}))
        self.assertFalse(policy.decide(self.context("tent", cargo=3)).deliver)
        self.assertTrue(policy.decide(self.context("bridge", cargo=3)).deliver)

    def test_red_cross_reservation_releases_after_red_is_seen(self):
        policy = ReserveRedCrossPolicy(1)
        self.assertFalse(policy.decide(self.context("tent", cargo=1)).deliver)
        context = self.context("tent", cargo=1, seen={"red_cross": 1})
        self.assertTrue(policy.decide(context).deliver)

    def test_dynamic_threshold_reserves_early_and_releases_late(self):
        future = {name: 0.75 for name in (
            "tent", "pillbox", "bridge", "panzer", "red_cross"
        )}
        policy = DynamicThresholdPolicy(future)
        self.assertFalse(policy.decide(self.context("tent", progress=0.1, cargo=1)).deliver)
        self.assertTrue(policy.decide(self.context("tent", progress=1.0, cargo=1)).deliver)

    def test_insufficient_time_rejects_even_high_value_target(self):
        context = self.context("red_cross", cargo=1)
        context = DeliveryDecisionContext(
            **{**context.__dict__, "elapsed_time_s": 590.0, "estimated_service_time_s": 20.0}
        )
        policy = FixedPriorityPolicy(frozenset({"red_cross"}))
        decision = policy.decide(context)
        self.assertFalse(decision.deliver)
        self.assertEqual(decision.reason, "insufficient_time")


if __name__ == "__main__":
    unittest.main()
