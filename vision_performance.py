"""Empirical V-SIM-04 lookup model with explicit unmeasured-data policy."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


@dataclass(frozen=True)
class VisionCondition:
    class_name: str
    height_m: float
    speed_mps: float
    successes: int
    trials: int

    @property
    def probability(self) -> float:
        return self.successes / self.trials


@dataclass(frozen=True)
class EmpiricalVisionModel:
    conditions: tuple[VisionCondition, ...]
    unmeasured_policy: str = "legacy"
    height_scale_m: float = 1.2
    speed_scale_mps: float = 0.5

    def __post_init__(self) -> None:
        if not self.conditions:
            raise ValueError("empirical vision table must contain dynamic conditions")
        if self.unmeasured_policy not in {"legacy", "nearest", "error"}:
            raise ValueError("unmeasured_policy must be legacy, nearest or error")
        if self.height_scale_m <= 0.0 or self.speed_scale_mps <= 0.0:
            raise ValueError("empirical distance scales must be positive")

    def probability(self, class_name: str, height_m: float, speed_mps: float) -> Optional[float]:
        candidates = [item for item in self.conditions if item.class_name == class_name]
        exact = [item for item in candidates if math.isclose(item.height_m, height_m, abs_tol=1e-6)
                 and math.isclose(item.speed_mps, speed_mps, abs_tol=1e-6)]
        if exact:
            return exact[0].probability
        if self.unmeasured_policy == "legacy":
            return None
        if self.unmeasured_policy == "error":
            raise ValueError(f"unmeasured vision condition: {class_name}, h={height_m}, v={speed_mps}")
        if not candidates:
            return None
        nearest = min(candidates, key=lambda item: (
            ((item.height_m - height_m) / self.height_scale_m) ** 2
            + ((item.speed_mps - speed_mps) / self.speed_scale_mps) ** 2,
            -item.trials,
            item.height_m,
            item.speed_mps,
        ))
        return nearest.probability


def load_empirical_vision_model(
    path: Path,
    unmeasured_policy: str = "legacy",
    height_scale_m: float = 1.2,
    speed_scale_mps: float = 0.5,
) -> EmpiricalVisionModel:
    conditions = []
    with path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            if row["kind"] != "dynamic" or not row["speed_mps"]:
                continue
            conditions.append(VisionCondition(
                class_name=row["class_name"],
                height_m=float(row["height_m"]),
                speed_mps=float(row["speed_mps"]),
                successes=int(row["successes"]),
                trials=int(row["trials"]),
            ))
    return EmpiricalVisionModel(
        tuple(conditions), unmeasured_policy, height_scale_m, speed_scale_mps
    )
