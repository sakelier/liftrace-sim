"""Frame-sampled, obstacle-aware visibility for downward-camera observations."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Mapping, Sequence

from obstacles import Obstacle, line_of_sight_blockers
from search_sim import Point3, SegmentObservation, Target


@dataclass(frozen=True)
class OcclusionConfig:
    sample_rate_hz: float
    minimum_visible_fraction: float
    minimum_consecutive_visible_sec: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.sample_rate_hz) or self.sample_rate_hz <= 0.0:
            raise ValueError("occlusion sample rate must be positive")
        if not 0.0 <= self.minimum_visible_fraction <= 1.0:
            raise ValueError("minimum visible fraction must be in [0, 1]")
        if self.minimum_consecutive_visible_sec < 0.0:
            raise ValueError("minimum consecutive visibility must be non-negative")


def load_occlusion_config(config: Mapping[str, object]) -> OcclusionConfig | None:
    section = config.get("occlusion", {})
    if not section or not bool(section.get("enabled", False)):
        return None
    return OcclusionConfig(
        sample_rate_hz=float(section["sample_rate_hz"]),
        minimum_visible_fraction=float(section["minimum_visible_fraction"]),
        minimum_consecutive_visible_sec=float(section["minimum_consecutive_visible_sec"]),
    )


def _target_samples(target: Target) -> tuple[Point3, ...]:
    hx, hy = target.size_x / 2.0, target.size_y / 2.0
    center = target.position
    if hx <= 0.0 or hy <= 0.0:
        return (center,)
    return (
        center,
        Point3(center.x - hx, center.y - hy, center.z),
        Point3(center.x - hx, center.y + hy, center.z),
        Point3(center.x + hx, center.y - hy, center.z),
        Point3(center.x + hx, center.y + hy, center.z),
    )


def evaluate_observation_occlusion(
    observation: SegmentObservation,
    segment_start: Point3,
    segment_end: Point3,
    target: Target,
    obstacles: Sequence[Obstacle],
    config: OcclusionConfig,
) -> SegmentObservation:
    """Annotate one geometrically valid pass with sampled 3-D visibility."""
    dx, dy = segment_end.x - segment_start.x, segment_end.y - segment_start.y
    length = math.hypot(dx, dy)
    if length <= 0.0:
        raise ValueError("occlusion evaluation requires a positive-length segment")
    ux, uy = dx / length, dy / length
    duration = observation.dwell_time_s
    frame_count = max(1, math.ceil(duration * config.sample_rate_hz))
    step_time = duration / frame_count
    target_samples = _target_samples(target)
    visible_frames = 0
    streak = max_streak = 0
    fraction_sum = 0.0
    blocker_ids: set[str] = set()
    first_visible_offset = None

    for frame_index in range(frame_count):
        fraction = (frame_index + 0.5) / frame_count
        distance = observation.entry_distance_m + fraction * observation.visible_path_length_m
        camera = Point3(
            segment_start.x + ux * distance,
            segment_start.y + uy * distance,
            segment_start.z + (segment_end.z - segment_start.z) * distance / length,
        )
        visible_points = 0
        for target_point in target_samples:
            blockers = line_of_sight_blockers(camera, target_point, obstacles)
            blocker_ids.update(blockers)
            visible_points += not blockers
        visible_fraction = visible_points / len(target_samples)
        fraction_sum += visible_fraction
        if visible_fraction >= config.minimum_visible_fraction:
            visible_frames += 1
            streak += 1
            max_streak = max(max_streak, streak)
            if first_visible_offset is None:
                first_visible_offset = frame_index * step_time
        else:
            streak = 0

    effective_time = visible_frames * step_time
    max_consecutive_time = max_streak * step_time
    return replace(
        observation,
        occlusion_evaluated=True,
        occlusion_passed=(
            max_consecutive_time + 1.0e-9
            >= config.minimum_consecutive_visible_sec
        ),
        effective_visible_time_s=effective_time,
        mean_visible_fraction=fraction_sum / frame_count,
        visible_frame_count=visible_frames,
        sampled_frame_count=frame_count,
        max_consecutive_visible_time_s=max_consecutive_time,
        first_effective_offset_s=first_visible_offset,
        occluding_obstacle_ids=tuple(sorted(blocker_ids)),
    )


def evaluate_occlusions(
    waypoints: Sequence[Point3],
    targets: Sequence[Target],
    observations: Sequence[SegmentObservation],
    obstacles: Sequence[Obstacle],
    config: OcclusionConfig,
) -> list[SegmentObservation]:
    target_by_id = {target.target_id: target for target in targets}
    return [
        evaluate_observation_occlusion(
            observation,
            waypoints[observation.segment_index],
            waypoints[observation.segment_index + 1],
            target_by_id[observation.target_id],
            obstacles,
            config,
        )
        for observation in observations
    ]
