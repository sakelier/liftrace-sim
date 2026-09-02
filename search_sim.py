#!/usr/bin/env python3
"""M0/M1 task-level geometry and probabilistic Cue simulation."""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, List, Mapping, Optional, Sequence, Tuple

import yaml

from vision_performance import EmpiricalVisionModel, load_empirical_vision_model


EPSILON = 1.0e-9


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class SearchArea:
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def __post_init__(self) -> None:
        values = (self.min_x, self.max_x, self.min_y, self.max_y)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("search-area bounds must be finite")
        if self.min_x >= self.max_x:
            raise ValueError("min_x must be smaller than max_x")
        if self.min_y >= self.max_y:
            raise ValueError("min_y must be smaller than max_y")

    @property
    def width_x(self) -> float:
        return self.max_x - self.min_x

    @property
    def width_y(self) -> float:
        return self.max_y - self.min_y

    @property
    def area(self) -> float:
        return self.width_x * self.width_y


@dataclass(frozen=True)
class M0Parameters:
    area: SearchArea
    lane_direction: str
    lane_spacing: float
    altitude: float
    speed: float
    turn_penalty: float

    def __post_init__(self) -> None:
        if self.lane_direction not in ("x", "y"):
            raise ValueError("lane_direction must be 'x' or 'y'")
        positive_values = (self.lane_spacing, self.altitude, self.speed)
        if not all(math.isfinite(value) and value > 0.0 for value in positive_values):
            raise ValueError("lane spacing, altitude and speed must be finite and positive")
        if not math.isfinite(self.turn_penalty) or self.turn_penalty < 0.0:
            raise ValueError("turn penalty must be finite and non-negative")


@dataclass(frozen=True)
class M0Metrics:
    area_m2: float
    lane_count: int
    waypoint_count: int
    lane_segment_count: int
    connector_segment_count: int
    lane_distance_m: float
    connector_distance_m: float
    route_distance_m: float
    turn_count: int
    flight_time_s: float
    turn_time_s: float
    total_time_s: float


@dataclass(frozen=True)
class CameraModel:
    horizontal_fov: float
    image_width: int
    image_height: int
    update_rate: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.horizontal_fov) or not 0.0 < self.horizontal_fov < math.pi:
            raise ValueError("horizontal FOV must be finite and between 0 and pi")
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError("camera image dimensions must be positive")
        if not math.isfinite(self.update_rate) or self.update_rate <= 0.0:
            raise ValueError("camera update rate must be finite and positive")

    @property
    def vertical_fov(self) -> float:
        return 2.0 * math.atan(
            math.tan(self.horizontal_fov / 2.0)
            * self.image_height / self.image_width
        )

    def footprint(self, height_above_target: float) -> Tuple[float, float]:
        """Return cross-track width and along-track length on a horizontal plane."""
        if not math.isfinite(height_above_target) or height_above_target <= 0.0:
            raise ValueError("camera must be above the target plane")
        cross_track = 2.0 * height_above_target * math.tan(self.horizontal_fov / 2.0)
        along_track = 2.0 * height_above_target * math.tan(self.vertical_fov / 2.0)
        return cross_track, along_track


@dataclass(frozen=True)
class Target:
    target_id: str
    class_name: str
    position: Point3
    size_x: float = 0.0
    size_y: float = 0.0

    def __post_init__(self) -> None:
        if not self.target_id:
            raise ValueError("target_id must not be empty")
        if not self.class_name:
            raise ValueError("class_name must not be empty")
        values = (
            self.position.x, self.position.y, self.position.z,
            self.size_x, self.size_y,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("target geometry must be finite")
        if self.size_x < 0.0 or self.size_y < 0.0:
            raise ValueError("target dimensions must be non-negative")


@dataclass(frozen=True)
class SegmentObservation:
    target_id: str
    class_name: str
    segment_index: int
    segment_kind: str
    visibility_mode: str
    cross_track_distance_m: float
    along_track_coordinate_m: float
    visible_path_length_m: float
    dwell_time_s: float
    entry_distance_m: float
    exit_distance_m: float
    entry_point: Point3
    exit_point: Point3
    footprint_cross_track_m: float
    footprint_along_track_m: float
    height_above_target_m: float = math.nan
    occlusion_evaluated: bool = False
    occlusion_passed: bool = True
    effective_visible_time_s: Optional[float] = None
    mean_visible_fraction: float = 1.0
    visible_frame_count: int = 0
    sampled_frame_count: int = 0
    max_consecutive_visible_time_s: float = 0.0
    first_effective_offset_s: Optional[float] = None
    occluding_obstacle_ids: Tuple[str, ...] = ()


@dataclass(frozen=True)
class TargetGenerationConfig:
    class_counts: Mapping[str, int]
    class_sizes: Mapping[str, Tuple[float, float]]
    minimum_separation: float
    maximum_attempts: int
    target_z: float = 0.0

    def __post_init__(self) -> None:
        if not self.class_counts or any(count < 0 for count in self.class_counts.values()):
            raise ValueError("target class counts must be non-negative")
        if any(name not in self.class_sizes for name in self.class_counts):
            raise ValueError("every generated class must have a configured size")
        if self.minimum_separation < 0.0 or not math.isfinite(self.minimum_separation):
            raise ValueError("minimum separation must be finite and non-negative")
        if self.maximum_attempts <= 0:
            raise ValueError("maximum attempts must be positive")


@dataclass(frozen=True)
class CueModel:
    base_probability_by_class: Mapping[str, float]
    dwell_time_constant: float
    cross_track_decay: float
    probability_override: Optional[float] = None
    empirical_model: Optional[EmpiricalVisionModel] = None

    def __post_init__(self) -> None:
        probabilities = list(self.base_probability_by_class.values())
        if any(not math.isfinite(value) or not 0.0 <= value <= 1.0 for value in probabilities):
            raise ValueError("base Cue probabilities must be in [0, 1]")
        if not math.isfinite(self.dwell_time_constant) or self.dwell_time_constant <= 0.0:
            raise ValueError("dwell time constant must be finite and positive")
        if not math.isfinite(self.cross_track_decay) or self.cross_track_decay < 0.0:
            raise ValueError("cross-track decay must be finite and non-negative")
        if self.probability_override is not None and not 0.0 <= self.probability_override <= 1.0:
            raise ValueError("Cue probability override must be in [0, 1]")


@dataclass(frozen=True)
class TargetCueResult:
    target_id: str
    class_name: str
    geometrically_visible: bool
    cued: bool
    first_cue_time_s: Optional[float]
    attempted_passes: int


@dataclass(frozen=True)
class MonteCarloSummary:
    trials: int
    seed: int
    mean_cue_rate: float
    mean_targets_cued: float
    probability_all_targets_cued: float
    mean_first_cue_time_s: Optional[float]
    class_cue_rates: Mapping[str, float]
    occlusion_evaluated_passes: int = 0
    occlusion_pass_rate: Optional[float] = None
    mean_visible_fraction: Optional[float] = None


def _inclusive_positions(lower: float, upper: float, spacing: float) -> List[float]:
    """Return regularly spaced positions and always include the upper boundary."""
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError("spacing must be finite and positive")
    if lower >= upper:
        raise ValueError("lower must be smaller than upper")

    positions = [float(lower)]
    while positions[-1] + spacing < upper - EPSILON:
        positions.append(positions[-1] + spacing)
    if upper - positions[-1] > EPSILON:
        positions.append(float(upper))
    return positions


def generate_boustrophedon(params: M0Parameters) -> List[Point3]:
    """Generate the same alternating boundary-to-boundary pattern as uav_mission."""
    area = params.area
    waypoints: List[Point3] = []

    if params.lane_direction == "x":
        offsets = _inclusive_positions(area.min_y, area.max_y, params.lane_spacing)
        for index, y in enumerate(offsets):
            start_x, end_x = (
                (area.min_x, area.max_x)
                if index % 2 == 0
                else (area.max_x, area.min_x)
            )
            waypoints.extend((
                Point3(start_x, y, params.altitude),
                Point3(end_x, y, params.altitude),
            ))
    else:
        offsets = _inclusive_positions(area.min_x, area.max_x, params.lane_spacing)
        for index, x in enumerate(offsets):
            start_y, end_y = (
                (area.min_y, area.max_y)
                if index % 2 == 0
                else (area.max_y, area.min_y)
            )
            waypoints.extend((
                Point3(x, start_y, params.altitude),
                Point3(x, end_y, params.altitude),
            ))

    return waypoints


def _distance(left: Point3, right: Point3) -> float:
    return math.sqrt(
        (right.x - left.x) ** 2
        + (right.y - left.y) ** 2
        + (right.z - left.z) ** 2
    )


def _is_turn(before: Point3, current: Point3, after: Point3) -> bool:
    first = (current.x - before.x, current.y - before.y, current.z - before.z)
    second = (after.x - current.x, after.y - current.y, after.z - current.z)
    first_norm = math.sqrt(sum(value * value for value in first))
    second_norm = math.sqrt(sum(value * value for value in second))
    if first_norm <= EPSILON or second_norm <= EPSILON:
        return False
    cosine = sum(a * b for a, b in zip(first, second)) / (first_norm * second_norm)
    return abs(abs(cosine) - 1.0) > 1.0e-8


def evaluate_m0(params: M0Parameters, waypoints: Sequence[Point3]) -> M0Metrics:
    if len(waypoints) < 2:
        raise ValueError("at least two waypoints are required")

    segment_lengths = [
        _distance(waypoints[index], waypoints[index + 1])
        for index in range(len(waypoints) - 1)
    ]
    lane_lengths = segment_lengths[0::2]
    connector_lengths = segment_lengths[1::2]
    turn_count = sum(
        _is_turn(waypoints[index - 1], waypoints[index], waypoints[index + 1])
        for index in range(1, len(waypoints) - 1)
    )

    lane_distance = sum(lane_lengths)
    connector_distance = sum(connector_lengths)
    route_distance = lane_distance + connector_distance
    flight_time = route_distance / params.speed
    turn_time = turn_count * params.turn_penalty

    return M0Metrics(
        area_m2=params.area.area,
        lane_count=len(lane_lengths),
        waypoint_count=len(waypoints),
        lane_segment_count=len(lane_lengths),
        connector_segment_count=len(connector_lengths),
        lane_distance_m=lane_distance,
        connector_distance_m=connector_distance,
        route_distance_m=route_distance,
        turn_count=turn_count,
        flight_time_s=flight_time,
        turn_time_s=turn_time,
        total_time_s=flight_time + turn_time,
    )


def observe_target_on_segment(
        start: Point3,
        end: Point3,
        target: Target,
        camera: CameraModel,
        speed: float,
        segment_index: int = 0,
        require_fully_in_frame: bool = True,
) -> Optional[SegmentObservation]:
    """Calculate the interval where a target is inside a heading-aligned footprint.

    The M1a model assumes a horizontal segment, downward camera and vehicle yaw
    aligned with segment heading. Image width is cross-track; image height is
    along-track. Target dimensions are axis-aligned in the world frame.
    """
    if not math.isfinite(speed) or speed <= 0.0:
        raise ValueError("speed must be finite and positive")
    if abs(start.z - end.z) > EPSILON:
        raise ValueError("M1a supports only constant-altitude segments")

    dx = end.x - start.x
    dy = end.y - start.y
    segment_length = math.hypot(dx, dy)
    if segment_length <= EPSILON:
        raise ValueError("segment must have positive horizontal length")

    height_above_target = start.z - target.position.z
    footprint_cross, footprint_along = camera.footprint(height_above_target)
    along_x, along_y = dx / segment_length, dy / segment_length
    cross_x, cross_y = -along_y, along_x
    relative_x = target.position.x - start.x
    relative_y = target.position.y - start.y
    target_along = relative_x * along_x + relative_y * along_y
    target_cross = relative_x * cross_x + relative_y * cross_y

    target_half_along = 0.0
    target_half_cross = 0.0
    if require_fully_in_frame:
        target_half_along = (
            abs(along_x) * target.size_x / 2.0
            + abs(along_y) * target.size_y / 2.0
        )
        target_half_cross = (
            abs(cross_x) * target.size_x / 2.0
            + abs(cross_y) * target.size_y / 2.0
        )

    along_limit = footprint_along / 2.0 - target_half_along
    cross_limit = footprint_cross / 2.0 - target_half_cross
    if along_limit < -EPSILON or cross_limit < -EPSILON:
        return None
    along_limit = max(along_limit, 0.0)
    cross_limit = max(cross_limit, 0.0)
    if abs(target_cross) > cross_limit + EPSILON:
        return None

    entry_distance = max(0.0, target_along - along_limit)
    exit_distance = min(segment_length, target_along + along_limit)
    if exit_distance < entry_distance - EPSILON:
        return None
    visible_length = max(0.0, exit_distance - entry_distance)

    def point_at(distance: float) -> Point3:
        return Point3(
            start.x + along_x * distance,
            start.y + along_y * distance,
            start.z,
        )

    return SegmentObservation(
        target_id=target.target_id,
        class_name=target.class_name,
        segment_index=segment_index,
        segment_kind="lane" if segment_index % 2 == 0 else "connector",
        visibility_mode="full_target" if require_fully_in_frame else "center",
        cross_track_distance_m=abs(target_cross),
        along_track_coordinate_m=target_along,
        visible_path_length_m=visible_length,
        dwell_time_s=visible_length / speed,
        entry_distance_m=entry_distance,
        exit_distance_m=exit_distance,
        entry_point=point_at(entry_distance),
        exit_point=point_at(exit_distance),
        footprint_cross_track_m=footprint_cross,
        footprint_along_track_m=footprint_along,
        height_above_target_m=height_above_target,
    )


def evaluate_target_visibility(
        waypoints: Sequence[Point3],
        targets: Sequence[Target],
        camera: CameraModel,
        speed: float,
        require_fully_in_frame: bool = True,
) -> List[SegmentObservation]:
    observations: List[SegmentObservation] = []
    for segment_index in range(len(waypoints) - 1):
        start, end = waypoints[segment_index], waypoints[segment_index + 1]
        for target in targets:
            observation = observe_target_on_segment(
                start=start,
                end=end,
                target=target,
                camera=camera,
                speed=speed,
                segment_index=segment_index,
                require_fully_in_frame=require_fully_in_frame,
            )
            if observation is not None and observation.visible_path_length_m > EPSILON:
                observations.append(observation)
    return observations


def generate_random_targets(
        area: SearchArea,
        generation: TargetGenerationConfig,
        rng: random.Random,
        forbidden_regions: Sequence[Tuple[float, float, float, float]] = (),
) -> List[Target]:
    """Generate targets using deterministic rejection sampling.

    Rectangles remain inside the search area. Centre separation is an explicit
    M1b assumption, not a competition rule.
    """
    targets: List[Target] = []
    for class_name, count in generation.class_counts.items():
        size_x, size_y = generation.class_sizes[class_name]
        if size_x <= 0.0 or size_y <= 0.0:
            raise ValueError("generated target sizes must be positive")
        min_x, max_x = area.min_x + size_x / 2.0, area.max_x - size_x / 2.0
        min_y, max_y = area.min_y + size_y / 2.0, area.max_y - size_y / 2.0
        if min_x > max_x or min_y > max_y:
            raise ValueError(f"target class {class_name} does not fit in search area")
        for class_index in range(count):
            for _ in range(generation.maximum_attempts):
                x, y = rng.uniform(min_x, max_x), rng.uniform(min_y, max_y)
                target_bounds = (
                    x - size_x / 2.0, x + size_x / 2.0,
                    y - size_y / 2.0, y + size_y / 2.0,
                )
                overlaps_forbidden = any(
                    target_bounds[0] <= region[1] + EPSILON
                    and target_bounds[1] >= region[0] - EPSILON
                    and target_bounds[2] <= region[3] + EPSILON
                    and target_bounds[3] >= region[2] - EPSILON
                    for region in forbidden_regions
                )
                if overlaps_forbidden:
                    continue
                if all(math.hypot(x - item.position.x, y - item.position.y)
                       >= generation.minimum_separation - EPSILON for item in targets):
                    targets.append(Target(
                        target_id=f"{class_name}_{class_index + 1}",
                        class_name=class_name,
                        position=Point3(x, y, generation.target_z),
                        size_x=size_x,
                        size_y=size_y,
                    ))
                    break
            else:
                raise RuntimeError(
                    f"could not place {class_name}_{class_index + 1} after "
                    f"{generation.maximum_attempts} attempts")
    return targets


def cue_probability(observation: SegmentObservation, model: CueModel) -> float:
    """Return the Cue probability for one visible pass.

    Exact empirical V-SIM-04 conditions use measured ``p_selected`` directly.
    Unmeasured conditions follow the empirical model's configured policy; the
    legacy dwell/cross-track formula remains the explicit fallback.
    """
    if model.probability_override is not None:
        return model.probability_override
    if observation.occlusion_evaluated and not observation.occlusion_passed:
        return 0.0
    if model.empirical_model is not None and math.isfinite(observation.height_above_target_m):
        speed = observation.visible_path_length_m / max(observation.dwell_time_s, EPSILON)
        empirical = model.empirical_model.probability(
            observation.class_name, observation.height_above_target_m, speed
        )
        if empirical is not None:
            return empirical
    base = model.base_probability_by_class.get(observation.class_name, 0.0)
    dwell_time = (
        observation.dwell_time_s
        if observation.effective_visible_time_s is None
        else observation.effective_visible_time_s
    )
    dwell_factor = 1.0 - math.exp(-dwell_time / model.dwell_time_constant)
    half_width = observation.footprint_cross_track_m / 2.0
    normalized_cross = observation.cross_track_distance_m / max(half_width, EPSILON)
    cross_factor = math.exp(-model.cross_track_decay * normalized_cross ** 2)
    return min(1.0, max(0.0, base * dwell_factor * cross_factor))


def segment_start_times(
        waypoints: Sequence[Point3], speed: float, turn_penalty: float,
) -> List[float]:
    """Return nominal start times, including every prior heading-change penalty."""
    if speed <= 0.0 or turn_penalty < 0.0:
        raise ValueError("invalid timing parameters")
    starts: List[float] = []
    elapsed = 0.0
    for index in range(len(waypoints) - 1):
        if index > 0 and _is_turn(
                waypoints[index - 1], waypoints[index], waypoints[index + 1]):
            elapsed += turn_penalty
        starts.append(elapsed)
        elapsed += _distance(waypoints[index], waypoints[index + 1]) / speed
    return starts


def simulate_cues(
        waypoints: Sequence[Point3],
        targets: Sequence[Target],
        observations: Sequence[SegmentObservation],
        model: CueModel,
        speed: float,
        turn_penalty: float,
        rng: random.Random,
) -> List[TargetCueResult]:
    starts = segment_start_times(waypoints, speed, turn_penalty)
    results: List[TargetCueResult] = []
    for target in targets:
        passes = sorted(
            (item for item in observations if item.target_id == target.target_id),
            key=lambda item: (item.segment_index, item.entry_distance_m),
        )
        first_cue_time: Optional[float] = None
        attempts = 0
        for item in passes:
            attempts += 1
            if rng.random() < cue_probability(item, model):
                first_cue_time = (
                    starts[item.segment_index]
                    + item.entry_distance_m / speed
                    + (item.first_effective_offset_s or 0.0)
                )
                break
        results.append(TargetCueResult(
            target_id=target.target_id,
            class_name=target.class_name,
            geometrically_visible=bool(passes),
            cued=first_cue_time is not None,
            first_cue_time_s=first_cue_time,
            attempted_passes=attempts,
        ))
    return results


def run_monte_carlo(
        params: M0Parameters,
        waypoints: Sequence[Point3],
        camera: CameraModel,
        generation: TargetGenerationConfig,
        cue_model: CueModel,
        trials: int,
        seed: int,
    require_fully_in_frame: bool = True,
    obstacles: Sequence[object] = (),
    occlusion_config: Optional[object] = None,
) -> MonteCarloSummary:
    if trials <= 0:
        raise ValueError("Monte Carlo trial count must be positive")
    total_targets = sum(generation.class_counts.values())
    if total_targets <= 0:
        raise ValueError("Monte Carlo requires at least one target")
    cued_counts: List[int] = []
    all_cued_count = 0
    first_times: List[float] = []
    class_totals = {name: 0 for name in generation.class_counts}
    class_cued = {name: 0 for name in generation.class_counts}
    occlusion_evaluated_passes = 0
    occlusion_passed_passes = 0
    visibility_fraction_sum = 0.0

    # Separate deterministic streams make every trial's target layout
    # independent of how many Cue draws a strategy consumes. Reusing the same
    # seed across parameter sets therefore gives a fair common scenario set.
    for trial_index in range(trials):
        placement_rng = random.Random(seed + 2 * trial_index)
        cue_rng = random.Random(seed + 2 * trial_index + 1)
        forbidden_regions = tuple(
            obstacle.bounds() for obstacle in obstacles if hasattr(obstacle, "bounds")
        )
        targets = generate_random_targets(
            params.area, generation, placement_rng, forbidden_regions
        )
        observations = evaluate_target_visibility(
            waypoints, targets, camera, params.speed, require_fully_in_frame)
        if obstacles and occlusion_config is not None:
            from occlusion import evaluate_occlusions
            observations = evaluate_occlusions(
                waypoints, targets, observations, obstacles, occlusion_config
            )
            occlusion_evaluated_passes += len(observations)
            occlusion_passed_passes += sum(item.occlusion_passed for item in observations)
            visibility_fraction_sum += sum(item.mean_visible_fraction for item in observations)
        results = simulate_cues(
            waypoints, targets, observations, cue_model,
            params.speed, params.turn_penalty, cue_rng)
        number_cued = sum(item.cued for item in results)
        cued_counts.append(number_cued)
        all_cued_count += number_cued == total_targets
        trial_first_times = [item.first_cue_time_s for item in results if item.cued]
        if trial_first_times:
            first_times.append(min(trial_first_times))
        for item in results:
            class_totals[item.class_name] += 1
            class_cued[item.class_name] += item.cued

    return MonteCarloSummary(
        trials=trials,
        seed=seed,
        mean_cue_rate=sum(cued_counts) / (trials * total_targets),
        mean_targets_cued=statistics.fmean(cued_counts),
        probability_all_targets_cued=all_cued_count / trials,
        mean_first_cue_time_s=(statistics.fmean(first_times) if first_times else None),
        class_cue_rates={
            name: class_cued[name] / class_totals[name]
            for name in generation.class_counts
        },
        occlusion_evaluated_passes=occlusion_evaluated_passes,
        occlusion_pass_rate=(
            occlusion_passed_passes / occlusion_evaluated_passes
            if occlusion_evaluated_passes else None
        ),
        mean_visible_fraction=(
            visibility_fraction_sum / occlusion_evaluated_passes
            if occlusion_evaluated_passes else None
        ),
    )


def load_parameters(path: Path) -> Tuple[M0Parameters, Mapping[str, object]]:
    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("configuration root must be a mapping")

    area_config = config["search_area"]
    coverage_config = config["coverage"]
    motion_config = config["motion"]
    params = M0Parameters(
        area=SearchArea(
            min_x=float(area_config["min_x_m"]),
            max_x=float(area_config["max_x_m"]),
            min_y=float(area_config["min_y_m"]),
            max_y=float(area_config["max_y_m"]),
        ),
        lane_direction=str(coverage_config["lane_direction"]),
        lane_spacing=float(coverage_config["lane_spacing_m"]),
        altitude=float(coverage_config["altitude_m"]),
        speed=float(motion_config["speed_mps"]),
        turn_penalty=float(motion_config["turn_penalty_s"]),
    )
    return params, config


def load_camera(config: Mapping[str, object]) -> CameraModel:
    camera_config = config["camera"]
    return CameraModel(
        horizontal_fov=float(camera_config["horizontal_fov_rad"]),
        image_width=int(camera_config["image_width_px"]),
        image_height=int(camera_config["image_height_px"]),
        update_rate=float(camera_config["update_rate_hz"]),
    )


def load_demo_targets(config: Mapping[str, object]) -> List[Target]:
    targets = []
    for item in config.get("m1a_demo_targets", []):
        targets.append(Target(
            target_id=str(item["id"]),
            class_name=str(item["class_name"]),
            position=Point3(
                float(item["x_m"]),
                float(item["y_m"]),
                float(item.get("z_m", 0.0)),
            ),
            size_x=float(item.get("size_x_m", 0.0)),
            size_y=float(item.get("size_y_m", 0.0)),
        ))
    return targets


def load_m1b(config: Mapping[str, object]) -> Tuple[TargetGenerationConfig, CueModel, int, int]:
    m1b = config["m1b"]
    generation_config = m1b["target_generation"]
    generation = TargetGenerationConfig(
        class_counts={
            name: int(item["count"])
            for name, item in generation_config["classes"].items()
        },
        class_sizes={
            name: (float(item["size_x_m"]), float(item["size_y_m"]))
            for name, item in generation_config["classes"].items()
        },
        minimum_separation=float(generation_config["minimum_center_separation_m"]),
        maximum_attempts=int(generation_config["maximum_placement_attempts"]),
        target_z=float(generation_config.get("target_z_m", 0.0)),
    )
    cue_config = m1b["cue_model"]
    empirical_model = None
    empirical_config = cue_config.get("empirical")
    if empirical_config and bool(empirical_config.get("enabled", False)):
        table_path = Path(__file__).resolve().parent / str(empirical_config["table"])
        empirical_model = load_empirical_vision_model(
            table_path,
            unmeasured_policy=str(empirical_config.get("unmeasured_policy", "legacy")),
            height_scale_m=float(empirical_config.get("height_scale_m", 1.2)),
            speed_scale_mps=float(empirical_config.get("speed_scale_mps", 0.5)),
        )
    cue_model = CueModel(
        base_probability_by_class={
            name: float(value)
            for name, value in cue_config["base_probability_by_class"].items()
        },
        dwell_time_constant=float(cue_config["dwell_time_constant_s"]),
        cross_track_decay=float(cue_config["cross_track_decay"]),
        probability_override=(
            None if cue_config.get("probability_override") is None
            else float(cue_config["probability_override"])
        ),
        empirical_model=empirical_model,
    )
    monte_carlo = m1b["monte_carlo"]
    return generation, cue_model, int(monte_carlo["trials"]), int(monte_carlo["seed"])


def _jsonable_observation(observation: SegmentObservation) -> Mapping[str, object]:
    payload = asdict(observation)
    return payload


def _write_json(
        path: Path,
        params: M0Parameters,
        metrics: M0Metrics,
        camera: CameraModel,
        targets: Sequence[Target],
        observations: Sequence[SegmentObservation],
        monte_carlo: Optional[MonteCarloSummary] = None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "model_stage": "M1b" if monte_carlo else "M1a",
        "parameters": {
            "area": asdict(params.area),
            "lane_direction": params.lane_direction,
            "lane_spacing_m": params.lane_spacing,
            "altitude_m": params.altitude,
            "speed_mps": params.speed,
            "turn_penalty_s": params.turn_penalty,
        },
        "camera": {
            **asdict(camera),
            "vertical_fov": camera.vertical_fov,
            "footprint_at_search_altitude_m": camera.footprint(params.altitude),
        },
        "metrics": asdict(metrics),
        "targets": [asdict(target) for target in targets],
        "full_target_observations": [
            _jsonable_observation(observation) for observation in observations
        ],
        "monte_carlo": asdict(monte_carlo) if monte_carlo else None,
    }
    with path.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


def _write_waypoints(path: Path, waypoints: Iterable[Point3]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("index", "x_m", "y_m", "z_m"))
        for index, waypoint in enumerate(waypoints):
            writer.writerow((index, waypoint.x, waypoint.y, waypoint.z))


def _print_summary(params: M0Parameters, metrics: M0Metrics) -> None:
    print("Liftrace Search Simulation — M1b")
    print(f"area: {params.area.width_x:.3f} m × {params.area.width_y:.3f} m "
          f"= {metrics.area_m2:.3f} m²")
    print(f"lanes: {metrics.lane_count}, waypoints: {metrics.waypoint_count}, "
          f"direction: {params.lane_direction}")
    print(f"distance: lanes={metrics.lane_distance_m:.3f} m, "
          f"connectors={metrics.connector_distance_m:.3f} m, "
          f"total={metrics.route_distance_m:.3f} m")
    print(f"turns: {metrics.turn_count} × {params.turn_penalty:.3f} s "
          f"= {metrics.turn_time_s:.3f} s")
    print(f"time: flight={metrics.flight_time_s:.3f} s, "
          f"total={metrics.total_time_s:.3f} s")


def _print_visibility_summary(
        params: M0Parameters,
        camera: CameraModel,
        targets: Sequence[Target],
        center_observations: Sequence[SegmentObservation],
        full_observations: Sequence[SegmentObservation],
) -> None:
    footprint_cross, footprint_along = camera.footprint(params.altitude)
    print(f"camera: hfov={math.degrees(camera.horizontal_fov):.2f}°, "
          f"vfov={math.degrees(camera.vertical_fov):.2f}°, "
          f"image={camera.image_width}×{camera.image_height} @ {camera.update_rate:.1f} Hz")
    print(f"footprint at z={params.altitude:.3f} m: "
          f"cross={footprint_cross:.3f} m, along={footprint_along:.3f} m")

    for target in targets:
        center_hits = [item for item in center_observations if item.target_id == target.target_id]
        full_hits = [item for item in full_observations if item.target_id == target.target_id]
        best = max(full_hits, key=lambda item: item.dwell_time_s, default=None)
        if best is None:
            print(f"target {target.target_id} ({target.class_name}): full_target not visible")
            continue
        print(
            f"target {target.target_id} ({target.class_name}): "
            f"center_segments={len(center_hits)}, full_segments={len(full_hits)}, "
            f"best_segment={best.segment_index}({best.segment_kind}), "
            f"cross={best.cross_track_distance_m:.3f} m, "
            f"visible={best.visible_path_length_m:.3f} m, "
            f"dwell={best.dwell_time_s:.3f} s"
        )


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parent / "config" / "baseline.yaml",
        help="M0/M1 YAML configuration",
    )
    parser.add_argument(
        "--monte-carlo", type=int, metavar="N",
        help="run N M1b trials (defaults to config value)",
    )
    parser.add_argument("--seed", type=int, help="override M1b random seed")
    parser.add_argument(
        "--no-monte-carlo", action="store_true",
        help="print only deterministic M0/M1a report",
    )
    parser.add_argument("--output-json", type=Path, help="optional summary JSON path")
    parser.add_argument("--waypoints-csv", type=Path, help="optional waypoint CSV path")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    params, config = load_parameters(args.config)
    camera = load_camera(config)
    targets = load_demo_targets(config)
    waypoints = generate_boustrophedon(params)
    simulation_waypoints = waypoints
    simulation_obstacles = ()
    simulation_occlusion = None
    if config.get("obstacles"):
        from obstacles import load_obstacles, plan_route_astar
        from occlusion import load_occlusion_config
        simulation_obstacles, horizontal_clearance, vertical_clearance = load_obstacles(config)
        obstacle_config = config["obstacles"]
        plan = plan_route_astar(
            waypoints,
            simulation_obstacles,
            (params.area.min_x, params.area.max_x, params.area.min_y, params.area.max_y),
            float(obstacle_config.get("planning_resolution_m", 0.1)),
            horizontal_clearance,
            vertical_clearance,
            bool(obstacle_config.get("allow_diagonal", True)),
        )
        simulation_waypoints = plan.waypoints
        simulation_occlusion = load_occlusion_config(config)
    metrics = evaluate_m0(params, waypoints)
    center_observations = evaluate_target_visibility(
        waypoints, targets, camera, params.speed, require_fully_in_frame=False)
    full_observations = evaluate_target_visibility(
        waypoints, targets, camera, params.speed, require_fully_in_frame=True)
    monte_carlo = None
    if not args.no_monte_carlo and "m1b" in config:
        generation, cue_model, configured_trials, configured_seed = load_m1b(config)
        monte_carlo = run_monte_carlo(
            params, simulation_waypoints, camera, generation, cue_model,
            trials=args.monte_carlo or configured_trials,
            seed=configured_seed if args.seed is None else args.seed,
            obstacles=simulation_obstacles,
            occlusion_config=simulation_occlusion,
        )
    _print_summary(params, metrics)
    _print_visibility_summary(
        params, camera, targets, center_observations, full_observations)
    if monte_carlo:
        first_time = ("none" if monte_carlo.mean_first_cue_time_s is None
                      else f"{monte_carlo.mean_first_cue_time_s:.3f} s")
        print(f"M1b Monte Carlo: trials={monte_carlo.trials}, seed={monte_carlo.seed}")
        print(f"Cue: mean_rate={monte_carlo.mean_cue_rate:.3f}, "
              f"mean_count={monte_carlo.mean_targets_cued:.3f}, "
              f"P(all)={monte_carlo.probability_all_targets_cued:.3f}, "
              f"mean_first_time={first_time}")
        print("class Cue rates: " + ", ".join(
            f"{name}={rate:.3f}" for name, rate in monte_carlo.class_cue_rates.items()))
        if monte_carlo.occlusion_pass_rate is not None:
            print(
                f"occlusion: passes={monte_carlo.occlusion_evaluated_passes}, "
                f"gate_pass_rate={monte_carlo.occlusion_pass_rate:.3f}, "
                f"mean_visible_fraction={monte_carlo.mean_visible_fraction:.3f}"
            )
    if args.output_json:
        _write_json(
            args.output_json,
            params,
            metrics,
            camera,
            targets,
            full_observations,
            monte_carlo,
        )
    if args.waypoints_csv:
        _write_waypoints(args.waypoints_csv, waypoints)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
