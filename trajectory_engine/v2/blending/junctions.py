from __future__ import annotations

from models.trajectory_keypoint import KeypointMotionMode, TrajectoryKeypoint
from trajectory_engine.v2.models import TrajectoryPassMode


def _is_cartesian_mode(mode: KeypointMotionMode) -> bool:
    return mode in (KeypointMotionMode.LINEAR, KeypointMotionMode.CUBIC)


def resolve_cartesian_exit_speed_mm_s(
    current_to_keypoint: TrajectoryKeypoint,
    next_to_keypoint: TrajectoryKeypoint | None,
) -> float:
    if TrajectoryPassMode.from_value(current_to_keypoint.pass_mode) != TrajectoryPassMode.FLY_BY:
        return 0.0
    if next_to_keypoint is None:
        return 0.0
    if not _is_cartesian_mode(current_to_keypoint.mode) or not _is_cartesian_mode(next_to_keypoint.mode):
        return 0.0
    current_speed = max(0.0, float(current_to_keypoint.linear_speed_mps) * 1000.0)
    next_speed = max(0.0, float(next_to_keypoint.linear_speed_mps) * 1000.0)
    return min(current_speed, next_speed)
