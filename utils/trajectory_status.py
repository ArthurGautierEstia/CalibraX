from __future__ import annotations

from models.trajectory_result import (
    SegmentResult,
    TrajectoryComputationStatus,
    TrajectoryResult,
    TrajectorySampleErrorCode,
)


def _axis_label(axis: int | None) -> str:
    if axis is None or axis < 0:
        return "axe inconnu"
    return f"J{axis + 1}"


def status_to_message(status: TrajectoryComputationStatus, axis: int | None = None) -> str:
    if status == TrajectoryComputationStatus.SUCCESS:
        return "valide"
    if status == TrajectoryComputationStatus.POINT_UNREACHABLE:
        return "point inatteignable"
    if status == TrajectoryComputationStatus.OVER_DYNAMIC_LIMIT:
        return f"limite dynamique depassee ({_axis_label(axis)})"
    if status == TrajectoryComputationStatus.NO_COMMON_ALLOWED_CONFIGURATION:
        return "aucune configuration autorisee commune"
    if status == TrajectoryComputationStatus.FORBIDDEN_CONFIGURATION:
        return "configuration interdite"
    return status.value


def sample_error_to_message(error_code: TrajectorySampleErrorCode, axis: int | None = None) -> str:
    if error_code == TrajectorySampleErrorCode.NONE:
        return "valide"
    if error_code == TrajectorySampleErrorCode.POINT_UNREACHABLE:
        return "point inatteignable"
    if error_code == TrajectorySampleErrorCode.OVER_DYNAMIC_LIMIT:
        return f"limite dynamique depassee ({_axis_label(axis)})"
    if error_code == TrajectorySampleErrorCode.FORBIDDEN_CONFIGURATION:
        return "configuration interdite"
    return error_code.value


def build_segment_issue_messages(segment: SegmentResult, segment_index: int) -> list[str]:
    if segment_index < 0:
        segment_index = 0
    prefix = f"Segment {segment_index + 1}"

    messages: list[str] = []
    seen_messages: set[str] = set()

    if segment.status != TrajectoryComputationStatus.SUCCESS:
        status_message = f"{prefix}: {status_to_message(segment.status, segment.first_error_axis)}"
        messages.append(status_message)
        seen_messages.add(status_message)

    first_axis_by_code: dict[TrajectorySampleErrorCode, int | None] = {}
    for sample in segment.samples:
        code = sample.error_code
        if code == TrajectorySampleErrorCode.NONE:
            continue
        if code not in first_axis_by_code:
            first_axis_by_code[code] = sample.error_axis

    ordered_codes = [
        TrajectorySampleErrorCode.POINT_UNREACHABLE,
        TrajectorySampleErrorCode.FORBIDDEN_CONFIGURATION,
        TrajectorySampleErrorCode.OVER_DYNAMIC_LIMIT,
    ]
    for code in ordered_codes:
        if code not in first_axis_by_code:
            continue
        message = f"{prefix}: {sample_error_to_message(code, first_axis_by_code[code])}"
        if message in seen_messages:
            continue
        messages.append(message)
        seen_messages.add(message)

    return messages


def build_trajectory_issue_messages(trajectory: TrajectoryResult | None) -> list[str]:
    if trajectory is None:
        return []
    messages: list[str] = []
    for index, segment in enumerate(trajectory.segments):
        messages.extend(build_segment_issue_messages(segment, index))
    return messages


def join_issue_messages(messages: list[str], separator: str = " | ") -> str:
    if not messages:
        return ""
    return separator.join(messages)
