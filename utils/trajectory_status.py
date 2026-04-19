from __future__ import annotations

from models.trajectory_result import (
    TrajectoryDynamicViolationKind,
    TrajectoryDynamicViolationSeverity,
    SegmentResult,
    TrajectoryComputationStatus,
    TrajectoryResult,
    TrajectorySampleErrorCode,
)


def _axis_label(axis: int | None) -> str:
    if axis is None or axis < 0:
        return "axe inconnu"
    return f"J{axis + 1}"


def _axes_label(axes: set[int]) -> str:
    if not axes:
        return "axe inconnu"
    return ", ".join(_axis_label(axis) for axis in sorted(axes))


def _dynamic_kind_message(kind: TrajectoryDynamicViolationKind) -> str:
    if kind == TrajectoryDynamicViolationKind.SPEED:
        return "vitesse dépassée"
    if kind == TrajectoryDynamicViolationKind.ACCELERATION:
        return "acceleration estimée dépassée"
    if kind == TrajectoryDynamicViolationKind.JERK:
        return "jerk dépassé"
    return kind.value


def _is_dynamic_error_status(status: TrajectoryComputationStatus) -> bool:
    return status in (
        TrajectoryComputationStatus.SPEED_LIMIT_EXCEEDED,
        TrajectoryComputationStatus.JERK_LIMIT_EXCEEDED,
    )


def _is_dynamic_error_code(error_code: TrajectorySampleErrorCode) -> bool:
    return error_code in (
        TrajectorySampleErrorCode.SPEED_LIMIT_EXCEEDED,
        TrajectorySampleErrorCode.JERK_LIMIT_EXCEEDED,
    )


def status_to_message(status: TrajectoryComputationStatus, axis: int | None = None) -> str:
    if status == TrajectoryComputationStatus.SUCCESS:
        return "valide"
    if status == TrajectoryComputationStatus.POINT_UNREACHABLE:
        return "point inatteignable"
    if status == TrajectoryComputationStatus.CONFIGURATION_JUMP:
        return f"saut de configuration détecté ({_axis_label(axis)})"
    if status == TrajectoryComputationStatus.SPEED_LIMIT_EXCEEDED:
        return f"vitesse dépassée ({_axis_label(axis)})"
    if status == TrajectoryComputationStatus.JERK_LIMIT_EXCEEDED:
        return f"jerk dépassé ({_axis_label(axis)})"
    if status == TrajectoryComputationStatus.NO_COMMON_ALLOWED_CONFIGURATION:
        return "aucune configuration autorisée commune"
    if status == TrajectoryComputationStatus.FORBIDDEN_CONFIGURATION:
        return "configuration interdite"
    return status.value


def sample_error_to_message(error_code: TrajectorySampleErrorCode, axis: int | None = None) -> str:
    if error_code == TrajectorySampleErrorCode.NONE:
        return "valide"
    if error_code == TrajectorySampleErrorCode.POINT_UNREACHABLE:
        return "point inatteignable"
    if error_code == TrajectorySampleErrorCode.CONFIGURATION_JUMP:
        return f"saut de configuration détecté ({_axis_label(axis)})"
    if error_code == TrajectorySampleErrorCode.SPEED_LIMIT_EXCEEDED:
        return f"vitesse dépassée ({_axis_label(axis)})"
    if error_code == TrajectorySampleErrorCode.JERK_LIMIT_EXCEEDED:
        return f"jerk dépassé ({_axis_label(axis)})"
    if error_code == TrajectorySampleErrorCode.FORBIDDEN_CONFIGURATION:
        return "configuration interdite"
    return error_code.value


def build_segment_dynamic_violation_messages(
    segment: SegmentResult,
    segment_index: int,
    severity: TrajectoryDynamicViolationSeverity,
) -> list[str]:
    prefix = f"Segment {max(0, segment_index) + 1}"
    grouped_axes: dict[TrajectoryDynamicViolationKind, set[int]] = {}

    for sample in segment.samples:
        for violation in sample.dynamic_violations:
            if violation.severity != severity:
                continue
            grouped_axes.setdefault(violation.kind, set()).add(violation.axis)

    ordered_kinds = [
        TrajectoryDynamicViolationKind.SPEED,
        TrajectoryDynamicViolationKind.ACCELERATION,
        TrajectoryDynamicViolationKind.JERK,
    ]
    messages: list[str] = []
    for kind in ordered_kinds:
        axes = grouped_axes.get(kind)
        if not axes:
            continue
        messages.append(f"{prefix}: {_dynamic_kind_message(kind)} ({_axes_label(axes)})")
    return messages


def build_segment_issue_messages(segment: SegmentResult, segment_index: int) -> list[str]:
    if segment_index < 0:
        segment_index = 0
    prefix = f"Segment {segment_index + 1}"

    messages: list[str] = []
    seen_messages: set[str] = set()

    if segment.status != TrajectoryComputationStatus.SUCCESS and not _is_dynamic_error_status(segment.status):
        status_message = f"{prefix}: {status_to_message(segment.status, segment.first_error_axis)}"
        messages.append(status_message)
        seen_messages.add(status_message)

    first_axis_by_code: dict[TrajectorySampleErrorCode, int | None] = {}
    for sample in segment.samples:
        code = sample.error_code
        if code == TrajectorySampleErrorCode.NONE:
            continue
        if _is_dynamic_error_code(code):
            continue
        if code not in first_axis_by_code:
            first_axis_by_code[code] = sample.error_axis

    ordered_codes = [
        TrajectorySampleErrorCode.POINT_UNREACHABLE,
        TrajectorySampleErrorCode.FORBIDDEN_CONFIGURATION,
        TrajectorySampleErrorCode.CONFIGURATION_JUMP,
    ]
    for code in ordered_codes:
        if code not in first_axis_by_code:
            continue
        message = f"{prefix}: {sample_error_to_message(code, first_axis_by_code[code])}"
        if message in seen_messages:
            continue
        messages.append(message)
        seen_messages.add(message)

    dynamic_messages = build_segment_dynamic_violation_messages(
        segment,
        segment_index,
        TrajectoryDynamicViolationSeverity.ERROR,
    )
    if not dynamic_messages and _is_dynamic_error_status(segment.status):
        dynamic_messages = [f"{prefix}: {status_to_message(segment.status, segment.first_error_axis)}"]

    for message in dynamic_messages:
        if message in seen_messages:
            continue
        messages.append(message)
        seen_messages.add(message)

    return messages


def build_segment_warning_messages(segment: SegmentResult, segment_index: int) -> list[str]:
    return build_segment_dynamic_violation_messages(
        segment,
        segment_index,
        TrajectoryDynamicViolationSeverity.WARNING,
    )


def build_trajectory_issue_messages(trajectory: TrajectoryResult | None) -> list[str]:
    if trajectory is None:
        return []
    messages: list[str] = []
    for index, segment in enumerate(trajectory.segments):
        messages.extend(build_segment_issue_messages(segment, index))
    return messages


def build_trajectory_warning_messages(trajectory: TrajectoryResult | None) -> list[str]:
    if trajectory is None:
        return []
    messages: list[str] = []
    for index, segment in enumerate(trajectory.segments):
        messages.extend(build_segment_warning_messages(segment, index))
    return messages


def join_issue_messages(messages: list[str], separator: str = " | ") -> str:
    if not messages:
        return ""
    return separator.join(messages)
