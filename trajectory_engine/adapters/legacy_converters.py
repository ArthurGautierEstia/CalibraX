from __future__ import annotations

from models.trajectory_preview import TrajectoryPreviewResult as LegacyPreviewResult
from models.trajectory_preview import TrajectoryPreviewSample as LegacyPreviewSample
from models.trajectory_preview import TrajectoryPreviewSegment as LegacyPreviewSegment
from models.trajectory_result import (
    JointDynamicStats as LegacyJointDynamicStats,
    SegmentResult as LegacySegmentResult,
    TrajectoryCollisionDiagnostic as LegacyCollisionDiagnostic,
    TrajectoryCollisionDomain as LegacyCollisionDomain,
    TrajectoryComputationStatus as LegacyTrajectoryComputationStatus,
    TrajectoryDynamicViolation as LegacyTrajectoryDynamicViolation,
    TrajectoryDynamicViolationKind as LegacyTrajectoryDynamicViolationKind,
    TrajectoryDynamicViolationSeverity as LegacyTrajectoryDynamicViolationSeverity,
    TrajectoryResult as LegacyTrajectoryResult,
    TrajectorySample as LegacyTrajectorySample,
    TrajectorySampleErrorCode as LegacyTrajectorySampleErrorCode,
    TrajectorySampleMgiSolution as LegacyTrajectorySampleMgiSolution,
)
from models.types import JointAngles6, Pose6, XYZ3
from trajectory_engine.models.pipeline import (
    TrajectoryComputationStatus,
    TrajectoryDynamicViolationSeverity,
    TrajectoryPreviewResult,
    TrajectoryPreviewSample,
    TrajectoryPreviewSegment,
    TrajectoryResult,
    TrajectorySample,
)


def _legacy_status(status: TrajectoryComputationStatus) -> LegacyTrajectoryComputationStatus:
    return LegacyTrajectoryComputationStatus[status.name]


def _legacy_error_code(error_code: LegacyTrajectorySampleErrorCode | object) -> LegacyTrajectorySampleErrorCode:
    if isinstance(error_code, LegacyTrajectorySampleErrorCode):
        return error_code
    return LegacyTrajectorySampleErrorCode[getattr(error_code, "name", str(error_code))]


def to_legacy_preview(preview: TrajectoryPreviewResult) -> LegacyPreviewResult:
    legacy = LegacyPreviewResult()
    legacy.status = _legacy_status(preview.status)
    legacy.first_error_segment_index = preview.first_error_segment_index
    legacy.analysis_pending = bool(preview.analysis_pending)
    for segment in preview.segments:
        legacy_segment = LegacyPreviewSegment()
        legacy_segment.status = _legacy_status(segment.status)
        legacy_segment.mode = segment.mode
        legacy_segment.duration_s = float(segment.duration_s)
        legacy_segment.last_time_s = float(segment.last_time_s)
        legacy_segment.in_direction = XYZ3.from_values(segment.in_direction.to_list())
        legacy_segment.out_direction = XYZ3.from_values(segment.out_direction.to_list())
        for sample in segment.samples:
            legacy_segment.samples.append(
                LegacyPreviewSample(
                    time_s=sample.time_s,
                    pose_base=sample.pose_base.copy(),
                    joints_deg=None if sample.joints_deg is None else sample.joints_deg.copy(),
                    reachable=sample.reachable,
                )
            )
        legacy.segments.append(legacy_segment)
    return legacy


def to_legacy_trajectory(trajectory: TrajectoryResult) -> LegacyTrajectoryResult:
    legacy = LegacyTrajectoryResult()
    legacy.status = _legacy_status(trajectory.status)
    legacy.first_error_segment_index = trajectory.first_error_segment_index
    for segment in trajectory.segments:
        legacy_segment = LegacySegmentResult()
        legacy_segment.status = _legacy_status(segment.status)
        legacy_segment.mode = segment.mode
        legacy_segment.in_direction = [float(v) for v in segment.in_direction[:3]]
        legacy_segment.out_direction = [float(v) for v in segment.out_direction[:3]]
        legacy_segment.duration = float(segment.duration)
        legacy_segment.last_time = float(segment.last_time)
        legacy_segment.first_error_sample_index = segment.first_error_sample_index
        legacy_segment.first_error_axis = segment.first_error_axis
        legacy_segment.joints_stats = [
            LegacyJointDynamicStats(
                max_positive_velocity=stats.max_positive_velocity,
                max_negative_velocity=stats.max_negative_velocity,
                max_acceleration=stats.max_acceleration,
                max_deceleration=stats.max_deceleration,
            )
            for stats in segment.joints_stats
        ]
        for sample in segment.samples:
            legacy_sample = LegacyTrajectorySample()
            legacy_sample.time = float(sample.time)
            legacy_sample.joints = [float(v) for v in sample.joints[:6]]
            legacy_sample.pose = [float(v) for v in sample.pose[:6]]
            legacy_sample.kinematics = sample.kinematics
            legacy_sample.reachable = bool(sample.reachable)
            legacy_sample.configuration = sample.configuration
            legacy_sample.velocity = float(sample.velocity)
            legacy_sample.acceleration = float(sample.acceleration)
            legacy_sample.cartesian_velocity = [float(v) for v in sample.cartesian_velocity[:6]]
            legacy_sample.cartesian_acceleration = [float(v) for v in sample.cartesian_acceleration[:6]]
            legacy_sample.cartesian_jerk = [float(v) for v in sample.cartesian_jerk[:6]]
            legacy_sample.cartesian_velocity_valid = bool(sample.cartesian_velocity_valid)
            legacy_sample.cartesian_acceleration_valid = bool(sample.cartesian_acceleration_valid)
            legacy_sample.cartesian_jerk_valid = bool(sample.cartesian_jerk_valid)
            legacy_sample.articular_velocity = [float(v) for v in sample.articular_velocity[:6]]
            legacy_sample.articular_acceleration = [float(v) for v in sample.articular_acceleration[:6]]
            legacy_sample.articular_jerk = [float(v) for v in sample.articular_jerk[:6]]
            legacy_sample.articular_velocity_valid = bool(sample.articular_velocity_valid)
            legacy_sample.articular_acceleration_valid = bool(sample.articular_acceleration_valid)
            legacy_sample.articular_jerk_valid = bool(sample.articular_jerk_valid)
            legacy_sample.dynamic_violations = [
                LegacyTrajectoryDynamicViolation(
                    kind=LegacyTrajectoryDynamicViolationKind[violation.kind.name],
                    axis=violation.axis,
                    value=violation.value,
                    limit=violation.limit,
                    severity=LegacyTrajectoryDynamicViolationSeverity[violation.severity.name],
                )
                for violation in sample.dynamic_violations
            ]
            legacy_sample.collisions = [
                LegacyCollisionDiagnostic(
                    domain=LegacyCollisionDomain[collision.domain.name],
                    owner_a=collision.owner_a,
                    name_a=collision.name_a,
                    source_index_a=collision.source_index_a,
                    owner_b=collision.owner_b,
                    name_b=collision.name_b,
                    source_index_b=collision.source_index_b,
                )
                for collision in sample.collisions
            ]
            legacy_sample.error_code = _legacy_error_code(sample.error_code)
            legacy_sample.error_axis = sample.error_axis
            legacy_sample.mgi_solutions = {
                key: LegacyTrajectorySampleMgiSolution(status=value.status, joints=list(value.joints))
                for key, value in sample.mgi_solutions.items()
            }
            legacy_segment.samples.append(legacy_sample)
        legacy.segments.append(legacy_segment)
    return legacy
