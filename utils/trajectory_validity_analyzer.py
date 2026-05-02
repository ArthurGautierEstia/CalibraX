from __future__ import annotations

from dataclasses import dataclass
import math
import threading

import numpy as np
from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot

from models.primitive_collider_models import PrimitiveCollider, PrimitiveColliderData, RobotAxisColliderData
from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.trajectory_result import (
    SegmentResult,
    TrajectoryCollisionDiagnostic,
    TrajectoryCollisionDomain,
    TrajectoryComputationStatus,
    TrajectoryResult,
    TrajectorySample,
    TrajectorySampleErrorCode,
)
from models.types import Pose6, XYZ3
from models.workspace_model import WorkspaceModel
from utils.collision_utils import (
    CollisionPair,
    CollisionWorldCache,
    build_world_frame_transforms,
    resolve_flange_world_transform,
)
import utils.math_utils as math_utils


@dataclass
class TrajectoryValiditySampleResult:
    segment_index: int
    sample_index: int
    error_code: TrajectorySampleErrorCode
    collisions: list[TrajectoryCollisionDiagnostic]
    tcp_world_xyz: XYZ3 | None = None


@dataclass
class TrajectoryValidityAnalysisResult:
    job_id: int
    cancelled: bool
    has_error: bool
    sample_results: list[TrajectoryValiditySampleResult]


class TrajectoryValidityCancelToken:
    def __init__(self) -> None:
        self._event = threading.Event()

    def request_cancel(self) -> None:
        self._event.set()

    def is_cancelled(self) -> bool:
        return self._event.is_set()


@dataclass
class TrajectoryValidityKinematicsSnapshot:
    dh_params: list[list[float]]
    measured_dh_params: list[list[float]]
    measured_dh_enabled: bool
    axis_reversed: list[int]
    corrections: list[list[float]]
    tool_pose: Pose6

    @staticmethod
    def _normalize_rows(
        values: list[list[float]],
        row_count: int,
        column_count: int,
    ) -> list[list[float]]:
        rows: list[list[float]] = []
        for row_index in range(row_count):
            source = values[row_index] if row_index < len(values) else []
            normalized = [float(source[col]) if col < len(source) else 0.0 for col in range(column_count)]
            rows.append(normalized)
        return rows

    @classmethod
    def from_robot_model(
        cls,
        robot_model: RobotModel,
        tool_model: ToolModel,
    ) -> "TrajectoryValidityKinematicsSnapshot":
        return cls(
            dh_params=cls._normalize_rows(robot_model.get_dh_params(), 6, 4),
            measured_dh_params=cls._normalize_rows(robot_model.get_measured_dh_params(), 6, 4),
            measured_dh_enabled=bool(robot_model.get_measured_dh_enabled()),
            axis_reversed=[int(v) for v in robot_model.get_axis_reversed()[:6]],
            corrections=cls._normalize_rows(robot_model.get_corrections(), 6, 6),
            tool_pose=tool_model.get_tool_pose(),
        )

    def _active_dh_params(self) -> list[list[float]]:
        return self.measured_dh_params if self.measured_dh_enabled else self.dh_params

    def compute_corrected_matrices(self, joints_deg: list[float]) -> list[np.ndarray] | None:
        if len(joints_deg) < 6:
            return None

        corrected_matrices: list[np.ndarray] = [np.eye(4, dtype=float)]
        transform = np.eye(4, dtype=float)
        dh_params = self._active_dh_params()
        axis_reversed = self.axis_reversed[:6]
        while len(axis_reversed) < 6:
            axis_reversed.append(1)

        for axis_index in range(6):
            row = dh_params[axis_index]
            alpha_rad = math.radians(float(row[0]))
            d_mm = float(row[1])
            theta_offset_rad = math.radians(float(row[2]))
            r_mm = float(row[3])
            joint_rad = math.radians(float(joints_deg[axis_index]) * float(axis_reversed[axis_index]))
            transform = transform @ math_utils.dh_modified(
                alpha_rad,
                d_mm,
                theta_offset_rad + joint_rad,
                r_mm,
            )
            transform = math_utils.correction_6d(transform, *self.corrections[axis_index])
            corrected_matrices.append(transform.copy())

        transform = transform @ math_utils.pose_zyx_to_matrix(self.tool_pose)
        transform = math_utils.correction_6d(transform, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        corrected_matrices.append(transform.copy())
        return corrected_matrices


@dataclass
class TrajectoryValidityContext:
    kinematics: TrajectoryValidityKinematicsSnapshot
    workspace_tcp_zone_colliders: list[PrimitiveCollider]
    workspace_collision_zones: list[PrimitiveColliderData]
    robot_axis_colliders: list[RobotAxisColliderData]
    tool_colliders: list[PrimitiveColliderData]
    evaluated_robot_axis_colliders: list[bool]
    robot_base_transform_world: np.ndarray

    @classmethod
    def from_models(
        cls,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
    ) -> "TrajectoryValidityContext":
        return cls(
            kinematics=TrajectoryValidityKinematicsSnapshot.from_robot_model(robot_model, tool_model),
            workspace_tcp_zone_colliders=workspace_model.get_workspace_tcp_zone_colliders(),
            workspace_collision_zones=workspace_model.get_workspace_collision_zones(),
            robot_axis_colliders=robot_model.get_axis_collider_data(),
            tool_colliders=tool_model.get_tool_collider_data(),
            evaluated_robot_axis_colliders=tool_model.get_evaluated_robot_axis_colliders(),
            robot_base_transform_world=np.array(
                workspace_model.get_robot_base_transform_world().matrix,
                dtype=float,
            ),
        )


class TrajectoryValidityAnalyzer:
    def __init__(self, context: TrajectoryValidityContext) -> None:
        self.context = context
        self._collision_cache = CollisionWorldCache()
        self._collision_cache.set_workspace_tcp_zone_colliders(context.workspace_tcp_zone_colliders)
        self._collision_cache.set_workspace_collision_zones(context.workspace_collision_zones)
        self._collision_cache.set_robot_axis_templates(context.robot_axis_colliders)
        self._collision_cache.set_tool_templates(context.tool_colliders)

    @staticmethod
    def _diagnostics_from_pairs(
        pairs: list[CollisionPair],
        domain: TrajectoryCollisionDomain,
    ) -> list[TrajectoryCollisionDiagnostic]:
        diagnostics: list[TrajectoryCollisionDiagnostic] = []
        for pair in pairs:
            diagnostics.append(
                TrajectoryCollisionDiagnostic(
                    domain=domain,
                    owner_a=pair.shape_a.owner,
                    name_a=pair.shape_a.name,
                    source_index_a=pair.shape_a.source_index,
                    owner_b=pair.shape_b.owner,
                    name_b=pair.shape_b.name,
                    source_index_b=pair.shape_b.source_index,
                )
            )
        return diagnostics

    def analyze_sample(
        self,
        segment_index: int,
        sample_index: int,
        sample: TrajectorySample,
    ) -> TrajectoryValiditySampleResult | None:
        if sample.error_code != TrajectorySampleErrorCode.NONE:
            return None
        if not sample.reachable:
            return None

        corrected_matrices = None
        if sample.kinematics is not None:
            corrected_matrices = sample.kinematics.corrected_matrices
        if corrected_matrices is None:
            corrected_matrices = self.context.kinematics.compute_corrected_matrices(sample.joints)
        if corrected_matrices is None:
            return None

        frame_world_transforms = build_world_frame_transforms(
            corrected_matrices,
            self.context.robot_base_transform_world,
        )
        flange_world_transform = resolve_flange_world_transform(frame_world_transforms)
        self._collision_cache.update_dynamic_world_shapes(frame_world_transforms, flange_world_transform)

        diagnostics = TrajectoryValidityAnalyzer._diagnostics_from_pairs(
            self._collision_cache.find_workspace_collisions(),
            TrajectoryCollisionDomain.WORKSPACE,
        )
        diagnostics.extend(
            TrajectoryValidityAnalyzer._diagnostics_from_pairs(
                self._collision_cache.find_robot_tool_collisions(
                    self.context.evaluated_robot_axis_colliders
                ),
                TrajectoryCollisionDomain.ROBOT_TOOL,
            )
        )
        if diagnostics:
            return TrajectoryValiditySampleResult(
                segment_index=segment_index,
                sample_index=sample_index,
                error_code=TrajectorySampleErrorCode.COLLISION_DETECTED,
                collisions=diagnostics,
                tcp_world_xyz=_tcp_world_xyz(frame_world_transforms),
            )

        tcp_world_xyz = _tcp_world_xyz(frame_world_transforms)
        if tcp_world_xyz is None:
            return None
        if not self._collision_cache.is_tcp_inside_workspace(np.array(tcp_world_xyz.to_list(), dtype=float)):
            return TrajectoryValiditySampleResult(
                segment_index=segment_index,
                sample_index=sample_index,
                error_code=TrajectorySampleErrorCode.TCP_WORKSPACE_EXIT,
                collisions=[],
                tcp_world_xyz=tcp_world_xyz,
            )

        return None

    def analyze_trajectory(
        self,
        job_id: int,
        trajectory: TrajectoryResult,
        cancel_token: TrajectoryValidityCancelToken,
    ) -> TrajectoryValidityAnalysisResult:
        sample_results: list[TrajectoryValiditySampleResult] = []
        for segment_index, segment in enumerate(trajectory.segments):
            for sample_index, sample in enumerate(segment.samples):
                if cancel_token.is_cancelled():
                    return TrajectoryValidityAnalysisResult(
                        job_id=job_id,
                        cancelled=True,
                        has_error=False,
                        sample_results=[],
                    )
                sample_result = self.analyze_sample(segment_index, sample_index, sample)
                if sample_result is not None:
                    sample_results.append(sample_result)

        return TrajectoryValidityAnalysisResult(
            job_id=job_id,
            cancelled=False,
            has_error=bool(sample_results),
            sample_results=sample_results,
        )


class TrajectoryValidityWorker(QObject):
    completed = pyqtSignal(int, object)
    cancelled = pyqtSignal(int)
    finished = pyqtSignal()

    def __init__(
        self,
        job_id: int,
        trajectory: TrajectoryResult,
        context: TrajectoryValidityContext,
        cancel_token: TrajectoryValidityCancelToken,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.job_id = int(job_id)
        self.trajectory = trajectory
        self.context = context
        self.cancel_token = cancel_token

    def request_cancel(self) -> None:
        self.cancel_token.request_cancel()

    @pyqtSlot()
    def run(self) -> None:
        try:
            analyzer = TrajectoryValidityAnalyzer(self.context)
            result = analyzer.analyze_trajectory(self.job_id, self.trajectory, self.cancel_token)
            if result.cancelled:
                self.cancelled.emit(self.job_id)
                return
            self.completed.emit(self.job_id, result)
        finally:
            self.finished.emit()


def prepare_trajectory_validity_analysis(trajectory: TrajectoryResult) -> bool:
    changed = False
    for segment in trajectory.segments:
        for sample in segment.samples:
            if sample.error_code not in _VALIDITY_ERROR_CODES:
                continue
            sample.error_code = TrajectorySampleErrorCode.NONE
            sample.error_axis = None
            sample.collisions = []
            changed = True
        _refresh_segment_status(segment)
    _refresh_trajectory_status(trajectory)
    return changed


def apply_trajectory_validity_result(
    trajectory: TrajectoryResult,
    result: TrajectoryValidityAnalysisResult,
) -> bool:
    if result.cancelled:
        return False

    changed = prepare_trajectory_validity_analysis(trajectory)
    for sample_result in result.sample_results:
        segment = _segment_at(trajectory, sample_result.segment_index)
        if segment is None:
            continue
        sample = _sample_at(segment, sample_result.sample_index)
        if sample is None:
            continue
        if sample.error_code != TrajectorySampleErrorCode.NONE:
            continue
        if not sample.reachable:
            continue

        sample.collisions = list(sample_result.collisions)
        sample.error_code = sample_result.error_code
        sample.error_axis = None
        _mark_segment_error(segment, sample_result.sample_index, sample_result.error_code)
        changed = True

        if trajectory.status == TrajectoryComputationStatus.SUCCESS:
            trajectory.status = _status_from_sample_error(sample_result.error_code)
            trajectory.first_error_segment_index = sample_result.segment_index

    return changed


def _segment_at(trajectory: TrajectoryResult, segment_index: int) -> SegmentResult | None:
    if segment_index < 0 or segment_index >= len(trajectory.segments):
        return None
    return trajectory.segments[segment_index]


def _sample_at(segment: SegmentResult, sample_index: int) -> TrajectorySample | None:
    if sample_index < 0 or sample_index >= len(segment.samples):
        return None
    return segment.samples[sample_index]


def _mark_segment_error(
    segment: SegmentResult,
    sample_index: int,
    error_code: TrajectorySampleErrorCode,
) -> None:
    if segment.first_error_sample_index is None:
        segment.first_error_sample_index = int(sample_index)
        segment.first_error_axis = None
    if segment.status == TrajectoryComputationStatus.SUCCESS:
        segment.status = _status_from_sample_error(error_code)


def _tcp_world_xyz(frame_world_transforms: list[np.ndarray]) -> XYZ3 | None:
    if not frame_world_transforms:
        return None
    tcp_transform = np.array(frame_world_transforms[-1], dtype=float)
    if tcp_transform.shape != (4, 4):
        return None
    return XYZ3(
        float(tcp_transform[0, 3]),
        float(tcp_transform[1, 3]),
        float(tcp_transform[2, 3]),
    )


def _status_from_sample_error(error_code: TrajectorySampleErrorCode) -> TrajectoryComputationStatus:
    if error_code == TrajectorySampleErrorCode.POINT_UNREACHABLE:
        return TrajectoryComputationStatus.POINT_UNREACHABLE
    if error_code == TrajectorySampleErrorCode.CONFIGURATION_JUMP:
        return TrajectoryComputationStatus.CONFIGURATION_JUMP
    if error_code == TrajectorySampleErrorCode.COLLISION_DETECTED:
        return TrajectoryComputationStatus.COLLISION_DETECTED
    if error_code == TrajectorySampleErrorCode.TCP_WORKSPACE_EXIT:
        return TrajectoryComputationStatus.TCP_WORKSPACE_EXIT
    if error_code == TrajectorySampleErrorCode.SPEED_LIMIT_EXCEEDED:
        return TrajectoryComputationStatus.SPEED_LIMIT_EXCEEDED
    if error_code == TrajectorySampleErrorCode.JERK_LIMIT_EXCEEDED:
        return TrajectoryComputationStatus.JERK_LIMIT_EXCEEDED
    if error_code == TrajectorySampleErrorCode.FORBIDDEN_CONFIGURATION:
        return TrajectoryComputationStatus.FORBIDDEN_CONFIGURATION
    return TrajectoryComputationStatus.SUCCESS


def _refresh_segment_status(segment: SegmentResult) -> None:
    for index, sample in enumerate(segment.samples):
        if sample.error_code == TrajectorySampleErrorCode.NONE:
            continue
        segment.status = _status_from_sample_error(sample.error_code)
        segment.first_error_sample_index = index
        segment.first_error_axis = sample.error_axis
        return

    if segment.status in _VALIDITY_STATUSES:
        segment.status = TrajectoryComputationStatus.SUCCESS
        segment.first_error_sample_index = None
        segment.first_error_axis = None


def _refresh_trajectory_status(trajectory: TrajectoryResult) -> None:
    for index, segment in enumerate(trajectory.segments):
        if segment.status == TrajectoryComputationStatus.SUCCESS:
            continue
        trajectory.status = segment.status
        trajectory.first_error_segment_index = index
        return

    trajectory.status = TrajectoryComputationStatus.SUCCESS
    trajectory.first_error_segment_index = None


_VALIDITY_ERROR_CODES = {
    TrajectorySampleErrorCode.COLLISION_DETECTED,
    TrajectorySampleErrorCode.TCP_WORKSPACE_EXIT,
}

_VALIDITY_STATUSES = {
    TrajectoryComputationStatus.COLLISION_DETECTED,
    TrajectoryComputationStatus.TCP_WORKSPACE_EXIT,
}
