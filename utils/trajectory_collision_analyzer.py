from __future__ import annotations

from dataclasses import dataclass
import math
import threading

import numpy as np
from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot

from models.primitive_collider_models import PrimitiveColliderData, RobotAxisColliderData
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
from models.types import Pose6
from models.workspace_model import WorkspaceModel
from utils.collision_utils import (
    CollisionPair,
    CollisionWorldCache,
    build_world_frame_transforms,
    resolve_flange_world_transform,
)
import utils.math_utils as math_utils


@dataclass
class TrajectoryCollisionSampleResult:
    segment_index: int
    sample_index: int
    collisions: list[TrajectoryCollisionDiagnostic]


@dataclass
class TrajectoryCollisionAnalysisResult:
    job_id: int
    cancelled: bool
    has_collision: bool
    sample_results: list[TrajectoryCollisionSampleResult]


class TrajectoryCollisionCancelToken:
    def __init__(self) -> None:
        self._event = threading.Event()

    def request_cancel(self) -> None:
        self._event.set()

    def is_cancelled(self) -> bool:
        return self._event.is_set()


@dataclass
class TrajectoryCollisionKinematicsSnapshot:
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
    ) -> "TrajectoryCollisionKinematicsSnapshot":
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
class TrajectoryCollisionContext:
    kinematics: TrajectoryCollisionKinematicsSnapshot
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
    ) -> "TrajectoryCollisionContext":
        return cls(
            kinematics=TrajectoryCollisionKinematicsSnapshot.from_robot_model(robot_model, tool_model),
            workspace_collision_zones=workspace_model.get_workspace_collision_zones(),
            robot_axis_colliders=robot_model.get_axis_collider_data(),
            tool_colliders=tool_model.get_tool_collider_data(),
            evaluated_robot_axis_colliders=tool_model.get_evaluated_robot_axis_colliders(),
            robot_base_transform_world=np.array(
                workspace_model.get_robot_base_transform_world().matrix,
                dtype=float,
            ),
        )


class TrajectoryCollisionAnalyzer:
    def __init__(self, context: TrajectoryCollisionContext) -> None:
        self.context = context
        self._collision_cache = CollisionWorldCache()
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

    def analyze_sample(self, sample: TrajectorySample) -> list[TrajectoryCollisionDiagnostic]:
        if sample.error_code != TrajectorySampleErrorCode.NONE:
            return []
        if not sample.reachable:
            return []

        corrected_matrices = None
        if sample.kinematics is not None:
            corrected_matrices = sample.kinematics.corrected_matrices
        if corrected_matrices is None:
            corrected_matrices = self.context.kinematics.compute_corrected_matrices(sample.joints)
        if corrected_matrices is None:
            return []

        frame_world_transforms = build_world_frame_transforms(
            corrected_matrices,
            self.context.robot_base_transform_world,
        )
        flange_world_transform = resolve_flange_world_transform(frame_world_transforms)
        self._collision_cache.update_dynamic_world_shapes(frame_world_transforms, flange_world_transform)

        diagnostics = TrajectoryCollisionAnalyzer._diagnostics_from_pairs(
            self._collision_cache.find_workspace_collisions(),
            TrajectoryCollisionDomain.WORKSPACE,
        )
        diagnostics.extend(
            TrajectoryCollisionAnalyzer._diagnostics_from_pairs(
                self._collision_cache.find_robot_tool_collisions(
                    self.context.evaluated_robot_axis_colliders
                ),
                TrajectoryCollisionDomain.ROBOT_TOOL,
            )
        )
        return diagnostics

    def analyze_trajectory(
        self,
        job_id: int,
        trajectory: TrajectoryResult,
        cancel_token: TrajectoryCollisionCancelToken,
    ) -> TrajectoryCollisionAnalysisResult:
        sample_results: list[TrajectoryCollisionSampleResult] = []
        for segment_index, segment in enumerate(trajectory.segments):
            for sample_index, sample in enumerate(segment.samples):
                if cancel_token.is_cancelled():
                    return TrajectoryCollisionAnalysisResult(
                        job_id=job_id,
                        cancelled=True,
                        has_collision=False,
                        sample_results=[],
                    )
                collisions = self.analyze_sample(sample)
                if collisions:
                    sample_results.append(
                        TrajectoryCollisionSampleResult(
                            segment_index=segment_index,
                            sample_index=sample_index,
                            collisions=collisions,
                        )
                    )

        return TrajectoryCollisionAnalysisResult(
            job_id=job_id,
            cancelled=False,
            has_collision=bool(sample_results),
            sample_results=sample_results,
        )


class TrajectoryCollisionWorker(QObject):
    completed = pyqtSignal(int, object)
    cancelled = pyqtSignal(int)
    finished = pyqtSignal()

    def __init__(
        self,
        job_id: int,
        trajectory: TrajectoryResult,
        context: TrajectoryCollisionContext,
        cancel_token: TrajectoryCollisionCancelToken,
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
            analyzer = TrajectoryCollisionAnalyzer(self.context)
            result = analyzer.analyze_trajectory(self.job_id, self.trajectory, self.cancel_token)
            if result.cancelled:
                self.cancelled.emit(self.job_id)
                return
            self.completed.emit(self.job_id, result)
        finally:
            self.finished.emit()


def apply_trajectory_collision_result(
    trajectory: TrajectoryResult,
    result: TrajectoryCollisionAnalysisResult,
) -> bool:
    if result.cancelled:
        return False

    applied_collision = False
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
        sample.error_code = TrajectorySampleErrorCode.COLLISION_DETECTED
        sample.error_axis = None
        _mark_segment_collision(segment, sample_result.sample_index)
        applied_collision = True

        if trajectory.status == TrajectoryComputationStatus.SUCCESS:
            trajectory.status = TrajectoryComputationStatus.COLLISION_DETECTED
            trajectory.first_error_segment_index = sample_result.segment_index

    return applied_collision


def _segment_at(trajectory: TrajectoryResult, segment_index: int) -> SegmentResult | None:
    if segment_index < 0 or segment_index >= len(trajectory.segments):
        return None
    return trajectory.segments[segment_index]


def _sample_at(segment: SegmentResult, sample_index: int) -> TrajectorySample | None:
    if sample_index < 0 or sample_index >= len(segment.samples):
        return None
    return segment.samples[sample_index]


def _mark_segment_collision(segment: SegmentResult, sample_index: int) -> None:
    if segment.first_error_sample_index is None:
        segment.first_error_sample_index = int(sample_index)
        segment.first_error_axis = None
    if segment.status == TrajectoryComputationStatus.SUCCESS:
        segment.status = TrajectoryComputationStatus.COLLISION_DETECTED
