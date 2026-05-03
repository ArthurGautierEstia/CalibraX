from __future__ import annotations

from dataclasses import dataclass

from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot

from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.trajectory_keypoint import TrajectoryKeypoint
from models.trajectory_options import TrajectoryBezierDegree
from models.trajectory_result import TrajectoryBuilderBehavior, TrajectoryResult, TrajectorySegment
from models.types import JointAngles6
from models.workspace_model import WorkspaceModel
from utils.trajectory_builder import TrajectoryBuilder
from utils.trajectory_validity_analyzer import (
    TrajectoryValidityAnalyzer,
    TrajectoryValidityAnalysisResult,
    TrajectoryValidityCancelToken,
    TrajectoryValidityContext,
    apply_trajectory_validity_result,
)


@dataclass
class TrajectoryFullAnalysisResult:
    job_id: int
    trajectory: TrajectoryResult
    validity: TrajectoryValidityAnalysisResult


class TrajectoryFullAnalysisWorker(QObject):
    completed = pyqtSignal(int, object)
    cancelled = pyqtSignal(int)
    finished = pyqtSignal()

    def __init__(
        self,
        job_id: int,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        current_joints: JointAngles6,
        keypoints: list[TrajectoryKeypoint],
        context: TrajectoryValidityContext,
        cancel_token: TrajectoryValidityCancelToken,
        sample_dt_s: float,
        smooth_time_enabled: bool,
        bezier_degree: TrajectoryBezierDegree,
        jerk_check_enabled: bool,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.job_id = int(job_id)
        self.robot_model = robot_model
        self.tool_model = tool_model
        self.workspace_model = workspace_model
        self.current_joints = current_joints.copy()
        self.keypoints = [keypoint.clone() for keypoint in keypoints]
        self.context = context
        self.cancel_token = cancel_token
        self.sample_dt_s = float(sample_dt_s)
        self.smooth_time_enabled = bool(smooth_time_enabled)
        self.bezier_degree = TrajectoryBezierDegree.from_value(bezier_degree)
        self.jerk_check_enabled = bool(jerk_check_enabled)

    def request_cancel(self) -> None:
        self.cancel_token.request_cancel()

    @staticmethod
    def _build_segments(keypoints: list[TrajectoryKeypoint]) -> list[TrajectorySegment]:
        if len(keypoints) < 2:
            return []
        return [TrajectorySegment(keypoints[i], keypoints[i + 1]) for i in range(len(keypoints) - 1)]

    def _compute_trajectory(self) -> TrajectoryResult:
        builder = TrajectoryBuilder(
            self.robot_model,
            self.tool_model,
            self.workspace_model,
            behavior=TrajectoryBuilderBehavior.CONTINUE_ON_ERROR,
            sample_dt_s=self.sample_dt_s,
            smooth_time_enabled=self.smooth_time_enabled,
            bezier_degree=self.bezier_degree,
            jerk_check_enabled=self.jerk_check_enabled,
        )
        if not self.keypoints:
            return TrajectoryResult()
        if len(self.keypoints) == 1:
            result = TrajectoryResult()
            first_segment = builder.compute_first_segment(self.current_joints.to_list(), self.keypoints[0], 0.0)
            result.segments.append(first_segment)
            if first_segment.status != result.status:
                result.status = first_segment.status
                result.first_error_segment_index = 0
            return result
        return builder.compute_trajectory(self.current_joints.to_list(), self._build_segments(self.keypoints))

    @pyqtSlot()
    def run(self) -> None:
        try:
            if self.cancel_token.is_cancelled():
                self.cancelled.emit(self.job_id)
                return

            trajectory = self._compute_trajectory()
            if self.cancel_token.is_cancelled():
                self.cancelled.emit(self.job_id)
                return

            analyzer = TrajectoryValidityAnalyzer(self.context)
            validity = analyzer.analyze_trajectory(self.job_id, trajectory, self.cancel_token)
            if validity.cancelled or self.cancel_token.is_cancelled():
                self.cancelled.emit(self.job_id)
                return
            apply_trajectory_validity_result(trajectory, validity)
            self.completed.emit(self.job_id, TrajectoryFullAnalysisResult(self.job_id, trajectory, validity))
        finally:
            self.finished.emit()
