from __future__ import annotations

from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot

from trajectory_engine.core.preview_builder import TrajectoryPreviewBuilder
from trajectory_engine.models.pipeline import (
    BuildCancelToken,
    BuildStatus,
    TrajectoryBuildRequest,
    TrajectoryPreviewResult,
)


class PreviewWorker(QObject):
    completed = pyqtSignal(int, object)
    cancelled = pyqtSignal(int)
    failed = pyqtSignal(int, str)

    def __init__(self, builder: TrajectoryPreviewBuilder, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._builder = builder

    @pyqtSlot(object, object)
    def process(self, request: object, cancel_token: object) -> None:
        if not isinstance(request, TrajectoryBuildRequest) or not isinstance(cancel_token, BuildCancelToken):
            return
        try:
            self._builder.set_cancel_token(cancel_token)
            self._builder.set_jerk_check_enabled(request.jerk_check_enabled)
            self._builder.set_cartesian_dynamic_limits(
                request.cartesian_accel_limit_mm_s2,
                request.cartesian_jerk_limit_mm_s3,
            )
            if cancel_token.is_cancelled():
                self.cancelled.emit(request.revision_id)
                return

            if not request.keypoints:
                result = TrajectoryPreviewResult(revision_id=request.revision_id, build_status=BuildStatus.COMPLETED)
                self.completed.emit(request.revision_id, result)
                return

            segments = []
            if len(request.keypoints) >= 2:
                from trajectory_engine.models.pipeline import TrajectorySegment

                segments = [
                    TrajectorySegment(request.keypoints[i], request.keypoints[i + 1])
                    for i in range(len(request.keypoints) - 1)
                ]

            if len(request.keypoints) == 1:
                result = TrajectoryPreviewResult(revision_id=request.revision_id, build_status=BuildStatus.RUNNING)
                first_preview = self._builder.compute_first_preview_segment(request.current_joints, request.keypoints[0], 0.0)
                result.segments.append(first_preview)
                result.status = first_preview.status
                result.first_error_segment_index = 0 if first_preview.status != result.status else None
                result.analysis_pending = bool(first_preview.samples)
                result.build_status = BuildStatus.CANCELLED if cancel_token.is_cancelled() else BuildStatus.COMPLETED
            else:
                result = self._builder.compute_preview(request.current_joints, segments)
                result.revision_id = request.revision_id

            if cancel_token.is_cancelled() or result.build_status == BuildStatus.CANCELLED:
                self.cancelled.emit(request.revision_id)
                return
            self.completed.emit(request.revision_id, result)
        except Exception as exc:
            self.failed.emit(request.revision_id, str(exc))
        finally:
            self._builder.set_cancel_token(None)
