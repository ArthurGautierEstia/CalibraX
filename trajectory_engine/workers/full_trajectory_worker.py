from __future__ import annotations

import time

from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot

from trajectory_engine.core.full_builder import TrajectoryBuilder
from trajectory_engine.models.pipeline import (
    BuildCancelToken,
    BuildStatus,
    TrajectoryBuildRequest,
    TrajectoryResult,
    TrajectorySegment,
)


class FullTrajectoryWorker(QObject):
    completed = pyqtSignal(int, object)
    cancelled = pyqtSignal(int)
    failed = pyqtSignal(int, str)
    benchmark_finished = pyqtSignal(int, str, float)

    def __init__(self, builder: TrajectoryBuilder, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._builder = builder

    @pyqtSlot(object, object)
    def process(self, request: object, cancel_token: object) -> None:
        if not isinstance(request, TrajectoryBuildRequest) or not isinstance(cancel_token, BuildCancelToken):
            return
        start_s = time.perf_counter()
        status = "failed"
        try:
            self._builder.set_cancel_token(cancel_token)
            self._builder.set_jerk_check_enabled(request.jerk_check_enabled)
            self._builder.set_cartesian_dynamic_limits(
                request.cartesian_accel_limit_mm_s2,
                request.cartesian_jerk_limit_mm_s3,
            )

            if cancel_token.is_cancelled():
                status = "cancelled"
                self.benchmark_finished.emit(request.revision_id, status, time.perf_counter() - start_s)
                self.cancelled.emit(request.revision_id)
                return

            if not request.keypoints:
                result = TrajectoryResult(build_status=BuildStatus.COMPLETED, revision_id=request.revision_id)
                status = "completed"
                self.benchmark_finished.emit(request.revision_id, status, time.perf_counter() - start_s)
                self.completed.emit(request.revision_id, result)
                return

            if len(request.keypoints) == 1:
                result = TrajectoryResult(build_status=BuildStatus.RUNNING, revision_id=request.revision_id)
                first_segment = self._builder.compute_first_segment(
                    request.current_joints.to_list(),
                    request.keypoints[0],
                    0.0,
                )
                result.segments.append(first_segment)
                if first_segment.status != result.status:
                    result.status = first_segment.status
                    result.first_error_segment_index = 0
                result.build_status = BuildStatus.CANCELLED if cancel_token.is_cancelled() else BuildStatus.COMPLETED
            else:
                segments = [
                    TrajectorySegment(request.keypoints[i], request.keypoints[i + 1])
                    for i in range(len(request.keypoints) - 1)
                ]
                result = self._builder.compute_trajectory(request.current_joints.to_list(), segments)
                result.revision_id = request.revision_id

            if cancel_token.is_cancelled() or result.build_status == BuildStatus.CANCELLED:
                status = "cancelled"
                self.benchmark_finished.emit(request.revision_id, status, time.perf_counter() - start_s)
                self.cancelled.emit(request.revision_id)
                return
            status = "completed"
            self.benchmark_finished.emit(request.revision_id, status, time.perf_counter() - start_s)
            self.completed.emit(request.revision_id, result)
        except Exception as exc:
            self.benchmark_finished.emit(request.revision_id, status, time.perf_counter() - start_s)
            self.failed.emit(request.revision_id, str(exc))
        finally:
            self._builder.set_cancel_token(None)
