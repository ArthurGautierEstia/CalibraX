from __future__ import annotations

from PyQt6.QtCore import QObject, QThread, QTimer, pyqtSignal

from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.workspace_model import WorkspaceModel
from trajectory_engine.core.chunking import build_validation_task_samples
from trajectory_engine.v2.builders.full_builder import TrajectoryBuilderV2
from trajectory_engine.v2.builders.preview_builder import TrajectoryPreviewBuilderV2
from trajectory_engine.core.validity_analyzer import (
    apply_validation_result,
    build_validity_context_snapshot,
    prepare_trajectory_validity_analysis,
)
from trajectory_engine.managers.validity_analyzer_manager import ValidityAnalyzerManager
from trajectory_engine.models import (
    BuildCancelToken,
    BuildStatus,
    TrajectoryBuildRequest,
    TrajectoryBuildTriggerMode,
    TrajectoryPreviewResult,
    TrajectoryResult,
    ValidationTask,
)
from trajectory_engine.workers.full_trajectory_worker import FullTrajectoryWorker
from trajectory_engine.workers.preview_worker import PreviewWorker


class TrajectoryBuildManager(QObject):
    preview_ready = pyqtSignal(int, object)
    result_ready = pyqtSignal(int, object)
    build_failed = pyqtSignal(int, str, str)

    _dispatch_preview = pyqtSignal(object, object)
    _dispatch_full = pyqtSignal(object, object)

    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        debounce_ms: int = 120,
        validity_pool_size: int = 1,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.robot_model = robot_model
        self.tool_model = tool_model
        self.workspace_model = workspace_model
        self._revision_sequence = 0
        self._active_revision_id = 0
        self._active_request: TrajectoryBuildRequest | None = None
        self._preview_token: BuildCancelToken | None = None
        self._full_token: BuildCancelToken | None = None
        self._computation_by_revision: dict[int, TrajectoryResult] = {}
        self._expected_task_ids_by_revision: dict[int, set[int]] = {}
        self._completed_task_ids_by_revision: dict[int, set[int]] = {}
        self._task_sequence = 0
        self._shutdown_requested = False

        self._preview_thread = QThread(self)
        self._preview_worker = PreviewWorker(
            TrajectoryPreviewBuilderV2(robot_model, tool_model, workspace_model),
        )
        self._preview_worker.moveToThread(self._preview_thread)
        self._dispatch_preview.connect(self._preview_worker.process)
        self._preview_worker.completed.connect(self._on_preview_completed)
        self._preview_worker.cancelled.connect(self._on_preview_cancelled)
        self._preview_worker.failed.connect(
            lambda revision_id, message: self._on_worker_failed(revision_id, "preview", message)
        )
        self._preview_thread.start()

        self._full_thread = QThread(self)
        self._full_worker = FullTrajectoryWorker(
            TrajectoryBuilderV2(robot_model, tool_model, workspace_model),
        )
        self._full_worker.moveToThread(self._full_thread)
        self._dispatch_full.connect(self._full_worker.process)
        self._full_worker.completed.connect(self._on_full_completed)
        self._full_worker.cancelled.connect(self._on_full_cancelled)
        self._full_worker.failed.connect(
            lambda revision_id, message: self._on_worker_failed(revision_id, "full", message)
        )
        self._full_thread.start()

        self._validity_manager = ValidityAnalyzerManager(pool_size=validity_pool_size, parent=self)
        self._validity_manager.result_ready.connect(self._on_validation_result_ready)
        self._validity_manager.task_failed.connect(self._on_validation_task_failed)

        self._full_debounce_timer = QTimer(self)
        self._full_debounce_timer.setSingleShot(True)
        self._full_debounce_timer.setInterval(max(1, int(debounce_ms)))
        self._full_debounce_timer.timeout.connect(self._submit_debounced_full_build)

    def submit(self, request: TrajectoryBuildRequest) -> int:
        if self._shutdown_requested:
            return 0
        previous_revision_id = self._active_revision_id
        self._revision_sequence += 1
        revision_id = self._revision_sequence
        normalized_request = TrajectoryBuildRequest(
            revision_id=revision_id,
            current_joints=request.current_joints.copy(),
            keypoints=[keypoint.clone() for keypoint in request.keypoints],
            sample_dt_s=float(request.sample_dt_s),
            smooth_time_enabled=bool(request.smooth_time_enabled),
            bezier_degree=request.bezier_degree,
            jerk_check_enabled=bool(request.jerk_check_enabled),
            cartesian_accel_limit_mm_s2=float(request.cartesian_accel_limit_mm_s2),
            cartesian_jerk_limit_mm_s3=float(request.cartesian_jerk_limit_mm_s3),
            trigger_mode=request.trigger_mode,
        )
        self._cancel_previous_work(previous_revision_id)

        self._active_revision_id = revision_id
        self._active_request = normalized_request
        self._preview_token = BuildCancelToken()
        self._dispatch_preview.emit(normalized_request, self._preview_token)

        if normalized_request.trigger_mode == TrajectoryBuildTriggerMode.FORCED_FULL:
            self._submit_full_build(normalized_request)
        else:
            self._full_debounce_timer.start()
        return revision_id

    def cancel_active(self) -> None:
        active_revision_id = self._active_revision_id
        self._cancel_previous_work(active_revision_id)
        self._active_revision_id = 0
        self._active_request = None

    def shutdown(self) -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self.cancel_active()
        self._validity_manager.shutdown()
        self._preview_thread.quit()
        self._preview_thread.wait()
        self._full_thread.quit()
        self._full_thread.wait()

    def _cancel_previous_work(self, revision_id: int) -> None:
        self._full_debounce_timer.stop()
        if self._preview_token is not None:
            self._preview_token.request_cancel()
            self._preview_token = None
        if self._full_token is not None:
            self._full_token.request_cancel()
            self._full_token = None
        if revision_id > 0:
            self._validity_manager.cancel_revision(revision_id)
            self._expected_task_ids_by_revision.pop(revision_id, None)
            self._completed_task_ids_by_revision.pop(revision_id, None)
            self._computation_by_revision.pop(revision_id, None)

    def _submit_debounced_full_build(self) -> None:
        if self._active_request is None:
            return
        self._submit_full_build(self._active_request)

    def _submit_full_build(self, request: TrajectoryBuildRequest) -> None:
        if self._full_token is not None:
            self._full_token.request_cancel()
        self._full_token = BuildCancelToken()
        self._dispatch_full.emit(request, self._full_token)

    def _on_preview_completed(self, revision_id: int, payload: object) -> None:
        if revision_id != self._active_revision_id:
            return
        if not isinstance(payload, TrajectoryPreviewResult):
            return
        self.preview_ready.emit(revision_id, payload)

    def _on_preview_cancelled(self, _revision_id: int) -> None:
        return

    def _on_full_cancelled(self, _revision_id: int) -> None:
        return

    def _on_full_completed(self, revision_id: int, payload: object) -> None:
        if revision_id != self._active_revision_id:
            return
        if not isinstance(payload, TrajectoryResult):
            return
        if payload.build_status == BuildStatus.CANCELLED:
            return
        prepare_trajectory_validity_analysis(payload)
        self._computation_by_revision[revision_id] = payload
        task_samples = build_validation_task_samples(payload)
        if not task_samples:
            self.result_ready.emit(revision_id, payload)
            self._computation_by_revision.pop(revision_id, None)
            return

        context = build_validity_context_snapshot(self.robot_model, self.tool_model, self.workspace_model)
        expected_task_ids: set[int] = set()
        self._completed_task_ids_by_revision[revision_id] = set()
        chunk_size = 128
        for start in range(0, len(task_samples), chunk_size):
            chunk = task_samples[start : start + chunk_size]
            self._task_sequence += 1
            task_id = self._task_sequence
            expected_task_ids.add(task_id)
            self._validity_manager.submit_task(
                ValidationTask(
                    revision_id=revision_id,
                    task_id=task_id,
                    samples=chunk,
                    context=context,
                    start_index=start,
                    end_index_exclusive=start + len(chunk),
                )
            )
        self._expected_task_ids_by_revision[revision_id] = expected_task_ids

    def _on_validation_result_ready(self, revision_id: int, payload: object) -> None:
        if revision_id != self._active_revision_id:
            return
        if revision_id not in self._computation_by_revision:
            return
        if not hasattr(payload, "task_id"):
            return
        computation = self._computation_by_revision[revision_id]
        apply_validation_result(computation, payload)
        completed = self._completed_task_ids_by_revision.setdefault(revision_id, set())
        completed.add(int(payload.task_id))
        expected = self._expected_task_ids_by_revision.get(revision_id, set())
        if expected and completed >= expected:
            self.result_ready.emit(revision_id, computation)
            self._expected_task_ids_by_revision.pop(revision_id, None)
            self._completed_task_ids_by_revision.pop(revision_id, None)
            self._computation_by_revision.pop(revision_id, None)

    def _on_worker_failed(self, revision_id: int, stage: str, message: str) -> None:
        if revision_id != self._active_revision_id:
            return
        self.build_failed.emit(revision_id, stage, message)

    def _on_validation_task_failed(self, revision_id: int, task_id: int, message: str) -> None:
        if revision_id != self._active_revision_id:
            return
        self.build_failed.emit(revision_id, "validation", f"validation task {task_id} failed: {message}")
