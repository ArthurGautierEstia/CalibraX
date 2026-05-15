from __future__ import annotations

from dataclasses import dataclass, field
import time

from PyQt6.QtCore import QObject, QThread, QTimer, pyqtSignal

from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.workspace_model import WorkspaceModel
from trajectory_engine.core.chunking import build_validation_task_samples
from trajectory_engine.core.full_builder import TrajectoryBuilder
from trajectory_engine.core.preview_builder import TrajectoryPreviewBuilder
from trajectory_engine.core.validity_analyzer import (
    apply_validation_result,
    build_validity_context_snapshot,
    prepare_trajectory_validity_analysis,
)
from trajectory_engine.managers.validity_analyzer_manager import ValidityAnalyzerManager
from trajectory_engine.models.pipeline import (
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


@dataclass
class _BuildBenchmarkSession:
    revision_id: int
    trigger_mode: TrajectoryBuildTriggerMode
    start_s: float
    validation_pool_size: int
    keypoint_count: int
    full_worker_elapsed_s: float | None = None
    full_worker_status: str = ""
    sample_count: int = 0
    validation_task_count: int = 0
    validation_first_start_s: float | None = None
    validation_last_finish_s: float | None = None
    validation_task_elapsed_total_s: float = 0.0
    validation_tasks_started: int = 0
    validation_tasks_finished: int = 0
    validators_worked: set[int] = field(default_factory=set)


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
        verbose_logging: bool = False,
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
        self._verbose_logging = bool(verbose_logging)
        self._benchmark_by_revision: dict[int, _BuildBenchmarkSession] = {}

        self._preview_thread = QThread(self)
        self._preview_worker = PreviewWorker(
            TrajectoryPreviewBuilder(robot_model, tool_model, workspace_model),
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
            TrajectoryBuilder(robot_model, tool_model, workspace_model),
        )
        self._full_worker.moveToThread(self._full_thread)
        self._dispatch_full.connect(self._full_worker.process)
        self._full_worker.benchmark_finished.connect(self._on_full_worker_benchmark_finished)
        self._full_worker.completed.connect(self._on_full_completed)
        self._full_worker.cancelled.connect(self._on_full_cancelled)
        self._full_worker.failed.connect(
            lambda revision_id, message: self._on_worker_failed(revision_id, "full", message)
        )
        self._full_thread.start()

        self._validity_manager = ValidityAnalyzerManager(pool_size=validity_pool_size, parent=self)
        self._validity_manager.task_started.connect(self._on_validation_task_started)
        self._validity_manager.task_finished.connect(self._on_validation_task_finished)
        self._validity_manager.result_ready.connect(self._on_validation_result_ready)
        self._validity_manager.task_failed.connect(self._on_validation_task_failed)

        self._full_debounce_timer = QTimer(self)
        self._full_debounce_timer.setSingleShot(True)
        self._full_debounce_timer.setInterval(max(1, int(debounce_ms)))
        self._full_debounce_timer.timeout.connect(self._submit_debounced_full_build)

    def set_verbose_logging(self, enabled: bool) -> None:
        self._verbose_logging = bool(enabled)

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
            self._finish_benchmark_session(revision_id, "cancelled")
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
        self._start_benchmark_session(request)
        self._dispatch_full.emit(request, self._full_token)

    @staticmethod
    def _sample_count(result: TrajectoryResult) -> int:
        count = 0
        for segment in result.segments:
            count += len(segment.samples)
        return count

    @staticmethod
    def _ms(duration_s: float | None) -> str:
        if duration_s is None:
            return "n/a"
        return f"{duration_s * 1000.0:.3f} ms"

    def _log_benchmark(self, message: str) -> None:
        if not self._verbose_logging:
            return
        print(f"[trajectory-benchmark] {message}", flush=True)

    def _start_benchmark_session(self, request: TrajectoryBuildRequest) -> None:
        if not self._verbose_logging:
            return
        session = _BuildBenchmarkSession(
            revision_id=request.revision_id,
            trigger_mode=request.trigger_mode,
            start_s=time.perf_counter(),
            validation_pool_size=self._validity_manager.pool_size(),
            keypoint_count=len(request.keypoints),
        )
        self._benchmark_by_revision[request.revision_id] = session
        self._log_benchmark(
            "start "
            f"revision={request.revision_id} "
            f"trigger={request.trigger_mode.value} "
            f"keypoints={len(request.keypoints)} "
            f"validators_pool={session.validation_pool_size}"
        )

    def _finish_benchmark_session(self, revision_id: int, status: str) -> None:
        session = self._benchmark_by_revision.pop(revision_id, None)
        if session is None:
            return
        validation_elapsed_s = None
        if session.validation_first_start_s is not None and session.validation_last_finish_s is not None:
            validation_elapsed_s = session.validation_last_finish_s - session.validation_first_start_s
        total_elapsed_s = time.perf_counter() - session.start_s
        self._log_benchmark(
            "finish "
            f"revision={revision_id} "
            f"status={status} "
            f"trigger={session.trigger_mode.value} "
            f"keypoints={session.keypoint_count} "
            f"samples={session.sample_count} "
            f"full_worker={self._ms(session.full_worker_elapsed_s)} "
            f"full_status={session.full_worker_status or 'n/a'} "
            f"validation_elapsed={self._ms(validation_elapsed_s)} "
            f"validation_task_cpu={self._ms(session.validation_task_elapsed_total_s)} "
            f"validation_tasks={session.validation_tasks_finished}/{session.validation_task_count} "
            f"validators_pool={session.validation_pool_size} "
            f"validators_worked={len(session.validators_worked)} "
            f"total={self._ms(total_elapsed_s)}"
        )

    def _on_full_worker_benchmark_finished(self, revision_id: int, status: str, elapsed_s: float) -> None:
        session = self._benchmark_by_revision.get(revision_id)
        if session is None:
            return
        session.full_worker_elapsed_s = float(elapsed_s)
        session.full_worker_status = str(status)
        if status == "cancelled":
            self._finish_benchmark_session(revision_id, "cancelled")

    def _on_validation_task_started(
        self,
        revision_id: int,
        _task_id: int,
        worker_index: int,
        started_s: float,
    ) -> None:
        session = self._benchmark_by_revision.get(revision_id)
        if session is None:
            return
        session.validators_worked.add(int(worker_index))
        session.validation_tasks_started += 1
        if session.validation_first_start_s is None or started_s < session.validation_first_start_s:
            session.validation_first_start_s = float(started_s)

    def _on_validation_task_finished(
        self,
        revision_id: int,
        _task_id: int,
        _worker_index: int,
        _status: str,
        elapsed_s: float,
        finished_s: float,
    ) -> None:
        session = self._benchmark_by_revision.get(revision_id)
        if session is None:
            return
        session.validation_tasks_finished += 1
        session.validation_task_elapsed_total_s += float(elapsed_s)
        if session.validation_last_finish_s is None or finished_s > session.validation_last_finish_s:
            session.validation_last_finish_s = float(finished_s)

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
        session = self._benchmark_by_revision.get(revision_id)
        if session is not None:
            session.sample_count = self._sample_count(payload)
        task_samples = build_validation_task_samples(payload)
        if not task_samples:
            self._finish_benchmark_session(revision_id, "completed")
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
        session = self._benchmark_by_revision.get(revision_id)
        if session is not None:
            session.validation_task_count = len(expected_task_ids)

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
            self._finish_benchmark_session(revision_id, "completed")
            self.result_ready.emit(revision_id, computation)
            self._expected_task_ids_by_revision.pop(revision_id, None)
            self._completed_task_ids_by_revision.pop(revision_id, None)
            self._computation_by_revision.pop(revision_id, None)

    def _on_worker_failed(self, revision_id: int, stage: str, message: str) -> None:
        if revision_id != self._active_revision_id:
            return
        if stage != "preview":
            self._finish_benchmark_session(revision_id, f"{stage}_failed")
        self.build_failed.emit(revision_id, stage, message)

    def _on_validation_task_failed(self, revision_id: int, task_id: int, message: str) -> None:
        if revision_id != self._active_revision_id:
            return
        self._finish_benchmark_session(revision_id, "validation_failed")
        self.build_failed.emit(revision_id, "validation", f"validation task {task_id} failed: {message}")
