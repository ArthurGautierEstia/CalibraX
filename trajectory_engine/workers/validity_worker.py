from __future__ import annotations

import time

from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot

from trajectory_engine.core.validity_analyzer import ValidityAnalyzer
from trajectory_engine.models.pipeline import BuildCancelToken, ValidationTask


class ValidityWorker(QObject):
    completed = pyqtSignal(int, object)
    cancelled = pyqtSignal(int, int)
    failed = pyqtSignal(int, int, str)
    task_started = pyqtSignal(int, int, int, float)
    task_finished = pyqtSignal(int, int, int, str, float, float)

    def __init__(self, worker_index: int, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._worker_index = int(worker_index)

    @pyqtSlot(object, object)
    def process(self, task: object, cancel_token: object) -> None:
        if not isinstance(task, ValidationTask) or not isinstance(cancel_token, BuildCancelToken):
            return
        start_s = time.perf_counter()
        self.task_started.emit(task.revision_id, task.task_id, self._worker_index, start_s)
        try:
            analyzer = ValidityAnalyzer(task.context)
            result = analyzer.analyze_task(task, cancel_token)
            if result.cancelled or cancel_token.is_cancelled():
                finish_s = time.perf_counter()
                self.task_finished.emit(
                    task.revision_id,
                    task.task_id,
                    self._worker_index,
                    "cancelled",
                    finish_s - start_s,
                    finish_s,
                )
                self.cancelled.emit(task.revision_id, task.task_id)
                return
            finish_s = time.perf_counter()
            self.task_finished.emit(
                task.revision_id,
                task.task_id,
                self._worker_index,
                "completed",
                finish_s - start_s,
                finish_s,
            )
            self.completed.emit(task.revision_id, result)
        except Exception as exc:
            finish_s = time.perf_counter()
            self.task_finished.emit(
                task.revision_id,
                task.task_id,
                self._worker_index,
                "failed",
                finish_s - start_s,
                finish_s,
            )
            self.failed.emit(task.revision_id, task.task_id, str(exc))
