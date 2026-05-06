from __future__ import annotations

from collections import defaultdict

from PyQt6.QtCore import QObject, QThread, pyqtSignal

from trajectory_engine.models import BuildCancelToken, ValidationResult, ValidationTask
from trajectory_engine.workers.validity_worker import ValidityWorker


class _WorkerDispatchProxy(QObject):
    dispatch = pyqtSignal(object, object)


class ValidityAnalyzerManager(QObject):
    result_ready = pyqtSignal(int, object)
    task_failed = pyqtSignal(int, int, str)

    def __init__(self, pool_size: int = 1, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._pool_size = max(1, int(pool_size))
        self._threads: list[QThread] = []
        self._workers: list[ValidityWorker] = []
        self._dispatchers: list[_WorkerDispatchProxy] = []
        self._task_tokens: dict[int, BuildCancelToken] = {}
        self._revision_tasks: dict[int, set[int]] = defaultdict(set)
        self._task_to_revision: dict[int, int] = {}
        self._next_worker_index = 0

        for _index in range(self._pool_size):
            thread = QThread(self)
            worker = ValidityWorker()
            dispatcher = _WorkerDispatchProxy(self)
            worker.moveToThread(thread)
            dispatcher.dispatch.connect(worker.process)
            worker.completed.connect(self._on_worker_completed)
            worker.cancelled.connect(self._on_worker_cancelled)
            worker.failed.connect(self._on_worker_failed)
            thread.start()
            self._threads.append(thread)
            self._workers.append(worker)
            self._dispatchers.append(dispatcher)

    def submit_task(self, task: ValidationTask) -> None:
        token = BuildCancelToken()
        self._task_tokens[task.task_id] = token
        self._revision_tasks[task.revision_id].add(task.task_id)
        self._task_to_revision[task.task_id] = task.revision_id
        worker_index = self._next_worker_index % len(self._dispatchers)
        self._next_worker_index += 1
        self._dispatchers[worker_index].dispatch.emit(task, token)

    def cancel_revision(self, revision_id: int) -> None:
        task_ids = list(self._revision_tasks.get(int(revision_id), set()))
        for task_id in task_ids:
            token = self._task_tokens.get(task_id)
            if token is not None:
                token.request_cancel()

    def shutdown(self) -> None:
        for token in list(self._task_tokens.values()):
            token.request_cancel()
        for thread in self._threads:
            thread.quit()
            thread.wait(2000)

    def _consume_task(self, revision_id: int, task_id: int) -> bool:
        task_revision = self._task_to_revision.get(task_id)
        if task_revision != revision_id:
            return False
        self._task_tokens.pop(task_id, None)
        self._task_to_revision.pop(task_id, None)
        revision_tasks = self._revision_tasks.get(revision_id)
        if revision_tasks is not None:
            revision_tasks.discard(task_id)
            if not revision_tasks:
                self._revision_tasks.pop(revision_id, None)
        return True

    def _on_worker_completed(self, revision_id: int, payload: object) -> None:
        if not isinstance(payload, ValidationResult):
            return
        if not self._consume_task(revision_id, payload.task_id):
            return
        self.result_ready.emit(revision_id, payload)

    def _on_worker_cancelled(self, revision_id: int, task_id: int) -> None:
        self._consume_task(revision_id, task_id)

    def _on_worker_failed(self, revision_id: int, task_id: int, message: str) -> None:
        self._consume_task(revision_id, task_id)
        self.task_failed.emit(revision_id, task_id, message)
