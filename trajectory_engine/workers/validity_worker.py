from __future__ import annotations

from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot

from trajectory_engine.core.validity_analyzer import ValidityAnalyzer
from trajectory_engine.models import BuildCancelToken, ValidationTask


class ValidityWorker(QObject):
    completed = pyqtSignal(int, object)
    cancelled = pyqtSignal(int, int)
    failed = pyqtSignal(int, int, str)

    @pyqtSlot(object, object)
    def process(self, task: object, cancel_token: object) -> None:
        if not isinstance(task, ValidationTask) or not isinstance(cancel_token, BuildCancelToken):
            return
        try:
            analyzer = ValidityAnalyzer(task.context)
            result = analyzer.analyze_task(task, cancel_token)
            if result.cancelled or cancel_token.is_cancelled():
                self.cancelled.emit(task.revision_id, task.task_id)
                return
            self.completed.emit(task.revision_id, result)
        except Exception as exc:
            self.failed.emit(task.revision_id, task.task_id, str(exc))
