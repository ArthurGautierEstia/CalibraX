from __future__ import annotations

import os

from PyQt6.QtCore import QObject, pyqtSignal
from PyQt6.QtWidgets import QFileDialog, QMessageBox

from models.project_file import ProjectFile


class ProjectController(QObject):
    project_changed = pyqtSignal()

    PROJECT_FILTER = "Projet CalibraX (*.calibrax.json);;JSON (*.json);;All files (*)"

    def __init__(self, main_controller, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._main_controller = main_controller
        self._main_window = main_controller.main_window
        self._current_project_file = ""
        self._recent_projects: list[str] = []
        self._is_dirty = False
        self._setup_connections()
        self._refresh_window_state()

    def _setup_connections(self) -> None:
        self._main_window.new_project_requested.connect(self.new_project)
        self._main_window.open_project_requested.connect(self.open_project_dialog)
        self._main_window.open_recent_project_requested.connect(self.load_project_from_path)
        self._main_window.save_project_requested.connect(self.save_project)
        self._main_window.save_project_as_requested.connect(self.save_project_as)

    def get_current_project_file(self) -> str:
        return self._current_project_file

    def get_recent_projects(self) -> list[str]:
        return list(self._recent_projects)

    def set_recent_projects(self, project_paths: list[str]) -> None:
        self._recent_projects = []
        for path in project_paths:
            self._add_recent_project(path)
        self._main_window.set_recent_projects(self._recent_projects)

    def new_project(self) -> None:
        if not self._confirm_discard_unsaved_project():
            return
        self._current_project_file = ""
        self._is_dirty = False
        self._main_controller.reset_project_configurations()
        self._refresh_window_state()
        self.project_changed.emit()

    def open_project_dialog(self) -> None:
        start_dir = self._project_directory()
        file_path, _ = QFileDialog.getOpenFileName(
            self._main_window,
            "Charger un projet",
            start_dir,
            ProjectController.PROJECT_FILTER,
        )
        if file_path:
            self.load_project_from_path(file_path)

    def load_project_from_path(self, file_path: str) -> bool:
        if not self._confirm_discard_unsaved_project():
            return False
        try:
            project_file = ProjectFile.load(file_path)
        except (OSError, ValueError, TypeError) as exc:
            QMessageBox.warning(self._main_window, "Projet invalide", f"Impossible de charger le projet.\n{exc}")
            return False

        project_dir = os.path.dirname(os.path.abspath(file_path))
        if not self._main_controller.load_project_configurations(project_file.configurations, project_dir):
            return False

        self._current_project_file = os.path.abspath(file_path)
        self._is_dirty = False
        self._add_recent_project(self._current_project_file)
        self._refresh_window_state()
        self.project_changed.emit()
        return True

    def save_project(self) -> bool:
        if not self._current_project_file:
            return self.save_project_as()
        return self._save_project_to_path(self._current_project_file)

    def save_project_as(self) -> bool:
        os.makedirs(self._project_directory(), exist_ok=True)
        file_path, _ = QFileDialog.getSaveFileName(
            self._main_window,
            "Enregistrer un projet",
            self._project_directory(),
            ProjectController.PROJECT_FILTER,
        )
        if not file_path:
            return False
        if not file_path.lower().endswith(".calibrax.json"):
            file_path += ".calibrax.json"
        return self._save_project_to_path(file_path)

    def mark_dirty(self) -> None:
        self._is_dirty = True
        self._refresh_window_state()

    def _save_project_to_path(self, file_path: str) -> bool:
        project_path = os.path.abspath(file_path)
        project_name = os.path.splitext(os.path.basename(project_path))[0]
        if project_name.lower().endswith(".calibrax"):
            project_name = project_name[: -len(".calibrax")]
        configurations = self._main_controller.get_project_configuration_paths(
            base_dir=os.path.dirname(project_path)
        )
        try:
            ProjectFile(name=project_name, configurations=configurations).save(project_path)
        except OSError as exc:
            QMessageBox.warning(self._main_window, "Erreur sauvegarde", f"Impossible d'enregistrer le projet.\n{exc}")
            return False

        self._current_project_file = project_path
        self._is_dirty = False
        self._add_recent_project(project_path)
        self._refresh_window_state()
        self.project_changed.emit()
        return True

    def _add_recent_project(self, file_path: str) -> None:
        normalized = os.path.abspath(str(file_path or "").strip())
        if not normalized:
            return
        self._recent_projects = [path for path in self._recent_projects if os.path.abspath(path) != normalized]
        self._recent_projects.insert(0, normalized)
        self._recent_projects = self._recent_projects[:10]
        self._main_window.set_recent_projects(self._recent_projects)

    def _confirm_discard_unsaved_project(self) -> bool:
        if not self._is_dirty:
            return True
        answer = QMessageBox.question(
            self._main_window,
            "Projet modifié",
            "Le projet courant contient des modifications non enregistrées. Continuer ?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        return answer == QMessageBox.StandardButton.Yes

    def _refresh_window_state(self) -> None:
        title = "Calibrax"
        if self._current_project_file:
            title += f" - {os.path.basename(self._current_project_file)}"
        if self._is_dirty:
            title += " *"
        self._main_window.setWindowTitle(title)

    def _project_directory(self) -> str:
        project_root = getattr(self._main_controller, "project_root", os.getcwd())
        return os.path.abspath(os.path.join(project_root, "user_data", "projects"))
