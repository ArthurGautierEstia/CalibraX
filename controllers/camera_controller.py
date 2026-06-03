from __future__ import annotations

import os

from PyQt6.QtCore import QObject
from PyQt6.QtWidgets import QFileDialog, QMessageBox

from controllers.viewer3d_controller import Viewer3DController
from models.camera_model import CameraConfiguration, CameraConfigurationFile, CameraModel
from views.camera_view import CameraView
from widgets.camera_view.camera_configuration_widget import CameraConfigurationWidget


STATUS_OK_COLOR = "#6fcf97"
STATUS_NONE = "Configuration non chargée"
STATUS_UNSAVED = "Configuration caméra non enregistrée"
STATUS_MODIFIED = "Modifications non enregistrées"
STATUS_SAVED = "Configuration caméra enregistrée"
STATUS_LOADED = "Configuration caméra chargée"
STATUS_UP_TO_DATE = "Configuration caméra à jour"


class CameraController(QObject):
    def __init__(
        self,
        camera_model: CameraModel,
        camera_view: CameraView,
        viewer3d_controller: Viewer3DController,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.camera_model = camera_model
        self.camera_view = camera_view
        self.viewer3d_controller = viewer3d_controller
        self.camera_widget = camera_view.get_configuration_widget()
        self._updating_from_view = False
        self._reference_data: dict | None = None
        self._has_reference = False
        self._was_dirty_since_reference = False
        self._clean_status_text = STATUS_NONE

        self._setup_connections()
        self._update_camera_view()
        self._update_configuration_status()
        self.viewer3d_controller.set_camera_model(self.camera_model)

    def _setup_connections(self) -> None:
        self.camera_model.cameras_changed.connect(self._on_camera_model_changed)
        self.camera_model.visibility_changed.connect(self._update_visibility_view)

        self.camera_widget.new_config_requested.connect(self._on_new_requested)
        self.camera_widget.load_config_requested.connect(self._on_load_requested)
        self.camera_widget.save_config_requested.connect(self._on_save_requested)
        self.camera_widget.save_as_config_requested.connect(self._on_save_as_requested)
        self.camera_widget.add_camera_requested.connect(self._on_add_camera_requested)
        self.camera_widget.duplicate_camera_requested.connect(self._on_duplicate_camera_requested)
        self.camera_widget.remove_camera_requested.connect(self._on_remove_camera_requested)
        self.camera_widget.camera_updated.connect(self._on_camera_updated)
        self.camera_widget.selection_changed.connect(self._on_selection_changed)

    def _on_camera_model_changed(self) -> None:
        self._update_camera_view()
        self._update_configuration_status()
        self.viewer3d_controller.refresh_cameras()

    def _update_camera_view(self) -> None:
        self.camera_widget.set_current_file_path(self.camera_model.get_current_file_path())
        self.camera_widget.set_cameras(self.camera_model.get_cameras())
        self._update_visibility_view()

    def _update_visibility_view(self) -> None:
        self.camera_widget.set_visibility_results(self.camera_model.get_visibility_results())

    def _on_new_requested(self) -> None:
        self.camera_model.set_current_file_path("")
        self.camera_model.set_setup_name("Camera setup")
        self.camera_model.set_cameras([])
        self._reference_data = None
        self._has_reference = False
        self._was_dirty_since_reference = False
        self._clean_status_text = STATUS_NONE
        self._update_camera_view()
        self._update_configuration_status()

    def _on_load_requested(self) -> None:
        selected_path, _ = QFileDialog.getOpenFileName(
            self.camera_widget,
            "Charger une configuration camera",
            self._camera_directory(),
            "JSON Files (*.json)",
        )
        if not selected_path:
            return
        self.load_configuration_from_path(selected_path, show_errors=True)

    def _on_save_requested(self) -> None:
        current_path = self.camera_model.get_current_file_path()
        if not current_path:
            self._on_save_as_requested()
            return
        self._save_to_path(current_path)

    def _on_save_as_requested(self) -> None:
        default_path = os.path.join(self._camera_directory(), "camera_setup.json")
        selected_path, _ = QFileDialog.getSaveFileName(
            self.camera_widget,
            "Enregistrer une configuration camera",
            default_path,
            "JSON Files (*.json)",
        )
        if not selected_path:
            return
        if not selected_path.lower().endswith(".json"):
            selected_path += ".json"
        self._save_to_path(selected_path)

    def _save_to_path(self, selected_path: str) -> bool:
        try:
            config = CameraConfigurationFile.from_camera_model(self.camera_model)
            errors = config.validate()
            if errors:
                raise ValueError("\n".join(errors))
            config.save(selected_path)
        except (OSError, ValueError, TypeError) as exc:
            QMessageBox.warning(
                self.camera_widget,
                "Erreur sauvegarde",
                f"Impossible de sauvegarder la configuration camera.\n{exc}",
            )
            return False
        self.camera_model.set_current_file_path(CameraConfigurationWidgetPath.normalize_project_path(selected_path))
        self._mark_current_configuration_as_reference(STATUS_SAVED)
        self._update_camera_view()
        return True

    def load_configuration_from_path(self, selected_path: str, show_errors: bool = False) -> bool:
        try:
            config = CameraConfigurationFile.load(selected_path)
        except (OSError, ValueError, TypeError) as exc:
            if show_errors:
                QMessageBox.warning(
                    self.camera_widget,
                    "Configuration camera invalide",
                    f"Impossible de charger la configuration camera.\n{exc}",
                )
            return False

        self.camera_model.load_configuration_file(
            config,
            file_path=CameraConfigurationWidgetPath.normalize_project_path(selected_path),
        )
        self._mark_current_configuration_as_reference(STATUS_LOADED)
        return True

    def _on_add_camera_requested(self) -> None:
        self.camera_model.add_camera()

    def _on_duplicate_camera_requested(self, index: int) -> None:
        cameras = self.camera_model.get_cameras()
        if not (0 <= index < len(cameras)):
            return
        source = cameras[index]
        duplicate = CameraConfiguration.from_dict(
            {
                **source.to_dict(),
                "id": f"{source.camera_id}_copy",
                "name": f"{source.name} copie",
            }
        )
        self.camera_model.add_camera(duplicate)

    def _on_camera_updated(self, index: int, updated_camera: CameraConfiguration) -> None:
        cameras = self.camera_model.get_cameras()
        if not (0 <= index < len(cameras)):
            return
        errors = updated_camera.validate()
        if errors:
            QMessageBox.warning(
                self.camera_widget,
                "Camera invalide",
                "\n".join(errors),
            )
            return
        duplicate_ids = [
            camera.camera_id
            for row, camera in enumerate(cameras)
            if row != index and camera.camera_id == updated_camera.camera_id
        ]
        if duplicate_ids:
            QMessageBox.warning(
                self.camera_widget,
                "Camera invalide",
                f"L'ID camera '{updated_camera.camera_id}' est deja utilise.",
            )
            return
        self.camera_model.update_camera(index, updated_camera)

    def _on_remove_camera_requested(self, index: int) -> None:
        self.camera_model.remove_camera(index)

    def _on_selection_changed(self, index: int) -> None:
        self.viewer3d_controller.set_selected_camera_index(index)

    def _camera_directory(self) -> str:
        root_dir = os.getcwd()
        camera_dir = os.path.abspath(os.path.join(root_dir, CameraModel.DEFAULT_CAMERA_DIRECTORY))
        os.makedirs(camera_dir, exist_ok=True)
        return camera_dir

    def _current_configuration_data(self) -> dict:
        return CameraConfigurationFile.from_camera_model(self.camera_model).to_dict()

    def _mark_current_configuration_as_reference(self, clean_status_text: str) -> None:
        self._reference_data = self._current_configuration_data()
        self._has_reference = True
        self._was_dirty_since_reference = False
        self._clean_status_text = clean_status_text
        self._update_configuration_status()

    def _update_configuration_status(self) -> None:
        current_data = self._current_configuration_data()
        if not self._has_reference:
            if current_data.get("cameras"):
                self.camera_widget.set_configuration_status(STATUS_UNSAVED, "#808080")
            else:
                self.camera_widget.set_configuration_status(STATUS_NONE, "#808080")
            return

        if current_data != self._reference_data:
            self._was_dirty_since_reference = True
            self.camera_widget.set_configuration_status(STATUS_MODIFIED, "#d97706")
            return

        if self._was_dirty_since_reference:
            self._clean_status_text = STATUS_UP_TO_DATE
            self._was_dirty_since_reference = False
        self.camera_widget.set_configuration_status(self._clean_status_text, STATUS_OK_COLOR)


class CameraConfigurationWidgetPath:
    @staticmethod
    def normalize_project_path(path: str) -> str:
        absolute_path = os.path.abspath(path)
        project_root = os.path.abspath(os.getcwd())
        try:
            common_path = os.path.commonpath([project_root, absolute_path])
        except ValueError:
            return absolute_path
        if common_path != project_root:
            return absolute_path
        relative_path = os.path.relpath(absolute_path, project_root).replace("\\", "/")
        return f"./{relative_path}" if not relative_path.startswith(".") else relative_path
