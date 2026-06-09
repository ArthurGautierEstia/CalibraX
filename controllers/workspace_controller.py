from __future__ import annotations

import json
import os

from PyQt6.QtCore import QObject, pyqtSignal
from PyQt6.QtWidgets import QFileDialog, QMessageBox

from models.workspace_file import WorkspaceFile
from models.workspace_model import WorkspaceModel
from models.types import Pose6
from models.workspace_cad_element import WorkspaceCadElement
from models.workspace_primitive_zone_models import WorkspacePrimitiveZoneData
from views.workspace_view import WorkspaceView
from widgets.workspace_view.workspace_configuration_widget import WorkspaceConfigurationWidget


STATUS_OK_COLOR = "#6fcf97"
STATUS_NONE = "Configuration non chargée"
STATUS_UNSAVED = "Configuration scene non enregistrée"
STATUS_MODIFIED = "Modifications non enregistrées"
STATUS_SAVED = "Configuration scene enregistrée"
STATUS_LOADED = "Configuration scene chargée"
STATUS_UP_TO_DATE = "Configuration scene à jour"


class WorkspaceController(QObject):
    validation_state_changed = pyqtSignal(bool)

    def __init__(
        self,
        workspace_model: WorkspaceModel,
        workspace_view: WorkspaceView,
        viewer3d_controller=None,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.workspace_model = workspace_model
        self.workspace_view = workspace_view
        self.workspace_widget = workspace_view.get_configuration_widget()
        self.viewer3d_controller = viewer3d_controller
        self._updating_from_view = False
        self._reference_data: dict | None = None
        self._has_reference = False
        self._was_dirty_since_reference = False
        self._clean_status_text = STATUS_NONE
        self._validation_icon_visible = False

        self._setup_connections()
        self._update_workspace_view()
        self._update_configuration_status()

    def _setup_connections(self) -> None:
        self.workspace_model.workspace_changed.connect(self._update_workspace_view)

        self.workspace_widget.scene_name_changed.connect(self._on_scene_name_changed)
        self.workspace_widget.robot_base_pose_world_changed.connect(self._on_robot_base_pose_world_changed)
        self.workspace_widget.workspace_cad_elements_changed.connect(self._on_workspace_cad_elements_changed)
        self.workspace_widget.workspace_tcp_zone_changed.connect(self._on_workspace_tcp_zone_changed)
        self.workspace_widget.workspace_tcp_zone_added.connect(self._on_workspace_tcp_zone_added)
        self.workspace_widget.workspace_tcp_zone_removed.connect(self._on_workspace_tcp_zone_removed)
        self.workspace_widget.workspace_collision_zone_changed.connect(self._on_workspace_collision_zone_changed)
        self.workspace_widget.workspace_collision_zone_added.connect(self._on_workspace_collision_zone_added)
        self.workspace_widget.workspace_collision_zone_removed.connect(self._on_workspace_collision_zone_removed)
        self.workspace_widget.workspace_save_requested.connect(self._on_save_workspace_requested)
        self.workspace_widget.workspace_save_as_requested.connect(self._on_save_as_workspace_requested)
        self.workspace_widget.workspace_load_requested.connect(self._on_load_workspace_requested)
        self.workspace_widget.workspace_clear_requested.connect(self._on_clear_workspace_requested)

    def _update_workspace_view(self) -> None:
        if self._updating_from_view:
            return
        workspace_dir = self._workspace_directory()
        self.workspace_widget.set_workspace_directory(WorkspaceConfigurationWidget._normalize_project_path(workspace_dir))
        self.workspace_widget.set_workspace_scene_name(self.workspace_model.get_workspace_scene_name())
        self.workspace_widget.set_workspace_file_path(self.workspace_model.get_workspace_file_path())
        self.workspace_widget.set_current_configuration_name(os.path.basename(self.workspace_model.get_workspace_file_path()))
        self.workspace_widget.set_robot_base_pose_world(self.workspace_model.get_robot_base_pose_world())
        self.workspace_widget.set_workspace_cad_elements(self.workspace_model.get_workspace_cad_elements())
        self.workspace_widget.set_workspace_tcp_zones(self.workspace_model.get_workspace_tcp_zones())
        self.workspace_widget.set_workspace_collision_zones(self.workspace_model.get_workspace_collision_zones())

    def _on_scene_name_changed(self, scene_name: str) -> None:
        self._updating_from_view = True
        try:
            self.workspace_model.set_workspace_scene_name(scene_name)
        finally:
            self._updating_from_view = False
        self._update_configuration_status()

    def _on_workspace_cad_elements_changed(self, values: list[WorkspaceCadElement]) -> None:
        self._updating_from_view = True
        try:
            self.workspace_model.set_workspace_cad_elements(values)
        finally:
            self._updating_from_view = False
        self._update_configuration_status()

    def _on_robot_base_pose_world_changed(self, pose: Pose6) -> None:
        self._updating_from_view = True
        try:
            self.workspace_model.set_robot_base_pose_world(pose)
        finally:
            self._updating_from_view = False
        self._update_configuration_status()

    def _on_workspace_tcp_zone_changed(self, index: int, value: WorkspacePrimitiveZoneData) -> None:
        self._updating_from_view = True
        try:
            self.workspace_model.update_workspace_tcp_zone(index, value)
        finally:
            self._updating_from_view = False
        self._update_configuration_status()

    def _on_workspace_tcp_zone_added(self, index: int, value: WorkspacePrimitiveZoneData) -> None:
        self._updating_from_view = True
        try:
            self.workspace_model.insert_workspace_tcp_zone(index, value)
        finally:
            self._updating_from_view = False
        self._update_configuration_status()

    def _on_workspace_tcp_zone_removed(self, index: int) -> None:
        self._updating_from_view = True
        try:
            self.workspace_model.remove_workspace_tcp_zone(index)
        finally:
            self._updating_from_view = False
        self._update_configuration_status()

    def _on_workspace_collision_zone_changed(self, index: int, value: WorkspacePrimitiveZoneData) -> None:
        self._updating_from_view = True
        try:
            self.workspace_model.update_workspace_collision_zone(index, value)
        finally:
            self._updating_from_view = False
        self._update_configuration_status()

    def _on_workspace_collision_zone_added(self, index: int, value: WorkspacePrimitiveZoneData) -> None:
        self._updating_from_view = True
        try:
            self.workspace_model.insert_workspace_collision_zone(index, value)
        finally:
            self._updating_from_view = False
        self._update_configuration_status()

    def _on_workspace_collision_zone_removed(self, index: int) -> None:
        self._updating_from_view = True
        try:
            self.workspace_model.remove_workspace_collision_zone(index)
        finally:
            self._updating_from_view = False
        self._update_configuration_status()

    def _on_save_workspace_requested(self) -> None:
        current_path = self.workspace_model.get_workspace_file_path()
        if not current_path:
            self._on_save_as_workspace_requested()
            return
        self._save_workspace_to_path(current_path)

    def _on_save_as_workspace_requested(self) -> None:
        workspace_dir = self._workspace_directory()
        scene_name = self.workspace_model.get_workspace_scene_name().strip()
        if scene_name == "":
            scene_name = WorkspaceModel.DEFAULT_WORKSPACE_SCENE_NAME
            self.workspace_model.set_workspace_scene_name(scene_name)

        default_file_name = f"{self._safe_scene_file_name(scene_name)}.json"
        default_target = os.path.join(workspace_dir, default_file_name)
        selected_path, _ = QFileDialog.getSaveFileName(
            self.workspace_widget,
            "Enregistrer un workspace",
            default_target,
            "JSON Files (*.json)",
        )
        if not selected_path:
            return
        if not selected_path.lower().endswith(".json"):
            selected_path += ".json"

        self._save_workspace_to_path(selected_path)

    def _save_workspace_to_path(self, selected_path: str) -> bool:
        try:
            workspace_file = WorkspaceFile.from_workspace_model(self.workspace_model)
            workspace_file.save(selected_path)
        except (OSError, ValueError, TypeError) as exc:
            QMessageBox.warning(self.workspace_widget, "Erreur sauvegarde", f"Impossible de sauvegarder la scene.\n{exc}")
            return False

        self.workspace_model.set_workspace_file_path(WorkspaceConfigurationWidget._normalize_project_path(selected_path))
        self._mark_current_configuration_as_reference(STATUS_SAVED)
        self._update_workspace_view()
        return True

    def _on_load_workspace_requested(self) -> None:
        workspace_dir = self._workspace_directory()
        selected_path, _ = QFileDialog.getOpenFileName(
            self.workspace_widget,
            "Charger un workspace",
            workspace_dir,
            "JSON Files (*.json)",
        )
        if not selected_path:
            return

        self._begin_loading_feedback("Chargement configuration scene ...")
        try:
            self.load_workspace_from_path(selected_path, show_errors=True)
        finally:
            self._end_loading_feedback()

    def _on_clear_workspace_requested(self) -> None:
        self.workspace_model.clear_workspace()
        self._reference_data = None
        self._has_reference = False
        self._was_dirty_since_reference = False
        self._clean_status_text = STATUS_NONE
        self._update_workspace_view()
        self._update_configuration_status()

    def new_configuration(self) -> None:
        self._on_clear_workspace_requested()

    def is_dirty(self) -> bool:
        return self._is_dirty()

    def _workspace_directory(self) -> str:
        root_dir = os.getcwd()
        workspace_dir = os.path.abspath(os.path.join(root_dir, WorkspaceModel.DEFAULT_WORKSPACE_DIRECTORY))
        os.makedirs(workspace_dir, exist_ok=True)
        return workspace_dir

    @staticmethod
    def _safe_scene_file_name(scene_name: str) -> str:
        forbidden = '<>:"/\\|?*'
        safe = scene_name.replace(" ", "_")
        safe = "".join("_" if c in forbidden else c for c in safe).strip().strip(".")
        return safe if safe else "scene"

    def load_workspace_from_path(self, selected_path: str, show_errors: bool = False) -> bool:
        try:
            workspace_file = WorkspaceFile.load(selected_path)
        except (OSError, ValueError, TypeError) as exc:
            if show_errors:
                QMessageBox.warning(
                    self.workspace_widget,
                    "Workspace invalide",
                    f"Impossible de charger le workspace.\n{exc}",
                )
            return False

        workspace_file.apply_to_workspace_model(
            self.workspace_model,
            file_path=WorkspaceConfigurationWidget._normalize_project_path(selected_path),
        )
        self._mark_current_configuration_as_reference(STATUS_LOADED)
        return True

    def _begin_loading_feedback(self, message: str) -> None:
        if self.viewer3d_controller is not None:
            self.viewer3d_controller.begin_loading_feedback(message)

    def _end_loading_feedback(self) -> None:
        if self.viewer3d_controller is not None:
            self.viewer3d_controller.end_loading_feedback()

    def _current_configuration_data(self) -> dict:
        return WorkspaceFile.from_workspace_model(self.workspace_model).to_dict()

    @staticmethod
    def _normalized_serialized_state(data: dict) -> str:
        return json.dumps(json.loads(json.dumps(data)), sort_keys=True, ensure_ascii=True)

    def _mark_current_configuration_as_reference(self, clean_status_text: str) -> None:
        self._reference_data = self._current_configuration_data()
        self._has_reference = True
        self._was_dirty_since_reference = False
        self._clean_status_text = clean_status_text
        self._update_configuration_status()

    def _is_dirty(self) -> bool:
        if self._reference_data is None:
            return True
        return (
            WorkspaceController._normalized_serialized_state(self._current_configuration_data())
            != WorkspaceController._normalized_serialized_state(self._reference_data)
        )

    def _update_configuration_status(self) -> None:
        current_data = self._current_configuration_data()
        show_validation_icon = self._should_show_validation_icon(current_data)
        if show_validation_icon != self._validation_icon_visible:
            self._validation_icon_visible = show_validation_icon
            self.validation_state_changed.emit(show_validation_icon)

        if not self._has_reference:
            has_content = any(
                (
                    current_data.get("cad_elements"),
                    current_data.get("tcp_zones"),
                    current_data.get("collision_zones"),
                    current_data.get("scene_name") != WorkspaceModel.DEFAULT_WORKSPACE_SCENE_NAME,
                    current_data.get("robot_base_pose_world") != Pose6.zeros().to_list(),
                )
            )
            self.workspace_widget.set_configuration_status(
                STATUS_UNSAVED if has_content else STATUS_NONE,
                "#808080",
            )
            return

        if self._is_dirty():
            self._was_dirty_since_reference = True
            self.workspace_widget.set_configuration_status(STATUS_MODIFIED, "#d97706")
            return

        if self._was_dirty_since_reference:
            self._clean_status_text = STATUS_UP_TO_DATE
            self._was_dirty_since_reference = False
        self.workspace_widget.set_configuration_status(self._clean_status_text, STATUS_OK_COLOR)

    def _should_show_validation_icon(self, current_data: dict | None = None) -> bool:
        if not self._has_reference:
            return False
        if current_data is None:
            current_data = self._current_configuration_data()
        if (
            WorkspaceController._normalized_serialized_state(current_data)
            != WorkspaceController._normalized_serialized_state(self._reference_data)
        ):
            return False
        return self._clean_status_text in {
            STATUS_SAVED,
            STATUS_LOADED,
            STATUS_UP_TO_DATE,
        }
