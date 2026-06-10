"""Contrôleur pour l'onglet Axes externes."""
from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
from PyQt6.QtCore import QObject, pyqtSignal

from models.external_axis import ExternalAxis
from models.external_axes_model import ExternalAxesModel
from models.workspace_model import WorkspaceModel
from utils.file_io import FileIOHandler
from utils.reference_frame_utils import FrameTransform
from utils.math_utils import matrix_to_pose_zyx
from views.external_axes_view import ExternalAxesView
from widgets.external_axes_view.external_axes_panel_widget import ExternalAxesPanelWidget

# Répertoire proposé par défaut pour la sauvegarde des configs axes
_DEFAULT_CONFIG_DIR = str(
    Path(__file__).parent.parent / "default_data" / "external_axes_configs"
)


def _normalize_project_path(path: str) -> str:
    if not path:
        return path
    absolute_path = os.path.abspath(path)
    project_root = os.path.abspath(os.getcwd())
    try:
        common_path = os.path.commonpath([project_root, absolute_path])
    except ValueError:
        return path
    if common_path != project_root:
        return path
    try:
        relative_path = os.path.relpath(absolute_path, project_root)
    except ValueError:
        return path
    relative_path = relative_path.replace("\\", "/")
    if relative_path == ".":
        return "./"
    return f"./{relative_path}" if not relative_path.startswith(".") else relative_path


class ExternalAxesController(QObject):
    STATUS_NONE = "Configuration non chargée"
    STATUS_UNSAVED = "Configuration axe externe non enregistrée"
    STATUS_MODIFIED = "Modifications non enregistrées"
    STATUS_SAVED = "Configuration axe externe enregistrée"
    STATUS_LOADED = "Configuration axe externe chargée"
    STATUS_UP_TO_DATE = "Configuration axe externe à jour"
    STATUS_OK_COLOR = "#6fcf97"
    validation_state_changed = pyqtSignal(bool)

    def __init__(
        self,
        external_axes_model: ExternalAxesModel,
        workspace_model: WorkspaceModel,
        external_axes_view: ExternalAxesView,
        viewer3d_controller,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.model = external_axes_model
        self.workspace_model = workspace_model
        self.view = external_axes_view
        self.viewer3d_controller = viewer3d_controller
        self._panel: ExternalAxesPanelWidget = external_axes_view.get_panel_widget()
        self._current_config_file = ""
        self._is_loading_configuration = False
        self._saved_snapshot: str | None = None
        self._has_saved_reference = False
        self._clean_status_text = ExternalAxesController.STATUS_NONE
        self._was_dirty_since_reference = False
        self._validation_icon_visible = False
        self._setup_connections()
        self._refresh_view()
        self._mark_as_none_reference()

    # ------------------------------------------------------------------
    # Connexions
    # ------------------------------------------------------------------

    def _setup_connections(self) -> None:
        # Vue → modèle
        self._panel.axis_added.connect(self._on_axis_added)
        self._panel.axis_removed.connect(self._on_axis_removed)
        self._panel.axis_updated.connect(self._on_axis_updated)
        self._panel.robot_mount_parent_changed.connect(self._on_robot_mount_changed)
        self._panel.save_config_requested.connect(self._on_save_config)
        self._panel.save_as_config_requested.connect(self._on_save_as_config)
        self._panel.load_config_requested.connect(self._on_load_config)
        self._panel.new_config_requested.connect(self._on_new_config)

        # Modèle → vue + viewer
        self.model.axes_changed.connect(self._on_model_axes_changed)
        self.model.axes_values_changed.connect(self._on_model_values_changed)
        self.model.mount_topology_changed.connect(self._on_model_topology_changed)

    # ------------------------------------------------------------------
    # Vue → modèle
    # ------------------------------------------------------------------

    def _on_axis_added(self, axis: ExternalAxis) -> None:
        self.model.add_axis(axis)

    def _on_axis_removed(self, axis_id: str) -> None:
        self.model.remove_axis(axis_id)

    def _on_axis_updated(self, axis_id: str, axis: ExternalAxis) -> None:
        self.model.update_axis(axis_id, axis)

    def _on_robot_mount_changed(self, parent_id) -> None:
        self.model.set_robot_mount_parent_id(parent_id)

    # ------------------------------------------------------------------
    # Modèle → vue + viewer
    # ------------------------------------------------------------------

    def _on_model_axes_changed(self) -> None:
        self._refresh_view()
        self._refresh_configuration_status()
        self._apply_robot_base_override()
        self._refresh_viewer_cad()

    def _on_model_values_changed(self) -> None:
        self._refresh_configuration_status()
        self._apply_robot_base_override()
        self._refresh_viewer_poses()

    def _on_model_topology_changed(self) -> None:
        self._refresh_configuration_status()
        self._apply_robot_base_override()
        self._refresh_viewer_poses()

    # ------------------------------------------------------------------
    # Utilitaires
    # ------------------------------------------------------------------

    def _refresh_view(self) -> None:
        self._panel.refresh_from_model(self.model)

    def _refresh_configuration_header(self) -> None:
        self._panel.set_current_configuration_name(
            os.path.basename(self._current_config_file) if self._current_config_file else "Aucune configuration"
        )

    def get_current_config_file(self) -> str:
        return self._current_config_file

    def is_dirty(self) -> bool:
        return self._is_dirty()

    def _has_axes_configuration(self) -> bool:
        return bool(self.model.get_axes())

    def _capture_current_snapshot(self) -> str:
        return ExternalAxesController._normalized_serialized_state(self.get_serializable_state())

    def _mark_as_saved_reference(self) -> None:
        self._saved_snapshot = self._capture_current_snapshot()
        self._has_saved_reference = True
        self._clean_status_text = ExternalAxesController.STATUS_SAVED
        self._was_dirty_since_reference = False
        self._refresh_configuration_status()

    def _mark_as_loaded_reference(self) -> None:
        self._saved_snapshot = self._capture_current_snapshot()
        self._has_saved_reference = True
        self._clean_status_text = ExternalAxesController.STATUS_LOADED
        self._was_dirty_since_reference = False
        self._refresh_configuration_status()

    def _mark_as_unsaved_reference(self) -> None:
        self._saved_snapshot = self._capture_current_snapshot()
        self._has_saved_reference = False
        self._clean_status_text = ExternalAxesController.STATUS_UNSAVED
        self._was_dirty_since_reference = False
        self._refresh_configuration_status()

    def _mark_as_none_reference(self) -> None:
        self._saved_snapshot = self._capture_current_snapshot()
        self._has_saved_reference = False
        self._clean_status_text = ExternalAxesController.STATUS_NONE
        self._was_dirty_since_reference = False
        self._refresh_configuration_status()

    def _is_dirty(self) -> bool:
        current_snapshot = self._capture_current_snapshot()
        return self._saved_snapshot is None or current_snapshot != self._saved_snapshot

    def _refresh_configuration_status(self) -> None:
        self._refresh_configuration_header()
        show_validation_icon = self._should_show_validation_icon()
        if show_validation_icon != self._validation_icon_visible:
            self._validation_icon_visible = show_validation_icon
            self.validation_state_changed.emit(show_validation_icon)
        if not self._has_axes_configuration():
            self._panel.set_configuration_status(ExternalAxesController.STATUS_NONE, "#808080")
            return
        if not self._has_saved_reference:
            if self._is_dirty():
                self._panel.set_configuration_status(ExternalAxesController.STATUS_UNSAVED, "#808080")
                return
            self._panel.set_configuration_status(ExternalAxesController.STATUS_NONE, "#808080")
            return
        if self._is_dirty():
            self._was_dirty_since_reference = True
            self._panel.set_configuration_status(ExternalAxesController.STATUS_MODIFIED, "#d97706")
            return
        if self._was_dirty_since_reference:
            self._panel.set_configuration_status(
                ExternalAxesController.STATUS_UP_TO_DATE,
                ExternalAxesController.STATUS_OK_COLOR,
            )
            return
        self._panel.set_configuration_status(
            self._clean_status_text,
            ExternalAxesController.STATUS_OK_COLOR,
        )

    def _should_show_validation_icon(self) -> bool:
        if not self._has_axes_configuration():
            return False
        if not self._has_saved_reference:
            return self._is_dirty()
        if self._is_dirty():
            return True
        if self._was_dirty_since_reference:
            return True
        return self._clean_status_text not in {
            ExternalAxesController.STATUS_UNSAVED,
            ExternalAxesController.STATUS_NONE,
        }

    @staticmethod
    def _normalized_serialized_state(data: dict) -> str:
        normalized = json.loads(json.dumps(data))
        ExternalAxesController._strip_runtime_axis_values(normalized)
        for axis_data in normalized.get("axes", []):
            axis_data["base_cad_model"] = _normalize_project_path(axis_data.get("base_cad_model", ""))
            for joint_data in axis_data.get("joints", []):
                joint_data["cad_model"] = _normalize_project_path(joint_data.get("cad_model", ""))
        return json.dumps(normalized, sort_keys=True, ensure_ascii=True)

    @staticmethod
    def _strip_runtime_axis_values(data: dict) -> None:
        for axis_data in data.get("axes", []):
            for joint_data in axis_data.get("joints", []):
                joint_data.pop("value", None)

    @staticmethod
    def _reset_runtime_axis_values(data: dict) -> dict:
        normalized = json.loads(json.dumps(data))
        for axis_data in normalized.get("axes", []):
            for joint_data in axis_data.get("joints", []):
                joint_data["value"] = 0.0
        return normalized

    def _infer_config_file_from_data(self, data: dict) -> str:
        config_dir = Path(_DEFAULT_CONFIG_DIR)
        if not config_dir.exists():
            return ""
        target_payload = ExternalAxesController._normalized_serialized_state(data)
        for candidate in sorted(config_dir.glob("*.json")):
            try:
                with open(candidate, "r", encoding="utf-8") as file:
                    candidate_data = json.load(file)
            except (OSError, ValueError, TypeError):
                continue
            if ExternalAxesController._normalized_serialized_state(candidate_data) == target_payload:
                return str(candidate)
        return ""

    def _apply_robot_base_override(self) -> None:
        """Met à jour le viewer3D avec la nouvelle base robot monde."""
        override = self.model.get_robot_world_base_matrix()
        if hasattr(self.viewer3d_controller, 'viewer_3d_widget'):
            self.viewer3d_controller.viewer_3d_widget.set_external_robot_base_override(override)
        if hasattr(self.viewer3d_controller, 'refresh_tcp_overlay'):
            self.viewer3d_controller.refresh_tcp_overlay()

    def _refresh_viewer_cad(self) -> None:
        if hasattr(self.viewer3d_controller, 'viewer_3d_widget'):
            world_transforms = self.model.compute_world_transforms()
            self.viewer3d_controller.viewer_3d_widget.reload_external_axes(
                self.model.get_axes(), world_transforms
            )

    def _refresh_viewer_poses(self) -> None:
        if hasattr(self.viewer3d_controller, 'viewer_3d_widget'):
            world_transforms = self.model.compute_world_transforms()
            self.viewer3d_controller.viewer_3d_widget.update_external_axes_poses(world_transforms)
            self.viewer3d_controller.viewer_3d_widget.refresh_camera_visibility()

    # ------------------------------------------------------------------
    # Sauvegarde / chargement manuel (boutons du panneau)
    # ------------------------------------------------------------------

    def _on_save_config(self) -> None:
        """Enregistre la configuration des axes externes dans le fichier courant."""
        os.makedirs(_DEFAULT_CONFIG_DIR, exist_ok=True)
        if not self._current_config_file:
            self._on_save_as_config()
            return

        data = self.get_serializable_state()
        FileIOHandler.write_json(self._current_config_file, data)
        self._mark_as_saved_reference()

    def _on_save_as_config(self) -> None:
        """Exporte la configuration des axes externes vers un nouveau fichier JSON."""
        os.makedirs(_DEFAULT_CONFIG_DIR, exist_ok=True)
        data = self.get_serializable_state()
        file_path = FileIOHandler.save_json(
            self._panel,
            "Enregistrer une configuration des axes externes",
            data,
            directory=_DEFAULT_CONFIG_DIR,
        )
        if file_path:
            self._current_config_file = file_path
            self._mark_as_saved_reference()

    def _on_load_config(self) -> None:
        """Importe une configuration des axes externes depuis un fichier JSON."""
        os.makedirs(_DEFAULT_CONFIG_DIR, exist_ok=True)
        file_path = FileIOHandler.select_file(
            self._panel,
            "Charger une configuration des axes externes",
            directory=_DEFAULT_CONFIG_DIR,
            filter="JSON Files (*.json)",
        )
        if not file_path:
            return
        self.viewer3d_controller.begin_loading_feedback("Chargement axes externes ...")
        try:
            self.load_configuration_from_path(file_path)
        finally:
            self.viewer3d_controller.end_loading_feedback()

    def _on_new_config(self) -> None:
        self._current_config_file = ""
        self.model.from_dict(
            {
                "axes": [],
                "robot_mount_parent_id": None,
            }
        )
        self._mark_as_none_reference()

    def new_configuration(self) -> None:
        self._on_new_config()

    def load_configuration_from_path(self, file_path: str) -> bool:
        if not file_path:
            return False
        _, data = FileIOHandler.load_json(file_path)
        if not data:
            return False
        self.restore_state(data, file_path=file_path)
        return True

    # ------------------------------------------------------------------
    # Sérialisation
    # ------------------------------------------------------------------

    def get_serializable_state(self) -> dict:
        data = self.model.to_dict()
        ExternalAxesController._strip_runtime_axis_values(data)
        for axis_data in data.get("axes", []):
            axis_data["base_cad_model"] = _normalize_project_path(axis_data.get("base_cad_model", ""))
            for joint_data in axis_data.get("joints", []):
                joint_data["cad_model"] = _normalize_project_path(joint_data.get("cad_model", ""))
        return data

    def restore_state(self, data: dict, file_path: str = "") -> None:
        if not data:
            return
        data = ExternalAxesController._reset_runtime_axis_values(data)
        self._is_loading_configuration = True
        try:
            self.model.from_dict(data)
        finally:
            self._is_loading_configuration = False
        resolved_file_path = file_path or self._infer_config_file_from_data(data)
        self._current_config_file = resolved_file_path
        if resolved_file_path:
            self._mark_as_loaded_reference()
        else:
            self._mark_as_unsaved_reference()

    def infer_config_file_from_state_data(self, data: dict) -> str:
        if not data:
            return ""
        return self._infer_config_file_from_data(data)
