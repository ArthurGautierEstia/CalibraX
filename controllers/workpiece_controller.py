"""Contrôleur pour l'onglet Pièce (outillage + pièce)."""
from __future__ import annotations

import os
from pathlib import Path
import json

import numpy as np
from PyQt6.QtCore import QObject, pyqtSignal
from PyQt6.QtWidgets import QFileDialog, QMessageBox

from models.external_axes_model import ExternalAxesModel
from models.tooling_model import ToolingModel
from models.types.pose6 import Pose6
from models.workspace_model import WorkspaceModel
from models.workpiece_model import WorkpieceModel
from utils.external_axes_kinematics import get_effective_robot_base_in_world
from utils.math_utils import pose_zyx_to_matrix
from views.workpiece_view import WorkpieceView
from widgets.workpiece_view.tooling_panel_widget import ToolingPanelWidget
from widgets.workpiece_view.workpiece_config_widget import WorkpieceConfigWidget

_DEFAULT_CONFIG_DIR = str(
    Path(__file__).parent.parent / "default_data" / "piece_configs"
)

STATUS_OK_COLOR = "#6fcf97"
STATUS_NONE = "Configuration non chargée"
STATUS_UNSAVED = "Configuration pièce non enregistrée"
STATUS_MODIFIED = "Modifications non enregistrées"
STATUS_SAVED = "Configuration pièce enregistrée"
STATUS_LOADED = "Configuration pièce chargée"
STATUS_UP_TO_DATE = "Configuration pièce à jour"


class WorkpieceController(QObject):
    validation_state_changed = pyqtSignal(bool)

    def __init__(
        self,
        workpiece_model: WorkpieceModel,
        tooling_model: ToolingModel,
        workspace_model: WorkspaceModel,
        external_axes_model: ExternalAxesModel,
        workpiece_view: WorkpieceView,
        viewer3d_controller,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.workpiece_model = workpiece_model
        self.tooling_model = tooling_model
        self.workspace_model = workspace_model
        self.external_axes_model = external_axes_model
        self.view = workpiece_view
        self.viewer3d_controller = viewer3d_controller

        self._tooling_panel: ToolingPanelWidget = workpiece_view.get_tooling_panel()
        self._piece_widget: WorkpieceConfigWidget = workpiece_view.get_piece_config()
        self._current_config_file = ""
        self._current_config_display_name = ""
        self._reference_data: dict | None = None
        self._has_reference = False
        self._was_dirty_since_reference = False
        self._clean_status_text = STATUS_NONE
        self._validation_icon_visible = False

        self._setup_connections()
        self._refresh_parent_frames()
        self._tooling_panel.set_from_model(self.tooling_model)
        self._piece_widget.set_data(self.workpiece_model)
        self._refresh_viewer()
        self._refresh_configuration_status()

    def _refresh_configuration_header(self) -> None:
        display_name = self._current_config_display_name
        if not display_name and self._current_config_file:
            display_name = os.path.basename(self._current_config_file)
        self.view.set_current_configuration_name(display_name or "Aucune configuration")

    def get_current_config_file(self) -> str:
        return self._current_config_file

    def is_dirty(self) -> bool:
        return self._is_dirty()

    def new_configuration(self) -> None:
        self._on_new()

    def _has_configuration_content(self, data: dict | None = None) -> bool:
        if data is None:
            data = self.get_serializable_state()
        tooling = data.get("tooling") or {}
        workpiece = data.get("workpiece") or {}
        return bool(
            tooling.get("parent_frame_id")
            or tooling.get("elements")
            or workpiece.get("cad_model")
            or workpiece.get("parent_frame_id")
            or workpiece.get("pose_in_parent") != Pose6.zeros().to_list()
            or workpiece.get("workpiece_frame_pose") != Pose6.zeros().to_list()
        )

    @staticmethod
    def _normalized_serialized_state(data: dict) -> str:
        return json.dumps(json.loads(json.dumps(data)), sort_keys=True, ensure_ascii=True)

    def _mark_current_configuration_as_reference(self, clean_status_text: str) -> None:
        self._reference_data = self.get_serializable_state()
        self._has_reference = True
        self._was_dirty_since_reference = False
        self._clean_status_text = clean_status_text
        self._refresh_configuration_status()

    def _is_dirty(self) -> bool:
        if self._reference_data is None:
            return True
        return (
            WorkpieceController._normalized_serialized_state(self.get_serializable_state())
            != WorkpieceController._normalized_serialized_state(self._reference_data)
        )

    def _refresh_configuration_status(self) -> None:
        self._refresh_configuration_header()
        current_data = self.get_serializable_state()
        show_validation_icon = self._should_show_validation_icon(current_data)
        if show_validation_icon != self._validation_icon_visible:
            self._validation_icon_visible = show_validation_icon
            self.validation_state_changed.emit(show_validation_icon)

        if not self._has_reference:
            self.view.set_configuration_status(
                STATUS_UNSAVED if self._has_configuration_content(current_data) else STATUS_NONE,
                "#808080",
            )
            return

        if self._is_dirty():
            self._was_dirty_since_reference = True
            self.view.set_configuration_status(STATUS_MODIFIED, "#d97706")
            return

        if self._was_dirty_since_reference:
            self._clean_status_text = STATUS_UP_TO_DATE
            self._was_dirty_since_reference = False
        self.view.set_configuration_status(self._clean_status_text, STATUS_OK_COLOR)

    def _should_show_validation_icon(self, current_data: dict | None = None) -> bool:
        if not self._has_reference:
            return False
        if current_data is None:
            current_data = self.get_serializable_state()
        if (
            WorkpieceController._normalized_serialized_state(current_data)
            != WorkpieceController._normalized_serialized_state(self._reference_data)
        ):
            return False
        return self._clean_status_text in {
            STATUS_SAVED,
            STATUS_LOADED,
            STATUS_UP_TO_DATE,
        }

    # ------------------------------------------------------------------
    # Connexions
    # ------------------------------------------------------------------

    def _setup_connections(self) -> None:
        # Widget → modèle
        self._tooling_panel.tooling_changed.connect(self._on_tooling_widget_changed)
        self._piece_widget.config_changed.connect(self._on_piece_widget_changed)

        # Modèle → widget + viewer
        self.tooling_model.tooling_changed.connect(self._on_tooling_model_changed)
        self.workpiece_model.workpiece_changed.connect(self._on_workpiece_model_changed)

        # Contexte externe → rebâtir les combos
        self.external_axes_model.axes_changed.connect(self._on_context_changed)
        self.external_axes_model.axes_values_changed.connect(self._refresh_viewer)
        self.external_axes_model.mount_topology_changed.connect(self._refresh_viewer)
        self.workspace_model.workspace_changed.connect(self._on_context_changed)

        # Boutons save/load/clear
        self.view.save_requested.connect(self._on_save)
        self.view.save_as_requested.connect(self._on_save_as)
        self.view.load_requested.connect(self._on_load)
        self.view.new_requested.connect(self._on_new)
        self.view.clear_piece_requested.connect(self._on_clear_piece)

    # ------------------------------------------------------------------
    # Réactions widget → modèle
    # ------------------------------------------------------------------

    def _on_tooling_widget_changed(self) -> None:
        self.tooling_model.blockSignals(True)
        self.tooling_model.set_parent_frame_id(self._tooling_panel.get_parent_frame_id())
        # Remplacer tous les éléments
        current_ids = [e.id for e in self.tooling_model.get_elements()]
        new_elements = self._tooling_panel.get_elements()
        # Mettre à jour le modèle directement
        self.tooling_model._parent_frame_id = self._tooling_panel.get_parent_frame_id()
        self.tooling_model._elements = [e.copy() for e in new_elements]
        self.tooling_model.blockSignals(False)
        # Rebâtir le combo pièce (outillage peut avoir été ajouté/supprimé)
        self._refresh_piece_parent_combo()
        self._refresh_viewer()
        self._refresh_configuration_status()

    def _on_piece_widget_changed(self) -> None:
        data = self._piece_widget.get_data()
        self.workpiece_model.blockSignals(True)
        self.workpiece_model.set_cad_model(data["cad_model"])
        self.workpiece_model.set_cad_color(tuple(data["cad_color"]))
        self.workpiece_model.set_parent_frame_id(data["parent_frame_id"])
        self.workpiece_model.set_pose_in_parent(Pose6(*data["pose_in_parent"]))
        self.workpiece_model.set_workpiece_frame_pose(Pose6(*data["workpiece_frame_pose"]))
        self.workpiece_model.blockSignals(False)
        self._refresh_viewer()
        self._refresh_configuration_status()

    # ------------------------------------------------------------------
    # Réactions modèle → widget
    # ------------------------------------------------------------------

    def _on_tooling_model_changed(self) -> None:
        self._tooling_panel.set_from_model(self.tooling_model)
        self._refresh_piece_parent_combo()
        self._refresh_viewer()
        self._refresh_configuration_status()

    def _on_workpiece_model_changed(self) -> None:
        self._piece_widget.set_data(self.workpiece_model)
        self._refresh_viewer()
        self._refresh_configuration_status()

    def _on_context_changed(self) -> None:
        self._refresh_parent_frames()
        self._refresh_viewer()

    # ------------------------------------------------------------------
    # Mise à jour des combos repère parent
    # ------------------------------------------------------------------

    def _refresh_parent_frames(self) -> None:
        axes = self.external_axes_model.get_axes()
        ext_axes = [(a.id, a.name) for a in axes]
        elements = self.workspace_model.get_workspace_cad_elements()
        ws_elements = [e.name for e in elements if e.name]

        self._tooling_panel.update_parent_frames(ext_axes, ws_elements)
        self._refresh_piece_parent_combo(ext_axes=ext_axes, ws_elements=ws_elements)

    def _refresh_piece_parent_combo(
        self,
        ext_axes: list | None = None,
        ws_elements: list | None = None,
    ) -> None:
        if ext_axes is None:
            axes = self.external_axes_model.get_axes()
            ext_axes = [(a.id, a.name) for a in axes]
        if ws_elements is None:
            elements = self.workspace_model.get_workspace_cad_elements()
            ws_elements = [e.name for e in elements if e.name]
        has_tooling = self.tooling_model.has_elements()
        self._piece_widget.update_parent_frames(ext_axes, ws_elements, has_tooling=has_tooling)

    # ------------------------------------------------------------------
    # Calculs cinématiques
    # ------------------------------------------------------------------

    def _get_parent_world_transform(self, parent_id: str) -> np.ndarray:
        """Retourne la matrice monde pour un ID de repère parent."""
        if parent_id == "" or parent_id == WorkpieceModel.FRAME_WORLD:
            return np.eye(4, dtype=float)

        if parent_id == WorkpieceModel.FRAME_ROBOT:
            return get_effective_robot_base_in_world(self.workspace_model, self.external_axes_model)

        if parent_id.startswith(WorkpieceModel.PREFIX_EXT):
            axis_id = parent_id[len(WorkpieceModel.PREFIX_EXT):]
            transforms = self.external_axes_model.compute_world_transforms()
            t = transforms.get(axis_id)
            if t is not None:
                return t["end"]
            return np.eye(4, dtype=float)

        if parent_id.startswith(WorkpieceModel.PREFIX_WS):
            elem_name = parent_id[len(WorkpieceModel.PREFIX_WS):]
            for elem in self.workspace_model.get_workspace_cad_elements():
                if elem.name == elem_name:
                    return pose_zyx_to_matrix(elem.pose)
            return np.eye(4, dtype=float)

        return np.eye(4, dtype=float)

    def _compute_tooling_world_transforms(self) -> list[dict]:
        """
        Calcule les matrices monde de chaque élément d'outillage.

        Chaîne : pour l'élément i :
          T_world_cao_i   = T_world_frame_{i-1} · pose_in_prev_i   (frame_{-1} = parent outillage)
          T_world_frame_i = T_world_cao_i       · element_frame_pose_i
        """
        T_parent = self._get_parent_world_transform(self.tooling_model.get_parent_frame_id())
        results = []
        T_prev_frame = T_parent
        for elem in self.tooling_model.get_elements():
            T_cao = T_prev_frame @ pose_zyx_to_matrix(elem.pose_in_prev)
            T_frame = T_cao @ pose_zyx_to_matrix(elem.element_frame_pose)
            results.append({
                "cad_model": elem.cad_model,
                "T_world": T_cao,
                "color": elem.cad_color,
                "frame_T_world": T_frame,
            })
            T_prev_frame = T_frame
        return results

    def _get_tooling_frame(self) -> np.ndarray:
        """Retourne la matrice monde du repère outillage (dernier élément)."""
        transforms = self._compute_tooling_world_transforms()
        if transforms:
            return transforms[-1]["frame_T_world"]
        # Pas d'éléments : repère parent de l'outillage
        return self._get_parent_world_transform(self.tooling_model.get_parent_frame_id())

    def _compute_piece_world_transform(self) -> np.ndarray:
        parent_id = self.workpiece_model.get_parent_frame_id()
        T_pose = pose_zyx_to_matrix(self.workpiece_model.get_pose_in_parent())

        if parent_id == WorkpieceModel.FRAME_TOOLING:
            return self._get_tooling_frame() @ T_pose

        return self._get_parent_world_transform(parent_id) @ T_pose

    def compute_workpiece_frame_in_robot(self) -> np.ndarray:
        """Retourne T_workpiece_frame dans le repère base robot (matrice 4×4).

        Chaîne : T_robot_inv × T_piece_world × T_workpiece_frame_local
        """
        T_piece_world = self._compute_piece_world_transform()
        T_frame_world = T_piece_world @ pose_zyx_to_matrix(self.workpiece_model.get_workpiece_frame_pose())
        T_robot_world = get_effective_robot_base_in_world(self.workspace_model, self.external_axes_model)
        T_robot_world_inv = np.linalg.inv(T_robot_world)
        return T_robot_world_inv @ T_frame_world

    # ------------------------------------------------------------------
    # Viewer
    # ------------------------------------------------------------------

    def _refresh_viewer(self) -> None:
        viewer = getattr(self.viewer3d_controller, "viewer_3d_widget", None)
        if viewer is None:
            return

        # Outillage
        tooling_elems = self._compute_tooling_world_transforms()
        viewer.reload_tooling(tooling_elems)

        # Pièce
        cad_model = self.workpiece_model.get_cad_model()
        color = self.workpiece_model.get_cad_color()
        T_world = self._compute_piece_world_transform()
        frame_T_world = T_world @ pose_zyx_to_matrix(self.workpiece_model.get_workpiece_frame_pose())
        viewer.reload_workpiece(cad_model, T_world, color, frame_T_world)

    # ------------------------------------------------------------------
    # Sauvegarde / chargement
    # ------------------------------------------------------------------

    def _on_clear_piece(self) -> None:
        self.workpiece_model.from_dict({})
        self._piece_widget.set_data(self.workpiece_model)
        self._refresh_viewer()
        self._refresh_configuration_status()

    def _on_save(self) -> None:
        if not self._current_config_file:
            self._on_save_as()
            return
        self._save_to_path(self._current_config_file)

    def _on_save_as(self) -> None:
        os.makedirs(_DEFAULT_CONFIG_DIR, exist_ok=True)
        path, _ = QFileDialog.getSaveFileName(
            self.view,
            "Enregistrer une configuration pièce",
            _DEFAULT_CONFIG_DIR,
            "JSON (*.json);;All files (*)",
        )
        if not path:
            return
        if not path.lower().endswith(".json"):
            path += ".json"
        self._save_to_path(path)

    def _save_to_path(self, path: str) -> bool:
        try:
            data = self.get_serializable_state()
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=4)
        except OSError as exc:
            QMessageBox.warning(self.view, "Erreur sauvegarde", f"Impossible d'enregistrer la configuration pièce.\n{exc}")
            return False
        self._current_config_file = str(path)
        self._current_config_display_name = os.path.basename(self._current_config_file)
        self._mark_current_configuration_as_reference(STATUS_SAVED)
        self._refresh_configuration_header()
        return True

    def _on_load(self) -> None:
        os.makedirs(_DEFAULT_CONFIG_DIR, exist_ok=True)
        path, _ = QFileDialog.getOpenFileName(
            self.view,
            "Charger une configuration pièce",
            _DEFAULT_CONFIG_DIR,
            "JSON (*.json);;All files (*)",
        )
        if not path:
            return
        self.load_configuration_from_path(path, show_errors=True)

    def _on_new(self) -> None:
        self.tooling_model.from_dict({})
        self.workpiece_model.from_dict({})
        self._current_config_file = ""
        self._current_config_display_name = ""
        self._reference_data = None
        self._has_reference = False
        self._was_dirty_since_reference = False
        self._clean_status_text = STATUS_NONE
        self._refresh_parent_frames()
        self._tooling_panel.set_from_model(self.tooling_model)
        self._piece_widget.set_data(self.workpiece_model)
        self._refresh_viewer()
        self._refresh_configuration_status()

    def load_configuration_from_path(self, path: str, show_errors: bool = False) -> bool:
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.restore_state(data)
        except (OSError, ValueError, KeyError) as exc:
            if show_errors:
                QMessageBox.warning(
                    self.view,
                    "Erreur chargement",
                    f"Impossible de charger la configuration pièce.\n{exc}",
                )
            return False
        self._current_config_file = str(path)
        self._current_config_display_name = os.path.basename(self._current_config_file)
        self._mark_current_configuration_as_reference(STATUS_LOADED)
        self._refresh_configuration_header()
        return True

    # ------------------------------------------------------------------
    # Sérialisation
    # ------------------------------------------------------------------

    def get_serializable_state(self) -> dict:
        return {
            "tooling": self.tooling_model.to_dict(),
            "workpiece": self.workpiece_model.to_dict(),
        }

    def restore_state(
        self,
        data: dict,
        mark_as_reference: bool = False,
        clean_status_text: str = STATUS_LOADED,
        display_name: str = "",
    ) -> None:
        if not data:
            return
        tooling_data = data.get("tooling") or {}
        workpiece_data = data.get("workpiece") or {}
        if tooling_data:
            self.tooling_model.from_dict(tooling_data)
        if workpiece_data:
            self.workpiece_model.from_dict(workpiece_data)
        self._refresh_parent_frames()
        self._tooling_panel.set_from_model(self.tooling_model)
        self._piece_widget.set_data(self.workpiece_model)
        self._refresh_viewer()
        if mark_as_reference:
            self._current_config_file = ""
            self._current_config_display_name = str(display_name or "").strip()
            self._mark_current_configuration_as_reference(clean_status_text)
        else:
            self._refresh_configuration_status()
