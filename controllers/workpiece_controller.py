"""Contrôleur pour l'onglet Pièce."""
from __future__ import annotations

import numpy as np
from PyQt6.QtCore import QObject

from models.external_axes_model import ExternalAxesModel
from models.types.pose6 import Pose6
from models.workspace_model import WorkspaceModel
from models.workpiece_model import WorkpieceModel
from utils.math_utils import pose_zyx_to_matrix
from views.workpiece_view import WorkpieceView
from widgets.workpiece_view.workpiece_config_widget import WorkpieceConfigWidget


class WorkpieceController(QObject):
    def __init__(
        self,
        workpiece_model: WorkpieceModel,
        workspace_model: WorkspaceModel,
        external_axes_model: ExternalAxesModel,
        workpiece_view: WorkpieceView,
        viewer3d_controller,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.model = workpiece_model
        self.workspace_model = workspace_model
        self.external_axes_model = external_axes_model
        self.view = workpiece_view
        self.viewer3d_controller = viewer3d_controller
        self._widget: WorkpieceConfigWidget = workpiece_view.get_config_widget()

        self._setup_connections()
        self._refresh_parent_frames()
        self._widget.set_data(self.model)
        self._refresh_viewer()

    # ------------------------------------------------------------------
    # Connexions
    # ------------------------------------------------------------------

    def _setup_connections(self) -> None:
        self._widget.config_changed.connect(self._on_widget_changed)
        self.model.workpiece_changed.connect(self._on_model_changed)
        self.external_axes_model.axes_changed.connect(self._on_context_changed)
        self.external_axes_model.axes_values_changed.connect(self._refresh_viewer)
        self.external_axes_model.mount_topology_changed.connect(self._refresh_viewer)
        self.workspace_model.workspace_changed.connect(self._on_context_changed)

    # ------------------------------------------------------------------
    # Réactions
    # ------------------------------------------------------------------

    def _on_widget_changed(self) -> None:
        data = self._widget.get_data()
        self.model.blockSignals(True)
        self.model.set_cad_model(data["cad_model"])
        self.model.set_cad_color(tuple(data["cad_color"]))
        self.model.set_parent_frame_id(data["parent_frame_id"])
        self.model.set_pose_in_parent(Pose6(*data["pose_in_parent"]))
        self.model.set_workpiece_frame_pose(Pose6(*data["workpiece_frame_pose"]))
        self.model.blockSignals(False)
        self._refresh_viewer()

    def _on_model_changed(self) -> None:
        self._widget.set_data(self.model)
        self._refresh_viewer()

    def _on_context_changed(self) -> None:
        """Les axes externes ou le workspace ont changé → rebâtir les combos + viewer."""
        self._refresh_parent_frames()
        self._refresh_viewer()

    # ------------------------------------------------------------------
    # Mise à jour UI
    # ------------------------------------------------------------------

    def _refresh_parent_frames(self) -> None:
        axes = self.external_axes_model.get_axes()
        ext_axes = [(a.id, a.name) for a in axes]
        elements = self.workspace_model.get_workspace_cad_elements()
        ws_elements = [e.name for e in elements if e.name]
        self._widget.update_parent_frames(ext_axes, ws_elements)

    # ------------------------------------------------------------------
    # Viewer
    # ------------------------------------------------------------------

    def _refresh_viewer(self) -> None:
        viewer = getattr(self.viewer3d_controller, "viewer_3d_widget", None)
        if viewer is None:
            return
        cad_model = self.model.get_cad_model()
        color = self.model.get_cad_color()
        T_world = self._compute_world_transform()
        frame_T_world = T_world @ pose_zyx_to_matrix(self.model.get_workpiece_frame_pose())
        viewer.reload_workpiece(cad_model, T_world, color, frame_T_world)

    def _compute_world_transform(self) -> np.ndarray:
        T_pose = pose_zyx_to_matrix(self.model.get_pose_in_parent())
        parent_id = self.model.get_parent_frame_id()

        if parent_id == "" or parent_id == WorkpieceModel.FRAME_WORLD:
            return T_pose

        if parent_id == WorkpieceModel.FRAME_ROBOT:
            viewer = getattr(self.viewer3d_controller, "viewer_3d_widget", None)
            if viewer is not None:
                T_robot = viewer._get_robot_base_world_transform()
                return T_robot @ T_pose
            return T_pose

        if parent_id.startswith(WorkpieceModel.PREFIX_EXT):
            axis_id = parent_id[len(WorkpieceModel.PREFIX_EXT):]
            transforms = self.external_axes_model.compute_world_transforms()
            t = transforms.get(axis_id)
            if t is not None:
                return t["end"] @ T_pose
            return T_pose

        if parent_id.startswith(WorkpieceModel.PREFIX_WS):
            elem_name = parent_id[len(WorkpieceModel.PREFIX_WS):]
            for elem in self.workspace_model.get_workspace_cad_elements():
                if elem.name == elem_name:
                    T_elem = pose_zyx_to_matrix(elem.pose)
                    return T_elem @ T_pose
            return T_pose

        return T_pose

    # ------------------------------------------------------------------
    # Sérialisation
    # ------------------------------------------------------------------

    def get_serializable_state(self) -> dict:
        return self.model.to_dict()

    def restore_state(self, data: dict) -> None:
        if data:
            self.model.from_dict(data)
            self._refresh_parent_frames()
            self._widget.set_data(self.model)
            self._refresh_viewer()
