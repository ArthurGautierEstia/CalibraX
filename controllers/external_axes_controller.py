"""Contrôleur pour l'onglet Axes externes."""
from __future__ import annotations

import numpy as np
from PyQt6.QtCore import QObject

from models.external_axis import ExternalAxis
from models.external_axes_model import ExternalAxesModel
from models.workspace_model import WorkspaceModel
from utils.reference_frame_utils import FrameTransform
from utils.math_utils import matrix_to_pose_zyx
from views.external_axes_view import ExternalAxesView
from widgets.external_axes_view.external_axes_panel_widget import ExternalAxesPanelWidget


class ExternalAxesController(QObject):
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
        self._setup_connections()
        self._refresh_view()

    # ------------------------------------------------------------------
    # Connexions
    # ------------------------------------------------------------------

    def _setup_connections(self) -> None:
        # Vue → modèle
        self._panel.axis_added.connect(self._on_axis_added)
        self._panel.axis_removed.connect(self._on_axis_removed)
        self._panel.axis_updated.connect(self._on_axis_updated)
        self._panel.robot_mount_parent_changed.connect(self._on_robot_mount_changed)

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
        self._refresh_viewer_cad()

    def _on_model_values_changed(self) -> None:
        self._apply_robot_base_override()
        self._refresh_viewer_poses()

    def _on_model_topology_changed(self) -> None:
        self._apply_robot_base_override()
        self._refresh_viewer_poses()

    # ------------------------------------------------------------------
    # Utilitaires
    # ------------------------------------------------------------------

    def _refresh_view(self) -> None:
        self._panel.refresh_from_model(self.model)

    def _apply_robot_base_override(self) -> None:
        """Met à jour le viewer3D avec la nouvelle base robot monde."""
        override = self.model.get_robot_world_base_matrix()
        if hasattr(self.viewer3d_controller, 'viewer_3d_widget'):
            self.viewer3d_controller.viewer_3d_widget.set_external_robot_base_override(override)

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

    # ------------------------------------------------------------------
    # Sérialisation
    # ------------------------------------------------------------------

    def get_serializable_state(self) -> dict:
        return self.model.to_dict()

    def restore_state(self, data: dict) -> None:
        if data:
            self.model.from_dict(data)
