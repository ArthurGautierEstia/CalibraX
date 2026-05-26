from __future__ import annotations

import numpy as np
from PyQt6.QtCore import QObject, pyqtSignal

from models.external_axis import ExternalAxis


class ExternalAxesModel(QObject):
    """Modèle centralisant tous les axes externes configurés.

    Signaux:
    - axes_changed      : structure modifiée (ajout/suppression/rename/poses/CAO)
    - axes_values_changed : valeurs q_i modifiées (fréquent, pendant le jog)
    - mount_topology_changed : lien parent modifié pour un axe ou pour le robot/pièce
    """

    axes_changed = pyqtSignal()
    axes_values_changed = pyqtSignal()
    mount_topology_changed = pyqtSignal()

    # ID spécial représentant "robot" comme charge montée
    ROBOT_MOUNT_ID = "__robot__"

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._axes: list[ExternalAxis] = []
        self._robot_mount_parent_id: str | None = None

    # ------------------------------------------------------------------
    # CRUD axes
    # ------------------------------------------------------------------

    def get_axes(self) -> list[ExternalAxis]:
        return [a.copy() for a in self._axes]

    def get_axis(self, axis_id: str) -> ExternalAxis | None:
        for a in self._axes:
            if a.id == axis_id:
                return a.copy()
        return None

    def add_axis(self, axis: ExternalAxis) -> None:
        self._axes.append(axis.copy())
        self.axes_changed.emit()

    def remove_axis(self, axis_id: str) -> None:
        for i, a in enumerate(self._axes):
            if a.id == axis_id:
                self._axes.pop(i)
                # Nettoyer les références parentales
                for other in self._axes:
                    if other.mount_parent_id == axis_id:
                        other.mount_parent_id = None
                if self._robot_mount_parent_id == axis_id:
                    self._robot_mount_parent_id = None
                    self.mount_topology_changed.emit()
                self.axes_changed.emit()
                return

    def update_axis(self, axis_id: str, axis: ExternalAxis) -> None:
        for i, a in enumerate(self._axes):
            if a.id == axis_id:
                new_axis = axis.copy()
                new_axis.id = axis_id  # conserver l'ID stable
                old = self._axes[i]
                self._axes[i] = new_axis
                if old.mount_parent_id != new_axis.mount_parent_id:
                    self.mount_topology_changed.emit()
                self.axes_changed.emit()
                return

    def reorder_axes(self, new_order: list[str]) -> None:
        id_to_axis = {a.id: a for a in self._axes}
        reordered = [id_to_axis[aid] for aid in new_order if aid in id_to_axis]
        if len(reordered) != len(self._axes):
            return
        self._axes = reordered
        self.axes_changed.emit()

    # ------------------------------------------------------------------
    # Valeurs articulaires (q_i)
    # ------------------------------------------------------------------

    def set_axis_joint_value(self, axis_id: str, joint_index: int, value: float) -> None:
        for a in self._axes:
            if a.id == axis_id:
                if joint_index < 0 or joint_index >= len(a.joints):
                    return
                j = a.joints[joint_index]
                clamped = float(np.clip(value, j.q_min, j.q_max))
                if clamped == j.value:
                    return
                j.value = clamped
                self.axes_values_changed.emit()
                return

    def get_axis_joint_value(self, axis_id: str, joint_index: int) -> float:
        for a in self._axes:
            if a.id == axis_id and joint_index < len(a.joints):
                return a.joints[joint_index].value
        return 0.0

    # ------------------------------------------------------------------
    # Montage robot
    # ------------------------------------------------------------------

    def get_robot_mount_parent_id(self) -> str | None:
        return self._robot_mount_parent_id

    def set_robot_mount_parent_id(self, axis_id: str | None) -> None:
        if axis_id == self._robot_mount_parent_id:
            return
        self._robot_mount_parent_id = axis_id
        self.mount_topology_changed.emit()

    # ------------------------------------------------------------------
    # Calcul des transformations monde (résolution topologique)
    # ------------------------------------------------------------------

    def compute_world_transforms(self) -> dict[str, dict]:
        """Calcule les matrices monde pour tous les axes (tri topologique).

        Returns:
            dict axis_id -> {
                "base": np.ndarray 4×4,
                "joint_links": [np.ndarray ...],
                "end": np.ndarray 4×4,
            }
        """
        world_transforms: dict[str, dict] = {}
        IDENTITY = np.eye(4, dtype=float)

        # Tri topologique simple (liste déjà ordonnée en général)
        remaining = list(self._axes)
        max_iters = len(remaining) + 1
        iters = 0
        while remaining:
            iters += 1
            if iters > max_iters:
                # Cycle détecté ou axe parent introuvable — on place à l'origine
                for a in remaining:
                    world_transforms[a.id] = a.compute_chain(IDENTITY)
                break
            pending_again = []
            for a in remaining:
                if a.mount_parent_id is None:
                    parent_T = IDENTITY
                elif a.mount_parent_id in world_transforms:
                    parent_T = world_transforms[a.mount_parent_id]["end"]
                else:
                    pending_again.append(a)
                    continue
                world_transforms[a.id] = a.compute_chain(parent_T)
            if len(pending_again) == len(remaining):
                # Pas de progrès → cycle
                for a in pending_again:
                    world_transforms[a.id] = a.compute_chain(IDENTITY)
                break
            remaining = pending_again

        return world_transforms

    def get_robot_world_base_matrix(self) -> np.ndarray:
        """Retourne T_world_robotBase en tenant compte du montage sur axe externe."""
        if self._robot_mount_parent_id is None:
            return None  # pas de montage, sera géré par workspace_model
        transforms = self.compute_world_transforms()
        if self._robot_mount_parent_id in transforms:
            return transforms[self._robot_mount_parent_id]["end"].copy()
        return None

    # ------------------------------------------------------------------
    # Sérialisation
    # ------------------------------------------------------------------

    def to_dict(self) -> dict:
        return {
            "axes": [a.to_dict() for a in self._axes],
            "robot_mount_parent_id": self._robot_mount_parent_id,
        }

    def from_dict(self, data: dict) -> None:
        self._axes = [ExternalAxis.from_dict(d) for d in data.get("axes", [])]
        self._robot_mount_parent_id = data.get("robot_mount_parent_id")
        self.axes_changed.emit()
        self.mount_topology_changed.emit()
