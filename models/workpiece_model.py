from __future__ import annotations

from PyQt6.QtCore import QObject, pyqtSignal

from models.types.pose6 import Pose6


class WorkpieceModel(QObject):
    """Modèle de la pièce à usiner / à étalonner.

    Stocke la CAO, la couleur, le repère parent, la pose dans ce repère,
    et la définition du repère pièce (pour l'import de programmes).
    """

    workpiece_changed = pyqtSignal()

    # Préfixes pour les IDs de repère parent
    PREFIX_EXT = "ext:"    # axe externe : "ext:<axis_id>"
    PREFIX_WS = "ws:"      # élément workspace : "ws:<element_name>"
    FRAME_WORLD = ""
    FRAME_ROBOT = "robot"
    FRAME_TOOLING = "tooling"  # repère du dernier élément d'outillage

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._cad_model: str = ""
        self._cad_color: tuple[float, float, float, float] = (0.8, 0.6, 0.2, 1.0)
        self._parent_frame_id: str = ""
        self._pose_in_parent: Pose6 = Pose6.zeros()
        self._workpiece_frame_pose: Pose6 = Pose6.zeros()

    # ------------------------------------------------------------------
    # Accesseurs
    # ------------------------------------------------------------------

    def get_cad_model(self) -> str:
        return self._cad_model

    def set_cad_model(self, path: str) -> None:
        self._cad_model = str(path)
        self.workpiece_changed.emit()

    def get_cad_color(self) -> tuple[float, float, float, float]:
        return self._cad_color

    def set_cad_color(self, color: tuple) -> None:
        self._cad_color = tuple(float(c) for c in color)
        self.workpiece_changed.emit()

    def get_parent_frame_id(self) -> str:
        return self._parent_frame_id

    def set_parent_frame_id(self, frame_id: str) -> None:
        self._parent_frame_id = str(frame_id)
        self.workpiece_changed.emit()

    def get_pose_in_parent(self) -> Pose6:
        return self._pose_in_parent.copy()

    def set_pose_in_parent(self, pose: Pose6) -> None:
        self._pose_in_parent = pose.copy()
        self.workpiece_changed.emit()

    def get_workpiece_frame_pose(self) -> Pose6:
        return self._workpiece_frame_pose.copy()

    def set_workpiece_frame_pose(self, pose: Pose6) -> None:
        self._workpiece_frame_pose = pose.copy()
        self.workpiece_changed.emit()

    # ------------------------------------------------------------------
    # Sérialisation
    # ------------------------------------------------------------------

    def to_dict(self) -> dict:
        return {
            "cad_model": self._cad_model,
            "cad_color": list(self._cad_color),
            "parent_frame_id": self._parent_frame_id,
            "pose_in_parent": self._pose_in_parent.to_list(),
            "workpiece_frame_pose": self._workpiece_frame_pose.to_list(),
        }

    def from_dict(self, data: dict) -> None:
        self._cad_model = str(data.get("cad_model", ""))
        color = data.get("cad_color", [0.8, 0.6, 0.2, 1.0])
        self._cad_color = tuple(float(c) for c in color)
        self._parent_frame_id = str(data.get("parent_frame_id", ""))
        self._pose_in_parent = Pose6(*[float(v) for v in data.get("pose_in_parent", [0] * 6)])
        self._workpiece_frame_pose = Pose6(*[float(v) for v in data.get("workpiece_frame_pose", [0] * 6)])
        self.workpiece_changed.emit()
