from __future__ import annotations

import uuid

from models.types.pose6 import Pose6


class ToolingElement:
    """Un élément constituant l'outillage.

    Chaîne cinématique :
        T_world_cao_i = T_world_frame_{i-1}  · pose_in_prev   (T_world_parent pour i=0)
        T_world_frame_i = T_world_cao_i      · element_frame_pose

    Le repère de l'élément (element_frame_pose) sert de repère parent
    pour l'élément suivant. Le repère du dernier élément = repère outillage.
    """

    DEFAULT_COLOR: tuple[float, float, float, float] = (0.45, 0.45, 0.50, 1.0)

    def __init__(
        self,
        name: str = "Élément",
        element_id: str | None = None,
        cad_model: str = "",
        cad_color: tuple[float, float, float, float] = DEFAULT_COLOR,
        pose_in_prev: Pose6 | None = None,
        element_frame_pose: Pose6 | None = None,
    ) -> None:
        self.id: str = str(element_id) if element_id else str(uuid.uuid4())
        self.name: str = str(name).strip() or "Élément"
        self.cad_model: str = str(cad_model)
        self.cad_color: tuple[float, float, float, float] = tuple(float(c) for c in cad_color)
        self.pose_in_prev: Pose6 = (pose_in_prev or Pose6.zeros()).copy()
        self.element_frame_pose: Pose6 = (element_frame_pose or Pose6.zeros()).copy()

    def copy(self) -> "ToolingElement":
        return ToolingElement(
            name=self.name,
            element_id=self.id,
            cad_model=self.cad_model,
            cad_color=self.cad_color,
            pose_in_prev=self.pose_in_prev,
            element_frame_pose=self.element_frame_pose,
        )

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "name": self.name,
            "cad_model": self.cad_model,
            "cad_color": list(self.cad_color),
            "pose_in_prev": self.pose_in_prev.to_list(),
            "element_frame_pose": self.element_frame_pose.to_list(),
        }

    @classmethod
    def from_dict(cls, data: dict) -> "ToolingElement":
        color_raw = data.get("cad_color", list(cls.DEFAULT_COLOR))
        return cls(
            name=str(data.get("name", "Élément")),
            element_id=str(data.get("id", "")),
            cad_model=str(data.get("cad_model", "")),
            cad_color=tuple(float(c) for c in color_raw),
            pose_in_prev=Pose6(*[float(v) for v in data.get("pose_in_prev", [0] * 6)]),
            element_frame_pose=Pose6(*[float(v) for v in data.get("element_frame_pose", [0] * 6)]),
        )
