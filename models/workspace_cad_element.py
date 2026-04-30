from __future__ import annotations

from models.types import Pose6


class WorkspaceCadElement:
    def __init__(self, name: str, cad_model: str, pose: Pose6 | None = None) -> None:
        if pose is not None and not isinstance(pose, Pose6):
            raise TypeError("pose must be a Pose6")
        normalized_name = str(name).strip()
        self.name = normalized_name if normalized_name != "" else "Element"
        self.cad_model = str(cad_model)
        self.pose = Pose6.zeros() if pose is None else pose.copy()

    def copy(self) -> "WorkspaceCadElement":
        return WorkspaceCadElement(self.name, self.cad_model, self.pose)

    def __eq__(self, other: object) -> bool:
        return (
            isinstance(other, WorkspaceCadElement)
            and self.name == other.name
            and self.cad_model == other.cad_model
            and self.pose == other.pose
        )
