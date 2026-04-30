from __future__ import annotations

from dataclasses import dataclass, field
import json
from typing import Any, TYPE_CHECKING

from models.primitive_collider_models import PrimitiveColliderData, PrimitiveColliderShape
from models.types import Pose6
from models.workspace_cad_element import WorkspaceCadElement
from utils.math_utils import safe_float

if TYPE_CHECKING:
    from models.workspace_model import WorkspaceModel


def _require_mapping(data: Any, name: str) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise TypeError(f"{name} must be a JSON object")
    return data


def _require_list(data: Any, name: str, length: int | None = None) -> list[Any]:
    if not isinstance(data, list):
        raise TypeError(f"{name} must be a JSON list")
    if length is not None and len(data) != length:
        raise ValueError(f"{name} must contain {length} values")
    return data


def _parse_pose6_list(data: Any, name: str) -> Pose6:
    values = _require_list(data, name, 6)
    return Pose6(*(safe_float(value, 0.0) for value in values))


def _parse_shape(data: Any, name: str) -> PrimitiveColliderShape:
    if not isinstance(data, str):
        raise TypeError(f"{name} must be a string")
    return PrimitiveColliderShape(data)


def _parse_workspace_cad_element(data: Any, name: str) -> WorkspaceCadElement:
    values = _require_mapping(data, name)
    required = ("name", "cad_model", "pose")
    missing = [key for key in required if key not in values]
    if missing:
        raise ValueError(f"{name} is missing keys: {', '.join(missing)}")
    return WorkspaceCadElement(
        name=str(values["name"]),
        cad_model=str(values["cad_model"]),
        pose=_parse_pose6_list(values["pose"], f"{name}.pose"),
    )


def _workspace_cad_element_to_dict(element: WorkspaceCadElement) -> dict[str, Any]:
    return {
        "name": element.name,
        "cad_model": element.cad_model,
        "pose": element.pose.to_list(),
    }


def _parse_primitive_collider(data: Any, name: str) -> PrimitiveColliderData:
    values = _require_mapping(data, name)
    required = (
        "name",
        "enabled",
        "shape",
        "pose",
        "size_x",
        "size_y",
        "size_z",
        "radius",
        "height",
    )
    missing = [key for key in required if key not in values]
    if missing:
        raise ValueError(f"{name} is missing keys: {', '.join(missing)}")
    return PrimitiveColliderData(
        name=str(values["name"]),
        enabled=bool(values["enabled"]),
        shape=_parse_shape(values["shape"], f"{name}.shape"),
        pose=_parse_pose6_list(values["pose"], f"{name}.pose"),
        size_x=safe_float(values["size_x"], 0.0),
        size_y=safe_float(values["size_y"], 0.0),
        size_z=safe_float(values["size_z"], 0.0),
        radius=safe_float(values["radius"], 0.0),
        height=safe_float(values["height"], 0.0),
    )


def _parse_primitive_colliders(data: Any, name: str) -> list[PrimitiveColliderData]:
    values = _require_list(data, name)
    return [_parse_primitive_collider(value, f"{name}[{index}]") for index, value in enumerate(values)]


def _primitive_collider_to_dict(collider: PrimitiveColliderData) -> dict[str, Any]:
    return {
        "name": collider.name,
        "enabled": collider.enabled,
        "shape": collider.shape.value,
        "pose": collider.pose.to_list(),
        "size_x": float(collider.size_x),
        "size_y": float(collider.size_y),
        "size_z": float(collider.size_z),
        "radius": float(collider.radius),
        "height": float(collider.height),
    }


@dataclass
class WorkspaceFile:
    scene_name: str = ""
    robot_base_pose_world: Pose6 = field(default_factory=Pose6.zeros)
    cad_elements: list[WorkspaceCadElement] = field(default_factory=list)
    tcp_zones: list[PrimitiveColliderData] = field(default_factory=list)
    collision_zones: list[PrimitiveColliderData] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not isinstance(self.robot_base_pose_world, Pose6):
            raise TypeError("robot_base_pose_world must be a Pose6")
        if not all(isinstance(element, WorkspaceCadElement) for element in self.cad_elements):
            raise TypeError("cad_elements must contain WorkspaceCadElement")
        if not all(isinstance(zone, PrimitiveColliderData) for zone in self.tcp_zones):
            raise TypeError("tcp_zones must contain PrimitiveColliderData")
        if not all(isinstance(zone, PrimitiveColliderData) for zone in self.collision_zones):
            raise TypeError("collision_zones must contain PrimitiveColliderData")

        self.scene_name = str(self.scene_name)
        self.robot_base_pose_world = self.robot_base_pose_world.copy()
        self.cad_elements = [element.copy() for element in self.cad_elements]
        self.tcp_zones = [zone.copy() for zone in self.tcp_zones]
        self.collision_zones = [zone.copy() for zone in self.collision_zones]

    @classmethod
    def from_workspace_model(cls, workspace_model: "WorkspaceModel") -> "WorkspaceFile":
        return cls(
            scene_name=workspace_model.get_workspace_scene_name(),
            robot_base_pose_world=workspace_model.get_robot_base_pose_world(),
            cad_elements=workspace_model.get_workspace_cad_elements(),
            tcp_zones=workspace_model.get_workspace_tcp_zones(),
            collision_zones=workspace_model.get_workspace_collision_zones(),
        )

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "WorkspaceFile":
        values = _require_mapping(data, "workspace")
        required = ("scene_name", "robot_base_pose_world", "cad_elements", "tcp_zones", "collision_zones")
        missing = [key for key in required if key not in values]
        if missing:
            raise ValueError(f"workspace is missing keys: {', '.join(missing)}")

        return cls(
            scene_name=str(values["scene_name"]),
            robot_base_pose_world=_parse_pose6_list(values["robot_base_pose_world"], "robot_base_pose_world"),
            cad_elements=[
                _parse_workspace_cad_element(value, f"cad_elements[{index}]")
                for index, value in enumerate(_require_list(values["cad_elements"], "cad_elements"))
            ],
            tcp_zones=_parse_primitive_colliders(values["tcp_zones"], "tcp_zones"),
            collision_zones=_parse_primitive_colliders(values["collision_zones"], "collision_zones"),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "scene_name": self.scene_name,
            "robot_base_pose_world": self.robot_base_pose_world.to_list(),
            "cad_elements": [_workspace_cad_element_to_dict(element) for element in self.cad_elements],
            "tcp_zones": [_primitive_collider_to_dict(zone) for zone in self.tcp_zones],
            "collision_zones": [_primitive_collider_to_dict(zone) for zone in self.collision_zones],
        }

    def apply_to_workspace_model(self, workspace_model: "WorkspaceModel", file_path: str | None = None) -> None:
        workspace_model.set_workspace_data(
            scene_name=self.scene_name,
            robot_base_pose_world=self.robot_base_pose_world,
            cad_elements=self.cad_elements,
            tcp_zones=self.tcp_zones,
            collision_zones=self.collision_zones,
            file_path=file_path,
        )

    def save(self, file_path: str) -> None:
        with open(file_path, "w", encoding="utf-8") as file:
            json.dump(self.to_dict(), file, indent=4)

    @classmethod
    def load(cls, file_path: str) -> "WorkspaceFile":
        with open(file_path, "r", encoding="utf-8") as file:
            data = json.load(file)
        return cls.from_dict(data)
