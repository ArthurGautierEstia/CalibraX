from __future__ import annotations

from dataclasses import dataclass, field
import json
from typing import Any, TYPE_CHECKING

from models.collider_models import parse_primitive_colliders, primitive_collider_to_dict
from utils.reference_frame_utils import normalize_pose6

if TYPE_CHECKING:
    from models.workspace_model import WorkspaceModel


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _normalize_pose(raw_pose: Any) -> list[float]:
    if isinstance(raw_pose, dict):
        values = [
            _safe_float(raw_pose.get("x", 0.0), 0.0),
            _safe_float(raw_pose.get("y", 0.0), 0.0),
            _safe_float(raw_pose.get("z", 0.0), 0.0),
            _safe_float(raw_pose.get("a", 0.0), 0.0),
            _safe_float(raw_pose.get("b", 0.0), 0.0),
            _safe_float(raw_pose.get("c", 0.0), 0.0),
        ]
    elif isinstance(raw_pose, list):
        values = [_safe_float(raw_pose[idx] if idx < len(raw_pose) else 0.0, 0.0) for idx in range(6)]
    else:
        values = [0.0] * 6
    return values[:6]


def normalize_workspace_cad_element(raw_value: Any, default_name: str = "Element") -> dict[str, Any]:
    data = raw_value if isinstance(raw_value, dict) else {}
    cad_model = data.get("cad_model", data.get("stl_file", data.get("file", "")))
    pose = data.get("pose", data.get("xyzabc", data.get("transform")))

    return {
        "name": str(data.get("name", default_name)),
        "cad_model": "" if cad_model is None else str(cad_model),
        "pose": _normalize_pose(pose),
    }


def parse_workspace_cad_elements(raw_values: Any) -> list[dict[str, Any]]:
    values = raw_values if isinstance(raw_values, list) else []
    parsed: list[dict[str, Any]] = []
    for idx, raw_value in enumerate(values):
        parsed.append(normalize_workspace_cad_element(raw_value, default_name=f"Element {idx + 1}"))
    return parsed


@dataclass
class WorkspaceFile:
    scene_name: str = ""
    robot_base_pose_world: list[float] = field(default_factory=lambda: [0.0] * 6)
    cad_elements: list[dict[str, Any]] = field(default_factory=list)
    tcp_zones: list[dict[str, Any]] = field(default_factory=list)
    collision_zones: list[dict[str, Any]] = field(default_factory=list)

    @classmethod
    def from_workspace_model(cls, workspace_model: "WorkspaceModel") -> "WorkspaceFile":
        return cls(
            scene_name=workspace_model.get_workspace_scene_name(),
            robot_base_pose_world=normalize_pose6(workspace_model.get_robot_base_pose_world()),
            cad_elements=[normalize_workspace_cad_element(v) for v in workspace_model.get_workspace_cad_elements()],
            tcp_zones=parse_primitive_colliders(workspace_model.get_workspace_tcp_zones(), default_shape="box"),
            collision_zones=parse_primitive_colliders(
                workspace_model.get_workspace_collision_zones(),
                default_shape="box",
            ),
        )

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "WorkspaceFile":
        if not isinstance(data, dict):
            raise TypeError("Le workspace doit etre un dictionnaire JSON.")

        scene_name = "" if data.get("scene_name") is None else str(data.get("scene_name"))
        if scene_name == "":
            scene_name = "" if data.get("name") is None else str(data.get("name"))

        return cls(
            scene_name=scene_name,
            robot_base_pose_world=normalize_pose6(
                data.get("robot_base_pose_world", data.get("robot_pose", data.get("base_pose")))
            ),
            cad_elements=parse_workspace_cad_elements(data.get("cad_elements", data.get("elements"))),
            tcp_zones=parse_primitive_colliders(data.get("tcp_zones", data.get("zones_tcp")), default_shape="box"),
            collision_zones=parse_primitive_colliders(
                data.get("collision_zones", data.get("zones_collision")),
                default_shape="box",
            ),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "scene_name": self.scene_name,
            "robot_base_pose_world": normalize_pose6(self.robot_base_pose_world),
            "cad_elements": [normalize_workspace_cad_element(v) for v in self.cad_elements],
            "tcp_zones": [primitive_collider_to_dict(v) for v in self.tcp_zones],
            "collision_zones": [primitive_collider_to_dict(v) for v in self.collision_zones],
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
