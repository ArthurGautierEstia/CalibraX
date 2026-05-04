from __future__ import annotations

from dataclasses import dataclass, field
import json
import os
from typing import Any

from models.primitive_collider_models import PrimitiveColliderData, PrimitiveColliderShape
from models.types import Pose6
from utils.math_utils import safe_float
from utils.mgi import RobotTool


def _require_mapping(data: Any, name: str) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise TypeError(f"{name} must be a JSON object")
    return data


def _require_list(data: Any, name: str, length: int | None = None) -> list[Any]:
    if not isinstance(data, list) or len(data) != length:
        if length is None and isinstance(data, list):
            return data
        raise ValueError(f"{name} must be a list of {length} values")
    return data


def _parse_pose6_mapping(data: Any, name: str) -> Pose6:
    values = _require_mapping(data, name)
    required = ("x", "y", "z", "a", "b", "c")
    missing = [key for key in required if key not in values]
    if missing:
        raise ValueError(f"{name} is missing keys: {', '.join(missing)}")
    return Pose6(*(safe_float(values[key], 0.0) for key in required))


def _parse_pose6_list(data: Any, name: str) -> Pose6:
    values = _require_list(data, name, 6)
    return Pose6(*(safe_float(value, 0.0) for value in values))


def _parse_shape(data: Any, name: str) -> PrimitiveColliderShape:
    if not isinstance(data, str):
        raise TypeError(f"{name} must be a string")
    return PrimitiveColliderShape(data)


def _parse_primitive_collider(data: Any, name: str) -> PrimitiveColliderData:
    values = _require_mapping(data, name)
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


def _parse_bool_list_6(data: Any, name: str) -> list[bool]:
    values = _require_list(data, name, 6)
    if not all(isinstance(value, bool) for value in values):
        raise TypeError(f"{name} must contain booleans only")
    return list(values)


@dataclass
class ToolConfigFile:
    name: str = ""
    tool: Pose6 = field(default_factory=Pose6.zeros)
    tool_cad_model: str = ""
    tool_cad_offset_rz: float = 0.0
    auto_load_on_startup: bool = False
    tool_colliders: list[PrimitiveColliderData] = field(default_factory=list)
    evaluated_robot_axis_colliders: list[bool] = field(default_factory=lambda: [True] * 6)

    def __post_init__(self) -> None:
        if not isinstance(self.tool, Pose6):
            raise TypeError("tool must be a Pose6")
        if not all(isinstance(collider, PrimitiveColliderData) for collider in self.tool_colliders):
            raise TypeError("tool_colliders must contain PrimitiveColliderData")
        if len(self.evaluated_robot_axis_colliders) != 6:
            raise ValueError("evaluated_robot_axis_colliders must contain 6 booleans")
        self.name = str(self.name)
        self.tool = self.tool.copy()
        self.tool_cad_model = str(self.tool_cad_model)
        self.tool_cad_offset_rz = float(self.tool_cad_offset_rz)
        self.auto_load_on_startup = bool(self.auto_load_on_startup)
        self.tool_colliders = [collider.copy() for collider in self.tool_colliders]
        self.evaluated_robot_axis_colliders = [bool(value) for value in self.evaluated_robot_axis_colliders]

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ToolConfigFile:
        values = _require_mapping(data, "tool profile")
        required = (
            "name",
            "tool",
            "tool_cad_model",
            "tool_cad_offset_rz",
            "tool_colliders",
            "evaluated_robot_axis_colliders",
        )
        missing = [key for key in required if key not in values]
        if missing:
            raise ValueError(f"tool profile is missing keys: {', '.join(missing)}")

        return cls(
            name=str(values["name"]),
            tool=_parse_pose6_mapping(values["tool"], "tool"),
            tool_cad_model=str(values["tool_cad_model"]),
            tool_cad_offset_rz=safe_float(values["tool_cad_offset_rz"], 0.0),
            auto_load_on_startup=bool(values.get("auto_load_on_startup", False)),
            tool_colliders=_parse_primitive_colliders(values["tool_colliders"], "tool_colliders"),
            evaluated_robot_axis_colliders=_parse_bool_list_6(
                values["evaluated_robot_axis_colliders"],
                "evaluated_robot_axis_colliders",
            ),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "tool": {
                "x": float(self.tool.x),
                "y": float(self.tool.y),
                "z": float(self.tool.z),
                "a": float(self.tool.a),
                "b": float(self.tool.b),
                "c": float(self.tool.c),
            },
            "tool_cad_model": self.tool_cad_model,
            "tool_cad_offset_rz": float(self.tool_cad_offset_rz),
            "auto_load_on_startup": bool(self.auto_load_on_startup),
            "tool_colliders": [_primitive_collider_to_dict(collider) for collider in self.tool_colliders],
            "evaluated_robot_axis_colliders": [bool(value) for value in self.evaluated_robot_axis_colliders],
        }

    def to_robot_tool(self) -> RobotTool:
        return RobotTool(*self.tool.to_tuple())

    @classmethod
    def from_robot_tool(
        cls,
        name: str,
        robot_tool: RobotTool,
        tool_cad_model: str,
        tool_cad_offset_rz: float,
        auto_load_on_startup: bool,
        tool_colliders: list[PrimitiveColliderData] | None = None,
        evaluated_robot_axis_colliders: list[bool] | None = None,
    ) -> ToolConfigFile:
        return cls(
            name=name,
            tool=Pose6(
                robot_tool.x,
                robot_tool.y,
                robot_tool.z,
                robot_tool.a,
                robot_tool.b,
                robot_tool.c,
            ),
            tool_cad_model=tool_cad_model,
            tool_cad_offset_rz=float(tool_cad_offset_rz),
            auto_load_on_startup=bool(auto_load_on_startup),
            tool_colliders=[] if tool_colliders is None else tool_colliders,
            evaluated_robot_axis_colliders=(
                [True] * 6 if evaluated_robot_axis_colliders is None else evaluated_robot_axis_colliders
            ),
        )

    def save(self, file_path: str) -> None:
        with open(file_path, "w", encoding="utf-8") as file:
            json.dump(self.to_dict(), file, indent=4)

    @classmethod
    def load(cls, file_path: str) -> ToolConfigFile:
        with open(file_path, "r", encoding="utf-8") as file:
            data = json.load(file)
        result = cls.from_dict(data)
        if not result.name:
            result.name = os.path.splitext(os.path.basename(file_path))[0]
        return result
