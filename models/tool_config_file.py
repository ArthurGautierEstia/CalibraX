from __future__ import annotations

from dataclasses import dataclass
import json
import os
from typing import Any

from models.collider_models import parse_primitive_colliders, primitive_collider_to_dict
from utils.mgi import RobotTool


@dataclass
class ToolConfigFile:
    name: str = ""
    tool: list[float] | None = None
    tool_cad_model: str = ""
    tool_cad_offset_rz: float = 0.0
    tool_colliders: list[dict[str, Any]] | None = None
    evaluated_robot_axis_colliders: list[bool] | None = None

    def __post_init__(self) -> None:
        if self.tool is None:
            self.tool = [0.0] * 6
        values = [float(value) for value in self.tool[:6]]
        while len(values) < 6:
            values.append(0.0)
        self.tool = values
        self.tool_cad_model = "" if self.tool_cad_model is None else str(self.tool_cad_model)
        self.tool_cad_offset_rz = float(self.tool_cad_offset_rz)
        self.tool_colliders = parse_primitive_colliders(self.tool_colliders, default_shape="cylinder")
        self.evaluated_robot_axis_colliders = self._normalize_evaluated_robot_axis_colliders(
            self.evaluated_robot_axis_colliders
        )

    @staticmethod
    def _safe_float(value: Any, default: float = 0.0) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _safe_bool(value: Any, default: bool = True) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            lowered = value.strip().lower()
            if lowered in {"1", "true", "yes", "on", "enabled", "active"}:
                return True
            if lowered in {"0", "false", "no", "off", "disabled", "inactive"}:
                return False
        return default

    @classmethod
    def _normalize_evaluated_robot_axis_colliders(cls, values: Any) -> list[bool]:
        normalized: list[bool] = []
        raw_values = values if isinstance(values, list) else []
        for axis in range(6):
            raw_value = raw_values[axis] if axis < len(raw_values) else True
            normalized.append(cls._safe_bool(raw_value, True))
        return normalized

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ToolConfigFile:
        if not isinstance(data, dict):
            raise TypeError("Le tool doit etre un dictionnaire JSON.")

        name = "" if data.get("name") is None else str(data.get("name"))
        tool_raw = data.get("tool", data.get("xyzabc", {}))
        if isinstance(tool_raw, list):
            values = [cls._safe_float(tool_raw[idx] if idx < len(tool_raw) else 0.0) for idx in range(6)]
        elif isinstance(tool_raw, dict):
            values = [
                cls._safe_float(tool_raw.get("x", 0.0)),
                cls._safe_float(tool_raw.get("y", 0.0)),
                cls._safe_float(tool_raw.get("z", 0.0)),
                cls._safe_float(tool_raw.get("a", 0.0)),
                cls._safe_float(tool_raw.get("b", 0.0)),
                cls._safe_float(tool_raw.get("c", 0.0)),
            ]
        else:
            values = [0.0] * 6

        return cls(
            name=name,
            tool=values,
            tool_cad_model="" if data.get("tool_cad_model") is None else str(data.get("tool_cad_model")),
            tool_cad_offset_rz=cls._safe_float(data.get("tool_cad_offset_rz", 0.0), 0.0),
            tool_colliders=parse_primitive_colliders(data.get("tool_colliders"), default_shape="cylinder"),
            evaluated_robot_axis_colliders=cls._normalize_evaluated_robot_axis_colliders(
                data.get("evaluated_robot_axis_colliders")
            ),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "tool": {
                "x": float(self.tool[0]),
                "y": float(self.tool[1]),
                "z": float(self.tool[2]),
                "a": float(self.tool[3]),
                "b": float(self.tool[4]),
                "c": float(self.tool[5]),
            },
            "tool_cad_model": self.tool_cad_model,
            "tool_cad_offset_rz": float(self.tool_cad_offset_rz),
            "tool_colliders": [primitive_collider_to_dict(collider) for collider in self.tool_colliders],
            "evaluated_robot_axis_colliders": [bool(value) for value in self.evaluated_robot_axis_colliders[:6]],
        }

    def to_robot_tool(self) -> RobotTool:
        return RobotTool(*self.tool[:6])

    @classmethod
    def from_robot_tool(
        cls,
        name: str,
        robot_tool: RobotTool,
        tool_cad_model: str,
        tool_cad_offset_rz: float,
        tool_colliders: list[dict[str, Any]] | None = None,
        evaluated_robot_axis_colliders: list[bool] | None = None,
    ) -> ToolConfigFile:
        return cls(
            name=name,
            tool=[
                float(robot_tool.x),
                float(robot_tool.y),
                float(robot_tool.z),
                float(robot_tool.a),
                float(robot_tool.b),
                float(robot_tool.c),
            ],
            tool_cad_model=tool_cad_model,
            tool_cad_offset_rz=float(tool_cad_offset_rz),
            tool_colliders=parse_primitive_colliders(tool_colliders, default_shape="cylinder"),
            evaluated_robot_axis_colliders=cls._normalize_evaluated_robot_axis_colliders(
                evaluated_robot_axis_colliders
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
