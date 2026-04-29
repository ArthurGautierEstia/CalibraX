from __future__ import annotations

from typing import Any

from PyQt6.QtCore import QObject, pyqtSignal

from models.primitive_collider_models import (
    PrimitiveColliderData,
    parse_primitive_collider_data,
    primitive_collider_data_to_dicts,
)
from utils.mgi import RobotTool


class ToolModel(QObject):
    DEFAULT_TOOL_CAD_MODEL: str = ""
    DEFAULT_TOOL_CAD_OFFSET_RZ: float = 0.0
    DEFAULT_TOOL_COLLIDERS: list[PrimitiveColliderData] = []
    DEFAULT_EVALUATED_ROBOT_AXIS_COLLIDERS: list[bool] = [True] * 6
    DEFAULT_TOOL_PROFILES_DIRECTORY: str = "./user_data/tools"
    DEFAULT_SELECTED_TOOL_PROFILE: str = ""

    tool_changed = pyqtSignal()
    tool_visual_changed = pyqtSignal()
    tool_profile_changed = pyqtSignal()
    tool_colliders_changed = pyqtSignal()
    tool_evaluated_robot_axis_colliders_changed = pyqtSignal()

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.tool = RobotTool()
        self.tool_profiles_directory: str = ToolModel.DEFAULT_TOOL_PROFILES_DIRECTORY
        self.selected_tool_profile: str = ToolModel.DEFAULT_SELECTED_TOOL_PROFILE
        self.tool_cad_model: str = ToolModel.DEFAULT_TOOL_CAD_MODEL
        self.tool_cad_offset_rz: float = ToolModel.DEFAULT_TOOL_CAD_OFFSET_RZ
        self.tool_colliders: list[PrimitiveColliderData] = parse_primitive_collider_data(
            ToolModel.DEFAULT_TOOL_COLLIDERS,
            default_shape="cylinder",
            default_name_prefix="Tool collider",
        )
        self._tool_colliders_revision: int = 0
        self.evaluated_robot_axis_colliders: list[bool] = list(
            ToolModel.DEFAULT_EVALUATED_ROBOT_AXIS_COLLIDERS
        )

    @staticmethod
    def _copy_tool(tool: RobotTool | None = None) -> RobotTool:
        source = tool if tool is not None else RobotTool()
        return RobotTool(
            float(source.x),
            float(source.y),
            float(source.z),
            float(source.a),
            float(source.b),
            float(source.c),
        )

    @staticmethod
    def _normalize_evaluated_robot_axis_colliders(values: list[bool] | None) -> list[bool]:
        raw_values = values if isinstance(values, list) else []
        normalized: list[bool] = []
        for axis in range(6):
            normalized.append(bool(raw_values[axis]) if axis < len(raw_values) else True)
        return normalized

    def get_tool(self) -> RobotTool:
        return self._copy_tool(self.tool)

    def set_tool(self, tool: RobotTool) -> None:
        normalized = self._copy_tool(tool)
        if vars(normalized) == vars(self.tool):
            return
        self.tool = normalized
        self.tool_changed.emit()

    def get_tool_profiles_directory(self) -> str:
        return str(self.tool_profiles_directory)

    def set_tool_profiles_directory(self, directory: str | None) -> None:
        normalized = "" if directory is None else str(directory).strip()
        if normalized == "":
            normalized = ToolModel.DEFAULT_TOOL_PROFILES_DIRECTORY
        if normalized == self.tool_profiles_directory:
            return
        self.tool_profiles_directory = normalized
        self.tool_profile_changed.emit()

    def get_selected_tool_profile(self) -> str:
        return str(self.selected_tool_profile)

    def set_selected_tool_profile(self, profile_path: str | None) -> None:
        normalized = "" if profile_path is None else str(profile_path).strip()
        if normalized == self.selected_tool_profile:
            return
        self.selected_tool_profile = normalized
        self.tool_profile_changed.emit()

    def get_tool_cad_model(self) -> str:
        return str(self.tool_cad_model)

    def set_tool_cad_model(self, tool_cad_model: str | None) -> None:
        normalized = "" if tool_cad_model is None else str(tool_cad_model)
        if normalized == self.tool_cad_model:
            return
        self.tool_cad_model = normalized
        self.tool_visual_changed.emit()

    def get_tool_cad_offset_rz(self) -> float:
        return float(self.tool_cad_offset_rz)

    def set_tool_cad_offset_rz(self, offset_deg: float) -> None:
        normalized = float(offset_deg)
        if normalized == self.tool_cad_offset_rz:
            return
        self.tool_cad_offset_rz = normalized
        self.tool_visual_changed.emit()

    def get_tool_colliders(self) -> list[dict[str, Any]]:
        return primitive_collider_data_to_dicts(
            self.tool_colliders,
            default_shape="cylinder",
            default_name_prefix="Tool collider",
        )

    def get_tool_collider_data(self) -> list[PrimitiveColliderData]:
        return [collider.copy() for collider in self.tool_colliders]

    def get_tool_colliders_revision(self) -> int:
        return int(self._tool_colliders_revision)

    def set_tool_colliders(
        self,
        tool_colliders: list[PrimitiveColliderData] | list[dict[str, Any]],
    ) -> None:
        normalized = parse_primitive_collider_data(
            tool_colliders,
            default_shape="cylinder",
            default_name_prefix="Tool collider",
        )
        if normalized == self.tool_colliders:
            return
        self.tool_colliders = normalized
        self._tool_colliders_revision += 1
        self.tool_colliders_changed.emit()

    def get_evaluated_robot_axis_colliders(self) -> list[bool]:
        return ToolModel._normalize_evaluated_robot_axis_colliders(self.evaluated_robot_axis_colliders)

    def set_evaluated_robot_axis_colliders(self, values: list[bool] | None) -> None:
        normalized = ToolModel._normalize_evaluated_robot_axis_colliders(values)
        if normalized == self.evaluated_robot_axis_colliders:
            return
        self.evaluated_robot_axis_colliders = normalized
        self.tool_evaluated_robot_axis_colliders_changed.emit()

