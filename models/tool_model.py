from __future__ import annotations

from typing import Any

from PyQt6.QtCore import QObject, pyqtSignal

from models.collider_models import parse_primitive_colliders
from utils.mgi import RobotTool


class ToolModel(QObject):
    DEFAULT_TOOL_CAD_MODEL: str = ""
    DEFAULT_TOOL_CAD_OFFSET_RZ: float = 0.0
    DEFAULT_TOOL_COLLIDERS: list[dict[str, Any]] = []
    DEFAULT_TOOL_RETRACTABLE_Z_MM: float = 0.0
    DEFAULT_TOOL_PROFILES_DIRECTORY: str = "./configurations/tools"
    DEFAULT_SELECTED_TOOL_PROFILE: str = ""

    tool_changed = pyqtSignal()
    tool_visual_changed = pyqtSignal()
    tool_profile_changed = pyqtSignal()
    tool_colliders_changed = pyqtSignal()
    tool_retractable_z_changed = pyqtSignal()

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.tool = RobotTool()
        self.tool_profiles_directory: str = ToolModel.DEFAULT_TOOL_PROFILES_DIRECTORY
        self.selected_tool_profile: str = ToolModel.DEFAULT_SELECTED_TOOL_PROFILE
        self.tool_cad_model: str = ToolModel.DEFAULT_TOOL_CAD_MODEL
        self.tool_cad_offset_rz: float = ToolModel.DEFAULT_TOOL_CAD_OFFSET_RZ
        self.tool_colliders: list[dict[str, Any]] = parse_primitive_colliders(
            ToolModel.DEFAULT_TOOL_COLLIDERS,
            default_shape="cylinder",
        )
        self.tool_retractable_z_mm: float = ToolModel.DEFAULT_TOOL_RETRACTABLE_Z_MM

    def get_tool(self) -> RobotTool:
        return self.tool

    def set_tool(self, tool: RobotTool) -> None:
        normalized = RobotTool(
            float(tool.x),
            float(tool.y),
            float(tool.z),
            float(tool.a),
            float(tool.b),
            float(tool.c),
        )
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
        return [dict(collider) for collider in parse_primitive_colliders(self.tool_colliders, default_shape="cylinder")]

    def set_tool_colliders(self, tool_colliders: list[dict[str, Any]]) -> None:
        normalized = parse_primitive_colliders(tool_colliders, default_shape="cylinder")
        if normalized == self.tool_colliders:
            return
        self.tool_colliders = normalized
        self.tool_colliders_changed.emit()

    def get_tool_retractable_z_mm(self) -> float:
        return float(self.tool_retractable_z_mm)

    def set_tool_retractable_z_mm(self, retractable_z_mm: float) -> None:
        normalized = float(retractable_z_mm)
        if normalized == self.tool_retractable_z_mm:
            return
        self.tool_retractable_z_mm = normalized
        self.tool_retractable_z_changed.emit()

