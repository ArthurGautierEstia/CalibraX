from __future__ import annotations

from PyQt6.QtCore import QObject, pyqtSignal

from models.primitive_collider_models import PrimitiveColliderData
from models.types import Pose6
from utils.mgi import RobotTool


class ToolModel(QObject):
    DEFAULT_TOOL_CAD_MODEL: str = ""
    DEFAULT_TOOL_CAD_OFFSET_RZ: float = 0.0
    DEFAULT_AUTO_LOAD_ON_STARTUP: bool = False
    DEFAULT_TOOL_COLLIDERS: list[PrimitiveColliderData] = []
    DEFAULT_EVALUATED_ROBOT_AXIS_COLLIDERS: list[bool] = [True] * 6
    DEFAULT_SELECTED_TOOL_PROFILE: str = ""

    tool_changed = pyqtSignal()
    tool_visual_changed = pyqtSignal()
    tool_profile_changed = pyqtSignal()
    tool_colliders_changed = pyqtSignal()
    tool_evaluated_robot_axis_colliders_changed = pyqtSignal()
    tool_startup_behavior_changed = pyqtSignal()

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.tool = RobotTool()
        self.selected_tool_profile: str = ToolModel.DEFAULT_SELECTED_TOOL_PROFILE
        self.tool_cad_model: str = ToolModel.DEFAULT_TOOL_CAD_MODEL
        self.tool_cad_offset_rz: float = ToolModel.DEFAULT_TOOL_CAD_OFFSET_RZ
        self.auto_load_on_startup: bool = ToolModel.DEFAULT_AUTO_LOAD_ON_STARTUP
        self.tool_colliders: list[PrimitiveColliderData] = [collider.copy() for collider in ToolModel.DEFAULT_TOOL_COLLIDERS]
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
    def _tool_to_pose(tool: RobotTool | None = None) -> Pose6:
        source = tool if tool is not None else RobotTool()
        return Pose6(
            source.x,
            source.y,
            source.z,
            source.a,
            source.b,
            source.c,
        )

    @staticmethod
    def _pose_to_tool(pose: Pose6) -> RobotTool:
        if not isinstance(pose, Pose6):
            raise TypeError("pose must be a Pose6")
        return RobotTool(*pose.to_tuple())

    @staticmethod
    def _copy_evaluated_robot_axis_colliders(values: list[bool]) -> list[bool]:
        if len(values) != 6:
            raise ValueError("evaluated_robot_axis_colliders must contain 6 values")
        return [bool(value) for value in values]

    def get_tool(self) -> RobotTool:
        return self._copy_tool(self.tool)

    def get_tool_pose(self) -> Pose6:
        return self._tool_to_pose(self.tool)

    def set_tool(self, tool: RobotTool) -> None:
        normalized = self._copy_tool(tool)
        if vars(normalized) == vars(self.tool):
            return
        self.tool = normalized
        self.tool_changed.emit()

    def set_tool_pose(self, pose: Pose6) -> None:
        self.set_tool(self._pose_to_tool(pose))

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

    def get_auto_load_on_startup(self) -> bool:
        return bool(self.auto_load_on_startup)

    def set_auto_load_on_startup(self, enabled: bool) -> None:
        normalized = bool(enabled)
        if normalized == self.auto_load_on_startup:
            return
        self.auto_load_on_startup = normalized
        self.tool_startup_behavior_changed.emit()

    def get_tool_colliders(self) -> list[PrimitiveColliderData]:
        return self.get_tool_collider_data()

    def get_tool_collider_data(self) -> list[PrimitiveColliderData]:
        return [collider.copy() for collider in self.tool_colliders]

    def get_tool_colliders_revision(self) -> int:
        return int(self._tool_colliders_revision)

    def set_tool_colliders(
        self,
        tool_colliders: list[PrimitiveColliderData],
    ) -> None:
        if not all(isinstance(collider, PrimitiveColliderData) for collider in tool_colliders):
            raise TypeError("tool_colliders must contain PrimitiveColliderData")
        normalized = [collider.copy() for collider in tool_colliders]
        if normalized == self.tool_colliders:
            return
        self.tool_colliders = normalized
        self._tool_colliders_revision += 1
        self.tool_colliders_changed.emit()

    def get_evaluated_robot_axis_colliders(self) -> list[bool]:
        return ToolModel._copy_evaluated_robot_axis_colliders(self.evaluated_robot_axis_colliders)

    def set_evaluated_robot_axis_colliders(self, values: list[bool]) -> None:
        normalized = ToolModel._copy_evaluated_robot_axis_colliders(values)
        if normalized == self.evaluated_robot_axis_colliders:
            return
        self.evaluated_robot_axis_colliders = normalized
        self.tool_evaluated_robot_axis_colliders_changed.emit()

