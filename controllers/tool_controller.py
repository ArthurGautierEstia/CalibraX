from __future__ import annotations

import os

from PyQt6.QtCore import QObject
from PyQt6.QtWidgets import QMessageBox

from models.primitive_collider_models import PrimitiveColliderData
from models.tool_config_file import ToolConfigFile
from models.tool_model import ToolModel
from utils.mgi import RobotTool
from widgets.tool_view.tool_configuration_widget import ToolConfigurationWidget


class ToolController(QObject):
    def __init__(
        self,
        tool_model: ToolModel,
        robot_configuration_widget: ToolConfigurationWidget,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.tool_model = tool_model
        self.robot_configuration_widget = robot_configuration_widget
        self._setup_connections()
        self.update_tool_view()

    def _setup_connections(self) -> None:
        self.tool_model.tool_changed.connect(self.update_tool_view)
        self.tool_model.tool_visual_changed.connect(self.update_tool_view)
        self.tool_model.tool_profile_changed.connect(self.update_tool_view)
        self.tool_model.tool_colliders_changed.connect(self.update_tool_view)
        self.tool_model.tool_evaluated_robot_axis_colliders_changed.connect(self.update_tool_view)

        self.robot_configuration_widget.tool_changed.connect(self._on_view_tool_changed)
        self.robot_configuration_widget.tool_cad_model_changed.connect(self._on_view_tool_cad_model_changed)
        self.robot_configuration_widget.tool_cad_offset_rz_changed.connect(self._on_view_tool_cad_offset_rz_changed)
        self.robot_configuration_widget.tool_colliders_changed.connect(self._on_view_tool_colliders_changed)
        self.robot_configuration_widget.tool_evaluated_robot_axis_colliders_changed.connect(
            self._on_view_tool_evaluated_robot_axis_colliders_changed
        )
        self.robot_configuration_widget.tool_profiles_directory_changed.connect(self._on_view_tool_profiles_directory_changed)
        self.robot_configuration_widget.selected_tool_profile_changed.connect(self._on_view_selected_tool_profile_changed)

    def _on_view_tool_changed(self, tool) -> None:
        self.tool_model.set_tool(tool)

    def _on_view_tool_cad_model_changed(self, tool_cad_model: str) -> None:
        self.tool_model.set_tool_cad_model(tool_cad_model)

    def _on_view_tool_cad_offset_rz_changed(self, offset_deg: float) -> None:
        self.tool_model.set_tool_cad_offset_rz(offset_deg)

    def _on_view_tool_colliders_changed(self, tool_colliders: list[PrimitiveColliderData]) -> None:
        self.tool_model.set_tool_colliders(tool_colliders)

    def _on_view_tool_evaluated_robot_axis_colliders_changed(self, values: list[bool]) -> None:
        self.tool_model.set_evaluated_robot_axis_colliders(values)

    def _on_view_tool_profiles_directory_changed(self, directory: str) -> None:
        self.tool_model.set_tool_profiles_directory(directory)

    def _on_view_selected_tool_profile_changed(self, profile_path: str) -> None:
        if not profile_path:
            self._apply_empty_tool()
            return
        self.load_tool_profile_from_path(profile_path, show_errors=True)

    def _apply_empty_tool(self) -> None:
        self.tool_model.set_selected_tool_profile("")
        self.tool_model.set_tool(RobotTool())
        self.tool_model.set_tool_cad_model("")
        self.tool_model.set_tool_cad_offset_rz(0.0)
        self.tool_model.set_tool_colliders([])
        self.tool_model.set_evaluated_robot_axis_colliders([True] * 6)

    def update_tool_view(self) -> None:
        self.robot_configuration_widget.set_tool_profiles_directory(self.tool_model.get_tool_profiles_directory())
        self.robot_configuration_widget.set_selected_tool_profile(self.tool_model.get_selected_tool_profile())
        self.robot_configuration_widget.set_tool(self.tool_model.get_tool())
        self.robot_configuration_widget.set_tool_cad_model(self.tool_model.get_tool_cad_model())
        self.robot_configuration_widget.set_tool_cad_offset_rz(self.tool_model.get_tool_cad_offset_rz())
        self.robot_configuration_widget.set_tool_colliders(self.tool_model.get_tool_collider_data())
        self.robot_configuration_widget.set_tool_evaluated_robot_axis_colliders(
            self.tool_model.get_evaluated_robot_axis_colliders()
        )

    def load_tool_profile_from_path(self, file_path: str, show_errors: bool = False) -> bool:
        try:
            profile = ToolConfigFile.load(file_path)
        except (OSError, ValueError, TypeError) as exc:
            if show_errors:
                QMessageBox.warning(
                    self.robot_configuration_widget,
                    "Tool invalide",
                    f"Impossible de charger {file_path}.\n{exc}",
                )
            return False

        normalized_path = self.robot_configuration_widget._normalize_project_path(file_path)
        directory = os.path.dirname(os.path.abspath(file_path))
        self.tool_model.set_tool_profiles_directory(self.robot_configuration_widget._normalize_project_path(directory))
        self.tool_model.set_selected_tool_profile(normalized_path)
        self.tool_model.set_tool(profile.to_robot_tool())
        self.tool_model.set_tool_cad_model(profile.tool_cad_model)
        self.tool_model.set_tool_cad_offset_rz(profile.tool_cad_offset_rz)
        self.tool_model.set_tool_colliders(profile.tool_colliders)
        self.tool_model.set_evaluated_robot_axis_colliders(profile.evaluated_robot_axis_colliders)
        return True
