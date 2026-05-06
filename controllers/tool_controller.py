from __future__ import annotations

import json

from PyQt6.QtCore import QObject, pyqtSignal
from PyQt6.QtWidgets import QMessageBox

from models.primitive_collider_models import PrimitiveColliderData
from models.tool_config_file import ToolConfigFile
from models.tool_model import ToolModel
from utils.mgi import RobotTool
from widgets.tool_view.tool_configuration_widget import ToolConfigurationWidget


class ToolController(QObject):
    STATUS_UNSAVED = "Configuration tool non enregistrée"
    STATUS_MODIFIED = "Modifications non enregistrées"
    STATUS_SAVED = "Configuration tool enregistrée"
    STATUS_LOADED = "Configuration tool chargée"
    STATUS_UP_TO_DATE = "Configuration tool à jour"
    validation_state_changed = pyqtSignal(bool)

    def __init__(
        self,
        tool_model: ToolModel,
        robot_configuration_widget: ToolConfigurationWidget,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.tool_model = tool_model
        self.robot_configuration_widget = robot_configuration_widget
        self._saved_snapshot: str | None = None
        self._has_saved_reference = False
        self._clean_status_text = ToolController.STATUS_UNSAVED
        self._was_dirty_since_reference = False
        self._validation_icon_visible = False
        self._setup_connections()
        self.update_tool_view()
        self._mark_as_unsaved_reference()

    def _setup_connections(self) -> None:
        self.tool_model.tool_changed.connect(self.update_tool_view)
        self.tool_model.tool_visual_changed.connect(self.update_tool_view)
        self.tool_model.tool_profile_changed.connect(self.update_tool_view)
        self.tool_model.tool_colliders_changed.connect(self.update_tool_view)
        self.tool_model.tool_evaluated_robot_axis_colliders_changed.connect(self.update_tool_view)
        self.tool_model.tool_startup_behavior_changed.connect(self.update_tool_view)

        self.robot_configuration_widget.tool_changed.connect(self._on_view_tool_changed)
        self.robot_configuration_widget.tool_name_changed.connect(self._on_view_tool_name_changed)
        self.robot_configuration_widget.tool_cad_model_changed.connect(self._on_view_tool_cad_model_changed)
        self.robot_configuration_widget.tool_cad_offset_rz_changed.connect(self._on_view_tool_cad_offset_rz_changed)
        self.robot_configuration_widget.tool_auto_load_on_startup_changed.connect(
            self._on_view_tool_auto_load_on_startup_changed
        )
        self.robot_configuration_widget.tool_colliders_changed.connect(self._on_view_tool_colliders_changed)
        self.robot_configuration_widget.tool_evaluated_robot_axis_colliders_changed.connect(
            self._on_view_tool_evaluated_robot_axis_colliders_changed
        )
        self.robot_configuration_widget.selected_tool_profile_changed.connect(self._on_view_selected_tool_profile_changed)
        self.robot_configuration_widget.tool_profile_saved.connect(self._on_tool_profile_saved)
        self.robot_configuration_widget.new_tool_requested.connect(self._apply_empty_tool)

    def _on_view_tool_changed(self, tool) -> None:
        self.tool_model.set_tool(tool)
        self._refresh_configuration_status()

    def _on_view_tool_name_changed(self, _name: str) -> None:
        self._refresh_configuration_status()

    def _on_view_tool_cad_model_changed(self, tool_cad_model: str) -> None:
        self.tool_model.set_tool_cad_model(tool_cad_model)
        self._refresh_configuration_status()

    def _on_view_tool_cad_offset_rz_changed(self, offset_deg: float) -> None:
        self.tool_model.set_tool_cad_offset_rz(offset_deg)
        self._refresh_configuration_status()

    def _on_view_tool_auto_load_on_startup_changed(self, enabled: bool) -> None:
        self.tool_model.set_auto_load_on_startup(enabled)
        self._refresh_configuration_status()

    def _on_view_tool_colliders_changed(self, tool_colliders: list[PrimitiveColliderData]) -> None:
        self.tool_model.set_tool_colliders(tool_colliders)
        self._refresh_configuration_status()

    def _on_view_tool_evaluated_robot_axis_colliders_changed(self, values: list[bool]) -> None:
        self.tool_model.set_evaluated_robot_axis_colliders(values)
        self._refresh_configuration_status()

    def _on_view_selected_tool_profile_changed(self, profile_path: str) -> None:
        if not profile_path:
            self._apply_empty_tool()
            return
        self.load_tool_profile_from_path(profile_path, show_errors=True)

    def _on_tool_profile_saved(self, _profile_path: str) -> None:
        self._mark_as_saved_reference()

    def _apply_empty_tool(self) -> None:
        self.tool_model.set_selected_tool_profile("")
        self.robot_configuration_widget.set_tool_name("")
        self.tool_model.set_tool(RobotTool())
        self.tool_model.set_tool_cad_model("")
        self.tool_model.set_tool_cad_offset_rz(0.0)
        self.tool_model.set_auto_load_on_startup(False)
        self.tool_model.set_tool_colliders([])
        self.tool_model.set_evaluated_robot_axis_colliders([True] * 6)
        self._mark_as_unsaved_reference()

    def reset_tool_configuration(self) -> None:
        self._apply_empty_tool()

    def update_tool_view(self) -> None:
        self.robot_configuration_widget.set_selected_tool_profile(self.tool_model.get_selected_tool_profile())
        self.robot_configuration_widget.set_tool(self.tool_model.get_tool())
        self.robot_configuration_widget.set_tool_cad_model(self.tool_model.get_tool_cad_model())
        self.robot_configuration_widget.set_tool_cad_offset_rz(self.tool_model.get_tool_cad_offset_rz())
        self.robot_configuration_widget.set_tool_auto_load_on_startup(self.tool_model.get_auto_load_on_startup())
        self.robot_configuration_widget.set_tool_colliders(self.tool_model.get_tool_collider_data())
        self.robot_configuration_widget.set_tool_evaluated_robot_axis_colliders(
            self.tool_model.get_evaluated_robot_axis_colliders()
        )
        self._refresh_configuration_status()

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
        self.tool_model.set_selected_tool_profile(normalized_path)
        self.robot_configuration_widget.set_tool_name(profile.name)
        self.tool_model.set_tool(profile.to_robot_tool())
        self.tool_model.set_tool_cad_model(profile.tool_cad_model)
        self.tool_model.set_tool_cad_offset_rz(profile.tool_cad_offset_rz)
        self.tool_model.set_auto_load_on_startup(profile.auto_load_on_startup)
        self.tool_model.set_tool_colliders(profile.tool_colliders)
        self.tool_model.set_evaluated_robot_axis_colliders(profile.evaluated_robot_axis_colliders)
        self._mark_as_loaded_reference()
        return True

    @staticmethod
    def _normalize_snapshot_value(value: object) -> object:
        if isinstance(value, float):
            return round(value, 9)
        if isinstance(value, list):
            return [ToolController._normalize_snapshot_value(item) for item in value]
        if isinstance(value, dict):
            return {
                str(key): ToolController._normalize_snapshot_value(item)
                for key, item in value.items()
            }
        return value

    def _capture_current_snapshot(self) -> str:
        tool_dict = ToolConfigFile.from_robot_tool(
            self.robot_configuration_widget.get_tool_name(),
            self.tool_model.get_tool(),
            self.tool_model.get_tool_cad_model(),
            self.tool_model.get_tool_cad_offset_rz(),
            self.tool_model.get_auto_load_on_startup(),
            self.tool_model.get_tool_collider_data(),
            self.tool_model.get_evaluated_robot_axis_colliders(),
        ).to_dict()
        normalized_tool_dict = ToolController._normalize_snapshot_value(tool_dict)
        return json.dumps(normalized_tool_dict, sort_keys=True, ensure_ascii=True)

    def _mark_as_saved_reference(self) -> None:
        self._saved_snapshot = self._capture_current_snapshot()
        self._has_saved_reference = True
        self._clean_status_text = ToolController.STATUS_SAVED
        self._was_dirty_since_reference = False
        self._refresh_configuration_status()

    def _mark_as_loaded_reference(self) -> None:
        self._saved_snapshot = self._capture_current_snapshot()
        self._has_saved_reference = True
        self._clean_status_text = ToolController.STATUS_LOADED
        self._was_dirty_since_reference = False
        self._refresh_configuration_status()

    def _mark_as_unsaved_reference(self) -> None:
        self._saved_snapshot = self._capture_current_snapshot()
        self._has_saved_reference = False
        self._clean_status_text = ToolController.STATUS_UNSAVED
        self._was_dirty_since_reference = False
        self._refresh_configuration_status()

    def _is_dirty(self) -> bool:
        current_snapshot = self._capture_current_snapshot()
        return self._saved_snapshot is None or current_snapshot != self._saved_snapshot

    def _refresh_configuration_status(self) -> None:
        show_validation_icon = self._should_show_validation_icon()
        if show_validation_icon != self._validation_icon_visible:
            self._validation_icon_visible = show_validation_icon
            self.validation_state_changed.emit(show_validation_icon)
        if not self._has_saved_reference:
            self.robot_configuration_widget.set_configuration_status(
                ToolController.STATUS_UNSAVED,
                "#808080",
            )
            return
        if self._is_dirty():
            self._was_dirty_since_reference = True
            self.robot_configuration_widget.set_configuration_status(
                ToolController.STATUS_MODIFIED,
                "#d97706",
            )
            return
        if self._was_dirty_since_reference:
            self.robot_configuration_widget.set_configuration_status(
                ToolController.STATUS_UP_TO_DATE,
                "#15803d",
            )
            return
        self.robot_configuration_widget.set_configuration_status(
            self._clean_status_text,
            "#15803d",
        )

    def _should_show_validation_icon(self) -> bool:
        if not self._has_saved_reference:
            return False
        if self._is_dirty():
            return True
        if self._was_dirty_since_reference:
            return True
        return self._clean_status_text != ToolController.STATUS_UNSAVED
