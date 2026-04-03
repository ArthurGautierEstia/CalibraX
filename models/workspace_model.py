from __future__ import annotations

from typing import Any

from PyQt6.QtCore import QObject, pyqtSignal

from models.collider_models import parse_primitive_colliders
from models.workspace_file import parse_workspace_cad_elements


class WorkspaceModel(QObject):
    DEFAULT_WORKSPACE_DIRECTORY: str = "./workspaces"
    DEFAULT_WORKSPACE_SCENE_NAME: str = "scene"

    workspace_changed = pyqtSignal()

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.workspace_scene_name: str = WorkspaceModel.DEFAULT_WORKSPACE_SCENE_NAME
        self.workspace_file_path: str = ""
        self.workspace_cad_elements: list[dict[str, Any]] = []
        self.workspace_tcp_zones: list[dict[str, Any]] = []
        self.workspace_collision_zones: list[dict[str, Any]] = []

    def get_workspace_scene_name(self) -> str:
        return str(self.workspace_scene_name)

    def set_workspace_scene_name(self, scene_name: str) -> None:
        normalized = str(scene_name).strip()
        if normalized == "":
            normalized = WorkspaceModel.DEFAULT_WORKSPACE_SCENE_NAME
        if normalized == self.workspace_scene_name:
            return
        self.workspace_scene_name = normalized
        self.workspace_changed.emit()

    def get_workspace_file_path(self) -> str:
        return str(self.workspace_file_path)

    def set_workspace_file_path(self, file_path: str | None) -> None:
        normalized = "" if file_path is None else str(file_path).strip()
        if normalized == self.workspace_file_path:
            return
        self.workspace_file_path = normalized
        self.workspace_changed.emit()

    def get_workspace_cad_elements(self) -> list[dict[str, Any]]:
        return parse_workspace_cad_elements(self.workspace_cad_elements)

    def set_workspace_cad_elements(self, cad_elements: list[dict[str, Any]], emit: bool = True) -> None:
        normalized = parse_workspace_cad_elements(cad_elements)
        if normalized == self.workspace_cad_elements:
            return
        self.workspace_cad_elements = normalized
        if emit:
            self.workspace_changed.emit()

    def get_workspace_tcp_zones(self) -> list[dict[str, Any]]:
        return parse_primitive_colliders(self.workspace_tcp_zones, default_shape="box")

    def set_workspace_tcp_zones(self, zones: list[dict[str, Any]], emit: bool = True) -> None:
        normalized = parse_primitive_colliders(zones, default_shape="box")
        if normalized == self.workspace_tcp_zones:
            return
        self.workspace_tcp_zones = normalized
        if emit:
            self.workspace_changed.emit()

    def get_workspace_collision_zones(self) -> list[dict[str, Any]]:
        return parse_primitive_colliders(self.workspace_collision_zones, default_shape="box")

    def set_workspace_collision_zones(self, zones: list[dict[str, Any]], emit: bool = True) -> None:
        normalized = parse_primitive_colliders(zones, default_shape="box")
        if normalized == self.workspace_collision_zones:
            return
        self.workspace_collision_zones = normalized
        if emit:
            self.workspace_changed.emit()

    def set_workspace_data(
        self,
        scene_name: str,
        cad_elements: list[dict[str, Any]],
        tcp_zones: list[dict[str, Any]],
        collision_zones: list[dict[str, Any]],
        file_path: str | None = None,
    ) -> None:
        normalized_scene_name = (
            str(scene_name).strip() if scene_name else WorkspaceModel.DEFAULT_WORKSPACE_SCENE_NAME
        )
        normalized_cad_elements = parse_workspace_cad_elements(cad_elements)
        normalized_tcp_zones = parse_primitive_colliders(tcp_zones, default_shape="box")
        normalized_collision_zones = parse_primitive_colliders(collision_zones, default_shape="box")
        normalized_file_path = "" if file_path is None else str(file_path).strip()

        has_changes = (
            normalized_scene_name != self.workspace_scene_name
            or normalized_cad_elements != self.workspace_cad_elements
            or normalized_tcp_zones != self.workspace_tcp_zones
            or normalized_collision_zones != self.workspace_collision_zones
            or normalized_file_path != self.workspace_file_path
        )
        if not has_changes:
            return

        self.workspace_scene_name = normalized_scene_name
        self.workspace_cad_elements = normalized_cad_elements
        self.workspace_tcp_zones = normalized_tcp_zones
        self.workspace_collision_zones = normalized_collision_zones
        self.workspace_file_path = normalized_file_path
        self.workspace_changed.emit()

