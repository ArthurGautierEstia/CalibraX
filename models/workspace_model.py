from __future__ import annotations

from typing import Any

from PyQt6.QtCore import QObject, pyqtSignal

from models.workspace_primitive_zone_models import (
    WorkspacePrimitiveZoneCollider,
    WorkspacePrimitiveZoneData,
    build_workspace_primitive_zone_colliders,
    parse_workspace_primitive_zones,
    workspace_primitive_zones_to_dict,
)
from models.workspace_file import parse_workspace_cad_elements
from utils.reference_frame_utils import FrameTransform, normalize_pose6


class WorkspaceModel(QObject):
    DEFAULT_WORKSPACE_DIRECTORY: str = "./user_data/workspaces"
    DEFAULT_WORKSPACE_SCENE_NAME: str = "scene"

    workspace_changed = pyqtSignal()
    workspace_tcp_zone_changed = pyqtSignal(int, object)
    workspace_tcp_zone_added = pyqtSignal(int, object)
    workspace_tcp_zone_removed = pyqtSignal(int)
    workspace_collision_zone_changed = pyqtSignal(int, object)
    workspace_collision_zone_added = pyqtSignal(int, object)
    workspace_collision_zone_removed = pyqtSignal(int)

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.workspace_scene_name: str = WorkspaceModel.DEFAULT_WORKSPACE_SCENE_NAME
        self.workspace_file_path: str = ""
        self.robot_base_pose_world: list[float] = [0.0] * 6
        self._robot_base_revision: int = 0
        self._workspace_structure_revision: int = 0
        self._robot_base_transform_world: FrameTransform = FrameTransform.from_pose(
            self.robot_base_pose_world,
            revision=self._robot_base_revision,
        )
        self.workspace_cad_elements: list[dict[str, Any]] = []
        self.workspace_tcp_zones: list[WorkspacePrimitiveZoneData] = []
        self.workspace_tcp_zone_colliders: list[WorkspacePrimitiveZoneCollider] = []
        self.workspace_collision_zones: list[WorkspacePrimitiveZoneData] = []
        self.workspace_collision_zone_colliders: list[WorkspacePrimitiveZoneCollider] = []

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

    def get_robot_base_pose_world(self) -> list[float]:
        return [float(v) for v in self.robot_base_pose_world[:6]]

    def get_robot_base_revision(self) -> int:
        return int(self._robot_base_revision)

    def get_workspace_structure_revision(self) -> int:
        return int(self._workspace_structure_revision)

    def get_robot_base_transform_world(self) -> FrameTransform:
        return self._robot_base_transform_world

    def _set_robot_base_pose_world_cached(self, pose: list[float]) -> None:
        self.robot_base_pose_world = [float(v) for v in pose[:6]]
        self._robot_base_revision += 1
        self._robot_base_transform_world = FrameTransform.from_pose(
            self.robot_base_pose_world,
            revision=self._robot_base_revision,
        )

    def _touch_workspace_structure(self) -> None:
        self._workspace_structure_revision += 1

    def set_robot_base_pose_world(self, pose: list[float], emit: bool = True) -> None:
        normalized = normalize_pose6(pose)
        if normalized == self.robot_base_pose_world:
            return
        self._set_robot_base_pose_world_cached(normalized)
        if emit:
            self.workspace_changed.emit()

    def get_workspace_cad_elements(self) -> list[dict[str, Any]]:
        return parse_workspace_cad_elements(self.workspace_cad_elements)

    def set_workspace_cad_elements(self, cad_elements: list[dict[str, Any]], emit: bool = True) -> None:
        normalized = parse_workspace_cad_elements(cad_elements)
        if normalized == self.workspace_cad_elements:
            return
        self.workspace_cad_elements = normalized
        self._touch_workspace_structure()
        if emit:
            self.workspace_changed.emit()

    def get_workspace_tcp_zones(self) -> list[WorkspacePrimitiveZoneData]:
        return [zone.copy() for zone in self.workspace_tcp_zones]

    def get_workspace_tcp_zone_colliders(self) -> list[WorkspacePrimitiveZoneCollider]:
        return [collider.copy() for collider in self.workspace_tcp_zone_colliders]

    def get_workspace_tcp_zones_as_dicts(self) -> list[dict[str, Any]]:
        return workspace_primitive_zones_to_dict(self.workspace_tcp_zones)

    def set_workspace_tcp_zones(
        self,
        zones: list[WorkspacePrimitiveZoneData] | list[dict[str, Any]],
        emit: bool = True,
    ) -> None:
        normalized = parse_workspace_primitive_zones(zones)
        if normalized == self.workspace_tcp_zones:
            return
        self.workspace_tcp_zones = normalized
        self.workspace_tcp_zone_colliders = build_workspace_primitive_zone_colliders(normalized)
        self._touch_workspace_structure()
        if emit:
            self.workspace_changed.emit()

    def update_workspace_tcp_zone(
        self,
        index: int,
        zone: WorkspacePrimitiveZoneData | dict[str, Any],
        emit: bool = True,
    ) -> None:
        if index < 0:
            return
        normalized_list = parse_workspace_primitive_zones([zone])
        if not normalized_list:
            return
        normalized_zone = normalized_list[0]
        if index >= len(self.workspace_tcp_zones):
            return
        if self.workspace_tcp_zones[index] == normalized_zone:
            return
        self.workspace_tcp_zones[index] = normalized_zone
        collider = normalized_zone.build_collider()
        if index < len(self.workspace_tcp_zone_colliders):
            self.workspace_tcp_zone_colliders[index] = collider
        else:
            self.workspace_tcp_zone_colliders.append(collider)
        self._touch_workspace_structure()
        if emit:
            self.workspace_tcp_zone_changed.emit(index, normalized_zone.copy())
            self.workspace_changed.emit()

    def insert_workspace_tcp_zone(
        self,
        index: int,
        zone: WorkspacePrimitiveZoneData | dict[str, Any],
        emit: bool = True,
    ) -> None:
        normalized_list = parse_workspace_primitive_zones([zone])
        if not normalized_list:
            return
        normalized_zone = normalized_list[0]
        bounded_index = max(0, min(int(index), len(self.workspace_tcp_zones)))
        self.workspace_tcp_zones.insert(bounded_index, normalized_zone)
        self.workspace_tcp_zone_colliders.insert(bounded_index, normalized_zone.build_collider())
        self._touch_workspace_structure()
        if emit:
            self.workspace_tcp_zone_added.emit(bounded_index, normalized_zone.copy())
            self.workspace_changed.emit()

    def remove_workspace_tcp_zone(self, index: int, emit: bool = True) -> None:
        if not (0 <= index < len(self.workspace_tcp_zones)):
            return
        self.workspace_tcp_zones.pop(index)
        if 0 <= index < len(self.workspace_tcp_zone_colliders):
            self.workspace_tcp_zone_colliders.pop(index)
        self._touch_workspace_structure()
        if emit:
            self.workspace_tcp_zone_removed.emit(index)
            self.workspace_changed.emit()

    def get_workspace_collision_zones(self) -> list[WorkspacePrimitiveZoneData]:
        return [zone.copy() for zone in self.workspace_collision_zones]

    def get_workspace_collision_zone_colliders(self) -> list[WorkspacePrimitiveZoneCollider]:
        return [collider.copy() for collider in self.workspace_collision_zone_colliders]

    def set_workspace_collision_zones(self, zones: list[WorkspacePrimitiveZoneData], emit: bool = True) -> None:
        normalized = parse_workspace_primitive_zones(zones)
        if normalized == self.workspace_collision_zones:
            return
        self.workspace_collision_zones = normalized
        self.workspace_collision_zone_colliders = build_workspace_primitive_zone_colliders(normalized)
        self._touch_workspace_structure()
        if emit:
            self.workspace_changed.emit()

    def update_workspace_collision_zone(
        self,
        index: int,
        zone: WorkspacePrimitiveZoneData,
        emit: bool = True,
    ) -> None:
        if index < 0:
            return
        normalized_list = parse_workspace_primitive_zones([zone])
        if not normalized_list:
            return
        normalized_zone = normalized_list[0]
        if index >= len(self.workspace_collision_zones):
            return
        if self.workspace_collision_zones[index] == normalized_zone:
            return
        self.workspace_collision_zones[index] = normalized_zone
        collider = normalized_zone.build_collider()
        if index < len(self.workspace_collision_zone_colliders):
            self.workspace_collision_zone_colliders[index] = collider
        else:
            self.workspace_collision_zone_colliders.append(collider)
        self._touch_workspace_structure()
        if emit:
            self.workspace_collision_zone_changed.emit(index, normalized_zone.copy())
            self.workspace_changed.emit()

    def insert_workspace_collision_zone(
        self,
        index: int,
        zone: WorkspacePrimitiveZoneData | dict[str, Any],
        emit: bool = True,
    ) -> None:
        normalized_list = parse_workspace_primitive_zones([zone])
        if not normalized_list:
            return
        normalized_zone = normalized_list[0]
        bounded_index = max(0, min(int(index), len(self.workspace_collision_zones)))
        self.workspace_collision_zones.insert(bounded_index, normalized_zone)
        self.workspace_collision_zone_colliders.insert(bounded_index, normalized_zone.build_collider())
        self._touch_workspace_structure()
        if emit:
            self.workspace_collision_zone_added.emit(bounded_index, normalized_zone.copy())
            self.workspace_changed.emit()

    def remove_workspace_collision_zone(self, index: int, emit: bool = True) -> None:
        if not (0 <= index < len(self.workspace_collision_zones)):
            return
        self.workspace_collision_zones.pop(index)
        if 0 <= index < len(self.workspace_collision_zone_colliders):
            self.workspace_collision_zone_colliders.pop(index)
        self._touch_workspace_structure()
        if emit:
            self.workspace_collision_zone_removed.emit(index)
            self.workspace_changed.emit()

    def set_workspace_data(
        self,
        scene_name: str,
        robot_base_pose_world: list[float],
        cad_elements: list[dict[str, Any]],
        tcp_zones: list[WorkspacePrimitiveZoneData] | list[dict[str, Any]],
        collision_zones: list[WorkspacePrimitiveZoneData] | list[dict[str, Any]],
        file_path: str | None = None,
    ) -> None:
        normalized_scene_name = (
            str(scene_name).strip() if scene_name else WorkspaceModel.DEFAULT_WORKSPACE_SCENE_NAME
        )
        normalized_robot_base_pose_world = normalize_pose6(robot_base_pose_world)
        normalized_cad_elements = parse_workspace_cad_elements(cad_elements)
        normalized_tcp_zones = parse_workspace_primitive_zones(tcp_zones)
        normalized_collision_zones = parse_workspace_primitive_zones(collision_zones)
        normalized_file_path = "" if file_path is None else str(file_path).strip()

        has_changes = (
            normalized_scene_name != self.workspace_scene_name
            or normalized_robot_base_pose_world != self.robot_base_pose_world
            or normalized_cad_elements != self.workspace_cad_elements
            or normalized_tcp_zones != self.workspace_tcp_zones
            or normalized_collision_zones != self.workspace_collision_zones
            or normalized_file_path != self.workspace_file_path
        )
        if not has_changes:
            return

        robot_base_changed = normalized_robot_base_pose_world != self.robot_base_pose_world
        workspace_structure_changed = (
            normalized_cad_elements != self.workspace_cad_elements
            or normalized_tcp_zones != self.workspace_tcp_zones
            or normalized_collision_zones != self.workspace_collision_zones
        )

        self.workspace_scene_name = normalized_scene_name
        if robot_base_changed:
            self._set_robot_base_pose_world_cached(normalized_robot_base_pose_world)
        self.workspace_cad_elements = normalized_cad_elements
        self.workspace_tcp_zones = normalized_tcp_zones
        self.workspace_tcp_zone_colliders = build_workspace_primitive_zone_colliders(normalized_tcp_zones)
        self.workspace_collision_zones = normalized_collision_zones
        self.workspace_collision_zone_colliders = build_workspace_primitive_zone_colliders(normalized_collision_zones)
        if workspace_structure_changed:
            self._touch_workspace_structure()
        self.workspace_file_path = normalized_file_path
        self.workspace_changed.emit()

    def clear_workspace(self) -> None:
        self.set_workspace_data(
            WorkspaceModel.DEFAULT_WORKSPACE_SCENE_NAME,
            [0.0] * 6,
            [],
            [],
            [],
            file_path="",
        )
