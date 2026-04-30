from __future__ import annotations

from PyQt6.QtCore import QObject, pyqtSignal

from models.primitive_collider_models import (
    PrimitiveCollider,
    PrimitiveColliderData,
    build_primitive_colliders,
)
from models.types import Pose6
from models.workspace_cad_element import WorkspaceCadElement
from utils.reference_frame_utils import FrameTransform


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
        self.robot_base_pose_world: Pose6 = Pose6.zeros()
        self._robot_base_revision: int = 0
        self._workspace_structure_revision: int = 0
        self._robot_base_transform_world: FrameTransform = FrameTransform.from_pose(
            self.robot_base_pose_world,
            revision=self._robot_base_revision,
        )
        self.workspace_cad_elements: list[WorkspaceCadElement] = []
        self.workspace_tcp_zones: list[PrimitiveColliderData] = []
        self.workspace_tcp_zone_colliders: list[PrimitiveCollider] = []
        self.workspace_collision_zones: list[PrimitiveColliderData] = []
        self.workspace_collision_zone_colliders: list[PrimitiveCollider] = []

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

    def get_robot_base_pose_world(self) -> Pose6:
        return self.robot_base_pose_world.copy()

    def get_robot_base_revision(self) -> int:
        return int(self._robot_base_revision)

    def get_workspace_structure_revision(self) -> int:
        return int(self._workspace_structure_revision)

    def get_robot_base_transform_world(self) -> FrameTransform:
        return self._robot_base_transform_world

    def _set_robot_base_pose_world_cached(self, pose: Pose6) -> None:
        self.robot_base_pose_world = pose.copy()
        self._robot_base_revision += 1
        self._robot_base_transform_world = FrameTransform.from_pose(
            self.robot_base_pose_world,
            revision=self._robot_base_revision,
        )

    def _touch_workspace_structure(self) -> None:
        self._workspace_structure_revision += 1

    def set_robot_base_pose_world(self, pose: Pose6, emit: bool = True) -> None:
        if not isinstance(pose, Pose6):
            raise TypeError("pose must be a Pose6")
        normalized = pose.copy()
        if normalized == self.robot_base_pose_world:
            return
        self._set_robot_base_pose_world_cached(normalized)
        if emit:
            self.workspace_changed.emit()

    def get_workspace_cad_elements(self) -> list[WorkspaceCadElement]:
        return [element.copy() for element in self.workspace_cad_elements]

    def set_workspace_cad_elements(self, cad_elements: list[WorkspaceCadElement], emit: bool = True) -> None:
        if not all(isinstance(element, WorkspaceCadElement) for element in cad_elements):
            raise TypeError("cad_elements must contain WorkspaceCadElement")
        normalized = [element.copy() for element in cad_elements]
        if normalized == self.workspace_cad_elements:
            return
        self.workspace_cad_elements = normalized
        self._touch_workspace_structure()
        if emit:
            self.workspace_changed.emit()

    def get_workspace_tcp_zones(self) -> list[PrimitiveColliderData]:
        return [zone.copy() for zone in self.workspace_tcp_zones]

    def get_workspace_tcp_zone_colliders(self) -> list[PrimitiveCollider]:
        return [collider.copy() for collider in self.workspace_tcp_zone_colliders]

    def set_workspace_tcp_zones(
        self,
        zones: list[PrimitiveColliderData],
        emit: bool = True,
    ) -> None:
        if not all(isinstance(zone, PrimitiveColliderData) for zone in zones):
            raise TypeError("zones must contain PrimitiveColliderData")
        normalized = [zone.copy() for zone in zones]
        if normalized == self.workspace_tcp_zones:
            return
        self.workspace_tcp_zones = normalized
        self.workspace_tcp_zone_colliders = build_primitive_colliders(normalized)
        self._touch_workspace_structure()
        if emit:
            self.workspace_changed.emit()

    def update_workspace_tcp_zone(
        self,
        index: int,
        zone: PrimitiveColliderData,
        emit: bool = True,
    ) -> None:
        if index < 0:
            return
        if not isinstance(zone, PrimitiveColliderData):
            raise TypeError("zone must be a PrimitiveColliderData")
        normalized_zone = zone.copy()
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
        zone: PrimitiveColliderData,
        emit: bool = True,
    ) -> None:
        if not isinstance(zone, PrimitiveColliderData):
            raise TypeError("zone must be a PrimitiveColliderData")
        normalized_zone = zone.copy()
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

    def get_workspace_collision_zones(self) -> list[PrimitiveColliderData]:
        return [zone.copy() for zone in self.workspace_collision_zones]

    def get_workspace_collision_zone_colliders(self) -> list[PrimitiveCollider]:
        return [collider.copy() for collider in self.workspace_collision_zone_colliders]

    def set_workspace_collision_zones(
        self,
        zones: list[PrimitiveColliderData],
        emit: bool = True,
    ) -> None:
        if not all(isinstance(zone, PrimitiveColliderData) for zone in zones):
            raise TypeError("zones must contain PrimitiveColliderData")
        normalized = [zone.copy() for zone in zones]
        if normalized == self.workspace_collision_zones:
            return
        self.workspace_collision_zones = normalized
        self.workspace_collision_zone_colliders = build_primitive_colliders(normalized)
        self._touch_workspace_structure()
        if emit:
            self.workspace_changed.emit()

    def update_workspace_collision_zone(
        self,
        index: int,
        zone: PrimitiveColliderData,
        emit: bool = True,
    ) -> None:
        if index < 0:
            return
        if not isinstance(zone, PrimitiveColliderData):
            raise TypeError("zone must be a PrimitiveColliderData")
        normalized_zone = zone.copy()
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
        zone: PrimitiveColliderData,
        emit: bool = True,
    ) -> None:
        if not isinstance(zone, PrimitiveColliderData):
            raise TypeError("zone must be a PrimitiveColliderData")
        normalized_zone = zone.copy()
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
        robot_base_pose_world: Pose6,
        cad_elements: list[WorkspaceCadElement],
        tcp_zones: list[PrimitiveColliderData],
        collision_zones: list[PrimitiveColliderData],
        file_path: str | None = None,
    ) -> None:
        normalized_scene_name = (
            str(scene_name).strip() if scene_name else WorkspaceModel.DEFAULT_WORKSPACE_SCENE_NAME
        )
        if not isinstance(robot_base_pose_world, Pose6):
            raise TypeError("robot_base_pose_world must be a Pose6")
        if not all(isinstance(element, WorkspaceCadElement) for element in cad_elements):
            raise TypeError("cad_elements must contain WorkspaceCadElement")
        if not all(isinstance(zone, PrimitiveColliderData) for zone in tcp_zones):
            raise TypeError("tcp_zones must contain PrimitiveColliderData")
        if not all(isinstance(zone, PrimitiveColliderData) for zone in collision_zones):
            raise TypeError("collision_zones must contain PrimitiveColliderData")
        normalized_robot_base_pose_world = robot_base_pose_world.copy()
        normalized_cad_elements = [element.copy() for element in cad_elements]
        normalized_tcp_zones = [zone.copy() for zone in tcp_zones]
        normalized_collision_zones = [zone.copy() for zone in collision_zones]
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
        self.workspace_tcp_zone_colliders = build_primitive_colliders(normalized_tcp_zones)
        self.workspace_collision_zones = normalized_collision_zones
        self.workspace_collision_zone_colliders = build_primitive_colliders(normalized_collision_zones)
        if workspace_structure_changed:
            self._touch_workspace_structure()
        self.workspace_file_path = normalized_file_path
        self.workspace_changed.emit()

    def clear_workspace(self) -> None:
        self.set_workspace_data(
            WorkspaceModel.DEFAULT_WORKSPACE_SCENE_NAME,
            Pose6.zeros(),
            [],
            [],
            [],
            file_path="",
        )
