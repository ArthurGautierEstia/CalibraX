from __future__ import annotations

import numpy as np
from PyQt6.QtCore import QObject, pyqtSignal

from models.primitive_collider_models import PrimitiveCollider, PrimitiveColliderData, RobotAxisColliderData
from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.workspace_model import WorkspaceModel


class CollisionSceneModel(QObject):
    scene_changed = pyqtSignal()

    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.robot_model = robot_model
        self.tool_model = tool_model
        self.workspace_model = workspace_model

        self.workspace_tcp_colliders: list[PrimitiveCollider] = []
        self.workspace_collision_colliders: list[PrimitiveCollider] = []
        self.robot_colliders: list[PrimitiveCollider] = []
        self.tool_colliders: list[PrimitiveCollider] = []

        self._robot_axis_collider_data: list[RobotAxisColliderData] = []
        self._tool_collider_data: list[PrimitiveColliderData] = []

        self._setup_connections()
        self._refresh_all(emit=False)

    def _setup_connections(self) -> None:
        self.workspace_model.workspace_changed.connect(self._on_workspace_changed)
        self.robot_model.axis_colliders_changed.connect(self._on_robot_axis_colliders_changed)
        self.robot_model.tcp_pose_changed.connect(self._on_robot_pose_changed)
        self.tool_model.tool_colliders_changed.connect(self._on_tool_colliders_changed)

    def get_workspace_tcp_colliders(self) -> list[PrimitiveCollider]:
        return [collider.copy() for collider in self.workspace_tcp_colliders]

    def get_workspace_collision_colliders(self) -> list[PrimitiveCollider]:
        return [collider.copy() for collider in self.workspace_collision_colliders]

    def get_robot_colliders(self) -> list[PrimitiveCollider]:
        return [collider.copy() for collider in self.robot_colliders]

    def get_tool_colliders(self) -> list[PrimitiveCollider]:
        return [collider.copy() for collider in self.tool_colliders]

    def _on_workspace_changed(self) -> None:
        self._refresh_workspace_colliders()
        self._refresh_attached_world_colliders()
        self.scene_changed.emit()

    def _on_robot_axis_colliders_changed(self) -> None:
        self._refresh_robot_axis_local_data()
        self._refresh_attached_world_colliders()
        self.scene_changed.emit()

    def _on_robot_pose_changed(self) -> None:
        self._refresh_attached_world_colliders()
        self.scene_changed.emit()

    def _on_tool_colliders_changed(self) -> None:
        self._refresh_tool_local_data()
        self._refresh_attached_world_colliders()
        self.scene_changed.emit()

    def _refresh_all(self, emit: bool = True) -> None:
        self._refresh_workspace_colliders()
        self._refresh_robot_axis_local_data()
        self._refresh_tool_local_data()
        self._refresh_attached_world_colliders()
        if emit:
            self.scene_changed.emit()

    def _refresh_workspace_colliders(self) -> None:
        self.workspace_tcp_colliders = self.workspace_model.get_workspace_tcp_zone_colliders()
        self.workspace_collision_colliders = self.workspace_model.get_workspace_collision_zone_colliders()

    def _refresh_robot_axis_local_data(self) -> None:
        self._robot_axis_collider_data = self.robot_model.get_axis_collider_data()

    def _refresh_tool_local_data(self) -> None:
        self._tool_collider_data = self.tool_model.get_tool_collider_data()

    def _refresh_attached_world_colliders(self) -> None:
        corrected_matrices = self._resolve_robot_corrected_matrices()
        if not corrected_matrices:
            self.robot_colliders = []
            self.tool_colliders = []
            return

        robot_base_world = np.array(self.workspace_model.get_robot_base_transform_world().matrix, dtype=float)
        self.robot_colliders = self._build_robot_world_colliders(corrected_matrices, robot_base_world)
        self.tool_colliders = self._build_tool_world_colliders(corrected_matrices, robot_base_world)

    def _resolve_robot_corrected_matrices(self) -> list[np.ndarray]:
        matrices = self.robot_model.get_current_tcp_corrected_dh_matrices()
        if matrices:
            return [np.array(matrix, dtype=float) for matrix in matrices]

        fk_result = self.robot_model.compute_fk_joints(self.robot_model.get_joints())
        if fk_result is None:
            return []

        return [np.array(matrix, dtype=float) for matrix in fk_result.corrected_matrices]

    def _build_robot_world_colliders(
        self,
        corrected_matrices: list[np.ndarray],
        robot_base_world: np.ndarray,
    ) -> list[PrimitiveCollider]:
        colliders: list[PrimitiveCollider] = []
        for data in self._robot_axis_collider_data:
            matrix_index = data.axis_index + 1
            if matrix_index >= len(corrected_matrices):
                continue
            base_transform = robot_base_world @ corrected_matrices[matrix_index]
            colliders.append(data.build_collider(base_transform=base_transform))
        return colliders

    def _build_tool_world_colliders(
        self,
        corrected_matrices: list[np.ndarray],
        robot_base_world: np.ndarray,
    ) -> list[PrimitiveCollider]:
        if not corrected_matrices:
            return []

        if len(corrected_matrices) >= 2:
            tool_matrix_index = len(corrected_matrices) - 2
        else:
            tool_matrix_index = len(corrected_matrices) - 1

        if tool_matrix_index < 0 or tool_matrix_index >= len(corrected_matrices):
            return []

        base_transform = robot_base_world @ corrected_matrices[tool_matrix_index]
        colliders: list[PrimitiveCollider] = []
        for data in self._tool_collider_data:
            colliders.append(
                data.build_collider(
                    owner="tool",
                    base_transform=base_transform,
                    attachment_key="tool_flange",
                    attachment_index=tool_matrix_index,
                )
            )
        return colliders
