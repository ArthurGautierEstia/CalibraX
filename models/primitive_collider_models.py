from __future__ import annotations

from enum import Enum

import numpy as np

import utils.math_utils as math_utils
from models.types import Pose6, XYZ3


class PrimitiveColliderShape(str, Enum):
    BOX = "box"
    CYLINDER = "cylinder"
    SPHERE = "sphere"


class AxisDirection(str, Enum):
    X = "x"
    Y = "y"
    Z = "z"


SUPPORTED_PRIMITIVE_COLLIDER_SHAPES = tuple(PrimitiveColliderShape)
SUPPORTED_AXIS_DIRECTIONS = tuple(AxisDirection)


def _freeze_matrix(matrix: np.ndarray) -> np.ndarray:
    normalized = np.array(matrix, dtype=float)
    if normalized.shape != (4, 4):
        raise ValueError("Expected a 4x4 transform matrix")
    normalized.setflags(write=False)
    return normalized


def _build_orientation_matrix(direction_axis: AxisDirection, positive_direction: bool) -> np.ndarray:
    rotation = np.eye(4, dtype=float)
    if direction_axis == AxisDirection.X:
        rotation[:3, :3] = math_utils.rot_y(90.0, degrees=True)
    elif direction_axis == AxisDirection.Y:
        rotation[:3, :3] = math_utils.rot_x(-90.0, degrees=True)

    if positive_direction:
        return _freeze_matrix(rotation)

    flip = np.eye(4, dtype=float)
    flip[:3, :3] = math_utils.rot_x(180.0, degrees=True)
    return _freeze_matrix(rotation @ flip)


_PRIMITIVE_EXTRUSION_ORIENTATIONS: dict[tuple[AxisDirection, bool], np.ndarray] = {
    (axis_name, positive): _build_orientation_matrix(axis_name, positive)
    for axis_name in AxisDirection
    for positive in (False, True)
}


def primitive_extrusion_orientation(
    direction_axis: AxisDirection,
    positive_direction: bool = True,
) -> np.ndarray:
    return _PRIMITIVE_EXTRUSION_ORIENTATIONS[(direction_axis, bool(positive_direction))]


class PrimitiveCollider:
    def __init__(
        self,
        owner: str,
        name: str,
        enabled: bool,
        shape: PrimitiveColliderShape,
        size_x: float,
        size_y: float,
        size_z: float,
        radius: float,
        height: float,
        local_transform: np.ndarray,
        base_transform: np.ndarray | None = None,
        attachment_key: str = "world",
        attachment_index: int = -1,
    ) -> None:
        if not isinstance(shape, PrimitiveColliderShape):
            raise TypeError("shape must be a PrimitiveColliderShape")

        normalized_name = str(name).strip()
        self.owner = str(owner).strip().lower() or "workspace"
        self.name = normalized_name if normalized_name != "" else "Collider"
        self.enabled = bool(enabled)
        self.shape = shape
        self.size_x = max(0.0, float(size_x))
        self.size_y = max(0.0, float(size_y))
        self.size_z = max(0.0, float(size_z))
        self.radius = max(0.0, float(radius))
        self.height = max(0.0, float(height))
        self.local_transform = _freeze_matrix(local_transform)
        resolved_base = np.eye(4, dtype=float) if base_transform is None else np.array(base_transform, dtype=float)
        self.base_transform = _freeze_matrix(resolved_base)
        self.world_transform = _freeze_matrix(self.base_transform @ self.local_transform)
        self.attachment_key = str(attachment_key).strip() or "world"
        self.attachment_index = int(attachment_index)

    def copy(self) -> "PrimitiveCollider":
        return PrimitiveCollider(
            owner=self.owner,
            name=self.name,
            enabled=self.enabled,
            shape=self.shape,
            size_x=self.size_x,
            size_y=self.size_y,
            size_z=self.size_z,
            radius=self.radius,
            height=self.height,
            local_transform=self.local_transform,
            base_transform=self.base_transform,
            attachment_key=self.attachment_key,
            attachment_index=self.attachment_index,
        )


class PrimitiveColliderData:
    def __init__(
        self,
        name: str,
        enabled: bool = True,
        shape: PrimitiveColliderShape = PrimitiveColliderShape.BOX,
        pose: Pose6 | None = None,
        size_x: float = 200.0,
        size_y: float = 200.0,
        size_z: float = 200.0,
        radius: float = 100.0,
        height: float = 200.0,
    ) -> None:
        if not isinstance(shape, PrimitiveColliderShape):
            raise TypeError("shape must be a PrimitiveColliderShape")
        if pose is not None and not isinstance(pose, Pose6):
            raise TypeError("pose must be a Pose6")

        normalized_name = str(name).strip()
        self.name = normalized_name if normalized_name != "" else "Zone"
        self.enabled = bool(enabled)
        self.shape = shape
        self.pose = Pose6.zeros() if pose is None else pose.copy()
        self.size_x = max(0.0, float(size_x))
        self.size_y = max(0.0, float(size_y))
        self.size_z = max(0.0, float(size_z))
        self.radius = max(0.0, float(radius))
        self.height = max(0.0, float(height))

    def copy(self) -> "PrimitiveColliderData":
        return PrimitiveColliderData(
            name=self.name,
            enabled=self.enabled,
            shape=self.shape,
            pose=self.pose,
            size_x=self.size_x,
            size_y=self.size_y,
            size_z=self.size_z,
            radius=self.radius,
            height=self.height,
        )

    def build_local_transform(self) -> np.ndarray:
        return _freeze_matrix(math_utils.pose_zyx_to_matrix(self.pose))

    def build_collider(
        self,
        owner: str = "workspace",
        base_transform: np.ndarray | None = None,
        attachment_key: str = "world",
        attachment_index: int = -1,
    ) -> PrimitiveCollider:
        return PrimitiveCollider(
            owner=owner,
            name=self.name,
            enabled=self.enabled,
            shape=self.shape,
            size_x=self.size_x,
            size_y=self.size_y,
            size_z=self.size_z,
            radius=self.radius,
            height=self.height,
            local_transform=self.build_local_transform(),
            base_transform=base_transform,
            attachment_key=attachment_key,
            attachment_index=attachment_index,
        )

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, PrimitiveColliderData):
            return False
        return (
            self.name == other.name
            and self.enabled == other.enabled
            and self.shape == other.shape
            and self.pose == other.pose
            and self.size_x == other.size_x
            and self.size_y == other.size_y
            and self.size_z == other.size_z
            and self.radius == other.radius
            and self.height == other.height
        )


class RobotAxisColliderData:
    def __init__(
        self,
        axis_index: int,
        enabled: bool = True,
        radius: float = 40.0,
        height: float = 200.0,
        direction_axis: AxisDirection = AxisDirection.Z,
        offset_xyz: XYZ3 | None = None,
    ) -> None:
        if not isinstance(direction_axis, AxisDirection):
            raise TypeError("direction_axis must be an AxisDirection")
        if offset_xyz is not None and not isinstance(offset_xyz, XYZ3):
            raise TypeError("offset_xyz must be an XYZ3")

        self.axis_index = max(0, int(axis_index))
        self.enabled = bool(enabled)
        self.radius = max(0.0, float(radius))
        self.height = float(height)
        self.direction_axis = direction_axis
        self.offset_xyz = XYZ3.zeros() if offset_xyz is None else offset_xyz.copy()

    def copy(self) -> "RobotAxisColliderData":
        return RobotAxisColliderData(
            axis_index=self.axis_index,
            enabled=self.enabled,
            radius=self.radius,
            height=self.height,
            direction_axis=self.direction_axis,
            offset_xyz=self.offset_xyz,
        )

    def build_local_transform(self) -> np.ndarray:
        translation = np.eye(4, dtype=float)
        translation[:3, 3] = np.array(self.offset_xyz.to_list(), dtype=float)
        orientation = primitive_extrusion_orientation(self.direction_axis, self.height >= 0.0)
        return _freeze_matrix(translation @ orientation)

    def build_collider(self, base_transform: np.ndarray | None = None) -> PrimitiveCollider:
        return PrimitiveCollider(
            owner="robot",
            name=f"Axis {self.axis_index + 1}",
            enabled=self.enabled,
            shape=PrimitiveColliderShape.CYLINDER,
            size_x=0.0,
            size_y=0.0,
            size_z=0.0,
            radius=self.radius,
            height=abs(self.height),
            local_transform=self.build_local_transform(),
            base_transform=base_transform,
            attachment_key=f"axis_{self.axis_index + 1}",
            attachment_index=self.axis_index + 1,
        )

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, RobotAxisColliderData):
            return False
        return (
            self.axis_index == other.axis_index
            and self.enabled == other.enabled
            and self.radius == other.radius
            and self.height == other.height
            and self.direction_axis == other.direction_axis
            and self.offset_xyz == other.offset_xyz
        )


def default_axis_colliders(axis_count: int = 6) -> list[RobotAxisColliderData]:
    axis_total = max(0, axis_count)
    default_directions = (
        AxisDirection.Z,
        AxisDirection.X,
        AxisDirection.Y,
        AxisDirection.Z,
        AxisDirection.Y,
        AxisDirection.Z,
    )
    enabled_axis_indexes = {0, 1, 2, 4}
    return [
        RobotAxisColliderData(
            axis_index=index,
            enabled=index in enabled_axis_indexes,
            radius=40.0,
            height=100.0,
            direction_axis=default_directions[index] if index < 6 else AxisDirection.Z,
            offset_xyz=XYZ3.zeros(),
        )
        for index in range(axis_total)
    ]


def build_primitive_colliders(
    values: list[PrimitiveColliderData],
    owner: str = "workspace",
    base_transform: np.ndarray | None = None,
    attachment_key: str = "world",
    attachment_index: int = -1,
) -> list[PrimitiveCollider]:
    return [
        value.build_collider(
            owner=owner,
            base_transform=base_transform,
            attachment_key=attachment_key,
            attachment_index=attachment_index,
        )
        for value in values
    ]
