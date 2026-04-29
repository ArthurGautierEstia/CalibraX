from __future__ import annotations

from typing import Any

import numpy as np

import utils.math_utils as math_utils
from utils.math_utils import safe_float
from models.collider_models import (
    default_axis_colliders,
    normalize_pose6,
    normalize_xyz3,
)


SUPPORTED_PRIMITIVE_COLLIDER_SHAPES = {"box", "cylinder", "sphere"}
SUPPORTED_AXIS_DIRECTIONS = {"x", "y", "z"}

def _safe_bool(value: Any, default: bool = True) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"1", "true", "yes", "on", "enabled", "active"}:
            return True
        if lowered in {"0", "false", "no", "off", "disabled", "inactive"}:
            return False
    return default


def _normalize_shape(value: Any, default: str = "box") -> str:
    if value is None:
        return default
    raw = str(value).strip().lower()
    mapping = {
        "pave": "box",
        "pave_droit": "box",
        "cuboid": "box",
        "cube": "box",
        "cylindre": "cylinder",
        "cylinder": "cylinder",
        "sphere": "sphere",
        "box": "box",
    }
    normalized = mapping.get(raw, raw)
    if normalized not in SUPPORTED_PRIMITIVE_COLLIDER_SHAPES:
        return default
    return normalized


def _normalize_axis_direction(value: Any, default: str = "z") -> str:
    if value is None:
        return default
    raw = str(value).strip().lower()
    mapping = {
        "x": "x",
        "axe_x": "x",
        "axis_x": "x",
        "y": "y",
        "axe_y": "y",
        "axis_y": "y",
        "z": "z",
        "axe_z": "z",
        "axis_z": "z",
    }
    normalized = mapping.get(raw, raw)
    if normalized not in SUPPORTED_AXIS_DIRECTIONS:
        return default
    return normalized


def _freeze_matrix(matrix: np.ndarray | list[list[float]]) -> np.ndarray:
    normalized = np.array(matrix, dtype=float)
    normalized.setflags(write=False)
    return normalized


def _build_orientation_matrix(direction_axis: str, positive_direction: bool) -> np.ndarray:
    rotation = np.eye(4, dtype=float)
    normalized_axis = _normalize_axis_direction(direction_axis, "z")
    if normalized_axis == "x":
        rotation[:3, :3] = math_utils.rot_y(90.0, degrees=True)
    elif normalized_axis == "y":
        rotation[:3, :3] = math_utils.rot_x(-90.0, degrees=True)

    if positive_direction:
        return _freeze_matrix(rotation)

    flip = np.eye(4, dtype=float)
    flip[:3, :3] = math_utils.rot_x(180.0, degrees=True)
    return _freeze_matrix(rotation @ flip)


_PRIMITIVE_EXTRUSION_ORIENTATIONS: dict[tuple[str, bool], np.ndarray] = {
    (axis_name, positive): _build_orientation_matrix(axis_name, positive)
    for axis_name in ("x", "y", "z")
    for positive in (False, True)
}


def primitive_extrusion_orientation(direction_axis: str, positive_direction: bool = True) -> np.ndarray:
    key = (_normalize_axis_direction(direction_axis, "z"), bool(positive_direction))
    return _PRIMITIVE_EXTRUSION_ORIENTATIONS[key]


class PrimitiveCollider:
    def __init__(
        self,
        owner: str,
        name: str,
        enabled: bool,
        shape: str,
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
        self.owner = str(owner).strip().lower() or "workspace"
        normalized_name = str(name).strip()
        self.name = normalized_name if normalized_name != "" else "Collider"
        self.enabled = bool(enabled)
        self.shape = _normalize_shape(shape, "box")
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
        shape: str = "box",
        pose: list[float] | tuple[float, ...] | None = None,
        size_x: float = 200.0,
        size_y: float = 200.0,
        size_z: float = 200.0,
        radius: float = 100.0,
        height: float = 200.0,
    ) -> None:
        normalized_pose = normalize_pose6([0.0] * 6 if pose is None else pose)
        normalized_name = str(name).strip()
        self.name = normalized_name if normalized_name != "" else "Zone"
        self.enabled = bool(enabled)
        self.shape = _normalize_shape(shape, "box")
        self.pose = tuple(float(value) for value in normalized_pose[:6])
        self.size_x = max(0.0, float(size_x))
        self.size_y = max(0.0, float(size_y))
        self.size_z = max(0.0, float(size_z))
        self.radius = max(0.0, float(radius))
        self.height = max(0.0, float(height))

    @classmethod
    def from_raw(
        cls,
        raw: object,
        index: int = 0,
        default_name: str | None = None,
        default_shape: str = "box",
    ) -> "PrimitiveColliderData":
        if isinstance(raw, PrimitiveColliderData):
            return raw.copy()

        data = raw if isinstance(raw, dict) else {}
        resolved_default_name = default_name if default_name is not None else f"Zone {index + 1}"
        pose = data.get("pose", data.get("xyzabc", data.get("transform")))
        return cls(
            name=str(data.get("name", resolved_default_name)),
            enabled=_safe_bool(data.get("enabled", data.get("active", True)), True),
            shape=_normalize_shape(data.get("shape", data.get("type", default_shape)), default_shape),
            pose=pose,
            size_x=max(0.0, safe_float(data.get("size_x", data.get("sx", 200.0)), 200.0)),
            size_y=max(0.0, safe_float(data.get("size_y", data.get("sy", 200.0)), 200.0)),
            size_z=max(0.0, safe_float(data.get("size_z", data.get("sz", 200.0)), 200.0)),
            radius=max(0.0, safe_float(data.get("radius", data.get("r", 100.0)), 100.0)),
            height=max(0.0, safe_float(data.get("height", data.get("h", 200.0)), 200.0)),
        )

    def copy(self) -> "PrimitiveColliderData":
        return PrimitiveColliderData(
            name=self.name,
            enabled=self.enabled,
            shape=self.shape,
            pose=list(self.pose),
            size_x=self.size_x,
            size_y=self.size_y,
            size_z=self.size_z,
            radius=self.radius,
            height=self.height,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "enabled": self.enabled,
            "shape": self.shape,
            "pose": [float(value) for value in self.pose],
            "size_x": float(self.size_x),
            "size_y": float(self.size_y),
            "size_z": float(self.size_z),
            "radius": float(self.radius),
            "height": float(self.height),
        }

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
        direction_axis: str = "z",
        offset_xyz: list[float] | tuple[float, ...] | None = None,
    ) -> None:
        self.axis_index = max(0, int(axis_index))
        self.enabled = bool(enabled)
        self.radius = max(0.0, float(radius))
        self.height = float(height)
        self.direction_axis = _normalize_axis_direction(direction_axis, "z")
        normalized_offset = normalize_xyz3([0.0, 0.0, 0.0] if offset_xyz is None else offset_xyz)
        self.offset_xyz = tuple(float(value) for value in normalized_offset[:3])

    @classmethod
    def from_raw(cls, raw: object, axis_index: int = 0) -> "RobotAxisColliderData":
        if isinstance(raw, RobotAxisColliderData):
            return raw.copy()

        defaults = default_axis_colliders(max(axis_index + 1, 6))
        base_value = defaults[axis_index] if axis_index < len(defaults) else None
        data = base_value.to_dict() if isinstance(base_value, RobotAxisColliderData) else {}
        if isinstance(raw, dict):
            data.update(raw)

        return cls(
            axis_index=int(data.get("axis", axis_index)),
            enabled=_safe_bool(data.get("enabled", data.get("active", True)), True),
            radius=max(0.0, safe_float(data.get("radius", data.get("r", 40.0)), 40.0)),
            height=float(safe_float(data.get("height", 200.0), 200.0)),
            direction_axis=_normalize_axis_direction(
                data.get("direction_axis", data.get("axis_direction", data.get("orientation_axis", "z"))),
                "z",
            ),
            offset_xyz=normalize_xyz3(data.get("offset_xyz")),
        )

    def copy(self) -> "RobotAxisColliderData":
        return RobotAxisColliderData(
            axis_index=self.axis_index,
            enabled=self.enabled,
            radius=self.radius,
            height=self.height,
            direction_axis=self.direction_axis,
            offset_xyz=list(self.offset_xyz),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "axis": self.axis_index,
            "enabled": self.enabled,
            "radius": float(self.radius),
            "height": float(self.height),
            "direction_axis": self.direction_axis,
            "offset_xyz": [float(value) for value in self.offset_xyz],
        }

    def build_local_transform(self) -> np.ndarray:
        translation = np.eye(4, dtype=float)
        translation[:3, 3] = np.array(self.offset_xyz, dtype=float)
        orientation = primitive_extrusion_orientation(self.direction_axis, self.height >= 0.0)
        return _freeze_matrix(translation @ orientation)

    def build_collider(self, base_transform: np.ndarray | None = None) -> PrimitiveCollider:
        return PrimitiveCollider(
            owner="robot",
            name=f"Axis {self.axis_index + 1}",
            enabled=self.enabled,
            shape="cylinder",
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


def parse_primitive_collider_data(
    raw_values: object,
    default_shape: str = "box",
    default_name_prefix: str = "Zone",
) -> list[PrimitiveColliderData]:
    values = raw_values if isinstance(raw_values, list) else []
    result: list[PrimitiveColliderData] = []
    for index, raw_value in enumerate(values):
        result.append(
            PrimitiveColliderData.from_raw(
                raw_value,
                index=index,
                default_name=f"{default_name_prefix} {index + 1}",
                default_shape=default_shape,
            )
        )
    return result


def primitive_collider_data_to_dicts(
    values: list[PrimitiveColliderData] | list[dict[str, Any]],
    default_shape: str = "box",
    default_name_prefix: str = "Zone",
) -> list[dict[str, Any]]:
    normalized = parse_primitive_collider_data(
        values,
        default_shape=default_shape,
        default_name_prefix=default_name_prefix,
    )
    return [value.to_dict() for value in normalized]


def build_primitive_colliders(
    values: list[PrimitiveColliderData] | list[dict[str, Any]],
    owner: str = "workspace",
    base_transform: np.ndarray | None = None,
    attachment_key: str = "world",
    attachment_index: int = -1,
    default_shape: str = "box",
    default_name_prefix: str = "Zone",
) -> list[PrimitiveCollider]:
    normalized = parse_primitive_collider_data(
        values,
        default_shape=default_shape,
        default_name_prefix=default_name_prefix,
    )
    return [
        value.build_collider(
            owner=owner,
            base_transform=base_transform,
            attachment_key=attachment_key,
            attachment_index=attachment_index,
        )
        for value in normalized
    ]


def parse_robot_axis_colliders(raw_values: object, axis_count: int = 6) -> list[RobotAxisColliderData]:
    values = raw_values if isinstance(raw_values, list) else []
    result: list[RobotAxisColliderData] = []
    for axis_index in range(max(0, axis_count)):
        raw_value = values[axis_index] if axis_index < len(values) else {}
        result.append(RobotAxisColliderData.from_raw(raw_value, axis_index=axis_index))
    return result


def robot_axis_colliders_to_dicts(
    values: list[RobotAxisColliderData] | list[dict[str, Any]],
    axis_count: int = 6,
) -> list[dict[str, Any]]:
    normalized = parse_robot_axis_colliders(values, axis_count=axis_count)
    return [value.to_dict() for value in normalized]
