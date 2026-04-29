from __future__ import annotations

from typing import Any

import numpy as np

import utils.math_utils as math_utils
from models.collider_models import normalize_pose6


SUPPORTED_WORKSPACE_PRIMITIVE_SHAPES = {"box", "cylinder", "sphere"}


class WorkspacePrimitiveZoneData:
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
        self.shape = self._normalize_shape(shape, "box")
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
    ) -> "WorkspacePrimitiveZoneData":
        if isinstance(raw, WorkspacePrimitiveZoneData):
            return raw.copy()

        data = raw if isinstance(raw, dict) else {}
        resolved_default_name = default_name if default_name is not None else f"Zone {index + 1}"
        pose = data.get("pose", data.get("xyzabc", data.get("transform")))
        return cls(
            name=str(data.get("name", resolved_default_name)),
            enabled=cls._safe_bool(data.get("enabled", data.get("active", True)), True),
            shape=cls._normalize_shape(data.get("shape", data.get("type", "box")), "box"),
            pose=pose,
            size_x=max(0.0, cls._safe_float(data.get("size_x", data.get("sx", 200.0)), 200.0)),
            size_y=max(0.0, cls._safe_float(data.get("size_y", data.get("sy", 200.0)), 200.0)),
            size_z=max(0.0, cls._safe_float(data.get("size_z", data.get("sz", 200.0)), 200.0)),
            radius=max(0.0, cls._safe_float(data.get("radius", data.get("r", 100.0)), 100.0)),
            height=max(0.0, cls._safe_float(data.get("height", data.get("h", 200.0)), 200.0)),
        )

    def copy(self) -> "WorkspacePrimitiveZoneData":
        return WorkspacePrimitiveZoneData(
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

    def build_collider(self) -> "WorkspacePrimitiveZoneCollider":
        return WorkspacePrimitiveZoneCollider(self)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, WorkspacePrimitiveZoneData):
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

    @staticmethod
    def _safe_float(value: Any, default: float = 0.0) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
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

    @staticmethod
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
        if normalized not in SUPPORTED_WORKSPACE_PRIMITIVE_SHAPES:
            return default
        return normalized


class WorkspacePrimitiveZoneCollider:
    def __init__(self, data: WorkspacePrimitiveZoneData) -> None:
        self.name = data.name
        self.enabled = data.enabled
        self.shape = data.shape
        self.pose = tuple(float(value) for value in data.pose[:6])
        self.size_x = float(data.size_x)
        self.size_y = float(data.size_y)
        self.size_z = float(data.size_z)
        self.radius = float(data.radius)
        self.height = float(data.height)
        self.world_transform = np.array(math_utils.pose_zyx_to_matrix(self.pose), dtype=float)
        self.world_transform.setflags(write=False)

    def copy(self) -> "WorkspacePrimitiveZoneCollider":
        return WorkspacePrimitiveZoneCollider(
            WorkspacePrimitiveZoneData(
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
        )


def parse_workspace_primitive_zones(raw_values: object) -> list[WorkspacePrimitiveZoneData]:
    values = raw_values if isinstance(raw_values, list) else []
    result: list[WorkspacePrimitiveZoneData] = []
    for index, raw_value in enumerate(values):
        result.append(WorkspacePrimitiveZoneData.from_raw(raw_value, index=index))
    return result


def workspace_primitive_zones_to_dict(
    values: list[WorkspacePrimitiveZoneData] | list[dict[str, Any]],
) -> list[dict[str, Any]]:
    normalized = parse_workspace_primitive_zones(values)
    return [value.to_dict() for value in normalized]


def build_workspace_primitive_zone_colliders(
    values: list[WorkspacePrimitiveZoneData] | list[dict[str, Any]],
) -> list[WorkspacePrimitiveZoneCollider]:
    normalized = parse_workspace_primitive_zones(values)
    return [value.build_collider() for value in normalized]
