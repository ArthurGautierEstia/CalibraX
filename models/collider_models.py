from __future__ import annotations

from typing import Any

import models.primitive_collider_models as primitive_collider_models
from utils.math_utils import safe_float

def normalize_xyz3(raw_xyz: Any) -> list[float]:
    if isinstance(raw_xyz, dict):
        return [
            safe_float(raw_xyz.get("x", 0.0), 0.0),
            safe_float(raw_xyz.get("y", 0.0), 0.0),
            safe_float(raw_xyz.get("z", 0.0), 0.0),
        ]
    if isinstance(raw_xyz, (list, tuple)):
        values = [safe_float(raw_xyz[idx] if idx < len(raw_xyz) else 0.0, 0.0) for idx in range(3)]
    else:
        values = [0.0, 0.0, 0.0]
    return values[:3]


def parse_primitive_colliders(
    raw_values: object,
    default_shape: primitive_collider_models.PrimitiveColliderShape = (
        primitive_collider_models.PrimitiveColliderShape.CYLINDER
    ),
) -> list["primitive_collider_models.PrimitiveColliderData"]:
    return primitive_collider_models.parse_primitive_collider_data(
        raw_values,
        default_shape=default_shape,
        default_name_prefix="Collider",
    )


def primitive_collider_to_dict(
    collider: "primitive_collider_models.PrimitiveColliderData | dict[str, Any]",
) -> dict[str, Any]:
    return primitive_collider_models.PrimitiveColliderData.from_raw(
        collider,
        default_name="Collider",
        default_shape=primitive_collider_models.PrimitiveColliderShape.CYLINDER,
    ).to_dict()


def default_axis_colliders(axis_count: int = 6) -> list["primitive_collider_models.RobotAxisColliderData"]:
    axis_total = max(0, axis_count)
    default_directions = (
        primitive_collider_models.AxisDirection.Z,
        primitive_collider_models.AxisDirection.X,
        primitive_collider_models.AxisDirection.Y,
        primitive_collider_models.AxisDirection.Y,
        primitive_collider_models.AxisDirection.Y,
        primitive_collider_models.AxisDirection.Z,
    )
    return [
        primitive_collider_models.RobotAxisColliderData(
            axis_index=index,
            enabled=True,
            radius=40.0,
            height=200.0,
            direction_axis=(
                default_directions[index]
                if index < 6
                else primitive_collider_models.AxisDirection.Z
            ),
            offset_xyz=(0.0, 0.0, 0.0),
        )
        for index in range(axis_total)
    ]


def parse_axis_colliders(
    raw_values: object,
    axis_count: int = 6,
) -> list["primitive_collider_models.RobotAxisColliderData"]:
    return primitive_collider_models.parse_robot_axis_colliders(raw_values, axis_count=axis_count)


def axis_colliders_to_dict(
    values: list["primitive_collider_models.RobotAxisColliderData"] | list[dict[str, Any]],
    axis_count: int = 6,
) -> list[dict[str, Any]]:
    return primitive_collider_models.robot_axis_colliders_to_dicts(values, axis_count=axis_count)
