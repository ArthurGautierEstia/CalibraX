from __future__ import annotations

from typing import Any

import models.primitive_collider_models as primitive_collider_models


def normalize_xyz3(raw_xyz: Any) -> list[float]:
    return primitive_collider_models.normalize_xyz3(raw_xyz)


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
    return primitive_collider_models.default_axis_colliders(axis_count)


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
