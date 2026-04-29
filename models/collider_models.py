from __future__ import annotations

from typing import Any, TYPE_CHECKING

from utils.math_utils import safe_float

if TYPE_CHECKING:
    from models.primitive_collider_models import PrimitiveColliderData, RobotAxisColliderData

def normalize_pose6(raw_pose: Any) -> list[float]:
    if isinstance(raw_pose, dict):
        return [
            safe_float(raw_pose.get("x", 0.0), 0.0),
            safe_float(raw_pose.get("y", 0.0), 0.0),
            safe_float(raw_pose.get("z", 0.0), 0.0),
            safe_float(raw_pose.get("a", 0.0), 0.0),
            safe_float(raw_pose.get("b", 0.0), 0.0),
            safe_float(raw_pose.get("c", 0.0), 0.0),
        ]
    if isinstance(raw_pose, (list, tuple)):
        values = [safe_float(raw_pose[idx] if idx < len(raw_pose) else 0.0, 0.0) for idx in range(6)]
        return values[:6]
    return [0.0] * 6


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
    default_shape: str = "cylinder",
) -> list["PrimitiveColliderData"]:
    from models.primitive_collider_models import parse_primitive_collider_data

    return parse_primitive_collider_data(
        raw_values,
        default_shape=default_shape,
        default_name_prefix="Collider",
    )


def primitive_collider_to_dict(collider: "PrimitiveColliderData | dict[str, Any]") -> dict[str, Any]:
    from models.primitive_collider_models import PrimitiveColliderData

    return PrimitiveColliderData.from_raw(
        collider,
        default_name="Collider",
        default_shape="cylinder",
    ).to_dict()


def default_axis_colliders(axis_count: int = 6) -> list["RobotAxisColliderData"]:
    from models.primitive_collider_models import RobotAxisColliderData

    axis_total = max(0, axis_count)
    default_directions = ("z", "x", "y", "y", "y", "z")
    return [
        RobotAxisColliderData(
            axis_index=index,
            enabled=True,
            radius=40.0,
            height=200.0,
            direction_axis=default_directions[index] if index < 6 else "z",
            offset_xyz=(0.0, 0.0, 0.0),
        )
        for index in range(axis_total)
    ]


def parse_axis_colliders(raw_values: object, axis_count: int = 6) -> list["RobotAxisColliderData"]:
    from models.primitive_collider_models import parse_robot_axis_colliders

    return parse_robot_axis_colliders(raw_values, axis_count=axis_count)


def axis_colliders_to_dict(
    values: list["RobotAxisColliderData"] | list[dict[str, Any]],
    axis_count: int = 6,
) -> list[dict[str, Any]]:
    from models.primitive_collider_models import robot_axis_colliders_to_dicts

    return robot_axis_colliders_to_dicts(values, axis_count=axis_count)
