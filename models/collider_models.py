from __future__ import annotations

from typing import Any


SUPPORTED_PRIMITIVE_SHAPES = {"box", "cylinder", "sphere"}
SUPPORTED_AXIS_DIRECTIONS = {"x", "y", "z"}


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


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


def _safe_shape(value: Any, default: str = "cylinder") -> str:
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
    if normalized not in SUPPORTED_PRIMITIVE_SHAPES:
        return default
    return normalized


def _safe_axis_direction(value: Any, default: str = "z") -> str:
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


def normalize_pose6(raw_pose: Any) -> list[float]:
    pose: list[float] = []
    if isinstance(raw_pose, dict):
        keys = ["x", "y", "z", "a", "b", "c"]
        pose = [_safe_float(raw_pose.get(key, 0.0), 0.0) for key in keys]
    elif isinstance(raw_pose, list):
        pose = [_safe_float(raw_pose[idx] if idx < len(raw_pose) else 0.0, 0.0) for idx in range(6)]
    else:
        pose = [0.0] * 6

    while len(pose) < 6:
        pose.append(0.0)
    return pose[:6]


def normalize_xyz3(raw_xyz: Any) -> list[float]:
    if isinstance(raw_xyz, (list, tuple)):
        values = [_safe_float(raw_xyz[idx] if idx < len(raw_xyz) else 0.0, 0.0) for idx in range(3)]
    else:
        values = [0.0, 0.0, 0.0]
    return values[:3]


def normalize_primitive_collider_dict(
    raw: Any,
    default_name: str = "Collider",
    default_shape: str = "cylinder",
) -> dict[str, Any]:
    data = raw if isinstance(raw, dict) else {}
    pose = normalize_pose6(data.get("pose", data.get("xyzabc", data.get("transform"))))

    return {
        "name": str(data.get("name", default_name)),
        "enabled": _safe_bool(data.get("enabled", data.get("active", True)), True),
        "shape": _safe_shape(data.get("shape", data.get("type", default_shape)), default_shape),
        "pose": pose,
        "size_x": max(0.0, _safe_float(data.get("size_x", data.get("sx", 100.0)), 100.0)),
        "size_y": max(0.0, _safe_float(data.get("size_y", data.get("sy", 100.0)), 100.0)),
        "size_z": max(0.0, _safe_float(data.get("size_z", data.get("sz", 100.0)), 100.0)),
        "radius": max(0.0, _safe_float(data.get("radius", data.get("r", 50.0)), 50.0)),
        "height": max(0.0, _safe_float(data.get("height", data.get("h", 100.0)), 100.0)),
    }


def parse_primitive_colliders(
    raw_values: Any,
    default_shape: str = "cylinder",
) -> list[dict[str, Any]]:
    values = raw_values if isinstance(raw_values, list) else []
    result: list[dict[str, Any]] = []
    for index, raw_value in enumerate(values):
        result.append(
            normalize_primitive_collider_dict(
                raw_value,
                default_name=f"Collider {index + 1}",
                default_shape=default_shape,
            )
        )
    return result


def primitive_collider_to_dict(collider: dict[str, Any]) -> dict[str, Any]:
    normalized = normalize_primitive_collider_dict(collider)
    return {
        "name": normalized["name"],
        "enabled": normalized["enabled"],
        "shape": normalized["shape"],
        "pose": [float(v) for v in normalized["pose"]],
        "size_x": float(normalized["size_x"]),
        "size_y": float(normalized["size_y"]),
        "size_z": float(normalized["size_z"]),
        "radius": float(normalized["radius"]),
        "height": float(normalized["height"]),
    }


def default_axis_colliders(axis_count: int = 6) -> list[dict[str, Any]]:
    defaults: list[dict[str, Any]] = []
    for index in range(max(0, axis_count)):
        defaults.append(
            {
                "axis": index,
                "enabled": True,
                "radius": 40.0,
                "direction_axis": "z",
                "height": 200.0,
                "offset_xyz": [0.0, 0.0, 0.0],
            }
        )

    # Preset simple des axes (q1..q6)
    if len(defaults) >= 6:
        defaults[0]["direction_axis"] = "z"  # Q1
        defaults[1]["direction_axis"] = "x"  # Q2
        defaults[2]["direction_axis"] = "y"  # Q3
        defaults[3]["direction_axis"] = "y"  # Q4
        defaults[4]["direction_axis"] = "y"  # Q5
        defaults[5]["direction_axis"] = "z"  # Q6

    return defaults


def normalize_axis_collider_dict(raw: Any, axis_index: int) -> dict[str, Any]:
    data = raw if isinstance(raw, dict) else {}
    height = _safe_float(data.get("height", 200.0), 200.0)

    return {
        "axis": axis_index,
        "enabled": _safe_bool(data.get("enabled", data.get("active", True)), True),
        "radius": max(0.0, _safe_float(data.get("radius", data.get("r", 40.0)), 40.0)),
        "height": float(height),
        "direction_axis": _safe_axis_direction(
            data.get("direction_axis", data.get("axis_direction", data.get("orientation_axis", "z"))),
            "z",
        ),
        "offset_xyz": normalize_xyz3(data.get("offset_xyz")),
    }


def parse_axis_colliders(raw_values: Any, axis_count: int = 6) -> list[dict[str, Any]]:
    values = raw_values if isinstance(raw_values, list) else []
    defaults = default_axis_colliders(axis_count)
    parsed: list[dict[str, Any]] = []
    for axis_index in range(max(0, axis_count)):
        base_value = defaults[axis_index] if axis_index < len(defaults) else {}
        raw_value = values[axis_index] if axis_index < len(values) else {}
        merged_value = dict(base_value)
        if isinstance(raw_value, dict):
            merged_value.update(raw_value)
        parsed.append(normalize_axis_collider_dict(merged_value, axis_index))
    return parsed


def axis_colliders_to_dict(values: list[dict[str, Any]], axis_count: int = 6) -> list[dict[str, Any]]:
    return parse_axis_colliders(values, axis_count)
