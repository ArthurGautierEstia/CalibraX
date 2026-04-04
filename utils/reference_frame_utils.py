from __future__ import annotations

from typing import Iterable

import numpy as np

import utils.math_utils as math_utils
from models.reference_frame import ReferenceFrame


def normalize_pose6(values: object) -> list[float]:
    if isinstance(values, dict):
        out = [
            float(values.get("x", 0.0)),
            float(values.get("y", 0.0)),
            float(values.get("z", 0.0)),
            float(values.get("a", 0.0)),
            float(values.get("b", 0.0)),
            float(values.get("c", 0.0)),
        ]
    elif isinstance(values, Iterable) and not isinstance(values, (str, bytes)):
        seq = list(values)
        out = []
        for idx in range(6):
            try:
                out.append(float(seq[idx] if idx < len(seq) else 0.0))
            except (TypeError, ValueError):
                out.append(0.0)
    else:
        out = [0.0] * 6
    return out[:6]


def pose_to_matrix(pose: object) -> np.ndarray:
    values = normalize_pose6(pose)
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = math_utils.euler_to_rotation_matrix(values[3], values[4], values[5], degrees=True)
    transform[:3, 3] = [values[0], values[1], values[2]]
    return transform


def matrix_to_pose(transform: np.ndarray) -> list[float]:
    matrix = np.array(transform, dtype=float)
    angles = math_utils.rotation_matrix_to_euler_zyx(matrix[:3, :3])
    return [
        float(matrix[0, 3]),
        float(matrix[1, 3]),
        float(matrix[2, 3]),
        float(angles[0]),
        float(angles[1]),
        float(angles[2]),
    ]


def base_pose_world_to_matrix(robot_base_pose_world: object) -> np.ndarray:
    return pose_to_matrix(robot_base_pose_world)


def pose_base_to_world(pose_base: object, robot_base_pose_world: object) -> list[float]:
    transform = base_pose_world_to_matrix(robot_base_pose_world) @ pose_to_matrix(pose_base)
    return matrix_to_pose(transform)


def pose_world_to_base(pose_world: object, robot_base_pose_world: object) -> list[float]:
    world_to_base = np.linalg.inv(base_pose_world_to_matrix(robot_base_pose_world))
    return matrix_to_pose(world_to_base @ pose_to_matrix(pose_world))


def xyz_base_to_world(xyz_base: object, robot_base_pose_world: object) -> list[float]:
    values = normalize_pose6(xyz_base)[:3]
    transform = base_pose_world_to_matrix(robot_base_pose_world)
    point = transform @ np.array([values[0], values[1], values[2], 1.0], dtype=float)
    return [float(point[0]), float(point[1]), float(point[2])]


def xyz_world_to_base(xyz_world: object, robot_base_pose_world: object) -> list[float]:
    values = normalize_pose6(xyz_world)[:3]
    transform = np.linalg.inv(base_pose_world_to_matrix(robot_base_pose_world))
    point = transform @ np.array([values[0], values[1], values[2], 1.0], dtype=float)
    return [float(point[0]), float(point[1]), float(point[2])]


def twist_base_to_world(twist_base: object, robot_base_pose_world: object) -> list[float]:
    values = normalize_pose6(twist_base)
    rotation = base_pose_world_to_matrix(robot_base_pose_world)[:3, :3]
    linear = rotation @ np.array(values[:3], dtype=float)
    angular = rotation @ np.array(values[3:6], dtype=float)
    return [float(v) for v in np.concatenate([linear, angular])]


def twist_world_to_base(twist_world: object, robot_base_pose_world: object) -> list[float]:
    values = normalize_pose6(twist_world)
    rotation = np.linalg.inv(base_pose_world_to_matrix(robot_base_pose_world)[:3, :3])
    linear = rotation @ np.array(values[:3], dtype=float)
    angular = rotation @ np.array(values[3:6], dtype=float)
    return [float(v) for v in np.concatenate([linear, angular])]


def transform_matrix_base_to_world(transform: np.ndarray, robot_base_pose_world: object) -> np.ndarray:
    return base_pose_world_to_matrix(robot_base_pose_world) @ np.array(transform, dtype=float)


def transform_points_base_to_world(points_xyz: np.ndarray, robot_base_pose_world: object) -> np.ndarray:
    points = np.array(points_xyz, dtype=float)
    if points.size == 0:
        return points
    rotation = base_pose_world_to_matrix(robot_base_pose_world)[:3, :3]
    translation = base_pose_world_to_matrix(robot_base_pose_world)[:3, 3]
    return (points @ rotation.T) + translation


def convert_pose_to_base_frame(
    pose: object,
    reference_frame: ReferenceFrame | str,
    robot_base_pose_world: object,
) -> list[float]:
    frame = ReferenceFrame.from_value(reference_frame)
    if frame == ReferenceFrame.WORLD:
        return pose_world_to_base(pose, robot_base_pose_world)
    return normalize_pose6(pose)


def convert_pose_from_base_frame(
    pose_base: object,
    reference_frame: ReferenceFrame | str,
    robot_base_pose_world: object,
) -> list[float]:
    frame = ReferenceFrame.from_value(reference_frame)
    if frame == ReferenceFrame.WORLD:
        return pose_base_to_world(pose_base, robot_base_pose_world)
    return normalize_pose6(pose_base)
