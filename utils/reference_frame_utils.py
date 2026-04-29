from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Iterable

import numpy as np

import utils.math_utils as math_utils
from utils.math_utils import safe_float
from models.pose6 import Pose6
from models.reference_frame import ReferenceFrame


@dataclass(frozen=True)
class FrameTransform:
    """Cached transform context for a frame pose expressed in world."""

    pose: Pose6
    matrix: np.ndarray
    inverse_matrix: np.ndarray
    rotation: np.ndarray
    inverse_rotation: np.ndarray
    translation: np.ndarray
    revision: int = 0

    @classmethod
    def from_pose(cls, pose: object, revision: int = 0) -> "FrameTransform":
        values = normalize_pose6(pose)
        matrix = math_utils.pose_zyx_to_matrix(values)
        inverse_matrix = math_utils.invert_homogeneous_transform(matrix)
        rotation = matrix[:3, :3].copy()
        inverse_rotation = inverse_matrix[:3, :3].copy()
        translation = matrix[:3, 3].copy()
        for array in (matrix, inverse_matrix, rotation, inverse_rotation, translation):
            array.setflags(write=False)
        return cls(
            pose=values,
            matrix=matrix,
            inverse_matrix=inverse_matrix,
            rotation=rotation,
            inverse_rotation=inverse_rotation,
            translation=translation,
            revision=int(revision),
        )

    def pose_list(self) -> list[float]:
        return self.pose.to_list()


def _as_frame_transform(value: object) -> FrameTransform:
    if isinstance(value, FrameTransform):
        return value
    return FrameTransform.from_pose(value)


def normalize_pose6(values: object) -> Pose6:
    if isinstance(values, Pose6):
        return values.copy()
    if isinstance(values, dict):
        return Pose6.from_mapping(values)
    if isinstance(values, Iterable) and not isinstance(values, (str, bytes)):
        seq = list(values)
        normalized = [safe_float(seq[idx] if idx < len(seq) else 0.0, 0.0) for idx in range(6)]
        return Pose6.from_sequence(normalized, fill_missing=True)
    return Pose6.zeros()


def pose_to_matrix(pose: object) -> np.ndarray:
    return math_utils.pose_zyx_to_matrix(normalize_pose6(pose))


def matrix_to_pose(transform: np.ndarray) -> Pose6:
    return math_utils.matrix_to_pose_zyx(transform)


def base_pose_world_to_matrix(robot_base_pose_world: object) -> np.ndarray:
    return _as_frame_transform(robot_base_pose_world).matrix.copy()


def pose_base_to_world(pose_base: object, robot_base_pose_world: object) -> Pose6:
    frame = _as_frame_transform(robot_base_pose_world)
    transform = frame.matrix @ pose_to_matrix(pose_base)
    return matrix_to_pose(transform)


def pose_world_to_base(pose_world: object, robot_base_pose_world: object) -> Pose6:
    frame = _as_frame_transform(robot_base_pose_world)
    return matrix_to_pose(frame.inverse_matrix @ pose_to_matrix(pose_world))


def xyz_base_to_world(xyz_base: object, robot_base_pose_world: object) -> list[float]:
    values = normalize_pose6(xyz_base)[:3]
    frame = _as_frame_transform(robot_base_pose_world)
    point = frame.matrix @ np.array([values[0], values[1], values[2], 1.0], dtype=float)
    return [float(point[0]), float(point[1]), float(point[2])]


def xyz_world_to_base(xyz_world: object, robot_base_pose_world: object) -> list[float]:
    values = normalize_pose6(xyz_world)[:3]
    frame = _as_frame_transform(robot_base_pose_world)
    point = frame.inverse_matrix @ np.array([values[0], values[1], values[2], 1.0], dtype=float)
    return [float(point[0]), float(point[1]), float(point[2])]


def twist_base_to_world(twist_base: object, robot_base_pose_world: object) -> Pose6:
    values = normalize_pose6(twist_base)
    frame = _as_frame_transform(robot_base_pose_world)
    linear = frame.rotation @ np.array(values[:3], dtype=float)
    angular = frame.rotation @ np.array(values[3:6], dtype=float)
    return Pose6.from_sequence(np.concatenate([linear, angular]).tolist())


def twist_world_to_base(twist_world: object, robot_base_pose_world: object) -> Pose6:
    values = normalize_pose6(twist_world)
    frame = _as_frame_transform(robot_base_pose_world)
    linear = frame.inverse_rotation @ np.array(values[:3], dtype=float)
    angular = frame.inverse_rotation @ np.array(values[3:6], dtype=float)
    return Pose6.from_sequence(np.concatenate([linear, angular]).tolist())


def transform_matrix_base_to_world(transform: np.ndarray, robot_base_pose_world: object) -> np.ndarray:
    frame = _as_frame_transform(robot_base_pose_world)
    return frame.matrix @ np.array(transform, dtype=float)


def transform_points_base_to_world(points_xyz: np.ndarray, robot_base_pose_world: object) -> np.ndarray:
    points = np.array(points_xyz, dtype=float)
    if points.size == 0:
        return points
    frame = _as_frame_transform(robot_base_pose_world)
    return (points @ frame.rotation.T) + frame.translation


def convert_pose_to_base_frame(
    pose: object,
    reference_frame: ReferenceFrame | str,
    robot_base_pose_world: object,
) -> Pose6:
    frame = ReferenceFrame.from_value(reference_frame)
    if frame == ReferenceFrame.WORLD:
        return pose_world_to_base(pose, robot_base_pose_world)
    return normalize_pose6(pose)


def convert_pose_from_base_frame(
    pose_base: object,
    reference_frame: ReferenceFrame | str,
    robot_base_pose_world: object,
) -> Pose6:
    frame = ReferenceFrame.from_value(reference_frame)
    if frame == ReferenceFrame.WORLD:
        return pose_base_to_world(pose_base, robot_base_pose_world)
    return normalize_pose6(pose_base)
