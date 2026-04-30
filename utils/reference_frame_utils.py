from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import utils.math_utils as math_utils
from models.reference_frame import ReferenceFrame
from models.types import Pose6, XYZ3


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
    def from_pose(cls, pose: Pose6, revision: int = 0) -> "FrameTransform":
        if not isinstance(pose, Pose6):
            raise TypeError("pose must be a Pose6")

        matrix = math_utils.pose_zyx_to_matrix(pose)
        inverse_matrix = math_utils.invert_homogeneous_transform(matrix)
        rotation = matrix[:3, :3].copy()
        inverse_rotation = inverse_matrix[:3, :3].copy()
        translation = matrix[:3, 3].copy()
        for array in (matrix, inverse_matrix, rotation, inverse_rotation, translation):
            array.setflags(write=False)
        return cls(
            pose=pose.copy(),
            matrix=matrix,
            inverse_matrix=inverse_matrix,
            rotation=rotation,
            inverse_rotation=inverse_rotation,
            translation=translation,
            revision=int(revision),
        )

    def pose_list(self) -> list[float]:
        return self.pose.to_list()


def _as_frame_transform(value: FrameTransform | Pose6) -> FrameTransform:
    if isinstance(value, FrameTransform):
        return value
    return FrameTransform.from_pose(value)


def pose_to_matrix(pose: Pose6) -> np.ndarray:
    return math_utils.pose_zyx_to_matrix(pose)


def matrix_to_pose(transform: np.ndarray) -> Pose6:
    return math_utils.matrix_to_pose_zyx(transform)


def base_pose_world_to_matrix(robot_base_pose_world: FrameTransform | Pose6) -> np.ndarray:
    return _as_frame_transform(robot_base_pose_world).matrix.copy()


def pose_base_to_world(pose_base: Pose6, robot_base_pose_world: FrameTransform | Pose6) -> Pose6:
    frame = _as_frame_transform(robot_base_pose_world)
    transform = frame.matrix @ pose_to_matrix(pose_base)
    return matrix_to_pose(transform)


def pose_world_to_base(pose_world: Pose6, robot_base_pose_world: FrameTransform | Pose6) -> Pose6:
    frame = _as_frame_transform(robot_base_pose_world)
    return matrix_to_pose(frame.inverse_matrix @ pose_to_matrix(pose_world))


def xyz_base_to_world(xyz_base: XYZ3, robot_base_pose_world: FrameTransform | Pose6) -> XYZ3:
    frame = _as_frame_transform(robot_base_pose_world)
    point = frame.matrix @ np.array([xyz_base.x, xyz_base.y, xyz_base.z, 1.0], dtype=float)
    return XYZ3(float(point[0]), float(point[1]), float(point[2]))


def xyz_world_to_base(xyz_world: XYZ3, robot_base_pose_world: FrameTransform | Pose6) -> XYZ3:
    frame = _as_frame_transform(robot_base_pose_world)
    point = frame.inverse_matrix @ np.array([xyz_world.x, xyz_world.y, xyz_world.z, 1.0], dtype=float)
    return XYZ3(float(point[0]), float(point[1]), float(point[2]))


def twist_base_to_world(twist_base: Pose6, robot_base_pose_world: FrameTransform | Pose6) -> Pose6:
    frame = _as_frame_transform(robot_base_pose_world)
    linear = frame.rotation @ np.array([twist_base.x, twist_base.y, twist_base.z], dtype=float)
    angular = frame.rotation @ np.array([twist_base.a, twist_base.b, twist_base.c], dtype=float)
    return Pose6(linear[0], linear[1], linear[2], angular[0], angular[1], angular[2])


def twist_world_to_base(twist_world: Pose6, robot_base_pose_world: FrameTransform | Pose6) -> Pose6:
    frame = _as_frame_transform(robot_base_pose_world)
    linear = frame.inverse_rotation @ np.array([twist_world.x, twist_world.y, twist_world.z], dtype=float)
    angular = frame.inverse_rotation @ np.array([twist_world.a, twist_world.b, twist_world.c], dtype=float)
    return Pose6(linear[0], linear[1], linear[2], angular[0], angular[1], angular[2])


def transform_matrix_base_to_world(transform: np.ndarray, robot_base_pose_world: FrameTransform | Pose6) -> np.ndarray:
    frame = _as_frame_transform(robot_base_pose_world)
    return frame.matrix @ np.array(transform, dtype=float)


def transform_points_base_to_world(points_xyz: np.ndarray, robot_base_pose_world: FrameTransform | Pose6) -> np.ndarray:
    points = np.array(points_xyz, dtype=float)
    if points.size == 0:
        return points
    frame = _as_frame_transform(robot_base_pose_world)
    return (points @ frame.rotation.T) + frame.translation


def convert_pose_to_base_frame(
    pose: Pose6,
    reference_frame: ReferenceFrame,
    robot_base_pose_world: FrameTransform | Pose6,
) -> Pose6:
    if reference_frame == ReferenceFrame.WORLD:
        return pose_world_to_base(pose, robot_base_pose_world)
    return pose.copy()


def convert_pose_from_base_frame(
    pose_base: Pose6,
    reference_frame: ReferenceFrame,
    robot_base_pose_world: FrameTransform | Pose6,
) -> Pose6:
    if reference_frame == ReferenceFrame.WORLD:
        return pose_base_to_world(pose_base, robot_base_pose_world)
    return pose_base.copy()
