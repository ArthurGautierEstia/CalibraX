from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

import utils.math_utils as math_utils
from models.collider_models import (
    normalize_xyz3,
    parse_axis_colliders,
    parse_primitive_colliders,
)


EPSILON = 1e-9
SUPPORTED_SHAPES = {"box", "cylinder", "sphere"}


@dataclass(eq=False)
class CollisionShape:
    owner: str
    name: str
    shape: str
    world_transform: np.ndarray
    size_x: float = 0.0
    size_y: float = 0.0
    size_z: float = 0.0
    radius: float = 0.0
    height: float = 0.0
    source_index: int | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        transform = np.array(self.world_transform, dtype=float)
        if transform.shape != (4, 4):
            raise ValueError("world_transform must be a 4x4 matrix")

        normalized_shape = self.shape if self.shape in SUPPORTED_SHAPES else "box"
        transform.setflags(write=False)
        self.world_transform = transform
        self.shape = normalized_shape
        self.size_x = max(0.0, float(self.size_x))
        self.size_y = max(0.0, float(self.size_y))
        self.size_z = max(0.0, float(self.size_z))
        self.radius = max(0.0, float(self.radius))
        self.height = max(0.0, float(self.height))

    @property
    def rotation(self) -> np.ndarray:
        return self.world_transform[:3, :3]

    @property
    def translation(self) -> np.ndarray:
        return self.world_transform[:3, 3]

    @property
    def center(self) -> np.ndarray:
        return self._local_to_world(self._local_center())

    def support(self, direction_world: np.ndarray) -> np.ndarray:
        direction = np.asarray(direction_world, dtype=float)
        if direction.shape != (3,):
            direction = direction.reshape(3)
        local_direction = self.rotation.T @ direction
        return self._local_to_world(self._local_support(local_direction))

    def _local_to_world(self, point_local: np.ndarray) -> np.ndarray:
        return self.translation + self.rotation @ point_local

    def _local_center(self) -> np.ndarray:
        if self.shape == "box":
            return np.array([0.0, 0.0, self.size_z * 0.5], dtype=float)
        if self.shape == "cylinder":
            return np.array([0.0, 0.0, self.height * 0.5], dtype=float)
        return np.zeros(3, dtype=float)

    def _local_support(self, direction_local: np.ndarray) -> np.ndarray:
        if self.shape == "box":
            return np.array(
                [
                    self.size_x * 0.5 if direction_local[0] >= 0.0 else -self.size_x * 0.5,
                    self.size_y * 0.5 if direction_local[1] >= 0.0 else -self.size_y * 0.5,
                    self.size_z if direction_local[2] >= 0.0 else 0.0,
                ],
                dtype=float,
            )

        if self.shape == "cylinder":
            dxy = direction_local[:2]
            nxy = np.linalg.norm(dxy)
            if nxy > EPSILON:
                xy = dxy / nxy * self.radius
            else:
                xy = np.array([self.radius, 0.0], dtype=float)
            z = self.height if direction_local[2] >= 0.0 else 0.0
            return np.array([xy[0], xy[1], z], dtype=float)

        norm = np.linalg.norm(direction_local)
        if norm <= EPSILON:
            return np.array([self.radius, 0.0, 0.0], dtype=float)
        return direction_local / norm * self.radius


@dataclass(frozen=True)
class CollisionPair:
    shape_a: CollisionShape
    shape_b: CollisionShape

    @property
    def owner_a(self) -> str:
        return self.shape_a.owner

    @property
    def owner_b(self) -> str:
        return self.shape_b.owner

    @property
    def name_a(self) -> str:
        return self.shape_a.name

    @property
    def name_b(self) -> str:
        return self.shape_b.name


class CollisionWorldCache:
    def __init__(self) -> None:
        self.workspace_shapes_world: list[CollisionShape] = []
        self.robot_shapes_world: list[CollisionShape] = []
        self.tool_shapes_world: list[CollisionShape] = []

    def set_workspace_collision_zones(self, zones: list[dict[str, Any]]) -> None:
        self.workspace_shapes_world = build_workspace_collision_shapes(zones)

    def update_robot_axis_colliders(
        self,
        axis_colliders: list[dict[str, Any]],
        corrected_matrices: list[np.ndarray],
        robot_base_world: object | None = None,
    ) -> None:
        self.robot_shapes_world = build_robot_axis_collision_shapes(
            axis_colliders,
            corrected_matrices,
            robot_base_world,
        )

    def update_tool_colliders(
        self,
        tool_colliders: list[dict[str, Any]],
        corrected_matrices: list[np.ndarray],
        robot_base_world: object | None = None,
    ) -> None:
        self.tool_shapes_world = build_tool_collision_shapes(
            tool_colliders,
            corrected_matrices,
            robot_base_world,
        )

    def find_workspace_collisions(self) -> list[CollisionPair]:
        moving_shapes = [*self.robot_shapes_world, *self.tool_shapes_world]
        return find_collisions(moving_shapes, self.workspace_shapes_world)

    @staticmethod
    def find_collisions(
        shapes_a: list[CollisionShape],
        shapes_b: list[CollisionShape],
    ) -> list[CollisionPair]:
        return find_collisions(shapes_a, shapes_b)


def build_workspace_collision_shapes(zones: list[dict[str, Any]]) -> list[CollisionShape]:
    shapes: list[CollisionShape] = []
    for index, zone in enumerate(parse_primitive_colliders(zones, default_shape="box")):
        if not _is_enabled_valid_primitive(zone):
            continue
        shapes.append(_primitive_to_shape(zone, "workspace", zone.get("name", f"Workspace zone {index + 1}"), index))
    return shapes


def build_robot_axis_collision_shapes(
    axis_colliders: list[dict[str, Any]],
    corrected_matrices: list[np.ndarray],
    robot_base_world: object | None = None,
) -> list[CollisionShape]:
    if not axis_colliders:
        return []

    matrices = _normalize_matrices(corrected_matrices)
    if not matrices:
        return []

    base_world = _as_transform_matrix(robot_base_world)
    shapes: list[CollisionShape] = []
    for axis_index, collider in enumerate(parse_axis_colliders(axis_colliders, 6)[:6]):
        if not bool(collider.get("enabled", True)):
            continue

        radius = max(0.0, float(collider.get("radius", 40.0)))
        signed_height = float(collider.get("height", 200.0))
        height = abs(signed_height)
        if radius <= EPSILON or height <= EPSILON:
            continue

        matrix_index = axis_index + 1
        if matrix_index >= len(matrices):
            continue

        translation = np.eye(4, dtype=float)
        translation[:3, 3] = normalize_xyz3(collider.get("offset_xyz"))
        orientation = primitive_extrusion_orientation(
            str(collider.get("direction_axis", "z")).strip().lower(),
            signed_height >= 0.0,
        )
        world_transform = base_world @ matrices[matrix_index] @ translation @ orientation
        shapes.append(
            CollisionShape(
                owner="robot",
                name=f"Robot collider J{axis_index + 1}",
                shape="cylinder",
                world_transform=world_transform,
                radius=radius,
                height=height,
                source_index=axis_index,
                metadata={"axis": axis_index, "direction_axis": collider.get("direction_axis", "z")},
            )
        )
    return shapes


def build_tool_collision_shapes(
    tool_colliders: list[dict[str, Any]],
    corrected_matrices: list[np.ndarray],
    robot_base_world: object | None = None,
) -> list[CollisionShape]:
    matrices = _normalize_matrices(corrected_matrices)
    if len(matrices) < 2:
        return []

    base_world = _as_transform_matrix(robot_base_world)
    flange_index = len(matrices) - 2
    flange_world = base_world @ matrices[flange_index]

    shapes: list[CollisionShape] = []
    for index, collider in enumerate(parse_primitive_colliders(tool_colliders, default_shape="cylinder")):
        if not _is_enabled_valid_primitive(collider):
            continue
        local_transform = math_utils.pose_zyx_to_matrix(collider.get("pose", [0.0] * 6))
        name = str(collider.get("name", f"Tool collider {index + 1}"))
        shapes.append(_primitive_to_shape(collider, "tool", name, index, flange_world @ local_transform))
    return shapes


def find_collisions(
    shapes_a: list[CollisionShape],
    shapes_b: list[CollisionShape],
) -> list[CollisionPair]:
    pairs: list[CollisionPair] = []
    for shape_a in shapes_a:
        for shape_b in shapes_b:
            if shape_a is shape_b:
                continue
            if intersects(shape_a, shape_b):
                pairs.append(CollisionPair(shape_a, shape_b))
    return pairs


def intersects(shape_a: CollisionShape, shape_b: CollisionShape, max_iters: int = 64) -> bool:
    return _gjk(shape_a, shape_b, max_iters=max_iters)


def primitive_extrusion_orientation(direction_axis: str, positive_direction: bool = True) -> np.ndarray:
    rotation = np.eye(4, dtype=float)
    normalized_axis = direction_axis if direction_axis in {"x", "y", "z"} else "z"
    if normalized_axis == "x":
        rotation[:3, :3] = math_utils.rot_y(90.0, degrees=True)
    elif normalized_axis == "y":
        rotation[:3, :3] = math_utils.rot_x(-90.0, degrees=True)

    if positive_direction:
        return rotation

    flip = np.eye(4, dtype=float)
    flip[:3, :3] = math_utils.rot_x(180.0, degrees=True)
    return rotation @ flip


def _primitive_to_shape(
    collider: dict[str, Any],
    owner: str,
    name: str,
    source_index: int,
    world_transform: np.ndarray | None = None,
) -> CollisionShape:
    transform = math_utils.pose_zyx_to_matrix(collider.get("pose", [0.0] * 6)) if world_transform is None else world_transform
    return CollisionShape(
        owner=owner,
        name=str(name),
        shape=str(collider.get("shape", "box")).strip().lower(),
        world_transform=transform,
        size_x=float(collider.get("size_x", 0.0)),
        size_y=float(collider.get("size_y", 0.0)),
        size_z=float(collider.get("size_z", 0.0)),
        radius=float(collider.get("radius", 0.0)),
        height=float(collider.get("height", 0.0)),
        source_index=source_index,
    )


def _is_enabled_valid_primitive(collider: dict[str, Any]) -> bool:
    if not bool(collider.get("enabled", True)):
        return False
    shape = str(collider.get("shape", "box")).strip().lower()
    if shape == "box":
        return (
            float(collider.get("size_x", 0.0)) > EPSILON
            and float(collider.get("size_y", 0.0)) > EPSILON
            and float(collider.get("size_z", 0.0)) > EPSILON
        )
    if shape == "cylinder":
        return float(collider.get("radius", 0.0)) > EPSILON and float(collider.get("height", 0.0)) > EPSILON
    if shape == "sphere":
        return float(collider.get("radius", 0.0)) > EPSILON
    return False


def _as_transform_matrix(value: object | None) -> np.ndarray:
    if value is None:
        return np.eye(4, dtype=float)

    matrix_value = getattr(value, "matrix", value)
    matrix = np.array(matrix_value, dtype=float)
    if matrix.shape == (4, 4):
        return matrix
    if matrix.shape == (6,):
        return math_utils.pose_zyx_to_matrix(matrix.tolist())
    raise ValueError("Expected a 4x4 transform matrix, a FrameTransform, or a pose6")


def _normalize_matrices(matrices: list[np.ndarray]) -> list[np.ndarray]:
    out: list[np.ndarray] = []
    for matrix in matrices:
        normalized = np.array(matrix, dtype=float)
        if normalized.shape == (4, 4):
            out.append(normalized)
    return out


def _cso_support(shape_a: CollisionShape, shape_b: CollisionShape, direction: np.ndarray) -> np.ndarray:
    return shape_a.support(direction) - shape_b.support(-direction)


def _gjk(shape_a: CollisionShape, shape_b: CollisionShape, max_iters: int = 64) -> bool:
    direction = shape_a.center - shape_b.center
    if np.linalg.norm(direction) <= EPSILON:
        direction = np.array([1.0, 0.0, 0.0], dtype=float)

    point = _cso_support(shape_a, shape_b, direction)
    if np.dot(point, direction) < -EPSILON:
        return False

    simplex = [point]
    direction = -point

    for _ in range(max_iters):
        if np.linalg.norm(direction) <= EPSILON:
            return True

        point = _cso_support(shape_a, shape_b, direction)
        if np.dot(point, direction) < -EPSILON:
            return False

        simplex.append(point)
        simplex, direction = _nearest_simplex(simplex)
        if direction is None:
            return True

    return True


def _nearest_simplex(points: list[np.ndarray]) -> tuple[list[np.ndarray], np.ndarray | None]:
    if len(points) == 2:
        return _line(points)
    if len(points) == 3:
        return _triangle(points)
    return _tetrahedron(points)


def _line(points: list[np.ndarray]) -> tuple[list[np.ndarray], np.ndarray]:
    b, a = points[0], points[1]
    ab = b - a
    ao = -a
    if np.dot(ab, ao) > 0.0:
        direction = np.cross(np.cross(ab, ao), ab)
        if np.linalg.norm(direction) <= EPSILON:
            direction = _any_perpendicular(ab)
        return [b, a], direction
    return [a], ao


def _triangle(points: list[np.ndarray]) -> tuple[list[np.ndarray], np.ndarray]:
    c, b, a = points[0], points[1], points[2]
    ab = b - a
    ac = c - a
    ao = -a
    abc = np.cross(ab, ac)

    if np.linalg.norm(abc) <= EPSILON:
        return _line([b, a])

    if np.dot(np.cross(abc, ac), ao) > 0.0:
        if np.dot(ac, ao) > 0.0:
            direction = np.cross(np.cross(ac, ao), ac)
            if np.linalg.norm(direction) <= EPSILON:
                direction = _any_perpendicular(ac)
            return [c, a], direction
        return _line([b, a])

    if np.dot(np.cross(ab, abc), ao) > 0.0:
        return _line([b, a])

    if np.dot(abc, ao) > 0.0:
        return [c, b, a], abc
    return [b, c, a], -abc


def _tetrahedron(points: list[np.ndarray]) -> tuple[list[np.ndarray], np.ndarray | None]:
    d, c, b, a = points[0], points[1], points[2], points[3]
    ab = b - a
    ac = c - a
    ad = d - a
    ao = -a

    abc = np.cross(ab, ac)
    acd = np.cross(ac, ad)
    adb = np.cross(ad, ab)

    if np.dot(abc, ad) > 0.0:
        abc = -abc
    if np.dot(acd, ab) > 0.0:
        acd = -acd
    if np.dot(adb, ac) > 0.0:
        adb = -adb

    if np.dot(abc, ao) > 0.0:
        return _triangle([c, b, a])
    if np.dot(acd, ao) > 0.0:
        return _triangle([d, c, a])
    if np.dot(adb, ao) > 0.0:
        return _triangle([b, d, a])
    return points, None


def _any_perpendicular(vector: np.ndarray) -> np.ndarray:
    if np.linalg.norm(vector) <= EPSILON:
        return np.array([1.0, 0.0, 0.0], dtype=float)
    axis = np.array([1.0, 0.0, 0.0], dtype=float)
    if abs(float(np.dot(vector / np.linalg.norm(vector), axis))) > 0.9:
        axis = np.array([0.0, 1.0, 0.0], dtype=float)
    return np.cross(vector, axis)


__all__ = [
    "CollisionPair",
    "CollisionShape",
    "CollisionWorldCache",
    "build_robot_axis_collision_shapes",
    "build_tool_collision_shapes",
    "build_workspace_collision_shapes",
    "find_collisions",
    "intersects",
    "primitive_extrusion_orientation",
]
