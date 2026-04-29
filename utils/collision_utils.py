from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

import utils.math_utils as math_utils
from models.pose6 import Pose6
from models.primitive_collider_models import (
    PrimitiveColliderData,
    RobotAxisColliderData,
    parse_primitive_collider_data,
    parse_robot_axis_colliders,
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


@dataclass(frozen=True)
class CollisionShapeTemplate:
    owner: str
    name: str
    shape: str
    local_transform: np.ndarray
    size_x: float = 0.0
    size_y: float = 0.0
    size_z: float = 0.0
    radius: float = 0.0
    height: float = 0.0
    source_index: int | None = None
    metadata: dict[str, Any] = field(default_factory=dict)
    attached_frame_index: int | None = None

    def __post_init__(self) -> None:
        local_transform = np.array(self.local_transform, dtype=float)
        if local_transform.shape != (4, 4):
            raise ValueError("local_transform must be a 4x4 matrix")
        local_transform.setflags(write=False)
        object.__setattr__(self, "local_transform", local_transform)
        object.__setattr__(self, "shape", self.shape if self.shape in SUPPORTED_SHAPES else "box")
        object.__setattr__(self, "size_x", max(0.0, float(self.size_x)))
        object.__setattr__(self, "size_y", max(0.0, float(self.size_y)))
        object.__setattr__(self, "size_z", max(0.0, float(self.size_z)))
        object.__setattr__(self, "radius", max(0.0, float(self.radius)))
        object.__setattr__(self, "height", max(0.0, float(self.height)))

    def build_world_shape(self, frame_world_transform: np.ndarray) -> CollisionShape:
        return CollisionShape(
            owner=self.owner,
            name=self.name,
            shape=self.shape,
            world_transform=np.array(frame_world_transform, dtype=float) @ self.local_transform,
            size_x=self.size_x,
            size_y=self.size_y,
            size_z=self.size_z,
            radius=self.radius,
            height=self.height,
            source_index=self.source_index,
            metadata=dict(self.metadata),
        )


class CollisionWorldCache:
    def __init__(self) -> None:
        self.workspace_shapes_world: list[CollisionShape] = []
        self.robot_shape_templates: list[CollisionShapeTemplate] = []
        self.tool_shape_templates: list[CollisionShapeTemplate] = []
        self.robot_shapes_world: list[CollisionShape] = []
        self.tool_shapes_world: list[CollisionShape] = []

    def set_workspace_collision_zones(self, zones: list[PrimitiveColliderData] | list[dict[str, Any]]) -> None:
        self.workspace_shapes_world = build_workspace_collision_shapes(zones)

    def set_robot_axis_templates(
        self,
        axis_colliders: list[RobotAxisColliderData] | list[dict[str, Any]],
    ) -> None:
        self.robot_shape_templates = build_robot_axis_collision_shape_templates(axis_colliders)
        self.robot_shapes_world = []

    def set_tool_templates(self, tool_colliders: list[PrimitiveColliderData] | list[dict[str, Any]]) -> None:
        self.tool_shape_templates = build_tool_collision_shape_templates(tool_colliders)
        self.tool_shapes_world = []

    def update_robot_world_shapes(self, frame_world_transforms: list[np.ndarray]) -> None:
        self.robot_shapes_world = instantiate_collision_shapes_from_templates(
            self.robot_shape_templates,
            frame_world_transforms=frame_world_transforms,
        )

    def update_tool_world_shapes(self, flange_world_transform: np.ndarray | None) -> None:
        if flange_world_transform is None:
            self.tool_shapes_world = []
            return
        self.tool_shapes_world = instantiate_collision_shapes_from_templates(
            self.tool_shape_templates,
            default_frame_world=flange_world_transform,
        )

    def update_dynamic_world_shapes(
        self,
        frame_world_transforms: list[np.ndarray],
        flange_world_transform: np.ndarray | None,
    ) -> None:
        self.update_robot_world_shapes(frame_world_transforms)
        self.update_tool_world_shapes(flange_world_transform)

    def update_robot_axis_colliders(
        self,
        axis_colliders: list[RobotAxisColliderData] | list[dict[str, Any]],
        corrected_matrices: list[np.ndarray],
        robot_base_world: object | None = None,
    ) -> None:
        self.set_robot_axis_templates(axis_colliders)
        frame_world_transforms = build_world_frame_transforms(corrected_matrices, robot_base_world)
        self.update_robot_world_shapes(frame_world_transforms)

    def update_tool_colliders(
        self,
        tool_colliders: list[PrimitiveColliderData] | list[dict[str, Any]],
        corrected_matrices: list[np.ndarray],
        robot_base_world: object | None = None,
    ) -> None:
        self.set_tool_templates(tool_colliders)
        frame_world_transforms = build_world_frame_transforms(corrected_matrices, robot_base_world)
        self.update_tool_world_shapes(resolve_flange_world_transform(frame_world_transforms))

    def find_workspace_collisions(self) -> list[CollisionPair]:
        moving_shapes = [*self.robot_shapes_world, *self.tool_shapes_world]
        return find_collisions(moving_shapes, self.workspace_shapes_world)

    def find_robot_tool_collisions(
        self,
        evaluated_robot_axis_colliders: list[bool] | None = None,
    ) -> list[CollisionPair]:
        robot_shapes = filter_robot_shapes_by_evaluated_axes(
            self.robot_shapes_world,
            evaluated_robot_axis_colliders,
        )
        return find_collisions(robot_shapes, self.tool_shapes_world)

    @staticmethod
    def find_collisions(
        shapes_a: list[CollisionShape],
        shapes_b: list[CollisionShape],
    ) -> list[CollisionPair]:
        return find_collisions(shapes_a, shapes_b)


def build_workspace_collision_shapes(
    zones: list[PrimitiveColliderData] | list[dict[str, Any]],
) -> list[CollisionShape]:
    shapes: list[CollisionShape] = []
    for index, zone in enumerate(
        parse_primitive_collider_data(zones, default_shape="box", default_name_prefix="Workspace zone")
    ):
        if not _is_enabled_valid_primitive(zone):
            continue
        shapes.append(_primitive_to_shape(zone, "workspace", zone.name, index))
    return shapes


def build_robot_axis_collision_shape_templates(
    axis_colliders: list[RobotAxisColliderData] | list[dict[str, Any]],
) -> list[CollisionShapeTemplate]:
    if not axis_colliders:
        return []

    templates: list[CollisionShapeTemplate] = []
    for axis_index, collider in enumerate(parse_robot_axis_colliders(axis_colliders, 6)[:6]):
        if not collider.enabled:
            continue

        radius = max(0.0, float(collider.radius))
        signed_height = float(collider.height)
        height = abs(signed_height)
        if radius <= EPSILON or height <= EPSILON:
            continue

        translation = np.eye(4, dtype=float)
        translation[:3, 3] = np.array(collider.offset_xyz, dtype=float)
        orientation = primitive_extrusion_orientation(
            collider.direction_axis,
            signed_height >= 0.0,
        )
        templates.append(
            CollisionShapeTemplate(
                owner="robot",
                name=f"Robot collider J{axis_index + 1}",
                shape="cylinder",
                local_transform=translation @ orientation,
                radius=radius,
                height=height,
                source_index=axis_index,
                metadata={"axis": axis_index, "direction_axis": collider.direction_axis},
                attached_frame_index=axis_index + 1,
            )
        )
    return templates


def build_tool_collision_shape_templates(
    tool_colliders: list[PrimitiveColliderData] | list[dict[str, Any]],
) -> list[CollisionShapeTemplate]:
    templates: list[CollisionShapeTemplate] = []
    for index, collider in enumerate(
        parse_primitive_collider_data(
            tool_colliders,
            default_shape="cylinder",
            default_name_prefix="Tool collider",
        )
    ):
        if not _is_enabled_valid_primitive(collider):
            continue
        templates.append(
            CollisionShapeTemplate(
                owner="tool",
                name=collider.name,
                shape=collider.shape,
                local_transform=collider.build_local_transform(),
                size_x=collider.size_x,
                size_y=collider.size_y,
                size_z=collider.size_z,
                radius=collider.radius,
                height=collider.height,
                source_index=index,
            )
        )
    return templates


def instantiate_collision_shapes_from_templates(
    templates: list[CollisionShapeTemplate],
    frame_world_transforms: list[np.ndarray] | None = None,
    default_frame_world: np.ndarray | None = None,
) -> list[CollisionShape]:
    normalized_frames = [] if frame_world_transforms is None else _normalize_matrices(frame_world_transforms)
    default_frame = np.eye(4, dtype=float) if default_frame_world is None else np.array(default_frame_world, dtype=float)

    shapes: list[CollisionShape] = []
    for template in templates:
        if template.attached_frame_index is None:
            frame_world = default_frame
        else:
            if template.attached_frame_index < 0 or template.attached_frame_index >= len(normalized_frames):
                continue
            frame_world = normalized_frames[template.attached_frame_index]
        shapes.append(template.build_world_shape(frame_world))
    return shapes


def build_world_frame_transforms(
    corrected_matrices: list[np.ndarray],
    robot_base_world: object | None = None,
) -> list[np.ndarray]:
    base_world = _as_transform_matrix(robot_base_world)
    matrices = _normalize_matrices(corrected_matrices)
    return [base_world @ matrix for matrix in matrices]


def resolve_flange_world_transform(frame_world_transforms: list[np.ndarray]) -> np.ndarray | None:
    matrices = _normalize_matrices(frame_world_transforms)
    if len(matrices) < 2:
        if not matrices:
            return None
        return matrices[-1]
    return matrices[-2]


def build_robot_axis_collision_shapes(
    axis_colliders: list[RobotAxisColliderData] | list[dict[str, Any]],
    corrected_matrices: list[np.ndarray],
    robot_base_world: object | None = None,
) -> list[CollisionShape]:
    templates = build_robot_axis_collision_shape_templates(axis_colliders)
    frame_world_transforms = build_world_frame_transforms(corrected_matrices, robot_base_world)
    return instantiate_collision_shapes_from_templates(templates, frame_world_transforms=frame_world_transforms)


def build_tool_collision_shapes(
    tool_colliders: list[PrimitiveColliderData] | list[dict[str, Any]],
    corrected_matrices: list[np.ndarray],
    robot_base_world: object | None = None,
) -> list[CollisionShape]:
    templates = build_tool_collision_shape_templates(tool_colliders)
    frame_world_transforms = build_world_frame_transforms(corrected_matrices, robot_base_world)
    flange_world_transform = resolve_flange_world_transform(frame_world_transforms)
    if flange_world_transform is None:
        return []
    return instantiate_collision_shapes_from_templates(templates, default_frame_world=flange_world_transform)


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


def filter_robot_shapes_by_evaluated_axes(
    robot_shapes: list[CollisionShape],
    evaluated_robot_axis_colliders: list[bool] | None = None,
) -> list[CollisionShape]:
    normalized_flags = _normalize_evaluated_robot_axis_colliders(evaluated_robot_axis_colliders)
    filtered: list[CollisionShape] = []
    for shape in robot_shapes:
        axis_index = shape.source_index
        if axis_index is None or not (0 <= axis_index < len(normalized_flags)):
            filtered.append(shape)
            continue
        if normalized_flags[axis_index]:
            filtered.append(shape)
    return filtered


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
    collider: PrimitiveColliderData,
    owner: str,
    name: str,
    source_index: int,
    world_transform: np.ndarray | None = None,
) -> CollisionShape:
    transform = collider.build_local_transform() if world_transform is None else world_transform
    return CollisionShape(
        owner=owner,
        name=str(name),
        shape=collider.shape,
        world_transform=transform,
        size_x=collider.size_x,
        size_y=collider.size_y,
        size_z=collider.size_z,
        radius=collider.radius,
        height=collider.height,
        source_index=source_index,
    )


def _is_enabled_valid_primitive(collider: PrimitiveColliderData) -> bool:
    if not collider.enabled:
        return False
    shape = collider.shape
    if shape == "box":
        return collider.size_x > EPSILON and collider.size_y > EPSILON and collider.size_z > EPSILON
    if shape == "cylinder":
        return collider.radius > EPSILON and collider.height > EPSILON
    if shape == "sphere":
        return collider.radius > EPSILON
    return False


def _as_transform_matrix(value: object | None) -> np.ndarray:
    if value is None:
        return np.eye(4, dtype=float)

    matrix_value = getattr(value, "matrix", value)
    matrix = np.array(matrix_value, dtype=float)
    if matrix.shape == (4, 4):
        return matrix
    if matrix.shape == (6,):
        return math_utils.pose_zyx_to_matrix(Pose6.from_sequence(matrix.tolist(), fill_missing=False))
    raise ValueError("Expected a 4x4 transform matrix, a FrameTransform, or a pose6")


def _normalize_matrices(matrices: list[np.ndarray]) -> list[np.ndarray]:
    out: list[np.ndarray] = []
    for matrix in matrices:
        normalized = np.array(matrix, dtype=float)
        if normalized.shape == (4, 4):
            out.append(normalized)
    return out


def _normalize_evaluated_robot_axis_colliders(values: list[bool] | None) -> list[bool]:
    raw_values = values if isinstance(values, list) else []
    normalized: list[bool] = []
    for axis in range(6):
        normalized.append(bool(raw_values[axis]) if axis < len(raw_values) else True)
    return normalized


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
    "CollisionShapeTemplate",
    "CollisionWorldCache",
    "build_robot_axis_collision_shape_templates",
    "build_robot_axis_collision_shapes",
    "build_tool_collision_shape_templates",
    "build_tool_collision_shapes",
    "build_workspace_collision_shapes",
    "build_world_frame_transforms",
    "filter_robot_shapes_by_evaluated_axes",
    "find_collisions",
    "instantiate_collision_shapes_from_templates",
    "intersects",
    "primitive_extrusion_orientation",
    "resolve_flange_world_transform",
]
