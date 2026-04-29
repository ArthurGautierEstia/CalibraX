import json
import unittest
from pathlib import Path

import numpy as np

import utils.math_utils as math_utils
from models.pose6 import Pose6
from models.primitive_collider_models import parse_robot_axis_colliders
from utils.collision_utils import (
    CollisionShape,
    CollisionWorldCache,
    build_tool_collision_shapes,
    filter_robot_shapes_by_evaluated_axes,
    intersects,
)


def _shape(
    shape: str,
    pose=None,
    size_x=0.0,
    size_y=0.0,
    size_z=0.0,
    radius=0.0,
    height=0.0,
) -> CollisionShape:
    pose_value = Pose6.zeros() if pose is None else Pose6.from_sequence(pose, fill_missing=False)
    return CollisionShape(
        owner="test",
        name=shape,
        shape=shape,
        world_transform=math_utils.pose_zyx_to_matrix(pose_value),
        size_x=size_x,
        size_y=size_y,
        size_z=size_z,
        radius=radius,
        height=height,
    )


class CollisionUtilsGjkTests(unittest.TestCase):
    def test_box_box_touch_overlap_and_separation(self):
        box_a = _shape("box", size_x=2.0, size_y=2.0, size_z=2.0)
        self.assertTrue(intersects(box_a, _shape("box", pose=[1.0, 0, 0, 0, 0, 0], size_x=2, size_y=2, size_z=2)))
        self.assertTrue(intersects(box_a, _shape("box", pose=[2.0, 0, 0, 0, 0, 0], size_x=2, size_y=2, size_z=2)))
        self.assertFalse(intersects(box_a, _shape("box", pose=[2.01, 0, 0, 0, 0, 0], size_x=2, size_y=2, size_z=2)))

    def test_sphere_sphere_touch_overlap_and_separation(self):
        sphere_a = _shape("sphere", radius=1.0)
        self.assertTrue(intersects(sphere_a, _shape("sphere", pose=[1.5, 0, 0, 0, 0, 0], radius=1.0)))
        self.assertTrue(intersects(sphere_a, _shape("sphere", pose=[2.0, 0, 0, 0, 0, 0], radius=1.0)))
        self.assertFalse(intersects(sphere_a, _shape("sphere", pose=[2.01, 0, 0, 0, 0, 0], radius=1.0)))

    def test_cylinder_box_and_sphere_with_rotation(self):
        cylinder = _shape("cylinder", radius=1.0, height=2.0)
        self.assertTrue(intersects(cylinder, _shape("box", pose=[1.5, 0, 0.5, 0, 0, 0], size_x=1, size_y=1, size_z=1)))
        self.assertFalse(intersects(cylinder, _shape("box", pose=[2.51, 0, 0.5, 0, 0, 0], size_x=1, size_y=1, size_z=1)))

        horizontal_cylinder = _shape("cylinder", pose=[0, 0, 0, 0, 90, 0], radius=0.5, height=2.0)
        self.assertTrue(intersects(horizontal_cylinder, _shape("sphere", pose=[2.4, 0, 0, 0, 0, 0], radius=0.5)))
        self.assertFalse(intersects(horizontal_cylinder, _shape("sphere", pose=[2.6, 0, 0, 0, 0, 0], radius=0.5)))


class CollisionUtilsGeometryTests(unittest.TestCase):
    def test_box_uses_calibrax_z_from_zero_to_size_z(self):
        box = _shape("box", size_x=10.0, size_y=10.0, size_z=900.0)
        np.testing.assert_allclose(box.support(np.array([0.0, 0.0, -1.0]))[2], 0.0)
        np.testing.assert_allclose(box.support(np.array([0.0, 0.0, 1.0]))[2], 900.0)

    def test_cylinder_uses_calibrax_z_from_zero_to_height(self):
        cylinder = _shape("cylinder", radius=10.0, height=120.0)
        np.testing.assert_allclose(cylinder.support(np.array([0.0, 0.0, -1.0]))[2], 0.0)
        np.testing.assert_allclose(cylinder.support(np.array([0.0, 0.0, 1.0]))[2], 120.0)

    def test_full_zyx_transform_is_used_by_support_function(self):
        box = _shape("box", pose=[0, 0, 0, 90, 0, 0], size_x=2.0, size_y=4.0, size_z=1.0)
        support = box.support(np.array([1.0, 0.0, 0.0]))
        np.testing.assert_allclose(support[0], 2.0, atol=1e-9)


class CollisionWorldCacheTests(unittest.TestCase):
    def test_robot_axis_collider_parsing_accepts_dict_inputs(self):
        parsed = parse_robot_axis_colliders(
            [
                {
                    "enabled": True,
                    "radius": 12.0,
                    "height": 34.0,
                    "direction_axis": "x",
                    "offset_xyz": [1.0, 2.0, 3.0],
                }
            ],
            axis_count=1,
        )

        self.assertEqual(1, len(parsed))
        self.assertEqual(0, parsed[0].axis_index)
        self.assertEqual(12.0, parsed[0].radius)
        self.assertEqual(34.0, parsed[0].height)
        self.assertEqual("x", parsed[0].direction_axis)
        self.assertEqual((1.0, 2.0, 3.0), parsed[0].offset_xyz)

    def test_workspace_collision_queries_do_not_rebuild_shapes(self):
        cache = CollisionWorldCache()
        cache.set_workspace_collision_zones(
            [
                {
                    "name": "Zone",
                    "enabled": True,
                    "shape": "box",
                    "pose": [0, 0, 0, 0, 0, 0],
                    "size_x": 10,
                    "size_y": 10,
                    "size_z": 10,
                }
            ]
        )
        cache.update_tool_colliders(
            [
                {
                    "name": "Tool",
                    "enabled": True,
                    "shape": "sphere",
                    "pose": [0, 0, 5, 0, 0, 0],
                    "radius": 1,
                }
            ],
            [np.eye(4), np.eye(4)],
        )

        workspace_ids = [id(shape) for shape in cache.workspace_shapes_world]
        tool_ids = [id(shape) for shape in cache.tool_shapes_world]
        workspace_transforms = [shape.world_transform.copy() for shape in cache.workspace_shapes_world]
        tool_transforms = [shape.world_transform.copy() for shape in cache.tool_shapes_world]

        first = cache.find_workspace_collisions()
        second = cache.find_workspace_collisions()

        self.assertEqual(1, len(first))
        self.assertEqual(1, len(second))
        self.assertEqual(workspace_ids, [id(shape) for shape in cache.workspace_shapes_world])
        self.assertEqual(tool_ids, [id(shape) for shape in cache.tool_shapes_world])
        for before, shape in zip(workspace_transforms, cache.workspace_shapes_world):
            np.testing.assert_allclose(before, shape.world_transform)
        for before, shape in zip(tool_transforms, cache.tool_shapes_world):
            np.testing.assert_allclose(before, shape.world_transform)

    def test_robot_update_replaces_only_robot_shapes_and_ignores_disabled_colliders(self):
        cache = CollisionWorldCache()
        cache.set_workspace_collision_zones(
            [{"enabled": True, "shape": "box", "pose": [0] * 6, "size_x": 1, "size_y": 1, "size_z": 1}]
        )
        cache.update_tool_colliders(
            [{"enabled": True, "shape": "sphere", "pose": [0] * 6, "radius": 1}],
            [np.eye(4), np.eye(4)],
        )
        workspace_ids = [id(shape) for shape in cache.workspace_shapes_world]
        tool_ids = [id(shape) for shape in cache.tool_shapes_world]

        axis_colliders = []
        for index in range(6):
            axis_colliders.append(
                {
                    "enabled": index == 0,
                    "radius": 1.0,
                    "height": 2.0,
                    "direction_axis": "z",
                    "offset_xyz": [0.0, 0.0, 0.0],
                }
            )
        matrices = [np.eye(4) for _ in range(8)]
        cache.update_robot_axis_colliders(axis_colliders, matrices)

        self.assertEqual(1, len(cache.robot_shapes_world))
        self.assertEqual(workspace_ids, [id(shape) for shape in cache.workspace_shapes_world])
        self.assertEqual(tool_ids, [id(shape) for shape in cache.tool_shapes_world])

    def test_dynamic_refresh_reuses_compiled_templates_and_workspace_shapes(self):
        cache = CollisionWorldCache()
        cache.set_workspace_collision_zones(
            [{"enabled": True, "shape": "box", "pose": [0] * 6, "size_x": 10, "size_y": 10, "size_z": 10}]
        )
        cache.set_robot_axis_templates(
            [
                {
                    "enabled": True,
                    "radius": 1.0,
                    "height": 2.0,
                    "direction_axis": "z",
                    "offset_xyz": [0.0, 0.0, 0.0],
                }
            ]
        )
        cache.set_tool_templates(
            [{"enabled": True, "shape": "sphere", "pose": [0] * 6, "radius": 1.0}]
        )

        workspace_ids = [id(shape) for shape in cache.workspace_shapes_world]
        robot_template_ids = [id(template) for template in cache.robot_shape_templates]
        tool_template_ids = [id(template) for template in cache.tool_shape_templates]

        frames_a = [np.eye(4) for _ in range(8)]
        cache.update_dynamic_world_shapes(frames_a, np.eye(4))
        first_robot_center = cache.robot_shapes_world[0].center.copy()

        frames_b = [np.eye(4) for _ in range(8)]
        frames_b[1] = math_utils.pose_zyx_to_matrix(Pose6.from_values(5.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        cache.update_dynamic_world_shapes(frames_b, np.eye(4))

        self.assertEqual(workspace_ids, [id(shape) for shape in cache.workspace_shapes_world])
        self.assertEqual(robot_template_ids, [id(template) for template in cache.robot_shape_templates])
        self.assertEqual(tool_template_ids, [id(template) for template in cache.tool_shape_templates])
        self.assertNotAlmostEqual(first_robot_center[0], cache.robot_shapes_world[0].center[0])

    def test_empty_robot_axis_colliders_do_not_create_defaults(self):
        cache = CollisionWorldCache()
        cache.update_robot_axis_colliders([], [np.eye(4) for _ in range(8)])
        self.assertEqual([], cache.robot_shapes_world)

    def test_default_data_can_build_tool_collision_shapes(self):
        tool_data_path = Path("default_data/tools/Torche_Soudure.json")
        if not tool_data_path.exists():
            self.skipTest("Default tool profile is not available")

        tool_data = json.loads(tool_data_path.read_text(encoding="utf-8"))
        matrices = [np.eye(4) for _ in range(8)]
        shapes = build_tool_collision_shapes(tool_data.get("tool_colliders", []), matrices)
        self.assertGreaterEqual(len(shapes), 1)
        for shape in shapes:
            self.assertEqual("tool", shape.owner)
            self.assertEqual((4, 4), shape.world_transform.shape)

    def test_tool_collision_shapes_use_flange_frame(self):
        matrices = [np.eye(4) for _ in range(8)]
        matrices[-2] = math_utils.pose_zyx_to_matrix(Pose6.from_values(10.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        matrices[-1] = math_utils.pose_zyx_to_matrix(Pose6.from_values(20.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        shapes = build_tool_collision_shapes(
            [{"enabled": True, "shape": "sphere", "pose": [0.0] * 6, "radius": 1.0}],
            matrices,
        )

        self.assertEqual(1, len(shapes))
        np.testing.assert_allclose(shapes[0].translation, np.array([10.0, 0.0, 0.0]))

    def test_robot_tool_filter_keeps_only_selected_robot_axes(self):
        cache = CollisionWorldCache()
        cache.robot_shapes_world = [
            CollisionShape(
                owner="robot",
                name="Robot collider J1",
                shape="sphere",
                world_transform=math_utils.pose_zyx_to_matrix(Pose6.zeros()),
                radius=1.0,
                source_index=0,
            ),
            CollisionShape(
                owner="robot",
                name="Robot collider J2",
                shape="sphere",
                world_transform=math_utils.pose_zyx_to_matrix(Pose6.from_values(10.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
                radius=1.0,
                source_index=1,
            ),
        ]
        cache.tool_shapes_world = [
            CollisionShape(
                owner="tool",
                name="Tool collider",
                shape="sphere",
                world_transform=math_utils.pose_zyx_to_matrix(Pose6.zeros()),
                radius=1.0,
                source_index=0,
            )
        ]

        filtered_pairs = cache.find_robot_tool_collisions([False, True, True, True, True, True])
        self.assertEqual([], filtered_pairs)

        unfiltered_pairs = cache.find_robot_tool_collisions([True, False, False, False, False, False])
        self.assertEqual(1, len(unfiltered_pairs))
        self.assertEqual("Robot collider J1", unfiltered_pairs[0].name_a)

    def test_filter_robot_shapes_by_evaluated_axes_keeps_shapes_without_axis_index(self):
        shapes = [
            CollisionShape(
                owner="robot",
                name="No axis",
                shape="sphere",
                world_transform=math_utils.pose_zyx_to_matrix(Pose6.zeros()),
                radius=1.0,
                source_index=None,
            ),
            CollisionShape(
                owner="robot",
                name="Axis 2",
                shape="sphere",
                world_transform=math_utils.pose_zyx_to_matrix(Pose6.zeros()),
                radius=1.0,
                source_index=1,
            ),
        ]

        filtered = filter_robot_shapes_by_evaluated_axes(shapes, [False, False, False, False, False, False])
        self.assertEqual(["No axis"], [shape.name for shape in filtered])


if __name__ == "__main__":
    unittest.main()
