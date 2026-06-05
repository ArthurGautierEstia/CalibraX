import unittest

import numpy as np

from models.collision_scene_model import CollisionSceneModel
from models.primitive_collider_models import PrimitiveColliderData, RobotAxisColliderData
from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.types import Pose6, XYZ3
from models.workspace_model import WorkspaceModel


class CollisionSceneModelTest(unittest.TestCase):
    def test_robot_colliders_stay_in_robot_base_frame(self):
        robot_model = RobotModel()
        tool_model = ToolModel()
        workspace_model = WorkspaceModel()
        workspace_model.set_robot_base_pose_world(Pose6(100.0, 200.0, 300.0, 0.0, 0.0, 90.0), emit=False)

        robot_model.set_axis_colliders(
            [
                RobotAxisColliderData(
                    axis_index=axis,
                    enabled=axis == 0,
                    radius=10.0,
                    height=50.0,
                    offset_xyz=XYZ3(1.0, 2.0, 3.0),
                )
                for axis in range(6)
            ]
        )

        scene_model = CollisionSceneModel(robot_model, tool_model, workspace_model)
        corrected_matrices = [np.eye(4, dtype=float) for _ in range(8)]
        corrected_matrices[1][:3, 3] = [10.0, 20.0, 30.0]

        colliders = scene_model._build_robot_base_colliders(corrected_matrices)

        self.assertEqual(len(colliders), 6)
        np.testing.assert_allclose(colliders[0].base_transform, corrected_matrices[1])
        np.testing.assert_allclose(colliders[0].world_transform[:3, 3], [11.0, 22.0, 33.0])

    def test_tool_colliders_are_built_in_world_frame(self):
        robot_model = RobotModel()
        tool_model = ToolModel()
        workspace_model = WorkspaceModel()
        workspace_model.set_robot_base_pose_world(Pose6(100.0, 200.0, 300.0, 0.0, 0.0, 0.0), emit=False)
        scene_model = CollisionSceneModel(robot_model, tool_model, workspace_model)

        corrected_matrices = [np.eye(4, dtype=float) for _ in range(8)]
        corrected_matrices[6][:3, 3] = [10.0, 20.0, 30.0]
        scene_model._tool_collider_data = [
            PrimitiveColliderData("Tool", pose=Pose6(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)),
        ]

        colliders = scene_model._build_tool_world_colliders(
            corrected_matrices,
            workspace_model.get_robot_base_transform_world().matrix,
        )

        self.assertEqual(len(colliders), 1)
        np.testing.assert_allclose(colliders[0].base_transform[:3, 3], [110.0, 220.0, 330.0])
        np.testing.assert_allclose(colliders[0].world_transform[:3, 3], [111.0, 222.0, 333.0])


if __name__ == "__main__":
    unittest.main()
