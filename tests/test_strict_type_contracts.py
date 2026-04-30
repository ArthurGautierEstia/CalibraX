import unittest

from models.primitive_collider_models import AxisDirection, PrimitiveColliderData, PrimitiveColliderShape, RobotAxisColliderData
from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.types import Pose6, XYZ3
from models.workspace_model import WorkspaceModel


class StrictTypeContractTests(unittest.TestCase):
    def test_tool_model_rejects_untyped_tool_pose(self):
        model = ToolModel()
        with self.assertRaises(TypeError):
            model.set_tool_pose([0.0] * 6)

    def test_tool_model_rejects_dict_colliders(self):
        model = ToolModel()
        with self.assertRaises(TypeError):
            model.set_tool_colliders([{"shape": "sphere"}])

    def test_workspace_model_rejects_untyped_base_pose(self):
        model = WorkspaceModel()
        with self.assertRaises(TypeError):
            model.set_robot_base_pose_world([0.0] * 6)

    def test_robot_model_rejects_dict_axis_colliders(self):
        model = RobotModel()
        with self.assertRaises(TypeError):
            model.set_axis_colliders([{"axis": 0}])

    def test_collider_models_require_enums_and_value_types(self):
        with self.assertRaises(TypeError):
            PrimitiveColliderData(name="Zone", shape="box", pose=Pose6.zeros())
        with self.assertRaises(TypeError):
            RobotAxisColliderData(axis_index=0, direction_axis="z", offset_xyz=XYZ3.zeros())
        with self.assertRaises(TypeError):
            RobotAxisColliderData(axis_index=0, direction_axis=AxisDirection.Z, offset_xyz=[0.0, 0.0, 0.0])

        PrimitiveColliderData(name="Zone", shape=PrimitiveColliderShape.BOX, pose=Pose6.zeros())
        RobotAxisColliderData(axis_index=0, direction_axis=AxisDirection.Z, offset_xyz=XYZ3.zeros())


if __name__ == "__main__":
    unittest.main()
