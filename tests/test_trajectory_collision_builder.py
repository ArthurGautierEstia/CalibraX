import unittest

from models.robot_model import RobotModel
from models.primitive_collider_models import PrimitiveColliderData, PrimitiveColliderShape
from models.tool_model import ToolModel
from models.trajectory_keypoint import KeypointMotionMode, KeypointTargetType, TrajectoryKeypoint
from models.trajectory_result import (
    TrajectoryCollisionDomain,
    TrajectoryComputationStatus,
    TrajectorySampleErrorCode,
)
from models.workspace_model import WorkspaceModel
from utils.trajectory_builder import TrajectoryBuilder


def _primitive(shape: PrimitiveColliderShape, radius: float = 50.0) -> PrimitiveColliderData:
    return PrimitiveColliderData(
        name="Collider",
        enabled=True,
        shape=shape,
        size_x=100.0,
        size_y=100.0,
        size_z=100.0,
        radius=radius,
        height=100.0,
    )


def _joint_keypoint(joints: list[float]) -> TrajectoryKeypoint:
    return TrajectoryKeypoint(
        target_type=KeypointTargetType.JOINT,
        joint_target=joints,
        mode=KeypointMotionMode.PTP,
    )


class TrajectoryCollisionBuilderTests(unittest.TestCase):
    def _build_builder(self) -> tuple[RobotModel, ToolModel, WorkspaceModel, TrajectoryBuilder]:
        robot_model = RobotModel()
        tool_model = ToolModel()
        workspace_model = WorkspaceModel()
        builder = TrajectoryBuilder(robot_model, tool_model, workspace_model)
        return robot_model, tool_model, workspace_model, builder

    def test_sample_without_collision_stays_valid(self):
        robot_model, _tool_model, _workspace_model, builder = self._build_builder()

        result = builder.compute_first_segment([0.0] * 6, _joint_keypoint([0.0] * 6), 0.0)

        self.assertEqual(TrajectoryComputationStatus.SUCCESS, result.status)
        self.assertEqual(1, len(result.samples))
        self.assertEqual(TrajectorySampleErrorCode.NONE, result.samples[0].error_code)
        self.assertEqual([], result.samples[0].collisions)

    def test_workspace_collision_sets_sample_and_segment_status(self):
        robot_model, _tool_model, workspace_model, builder = self._build_builder()
        workspace_model.set_workspace_collision_zones([_primitive(PrimitiveColliderShape.BOX)])

        result = builder.compute_first_segment([0.0] * 6, _joint_keypoint([0.0] * 6), 0.0)

        self.assertEqual(TrajectoryComputationStatus.COLLISION_DETECTED, result.status)
        self.assertEqual(TrajectorySampleErrorCode.COLLISION_DETECTED, result.samples[0].error_code)
        self.assertTrue(
            any(collision.domain == TrajectoryCollisionDomain.WORKSPACE for collision in result.samples[0].collisions)
        )

    def test_robot_tool_collision_sets_sample_and_segment_status(self):
        robot_model, tool_model, _workspace_model, builder = self._build_builder()
        tool_model.set_tool_colliders([_primitive(PrimitiveColliderShape.SPHERE, radius=60.0)])

        result = builder.compute_first_segment([0.0] * 6, _joint_keypoint([0.0] * 6), 0.0)

        self.assertEqual(TrajectoryComputationStatus.COLLISION_DETECTED, result.status)
        self.assertEqual(TrajectorySampleErrorCode.COLLISION_DETECTED, result.samples[0].error_code)
        self.assertTrue(
            any(collision.domain == TrajectoryCollisionDomain.ROBOT_TOOL for collision in result.samples[0].collisions)
        )

    def test_workspace_and_robot_tool_collisions_are_both_reported(self):
        robot_model, tool_model, workspace_model, builder = self._build_builder()
        workspace_model.set_workspace_collision_zones([_primitive(PrimitiveColliderShape.BOX)])
        tool_model.set_tool_colliders([_primitive(PrimitiveColliderShape.SPHERE, radius=60.0)])

        result = builder.compute_first_segment([0.0] * 6, _joint_keypoint([0.0] * 6), 0.0)

        domains = {collision.domain for collision in result.samples[0].collisions}
        self.assertEqual(
            {TrajectoryCollisionDomain.WORKSPACE, TrajectoryCollisionDomain.ROBOT_TOOL},
            domains,
        )


if __name__ == "__main__":
    unittest.main()
