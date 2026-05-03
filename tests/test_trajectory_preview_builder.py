import unittest

from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.trajectory_keypoint import KeypointMotionMode, KeypointTargetType, TrajectoryKeypoint
from models.trajectory_result import TrajectoryComputationStatus, TrajectorySegment
from models.types import JointAngles6, Pose6
from models.workspace_model import WorkspaceModel
from utils.trajectory_preview_builder import TrajectoryPreviewBuilder


def _joint_keypoint(joints: JointAngles6, mode: KeypointMotionMode = KeypointMotionMode.PTP) -> TrajectoryKeypoint:
    return TrajectoryKeypoint(
        target_type=KeypointTargetType.JOINT,
        joint_target=joints.to_list(),
        mode=mode,
    )


def _cartesian_keypoint(pose: Pose6, mode: KeypointMotionMode) -> TrajectoryKeypoint:
    return TrajectoryKeypoint(
        target_type=KeypointTargetType.CARTESIAN,
        cartesian_target=pose,
        mode=mode,
        linear_speed_mps=0.5,
    )


class TrajectoryPreviewBuilderTests(unittest.TestCase):
    def _build_builder(self) -> tuple[RobotModel, ToolModel, WorkspaceModel, TrajectoryPreviewBuilder]:
        robot_model = RobotModel()
        tool_model = ToolModel()
        workspace_model = WorkspaceModel()
        return robot_model, tool_model, workspace_model, TrajectoryPreviewBuilder(
            robot_model,
            tool_model,
            workspace_model,
        )

    def test_ptp_preview_has_pose_and_joints(self):
        _robot_model, _tool_model, _workspace_model, builder = self._build_builder()
        current = JointAngles6.zeros()
        target = _joint_keypoint(JointAngles6.zeros())

        result = builder.compute_first_preview_segment(current, target, 0.0)

        self.assertEqual(TrajectoryComputationStatus.SUCCESS, result.status)
        self.assertTrue(result.samples)
        self.assertIsNotNone(result.samples[-1].joints_deg)
        self.assertIsInstance(result.samples[-1].pose_base, Pose6)
        self.assertGreaterEqual(result.last_time_s, result.duration_s)

    def test_linear_preview_has_cartesian_pose_without_sample_mgi(self):
        _robot_model, _tool_model, _workspace_model, builder = self._build_builder()
        start_pose = Pose6(300.0, 0.0, 500.0, 0.0, 90.0, 0.0)
        end_pose = Pose6(350.0, 25.0, 520.0, 0.0, 90.0, 0.0)
        segment = TrajectorySegment(
            _cartesian_keypoint(start_pose, KeypointMotionMode.LINEAR),
            _cartesian_keypoint(end_pose, KeypointMotionMode.LINEAR),
        )

        result = builder.compute_preview(JointAngles6.zeros(), [segment])

        self.assertEqual(TrajectoryComputationStatus.SUCCESS, result.status)
        self.assertTrue(result.sample_count() > 0)
        self.assertTrue(all(sample.joints_deg is None for sample in result.segments[-1].samples))
        self.assertIsInstance(result.segments[-1].samples[-1].pose_base, Pose6)


if __name__ == "__main__":
    unittest.main()
