import io
import unittest
from contextlib import redirect_stdout

from models.robot_model import RobotModel
from models.primitive_collider_models import PrimitiveColliderData, PrimitiveColliderShape
from models.tool_model import ToolModel
from models.trajectory_keypoint import KeypointMotionMode, KeypointTargetType, TrajectoryKeypoint
from models.trajectory_result import (
    TrajectoryCollisionDiagnostic,
    TrajectoryCollisionDomain,
    TrajectoryComputationStatus,
    SegmentResult,
    TrajectoryResult,
    TrajectorySampleErrorCode,
)
from models.types import Pose6
from models.workspace_model import WorkspaceModel
from controllers.trajectory_controller import TrajectoryController
from utils.trajectory_builder import TrajectoryBuilder
from utils.trajectory_validity_analyzer import (
    TrajectoryValidityAnalyzer,
    TrajectoryValidityAnalysisResult,
    TrajectoryValidityCancelToken,
    TrajectoryValidityContext,
    TrajectoryValiditySampleResult,
    apply_trajectory_validity_result,
    prepare_trajectory_validity_analysis,
)


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


def _remote_tcp_zone(radius: float = 1.0) -> PrimitiveColliderData:
    return PrimitiveColliderData(
        name="Remote TCP zone",
        enabled=True,
        shape=PrimitiveColliderShape.SPHERE,
        pose=Pose6(1_000_000.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        radius=radius,
    )


def _joint_keypoint(joints: list[float]) -> TrajectoryKeypoint:
    return TrajectoryKeypoint(
        target_type=KeypointTargetType.JOINT,
        joint_target=joints,
        mode=KeypointMotionMode.PTP,
    )


def _trajectory_from_segment(status: TrajectoryComputationStatus, segment: SegmentResult) -> TrajectoryResult:
    trajectory = TrajectoryResult()
    trajectory.segments.append(segment)
    if status != TrajectoryComputationStatus.SUCCESS:
        trajectory.status = status
        trajectory.first_error_segment_index = 0
    return trajectory


class TrajectoryValidityBuilderTests(unittest.TestCase):
    def _build_builder(self) -> tuple[RobotModel, ToolModel, WorkspaceModel, TrajectoryBuilder]:
        robot_model = RobotModel()
        tool_model = ToolModel()
        workspace_model = WorkspaceModel()
        builder = TrajectoryBuilder(robot_model, tool_model, workspace_model)
        return robot_model, tool_model, workspace_model, builder

    def _analyze_and_apply(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        trajectory: TrajectoryResult,
    ) -> bool:
        prepared = prepare_trajectory_validity_analysis(trajectory)
        context = TrajectoryValidityContext.from_models(robot_model, tool_model, workspace_model)
        analyzer = TrajectoryValidityAnalyzer(context)
        result = analyzer.analyze_trajectory(1, trajectory, TrajectoryValidityCancelToken())
        return apply_trajectory_validity_result(trajectory, result) or prepared

    def test_sample_without_collision_stays_valid(self):
        robot_model, _tool_model, _workspace_model, builder = self._build_builder()

        result = builder.compute_first_segment([0.0] * 6, _joint_keypoint([0.0] * 6), 0.0)

        self.assertEqual(TrajectoryComputationStatus.SUCCESS, result.status)
        self.assertEqual(1, len(result.samples))
        self.assertEqual(TrajectorySampleErrorCode.NONE, result.samples[0].error_code)
        self.assertEqual([], result.samples[0].collisions)
        self.assertIsNotNone(result.samples[0].kinematics)
        self.assertTrue(bool(result.samples[0].kinematics.corrected_matrices))

    def test_workspace_collision_sets_sample_and_segment_status(self):
        robot_model, tool_model, workspace_model, builder = self._build_builder()
        workspace_model.set_workspace_collision_zones([_primitive(PrimitiveColliderShape.BOX)])

        segment = builder.compute_first_segment([0.0] * 6, _joint_keypoint([0.0] * 6), 0.0)
        trajectory = _trajectory_from_segment(segment.status, segment)

        self.assertEqual(TrajectoryComputationStatus.SUCCESS, trajectory.status)
        self.assertEqual(TrajectorySampleErrorCode.NONE, segment.samples[0].error_code)

        applied = self._analyze_and_apply(robot_model, tool_model, workspace_model, trajectory)

        self.assertTrue(applied)
        self.assertEqual(TrajectoryComputationStatus.COLLISION_DETECTED, trajectory.status)
        self.assertEqual(TrajectoryComputationStatus.COLLISION_DETECTED, segment.status)
        self.assertEqual(TrajectorySampleErrorCode.COLLISION_DETECTED, segment.samples[0].error_code)
        self.assertTrue(
            any(collision.domain == TrajectoryCollisionDomain.WORKSPACE for collision in segment.samples[0].collisions)
        )

    def test_robot_tool_collision_sets_sample_and_segment_status(self):
        robot_model, tool_model, _workspace_model, builder = self._build_builder()
        tool_model.set_tool_colliders([_primitive(PrimitiveColliderShape.SPHERE, radius=60.0)])

        segment = builder.compute_first_segment([0.0] * 6, _joint_keypoint([0.0] * 6), 0.0)
        trajectory = _trajectory_from_segment(segment.status, segment)

        self.assertEqual(TrajectoryComputationStatus.SUCCESS, trajectory.status)
        self.assertEqual(TrajectorySampleErrorCode.NONE, segment.samples[0].error_code)

        applied = self._analyze_and_apply(robot_model, tool_model, _workspace_model, trajectory)

        self.assertTrue(applied)
        self.assertEqual(TrajectoryComputationStatus.COLLISION_DETECTED, trajectory.status)
        self.assertEqual(TrajectoryComputationStatus.COLLISION_DETECTED, segment.status)
        self.assertEqual(TrajectorySampleErrorCode.COLLISION_DETECTED, segment.samples[0].error_code)
        self.assertTrue(
            any(collision.domain == TrajectoryCollisionDomain.ROBOT_TOOL for collision in segment.samples[0].collisions)
        )

    def test_workspace_and_robot_tool_collisions_are_both_reported(self):
        robot_model, tool_model, workspace_model, builder = self._build_builder()
        workspace_model.set_workspace_collision_zones([_primitive(PrimitiveColliderShape.BOX)])
        tool_model.set_tool_colliders([_primitive(PrimitiveColliderShape.SPHERE, radius=60.0)])

        segment = builder.compute_first_segment([0.0] * 6, _joint_keypoint([0.0] * 6), 0.0)
        trajectory = _trajectory_from_segment(segment.status, segment)

        applied = self._analyze_and_apply(robot_model, tool_model, workspace_model, trajectory)

        self.assertTrue(applied)
        domains = {collision.domain for collision in segment.samples[0].collisions}
        self.assertEqual(
            {TrajectoryCollisionDomain.WORKSPACE, TrajectoryCollisionDomain.ROBOT_TOOL},
            domains,
        )

    def test_no_tcp_zone_keeps_sample_valid(self):
        robot_model, tool_model, workspace_model, builder = self._build_builder()
        segment = builder.compute_first_segment([0.0] * 6, _joint_keypoint([0.0] * 6), 0.0)
        trajectory = _trajectory_from_segment(segment.status, segment)

        applied = self._analyze_and_apply(robot_model, tool_model, workspace_model, trajectory)

        self.assertFalse(applied)
        self.assertEqual(TrajectoryComputationStatus.SUCCESS, trajectory.status)
        self.assertEqual(TrajectorySampleErrorCode.NONE, segment.samples[0].error_code)

    def test_tcp_inside_zone_keeps_sample_valid(self):
        robot_model, tool_model, workspace_model, builder = self._build_builder()
        workspace_model.set_workspace_tcp_zones([_primitive(PrimitiveColliderShape.SPHERE, radius=1_000_000.0)])
        segment = builder.compute_first_segment([0.0] * 6, _joint_keypoint([0.0] * 6), 0.0)
        trajectory = _trajectory_from_segment(segment.status, segment)

        applied = self._analyze_and_apply(robot_model, tool_model, workspace_model, trajectory)

        self.assertFalse(applied)
        self.assertEqual(TrajectoryComputationStatus.SUCCESS, trajectory.status)
        self.assertEqual(TrajectorySampleErrorCode.NONE, segment.samples[0].error_code)

    def test_tcp_outside_all_zones_sets_sample_and_segment_status(self):
        robot_model, tool_model, workspace_model, builder = self._build_builder()
        workspace_model.set_workspace_tcp_zones([_remote_tcp_zone()])
        segment = builder.compute_first_segment([0.0] * 6, _joint_keypoint([0.0] * 6), 0.0)
        trajectory = _trajectory_from_segment(segment.status, segment)

        applied = self._analyze_and_apply(robot_model, tool_model, workspace_model, trajectory)

        self.assertTrue(applied)
        self.assertEqual(TrajectoryComputationStatus.TCP_WORKSPACE_EXIT, trajectory.status)
        self.assertEqual(TrajectoryComputationStatus.TCP_WORKSPACE_EXIT, segment.status)
        self.assertEqual(TrajectorySampleErrorCode.TCP_WORKSPACE_EXIT, segment.samples[0].error_code)
        self.assertEqual([], segment.samples[0].collisions)

    def test_disabled_tcp_zone_is_ignored(self):
        robot_model, tool_model, workspace_model, builder = self._build_builder()
        zone = _primitive(PrimitiveColliderShape.SPHERE, radius=1.0)
        zone.enabled = False
        workspace_model.set_workspace_tcp_zones([zone])
        segment = builder.compute_first_segment([0.0] * 6, _joint_keypoint([0.0] * 6), 0.0)
        trajectory = _trajectory_from_segment(segment.status, segment)

        applied = self._analyze_and_apply(robot_model, tool_model, workspace_model, trajectory)

        self.assertFalse(applied)
        self.assertEqual(TrajectoryComputationStatus.SUCCESS, trajectory.status)
        self.assertEqual(TrajectorySampleErrorCode.NONE, segment.samples[0].error_code)

    def test_collision_takes_priority_over_tcp_zone_exit(self):
        robot_model, tool_model, workspace_model, builder = self._build_builder()
        workspace_model.set_workspace_collision_zones([_primitive(PrimitiveColliderShape.BOX)])
        workspace_model.set_workspace_tcp_zones([_remote_tcp_zone()])
        segment = builder.compute_first_segment([0.0] * 6, _joint_keypoint([0.0] * 6), 0.0)
        trajectory = _trajectory_from_segment(segment.status, segment)

        applied = self._analyze_and_apply(robot_model, tool_model, workspace_model, trajectory)

        self.assertTrue(applied)
        self.assertEqual(TrajectoryComputationStatus.COLLISION_DETECTED, trajectory.status)
        self.assertEqual(TrajectorySampleErrorCode.COLLISION_DETECTED, segment.samples[0].error_code)
        self.assertTrue(segment.samples[0].collisions)

    def test_validity_reanalysis_clears_stale_tcp_zone_error(self):
        robot_model, tool_model, workspace_model, builder = self._build_builder()
        workspace_model.set_workspace_tcp_zones([_remote_tcp_zone()])
        segment = builder.compute_first_segment([0.0] * 6, _joint_keypoint([0.0] * 6), 0.0)
        trajectory = _trajectory_from_segment(segment.status, segment)
        self.assertTrue(self._analyze_and_apply(robot_model, tool_model, workspace_model, trajectory))
        self.assertEqual(TrajectorySampleErrorCode.TCP_WORKSPACE_EXIT, segment.samples[0].error_code)

        workspace_model.set_workspace_tcp_zones([])
        applied = self._analyze_and_apply(robot_model, tool_model, workspace_model, trajectory)

        self.assertTrue(applied)
        self.assertEqual(TrajectoryComputationStatus.SUCCESS, trajectory.status)
        self.assertEqual(TrajectoryComputationStatus.SUCCESS, segment.status)
        self.assertEqual(TrajectorySampleErrorCode.NONE, segment.samples[0].error_code)

    def test_cancelled_validity_analysis_does_not_mutate_trajectory(self):
        robot_model, tool_model, workspace_model, builder = self._build_builder()
        workspace_model.set_workspace_collision_zones([_primitive(PrimitiveColliderShape.BOX)])
        segment = builder.compute_first_segment([0.0] * 6, _joint_keypoint([0.0] * 6), 0.0)
        trajectory = _trajectory_from_segment(segment.status, segment)
        context = TrajectoryValidityContext.from_models(robot_model, tool_model, workspace_model)
        analyzer = TrajectoryValidityAnalyzer(context)
        cancel_token = TrajectoryValidityCancelToken()
        cancel_token.request_cancel()

        result = analyzer.analyze_trajectory(1, trajectory, cancel_token)
        applied = apply_trajectory_validity_result(trajectory, result)

        self.assertTrue(result.cancelled)
        self.assertFalse(applied)
        self.assertEqual(TrajectoryComputationStatus.SUCCESS, trajectory.status)
        self.assertEqual(TrajectorySampleErrorCode.NONE, segment.samples[0].error_code)

    def test_stale_validity_worker_result_is_ignored_by_controller(self):
        _robot_model, _tool_model, _workspace_model, builder = self._build_builder()
        segment = builder.compute_first_segment([0.0] * 6, _joint_keypoint([0.0] * 6), 0.0)
        trajectory = _trajectory_from_segment(segment.status, segment)
        collision = TrajectoryCollisionDiagnostic(
            domain=TrajectoryCollisionDomain.WORKSPACE,
            owner_a="robot",
            name_a="Robot collider J1",
            source_index_a=0,
            owner_b="workspace",
            name_b="Zone collision",
            source_index_b=0,
        )
        stale_result = TrajectoryValidityAnalysisResult(
            job_id=1,
            cancelled=False,
            has_error=True,
            sample_results=[
                TrajectoryValiditySampleResult(
                    segment_index=0,
                    sample_index=0,
                    error_code=TrajectorySampleErrorCode.COLLISION_DETECTED,
                    collisions=[collision],
                )
            ],
        )
        controller = TrajectoryController.__new__(TrajectoryController)
        controller._active_collision_job_id = 2
        controller._collision_job_traj_ids = {1: 1}
        controller._collision_job_total_start_s = {}
        controller._collision_job_detect_start_s = {}
        controller.current_trajectory = trajectory

        with redirect_stdout(io.StringIO()):
            TrajectoryController._on_collision_worker_completed(controller, 1, stale_result)

        self.assertEqual(TrajectoryComputationStatus.SUCCESS, trajectory.status)
        self.assertEqual(TrajectorySampleErrorCode.NONE, segment.samples[0].error_code)
        self.assertEqual([], segment.samples[0].collisions)


if __name__ == "__main__":
    unittest.main()
