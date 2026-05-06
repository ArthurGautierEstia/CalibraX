from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import threading

import numpy as np

from models.primitive_collider_models import PrimitiveCollider, PrimitiveColliderData, RobotAxisColliderData
from models.trajectory_keypoint import KeypointMotionMode, TrajectoryKeypoint
from models.trajectory_options import TrajectoryBezierDegree
from models.types import JointAngles6, Pose6, TrajectorySampleKinematics, XYZ3
from utils.mgi import MgiConfigKey


BuildRevisionId = int


class BuildStatus(Enum):
    QUEUED = "QUEUED"
    RUNNING = "RUNNING"
    CANCELLED = "CANCELLED"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


class TrajectoryBuildTriggerMode(Enum):
    LIVE_PREVIEW = "live_preview"
    DEBOUNCED_FULL = "debounced_full"
    FORCED_FULL = "forced_full"


class TrajectoryComputationStatus(Enum):
    SUCCESS = "SUCCESS"
    POINT_UNREACHABLE = "POINT_UNREACHABLE"
    SPEED_LIMIT_EXCEEDED = "SPEED_LIMIT_EXCEEDED"
    JERK_LIMIT_EXCEEDED = "JERK_LIMIT_EXCEEDED"
    CONFIGURATION_JUMP = "CONFIGURATION_JUMP"
    COLLISION_DETECTED = "COLLISION_DETECTED"
    TCP_WORKSPACE_EXIT = "TCP_WORKSPACE_EXIT"
    NO_COMMON_ALLOWED_CONFIGURATION = "NO_COMMON_ALLOWED_CONFIGURATION"
    FORBIDDEN_CONFIGURATION = "FORBIDDEN_CONFIGURATION"


class TrajectorySampleErrorCode(Enum):
    NONE = "NONE"
    POINT_UNREACHABLE = "POINT_UNREACHABLE"
    SPEED_LIMIT_EXCEEDED = "SPEED_LIMIT_EXCEEDED"
    JERK_LIMIT_EXCEEDED = "JERK_LIMIT_EXCEEDED"
    CONFIGURATION_JUMP = "CONFIGURATION_JUMP"
    COLLISION_DETECTED = "COLLISION_DETECTED"
    TCP_WORKSPACE_EXIT = "TCP_WORKSPACE_EXIT"
    FORBIDDEN_CONFIGURATION = "FORBIDDEN_CONFIGURATION"


class TrajectoryBuilderBehavior(Enum):
    CONTINUE_ON_ERROR = "CONTINUE_ON_ERROR"
    STOP_ON_ERROR = "STOP_ON_ERROR"


class TrajectoryDynamicViolationKind(Enum):
    SPEED = "SPEED"
    ACCELERATION = "ACCELERATION"
    JERK = "JERK"


class TrajectoryDynamicViolationSeverity(Enum):
    ERROR = "ERROR"
    WARNING = "WARNING"


class TrajectoryCollisionDomain(Enum):
    WORKSPACE = "WORKSPACE"
    ROBOT_TOOL = "ROBOT_TOOL"


class BuildCancelToken:
    def __init__(self) -> None:
        self._event = threading.Event()

    def request_cancel(self) -> None:
        self._event.set()

    def is_cancelled(self) -> bool:
        return self._event.is_set()


@dataclass
class TrajectoryBuildRequest:
    revision_id: BuildRevisionId
    current_joints: JointAngles6
    keypoints: list[TrajectoryKeypoint]
    sample_dt_s: float
    smooth_time_enabled: bool
    bezier_degree: TrajectoryBezierDegree
    jerk_check_enabled: bool
    trigger_mode: TrajectoryBuildTriggerMode


class TrajectoryDynamicViolation:
    def __init__(
        self,
        kind: TrajectoryDynamicViolationKind,
        axis: int,
        value: float,
        limit: float,
        severity: TrajectoryDynamicViolationSeverity,
    ) -> None:
        self.kind = kind
        self.axis = int(axis)
        self.value = float(value)
        self.limit = float(limit)
        self.severity = severity


class TrajectoryCollisionDiagnostic:
    def __init__(
        self,
        domain: TrajectoryCollisionDomain,
        owner_a: str,
        name_a: str,
        source_index_a: int | None,
        owner_b: str,
        name_b: str,
        source_index_b: int | None,
    ) -> None:
        self.domain = domain
        self.owner_a = str(owner_a)
        self.name_a = str(name_a)
        self.source_index_a = None if source_index_a is None else int(source_index_a)
        self.owner_b = str(owner_b)
        self.name_b = str(name_b)
        self.source_index_b = None if source_index_b is None else int(source_index_b)


class TrajectorySampleMgiSolution:
    def __init__(
        self,
        status: str = "",
        joints: list[float] | None = None,
    ) -> None:
        self.status = str(status)
        joints_6 = [] if joints is None else [float(v) for v in joints[:6]]
        while len(joints_6) < 6:
            joints_6.append(0.0)
        self.joints = joints_6


class TrajectorySegment:
    def __init__(self, from_keypoint: TrajectoryKeypoint, to_keypoint: TrajectoryKeypoint) -> None:
        self.from_keypoint = from_keypoint
        self.to_keypoint = to_keypoint


class TrajectorySample:
    def __init__(self) -> None:
        self.time = 0.0
        self.joints = [0.0] * 6
        self.pose = [0.0] * 6
        self.kinematics: TrajectorySampleKinematics | None = None
        self.reachable = True
        self.configuration: MgiConfigKey | None = None
        self.velocity = 0.0
        self.acceleration = 0.0
        self.cartesian_velocity = [0.0] * 6
        self.cartesian_acceleration = [0.0] * 6
        self.cartesian_jerk = [0.0] * 6
        self.cartesian_velocity_valid = False
        self.cartesian_acceleration_valid = False
        self.cartesian_jerk_valid = False
        self.articular_velocity = [0.0] * 6
        self.articular_acceleration = [0.0] * 6
        self.articular_jerk = [0.0] * 6
        self.articular_velocity_valid = False
        self.articular_acceleration_valid = False
        self.articular_jerk_valid = False
        self.dynamic_violations: list[TrajectoryDynamicViolation] = []
        self.collisions: list[TrajectoryCollisionDiagnostic] = []
        self.error_code = TrajectorySampleErrorCode.NONE
        self.error_axis: int | None = None
        self.mgi_solutions: dict[MgiConfigKey, TrajectorySampleMgiSolution] = {}


class JointDynamicStats:
    def __init__(
        self,
        max_positive_velocity: float = 0.0,
        max_negative_velocity: float = 0.0,
        max_acceleration: float = 0.0,
        max_deceleration: float = 0.0,
    ) -> None:
        self.max_positive_velocity = float(max_positive_velocity)
        self.max_negative_velocity = float(max_negative_velocity)
        self.max_acceleration = float(max_acceleration)
        self.max_deceleration = float(max_deceleration)


class SegmentResult:
    def __init__(self) -> None:
        self.status = TrajectoryComputationStatus.SUCCESS
        self.samples: list[TrajectorySample] = []
        self.mode = KeypointMotionMode.PTP
        self.in_direction = [0.0, 0.0, 0.0]
        self.out_direction = [0.0, 0.0, 0.0]
        self.duration = 0.0
        self.last_time = 0.0
        self.joints_stats = [JointDynamicStats() for _ in range(6)]
        self.first_error_sample_index: int | None = None
        self.first_error_axis: int | None = None


class TrajectoryResult:
    def __init__(
        self,
        status: TrajectoryComputationStatus = TrajectoryComputationStatus.SUCCESS,
        segments: list[SegmentResult] | None = None,
        first_error_segment_index: int | None = None,
        build_status: BuildStatus = BuildStatus.QUEUED,
        revision_id: BuildRevisionId = 0,
    ) -> None:
        self.status = status
        self.segments = [] if segments is None else list(segments)
        self.first_error_segment_index = first_error_segment_index
        self.build_status = build_status
        self.revision_id = int(revision_id)


class TrajectoryPreviewSample:
    def __init__(
        self,
        time_s: float = 0.0,
        pose_base: Pose6 | None = None,
        joints_deg: JointAngles6 | None = None,
        reachable: bool = True,
        sample_index: int = -1,
    ) -> None:
        self.time_s = float(time_s)
        self.pose_base = Pose6.zeros() if pose_base is None else pose_base.copy()
        self.joints_deg = None if joints_deg is None else joints_deg.copy()
        self.reachable = bool(reachable)
        self.sample_index = int(sample_index)


class TrajectoryPreviewSegment:
    def __init__(self) -> None:
        self.status = TrajectoryComputationStatus.SUCCESS
        self.mode = KeypointMotionMode.PTP
        self.samples: list[TrajectoryPreviewSample] = []
        self.in_direction = XYZ3.zeros()
        self.out_direction = XYZ3.zeros()
        self.duration_s = 0.0
        self.last_time_s = 0.0


class TrajectoryPreviewResult:
    def __init__(
        self,
        revision_id: BuildRevisionId = 0,
        build_status: BuildStatus = BuildStatus.QUEUED,
    ) -> None:
        self.status = TrajectoryComputationStatus.SUCCESS
        self.segments: list[TrajectoryPreviewSegment] = []
        self.first_error_segment_index: int | None = None
        self.analysis_pending = False
        self.revision_id = int(revision_id)
        self.build_status = build_status

    def sample_count(self) -> int:
        total = 0
        for segment in self.segments:
            total += len(segment.samples)
        return total


@dataclass
class ValidityContextSnapshot:
    dh_params: list[list[float]]
    measured_dh_params: list[list[float]]
    measured_dh_enabled: bool
    axis_reversed: list[int]
    corrections: list[list[float]]
    tool_pose: Pose6
    workspace_tcp_zone_colliders: list[PrimitiveCollider]
    workspace_collision_zones: list[PrimitiveColliderData]
    robot_axis_colliders: list[RobotAxisColliderData]
    tool_colliders: list[PrimitiveColliderData]
    evaluated_robot_axis_colliders: list[bool]
    robot_base_transform_world: np.ndarray


@dataclass
class ValidationTaskSample:
    global_sample_index: int
    segment_index: int
    sample_index: int
    sample: TrajectorySample


@dataclass
class ValidationTask:
    revision_id: BuildRevisionId
    task_id: int
    samples: list[ValidationTaskSample]
    context: ValidityContextSnapshot
    start_index: int
    end_index_exclusive: int


@dataclass
class SampleValidationResult:
    global_sample_index: int
    segment_index: int
    sample_index: int
    error_code: TrajectorySampleErrorCode
    collisions: list[TrajectoryCollisionDiagnostic]
    tcp_world_xyz: XYZ3 | None = None


@dataclass
class ValidationResult:
    revision_id: BuildRevisionId
    task_id: int
    cancelled: bool
    sample_results: list[SampleValidationResult]


PreviewSample = TrajectoryPreviewSample
PreviewSegment = TrajectoryPreviewSegment
PreviewTrajectory = TrajectoryPreviewResult
ComputedSample = TrajectorySample
ComputedSegment = SegmentResult
TrajectoryComputation = TrajectoryResult
