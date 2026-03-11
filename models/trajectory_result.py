from enum import Enum

from models.trajectory_keypoint import KeypointMotionMode, TrajectoryKeypoint
from utils.mgi import MgiConfigKey


class TrajectoryComputationStatus(Enum):
    SUCCESS = "SUCCESS"
    POINT_UNREACHABLE = "POINT_UNREACHABLE"
    OVER_DYNAMIC_LIMIT = "OVER_DYNAMIC_LIMIT"
    NO_COMMON_ALLOWED_CONFIGURATION = "NO_COMMON_ALLOWED_CONFIGURATION"
    FORBIDDEN_CONFIGURATION = "FORBIDDEN_CONFIGURATION"


class TrajectorySampleErrorCode(Enum):
    NONE = "NONE"
    POINT_UNREACHABLE = "POINT_UNREACHABLE"
    OVER_DYNAMIC_LIMIT = "OVER_DYNAMIC_LIMIT"
    FORBIDDEN_CONFIGURATION = "FORBIDDEN_CONFIGURATION"


class TrajectoryBuilderBehavior(Enum):
    CONTINUE_ON_ERROR = "CONTINUE_ON_ERROR"
    STOP_ON_ERROR = "STOP_ON_ERROR"


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
        self.reachable = True
        self.configuration: MgiConfigKey | None = None
        self.velocity = 0.0
        self.acceleration = 0.0
        self.cartesian_velocity = [0.0] * 6
        self.cartesian_acceleration = [0.0] * 6
        self.articular_velocity = [0.0] * 6
        self.articular_acceleration = [0.0] * 6
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
    ) -> None:
        self.status = status
        self.segments = [] if segments is None else list(segments)
        self.first_error_segment_index = first_error_segment_index
