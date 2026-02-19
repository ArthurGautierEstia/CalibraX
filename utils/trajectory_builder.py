from models.robot_model import RobotModel
from models.trajectory_keypoint import KeypointMotionMode, TrajectoryKeypoint
from models.trajectory_result import (
    SegmentResult,
    TrajectoryBuilderBehavior,
    TrajectoryComputationStatus,
    TrajectoryResult,
    TrajectorySample,
    TrajectorySegment,
)


class TrajectoryBuilder:
    DEFAULT_SAMPLE_DT_S = 0.004  # 4 ms

    def __init__(
        self,
        robot_model: RobotModel,
        behavior: TrajectoryBuilderBehavior = TrajectoryBuilderBehavior.CONTINUE_ON_ERROR,
        sample_dt_s: float = DEFAULT_SAMPLE_DT_S,
    ) -> None:
        self.robot_model = robot_model
        self.behavior = behavior
        self.sample_dt_s = sample_dt_s if sample_dt_s > 0.0 else TrajectoryBuilder.DEFAULT_SAMPLE_DT_S

    @staticmethod
    def linear_speed_mps_to_mmps(speed_mps: float) -> float:
        return float(speed_mps) * 1000.0

    @staticmethod
    def _new_empty_segment(last_time_s: float) -> SegmentResult:
        segment = SegmentResult()
        segment.last_time = float(last_time_s)
        return segment

    @staticmethod
    def _extract_previous_samples(result: SegmentResult) -> tuple[TrajectorySample | None, TrajectorySample | None]:
        if not result.samples:
            return None, None
        if len(result.samples) == 1:
            return result.samples[-1], None
        return result.samples[-1], result.samples[-2]

    @staticmethod
    def _accumulate_status(
        trajectory: TrajectoryResult,
        segment_result: SegmentResult,
        segment_index: int,
    ) -> None:
        if segment_result.status == TrajectoryComputationStatus.SUCCESS:
            return
        if trajectory.status != TrajectoryComputationStatus.SUCCESS:
            return
        trajectory.status = segment_result.status
        trajectory.first_error_segment_index = segment_index

    def _should_stop_on_error(self, status: TrajectoryComputationStatus) -> bool:
        if status == TrajectoryComputationStatus.SUCCESS:
            return False
        return self.behavior == TrajectoryBuilderBehavior.STOP_ON_ERROR

    def compute_trajectory(
        self,
        current_joints: list[float],
        segments: list[TrajectorySegment],
    ) -> TrajectoryResult:
        trajectory = TrajectoryResult()
        if not segments:
            return trajectory

        previous_sample_1: TrajectorySample | None = None
        previous_sample_2: TrajectorySample | None = None
        start_time_s = 0.0

        first_result = TrajectoryBuilder.compute_first_segment(current_joints, segments[0].from_keypoint, start_time_s)
        trajectory.segments.append(first_result)
        TrajectoryBuilder._accumulate_status(trajectory, first_result, segment_index=0)
        if self._should_stop_on_error(first_result.status):
            return trajectory

        previous_sample_1, previous_sample_2 = TrajectoryBuilder._extract_previous_samples(first_result)
        start_time_s = first_result.last_time

        for idx, segment in enumerate(segments, start=1):
            segment_result = TrajectoryBuilder.compute_segment(segment, previous_sample_1, previous_sample_2, start_time_s)
            trajectory.segments.append(segment_result)
            TrajectoryBuilder._accumulate_status(trajectory, segment_result, segment_index=idx)
            if self._should_stop_on_error(segment_result.status):
                break

            previous_sample_1, previous_sample_2 = TrajectoryBuilder._extract_previous_samples(segment_result)
            start_time_s = segment_result.last_time

        return trajectory

    @staticmethod
    def compute_first_segment(
        current_joints: list[float],
        to_keypoint: TrajectoryKeypoint,
        start_time_s: float = 0.0,
    ) -> SegmentResult:
        # Stub only: algorithm will be implemented later.
        _ = current_joints
        _ = to_keypoint
        return TrajectoryBuilder._new_empty_segment(start_time_s)

    @staticmethod
    def compute_segment(
        segment: TrajectorySegment,
        previous_sample_1: TrajectorySample | None = None,
        previous_sample_2: TrajectorySample | None = None,
        start_time_s: float = 0.0,
    ) -> SegmentResult:
        functor = TrajectoryBuilder.compute_PTP_segment if segment.to_keypoint.mode ==  KeypointMotionMode.PTP \
            else (TrajectoryBuilder.compute_LIN_segment if segment.to_keypoint.mode == KeypointMotionMode.LINEAR \
            else TrajectoryBuilder.compute_cubique_segment)
        return functor(segment, previous_sample_1, previous_sample_2, start_time_s)

    @staticmethod
    def compute_PTP_segment(
        segment: TrajectorySegment,
        previous_sample_1: TrajectorySample | None = None,
        previous_sample_2: TrajectorySample | None = None,
        start_time_s: float = 0.0,
    ) -> SegmentResult:
        # Stub only: algorithm will be implemented later.
        _ = segment
        _ = previous_sample_1
        _ = previous_sample_2
        return TrajectoryBuilder._new_empty_segment(start_time_s)

    @staticmethod
    def compute_LIN_segment(
        segment: TrajectorySegment,
        previous_sample_1: TrajectorySample | None = None,
        previous_sample_2: TrajectorySample | None = None,
        start_time_s: float = 0.0,
    ) -> SegmentResult:
        # Stub only: algorithm will be implemented later.
        _ = segment
        _ = previous_sample_1
        _ = previous_sample_2
        return TrajectoryBuilder._new_empty_segment(start_time_s)

    @staticmethod
    def compute_cubique_segment(
        segment: TrajectorySegment,
        previous_sample_1: TrajectorySample | None = None,
        previous_sample_2: TrajectorySample | None = None,
        start_time_s: float = 0.0,
    ) -> SegmentResult:
        # Stub only: algorithm will be implemented later.
        _ = segment
        _ = previous_sample_1
        _ = previous_sample_2
        return TrajectoryBuilder._new_empty_segment(start_time_s)
