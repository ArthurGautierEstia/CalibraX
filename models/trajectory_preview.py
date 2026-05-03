from __future__ import annotations

from models.trajectory_keypoint import KeypointMotionMode
from models.trajectory_result import TrajectoryComputationStatus
from models.types import JointAngles6, Pose6, XYZ3


class TrajectoryPreviewSample:
    def __init__(
        self,
        time_s: float = 0.0,
        pose_base: Pose6 | None = None,
        joints_deg: JointAngles6 | None = None,
        reachable: bool = True,
    ) -> None:
        self.time_s = float(time_s)
        self.pose_base = Pose6.zeros() if pose_base is None else pose_base.copy()
        self.joints_deg = None if joints_deg is None else joints_deg.copy()
        self.reachable = bool(reachable)


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
    def __init__(self) -> None:
        self.status = TrajectoryComputationStatus.SUCCESS
        self.segments: list[TrajectoryPreviewSegment] = []
        self.first_error_segment_index: int | None = None
        self.analysis_pending = False

    def sample_count(self) -> int:
        total = 0
        for segment in self.segments:
            total += len(segment.samples)
        return total
