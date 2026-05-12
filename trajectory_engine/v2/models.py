from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from models.types import JointAngles6, Pose6, XYZ3


class TrajectoryPassMode(Enum):
    STOP = "STOP"
    FLY_BY = "FLY_BY"

    @staticmethod
    def from_value(value: object) -> "TrajectoryPassMode":
        if isinstance(value, TrajectoryPassMode):
            return value
        try:
            return TrajectoryPassMode(str(value))
        except ValueError:
            return TrajectoryPassMode.STOP


class SegmentDynamicPhaseKind(Enum):
    ACCEL = "ACCEL"
    CRUISE = "CRUISE"
    DECEL = "DECEL"
    TRANSITION = "TRANSITION"


class SegmentDynamicProfileKind(Enum):
    TRAPEZOIDAL = "TRAPEZOIDAL"
    TRIANGULAR = "TRIANGULAR"


@dataclass(frozen=True)
class Bezier7ControlPoints3D:
    p0: XYZ3
    p1: XYZ3
    p2: XYZ3
    p3: XYZ3
    p4: XYZ3
    p5: XYZ3
    p6: XYZ3
    p7: XYZ3

    def point_at(self, index: int) -> XYZ3:
        if index == 0:
            return self.p0.copy()
        if index == 1:
            return self.p1.copy()
        if index == 2:
            return self.p2.copy()
        if index == 3:
            return self.p3.copy()
        if index == 4:
            return self.p4.copy()
        if index == 5:
            return self.p5.copy()
        if index == 6:
            return self.p6.copy()
        if index == 7:
            return self.p7.copy()
        raise IndexError("Bezier7 control point index must be in [0, 7]")


@dataclass(frozen=True)
class Bezier7Coefficients3D:
    a0: XYZ3
    a1: XYZ3
    a2: XYZ3
    a3: XYZ3
    a4: XYZ3
    a5: XYZ3
    a6: XYZ3
    a7: XYZ3


@dataclass
class ArcLengthLut:
    parameters_u: list[float]
    distances_mm: list[float]
    total_length_mm: float


@dataclass(frozen=True)
class MotionScalarState:
    time_s: float
    position: float
    velocity: float
    acceleration: float
    jerk: float
    phase: SegmentDynamicPhaseKind
    segment_index: int


@dataclass(frozen=True)
class DynamicLimits:
    cartesian_speed_mm_s: float
    cartesian_accel_mm_s2: float
    cartesian_jerk_mm_s3: float


@dataclass(frozen=True)
class SegmentDynamicResolution:
    profile_kind: SegmentDynamicProfileKind
    peak_speed_mm_s: float
    target_speed_reached: bool
    accel_distance_mm: float
    cruise_distance_mm: float
    decel_distance_mm: float


@dataclass(frozen=True)
class SegmentSpeedProfile:
    segment_index: int
    length_mm: float
    target_speed_mm_s: float
    entry_speed_mm_s: float
    exit_speed_mm_s: float


@dataclass(frozen=True)
class RuntimeSegment:
    mode: object
    curve: object | None
    arc_lut: ArcLengthLut | None
    start_pose: Pose6
    end_pose: Pose6
    speed_profile: SegmentSpeedProfile
    out_direction: XYZ3
    in_direction: XYZ3


@dataclass
class TrajectoryRuntimePlan:
    segments: list[RuntimeSegment]
    total_duration_s: float


@dataclass(frozen=True)
class PtpMotionPlan:
    start_joints_deg: JointAngles6
    delta_joints_deg: JointAngles6
    duration_s: float
