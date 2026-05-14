from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from models.types import JointAngles6, Pose6


class RobotProgramBrand(Enum):
    KUKA = "KUKA"


class RobotProgramMotionMode(Enum):
    PTP = "PTP"
    LINEAR = "LINEAR"
    CIRCULAR = "CIRCULAR"


class RobotProgramTargetType(Enum):
    CARTESIAN = "CARTESIAN"
    JOINT = "JOINT"


class ProgramCompensationOutputMode(Enum):
    CARTESIAN = "CARTESIAN"
    ARTICULAR = "ARTICULAR"


class TrajectoryPlaybackMode(Enum):
    THEORETICAL = "THEORETICAL"
    REAL = "REAL"
    COMPENSATED = "COMPENSATED"


@dataclass(frozen=True)
class RobotProgramTarget:
    target_type: RobotProgramTargetType
    cartesian_pose: Pose6 = field(default_factory=Pose6.zeros)
    joint_angles: JointAngles6 = field(default_factory=JointAngles6.zeros)


@dataclass(frozen=True)
class RobotProgramMotion:
    mode: RobotProgramMotionMode
    target: RobotProgramTarget
    line_number: int
    source: str
    base_pose: Pose6 = field(default_factory=Pose6.zeros)
    tool_pose: Pose6 = field(default_factory=Pose6.zeros)
    via_target: RobotProgramTarget | None = None
    cp_speed_mps: float | None = None


@dataclass(frozen=True)
class RobotProgram:
    brand: RobotProgramBrand
    source_path: str
    source_text: str
    motions: list[RobotProgramMotion] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class ProgramSimulationSample:
    time_s: float
    motion_mode: RobotProgramMotionMode
    source_line: int
    joints_deg: JointAngles6
    nominal_pose_base: Pose6
    measured_pose_base: Pose6 | None = None


@dataclass(frozen=True)
class ProgramSimulationResult:
    nominal_samples: list[ProgramSimulationSample] = field(default_factory=list)
    cartesian_compensated_samples: list[ProgramSimulationSample] = field(default_factory=list)
    articular_compensated_samples: list[ProgramSimulationSample] = field(default_factory=list)
    cartesian_compensated_program: RobotProgram | None = None
    articular_compensated_program: RobotProgram | None = None
    warnings: list[str] = field(default_factory=list)
    compensation_computed: bool = False
