from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from models.types import JointAngles6, Pose6
from models.types.external_axis_program_target import ExternalAxisProgramTarget
from models.types.motion_approximation import MotionApproximation


class RobotProgramBrand(Enum):
    KUKA = "KUKA"


class RobotProgramMotionMode(Enum):
    PTP = "PTP"
    LINEAR = "LINEAR"
    CIRCULAR = "CIRCULAR"
    EXTERNAL_AXIS = "EXTERNAL_AXIS"


class RobotProgramTargetType(Enum):
    CARTESIAN = "CARTESIAN"
    JOINT = "JOINT"


class ProgramBaseSource(str, Enum):
    MANUAL = "MANUAL"
    WORKPIECE = "WORKPIECE"
    WORLD = "WORLD"
    ROBOT = "ROBOT"
    PROGRAM_FILE = "PROGRAM_FILE"
    EXTERNAL_AXIS = "EXTERNAL_AXIS"


@dataclass(frozen=True)
class ProgramBaseSpec:
    """Descripteur du repère base programme évaluable à un état d'axes externes donné.

    Contrairement à ``base_pose`` (la base bakée en repère robot à l'instant du clic
    Simuler), ce descripteur conserve l'IDENTITÉ de l'élément source. Il permet au
    simulateur de ré-évaluer le repère source à l'état d'axes *simulé* (suivi pièce /
    positionneur pendant le programme). L'offset utilisateur reste baké dans
    ``base_pose`` : il accompagne rigidement la source.
    """
    source: ProgramBaseSource
    ext_axis_id: str | None = None
    manual_pose: Pose6 = field(default_factory=Pose6.zeros)
    file_pose: Pose6 = field(default_factory=Pose6.zeros)


class ProgramCompensationOutputMode(Enum):
    CARTESIAN = "CARTESIAN"
    ARTICULAR = "ARTICULAR"


class TrajectoryPlaybackMode(Enum):
    THEORETICAL = "THEORETICAL"
    REAL = "REAL"
    COMPENSATED = "COMPENSATED"


class ProgramOrigin(Enum):
    LOADED_KRL = "LOADED_KRL"
    IMPORTED_APT = "IMPORTED_APT"
    IMPORTED_CATNC = "IMPORTED_CATNC"
    BUILT = "BUILT"


class MotionRole(Enum):
    NORMAL = "NORMAL"
    HOME_START = "HOME_START"
    HOME_END = "HOME_END"
    APPROACH = "APPROACH"
    RETRACT = "RETRACT"
    EXTERNAL_SETUP = "EXTERNAL_SETUP"


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
    role: MotionRole = MotionRole.NORMAL
    approximation: MotionApproximation = field(default_factory=MotionApproximation.none)
    external_axis_target: ExternalAxisProgramTarget | None = None


@dataclass(frozen=True)
class RobotProgram:
    brand: RobotProgramBrand
    source_path: str
    source_text: str
    program_base_pose: Pose6 = field(default_factory=Pose6.zeros)
    motions: list[RobotProgramMotion] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    origin: ProgramOrigin = ProgramOrigin.LOADED_KRL


@dataclass(frozen=True)
class ProgramSimulationSample:
    time_s: float
    motion_mode: RobotProgramMotionMode
    source_line: int
    joints_deg: JointAngles6
    nominal_pose_base: Pose6
    measured_pose_base: Pose6 | None = None
    ext_axis_values: tuple[float, ...] = field(default_factory=tuple)
    nominal_pose_world: Pose6 | None = None
    measured_pose_world: Pose6 | None = None


@dataclass(frozen=True)
class ProgramSimulationResult:
    nominal_samples: list[ProgramSimulationSample] = field(default_factory=list)
    cartesian_compensated_samples: list[ProgramSimulationSample] = field(default_factory=list)
    articular_compensated_samples: list[ProgramSimulationSample] = field(default_factory=list)
    cartesian_compensated_program: RobotProgram | None = None
    articular_compensated_program: RobotProgram | None = None
    warnings: list[str] = field(default_factory=list)
    compensation_computed: bool = False
