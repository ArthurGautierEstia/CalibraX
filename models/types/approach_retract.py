from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ApproachAxisRef(Enum):
    TOOL_Z = "TOOL_Z"
    PIECE_X = "PIECE_X"
    PIECE_Y = "PIECE_Y"
    PIECE_Z = "PIECE_Z"


@dataclass(frozen=True)
class ApproachRetractConfig:
    enabled: bool
    axis_ref: ApproachAxisRef
    distance_mm: float
    speed_mps: float

    @classmethod
    def default_approach(cls) -> ApproachRetractConfig:
        return cls(enabled=False, axis_ref=ApproachAxisRef.TOOL_Z, distance_mm=50.0, speed_mps=0.2)

    @classmethod
    def default_retract(cls) -> ApproachRetractConfig:
        return cls(enabled=False, axis_ref=ApproachAxisRef.TOOL_Z, distance_mm=50.0, speed_mps=0.2)
