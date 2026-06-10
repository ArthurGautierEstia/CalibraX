from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


class ApproachAxisRef(Enum):
    TOOL_Z = "TOOL_Z"
    PIECE_X = "PIECE_X"
    PIECE_Y = "PIECE_Y"
    PIECE_Z = "PIECE_Z"


@dataclass(frozen=True)
class ApproachRetractStep:
    axis_ref: ApproachAxisRef
    distance_mm: float
    speed_mps: float
    inverted: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "axis_ref": self.axis_ref.value,
            "distance_mm": self.distance_mm,
            "speed_mps": self.speed_mps,
            "inverted": self.inverted,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ApproachRetractStep:
        try:
            axis_ref = ApproachAxisRef(data.get("axis_ref", ApproachAxisRef.TOOL_Z.value))
        except ValueError:
            axis_ref = ApproachAxisRef.TOOL_Z
        return cls(
            axis_ref=axis_ref,
            distance_mm=float(data.get("distance_mm", 50.0)),
            speed_mps=float(data.get("speed_mps", 0.2)),
            inverted=bool(data.get("inverted", False)),
        )


_DEFAULT_STEP = ApproachRetractStep(ApproachAxisRef.TOOL_Z, 50.0, 0.2)


@dataclass(frozen=True)
class ApproachRetractConfig:
    enabled: bool
    steps: tuple[ApproachRetractStep, ...]

    @classmethod
    def default_approach(cls) -> ApproachRetractConfig:
        return cls(enabled=False, steps=(_DEFAULT_STEP,))

    @classmethod
    def default_retract(cls) -> ApproachRetractConfig:
        return cls(enabled=False, steps=(_DEFAULT_STEP,))
